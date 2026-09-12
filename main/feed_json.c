/*
 * JSON snapshot feed -- TCP :8888, one NDJSON line per tick with the full
 * aircraft table, for the Android app.
 *
 * Deliberately simpler than feed_avr.c/feed_beast.c on the producer side:
 * those queue individual decoded frames from the demod hot path and must
 * never block it, so they need a lock-free ring between producer and sender.
 * This feed has exactly one producer call (aircraft_export_ndjson(), in
 * class_driver.c) made from this module's own task right before it
 * broadcasts -- no ring, no per-client queue, and a client that misses a tick
 * simply gets the next snapshot whole.
 *
 * What it does NOT get to be simple about is the write itself. An AVR or
 * Beast batch is many self-delimiting records, so a short write costs the
 * peer one frame and it resyncs on the next '*' or 0x1A. A snapshot is ONE
 * line whose terminating '\n' is the last byte: a short write costs the peer
 * the line boundary, and a reader splitting on '\n' then never completes a
 * line at all. See send_frame().
 */

#include <errno.h>
#include <fcntl.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"

#include "feed_json.h"

/* Defined in class_driver.c, where s_aircraft[] lives; not put in a shared
 * header to avoid a dependency edge from this feed module into the TUI's
 * internals -- matches net_eth.c's extern declaration of
 * esp_libusb_stream_dropped() for the same reason. Returns 0 if the table did
 * not fit, rather than a truncated line. */
extern size_t aircraft_export_ndjson(char *buf, size_t bufsize);

#define FEED_PORT     8888
#define MAX_CLIENTS   4
#define TICK_MS       750   /* ~1.3 Hz, inside the plan's 1-2 Hz target */
/* MAX_TRACKED=64 aircraft at ~165 B each plus the header and "]}\n" is
 * ~10.7 KB worst case. */
#define SNAPSHOT_MAX  16384

/* Per-client budget for pushing one whole snapshot out, and the poll step
 * inside it. A snapshot is ~2x CONFIG_LWIP_TCP_SND_BUF_DEFAULT (5760), so it
 * needs at least two passes through the send buffer even on an idle link, and
 * the second one cannot start until the peer's ACKs open the window -- one
 * RTT, single-digit ms on this LAN and on the SoftAP. 150 ms is ~10x that, so
 * only a peer that has genuinely stopped reading ever reaches it.
 * SEND_WAIT_MS is one FreeRTOS tick at CONFIG_FREERTOS_HZ=100; anything
 * smaller rounds to pdMS_TO_TICKS(x)==0 and busy-spins. */
#define SEND_BUDGET_MS   150
#define SEND_WAIT_MS      10
/* Consecutive ticks a client may fail to take a whole snapshot before it is
 * dropped; the test is `> STALL_LIMIT`, as in feed_avr.c, so it goes on the
 * fifth miss -- ~3.8 s, just past the app's own 3 s read timeout. There is no
 * point holding a slot for a peer that has already given up on us. */
#define STALL_LIMIT        4

/* Drop a peer that stops acknowledging anything after ~25 s
 * (keep_idle + keep_cnt * keep_intvl) rather than the ~90 s lwIP's retransmit
 * escalation takes (CONFIG_LWIP_TCP_MAXRTX=12 at RTO 1500 ms with backoff).
 * SO_KEEPALIVE on its own does nothing useful here: TCP_KEEPIDLE_DEFAULT is
 * 7200 s and setting the option does not change it. Note this covers only a
 * peer that has vanished -- one that is alive but has stopped reading keeps
 * ACKing zero-window probes, so pcb->tmr keeps moving and keepalive never
 * fires. That case is STALL_LIMIT's job. */
#define KEEP_IDLE_S       10
#define KEEP_INTVL_S       5
#define KEEP_CNT           3

static const char *TAG = "json";

static int  s_listen = -1;
static struct { int fd; uint16_t stall; } s_cli[MAX_CLIENTS];
static volatile int s_nclients;
static EXT_RAM_BSS_ATTR char s_snapshot[SNAPSHOT_MAX];   /* task-context only, see class_driver.c */

int feed_json_clients(void) { return s_nclients; }

/* `why` distinguishes "the peer hung up" from "we gave up on it", same as
 * feed_beast.c -- without it a client that reconnects on its own and one this
 * board is dropping look identical in the log. */
static void client_close(int i, const char *why)
{
    close(s_cli[i].fd);
    s_cli[i].fd    = -1;
    s_cli[i].stall = 0;
    s_nclients--;
    ESP_LOGI(TAG, "client gone, %d left (%s)", s_nclients, why);
}

static void listen_open(void)
{
    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0) return;

    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(FEED_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0 || listen(fd, 2) < 0) {
        close(fd);
        return;
    }
    fcntl(fd, F_SETFL, O_NONBLOCK);
    s_listen = fd;
    ESP_LOGI(TAG, "JSON snapshot listening on :%d", FEED_PORT);
}

static void accept_new(void)
{
    int fd = accept(s_listen, NULL, NULL);
    if (fd < 0) return;

    int slot = -1;
    for (int i = 0; i < MAX_CLIENTS; i++)
        if (s_cli[i].fd < 0) { slot = i; break; }
    if (slot < 0) { close(fd); return; }

    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY,  &one, sizeof(one));
    setsockopt(fd, SOL_SOCKET,  SO_KEEPALIVE, &one, sizeof(one));
    int v = KEEP_IDLE_S;  setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE,  &v, sizeof(v));
    v = KEEP_INTVL_S;     setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &v, sizeof(v));
    v = KEEP_CNT;         setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT,   &v, sizeof(v));
    /* Non-blocking, and deliberately NO SO_SNDTIMEO: see send_frame(). */
    fcntl(fd, F_SETFL, O_NONBLOCK);

    s_cli[slot].fd    = fd;
    s_cli[slot].stall = 0;
    s_nclients++;
    ESP_LOGI(TAG, "client connected, %d total", s_nclients);
}

/* Push one whole snapshot to one client.
 *   1  = the entire line went out
 *   0  = ran out of budget part-way (caller counts a stall)
 *  -1  = hard error, close it
 *
 * This is a loop and not a single send() because of a genuine lwIP trap: a
 * snapshot is bigger than the send buffer, so ONE send() can never take all
 * of it, whatever the socket's blocking mode. SO_SNDTIMEO does not help and
 * is in fact the opposite of what it reads like -- api_lib.c's
 * netconn_write_vectors_partly() does `if (conn->send_timeout != 0)
 * dontblock = 1;`, and api_msg.c's do_writemore() then finishes after a
 * single tcp_write() pass (`if ((offset == len) || dontblock) write_finished
 * = 1;`). So a socket with a send timeout returns a SHORT COUNT, not a
 * blocking write and not an error, and the timeout value is never even
 * consulted. lwip_send() reports that as a positive return < len, which is
 * why ignoring anything but `r < 0` silently truncated every snapshot past
 * ~35 aircraft and starved the peer of the '\n' it splits lines on. */
static int send_frame(int i, const char *buf, size_t n)
{
    size_t  off = 0;
    int64_t end = esp_timer_get_time() + SEND_BUDGET_MS * 1000;

    while (off < n) {
        int r = send(s_cli[i].fd, buf + off, n - off, 0);
        if (r > 0) { off += (size_t)r; continue; }
        if (r < 0 && errno != EWOULDBLOCK && errno != EAGAIN) return -1;
        if (esp_timer_get_time() >= end) break;
        /* Window is closed; it reopens on the peer's ACKs, which arrive in
         * the tcpip task, so yield rather than spin. */
        vTaskDelay(pdMS_TO_TICKS(SEND_WAIT_MS));
    }
    return off == n ? 1 : 0;
}

static void broadcast(const char *buf, size_t n)
{
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_cli[i].fd < 0) continue;

        int r = send_frame(i, buf, n);
        if (r < 0) {
            client_close(i, "send error");
        } else if (r > 0) {
            s_cli[i].stall = 0;
        } else if (++s_cli[i].stall > STALL_LIMIT) {
            /* Held its slot without taking a snapshot for STALL_LIMIT ticks.
             * Left alone this is how :8888 wedges: MAX_CLIENTS slots leak to
             * peers that never send a FIN (phone out of range, app killed,
             * app frozen by Android's doze with the socket still open), and
             * once all four are gone accept_new() takes the handshake and
             * closes immediately -- which looks to the client exactly like
             * "connected, then dropped", forever. */
            client_close(i, "too slow");
        }
        /* A stall short-wrote part of a line, so the peer's next complete
         * line is the tail of this one glued to the whole of the next
         * snapshot: one corrupt line, then it resyncs. That is the cost of
         * keeping a session through a hiccup instead of dropping it. */
    }
}

static void poll_closed(void)
{
    char scratch[32];
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_cli[i].fd < 0) continue;
        /* An idle feed would otherwise not notice a FIN until the next tick. */
        int r = recv(s_cli[i].fd, scratch, sizeof(scratch), MSG_DONTWAIT);
        if (r == 0)
            client_close(i, "peer closed");
        else if (r < 0 && errno != EWOULDBLOCK && errno != EAGAIN)
            client_close(i, "recv error");
    }
}

static void feed_task(void *arg)
{
    (void)arg;
    while (1) {
        if (s_listen < 0) listen_open();
        if (s_listen >= 0) accept_new();

        poll_closed();

        if (s_nclients) {
            size_t n = aircraft_export_ndjson(s_snapshot, sizeof(s_snapshot));
            if (n) broadcast(s_snapshot, n);
            else   ESP_LOGW(TAG, "snapshot did not fit %d B, tick skipped", SNAPSHOT_MAX);
        }
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
    }
}

esp_err_t feed_json_start(void)
{
    for (int i = 0; i < MAX_CLIENTS; i++) s_cli[i].fd = -1;

    /* core0, same as feed_avr/feed_beast and for the same reason: core1 is
     * the demod loop. */
    BaseType_t ok = xTaskCreatePinnedToCore(feed_task, "json", 4096, NULL, 3, NULL, 0);
    return ok == pdTRUE ? ESP_OK : ESP_FAIL;
}
