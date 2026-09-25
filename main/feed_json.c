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
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "lwip/priv/tcpip_priv.h"   /* tcpip_api_call()                    */
#include "lwip/priv/tcp_priv.h"     /* tcp_active_pcbs & co, for the census */

#include "adsb.h"        /* ADSB_SNAPSHOT_MAX only */
#include "feed_json.h"
#include "shell.h"       /* sys_log() */

/* Defined in class_driver.c, where s_aircraft[] lives; not put in a shared
 * header to avoid a dependency edge from this feed module into the TUI's
 * internals -- matches net_eth.c's extern declaration of
 * esp_libusb_stream_dropped() for the same reason. Returns 0 if the table did
 * not fit, rather than a truncated line. */
extern size_t aircraft_export_ndjson(char *buf, size_t bufsize);

#define FEED_PORT     8888
#define MAX_CLIENTS   4
#define TICK_MS       750   /* ~1.3 Hz, inside the plan's 1-2 Hz target */
#define SNAPSHOT_MAX  ADSB_SNAPSHOT_MAX

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

/* How often feed_task() takes a TCP census (see feed_json_tcp_census()), in
 * ticks: ~6 s. A wedge has to show up in two in a row before it is logged, so
 * a burst of handshakes that fills the backlog for one tick stays quiet. */
#define CENSUS_TICKS       8

static const char *TAG = "json";

static int  s_listen = -1;
static struct { int fd; uint16_t stall; } s_cli[MAX_CLIENTS];
static volatile int s_nclients;
static volatile uint32_t s_resets;
static EXT_RAM_BSS_ATTR char s_snapshot[SNAPSHOT_MAX];   /* task-context only, see class_driver.c */

int      feed_json_clients(void) { return s_nclients; }
uint32_t feed_json_resets(void)  { return s_resets; }

/* `why` distinguishes "the peer hung up" from "we gave up on it", same as
 * feed_beast.c -- without it a client that reconnects on its own and one this
 * board is dropping look identical in the log.
 *
 * `reset` is for the "we gave up on it" cases. A peer that stopped ACKing --
 * the usual one is a phone that re-associated to the SoftAP under a fresh
 * random MAC and a fresh DHCP lease, orphaning the old connection -- leaves up
 * to a send buffer (5760 B of internal RAM) unacknowledged. A plain close()
 * queues a FIN behind that and the pcb sits in FIN_WAIT_1 retransmitting it
 * through CONFIG_LWIP_TCP_MAXRTX's whole backoff. SO_LINGER {1, 0} makes
 * lwip_close() tcp_abort() instead (api_msg.c: linger == 0 with unsent or
 * unacked data): one RST, pcb and buffers freed at once. It needs
 * CONFIG_LWIP_SO_LINGER, which sdkconfig.defaults turns on for this. */
static void client_close(int i, const char *why, bool reset)
{
    if (reset) {
        struct linger lg = { .l_onoff = 1, .l_linger = 0 };
        setsockopt(s_cli[i].fd, SOL_SOCKET, SO_LINGER, &lg, sizeof(lg));
        s_resets++;
    }
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
    /* Backlog MAX_CLIENTS rather than 2: lwIP's accept_function() aborts a
     * pcb outright (the peer sees a RST) once the backlog is full, and the
     * app's manual mode probes :8888 on every WiFi Network it can see, so
     * several handshakes can land inside one tick. */
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0 ||
        listen(fd, MAX_CLIENTS) < 0) {
        close(fd);
        return;
    }
    fcntl(fd, F_SETFL, O_NONBLOCK);
    s_listen = fd;
    ESP_LOGI(TAG, "JSON snapshot listening on :%d", FEED_PORT);
}

/* Drains the whole accept queue, because the tick is 750 ms: taking one
 * connection per tick left the rest sitting in the backlog for most of a
 * second each, and the app's probe gives a candidate 2 s to produce a line.
 *
 * Every way this can turn a client away is logged. None of them used to be,
 * which is what made the wedge invisible: a full slot table and an exhausted
 * lwIP socket pool (accept() -> ENFILE, after lwip_accept() has already
 * completed the handshake and deleted the netconn) both look to the peer like
 * a connection that succeeded and was dropped, and the board said nothing at
 * all about either. */
static void accept_new(void)
{
    for (;;) {
        struct sockaddr_in peer;
        socklen_t          plen = sizeof(peer);
        int fd = accept(s_listen, (struct sockaddr *)&peer, &plen);
        if (fd < 0) {
            if (errno != EWOULDBLOCK && errno != EAGAIN)
                ESP_LOGW(TAG, "accept failed, errno %d", errno);
            return;
        }

        char ip[INET_ADDRSTRLEN] = "?";
        inet_ntop(AF_INET, &peer.sin_addr, ip, sizeof(ip));

        int slot = -1;
        for (int i = 0; i < MAX_CLIENTS; i++)
            if (s_cli[i].fd < 0) { slot = i; break; }
        if (slot < 0) {
            ESP_LOGW(TAG, "%s turned away, all %d slots busy", ip, MAX_CLIENTS);
            close(fd);
            continue;
        }

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
        ESP_LOGI(TAG, "client connected from %s, %d total", ip, s_nclients);
    }
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
            client_close(i, "send error", true);
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
            client_close(i, "too slow", true);
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
            client_close(i, "peer closed", false);
        else if (r < 0 && errno != EWOULDBLOCK && errno != EAGAIN)
            client_close(i, "recv error", false);
    }
}

/* ── TCP census ──────────────────────────────────────────────────────────
 *
 * There are two ways lwIP turns a client of this port away that nothing in
 * this file can see, because both happen before accept(): tcp_listen_input()
 * returns without a word -- no RST, no SYN-ACK -- when the listener's backlog
 * is full (accepts_pending >= backlog) or when tcp_alloc() finds no pcb
 * (CONFIG_LWIP_MAX_ACTIVE_TCP reached and nothing in TIME_WAIT / LAST_ACK /
 * CLOSING / FIN_WAIT_x left to kill; ESTABLISHED and SYN_RCVD of equal
 * priority are never evicted). The peer sees a connect() that times out while
 * the board still answers ping and hands out DHCP leases -- exactly the
 * "SoftAP up, :8888 dead" state the app hit on 2026-09-25 and nobody could
 * explain, because the evidence was gone by the time anyone looked.
 *
 * The pcb lists belong to the tcpip thread (CONFIG_LWIP_TCPIP_CORE_LOCKING is
 * off), so the walk runs there through tcpip_api_call(). */
struct census_call {
    struct tcpip_api_call_data call;   /* must be first */
    feed_json_tcp_t           *out;
};

static err_t census_fn(struct tcpip_api_call_data *c)
{
    feed_json_tcp_t *t = ((struct census_call *)c)->out;

    for (struct tcp_pcb *p = tcp_active_pcbs; p; p = p->next) {
        switch (p->state) {
        case ESTABLISHED: t->established++; break;
        case SYN_RCVD:    t->syn_rcvd++;    break;
        case FIN_WAIT_1: case FIN_WAIT_2: case CLOSING:
        case CLOSE_WAIT: case LAST_ACK:
                          t->closing++;     break;
        default:          t->other++;       break;
        }
    }
    for (struct tcp_pcb *p = tcp_tw_pcbs; p; p = p->next) t->time_wait++;

    t->pending = -1;
    for (struct tcp_pcb_listen *l = tcp_listen_pcbs.listen_pcbs; l; l = l->next)
        if (l->local_port == FEED_PORT) {
            t->pending = l->accepts_pending;
            t->backlog = l->backlog;
        }
    return ERR_OK;
}

void feed_json_tcp_census(feed_json_tcp_t *t)
{
    memset(t, 0, sizeof(*t));
    struct census_call c = { .out = t };
    tcpip_api_call(census_fn, &c.call);
    t->pcb_limit     = CONFIG_LWIP_MAX_ACTIVE_TCP;
    t->internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
}

/* Which of the two silent refusals the census says is in force, or NULL. The
 * pcb test counts only what tcp_alloc() cannot reclaim. */
static const char *census_wedged(const feed_json_tcp_t *t)
{
    if (t->pending >= 0 && t->pending >= t->backlog) return "backlog full";
    if (t->established + t->syn_rcvd + t->other >= t->pcb_limit) return "no pcb";
    return NULL;
}

/* sys_log(), not ESP_LOGW: sys_log() lands in the event ring, so `log tail`
 * still has it after the fact, over SSH or serial, without a reboot. */
static void census_check(void)
{
    static int     s_bad;         /* consecutive wedged samples */
    static int64_t s_since_us;

    feed_json_tcp_t t;
    feed_json_tcp_census(&t);
    const char *why = census_wedged(&t);

    if (why) {
        if (++s_bad == 1) s_since_us = esp_timer_get_time();
        if (s_bad == 2)
            sys_log(4, "JSON     :%d refusing SYNs (%s): est %u syn %u fin %u tw %u "
                       "pcb %u/%u, backlog %d/%u, internal %u B",
                    FEED_PORT, why, t.established, t.syn_rcvd, t.closing, t.time_wait,
                    t.established + t.syn_rcvd + t.closing + t.other + t.time_wait,
                    t.pcb_limit, t.pending, t.backlog, (unsigned)t.internal_free);
    } else {
        if (s_bad >= 2)
            sys_log(1, "JSON     :%d accepting again after %d s", FEED_PORT,
                    (int)((esp_timer_get_time() - s_since_us) / 1000000));
        s_bad = 0;
    }
}

static void feed_task(void *arg)
{
    (void)arg;
    for (uint32_t tick = 0; ; tick++) {
        if (s_listen < 0) listen_open();
        if (s_listen >= 0) accept_new();

        poll_closed();
        if (tick % CENSUS_TICKS == 0) census_check();

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
