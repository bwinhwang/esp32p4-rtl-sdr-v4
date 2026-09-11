/*
 * Beast binary ADS-B feed -- TCP :30005, the framing dump1090/readsb call
 * "Beast": <0x1A> <'2'|'3'> <6-byte big-endian timestamp> <1-byte signal>
 * <7 or 14 bytes Mode-S data>, with every 0x1A inside those last three fields
 * doubled (byte-stuffed) so a bare 0x1A only ever marks a frame start.
 *
 * Structured exactly like feed_avr.c and for the same reason: on_msg() runs
 * on adsb_rx_task (core1), the demod hot path, so it only formats into a
 * ring; feed_task on core0 does every socket call, non-blocking.
 */

#include <errno.h>
#include <fcntl.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/sockets.h"

#include "feed_beast.h"
#include "mode-s.h"

#define FEED_PORT     30005
#define MAX_CLIENTS   4
#define RING_SLOTS    128   /* power of two */
/* Worst case: marker + type, then every one of the 6 timestamp + 1 signal +
 * 14 data bytes happens to be 0x1A and gets doubled. */
#define BEAST_LINE_MAX (2 + 2 * (6 + 1 + MODE_S_LONG_MSG_BYTES))
#define BATCH_MAX     2048
#define DRAIN_MS      20
#define STALL_LIMIT   50   /* consecutive full-buffer ticks before a client goes */

static const char *TAG = "beast";

typedef struct {
    uint8_t len;
    uint8_t s[BEAST_LINE_MAX];
} line_t;

static line_t s_ring[RING_SLOTS];

/* Single producer (adsb_rx_task) writes s_head, single consumer (feed_task)
 * writes s_tail -- see feed_avr.c's identical comment on why the
 * release/acquire pair here is load-bearing across the P4's two cores. */
static uint32_t s_head, s_tail;

static int      s_listen = -1;
static struct { int fd; uint16_t stall; } s_cli[MAX_CLIENTS];
static volatile int s_nclients;

static uint32_t s_sent, s_drop;
static uint8_t  s_batch[BATCH_MAX];

int      feed_beast_clients(void) { return s_nclients; }
uint32_t feed_beast_sent(void)    { return s_sent; }
uint32_t feed_beast_dropped(void) { return s_drop; }

void feed_beast_push(const unsigned char *msg, int msgbits,
                      uint64_t timestamp_12mhz, int signal_level)
{
    if (s_nclients == 0) return;

    int nbytes = msgbits / 8;
    if (nbytes != MODE_S_LONG_MSG_BYTES && nbytes != MODE_S_LONG_MSG_BYTES / 2)
        return; // Beast only frames Mode-S short (7B) and long (14B)

    uint32_t h = s_head;

    /* Conservative for the same reason as feed_avr.c: a stale tail can only
     * read as older than it is, never lets a slot the consumer still owns be
     * reused. */
    if (h - __atomic_load_n(&s_tail, __ATOMIC_RELAXED) >= RING_SLOTS) {
        s_drop++;
        return;
    }

    uint8_t sig = (uint8_t)(signal_level < 0 ? 0 : signal_level > 255 ? 255 : signal_level);
    uint8_t raw[6 + 1 + MODE_S_LONG_MSG_BYTES];
    int     n = 0;
    for (int i = 5; i >= 0; i--) raw[n++] = (uint8_t)(timestamp_12mhz >> (i * 8));
    raw[n++] = sig;
    memcpy(raw + n, msg, nbytes);
    n += nbytes;

    line_t  *l = &s_ring[h & (RING_SLOTS - 1)];
    uint8_t *p = l->s;
    *p++ = 0x1A;
    *p++ = (nbytes == MODE_S_LONG_MSG_BYTES) ? '3' : '2';
    for (int i = 0; i < n; i++) {
        *p++ = raw[i];
        if (raw[i] == 0x1A) *p++ = 0x1A;   /* byte-stuff */
    }
    l->len = (uint8_t)(p - l->s);

    __atomic_store_n(&s_head, h + 1, __ATOMIC_RELEASE);
}

/* `why` distinguishes "the peer hung up" from "we gave up on it" -- without
 * it every close looks identical in the log, and there is no way to tell a
 * client that reconnects on its own from one this board is dropping for
 * falling behind. */
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
    ESP_LOGI(TAG, "Beast listening on :%d", FEED_PORT);
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
    /* We batch a tick's worth of frames ourselves, so Nagle would only add
     * latency on top of that. */
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY,  &one, sizeof(one));
    setsockopt(fd, SOL_SOCKET,  SO_KEEPALIVE, &one, sizeof(one));
    fcntl(fd, F_SETFL, O_NONBLOCK);

    s_cli[slot].fd    = fd;
    s_cli[slot].stall = 0;
    s_nclients++;
    ESP_LOGI(TAG, "client connected, %d total", s_nclients);
}

static size_t drain(uint32_t *lines)
{
    size_t   n = 0;
    uint32_t t = s_tail;
    uint32_t h = __atomic_load_n(&s_head, __ATOMIC_ACQUIRE);

    *lines = 0;
    while (t != h && n + BEAST_LINE_MAX <= sizeof(s_batch)) {
        line_t *l = &s_ring[t & (RING_SLOTS - 1)];
        memcpy(s_batch + n, l->s, l->len);
        n += l->len;
        t++;
        (*lines)++;
    }
    __atomic_store_n(&s_tail, t, __ATOMIC_RELEASE);
    return n;
}

static void broadcast(const uint8_t *buf, size_t n)
{
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_cli[i].fd < 0) continue;

        int r = send(s_cli[i].fd, buf, n, 0);
        if (r == (int)n) {
            s_cli[i].stall = 0;
        } else if (r < 0 && (errno == EWOULDBLOCK || errno == EAGAIN)) {
            /* Reader can't keep up. Dropping its batch is the whole point of
             * not blocking here; only a client that never recovers is closed. */
            if (++s_cli[i].stall > STALL_LIMIT) client_close(i, "write stalled");
        } else if (r < 0) {
            client_close(i, strerror(errno));
        }
        /* A short write truncates one frame mid-stream. The next lone 0x1A
         * followed by a valid type byte resyncs both dump1090 and readsb. */
    }
}

static void poll_closed(void)
{
    char scratch[32];
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_cli[i].fd < 0) continue;
        /* An idle feed would otherwise not notice a FIN until the next frame. */
        int r = recv(s_cli[i].fd, scratch, sizeof(scratch), MSG_DONTWAIT);
        if (r == 0)
            client_close(i, "peer closed");
        else if (r < 0 && errno != EWOULDBLOCK && errno != EAGAIN)
            client_close(i, strerror(errno));
    }
}

static void feed_task(void *arg)
{
    (void)arg;
    while (1) {
        if (s_listen < 0) listen_open();
        if (s_listen >= 0) accept_new();

        poll_closed();

        uint32_t lines;
        size_t   n = drain(&lines);
        if (n && s_nclients) {
            broadcast(s_batch, n);
            s_sent += lines;
        }
        vTaskDelay(pdMS_TO_TICKS(DRAIN_MS));
    }
}

esp_err_t feed_beast_start(void)
{
    for (int i = 0; i < MAX_CLIENTS; i++) s_cli[i].fd = -1;

    /* core0: core1 is the demod loop. Below usb_recover_task's 4 so USB
     * recovery always wins; same priority as feed_avr's task since neither
     * competes meaningfully with the other. */
    BaseType_t ok = xTaskCreatePinnedToCore(feed_task, "beast", 4096, NULL, 3, NULL, 0);
    return ok == pdTRUE ? ESP_OK : ESP_FAIL;
}
