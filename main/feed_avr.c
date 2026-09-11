/*
 * AVR raw ADS-B feed -- "*8D4B1A...;\n" per frame, TCP :30001.
 *
 * Split across two tasks on purpose. on_msg() runs on adsb_rx_task (core1),
 * which is the demod hot path: a blocking send() there stalls the 2 MSPS loop
 * and drops IQ. So the producer only formats into a ring, and feed_task on
 * core0 does every socket call, non-blocking.
 */

#include <errno.h>
#include <fcntl.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "lwip/sockets.h"

#include "feed_avr.h"
#include "mode-s.h"

#define FEED_PORT     30001
#define MAX_CLIENTS   4
#define RING_SLOTS    128                                 /* power of two */
#define AVR_LINE_MAX      (1 + MODE_S_LONG_MSG_BYTES * 2 + 2) /* '*' + hex + ";\n" */
#define BATCH_MAX     2048
#define DRAIN_MS      20
#define STALL_LIMIT   50   /* consecutive full-buffer ticks before a client goes */

static const char *TAG = "feed";

typedef struct {
    uint8_t len;
    char    s[AVR_LINE_MAX];
} line_t;

static EXT_RAM_BSS_ATTR line_t s_ring[RING_SLOTS];   /* task-context only, see class_driver.c */

/* Single producer (adsb_rx_task) writes s_head, single consumer (feed_task)
 * writes s_tail. The release/acquire pair is load-bearing on the P4's two
 * cores: without it the consumer can observe the new head before the slot
 * contents it points at. */
static uint32_t s_head, s_tail;

static int      s_listen = -1;
static struct { int fd; uint16_t stall; } s_cli[MAX_CLIENTS];
static volatile int s_nclients;

static uint32_t s_sent, s_drop;
static EXT_RAM_BSS_ATTR char s_batch[BATCH_MAX];

int      feed_avr_clients(void) { return s_nclients; }
uint32_t feed_avr_sent(void)    { return s_sent; }
uint32_t feed_avr_dropped(void) { return s_drop; }

void feed_avr_push(const unsigned char *msg, int msgbits)
{
    if (s_nclients == 0) return;

    int nbytes = msgbits / 8;
    if (nbytes <= 0 || nbytes > MODE_S_LONG_MSG_BYTES) return;

    static const char HEX[] = "0123456789ABCDEF";
    uint32_t h = s_head;

    /* A stale tail can only read as older than it is, which makes this test
     * conservative -- it never lets a slot the consumer still owns be reused. */
    if (h - __atomic_load_n(&s_tail, __ATOMIC_RELAXED) >= RING_SLOTS) {
        s_drop++;
        return;
    }

    line_t *l = &s_ring[h & (RING_SLOTS - 1)];
    char   *p = l->s;
    *p++ = '*';
    for (int i = 0; i < nbytes; i++) {
        *p++ = HEX[msg[i] >> 4];
        *p++ = HEX[msg[i] & 0x0F];
    }
    *p++ = ';';
    *p++ = '\n';
    l->len = (uint8_t)(p - l->s);

    __atomic_store_n(&s_head, h + 1, __ATOMIC_RELEASE);
}

static void client_close(int i)
{
    close(s_cli[i].fd);
    s_cli[i].fd    = -1;
    s_cli[i].stall = 0;
    s_nclients--;
    ESP_LOGI(TAG, "client gone, %d left", s_nclients);
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
    ESP_LOGI(TAG, "AVR raw listening on :%d", FEED_PORT);
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
    while (t != h && n + AVR_LINE_MAX <= sizeof(s_batch)) {
        line_t *l = &s_ring[t & (RING_SLOTS - 1)];
        memcpy(s_batch + n, l->s, l->len);
        n += l->len;
        t++;
        (*lines)++;
    }
    __atomic_store_n(&s_tail, t, __ATOMIC_RELEASE);
    return n;
}

static void broadcast(const char *buf, size_t n)
{
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_cli[i].fd < 0) continue;

        int r = send(s_cli[i].fd, buf, n, 0);
        if (r == (int)n) {
            s_cli[i].stall = 0;
        } else if (r < 0 && (errno == EWOULDBLOCK || errno == EAGAIN)) {
            /* Reader can't keep up. Dropping its batch is the whole point of
             * not blocking here; only a client that never recovers is closed. */
            if (++s_cli[i].stall > STALL_LIMIT) client_close(i);
        } else if (r < 0) {
            client_close(i);
        }
        /* A short write truncates one line mid-hex. Both dump1090 and readsb
         * resync on the next '*', so the peer loses that one frame. */
    }
}

static void poll_closed(void)
{
    char scratch[32];
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_cli[i].fd < 0) continue;
        /* An idle feed would otherwise not notice a FIN until the next frame. */
        int r = recv(s_cli[i].fd, scratch, sizeof(scratch), MSG_DONTWAIT);
        if (r == 0 || (r < 0 && errno != EWOULDBLOCK && errno != EAGAIN))
            client_close(i);
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

esp_err_t feed_avr_start(void)
{
    for (int i = 0; i < MAX_CLIENTS; i++) s_cli[i].fd = -1;

    /* core0: core1 is the demod loop. Below usb_recover_task's 4 so USB
     * recovery always wins. */
    BaseType_t ok = xTaskCreatePinnedToCore(feed_task, "feed", 4096, NULL, 3, NULL, 0);
    return ok == pdTRUE ? ESP_OK : ESP_FAIL;
}
