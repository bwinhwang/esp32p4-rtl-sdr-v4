/*
 * JSON snapshot feed -- TCP :8888, one NDJSON line per tick with the full
 * aircraft table, for the planned Android app.
 *
 * Deliberately simpler than feed_avr.c/feed_beast.c: those queue individual
 * decoded frames from the demod hot path and must never block it, so they
 * need a lock-free ring between producer and sender. This feed instead has
 * exactly one producer call (aircraft_export_ndjson(), in class_driver.c)
 * made from this module's own task right before it broadcasts -- there is
 * no ring, no backpressure logic, and a client that can't keep up just
 * misses that tick's snapshot and gets the next one whole.
 */

#include <errno.h>
#include <fcntl.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "lwip/sockets.h"

#include "feed_json.h"

/* Defined in class_driver.c, where s_aircraft[] lives; not put in a shared
 * header to avoid a dependency edge from this feed module into the TUI's
 * internals -- matches net_eth.c's extern declaration of
 * esp_libusb_stream_dropped() for the same reason. */
extern size_t aircraft_export_ndjson(char *buf, size_t bufsize);

#define FEED_PORT     8888
#define MAX_CLIENTS   4
#define TICK_MS       750   /* ~1.3 Hz, inside the plan's 1-2 Hz target */
#define SNAPSHOT_MAX  4096  /* MAX_TRACKED=16 aircraft, ~200B each, generous */

static const char *TAG = "json";

static int  s_listen = -1;
static int  s_cli[MAX_CLIENTS];
static volatile int s_nclients;
static EXT_RAM_BSS_ATTR char s_snapshot[SNAPSHOT_MAX];   /* task-context only, see class_driver.c */

int feed_json_clients(void) { return s_nclients; }

static void client_close(int i)
{
    close(s_cli[i]);
    s_cli[i] = -1;
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
    ESP_LOGI(TAG, "JSON snapshot listening on :%d", FEED_PORT);
}

static void accept_new(void)
{
    int fd = accept(s_listen, NULL, NULL);
    if (fd < 0) return;

    int slot = -1;
    for (int i = 0; i < MAX_CLIENTS; i++)
        if (s_cli[i] < 0) { slot = i; break; }
    if (slot < 0) { close(fd); return; }

    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    setsockopt(fd, SOL_SOCKET,  SO_KEEPALIVE, &one, sizeof(one));
    fcntl(fd, F_SETFL, O_NONBLOCK);

    s_cli[slot] = fd;
    s_nclients++;
    ESP_LOGI(TAG, "client connected, %d total", s_nclients);
}

static void poll_closed(void)
{
    char scratch[32];
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_cli[i] < 0) continue;
        int r = recv(s_cli[i], scratch, sizeof(scratch), MSG_DONTWAIT);
        if (r == 0 || (r < 0 && errno != EWOULDBLOCK && errno != EAGAIN))
            client_close(i);
    }
}

static void broadcast(const char *buf, size_t n)
{
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_cli[i] < 0) continue;
        /* No retry and no partial-write bookkeeping: this is the whole
         * point of the one-shared-snapshot design (see file header) -- a
         * short write or EWOULDBLOCK here just costs that client one tick,
         * never a queue to catch up on. Only a hard error drops it. */
        int r = send(s_cli[i], buf, n, 0);
        if (r < 0 && errno != EWOULDBLOCK && errno != EAGAIN)
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

        if (s_nclients) {
            size_t n = aircraft_export_ndjson(s_snapshot, sizeof(s_snapshot));
            broadcast(s_snapshot, n);
        }
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
    }
}

esp_err_t feed_json_start(void)
{
    for (int i = 0; i < MAX_CLIENTS; i++) s_cli[i] = -1;

    /* core0, same as feed_avr/feed_beast and for the same reason: core1 is
     * the demod loop. */
    BaseType_t ok = xTaskCreatePinnedToCore(feed_task, "json", 4096, NULL, 3, NULL, 0);
    return ok == pdTRUE ? ESP_OK : ESP_FAIL;
}
