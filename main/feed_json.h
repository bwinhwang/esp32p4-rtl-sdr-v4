#pragma once

#include <stdint.h>
#include "esp_err.h"

/* JSON snapshot feed for the planned Android app: periodic (~1.3 Hz) full
 * aircraft-table snapshots as NDJSON on TCP :8888, schema modeled on
 * dump1090's aircraft.json. Port is arbitrary but deliberately outside the
 * dump1090/readsb 300xx family (:30001 AVR raw, :30005 Beast, :30003
 * reserved if SBS-1 is ever added) and off 80/8080 (reserved for the planned
 * HTTP config page) so none of the four collide.
 *
 * Unlike feed_avr.c/feed_beast.c this pushes one shared snapshot to every
 * client on a timer rather than queuing per-message frames: a slow client
 * just misses a tick instead of needing its own backpressure handling. */
esp_err_t feed_json_start(void);

int      feed_json_clients(void);
/* Clients this board dropped with a RST (stalled or send error), since boot. */
uint32_t feed_json_resets(void);

/* The board-wide TCP pcb picture plus :8888's listen backlog -- what decides
 * whether a new connection to this port is answered at all. See feed_json.c. */
typedef struct {
    uint8_t  established, syn_rcvd, closing, time_wait, other;
    uint8_t  pcb_limit;      /* CONFIG_LWIP_MAX_ACTIVE_TCP                 */
    int8_t   pending;        /* :8888 accepts_pending, -1 = not listening  */
    uint8_t  backlog;
    uint32_t internal_free;  /* bytes; lwIP's pcbs and pbufs come from here */
} feed_json_tcp_t;
void feed_json_tcp_census(feed_json_tcp_t *t);
