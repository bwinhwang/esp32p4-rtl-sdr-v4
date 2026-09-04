#pragma once

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

int feed_json_clients(void);
