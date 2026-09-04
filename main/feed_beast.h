#pragma once

#include <stdint.h>
#include "esp_err.h"

/* Beast binary feed: dump1090/readsb's binary framing on TCP :30005 -- the
 * same demod output as feed_avr.c's raw-hex feed, but carrying the two
 * fields Beast frames have and AVR raw doesn't: a per-message timestamp and
 * a coarse signal-level byte. Starts before any interface has an address. */
esp_err_t feed_beast_start(void);

/* Formats and queues one frame. Called from on_msg() on the demod task, so it
 * never blocks and never touches a socket -- see feed_beast.c.
 * timestamp_12mhz is a receiver-local monotonic 12 MHz tick count (see
 * mode-s.h) -- fine for feeder compatibility, not GPS/PPS-synced so not for
 * real cross-receiver MLAT. signal_level is 0-255, relative only. */
void feed_beast_push(const unsigned char *msg, int msgbits,
                      uint64_t timestamp_12mhz, int signal_level);

int      feed_beast_clients(void);
uint32_t feed_beast_sent(void);
uint32_t feed_beast_dropped(void);
