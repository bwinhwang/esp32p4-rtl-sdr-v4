#pragma once

#include <stdint.h>
#include "esp_err.h"

/* AVR raw feed: hex Mode-S frames on TCP :30001, the format dump1090 and readsb
 * both accept as input. Starts before any interface has an address. */
esp_err_t feed_avr_start(void);

/* Formats and queues one frame. Called from on_msg() on the demod task, so it
 * never blocks and never touches a socket -- see feed_avr.c. */
void feed_avr_push(const unsigned char *msg, int msgbits);

int      feed_avr_clients(void);
uint32_t feed_avr_sent(void);
uint32_t feed_avr_dropped(void);
