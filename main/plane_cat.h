#pragma once

#include <stdint.h>

typedef enum {
    PLANE_UNKNOWN = 0,
    PLANE_COMMERCIAL,
    PLANE_GA,
    PLANE_MILITARY,
} plane_cat_t;

/* callsign may be NULL or empty -- the ICAO range check still applies.
 * emitter is the TC1-4 category in readsb's byte form (0xA0..0xD7, 0 = not
 * seen); it only settles what the address and callsign leave unknown. */
plane_cat_t plane_classify(uint32_t icao, const char *callsign, uint8_t emitter);
const char *plane_cat_label(plane_cat_t c);
/* "A3" into buf (>= 4 bytes), "" for 0. Returns buf. */
const char *plane_emitter_label(uint8_t emitter, char *buf);
