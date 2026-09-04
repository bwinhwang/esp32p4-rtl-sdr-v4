#pragma once

#include <stdint.h>

typedef enum {
    PLANE_UNKNOWN = 0,
    PLANE_COMMERCIAL,
    PLANE_GA,
    PLANE_MILITARY,
} plane_cat_t;

/* callsign may be NULL or empty -- the ICAO range check still applies. */
plane_cat_t plane_classify(uint32_t icao, const char *callsign);
const char *plane_cat_label(plane_cat_t c);
