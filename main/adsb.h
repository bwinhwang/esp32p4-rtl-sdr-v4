#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "plane_cat.h"

/* ═════════════════════════════════════════════════════════════════════════════
 * The receiver's state as the display and the commands see it: the aircraft
 * table, the demodulator's rates, the event ring, the alert audio.
 * Implemented in class_driver.c.
 *
 * The table has no lock. adsb_rx_task is its only writer -- on_msg() and the
 * once-a-second tracker_tick() both run there -- and every reader here takes
 * a torn field as a wrong number for one frame. Anything that must write it
 * from another task defers through adsb_inject_test()'s mechanism.
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Table depth, not screen depth. A full table drops new contacts on the floor
 * (find_or_create() returns NULL) rather than evicting, so size it for the
 * busiest sky, not the screen. ~120 B each. */
#define MAX_TRACKED  64

typedef struct {
    int     raw_lat;
    int     raw_lon;
    int64_t ts_us;
    bool    valid;
} cpr_frame_t;

typedef struct {
    uint32_t    icao;
    char        callsign[9];
    int         altitude;
    int         velocity;
    int         heading;
    float       lat;
    float       lon;
    bool        pos_valid;
    float       dist_km;        /* from CONFIG_ADSB_RX_LAT/LON; valid with pos_valid */
    float       brg_deg;        /* 0..360, true north */
    int         ew_velocity;
    int         ns_velocity;
    int         vert_rate;      /* ft/min */
    int         squawk;         /* 0 = none received yet */
    uint8_t     sig;            /* signal_level of the last frame, 0-255, relative */
    int         msg_count;
    int64_t     last_seen_us;
    cpr_frame_t cpr_even;
    cpr_frame_t cpr_odd;
    plane_cat_t category;
    bool        active;
} aircraft_t;

/* MAX_TRACKED entries; `active` says which are live. */
const aircraft_t *adsb_aircraft(void);

typedef struct {
    int   msg_rate;         /* good frames in the last second */
    int   msg_total;
    float dec_pct;          /* frames passing CRC, of those with a plausible preamble; smoothed */
    float fix_pct;          /* of the good ones, those that needed a single-bit fix */
    float max_range_km;     /* farthest position decoded since boot */
} adsb_stats_t;

void adsb_stats_get(adsb_stats_t *s);

/* Expires stale contacts and closes the rate window. adsb_rx_task does this
 * itself; a display calls it so the table keeps moving with no dongle, and it
 * is a no-op while that task exists. */
void adsb_tick(void);

/* The 't' key: one synthetic contact, cycling the four categories. */
void adsb_inject_test(void);

/* ── event ring ────────────────────────────────────────────────────────────
 * The entry points (sys_log/air_log/ui_log) are in shell.h, since every
 * module logs; this is the read side for the display. Events only: an
 * air_log() line in colour 0 is a measurement and is echoed, never kept. */
typedef struct {
    char    text[80];
    uint8_t color;      /* 0 dim, 1 bright, 2 amber, 3 cyan, 4 red */
    uint8_t facility;   /* LOG_SYS / LOG_AIR / LOG_UI */
} log_entry_t;

/* Ring depth. A tall terminal's panel can show all of it; `log tail` reads it
 * with the board's lines included. Internal RAM (any context may log). */
#define ADSB_LOG_LINES  64

#define LOG_SYS  0
#define LOG_AIR  1
#define LOG_UI   2

/* Up to n entries, newest first, LOG_SYS skipped: the board's lines go to the
 * console, the panel is for the sky. Pointers into the ring -- read promptly. */
int adsb_log_recent(const log_entry_t **out, int n);

/* ── alert audio ────────────────────────────────────────────────────────── */
int  adsb_volume(void);
void adsb_set_volume(int pct);      /* clamped to 0..100 */
bool adsb_muted(void);
void adsb_set_muted(bool muted);
