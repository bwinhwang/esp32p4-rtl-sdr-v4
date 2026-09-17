#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
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
 * busiest sky, not the screen. ~550 B each, in PSRAM. */
#define MAX_TRACKED  64

/* One JSON snapshot of the whole table (feed_json.c, /aircraft.json). A row
 * with every optional field present is ~650 B. */
#define ADSB_SNAPSHOT_MAX  49152

typedef struct {
    int     raw_lat;
    int     raw_lon;
    int64_t ts_us;
    bool    surface;    /* TC5-8 encoding; pairs only with its own kind */
    bool    valid;
} cpr_frame_t;

/* A value with the time it was last reported; us == 0 means never. The JSON
 * export shows it while under OPT_FRESH_US old -- these come from registers
 * an interrogator may stop asking for while the contact stays alive. `es`
 * marks an extended-squitter source, which a Comm-B reading of the same
 * quantity does not replace while fresh. */
typedef struct {
    int64_t us;
    float   v;
    bool    es;
} opt_f_t;

#define OPT_FRESH_US  60000000LL
#define FRESH_AT(us, now)  ((us) && (now) - (us) < OPT_FRESH_US)
#define OPT_FRESH(o, now)  FRESH_AT((o).us, now)

typedef struct {
    uint32_t    icao;
    char        callsign[9];
    int64_t     cs_adsb_us;     /* last TC1-4 callsign; a BDS 2,0 one only fills in when stale */
    int         altitude;
    bool        alt_geom;       /* altitude is geometric (TC20-22), not barometric */
    bool        on_ground;      /* surface position, or a ground flag in FS/VS/CA */
    int         velocity;
    int         heading;
    int64_t     vel_us;         /* last ADS-B velocity (TC19 / TC5-8); BDS 5,0 fills in when stale */
    float       lat;
    float       lon;
    bool        pos_valid;
    int64_t     pos_us;         /* when lat/lon were last accepted */
    uint8_t     pos_rejects;    /* consecutive fixes dropped as implausible */
    float       dist_km;        /* from the antenna position; valid with rng_valid */
    float       brg_deg;        /* 0..360, true north */
    bool        rng_valid;      /* pos_valid AND an antenna position is set */
    int         ew_velocity;
    int         ns_velocity;
    int         vert_rate;      /* ft/min */
    int         squawk;         /* 0 = none received yet */
    uint8_t     emergency;      /* TC28 state, 0 = none; the 7x00 squawks are the other signal */
    uint8_t     emitter;        /* TC1-4 category as readsb's byte, 0xA0..0xD7; 0 = not yet seen */
    bool        alert;          /* FS/SS: squawk changed or emergency; held STATUS_HOLD_US */
    bool        spi;            /* FS/SS: IDENT pressed; same hold */
    int64_t     status_us;      /* last message carrying alert/spi */
    bool        ra_active;      /* ACAS RA in progress: cleared by "clear of conflict" or RA_HOLD_US */
    int64_t     ra_us;
    char        ra_text[48];    /* the advisory, as logged */

    /* ── the rest of readsb's aircraft.json: JSON feed only, nothing on
     * screen reads them ── */
    opt_f_t     ias, tas, mach;             /* TC19 sub 3/4, BDS 5,0 / 6,0 */
    opt_f_t     roll, track_rate;           /* BDS 5,0 */
    opt_f_t     mag_heading;                /* BDS 6,0 */
    opt_f_t     baro_rate, geom_rate;       /* TC19 by its source bit, BDS 6,0 */
    opt_f_t     geom_delta;                 /* TC19: geometric minus barometric, gives the other altitude */
    opt_f_t     nav_alt_mcp, nav_alt_fms, nav_qnh, nav_heading;   /* TC29, BDS 4,0 */
    opt_f_t     nav_modes;                  /* MODE_S_NAV_MODE_* bits in v */
    opt_f_t     nac_v;                      /* TC19, TC31 surface */
    int64_t     acc_us;                     /* nac_p / sil / sil_type / nic_baro: TC29 or TC31 */
    uint8_t     nac_p, sil, sil_type, nic_baro;
    int64_t     ops_us;                     /* TC31 */
    uint8_t     version, nic_a, nic_c, gva, sda;
    bool        sda_valid, gva_valid, cc_acas;
    uint8_t     nic;                        /* of the last accepted position; valid with pos_valid */
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

/* ── antenna position ──────────────────────────────────────────────────────
 * NVS only (namespace "rxcfg"), no compile-time default: one image serves
 * any location, and until a position is set every contact has rng_valid
 * false rather than a plausible range against someone else's home.
 * adsb_pos_init() loads it -- call after nvs_flash_init(), before the
 * display or the demodulator can run. The setters persist first, then hand
 * the change to adsb_rx_task, which re-ranges the whole table (same rule as
 * adsb_inject_test(): that task is the table's only writer). */
void      adsb_pos_init(void);
bool      adsb_pos_get(float *lat, float *lon);     /* false when unset */
bool      adsb_pos_parse(const char *lat_s, const char *lon_s, float *lat, float *lon);
esp_err_t adsb_pos_set(float lat, float lon);       /* validates, persists */
esp_err_t adsb_pos_clear(void);

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
