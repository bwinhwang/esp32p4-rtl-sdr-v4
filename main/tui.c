/*
 * The aircraft display -- see tui.h.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "adsb.h"
#include "screen.h"
#include "map.h"
#include "tui.h"

/* A repaint is ~8 KB and every byte out of UART0 costs ~7 us of core0, so
 * this sets core0 load directly (150 ms was 63% of the core). */
#define TUI_REFRESH_MS  500

#define TUI_COLS        120
#define TABLE_W         85      /* every table row is exactly this wide, and the map under it */
#define LOG_W           (TUI_COLS - TABLE_W - 2)    /* the column to the right of "│ " */
#define MAP_W           TABLE_W
/* The map's scale comes from its row count: the top and bottom rows are the
 * zoom range north and south of the antenna, and the width shows whatever
 * falls in it east and west. Rows are odd so the antenna has a centre row.
 * The map comes first: it never drops below MIN, the table gets what is left. */
#define MAP_MIN_ROWS    21
#define MAP_MAX_ROWS    39
#define TABLE_MIN_ROWS  5
/* Terminal cells are about this much taller than wide; land and blips use
 * the same figure or the coast drifts under the aircraft. */
#define MAP_ASPECT      2.2f
#define MAP_CX          (MAP_W / 2)
#define CHROME_ROWS     6       /* title, rule, header, rule, map title, footer */

#define DEFAULT_ROWS    40
#define STALE_S         15      /* a row dims after this long without a frame */
#define EL              "\033[K"

/* ── phosphor-green palette ─────────────────────────────────────────────── */
#define RESET    "\033[0m"
#define BOLD     "\033[1m"
#define PH_HI    "\033[38;2;0;255;80m"
#define PH_MID   "\033[38;2;0;200;60m"
#define PH_DIM   "\033[38;2;0;100;30m"
#define PH_SCAN  "\033[38;2;140;255;140m"
#define PH_GRID  "\033[38;2;0;60;20m"
#define AC_AMBER "\033[38;2;255;180;0m"
#define AC_RED   "\033[38;2;255;60;60m"
#define AC_CYAN  "\033[38;2;0;220;220m"
#define BG_LAND  "\033[48;2;0;36;16m"
#define FG_LAND  "\033[38;2;0;36;16m"   /* same shade: partial cells are drawn as fg blocks */
#define BG_SEA   "\033[49m"
#define FG_NONE  "\033[39m"
#define HL       "\xe2\x94\x80"
#define VL       "\xe2\x94\x82"

static const int   RANGES_KM[] = { 20, 50, 100, 150 };  /* zoom: antenna to the top row */
static const int   BAR_KM[]    = {  5, 20,  50,  50 };  /* scale bar for each zoom */
#define N_RANGES (int)(sizeof(RANGES_KM) / sizeof(RANGES_KM[0]))
enum { SORT_DIST, SORT_ALT, SORT_MSGS, SORT_SEEN, SORT_COUNT };
static const char *SORT_NAME[SORT_COUNT] = { "dist", "alt", "msgs", "seen" };

/* Scalars the key handler writes and the draw task reads; a frame drawn with
 * the old value is a frame late, nothing worse. */
static volatile int s_rows;
static volatile int s_sort  = SORT_DIST;
static volatile int s_range = 2;
static volatile float s_pan_e, s_pan_n;     /* view centre, km from the antenna; zoom keeps it */

static char s_rule[TUI_COLS * 3 + 1];

/* ═══════════════════════════════════════════════════════════════════════════
 * MAP
 * ═══════════════════════════════════════════════════════════════════════════ */

enum { C_NONE, C_GRID, C_DIM, C_MID, C_HI, C_RED, C_CYAN, C_SCAN, C_LAND };
static const char *COLOUR[] = { FG_NONE, PH_GRID, PH_DIM, PH_MID, PH_HI, AC_RED, AC_CYAN, PH_SCAN, FG_LAND };

/* Cell glyphs below 0x10 are multi-byte; ch holds the index. */
enum { G_N = 1, G_NE, G_E, G_SE, G_S, G_SW, G_W, G_NW, G_DOT, G_APT, G_BAR_L, G_BAR, G_BAR_R };
static const char *GLYPH[] = {
    NULL, "\xe2\x86\x91", "\xe2\x86\x97", "\xe2\x86\x92", "\xe2\x86\x98",     /* ↑ ↗ → ↘ */
    "\xe2\x86\x93", "\xe2\x86\x99", "\xe2\x86\x90", "\xe2\x86\x96",           /* ↓ ↙ ← ↖ */
    "\xe2\x80\xa2", "\xe2\x96\xaa", "\xe2\x94\x9c", HL, "\xe2\x94\xa4",       /* • ▪ ├ ─ ┤ */
};

/* Land is sampled 2x2 per cell; the mask bit order (TL TR BL BR) indexes
 * this table, so a cell with only its top half ashore prints as a half block. */
static const char *QUAD[16] = {
    " ",            "\xe2\x96\x98", "\xe2\x96\x9d", "\xe2\x96\x80",   /*   ▘ ▝ ▀ */
    "\xe2\x96\x96", "\xe2\x96\x8c", "\xe2\x96\x9e", "\xe2\x96\x9b",   /* ▖ ▌ ▞ ▛ */
    "\xe2\x96\x97", "\xe2\x96\x9a", "\xe2\x96\x90", "\xe2\x96\x9c",   /* ▗ ▚ ▐ ▜ */
    "\xe2\x96\x84", "\xe2\x96\x99", "\xe2\x96\x9f", " ",            /* ▄ ▙ ▟   */
};
static const uint8_t QUAD_N[16] = { 0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4 };

typedef struct {
    int     rows;                               /* in use this frame */
    char    ch[MAP_MAX_ROWS][MAP_W];
    uint8_t col[MAP_MAX_ROWS][MAP_W];
    uint8_t land[MAP_MAX_ROWS][MAP_W];          /* quadrant mask, see QUAD */
    uint8_t stack[MAP_MAX_ROWS][MAP_W];         /* aircraft already placed in this cell */
    uint8_t occ[MAP_MAX_ROWS][MAP_W];           /* taken by a blip, a label or the antenna */
} map_grid_t;

static EXT_RAM_BSS_ATTR map_grid_t s_map;   /* draw task only */

/* Aircraft folded into a shared headcount digit this frame -- '(%d shown)' on
 * the title already covers the table row limit, this covers the map's own
 * lossy step so MAP's header can say so instead of just looking sparse. */
static int  s_map_folded;
static bool s_map_ok;                       /* map_data.h was baked for this antenna */

static void mput(int y, int x, char ch, uint8_t col)
{
    if (y < 0 || y >= s_map.rows || x < 0 || x >= MAP_W) return;
    s_map.ch[y][x]  = ch;
    s_map.col[y][x] = col;
}

static bool free_run(int y, int x0, int n)
{
    if (x0 < 0 || x0 + n > MAP_W) return false;
    for (int x = x0; x < x0 + n; x++) if (s_map.occ[y][x]) return false;
    return true;
}

static void take_run(int y, int x0, int n)
{
    for (int x = x0; x < x0 + n; x++) s_map.occ[y][x] = 1;
}

static void map_land_cell(int y, int x) { s_map.land[y >> 1][x >> 1] |= 1 << ((y & 1) * 2 + (x & 1)); }

/* Nearest first, and one that would sit on a blip, a label or the antenna
 * gives way whole rather than leaving three letters with no marker. */
static void map_airport(int y, int x, const char *code)
{
    int n = 1 + (int)strlen(code);
    if (x + n > MAP_W) n = MAP_W - x;
    if (!free_run(y, x, n)) return;
    mput(y, x, G_APT, C_DIM);
    for (int i = 1; i < n; i++) mput(y, x + i, code[i - 1], C_DIM);
    take_run(y, x, n);
}

static uint8_t cat_colour(plane_cat_t c)
{
    switch (c) {
        case PLANE_MILITARY:   return C_RED;
        case PLANE_COMMERCIAL: return C_HI;
        case PLANE_GA:         return C_CYAN;
        default:               return C_MID;
    }
}

static bool ac_emergency(const aircraft_t *a)
{
    return a->squawk == 7500 || a->squawk == 7600 || a->squawk == 7700;
}

static uint8_t map_priority(const aircraft_t *a)
{
    if (ac_emergency(a)) return 0;
    switch (a->category) {
        case PLANE_MILITARY:   return 1;
        case PLANE_COMMERCIAL: return 2;
        case PLANE_GA:         return 3;
        default:                return 4;
    }
}

static uint8_t map_colour(const aircraft_t *a)
{
    return ac_emergency(a) ? C_RED : cat_colour(a->category);
}

/* Draw order for a shared cell: least important first, so the more important
 * target is the one left standing -- emergency squawk beats military beats
 * airline beats GA beats everything else, closer wins within a tier. Decoupled
 * from idx[]'s table order (s_sort may be alt/msgs/seen, not distance). */
static bool map_before(const aircraft_t *a, const aircraft_t *b)
{
    uint8_t pa = map_priority(a), pb = map_priority(b);
    if (pa != pb) return pa > pb;
    return a->dist_km > b->dist_km;
}

static void sort_map_cand(const aircraft_t *ac, int *cand, int n)
{
    for (int i = 1; i < n; i++) {
        int v = cand[i], j = i;
        while (j > 0 && map_before(&ac[v], &ac[cand[j - 1]])) { cand[j] = cand[j - 1]; j--; }
        cand[j] = v;
    }
}

static char heading_glyph(const aircraft_t *a)
{
    if (!a->velocity) return G_DOT;
    return (char)(G_N + ((a->heading + 22) / 45) % 8);
}

static void render_map(const aircraft_t *ac, const int *idx, int n, int rows)
{
    s_map.rows = rows;
    memset(s_map.ch,    ' ',    (size_t)rows * MAP_W);
    memset(s_map.col,   C_NONE, (size_t)rows * MAP_W);
    memset(s_map.land,  0,      (size_t)rows * MAP_W);
    memset(s_map.stack, 0,      (size_t)rows * MAP_W);
    memset(s_map.occ,   0,      (size_t)rows * MAP_W);

    const int   cy     = rows / 2;
    const float km_row = (float)RANGES_KM[s_range] / (float)cy;
    const float km_col = km_row / MAP_ASPECT;
    /* The antenna's cell, fractional and possibly off the map once panned. */
    const float cy_a   = (float)cy + s_pan_n / km_row;
    const float cx_a   = (float)MAP_CX - s_pan_e / km_col;
    const int   ay     = (int)lroundf(cy_a), ax = (int)lroundf(cx_a);
    const bool  a_in   = ay >= 0 && ay < rows && ax >= 0 && ax < MAP_W;

    /* Quarter cells, with the antenna still on the centre of its character. */
    if (s_map_ok) map_land(rows * 2, MAP_W * 2, cy_a * 2 + 0.5f, cx_a * 2 + 0.5f,
                           km_row / 2, km_col / 2, map_land_cell);

    /* Scale bar in the bottom-left corner, sized to the zoom. */
    int  bar_km = BAR_KM[s_range];
    int  bar    = (int)lroundf((float)bar_km / km_col);
    char bar_txt[12];
    int  bar_len = snprintf(bar_txt, sizeof(bar_txt), " %d km", bar_km);
    mput(rows - 1, 1, G_BAR_L, C_DIM);
    for (int x = 2; x < bar; x++) mput(rows - 1, x, G_BAR, C_DIM);
    mput(rows - 1, bar, G_BAR_R, C_DIM);
    for (int i = 0; i < bar_len; i++) mput(rows - 1, bar + 1 + i, bar_txt[i], C_DIM);
    take_run(rows - 1, 1, bar + bar_len);

    if (a_in) s_map.occ[ay][ax] = 1;            /* labels keep off the antenna */

    int cand[MAX_TRACKED], nc = 0;
    for (int k = 0; k < n; k++)
        if (ac[idx[k]].pos_valid) cand[nc++] = idx[k];
    sort_map_cand(ac, cand, nc);

    /* Blips first so no label is laid where a later blip lands; the blip is
     * a heading arrow, or a headcount digit once a second aircraft shares
     * the cell. */
    int px[MAX_TRACKED], py[MAX_TRACKED];
    for (int k = 0; k < nc; k++) {
        const aircraft_t *a = &ac[cand[k]];
        float b = a->brg_deg * (float)M_PI / 180.0f;
        px[k] = (int)lroundf(cx_a + a->dist_km * sinf(b) / km_col);
        py[k] = (int)lroundf(cy_a - a->dist_km * cosf(b) / km_row);
        if (py[k] < 0 || py[k] >= rows || px[k] < 0 || px[k] >= MAP_W) { px[k] = -1; continue; }
        int cnt = ++s_map.stack[py[k]][px[k]];
        mput(py[k], px[k], cnt == 1 ? heading_glyph(a) : cnt <= 9 ? (char)('0' + cnt) : '+', map_colour(a));
        s_map.occ[py[k]][px[k]] = 1;
    }

    /* Labels most important first: right of the blip, else left, else none. */
    for (int k = nc - 1; k >= 0; k--) {
        const aircraft_t *a = &ac[cand[k]];
        if (px[k] < 0 || s_map.stack[py[k]][px[k]] > 1 || !a->callsign[0]) continue;
        int len = 0;
        while (len < 3 && a->callsign[len]) len++;
        int x0;
        if      (free_run(py[k], px[k] + 1,   len)) x0 = px[k] + 1;
        else if (free_run(py[k], px[k] - len, len)) x0 = px[k] - len;
        else continue;
        for (int i = 0; i < len; i++) mput(py[k], x0 + i, a->callsign[i], map_colour(a));
        take_run(py[k], x0, len);
    }

    if (s_map_ok) map_airports(rows, MAP_W, cy_a, cx_a, km_row, km_col, map_airport);
    if (a_in && !s_map.stack[ay][ax]) mput(ay, ax, '+', C_SCAN);

    int folded = 0;
    for (int y = 0; y < rows; y++)
        for (int x = 0; x < MAP_W; x++)
            if (s_map.stack[y][x] >= 2) folded += s_map.stack[y][x];
    s_map_folded = folded;
}

/* One escape per colour run, not per cell; land is a background colour that
 * text sits on, so it is tracked separately from the foreground. An empty
 * cell on the coast is a block glyph in the land shade instead, and a cell
 * with text on it takes whichever the majority of its quarters is. */
static void emit_map_row(int y)
{
    uint8_t cur  = C_NONE;
    bool    land = false;
    for (int x = 0; x < MAP_W; x++) {
        char        ch = s_map.ch[y][x];
        uint8_t     m  = s_map.land[y][x];
        const char *q  = NULL;
        uint8_t     c  = cur;
        bool        l;
        if (ch != ' ' || s_map.occ[y][x]) {
            if (ch != ' ') c = s_map.col[y][x];
            l = QUAD_N[m] >= 2;
        } else if (m == 0 || m == 15) {
            l = m != 0;
        } else {
            c = C_LAND; l = false; q = QUAD[m];
        }
        if (l != land) { fb_puts(l ? BG_LAND : BG_SEA); land = l; }
        if (c != cur)  { fb_puts(COLOUR[c]); cur = c; }
        if (q)                             fb_puts(q);
        else if ((unsigned char)ch < 0x10) fb_puts(GLYPH[(int)ch]);
        else                               fb_putc(ch);
    }
    if (cur != C_NONE || land) fb_puts(RESET);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TABLE
 * ═══════════════════════════════════════════════════════════════════════════ */

static bool before(const aircraft_t *a, const aircraft_t *b)
{
    switch (s_sort) {
        case SORT_DIST:
            if (a->pos_valid != b->pos_valid) return a->pos_valid;
            if (a->pos_valid && a->dist_km != b->dist_km) return a->dist_km < b->dist_km;
            break;
        case SORT_ALT:
            if (a->altitude != b->altitude) return a->altitude > b->altitude;
            break;
        case SORT_MSGS:
            if (a->msg_count != b->msg_count) return a->msg_count > b->msg_count;
            break;
        default:
            break;
    }
    return a->last_seen_us > b->last_seen_us;
}

static void sort_idx(const aircraft_t *ac, int *idx, int n)
{
    for (int i = 1; i < n; i++) {
        int v = idx[i], j = i;
        while (j > 0 && before(&ac[v], &ac[idx[j - 1]])) { idx[j] = idx[j - 1]; j--; }
        idx[j] = v;
    }
}

static const char ROW_FMT[] = "  %06lX  %-8s  %-3s  %4s  %6s  %4s  %3s  %5s  %4s  %3s  %3u  %5d  %4d ";

static void draw_row(const aircraft_t *a, int64_t now)
{
    int  seen  = (int)((now - a->last_seen_us) / 1000000);
    bool stale = seen > STALE_S;
    bool emerg = a->squawk == 7500 || a->squawk == 7600 || a->squawk == 7700;

    char sqk[6] = "----", alt[8] = "--", spd[6] = "--", hdg[5] = "---";
    char vs[12] = "--",   dist[6] = "--", brg[5] = "---";
    if (a->squawk)   snprintf(sqk, sizeof(sqk), "%04d", a->squawk);
    if (a->altitude) snprintf(alt, sizeof(alt), "%d",   a->altitude);
    if (a->velocity) {
        snprintf(spd, sizeof(spd), "%d",   a->velocity);
        snprintf(hdg, sizeof(hdg), "%03d", a->heading);
    }
    int vr = a->vert_rate > 9999 ? 9999 : a->vert_rate < -9999 ? -9999 : a->vert_rate;
    if (vr > 200 || vr < -200) snprintf(vs, sizeof(vs), "%+d", vr);
    if (a->pos_valid) {
        snprintf(dist, sizeof(dist), "%.0f",   a->dist_km > 9999.0f ? 9999.0f : a->dist_km);
        snprintf(brg,  sizeof(brg),  "%03.0f", a->brg_deg);
    }
    int         msgs = a->msg_count > 99999 ? 99999 : a->msg_count;
    const char *cs   = a->callsign[0] ? a->callsign : "--------";
    const char *cat  = plane_cat_label(a->category);

    if (stale || emerg) {
        fb_puts(emerg ? AC_RED BOLD : PH_DIM);
        fb_printf(ROW_FMT, (unsigned long)a->icao, cs, cat, sqk, alt, spd, hdg, vs,
                  dist, brg, (unsigned)a->sig, msgs, seen);
        fb_puts(RESET);
        return;
    }

    const char *cat_col = a->category == PLANE_MILITARY   ? AC_RED
                        : a->category == PLANE_COMMERCIAL ? PH_MID
                        : a->category == PLANE_GA         ? AC_CYAN : PH_DIM;
    const char *alt_col = vr > 200 ? PH_HI : vr < -200 ? AC_AMBER : PH_MID;
    const char *vs_col  = vr > 200 ? PH_HI : vr < -200 ? AC_AMBER : PH_DIM;

    fb_printf("  " AC_CYAN "%06lX"
              "  %s%-8s"
              "  %s%-3s"
              "  " PH_MID "%4s"
              "  %s%6s"
              "  " PH_MID "%4s  %3s"
              "  %s%5s"
              "  " PH_MID "%4s  %3s  %3u"
              "  " PH_DIM "%5d  %4d " RESET,
              (unsigned long)a->icao,
              a->callsign[0] ? PH_HI : PH_DIM, cs,
              cat_col, cat,
              sqk,
              alt_col, alt,
              spd, hdg,
              vs_col, vs,
              dist, brg, (unsigned)a->sig,
              msgs, seen);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * FRAME
 * ═══════════════════════════════════════════════════════════════════════════ */

static void rule(int cols) { fb_printf(PH_GRID "%.*s" RESET, cols * 3, s_rule); }

/* The map comes first: it takes the terminal minus the table's need (sized
 * to its contents in steps of five so the map does not hop on every contact),
 * but never less than MAP_MIN_ROWS -- a crowded table loses rows ("(n shown)"
 * on the title) before the map shrinks. Past MAP_MAX_ROWS the table gets the
 * surplus; on a very short terminal the table still keeps TABLE_MIN_ROWS. */
static void layout(int n, int *table_rows, int *map_rows)
{
    int avail = (s_rows > 0 ? s_rows : DEFAULT_ROWS) - CHROME_ROWS;
    int t = (n + 4) / 5 * 5;
    if (t < TABLE_MIN_ROWS) t = TABLE_MIN_ROWS;
    int r = avail - t;
    if (r < MAP_MIN_ROWS) r = MAP_MIN_ROWS;
    if (r > MAP_MAX_ROWS) r = MAP_MAX_ROWS;
    if (avail - r < TABLE_MIN_ROWS) r = avail - TABLE_MIN_ROWS;
    if (!(r & 1)) r--;
    if (r < 3) r = 3;
    t = avail - r;
    *table_rows = t < 0 ? 0 : t;
    *map_rows   = r;
}

static void draw_title(const adsb_stats_t *st, int live, int shown, int64_t now)
{
    unsigned s = (unsigned)(now / 1000000);
    char up[24];
    if (s >= 86400) snprintf(up, sizeof(up), "%ud %02u:%02u", s / 86400, (s % 86400) / 3600, (s % 3600) / 60);
    else            snprintf(up, sizeof(up), "%02u:%02u:%02u", s / 3600, (s % 3600) / 60, s % 60);

    fb_printf(PH_HI BOLD " ADS-B 1090" RESET
              PH_DIM "  UP " PH_HI "%s"
              PH_DIM "  ACFT " PH_HI BOLD "%d" RESET, up, live);
    if (live > shown) fb_printf(PH_DIM " (%d shown)", shown);
    fb_printf(PH_DIM "  MSG/S " PH_SCAN "%d"
              PH_DIM "  TOTAL " PH_MID "%d"
              PH_DIM "  DEC " PH_HI "%.1f%%"
              PH_DIM "  FIX " PH_MID "%.1f%%"
              PH_DIM "  MAX " PH_HI "%.0f km",
              st->msg_rate, st->msg_total, st->dec_pct, st->fix_pct, st->max_range_km);
    if (adsb_muted()) fb_puts(AC_RED "  MUTED" RESET EL "\n");
    else              fb_printf(PH_DIM "  VOL " PH_HI "%d" RESET EL "\n", adsb_volume());
}

static void tui_draw(int64_t now)
{
    adsb_tick();

    adsb_stats_t st;
    adsb_stats_get(&st);
    const aircraft_t *ac = adsb_aircraft();

    int idx[MAX_TRACKED], n = 0;
    for (int i = 0; i < MAX_TRACKED; i++)
        if (ac[i].active) idx[n++] = i;
    sort_idx(ac, idx, n);

    int table_rows, map_rows;
    layout(n, &table_rows, &map_rows);
    render_map(ac, idx, n, map_rows);

    /* The map can't plot what has no fix yet, and folds a real collision
     * into one headcount digit -- say so on its title instead of just looking
     * sparse next to the table's ACFT count. */
    int n_nofix = 0;
    for (int i = 0; i < n; i++) if (!ac[idx[i]].pos_valid) n_nofix++;
    char note[80] = "";
    int  nl = 0;
    if (s_map_folded && n_nofix) nl = snprintf(note, sizeof(note), " (%d folded, %d no-fix)", s_map_folded, n_nofix);
    else if (s_map_folded)       nl = snprintf(note, sizeof(note), " (%d folded)", s_map_folded);
    else if (n_nofix)            nl = snprintf(note, sizeof(note), " (%d no-fix)", n_nofix);
    if (!s_map_ok) snprintf(note + nl, sizeof(note) - nl, " (map baked elsewhere: rerun tools/mkmap.py)");

    /* The log column runs beside everything -- table, rule, map title and
     * map -- newest first from the top, so it is as tall as the terminal
     * and the latest event is always level with the table header. */
    static const char *LOG_COLOUR[] = { PH_DIM, PH_HI, AC_AMBER, AC_CYAN, AC_RED };
    const log_entry_t *e[ADSB_LOG_LINES];
    int rows = table_rows + 2 + map_rows;
    int ne   = adsb_log_recent(e, rows < ADSB_LOG_LINES ? rows : ADSB_LOG_LINES);

    fb_puts("\033[H");
    draw_title(&st, n, n < table_rows ? n : table_rows, now);
    rule(TUI_COLS);
    fb_puts(EL "\n");

    fb_printf(PH_MID "  %-6s  %-8s  %-3s  %4s  %6s  %4s  %3s  %5s  %4s  %3s  %3s  %5s  %4s " RESET,
              "ICAO", "CALLSIGN", "CAT", "SQK", "ALT ft", "SPD", "HDG", "V/S",
              "DIST", "BRG", "SIG", "MSGS", "SEEN");
    fb_puts(PH_GRID VL RESET PH_DIM " EVENT LOG" RESET EL "\n");

    for (int r = 0; r < rows; r++) {
        if (r < table_rows) {
            if (r < n) draw_row(&ac[idx[r]], now);
            else       fb_rep(' ', TABLE_W);
        } else if (r == table_rows) {
            rule(TABLE_W);
        } else if (r == table_rows + 1) {
            char hdr[64];
            int  len = snprintf(hdr, sizeof(hdr), "  MAP  %d km", RANGES_KM[s_range]);
            float pe = s_pan_e, pn = s_pan_n;
            if (pe != 0 || pn != 0) {
                len += snprintf(hdr + len, sizeof(hdr) - len, "   centre");
                if (pn != 0) len += snprintf(hdr + len, sizeof(hdr) - len, " %.0f km %c", fabsf(pn), pn > 0 ? 'N' : 'S');
                if (pe != 0) len += snprintf(hdr + len, sizeof(hdr) - len, " %.0f km %c", fabsf(pe), pe > 0 ? 'E' : 'W');
            }
            fb_printf(PH_MID "%s" PH_DIM "%-*.*s" RESET, hdr, TABLE_W - len, TABLE_W - len, note);
        } else {
            emit_map_row(r - table_rows - 2);
        }
        fb_puts(PH_GRID VL RESET " ");
        if (r < ne) fb_printf("%s%.*s" RESET, LOG_COLOUR[e[r]->color < 5 ? e[r]->color : 0], LOG_W, e[r]->text);
        fb_puts(EL "\n");
    }

    fb_printf(PH_DIM " [Q] SHELL   [S] SORT %s   [<] [>] ZOOM %d km   [ARROWS] PAN   [C] CENTRE   [M] MUTE   [+/-] VOL   [T] TEST"
              RESET EL "\033[J", SORT_NAME[s_sort], RANGES_KM[s_range]);
}

/* An eighth of the view per press, at the current zoom and height, kept
 * inside the baked box so the map cannot scroll off its own data. */
static void pan(int dx, int dy)
{
    int   rows   = s_map.rows ? s_map.rows : MAP_MIN_ROWS;
    float km_row = (float)RANGES_KM[s_range] / (float)(rows / 2);
    float km_col = km_row / MAP_ASPECT;
    float ew, ns;
    map_extent(&ew, &ns);
    s_pan_e = fminf(ew, fmaxf(-ew, s_pan_e + (float)dx * MAP_W * km_col / 8));
    s_pan_n = fminf(ns, fmaxf(-ns, s_pan_n + (float)dy * rows  * km_row / 8));
}

/* One keystroke, from whichever console task read it. The keys that leave
 * the display never reach here. Arrow keys arrive as ESC [ A..D (ESC O A..D
 * in application mode) one byte at a time, so a two-state parser folds them
 * onto hjkl; an ESC followed by anything else is dropped and the next byte
 * is taken as itself. */
static void tui_key(uint8_t key)
{
    static int esc;
    if (esc == 1) { esc = (key == '[' || key == 'O') ? 2 : 0; if (esc) return; }
    else if (esc == 2) {
        esc = 0;
        switch (key) {
            case 'A': key = 'k'; break;
            case 'B': key = 'j'; break;
            case 'C': key = 'l'; break;
            case 'D': key = 'h'; break;
            case 'H': key = 'c'; break;
            default:             return;
        }
    }
    switch (key) {
        case 0x1b:          esc = 1;                                    return;
        case 's': case 'S': s_sort = (s_sort + 1) % SORT_COUNT;         break;
        case '<': case ',': case '[': if (s_range > 0) s_range--;      break;
        case '>': case '.': case ']': if (s_range < N_RANGES - 1) s_range++; break;
        case 'h': case 'H': pan(-1,  0);                                break;
        case 'l': case 'L': pan( 1,  0);                                break;
        case 'k': case 'K': pan( 0,  1);                                break;
        case 'j': case 'J': pan( 0, -1);                                break;
        case 'c': case 'C': s_pan_e = s_pan_n = 0;                      break;
        case 'm': case 'M': adsb_set_muted(!adsb_muted());              break;
        case '+': case '=': adsb_set_volume(adsb_volume() + 10);        break;
        case '-': case '_': adsb_set_volume(adsb_volume() - 10);        break;
        case 't': case 'T': adsb_inject_test();                         return;
        default:                                                        return;
    }
    screen_wake(SCREEN_TUI);
}

void tui_set_rows(int rows) { s_rows = rows; }

void tui_init(void)
{
    for (int i = 0; i < TUI_COLS; i++) memcpy(s_rule + i * 3, HL, 3);
    s_rule[TUI_COLS * 3] = '\0';
    s_map_ok = map_matches(strtof(CONFIG_ADSB_RX_LAT, NULL), strtof(CONFIG_ADSB_RX_LON, NULL));
    screen_register(SCREEN_TUI, "the display", TUI_COLS, TUI_REFRESH_MS, tui_draw, tui_key);
}
