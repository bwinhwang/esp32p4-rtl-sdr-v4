/*
 * The aircraft display -- see tui.h.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_timer.h"
#include "adsb.h"
#include "screen.h"
#include "tui.h"

/* A repaint is ~8 KB and every byte out of UART0 costs ~7 us of core0, so
 * this sets core0 load directly (150 ms was 63% of the core). */
#define TUI_REFRESH_MS  500

#define TUI_COLS        120
#define TABLE_W         85      /* every table row is exactly this wide */
#define RADAR_COLS      33      /* odd, so the crosshair has a centre column */
#define RADAR_ROWS      15
/* Terminal cells are about this much taller than wide; rings, sweep and blip
 * placement all use the same figure or a blip lands off its own ring. */
#define RADAR_ASPECT    2.2f
#define RADAR_CX        (RADAR_COLS / 2)
#define RADAR_CY        (RADAR_ROWS / 2)
#define LOG_MIN         6
#define CHROME_ROWS     6       /* title, rule, header, rule, log title, footer */
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
#define HL       "\xe2\x94\x80"
#define VL       "\xe2\x94\x82"

static const int   RANGES_KM[] = { 50, 100, 200 };
enum { SORT_DIST, SORT_ALT, SORT_MSGS, SORT_SEEN, SORT_COUNT };
static const char *SORT_NAME[SORT_COUNT] = { "dist", "alt", "msgs", "seen" };

/* Scalars the key handler writes and the draw task reads; a frame drawn with
 * the old value is a frame late, nothing worse. */
static volatile int s_rows;
static volatile int s_sort  = SORT_DIST;
static volatile int s_range = 1;

static int  s_sweep;
static char s_rule[TUI_COLS * 3 + 1];

/* ═══════════════════════════════════════════════════════════════════════════
 * RADAR
 * ═══════════════════════════════════════════════════════════════════════════ */

enum { C_NONE, C_GRID, C_DIM, C_MID, C_SCAN, C_HI, C_RED, C_CYAN };
static const char *COLOUR[] = { RESET, PH_GRID, PH_DIM, PH_MID, PH_SCAN, PH_HI, AC_RED, AC_CYAN };

typedef struct {
    char    ch[RADAR_ROWS][RADAR_COLS];
    uint8_t col[RADAR_ROWS][RADAR_COLS];
} radar_t;

static radar_t s_radar;     /* draw task only */

static void rput(int y, int x, char ch, uint8_t col)
{
    if (y < 0 || y >= RADAR_ROWS || x < 0 || x >= RADAR_COLS) return;
    s_radar.ch[y][x]  = ch;
    s_radar.col[y][x] = col;
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

/* idx[] is the sorted table; drawn farthest first so a near blip wins a
 * shared cell. */
static void render_radar(const aircraft_t *ac, const int *idx, int n)
{
    memset(s_radar.ch,  ' ',    sizeof(s_radar.ch));
    memset(s_radar.col, C_NONE, sizeof(s_radar.col));

    const float range = (float)RANGES_KM[s_range];
    const float rx = (float)RADAR_CX * 0.90f;
    const float ry = rx / RADAR_ASPECT;
    const float sa = (float)s_sweep * (float)M_PI / 180.0f;

    for (int y = 0; y < RADAR_ROWS; y++) {
        for (int x = 0; x < RADAR_COLS; x++) {
            float dx = (float)(x - RADAR_CX);
            float dy = (float)(y - RADAR_CY) * RADAR_ASPECT;
            float d  = sqrtf(dx * dx + dy * dy);
            if      (fabsf(d - rx)        < 0.55f) rput(y, x, ':', C_GRID);
            else if (fabsf(d - rx * 0.5f) < 0.55f) rput(y, x, '.', C_DIM);
            if (d > rx || d < 1.0f) continue;
            float diff = atan2f(dy, dx) - sa;
            while (diff >  (float)M_PI) diff -= 2.0f * (float)M_PI;
            while (diff < -(float)M_PI) diff += 2.0f * (float)M_PI;
            if      (fabsf(diff) < 0.07f)                            rput(y, x, '/', C_SCAN);
            else if (fabsf(diff) < 0.18f && s_radar.ch[y][x] == ' ') rput(y, x, ',', C_DIM);
        }
    }

    rput(RADAR_CY,     RADAR_CX,     '+', C_MID);
    rput(RADAR_CY - 1, RADAR_CX,     '|', C_MID);
    rput(RADAR_CY + 1, RADAR_CX,     '|', C_MID);
    rput(RADAR_CY,     RADAR_CX - 1, '-', C_DIM);
    rput(RADAR_CY,     RADAR_CX + 1, '-', C_DIM);
    rput(0,              RADAR_CX,       'N', C_MID);
    rput(RADAR_ROWS - 1, RADAR_CX,       'S', C_MID);
    rput(RADAR_CY,       0,              'W', C_MID);
    rput(RADAR_CY,       RADAR_COLS - 1, 'E', C_MID);

    for (int k = n - 1; k >= 0; k--) {
        const aircraft_t *a = &ac[idx[k]];
        if (!a->pos_valid || a->dist_km > range) continue;
        float b  = a->brg_deg * (float)M_PI / 180.0f;
        int   px = RADAR_CX + (int)lroundf( a->dist_km * sinf(b) / range * rx);
        int   py = RADAR_CY + (int)lroundf(-a->dist_km * cosf(b) / range * ry);
        uint8_t col = cat_colour(a->category);
        rput(py, px, '*', col);
        for (int i = 0; i < 3 && a->callsign[i]; i++)
            rput(py, px + 1 + i, a->callsign[i], col);
    }
}

/* One escape per colour run, not per cell -- the panel is mostly blank and
 * this is what keeps it cheap. */
static void emit_radar_row(int y)
{
    uint8_t cur = C_NONE;
    for (int x = 0; x < RADAR_COLS; x++) {
        char    ch = s_radar.ch[y][x];
        uint8_t c  = ch == ' ' ? cur : s_radar.col[y][x];
        if (c != cur) { fb_puts(COLOUR[c]); cur = c; }
        fb_putc(ch);
    }
    if (cur != C_NONE) fb_puts(RESET);
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

static void rule(void) { fb_puts(PH_GRID); fb_puts(s_rule); fb_puts(RESET EL "\n"); }

/* The table is sized to its contents -- never shorter than the radar, in
 * steps of five so the log does not hop on every contact -- and the log takes
 * the rest of the terminal, so a tall window buys history rather than blank
 * rows. When even the radar does not fit, the log gives way first. */
static void layout(int n, int *table_rows, int *log_rows)
{
    int avail = (s_rows > 0 ? s_rows : DEFAULT_ROWS) - CHROME_ROWS;
    int t = (n + 4) / 5 * 5;
    if (t > avail - LOG_MIN) t = avail - LOG_MIN;
    if (t < RADAR_ROWS)      t = RADAR_ROWS;
    int l = avail - t;
    if (l < 2) l = 2;
    *table_rows = t;
    *log_rows   = l;
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

    int table_rows, log_rows;
    layout(n, &table_rows, &log_rows);

    s_sweep = (s_sweep + 4) % 360;
    render_radar(ac, idx, n);

    fb_puts("\033[H");
    draw_title(&st, n, n < table_rows ? n : table_rows, now);
    rule();

    fb_printf(PH_MID "  %-6s  %-8s  %-3s  %4s  %6s  %4s  %3s  %5s  %4s  %3s  %3s  %5s  %4s " RESET,
              "ICAO", "CALLSIGN", "CAT", "SQK", "ALT ft", "SPD", "HDG", "V/S",
              "DIST", "BRG", "SIG", "MSGS", "SEEN");
    fb_printf(PH_GRID VL RESET PH_MID " RADAR  %d km" RESET EL "\n", RANGES_KM[s_range]);

    for (int r = 0; r < table_rows; r++) {
        if (r < n) draw_row(&ac[idx[r]], now);
        else       fb_rep(' ', TABLE_W);
        fb_puts(PH_GRID VL RESET " ");
        if (r < RADAR_ROWS) emit_radar_row(r);
        fb_puts(EL "\n");
    }

    rule();
    fb_puts(PH_DIM "  EVENT LOG" RESET EL "\n");

    /* Newest first, directly under the table: the log is as tall as the
     * terminal allows, and the eye should not have to cross a blank region to
     * find the latest event. */
    static const char *LOG_COLOUR[] = { PH_DIM, PH_HI, AC_AMBER, AC_CYAN, AC_RED };
    const log_entry_t *e[ADSB_LOG_LINES];
    int ne = adsb_log_recent(e, log_rows < ADSB_LOG_LINES ? log_rows : ADSB_LOG_LINES);
    for (int r = 0; r < log_rows; r++) {
        if (r < ne) fb_printf("  %s%s" RESET, LOG_COLOUR[e[r]->color < 5 ? e[r]->color : 0], e[r]->text);
        fb_puts(EL "\n");
    }

    fb_printf(PH_DIM " [Q] SHELL   [S] SORT %s   [<] [>] RANGE %d km   [M] MUTE   [+/-] VOL   [T] TEST PLANE"
              RESET EL "\033[J", SORT_NAME[s_sort], RANGES_KM[s_range]);
}

/* One keystroke, from whichever console task read it. The keys that leave
 * the display never reach here. */
static void tui_key(uint8_t key)
{
    switch (key) {
        case 's': case 'S': s_sort = (s_sort + 1) % SORT_COUNT;         break;
        case '<': case ',': case '[': if (s_range > 0) s_range--;      break;
        case '>': case '.': case ']': if (s_range < 2) s_range++;      break;
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
    screen_register(SCREEN_TUI, "the display", TUI_COLS, TUI_REFRESH_MS, tui_draw, tui_key);
}
