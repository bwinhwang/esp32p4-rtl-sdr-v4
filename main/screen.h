#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ═════════════════════════════════════════════════════════════════════════════
 * Full-screen displays as a service -- one draw task, one frame buffer, any
 * number of viewers.
 *
 * A *screen* is something that repaints a terminal on a period: the radar
 * display (`tui`) and the system monitor (`top`). A *sink* is a viewer: a
 * transport attaches one when its command runs and detaches it when the user
 * leaves. The screen knows nothing about UART ports or sockets, and shell.c,
 * which owns the transports, knows nothing about what a screen paints.
 *
 * Each sink names the screen it is watching, so two viewers can be on
 * different screens at once (radar on the serial console, `top` over SSH); a
 * screen is only painted while at least one sink is on it, so the core0 cost
 * of console bytes is paid only for what somebody is looking at. Within one
 * screen the state is shared, not per-viewer: two people on the radar see the
 * same panel and `R` from either switches it for both.
 *
 * A sink is handed one *run* of a frame -- already CRLF-expanded, neither a
 * whole line nor a whole frame, since the assembler flushes a fixed buffer
 * whenever it fills. It is called from the draw task (priority 2, core0) and
 * may block briefly; the SSH one does, waiting for room in its ring.
 * ═══════════════════════════════════════════════════════════════════════════ */

typedef enum {
    SCREEN_NONE = 0,        /* "at a prompt" in shell.c's per-transport state */
    SCREEN_TUI,
    SCREEN_TOP,
    SCREEN_COUNT
} screen_id_t;

/* Paints one frame with the fb_* calls below; the service flushes it. */
typedef void (*screen_draw_fn)(int64_t now);
/* One keystroke from whichever task read it. The keys that leave a screen are
 * handled in shell.c and never reach this. */
typedef void (*screen_key_fn)(uint8_t key);
typedef void (*screen_sink_fn)(void *ctx, const char *data, size_t n);

/* Creates the paint lock and the draw task. Call once, early in app_main; the
 * task idles until a sink attaches. */
void screen_start(void);

/* `cols` is the terminal width the screen needs, borders included -- the
 * entering command compares it against an SSH client's PTY width. */
void screen_register(screen_id_t id, const char *name, int cols,
                     uint32_t period_ms, screen_draw_fn draw, screen_key_fn key);

const char *screen_name(screen_id_t id);
int         screen_cols(screen_id_t id);
uint32_t    screen_period(screen_id_t id);
void        screen_set_period(screen_id_t id, uint32_t period_ms);

/* Repaint on the draw task's next pass instead of a period from now. */
void screen_wake(screen_id_t id);

/* Clears the caller's terminal (through stdout, so on whichever transport is
 * running the command), then publishes the sink. False if the table is full.
 * The clear is done here, before publishing, because the draw task starts the
 * moment a slot appears and a frame begun first would be half-wiped. */
bool screen_attach(screen_id_t id, screen_sink_fn fn, void *ctx);

/* Stops frames reaching this sink. Follow it with screen_hold() before writing
 * anything of your own -- a run may already be in flight. */
void screen_detach(screen_sink_fn fn, void *ctx);

/* Returns once any frame in flight has finished. The draw task sits at
 * priority 2 and both console readers above it, so without this a prompt
 * lands in the middle of a repaint. */
void screen_hold(void);

void screen_key(screen_id_t id, uint8_t key);
int  screen_viewers(screen_id_t id);

/* ── frame assembly, for draw callbacks only ───────────────────────────────── */
void fb_puts(const char *s);
void fb_putc(char c);
void fb_rep(char c, int n);
void fb_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* Frame-cost probe: per-second totals across every screen, refreshed once a
 * second while anything is painting. `bytes` is per frame. */
typedef struct {
    uint32_t fps, bytes, asm_ms, out_ms;
} screen_probe_t;

void screen_probe_get(screen_probe_t *p);
