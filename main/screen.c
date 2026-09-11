/* ═════════════════════════════════════════════════════════════════════════════
 * screen.c -- the display service: frame buffer, sinks, one draw task.
 *
 * Moved out of class_driver.c when `top` became the second full-screen
 * display; the contract is in screen.h.
 * ═══════════════════════════════════════════════════════════════════════════ */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_attr.h"

#include "screen.h"

#define CLS             "\033[2J\033[H"
#define SCREEN_MAX_SINKS 2      /* one per transport */
#define POLL_MS          100    /* how often the draw task checks what is due */

typedef struct {
    const char     *name;
    int             cols;
    uint32_t        period_ms;
    screen_draw_fn  draw;
    screen_key_fn   key;
    int64_t         last_draw;
} screen_t;

static screen_t s_screens[SCREEN_COUNT];

/* ── frame buffer ─────────────────────────────────────────────────────────
 *
 * A radar frame is ~20 KB, and both costs sitting on top of the bytes are
 * per-call, not per-byte: stdio's stream lock (taken ~4000x per frame,
 * bottoming out in a cross-core spinlock core1's demod loop contends for), and
 * uart_vfs's write(), which loops PER CHARACTER to do the \n -> \r\n
 * translation and calls uart_write_bytes(&c, 1) for each -- a mutex take/give
 * and a ringbuf send per byte, ~20000 per frame.
 *
 * So the frame is assembled here with memcpy and handed to the sinks in whole
 * runs, CRLF expanded by hand: stdio and the VFS are both out of the path.
 * This is why neither -O2 nor setvbuf ever moved the number -- the cost was
 * inside prebuilt libc and IDF, and it was never the write count. printf()
 * elsewhere (ESP_LOG, boot) still goes the normal way.
 * ───────────────────────────────────────────────────────────────────────── */

static EXT_RAM_BSS_ATTR char s_fb[4096];
static int  s_fb_len;

/* Frame-cost probe. Reasoning about where the frame time goes has been wrong
 * three times running, so these numbers get measured and shown instead:
 * per-second totals of frames, bytes, time assembling and time writing. */
static uint32_t       s_pf_bytes, s_pf_out_us;   /* this frame */
static screen_probe_t s_probe;                   /* last second */

/* Held for the duration of one frame. shell.c takes it when a display leaves
 * the foreground, so a repaint already in flight finishes before the prompt is
 * drawn over it -- otherwise the frame and the banner interleave on UART0 (the
 * draw task is priority 2, the console task 3, so the console does preempt it
 * mid-frame). */
static SemaphoreHandle_t s_paint_lock;

/* ── sinks ────────────────────────────────────────────────────────────────
 * No lock, by the same discipline the rest of the display uses: `fn` is the
 * slot's published flag, so attach writes it last and detach clears it first,
 * and detach is followed by screen_hold() to wait out a run already in
 * flight. Reading it per run rather than once per frame is what bounds that
 * wait -- for the SSH sink the task calling detach is also the one draining
 * the ring the sink would otherwise sit waiting for room in, so it has to stop
 * being called at the next run and not at the end of the repaint. Whatever it
 * was already sent is wiped by the clear-screen the leaver prints anyway.
 *
 * Not safe against two *concurrent* attaches, and does not need to be: the
 * only callers are the `tui`/`top` commands, and shell.c's exclusive console
 * ownership means at most one of them is ever running. A viewer attached from
 * some other task would have to bring its own serialisation. */
static struct {
    screen_sink_fn volatile fn;     /* NULL = free slot */
    void                   *ctx;
    screen_id_t             screen;
} s_sinks[SCREEN_MAX_SINKS];

static screen_id_t s_painting;      /* which screen fb_out() routes for */

int screen_viewers(screen_id_t id)
{
    int n = 0;
    for (int i = 0; i < SCREEN_MAX_SINKS; i++)
        if (s_sinks[i].fn && s_sinks[i].screen == id) n++;
    return n;
}

bool screen_attach(screen_id_t id, screen_sink_fn fn, void *ctx)
{
    for (int i = 0; i < SCREEN_MAX_SINKS; i++) {
        if (s_sinks[i].fn) continue;
        printf(CLS);
        fflush(stdout);
        s_sinks[i].ctx    = ctx;
        s_sinks[i].screen = id;
        s_sinks[i].fn     = fn;     /* last: this is what publishes the slot */
        screen_wake(id);
        return true;
    }
    return false;
}

void screen_detach(screen_sink_fn fn, void *ctx)
{
    for (int i = 0; i < SCREEN_MAX_SINKS; i++)
        if (s_sinks[i].fn == fn && s_sinks[i].ctx == ctx) s_sinks[i].fn = NULL;
}

static void fb_out(const char *p, size_t n)
{
    for (int i = 0; i < SCREEN_MAX_SINKS; i++) {
        screen_sink_fn fn = s_sinks[i].fn;
        if (fn && s_sinks[i].screen == s_painting) fn(s_sinks[i].ctx, p, n);
    }
}

static void fb_flush(void)
{
    if (s_fb_len <= 0) return;

    int64_t     t0  = esp_timer_get_time();
    const char *p   = s_fb;
    int         rem = s_fb_len;
    s_pf_bytes += (uint32_t)s_fb_len;
    s_fb_len = 0;

    while (rem > 0) {
        const char *nl  = memchr(p, '\n', (size_t)rem);
        int         run = nl ? (int)(nl - p) : rem;
        if (run > 0) fb_out(p, (size_t)run);
        if (!nl) break;
        fb_out("\r\n", 2);
        p    = nl + 1;
        rem -= run + 1;
    }
    s_pf_out_us += (uint32_t)(esp_timer_get_time() - t0);
}

static void probe_frame(int64_t now, uint32_t frame_us)
{
    static int64_t  win;
    static uint32_t n, bytes, out_us, tot_us;

    n++;
    bytes  += s_pf_bytes;
    out_us += s_pf_out_us;
    tot_us += frame_us;

    if (!win) { win = now; return; }
    if (now - win < 1000000LL) return;

    s_probe.fps    = n;
    s_probe.bytes  = bytes / n;
    s_probe.out_ms = out_us / 1000;
    s_probe.asm_ms = (tot_us - out_us) / 1000;
    win = now;
    n = bytes = out_us = tot_us = 0;
}

void screen_probe_get(screen_probe_t *p) { *p = s_probe; }

/* Flushing mid-frame when full keeps the buffer small without ever
 * truncating a row -- the flush count was never what cost anything. */
static void fb_room(int need)
{
    if (s_fb_len + need > (int)sizeof(s_fb)) fb_flush();
}

void fb_puts(const char *s)
{
    int n = (int)strlen(s);
    fb_room(n);
    if (n > (int)sizeof(s_fb)) n = (int)sizeof(s_fb);
    memcpy(s_fb + s_fb_len, s, (size_t)n);
    s_fb_len += n;
}

void fb_putc(char c)
{
    fb_room(1);
    s_fb[s_fb_len++] = c;
}

void fb_rep(char c, int n)
{
    while (n > 0) {
        fb_room(1);
        int chunk = (int)sizeof(s_fb) - s_fb_len;
        if (chunk > n) chunk = n;
        memset(s_fb + s_fb_len, c, (size_t)chunk);
        s_fb_len += chunk;
        n        -= chunk;
    }
}

void fb_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    fb_room(512);       /* a fully coloured 120-column line is ~300 bytes */
    int room = (int)sizeof(s_fb) - s_fb_len;
    int n = vsnprintf(s_fb + s_fb_len, (size_t)room, fmt, ap);
    va_end(ap);
    if (n > 0) s_fb_len += (n < room ? n : room - 1);
}

/* ── screens ──────────────────────────────────────────────────────────── */

static bool valid(screen_id_t id) { return id > SCREEN_NONE && id < SCREEN_COUNT; }

void screen_register(screen_id_t id, const char *name, int cols,
                     uint32_t period_ms, screen_draw_fn draw, screen_key_fn key)
{
    if (!valid(id)) return;
    s_screens[id] = (screen_t){ .name = name, .cols = cols, .period_ms = period_ms,
                                .draw = draw, .key = key };
}

const char *screen_name(screen_id_t id)   { return valid(id) ? s_screens[id].name : "?"; }
int         screen_cols(screen_id_t id)   { return valid(id) ? s_screens[id].cols : 0; }
uint32_t    screen_period(screen_id_t id) { return valid(id) ? s_screens[id].period_ms : 0; }

void screen_set_period(screen_id_t id, uint32_t period_ms)
{
    if (valid(id)) s_screens[id].period_ms = period_ms;
}

void screen_wake(screen_id_t id)
{
    if (valid(id)) s_screens[id].last_draw = 0;
}

void screen_key(screen_id_t id, uint8_t key)
{
    if (valid(id) && s_screens[id].key) s_screens[id].key(key);
}

void screen_hold(void)
{
    if (!s_paint_lock) return;
    xSemaphoreTake(s_paint_lock, portMAX_DELAY);
    xSemaphoreGive(s_paint_lock);
}

static void paint(screen_id_t id, int64_t now)
{
    xSemaphoreTake(s_paint_lock, portMAX_DELAY);
    /* The viewer can have left while this waited -- screen_hold() is the other
     * side of this lock and gives it back straight away. */
    if (!screen_viewers(id)) { xSemaphoreGive(s_paint_lock); return; }

    /* The frame goes straight to the transport, so anything another task left
     * in stdout's buffer has to get out first or it lands mid-frame. */
    fflush(stdout);

    s_pf_bytes = s_pf_out_us = 0;
    s_painting = id;
    s_screens[id].draw(now);
    fb_flush();
    s_painting = SCREEN_NONE;
    probe_frame(now, (uint32_t)(esp_timer_get_time() - now));

    xSemaphoreGive(s_paint_lock);
}

/* A frame is written to the console UART in full before this returns -- a
 * full radar frame is ~20 KB of UTF-8 box drawing and ANSI colour. Drawing it
 * from adsb_rx_task meant the IQ ring overflowed for the whole duration of
 * every frame, which at the old hardcoded 115200 baud was over a second. The
 * screens read the aircraft table without a lock: a torn frame is cosmetic, a
 * starved demod loop is not. */
static void draw_task(void *arg)
{
    for (;;) {
        int64_t now = esp_timer_get_time();
        for (screen_id_t id = SCREEN_NONE + 1; id < SCREEN_COUNT; id++) {
            screen_t *s = &s_screens[id];
            if (!s->draw || !screen_viewers(id)) continue;
            if (now - s->last_draw < (int64_t)s->period_ms * 1000) continue;
            s->last_draw = now;
            paint(id, now);
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

void screen_start(void)
{
    s_paint_lock = xSemaphoreCreateMutex();
    /* Created once and never destroyed: a screen with no viewer costs one
     * table walk per POLL_MS, so entering one is a state change and not a
     * task lifecycle. */
    xTaskCreatePinnedToCore(draw_task, "screen", 6144, NULL, 2, NULL, 0);
}
