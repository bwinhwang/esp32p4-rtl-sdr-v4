/* ═════════════════════════════════════════════════════════════════════════════
 * top.c -- system statistics and the `top` screen.
 *
 * Per-core busy percentage comes from how much of each interval that core's
 * IDLE task got. IDF's FreeRTOS has no per-task run-time getter, so the whole
 * task list has to be walked, and uxTaskGetSystemState() holds a cross-core
 * spinlock for the walk -- hence the 1 Hz cap in top_stats_sample().
 *
 * The run-time counter is esp_timer microseconds, 64-bit
 * (CONFIG_FREERTOS_RUN_TIME_COUNTER_TYPE_U64): elapsed wall time is exactly
 * one core's budget, and TIME+ is the counter itself. Deltas are still taken
 * in the counter's own type so a build that drops back to 32 bits keeps the
 * percentages right across the wrap and only loses TIME+.
 * ═══════════════════════════════════════════════════════════════════════════ */

#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/idf_additions.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_attr.h"

#include "screen.h"
#include "top.h"

#define TOP_COLS        80
#define TOP_DEFAULT_S   2
#define TOP_MIN_S       1
#define TOP_MAX_S       60
#define HEADER_LINES    7       /* title, cpu, heap, psram, display, blank, column header */

#define BOLD    "\033[1m"
#define REV     "\033[7m"
#define WARN    "\033[33m"
#define RESET   "\033[0m"

static SemaphoreHandle_t s_lock;
static top_stats_t       s_stats = { .cpu_busy = { -1, -1 } };
static int               s_rows;

static EXT_RAM_BSS_ATTR TaskStatus_t s_st[TOP_MAX_TASKS];

/* Per-handle history for the deltas, double-buffered because the new table is
 * built while the old one is still being matched against. Matching is on the
 * handle: uxTaskGetSystemState() gives no stable ordering, and tasks come and
 * go (rtlsdr_setup is transient). An unmatched handle reads 0% for one
 * interval and starts TIME+ from its counter. */
typedef struct {
    TaskHandle_t                hdl;
    configRUN_TIME_COUNTER_TYPE rt;
    uint64_t                    time_us;
} hist_t;

static EXT_RAM_BSS_ATTR hist_t s_hist[2][TOP_MAX_TASKS];
static int s_hist_n, s_hist_cur;

static char state_char(eTaskState st)
{
    switch (st) {
        case eRunning:   return 'R';
        case eReady:     return 'r';
        case eBlocked:   return 'B';
        case eSuspended: return 'S';
        case eDeleted:   return 'D';
        default:         return '?';
    }
}

void top_stats_sample(int64_t now)
{
    static TaskHandle_t                idle_hdl[2];
    static configRUN_TIME_COUNTER_TYPE last_idle[2];
    static int64_t                     last_us;

    if (last_us && (now - last_us) < 1000000LL) return;
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
    /* Re-checked under the lock: two callers can pass the gate together. */
    if (last_us && (now - last_us) < 1000000LL) goto out;

    if (!idle_hdl[0]) {
        idle_hdl[0] = xTaskGetIdleTaskHandleForCore(0);
        idle_hdl[1] = xTaskGetIdleTaskHandleForCore(1);
    }

    configRUN_TIME_COUNTER_TYPE total;
    UBaseType_t n = uxTaskGetSystemState(s_st, TOP_MAX_TASKS, &total);
    if (n == 0) goto out;    /* array too small -- raise TOP_MAX_TASKS */

    int64_t span = last_us ? now - last_us : 0;

    configRUN_TIME_COUNTER_TYPE idle[2] = { 0, 0 };
    for (UBaseType_t i = 0; i < n; i++)
        for (int c = 0; c < 2; c++)
            if (s_st[i].xHandle == idle_hdl[c]) idle[c] = s_st[i].ulRunTimeCounter;

    if (span > 0) {
        for (int c = 0; c < 2; c++) {
            int64_t busy = span - (int64_t)(configRUN_TIME_COUNTER_TYPE)(idle[c] - last_idle[c]);
            if (busy < 0)    busy = 0;
            if (busy > span) busy = span;
            s_stats.cpu_busy[c] = (int)(busy * 100 / span);
        }
    }
    last_idle[0] = idle[0];
    last_idle[1] = idle[1];
    s_stats.span_ms = (uint32_t)(span / 1000);

    const hist_t *prev   = s_hist[s_hist_cur];
    hist_t       *next   = s_hist[s_hist_cur ^ 1];

    for (UBaseType_t i = 0; i < n; i++) {
        const TaskStatus_t *st = &s_st[i];
        top_task_t         *t  = &s_stats.task[i];

        snprintf(t->name, sizeof(t->name), "%s", st->pcTaskName ? st->pcTaskName : "?");
#if ( configTASKLIST_INCLUDE_COREID == 1 )
        t->core = (st->xCoreID == tskNO_AFFINITY) ? -1 : (int8_t)st->xCoreID;
#else
        t->core = -1;
#endif
        t->prio      = (uint8_t)st->uxCurrentPriority;
        t->state     = state_char(st->eCurrentState);
        t->stack_hwm = st->usStackHighWaterMark;
        t->pct10     = 0;
        t->time_us   = (uint64_t)st->ulRunTimeCounter;

        for (int p = 0; p < s_hist_n; p++) {
            if (prev[p].hdl != st->xHandle) continue;
            configRUN_TIME_COUNTER_TYPE d = st->ulRunTimeCounter - prev[p].rt;
            if (span > 0) {
                uint32_t pct10 = (uint32_t)((uint64_t)d * 1000 / (uint64_t)span);
                t->pct10 = pct10 > 1000 ? 1000 : (uint16_t)pct10;
            }
            t->time_us = prev[p].time_us + d;
            break;
        }
        next[i] = (hist_t){ .hdl = st->xHandle, .rt = st->ulRunTimeCounter, .time_us = t->time_us };
    }
    s_hist_n   = (int)n;
    s_hist_cur ^= 1;
    s_stats.ntasks = (int)n;

    /* insertion sort, busiest first -- n is ~20 */
    for (int i = 1; i < s_stats.ntasks; i++) {
        top_task_t k = s_stats.task[i];
        int j = i - 1;
        while (j >= 0 && s_stats.task[j].pct10 < k.pct10) { s_stats.task[j + 1] = s_stats.task[j]; j--; }
        s_stats.task[j + 1] = k;
    }

    /* Explicitly MALLOC_CAP_INTERNAL, not esp_get_free_heap_size(): with
     * CONFIG_SPIRAM_USE_MALLOC that folds 32 MB of PSRAM into one figure and
     * hides the number that actually runs out. */
    s_stats.heap_free = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    s_stats.heap_min  = (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    s_stats.heap_big  = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
#if CONFIG_SPIRAM
    s_stats.psram_free = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    s_stats.psram_min  = (uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);
    s_stats.psram_big  = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
#endif

    last_us = now;
out:
    if (s_lock) xSemaphoreGive(s_lock);
}

const top_stats_t *top_stats(void) { return &s_stats; }

void top_set_rows(int rows) { s_rows = rows; }

/* ── rendering, shared by the screen and `tasks` ────────────────────────── */

enum { TONE_PLAIN, TONE_BOLD, TONE_HEAD, TONE_WARN };
typedef void (*emit_fn)(int tone, const char *line);

static void fmt_time(char *b, size_t n, uint64_t us)
{
    uint64_t cs  = us / 10000;
    uint64_t min = cs / 6000;
    if (min < 10000)
        snprintf(b, n, "%4llu:%02llu.%02llu", (unsigned long long)min,
                 (unsigned long long)((cs / 100) % 60), (unsigned long long)(cs % 100));
    else
        snprintf(b, n, "%6lluh%02llu", (unsigned long long)(min / 60),
                 (unsigned long long)(min % 60));
}

static void render(const top_stats_t *s, int64_t now, int max_rows, bool live, emit_fn emit)
{
    char line[TOP_COLS + 32];
    int64_t up = now / 1000000;

    snprintf(line, sizeof(line), "top - up %lldd %02lld:%02lld:%02lld  every %lus  %d tasks%s",
             (long long)(up / 86400), (long long)((up % 86400) / 3600),
             (long long)((up % 3600) / 60), (long long)(up % 60),
             (unsigned long)(screen_period(SCREEN_TOP) / 1000), s->ntasks,
             live ? "   [q] quit  [+/-] interval" : "");
    emit(TONE_BOLD, line);

    if (s->cpu_busy[0] < 0)
        snprintf(line, sizeof(line), "CPU0  --%%   CPU1  --%%   (first interval)");
    else
        snprintf(line, sizeof(line), "CPU0 %3d%% busy   CPU1 %3d%% busy   (over %lu.%lu s)",
                 s->cpu_busy[0], s->cpu_busy[1],
                 (unsigned long)(s->span_ms / 1000), (unsigned long)(s->span_ms % 1000 / 100));
    emit((s->cpu_busy[0] > 80 || s->cpu_busy[1] > 80) ? TONE_WARN : TONE_PLAIN, line);

    snprintf(line, sizeof(line), "heap  internal  free %7lu  min %7lu  largest %7lu",
             (unsigned long)s->heap_free, (unsigned long)s->heap_min, (unsigned long)s->heap_big);
    emit(s->heap_big < 32768 ? TONE_WARN : TONE_PLAIN, line);
#if CONFIG_SPIRAM
    snprintf(line, sizeof(line), "      psram     free %7luK min %7luK largest %7luK",
             (unsigned long)(s->psram_free / 1024), (unsigned long)(s->psram_min / 1024),
             (unsigned long)(s->psram_big / 1024));
#else
    snprintf(line, sizeof(line), "      psram     not enabled in this build");
#endif
    emit(TONE_PLAIN, line);

    screen_probe_t pr;
    screen_probe_get(&pr);
    snprintf(line, sizeof(line), "display  %lu frame/s  %lu B/frame  asm %lu ms/s  out %lu ms/s",
             (unsigned long)pr.fps, (unsigned long)pr.bytes,
             (unsigned long)pr.asm_ms, (unsigned long)pr.out_ms);
    emit(TONE_PLAIN, line);

    emit(TONE_PLAIN, "");
    snprintf(line, sizeof(line), "  %-16s %4s %4s %2s %6s %8s %10s",
             "TASK", "CORE", "PRIO", "ST", "CPU%", "STACK", "TIME+");
    emit(TONE_HEAD, line);

    for (int i = 0; i < s->ntasks && i < max_rows; i++) {
        const top_task_t *t = &s->task[i];
        char core[8], tm[16];
        if (t->core < 0) snprintf(core, sizeof(core), "-");
        else             snprintf(core, sizeof(core), "%d", t->core);
        fmt_time(tm, sizeof(tm), t->time_us);
        snprintf(line, sizeof(line), "  %-16s %4s %4u  %c %4u.%u %8lu %10s",
                 t->name, core, (unsigned)t->prio, t->state,
                 (unsigned)(t->pct10 / 10), (unsigned)(t->pct10 % 10),
                 (unsigned long)t->stack_hwm, tm);
        emit((t->pct10 >= 800 || t->stack_hwm < 512) ? TONE_WARN : TONE_PLAIN, line);
    }
}

/* ── the screen ─────────────────────────────────────────────────────────── */

static bool s_first_line;

/* Home + overwrite + erase-to-EOL per line, erase-below at the end: no
 * clear-screen per frame, so no flicker. No newline after the last line --
 * on a terminal exactly as tall as the frame that would scroll it. */
static void emit_live(int tone, const char *line)
{
    static const char *pre[] = { "", BOLD, REV, WARN };
    if (!s_first_line) fb_puts("\n");
    s_first_line = false;
    fb_puts(pre[tone]);
    fb_puts(line);
    if (tone == TONE_HEAD) fb_rep(' ', TOP_COLS - 1 - (int)strlen(line));
    if (tone != TONE_PLAIN) fb_puts(RESET);
    fb_puts("\033[K");
}

static void top_draw(int64_t now)
{
    top_stats_sample(now);
    int rows = s_rows > 0 ? s_rows - HEADER_LINES : TOP_MAX_TASKS;
    if (rows < 1) rows = 1;

    s_first_line = true;
    fb_puts("\033[H");
    render(&s_stats, now, rows, true, emit_live);
    fb_puts("\033[J");
}

static void top_key(uint8_t key)
{
    uint32_t s = screen_period(SCREEN_TOP) / 1000;
    if      (key == '+' || key == '=') s++;
    else if (key == '-' || key == '_') s--;
    else return;
    if (s < TOP_MIN_S) s = TOP_MIN_S;
    if (s > TOP_MAX_S) s = TOP_MAX_S;
    screen_set_period(SCREEN_TOP, s * 1000);
    screen_wake(SCREEN_TOP);
}

/* ── `tasks` ────────────────────────────────────────────────────────────── */

static void emit_batch(int tone, const char *line)
{
    (void)tone;
    printf("%s\n", line);
}

int top_batch(void)
{
    top_stats_sample(esp_timer_get_time());
    vTaskDelay(pdMS_TO_TICKS(1000));
    int64_t now = esp_timer_get_time();
    top_stats_sample(now);

    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
    render(&s_stats, now, TOP_MAX_TASKS, false, emit_batch);
    if (s_lock) xSemaphoreGive(s_lock);
    return 0;
}

void top_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    screen_register(SCREEN_TOP, "top", TOP_COLS, TOP_DEFAULT_S * 1000, top_draw, top_key);
}
