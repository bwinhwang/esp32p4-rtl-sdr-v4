#pragma once

#include <stdint.h>

/* ═════════════════════════════════════════════════════════════════════════════
 * System statistics -- per-core load, heap, per-task CPU/stack -- and the
 * `top` screen that shows them. `tasks` is the same table printed once.
 *
 * The sampler is shared: the draw task samples for whichever screen is up and
 * `tasks` samples from the console task, so the sample itself is under a
 * mutex. The *result* is read without one by the draw task -- a torn frame is
 * cosmetic -- and only the draw task may hold the pointer top_stats() hands
 * out. Anything on another task goes through top_batch().
 * ═══════════════════════════════════════════════════════════════════════════ */

#define TOP_MAX_TASKS  32

typedef struct {
    char     name[16];      /* configMAX_TASK_NAME_LEN */
    int8_t   core;          /* -1 = no affinity */
    uint8_t  prio;
    char     state;         /* R running, r ready, B blocked, S suspended, D deleted */
    uint16_t pct10;         /* tenths of a percent of ONE core, not of both */
    uint32_t stack_hwm;     /* unused-stack margin, bytes */
    uint64_t time_us;       /* total run time */
} top_task_t;

typedef struct {
    int        cpu_busy[2];             /* -1 until the first interval */
    uint32_t   span_ms;                 /* the interval the percentages cover */
    uint32_t   heap_free, heap_min, heap_big;
    uint32_t   psram_free, psram_min, psram_big;
    int        ntasks;
    top_task_t task[TOP_MAX_TASKS];     /* busiest first */
} top_stats_t;

/* Registers the screen and creates the sampler's mutex. Call after
 * screen_start() and before the console can run a command. */
void top_init(void);

/* Takes a fresh sample if the last one is at least a second old, otherwise
 * returns at once: uxTaskGetSystemState() holds a cross-core spinlock for the
 * whole task-list walk, so the rate is capped regardless of who asks. */
void top_stats_sample(int64_t now);

/* The latest sample. Draw task only -- see above. */
const top_stats_t *top_stats(void);

/* Rows the viewer's terminal has, so the task table is cut to fit; 0 = unknown,
 * which prints every task and leaves fitting to the terminal (the serial
 * console never reports its size, and like the radar it is assumed to be a
 * big window). Shared across viewers like every other screen state. */
void top_set_rows(int rows);

/* The `tasks` command: sample over one second and print the table as plain
 * text, on whichever task is running the command. */
int top_batch(void);
