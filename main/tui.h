#pragma once

/* ═════════════════════════════════════════════════════════════════════════════
 * The aircraft display: the `tui` command's screen. Table sorted by distance
 * (or altitude, message count, freshness), a radar with a switchable range,
 * the sky's event log. Everything about the board itself is in `top`.
 *
 * 120 columns; the height follows the terminal (see tui_set_rows). Reads the
 * tracker through adsb.h from the draw task only, and never writes it -- the
 * `t` key goes through adsb_inject_test().
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Registers the screen. Call after screen_start(). */
void tui_init(void);

/* Rows the viewer's terminal has; the table takes what is left after the
 * fixed lines. 0 = unknown, which assumes 40. Shared across viewers like
 * every other screen state. */
void tui_set_rows(int rows);
