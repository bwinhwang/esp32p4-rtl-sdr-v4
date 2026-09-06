#pragma once

#include <stdbool.h>
#include <stdint.h>

/* ═════════════════════════════════════════════════════════════════════════════
 * Console REPL (esp_console) sharing the one console UART with the TUI.
 *
 * The TUI owns UART0 -- adsb_rx_task installs the driver and is its only
 * reader -- so esp_console_new_repl_uart() cannot be used: it would install
 * the driver a second time and start its own blocking reader competing for the
 * same bytes. Instead only the *Eval* half of esp_console is used
 * (esp_console_run(), which is transport-agnostic) and the *Read* half is
 * driven by the key handler that already exists.
 *
 * Bytes are handed over rather than executed in place because that key handler
 * lives in adsb_rx_task on core1 -- the demod hot path. A command may block for
 * seconds (a WiFi write, a USB reset), which would stall the loop and drop IQ
 * samples, so shell_feed() only queues and a task on core0 does the work.
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Registers the commands and starts the shell task. Call once from app_main;
 * safe to call before the dongle enumerates or the network is up (commands
 * report those as unavailable rather than faulting). */
void shell_init(void);

/* True while the shell owns the console. tui_draw() returns early on this, and
 * the key handler routes bytes to shell_feed() instead of the TUI hotkeys. */
bool shell_active(void);

/* Hands the console to the shell. Called from the TUI key handler. */
void shell_enter(void);

/* Queues one byte of console input. No-op when the shell is not active. */
void shell_feed(uint8_t byte);

/* ── implemented in class_driver.c ───────────────────────────────────────────
 * The commands that read the receiver's own state (the aircraft table, the
 * rtlsdr handle) need statics private to class_driver.c, so they are registered
 * there. shell_init() calls this after esp_console_init(). */
void adsb_register_shell_cmds(void);

/* Repaints the TUI after the shell releases the console. */
void adsb_tui_resume(void);
