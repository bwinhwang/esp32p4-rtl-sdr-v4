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

/* True while the *UART* shell owns the console. tui_draw() returns early on
 * this, and the key handler routes bytes to shell_feed() instead of the TUI
 * hotkeys. A remote session does not suspend the TUI: it writes to a socket,
 * not to UART0, so the radar keeps running on the serial monitor while
 * somebody is logged in over the network. */
bool shell_active(void);

/* Hands the console to the shell. Called from the TUI key handler. */
void shell_enter(void);

/* Queues one byte of console input. No-op when the shell is not active. */
void shell_feed(uint8_t byte);

/* ── remote sessions (net_ssh.c) ─────────────────────────────────────────────
 * The UART path above hands bytes over to the shell task because its reader
 * sits in adsb_rx_task on the demod hot path. A network transport has no such
 * problem -- its own task is already on core0 and may block -- so it drives
 * the line editor directly instead, in its own task. That keeps every libssh
 * call on one thread, which libssh sessions require anyway.
 *
 * shell_remote_open() claims the console; it fails if the UART shell or
 * another remote session already holds it (one line buffer, one command
 * context, and one global stdout -- see net_ssh.c).
 *
 * shell_remote_byte() runs one input byte to completion in the caller's task
 * and returns false once the user has left ('exit', 'quit' or Ctrl-D), after
 * which the caller must call shell_remote_close(). */
bool shell_remote_open(void);
bool shell_remote_byte(uint8_t byte);
void shell_remote_close(void);

/* ── implemented in class_driver.c ───────────────────────────────────────────
 * The commands that read the receiver's own state (the aircraft table, the
 * rtlsdr handle) need statics private to class_driver.c, so they are registered
 * there. shell_init() calls this after esp_console_init(). */
void adsb_register_shell_cmds(void);

/* Repaints the TUI after the shell releases the console. */
void adsb_tui_resume(void);
