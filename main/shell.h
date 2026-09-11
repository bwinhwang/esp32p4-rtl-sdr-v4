#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ═════════════════════════════════════════════════════════════════════════════
 * Console REPL (esp_console) -- the serial console's normal, top-level face.
 *
 * The serial port behaves like the SSH session does: it comes up at a `p4> `
 * prompt and the full-screen displays are commands among the others (`tui`,
 * `top`). A screen is therefore a *foreground application* on a transport
 * rather than the thing that owns it, which is what lets the console be useful
 * with no dongle enumerated, no antenna and no network -- and what lets either
 * screen work over SSH as well, on either transport or both at once (see
 * screen.h).
 *
 * esp_console_new_repl_uart() is still not used, and for the original reason:
 * it installs its own UART driver and its own blocking reader, while the TUI's
 * fb_flush() writes UART0 directly and needs the driver installed with a TX
 * ring of its own. Only the *Eval* half of esp_console is used
 * (esp_console_run(), which takes a line from anywhere); the *Read* half is
 * the small raw-byte line editor in shell.c, shared by both transports.
 *
 * Two owners can want the one console: the serial port and one SSH session.
 * There is one line buffer, one command context and -- under picolibc -- one
 * global stdout, so ownership is exclusive. SSH *preempts* the serial shell
 * rather than being refused by it (otherwise the serial side, which is now
 * always at a prompt, would lock SSH out forever); the serial shell comes back
 * by itself when the session ends. See shell_remote_claim().
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Installs the UART0 driver, registers the commands and starts the console
 * task. Call once, early in app_main: everything printed afterwards goes out
 * through the driver's interrupt-driven TX rather than uart_vfs's busy-spin.
 * The task stays quiet until shell_console_start(). */
void shell_init(void);

/* Prints the banner and the first prompt. Called from app_main once the noisy
 * part of boot is over, so the prompt is not immediately scrolled away. */
void shell_console_start(void);

/* True when a screen (`tui` or `top`) is in front on whichever transport
 * currently owns stdout -- the question sys_log() has to answer before echoing
 * an event line into it. Deliberately not the same as "somebody is watching a
 * screen": with the radar up on the serial monitor and an SSH session sitting
 * at a prompt, the session is exactly where those lines should go. */
bool shell_screen_foreground(void);

/* ── remote sessions (net_ssh.c) ─────────────────────────────────────────────
 * The transport drives the line editor directly, in its own task, rather than
 * queueing to the console task: its task is already on core0 and may block,
 * and it keeps every libssh call on one thread, which libssh requires anyway.
 *
 * The call order matters and is not interchangeable:
 *
 *   shell_remote_claim()   -- take the console away from the serial shell,
 *                             waiting for it to finish whatever it is in the
 *                             middle of. Nothing is printed. MUST come before
 *                             the stdout swap: the serial shell echoes with
 *                             printf(), and until it has let go those bytes
 *                             would land in the SSH session.
 *   <swap stdout>
 *   shell_remote_open()    -- print the banner and the first prompt, which is
 *                             why the swap has to be in place first.
 *   shell_remote_byte()    -- run one input byte to completion; returns false
 *                             once the user has left ('exit', 'quit', Ctrl-D).
 *                             While the session has a screen in front the
 *                             byte drives its hotkeys instead of the line
 *                             editor, exactly as the console task does for the
 *                             serial side.
 *   <restore stdout>
 *   shell_remote_close()   -- hand the console back to the serial shell, which
 *                             reprints its prompt. Restore stdout FIRST or its
 *                             banner follows the closing session out. Also
 *                             drops the screen if the session still had one
 *                             up, so no frame is written into a dead ring.
 *
 * shell_remote_claim() returns false only if another SSH session already holds
 * the console, or if the serial shell did not go quiet in time. */
bool shell_remote_claim(void);
void shell_remote_open(void);
bool shell_remote_byte(uint8_t byte);
void shell_remote_close(void);

/* ── implemented in class_driver.c ───────────────────────────────────────────
 * The commands that read the receiver's own state (the aircraft table, the
 * rtlsdr handle) need statics private to class_driver.c, so they are
 * registered there. shell_init() calls this after esp_console_init(). */
void adsb_register_shell_cmds(void);

/* Registers the radar display with screen.c. Call after screen_start(). */
void adsb_tui_start(void);

/* ── the event log (class_driver.c) ──────────────────────────────────────────
 * One ring, three kinds of line, and separating them is the whole point:
 *
 *   sys_log -- the board: USB, the IQ stream, WiFi, Ethernet, OTA, the web
 *              server, the console itself. A handful a minute at worst, and
 *              what somebody sitting at a prompt is actually waiting for.
 *   air_log -- the sky: contacts, idents, positions, altitudes, velocities.
 *              The display's subject matter, and several a second with an
 *              antenna connected -- which is exactly what made the prompt
 *              unusable before this split existed.
 *   ui_log  -- the display answering a keystroke. Only the person who pressed
 *              the key cares, and with two transports that person is not
 *              necessarily the one holding the prompt it would print into.
 *
 * All three go to the LOG panel; `log echo` decides which also reach whichever
 * console is at a prompt, and the default is ADSB_ECHO_SYS -- the board talks
 * to you, the sky stays in the display. `log tail` reads the ring regardless.
 *
 * These are the only reason net_wifi.c, net_eth.c, ota.c and web_config.c
 * include this header -- a far lighter dependency than esp_libusb.h, where the
 * declaration used to live and which drags usb_host.h in behind it. */
void sys_log(uint8_t color, const char *fmt, ...);
void air_log(uint8_t color, const char *fmt, ...);
void ui_log(uint8_t color, const char *fmt, ...);

#define ADSB_ECHO_OFF    0   /* nothing                                      */
#define ADSB_ECHO_SYS    1   /* board events only -- the default at a prompt */
#define ADSB_ECHO_BRIEF  2   /* ...plus first and lost contacts              */
#define ADSB_ECHO_ALL    3   /* everything the LOG panel shows               */

void adsb_log_dump(int n);
void adsb_log_echo_set(int level);
int  adsb_log_echo_get(void);

/* Prints a line that arrived on its own schedule -- an echoed event, an
 * esp_log line from any task -- without wrecking the line the user is halfway
 * through typing. The prompt and the input buffer are wiped, the line goes
 * out, and both are drawn again; with no prompt up (boot, a running command)
 * it is a plain print. Whichever transport owns stdout is the one that gets
 * it, so this covers the SSH session as well.
 *
 * Not for command output: a command already runs with the console to itself. */
void shell_async_print(const char *line);
