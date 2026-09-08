#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ═════════════════════════════════════════════════════════════════════════════
 * Console REPL (esp_console) -- the serial console's normal, top-level face.
 *
 * The serial port behaves like the SSH session does: it comes up at a `p4> `
 * prompt and the ADS-B display is one command among the others (`tui`). The
 * TUI is therefore a *foreground application* on a transport rather than the
 * thing that owns it, which is what lets the console be useful with no dongle
 * enumerated, no antenna and no network -- and what lets `tui` work over SSH
 * as well, on either transport or both at once (see the sink API below).
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

/* True when the display is in front on whichever transport currently owns
 * stdout -- the question sys_log() has to answer before echoing an event line
 * into it. Deliberately not the same as "somebody is watching the display":
 * with the radar up on the serial monitor and an SSH session sitting at a
 * prompt, the session is exactly where those lines should go. */
bool shell_tui_foreground(void);

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
 *                             While the session has the display in front the
 *                             byte drives its hotkeys instead of the line
 *                             editor, exactly as the console task does for the
 *                             serial side.
 *   <restore stdout>
 *   shell_remote_close()   -- hand the console back to the serial shell, which
 *                             reprints its prompt. Restore stdout FIRST or its
 *                             banner follows the closing session out. Also
 *                             drops the display if the session still had it
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

/* Starts the TUI's own draw task. It runs from boot and paints only while at
 * least one sink is attached, so `tui` is a state change and not a task that
 * gets created and destroyed. */
void adsb_tui_start(void);

/* ── the display as a service ────────────────────────────────────────────────
 * One draw task, one assembled frame, and any number of viewers. A transport
 * attaches when its `tui` runs and detaches when the user leaves; the display
 * itself knows nothing about UART ports, sockets, or how many of either exist,
 * so a third kind of viewer never touches class_driver.c.
 *
 * A sink is handed one *run* of the frame -- already CRLF-expanded, neither a
 * whole line nor a whole frame, since the assembler flushes a fixed buffer
 * whenever it fills. It is called from the draw task (priority 2, core0) and
 * may block briefly; the SSH one does, waiting for room in its ring.
 *
 * State is shared, not per-viewer: two people watching see the same panel, and
 * `R` from either switches it for both. That is the deliberate limit of this
 * being one frame rather than one render per client. */
typedef void (*tui_sink_fn)(void *ctx, const char *data, size_t n);

/* False if the table is full. Attach *after* adsb_tui_resume(), so the screen
 * is cleared before a frame can start. */
bool tui_attach(tui_sink_fn fn, void *ctx);

/* Stops the frame reaching this sink. Follow it with adsb_tui_hold() before
 * writing anything of your own -- a run may already be in flight. */
void tui_detach(tui_sink_fn fn, void *ctx);

/* Terminal width the display needs, borders included. `tui` compares it
 * against the size an SSH client reported for its PTY. */
int adsb_tui_cols(void);

/* Clears the screen and forces the next frame out immediately -- called when
 * the TUI comes to the foreground. */
void adsb_tui_resume(void);

/* Returns once any frame in flight has finished. Called after a transport has
 * dropped out of the sink mask, before it writes anything of its own: the draw
 * task sits at priority 2 and both readers above it, so without this the
 * prompt lands in the middle of a ~13 KB repaint. */
void adsb_tui_hold(void);

/* One keystroke for the TUI, from whichever task read it. The keys that leave
 * the TUI are handled in shell.c and never reach this. */
void adsb_tui_key(uint8_t key);

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
