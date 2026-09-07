#pragma once

#include <stdbool.h>
#include <stdint.h>

/* ═════════════════════════════════════════════════════════════════════════════
 * Console REPL (esp_console) -- the serial console's normal, top-level face.
 *
 * The serial port behaves like the SSH session does: it comes up at a `p4> `
 * prompt and the ADS-B display is one command among the others (`tui`). The
 * TUI is therefore a *foreground application* on UART0 rather than the thing
 * that owns it, which is what lets the console be useful with no dongle
 * enumerated, no antenna and no network.
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

/* True while the TUI is the serial port's foreground application. tui_draw()
 * and tui_log() test this: the TUI paints only when it is in front, and log
 * lines echo to the console only when it is not (they are in its LOG panel
 * instead). An SSH session does not change this -- it writes to a socket, so
 * the radar keeps painting on the serial monitor while somebody is logged in. */
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
 *   <restore stdout>
 *   shell_remote_close()   -- hand the console back to the serial shell, which
 *                             reprints its prompt. Restore stdout FIRST or its
 *                             banner follows the closing session out.
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

/* Starts the TUI's own draw task. It runs from boot and paints only while
 * shell_tui_foreground() is true, so `tui` is a state change and not a task
 * that gets created and destroyed. */
void adsb_tui_start(void);

/* Clears the screen and forces the next frame out immediately -- called when
 * the TUI comes to the foreground. */
void adsb_tui_resume(void);

/* Returns once any frame in flight has finished. Called after the TUI leaves
 * the foreground, before anything else writes UART0: the draw task sits at
 * priority 2 and the console task at 3, so without this the prompt lands in
 * the middle of a ~13 KB repaint. */
void adsb_tui_hold(void);

/* One keystroke for the TUI, from the console task. The keys that leave the
 * TUI are handled in shell.c and never reach this. */
void adsb_tui_key(uint8_t key);

/* The TUI's log ring, reachable from the shell: `log tail` prints the last n
 * lines, `log echo` decides which new ones also go to the console while the
 * TUI is in the background.
 *
 * ADSB_ECHO_ALL is unusable with an antenna connected -- every altitude and
 * velocity update logs a line -- so the console default is ADSB_ECHO_BRIEF:
 * first contacts, receiver bring-up, and anything that went wrong. */
#define ADSB_ECHO_OFF    0
#define ADSB_ECHO_BRIEF  1
#define ADSB_ECHO_ALL    2

void adsb_log_dump(int n);
void adsb_log_echo_set(int level);
int  adsb_log_echo_get(void);
