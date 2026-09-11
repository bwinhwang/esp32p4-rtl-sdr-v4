# Console REPL and SSH console

`main/shell.c` owns the command line, `main/net_ssh.c` puts the same REPL on a socket,
`main/class_driver.c` owns the TUI frame and the event ring. This is the design and the list of
things that break if changed.

## Model

**The serial port is a command line, and the TUI is one command in it.** The board boots to a
`p4> ` prompt — same console, same commands, same line editor the SSH server serves — and `tui`
puts the radar display in the foreground; `q`, `:`, Ctrl-C or Ctrl-D returns to the prompt.
`shell_init()` installs the UART driver at the top of `app_main`, before anything logs, and the
console task is UART0's only reader for the life of the board — so the shell exists with no
dongle, which is exactly when `usb` and `wifi sta` are wanted. The display is opt-in, so
`TUI_REFRESH_MS`'s core0 cost is not paid unless someone is looking.

**Ownership and display are two independent axes.** Ownership is an enum
(`OWNER_NONE`/`OWNER_UART`/`OWNER_SSH`): who may use the *one* line buffer, command context and
global stdout. "The TUI is in front here" is a separate flag per transport. So the serial shell and
an SSH session can never both be at a prompt, while `tui` on either or both at once is fine.

**esp_console is used for Eval only.** `esp_console_new_repl_uart()` is not used: the stock REPL
installs its own UART driver and blocking reader, and this project needs the driver installed with
a TX ring that `fb_flush()` writes into directly. `esp_console_run()` takes a line from anywhere;
the Read half is the small raw-byte line editor in `shell.c`, shared by both transports — which is
what makes the socket transport a matter of where bytes come from and where `printf` goes. No
linenoise (it wants to own stdin and block in it): editing is backspace, Ctrl-C, Ctrl-U; no
history, no completion.

**Commands split by what they can reach**: generic ones (`free`, `tasks`, `net`, `wifi`, `log`,
`sys`, `tui`, `ota`, `ssh`, `restart`, `exit`) in `shell.c`; `ac`, `usb`, `sdr` registered from
`class_driver.c` because they read its statics (`s_aircraft`, `rtldev`). `tasks` samples
`uxTaskGetSystemState()` twice a second apart rather than reusing the TUI's cached `s_tstat`, which
is frozen whenever the display is in the background. esp_hosted registers `mem-dump`, `task-dump`,
`cpu-dump`, `heap-trace`, `sock-dump`, `host-power-save` into the same console.

## The TUI as a service

`tui_draw()` assembles **one** frame; `fb_flush()` hands each run to every attached *sink*
(`tui_attach(fn, ctx)` / `tui_detach(fn, ctx)` in `shell.h`, implemented in `class_driver.c`).
`class_driver.c` includes neither `driver/uart.h` nor `net_ssh.h`; `shell.c` owns the two adapters
(`uart_sink` → `uart_write_bytes`, `ssh_sink` → `net_ssh_tui_write`, both below stdio for the
per-call cost). A third viewer is an adapter plus a `TUI_MAX_SINKS` bump.

**State is shared, not per-viewer** — one panel for everyone, `R` from either viewer switches
both, fixed `TERM_W` (154) rather than each client's width. Per-client state would multiply
core0's per-byte cost by the viewer count. `tui` compares `adsb_tui_cols()` against the client's
PTY width and refuses a narrow window (a wrapped frame looks like a broken build).

Rules of the attach path:

- **No lock on the sink table.** `fn` is the slot's published flag: attach writes it last, detach
  clears it first and then calls `adsb_tui_hold()` to wait out a run in flight. Not safe against
  two *concurrent* attaches — doesn't need to be, console ownership is exclusive and the only
  callers are the two `tui` commands. Any other task attaching must bring its own serialisation.
- **Clear the screen before attaching, never after.** The draw task starts the moment a slot is
  published.
- **Over SSH the command keeps the console; over serial it gives it up.** Serial `cmd_tui()`
  releases ownership so `run_line()` does not print a prompt over the first frame. Doing that in a
  session would let the serial shell `settle()` in behind and send its banner down the wire
  (stdout still points there), so the SSH path stays `OWNER_SSH` and `shell_remote_byte()` routes
  keys to the hotkeys. Leaving prints the clear-screen and the prompt itself.
- **`net_ssh_tui_write()` waits for ring room (bounded, 200 ms) instead of dropping** — half an
  escape leaves the screen wrong, and a frame (~20 KB) exceeds the ring (16 KB). Bounded because
  the task draining that ring is the one that processes the key leaving the display. `fb_out()`
  re-reads the slot per run so a detached sink stops being called immediately.
- **`shell_tui_foreground()` means "on whichever transport owns stdout"** — the only question the
  event echo needs answered.
- **`q` on serial is refused while an SSH session holds the console** (no prompt to return to,
  and keys from an owner-less transport are dropped, so the port would be stranded blank).
  `leave_tui()` stays put and says so through the LOG panel.

**`s_paint_lock` interlocks the frame and the prompt.** The draw task is prio 2, the console task
3; leaving the TUI would otherwise print the banner into a ~13 KB repaint. `adsb_tui_hold()` from
`leave_tui()` waits the frame out. Anything else that writes UART0 in response to a keystroke
needs the same.

**The `t` hotkey defers to the demod loop.** `inject_fake_aircraft()` writes `s_aircraft[]`, which
has no lock; the key is read on core0, so `adsb_tui_key()` sets `s_inject_req` and `adsb_rx_task`
does the work. With no dongle that task does not exist, so it runs inline. Same for anything else
in a key handler that touches the aircraft table.

## SSH preempts the serial shell

The serial side sits at a prompt permanently, so a "console already claimed" rule would lock SSH
out forever. `shell_remote_claim()` takes the console; the serial shell gets it back by itself when
the session closes. A mutex the console task holds for as long as it is printing guarantees it is
not mid-`printf()` when `net_ssh.c` swaps `stdout`. Hence the order in `shell.h`, not
interchangeable:

    claim → swap stdout → print banner        …        restore stdout → close

`exit` over SSH sets a flag rather than releasing ownership itself, or the serial banner prints
into the dying channel.

## Event log

One ring (32 entries; the panel shows `LOG_SHOW` = 7), three entry points declared in `shell.h`
over a common `log_put()`:

| | covers | LOG panel | reaches a prompt at |
|---|---|---|---|
| `sys_log()` | the board — USB, IQ stream, WiFi, Ethernet, OTA, web server, console | **no** | `sys` and above |
| `air_log()` | the sky — CONTACT / IDENT / FIX / ALT / VEL / LOST | yes | `brief` (colours 1 and 4 only) and `all` |
| `ui_log()` | the display answering a key — PANEL, AUDIO, `leave_tui()`'s refusal | yes | `all` only |

The split is on both ends: the panel is aircraft-only, the prompt board-only by default. A board
line in a seven-line panel (a link retrying every 30 s) wipes the aircraft out, and the header row
already carries ETH/WIFI/FEED and the radar panel `USB drop`. `tui_draw()` walks the ring backwards
collecting the last `LOG_SHOW` non-`LOG_SYS` entries. The console default is `sys` (ALT/VEL lines
arrive several a second with traffic); it starts at `all` so a boot failure is not silent, and
`shell_console_start()` drops it once the prompt is up. `ui_log()` exists because `PANEL switched
to WFALL` is feedback for whoever pressed `R` — through `sys_log()` it would print into the *other*
transport's prompt. `log tail` is the one view that shows every facility in one stream.

**`shell_async_print()`** — asynchronous output (a WiFi event, a USB stall) wipes the prompt line
(`\r\033[K`), prints, then redraws prompt and buffer through `stdout`, so SSH gets the same
treatment. Three gates, each a bug if dropped: no redraw while a command runs (`s_running` — its
output is mid-line), none while the display owns stdout (escapes would land in the frame), none
when the console lock is not free. The lock is taken with **no wait** — the console task holds it
for a command's whole duration, and waiting would park the WiFi event task in `printf` for as long
as `tasks` samples its 1 s window. A call from the console task's own thread skips the take (it
would deadlock on its own mutex).

**`esp_log` goes through the same path** (`esp_log_set_vprintf()` at the end of `shell_init()`) —
otherwise IDF's, the WiFi stack's and esp_hosted's lines, which are most of what interrupts a
session, would not be covered. One `vprintf` is one line; the hook strips the trailing newline. A
per-task re-entry guard **drops** the line rather than recursing (anything under `printf()` may
log; a loop there is unbreakable on a headless board). An esp_log line still lands in a frame
while the display is in front — the next repaint wipes it, and losing an IDF error is worse.

## SSH server (`net_ssh.c`)

`david-cermak/libssh` 0.12.2 on `CONFIG_ADSB_SSH_PORT` (default 22); build requirements (termios
Kconfig, mbedTLS patch) are in the repo `CLAUDE.md`. Verified with OpenSSH and paramiko:
ssh-ed25519 host key, aes128-ctr / hmac-sha2-256, connect+auth 0.4-1.5 s, PTY size negotiated.
libssh spawns no tasks of its own; a session costs ~23-31 KB of internal RAM.

**Three design points, all forced:**

- **Output is taken at `stdout`, and under picolibc that is global.** Commands print with
  `printf()` and several are not ours to change (esp_hosted's). IDF v6.0 lists "cannot redefine
  stdin/stdout/stderr per task" as a breaking change; the thread-local copies live behind
  `components/console/private_include/console_stdio_private.h`, unreachable from a component. So
  `net_ssh.c` swaps the *global* `stdout` — safe only because `shell.c` guarantees one owner at a
  time. The target is a **`funopen()`** stream (picolibc has it; no VFS driver needed). IDF's
  `esp_libc/platform_include/stdio.h` shadows picolibc's header and keeps newlib's older
  `(char *, int)` prototype — match that one. Consequence: `ESP_LOG` from every task follows stdout
  into the session while it lasts. The serial display is unaffected — `fb_flush()` never goes
  through stdout.
- **The write callback never touches libssh.** Any task can `printf()` while stdout points at the
  session, and a libssh session must not be used from two threads. The callback only appends to
  an 8 KB PSRAM ring; the SSH task drains it and does every `ssh_channel_write()` (same shape as
  `feed_avr.c`/`feed_beast.c`). The line editor also runs in the SSH task —
  `shell_remote_byte()` runs a byte to completion in the caller's thread.
- **One session at a time**, via the owner enum; a second caller is turned away without entering
  libssh (bug 3 below).

**Six bugs found on hardware — none announced itself.** 1-4 were found with paramiko; 5-6 were
**not**, because paramiko concatenates bytes and never renders them. **Test this path with a real
`ssh` client on a real terminal.**

1. **`ssh_init()` before any `ssh_pki_*` call** — generating the host key first fails with a bare
   `SSH_ERROR`.
2. **Never `ssh_disconnect()` from inside an auth callback**, and don't rely on
   `ssh_event_dopoll()` to notice a client that walked away — a denied client left the server
   unable to accept for ~30 s. `serve()` also breaks on `ssh_is_connected()`, on
   `ssh_get_status() & (SSH_CLOSED|SSH_CLOSED_ERROR)`, and on a short post-denial grace.
3. **Never create or destroy a libssh session while another one is live.** Turning away a second
   caller with `ssh_new()`/`ssh_bind_accept()`/`ssh_free()` silently killed the session being
   served — output just stopped. `reject_extra()` does `select()` + `accept()` + `close()` on
   `ssh_bind_get_fd()`. (The task is single-threaded, so while serving it is not in
   `ssh_bind_accept()`; without this a second caller waits out its own timeout.)
4. **Swap `stdout` before `shell_remote_open()`** (which prints the banner) — and
   `shell_remote_claim()` before the swap. `SSH_OPTIONS_HOST` is the *configured* host and is
   empty server-side; the peer address comes from `getpeername()` on `ssh_get_fd()`.
5. **The stream does its own `\n → \r\n`.** The serial console gets that from the UART VFS
   (`CONFIG_LIBC_STDOUT_LINE_ENDING_CRLF`); a `funopen()` stream bypasses the VFS, and on a real
   PTY every line staircases to the right.
6. **Swap `tls_stdout`/`tls_stderr` alongside the globals** (declared as local externs, exactly as
   ESP-IDF's own `esp_console_repl_chip.c` does). `components/console/CMakeLists.txt` compiles
   `commands.c` and all of argtable3 with `--include=console_stdio_private.h` under
   `CONFIG_LIBC_PICOLIBC`, which redefines `printf()` to `fprintf(tls_stdout, …)` — so `help` and
   argtable's error messages went to the serial console and the client saw nothing. Grepping for
   `printf` will not reveal this.

**Credentials and the host key are in NVS** (`netcfg`: `ssh_user`/`ssh_pass`/`ssh_key`), never in
the image. The Ed25519 key is generated on the board on first use. **With no password stored the
server does not listen at all.**

## Command reference

Not in `README.md` (which stops at the console itself), so kept here.

| command | effect |
|---|---|
| `tui` / `q` | enter / leave the display (`:`, Ctrl-C, Ctrl-D also leave). Inside: `R` cycles RADAR/WFALL/TASKS, `t` injects a synthetic contact |
| `tasks`, `usb`, `free`, `sys`, `ac`, `sdr`, `net` | TASKS panel / USB stream probe / heap / board / aircraft table / tuner / network as text — the way to read numbers that have to be copied. `net` also prints the STA failure count and seconds to the next attempt |
| `log echo <off\|sys\|brief\|all>`, `log tail` | move the console end of the event split; dump every facility in one stream |
| `wifi sta <ssid> <pass>` | write upstream credentials to NVS (echoed in clear — fine on a cable, remember it before exposing the shell) |
| `wifi sta on\|off`, `wifi sta clear` | stop / resume the join loop keeping credentials; forget them |
| `wifi ap on\|off [force]` | SoftAP switch; `off` is refused unless Ethernet has a lease or the STA is joined, `force` overrides |
| `ssh`, `ssh set <user> <pass>`, `ssh off`, `ssh regen` | status + SHA256 fingerprint; enable; disable; throw the host key away |
| `ota`, `ota rollback` | slot names/states + app version; revert now (`esp_ota_mark_app_invalid_rollback_and_reboot()` if still pending-verify, else boot the other slot) |
| `restart`, `exit` | reboot; end the SSH session |
