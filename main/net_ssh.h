#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

/* ═════════════════════════════════════════════════════════════════════════════
 * SSH server -- the console REPL of shell.c, reachable over the network.
 *
 * Why SSH and not raw TCP: both need zero client development, but the client
 * is only preinstalled everywhere for SSH. The NAS on this LAN has no telnet
 * and no nc; macOS dropped its telnet client in 2017 and Windows ships the
 * OpenSSH client by default while its telnet client is an off-by-default
 * optional feature. The encryption is incidental here -- a home LAN and an
 * isolated car SoftAP do not need it.
 *
 * Built on david-cermak/libssh 0.12.x (vendored upstream libssh). 0.12 has
 * native mbedTLS-v4 support, which is what IDF v6.0 ships, so the
 * mbedtls_v3_shim component is NOT needed.
 *
 * ── one session at a time ────────────────────────────────────────────────────
 * Two independent reasons, either of which alone would force it:
 *
 *   - shell.c keeps one line buffer and one command context, and
 *   - stdout is a single global under picolibc (see net_ssh.c).
 *
 * A second connection is answered with a "console busy" line and closed.
 *
 * ── credentials and the host key live in NVS, never in the image ─────────────
 * Same reason as the WiFi credentials in net_wifi.c: sdkconfig is tracked and
 * this repo's remote is a public fork. The host key is generated on the board
 * on first use; the password is set from the serial console with
 * `ssh set <user> <pass>`. With no password stored the server does not listen
 * at all, so a board that has never been configured is not reachable.
 * ═══════════════════════════════════════════════════════════════════════════ */

typedef enum {
    NET_SSH_OFF = 0,     /* not started */
    NET_SSH_NOCRED,      /* running but not listening: no password in NVS */
    NET_SSH_LISTEN,      /* listening, no client */
    NET_SSH_SESSION,     /* a client is connected */
} net_ssh_state_t;

/* Starts the server task. Safe to call before the network is up -- bind() is
 * on INADDR_ANY, so Ethernet, the SoftAP and the STA are all covered without
 * per-interface code, exactly like the three feeds. */
esp_err_t net_ssh_start(void);

net_ssh_state_t net_ssh_state(void);

/* Peer address of the current session, or "---". */
void net_ssh_peer_str(char *dst, size_t n);

/* Terminal width the client asked for, 0 if it never requested a PTY or no
 * session is up. The radar wants a wide window, so `tui` warns on a narrow
 * one rather than letting the frame wrap into nonsense. */
int net_ssh_pty_cols(void);

/* One already-CRLF-expanded run of the display's frame, from fb_flush(). Not
 * a general output path: it bypasses stdout, and it blocks briefly when the
 * ring is full instead of dropping. No-op when no session is up. */
void net_ssh_tui_write(const char *data, size_t n);

/* Username the server will accept ("---" when no credentials are stored). */
void net_ssh_user_str(char *dst, size_t n);

/* Stores the login credentials in NVS and starts listening if the server was
 * waiting for them. An empty password clears them and stops the listener. */
esp_err_t net_ssh_set_login(const char *user, const char *pass);

/* Discards the stored host key so the next start generates a fresh one. Every
 * client will report a changed host key afterwards, which is the point. */
esp_err_t net_ssh_reset_hostkey(void);

/* SHA256 fingerprint of the host key in OpenSSH's "SHA256:base64" form, for
 * comparing against what the client prints on first connect. */
void net_ssh_fingerprint_str(char *dst, size_t n);
