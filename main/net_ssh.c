/* ═════════════════════════════════════════════════════════════════════════════
 * net_ssh.c -- shell.c's REPL over SSH.
 *
 * See net_ssh.h for why SSH rather than raw TCP, and why one session at a time.
 * Two things in here are not obvious from the code.
 *
 * ── how command output reaches the client ────────────────────────────────────
 * The commands print with printf(), and several of them are not ours to change
 * (esp_hosted registers mem-dump, task-dump, cpu-dump, heap-trace, sock-dump
 * and host-power-save into the same console). So the output has to be taken at
 * stdout, not at each call site.
 *
 * Under picolibc -- IDF v6.0's default -- stdout is a plain global FILE*
 * (esp_libc/src/picolibc/picolibc_init.c). IDF v6.0's migration guide lists
 * "not possible to redefine stdin/stdout/stderr for specific tasks" as a
 * breaking change: the thread-local copies exist but live behind
 * components/console/private_include/console_stdio_private.h and are not
 * reachable from here. So redirecting stdout redirects it for every task.
 *
 * That is exactly what this file does, and it is safe *because* of the
 * one-session-at-a-time rule: shell.c hands the console to one owner at a
 * time, and the TUI does not go through stdout at all (fb_flush() calls
 * uart_write_bytes() directly), so the radar keeps painting on the serial
 * monitor while a remote session is up. The visible consequence is that
 * ESP_LOG output from every task follows stdout into the SSH session for its
 * duration, which for a debug console is the wanted behaviour.
 *
 * The redirection target is a funopen() stream -- picolibc has funopen(), so
 * no custom VFS driver is needed for this.
 *
 * ── why the write callback does not touch libssh ─────────────────────────────
 * A libssh session must not be used from two threads at once, and any task in
 * the system can printf() while stdout points here. So the callback only
 * appends to a ring buffer; the SSH task drains it and does every
 * ssh_channel_write() itself. That keeps all libssh calls on one thread and
 * makes a log from a high-priority task a memcpy rather than a blocking
 * network write. Same shape as feed_avr.c / feed_beast.c.
 *
 * For the same reason the line editor runs in *this* task rather than being
 * queued to shell.c's: this task is on core0 and may block, so there is
 * nothing to hand off (shell_remote_byte(), see shell.h).
 * ═══════════════════════════════════════════════════════════════════════════ */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include <libssh/libssh.h>
#include <libssh/server.h>
#include <libssh/callbacks.h>

#include "net_ssh.h"
#include "shell.h"

static const char *TAG = "ssh";

#define NVS_NS          "netcfg"        /* shared with net_wifi.c / web_config.c */
#define KEY_HOSTKEY     "ssh_key"
#define KEY_USER        "ssh_user"
#define KEY_PASS        "ssh_pass"

#define USER_MAX        32
#define PASS_MAX        64
/* `help` prints a few KB in one uninterruptible burst -- run_line() calls
 * esp_console_run() synchronously and nothing drains the ring until it
 * returns -- so this is sized for that, not for steady-state traffic. */
#define OUT_RING_SZ     16384
#define READ_TIMEOUT_MS 20              /* poll interval while the ring is empty */
/* While the ring has backlog, run_shell() polls for input at this cadence
 * instead of READ_TIMEOUT_MS -- see the comment there for why. */
#define BUSY_POLL_MS    2
#define FLUSH_BUDGET_US (60 * 1000)     /* max ms flush_ring() writes before yielding to a read */
#define AUTH_TRIES      3

/* ═════════════════════════════════════════════════════════════════════════════
 * Output ring
 * ═══════════════════════════════════════════════════════════════════════════ */

static uint8_t          *s_ring;
static size_t            s_head, s_tail;    /* head writes, tail reads */
static SemaphoreHandle_t s_ring_mux;
static uint32_t          s_ring_dropped;

static size_t ring_used(void)
{
    return (s_head - s_tail) % OUT_RING_SZ;
}

/* Returns how many bytes were accepted. The two callers want opposite things
 * from a short push: a log line is dropped (never block a logger on the
 * network), a frame waits for room -- see net_ssh_tui_write(). */
static size_t ring_push(const uint8_t *data, size_t n)
{
    if (!s_ring || xSemaphoreTake(s_ring_mux, 0) != pdTRUE) return 0;

    size_t space = OUT_RING_SZ - 1 - ring_used();
    if (n > space) n = space;

    for (size_t i = 0; i < n; i++) {
        s_ring[s_head] = data[i];
        s_head = (s_head + 1) % OUT_RING_SZ;
    }
    xSemaphoreGive(s_ring_mux);
    return n;
}

static void log_push(const uint8_t *data, size_t n)
{
    s_ring_dropped += (uint32_t)(n - ring_push(data, n));
}

static size_t ring_pop(uint8_t *dst, size_t max)
{
    if (!s_ring || xSemaphoreTake(s_ring_mux, portMAX_DELAY) != pdTRUE) return 0;
    size_t n = ring_used();
    if (n > max) n = max;
    for (size_t i = 0; i < n; i++) {
        dst[i] = s_ring[s_tail];
        s_tail = (s_tail + 1) % OUT_RING_SZ;
    }
    xSemaphoreGive(s_ring_mux);
    return n;
}

static void ring_reset(void)
{
    if (xSemaphoreTake(s_ring_mux, portMAX_DELAY) != pdTRUE) return;
    s_head = s_tail = 0;
    s_ring_dropped  = 0;
    xSemaphoreGive(s_ring_mux);
}

#define TUI_STALL_MS  200

/* The display's frame, straight from fb_flush() in class_driver.c: ~20 KB per
 * repaint, already CRLF-expanded, bypassing stdout for the same reason it
 * bypasses the UART VFS on the serial side -- stdio and the VFS both cost per
 * call, not per byte, and a frame is thousands of calls.
 *
 * A frame must not be dropped a piece at a time the way a log line can: what
 * would be lost is half an ANSI escape, and the screen then stays wrong until
 * the next repaint. So this waits for room -- the draw task is priority 2 and
 * has nowhere to be. It waits with a bound because the task that drains this
 * ring is also the one that processes the keystroke leaving the display: if it
 * is stuck in ssh_channel_write() against a client that stopped reading, room
 * is never coming. */
void net_ssh_tui_write(const char *data, size_t n)
{
    if (!s_ring || net_ssh_state() != NET_SSH_SESSION) return;

    for (int waited = 0; n > 0; ) {
        size_t did = ring_push((const uint8_t *)data, n);
        data += did;
        n    -= did;
        if (n == 0) break;
        if (waited >= TUI_STALL_MS) { s_ring_dropped += (uint32_t)n; return; }
        vTaskDelay(pdMS_TO_TICKS(10));
        waited += 10;
    }
}

/* The funopen() stream stdout is pointed at. Allocated once and never closed:
 * a task can be inside fwrite() at the moment the session ends, so the stream
 * and the ring have to outlive every session. */
static FILE *s_out;

#if CONFIG_LIBC_PICOLIBC
/* Swapping the global stdout is not enough. ESP-IDF compiles the whole console
 * component -- commands.c and all of argtable3 -- with
 * `--include=console_stdio_private.h` (see components/console/CMakeLists.txt,
 * under `if(CONFIG_LIBC_PICOLIBC)`), and that header redefines printf() to
 * fprintf(tls_stdout, ...) and stdout to tls_stdout. So the built-in `help`
 * command, and argtable's argument-error messages, write to the *thread-local*
 * stream, not the global one -- with only the global swapped, `help` printed
 * nothing at all to the client and went to the serial console instead.
 *
 * These are real exported symbols from esp_libc's picolibc_init.c (they exist
 * because CONFIG_LIBC_PICOLIBC_NEWLIB_COMPATIBILITY is on). ESP-IDF's own
 * esp_console_repl_chip.c reaches them with exactly this local extern. Being
 * thread-local, setting them here affects only the SSH task. */
extern __thread FILE *tls_stdin;
extern __thread FILE *tls_stdout;
extern __thread FILE *tls_stderr;
#endif

/* Point everything this task prints through at `f`, saving what was there. */
typedef struct { FILE *out, *err, *tls_out, *tls_err; } stdio_save_t;

static void stdio_redirect(FILE *f, stdio_save_t *save)
{
    save->out = stdout;
    save->err = stderr;
    stdout = stderr = f;
#if CONFIG_LIBC_PICOLIBC
    save->tls_out = tls_stdout;
    save->tls_err = tls_stderr;
    tls_stdout = tls_stderr = f;
#endif
}

static void stdio_restore(const stdio_save_t *save)
{
#if CONFIG_LIBC_PICOLIBC
    tls_stdout = save->tls_out;
    tls_stderr = save->tls_err;
#endif
    stdout = save->out;
    stderr = save->err;
}

/* Signatures come from IDF's esp_libc/platform_include/stdio.h, which shadows
 * picolibc's and keeps newlib's older (char*, int) prototype for funopen(). */
static int out_write(void *cookie, const char *data, int n)
{
    static char prev;           /* last byte written; one session at a time */

    (void)cookie;
    if (n <= 0) return n;

    /* The console UART gets its \n -> \r\n translation from the UART VFS
     * (CONFIG_LIBC_STDOUT_LINE_ENDING_CRLF). This stream bypasses the VFS
     * entirely, so it has to do it itself: on a PTY a bare LF moves down
     * without returning to column 0 and every line of output staircases to
     * the right. Nothing already carrying a CR is doubled. */
    int start = 0;
    for (int i = 0; i < n; i++) {
        if (data[i] != '\n') continue;
        if ((i > 0 ? data[i - 1] : prev) == '\r') continue;   /* already CRLF */
        if (i > start) log_push((const uint8_t *)data + start, (size_t)(i - start));
        log_push((const uint8_t *)"\r\n", 2);
        start = i + 1;
    }
    if (n > start) log_push((const uint8_t *)data + start, (size_t)(n - start));

    prev = data[n - 1];
    return n;                   /* drops are counted, not reported as errors */
}

static int out_read(void *cookie, char *buf, int n)
{
    (void)cookie; (void)buf; (void)n;
    return 0;                   /* EOF: this task reads the channel itself */
}

/* ═════════════════════════════════════════════════════════════════════════════
 * Stored configuration
 * ═══════════════════════════════════════════════════════════════════════════ */

static char s_user[USER_MAX];
static char s_pass[PASS_MAX];
static char s_hostkey_fp[80] = "---";

static volatile net_ssh_state_t s_state;
static char s_peer[48] = "---";
static int  s_cols, s_rows;             /* from the client's PTY request */

static esp_err_t nvs_get_str_into(nvs_handle_t h, const char *key, char *dst, size_t cap)
{
    size_t len = cap;
    esp_err_t err = nvs_get_str(h, key, dst, &len);
    if (err != ESP_OK) dst[0] = '\0';
    return err;
}

static void load_login(void)
{
    nvs_handle_t h;
    s_user[0] = s_pass[0] = '\0';
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    nvs_get_str_into(h, KEY_USER, s_user, sizeof(s_user));
    nvs_get_str_into(h, KEY_PASS, s_pass, sizeof(s_pass));
    nvs_close(h);
    if (s_user[0] == '\0') strlcpy(s_user, "p4", sizeof(s_user));
}

esp_err_t net_ssh_set_login(const char *user, const char *pass)
{
    if (!pass) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    if (pass[0] == '\0') {
        nvs_erase_key(h, KEY_PASS);         /* clearing the password stops the listener */
    } else {
        err = nvs_set_str(h, KEY_PASS, pass);
    }
    if (err == ESP_OK && user && user[0]) err = nvs_set_str(h, KEY_USER, user);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);

    if (err == ESP_OK) load_login();
    return err;
}

esp_err_t net_ssh_reset_hostkey(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_erase_key(h, KEY_HOSTKEY);
    if (err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

/* ── host key ─────────────────────────────────────────────────────────────────
 * Ed25519, generated on the board on first use and kept in NVS. Not a Kconfig
 * blob and not embedded in the image: sdkconfig is tracked and this repo's
 * remote is a public fork, so a committed private key would be published --
 * the same reason the WiFi credentials live in NVS (net_wifi.c). */
static char *hostkey_load_or_create(void)
{
    nvs_handle_t h;
    char *pem = NULL;

    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return NULL;

    size_t len = 0;
    if (nvs_get_str(h, KEY_HOSTKEY, NULL, &len) == ESP_OK && len > 1) {
        pem = malloc(len);
        if (pem && nvs_get_str(h, KEY_HOSTKEY, pem, &len) == ESP_OK) {
            nvs_close(h);
            return pem;
        }
        free(pem);
        pem = NULL;
    }

    ESP_LOGI(TAG, "no host key stored, generating an Ed25519 one (a few seconds)");
    ssh_key key = NULL;
    if (ssh_pki_generate_key(SSH_KEYTYPE_ED25519, NULL, &key) != SSH_OK || !key) {
        ESP_LOGE(TAG, "host key generation failed");
        nvs_close(h);
        return NULL;
    }

    char *b64 = NULL;
    if (ssh_pki_export_privkey_base64(key, NULL, NULL, NULL, &b64) == SSH_OK && b64) {
        if (nvs_set_str(h, KEY_HOSTKEY, b64) == ESP_OK && nvs_commit(h) == ESP_OK) {
            pem = strdup(b64);
            ESP_LOGI(TAG, "host key generated and stored in NVS");
        } else {
            ESP_LOGE(TAG, "storing the host key failed");
        }
        ssh_string_free_char(b64);
    }
    ssh_key_free(key);
    nvs_close(h);
    return pem;
}

static void hostkey_fingerprint(const char *pem)
{
    ssh_key priv = NULL, pub = NULL;
    unsigned char *hash = NULL;
    size_t hlen = 0;
    char *fp = NULL;

    if (ssh_pki_import_privkey_base64(pem, NULL, NULL, NULL, &priv) != SSH_OK) return;
    if (ssh_pki_export_privkey_to_pubkey(priv, &pub) == SSH_OK &&
        ssh_get_publickey_hash(pub, SSH_PUBLICKEY_HASH_SHA256, &hash, &hlen) == 0) {
        fp = ssh_get_fingerprint_hash(SSH_PUBLICKEY_HASH_SHA256, hash, hlen);
        if (fp) {
            strlcpy(s_hostkey_fp, fp, sizeof(s_hostkey_fp));
            ssh_string_free_char(fp);
        }
        ssh_clean_pubkey_hash(&hash);
    }
    if (pub)  ssh_key_free(pub);
    if (priv) ssh_key_free(priv);
}

/* ═════════════════════════════════════════════════════════════════════════════
 * Session callbacks
 * ═══════════════════════════════════════════════════════════════════════════ */

typedef struct {
    int         authed;
    int         tries;
    ssh_channel channel;
} sess_t;

/* The listener, so the session loop can turn away a second caller. */
static ssh_bind s_bind;

static int cb_auth_none(ssh_session session, const char *user, void *ud)
{
    (void)user; (void)ud;
    ssh_set_auth_methods(session, SSH_AUTH_METHOD_PASSWORD);
    return SSH_AUTH_DENIED;
}

static int cb_auth_password(ssh_session session, const char *user,
                            const char *password, void *ud)
{
    sess_t *s = (sess_t *)ud;

    /* Constant-time-ish is pointless here (the password is set over a serial
     * cable on a LAN device), but a rate limit is not: three tries then drop. */
    if (s_pass[0] && strcmp(user, s_user) == 0 && strcmp(password, s_pass) == 0) {
        s->authed = 1;
        return SSH_AUTH_SUCCESS;
    }
    /* Deliberately no ssh_disconnect() here: tearing the session down from
     * inside its own callback left libssh in a state that took ~15 s to
     * unwind, during which the server accepted nothing. serve()'s event loop
     * watches `tries` and does the teardown on the way out instead. */
    (void)session;
    ESP_LOGW(TAG, "auth failed for user '%s'", user);
    s->tries++;
    return SSH_AUTH_DENIED;
}

static int cb_pty(ssh_session session, ssh_channel channel, const char *term,
                  int cols, int rows, int py, int px, void *ud)
{
    (void)session; (void)channel; (void)py; (void)px; (void)ud;
    s_cols = cols;
    s_rows = rows;
    ESP_LOGI(TAG, "pty %s %dx%d", term ? term : "?", cols, rows);
    return SSH_OK;
}

static int cb_pty_resize(ssh_session session, ssh_channel channel,
                         int cols, int rows, int px, int py, void *ud)
{
    (void)session; (void)channel; (void)px; (void)py; (void)ud;
    s_cols = cols;
    s_rows = rows;
    return SSH_OK;
}

static int cb_shell(ssh_session session, ssh_channel channel, void *ud)
{
    (void)session; (void)channel; (void)ud;
    return SSH_OK;
}

static struct ssh_channel_callbacks_struct s_chan_cb = {
    .channel_pty_request_function       = cb_pty,
    .channel_pty_window_change_function = cb_pty_resize,
    .channel_shell_request_function     = cb_shell,
};

static ssh_channel cb_channel_open(ssh_session session, void *ud)
{
    sess_t *s = (sess_t *)ud;
    if (s->channel) return NULL;            /* one channel per session */
    s->channel      = ssh_channel_new(session);
    s_chan_cb.userdata = s;
    ssh_callbacks_init(&s_chan_cb);
    ssh_set_channel_callbacks(s->channel, &s_chan_cb);
    return s->channel;
}

/* ═════════════════════════════════════════════════════════════════════════════
 * The shell session
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Draining the whole backlog before returning to ssh_channel_read_timeout()
 * is fine on a healthy link -- each write is sub-ms and the loop is back
 * reading within READ_TIMEOUT_MS regardless. Over a slow/lossy link (a phone
 * hotspot) each ssh_channel_write() can itself take tens to hundreds of ms,
 * and a full 16KB backlog is up to 32 of them: the read side starves for as
 * long as that takes, which is what makes keystrokes feel dead. Capping by
 * wall-clock time rather than chunk count costs nothing on the healthy path
 * (the budget is never hit) and bounds the stall on the slow one -- the rest
 * of the ring just waits for the next pass through the loop.
 *
 * The bound only helps if the loop actually comes back promptly -- see
 * run_shell()'s BUSY_POLL_MS for the other half of this. */
static bool flush_ring(ssh_channel ch)
{
    uint8_t buf[512];
    size_t  n;
    int64_t t0 = esp_timer_get_time();
    while ((n = ring_pop(buf, sizeof(buf))) > 0) {
        if (ssh_channel_write(ch, buf, (uint32_t)n) == SSH_ERROR) return false;
        if (esp_timer_get_time() - t0 >= FLUSH_BUDGET_US) break;
    }
    return true;
}

/* The channel of the session run_shell() is currently serving -- valid only
 * for the duration of that call, which is this task's. net_ssh_write_now()
 * needs it to drain the ring itself rather than wait on the loop below. */
static ssh_channel s_active_channel;

/* See the declaration in net_ssh.h for why this exists instead of reusing
 * net_ssh_tui_write(). Draining first and pushing as room opens up, rather
 * than pushing once and then draining, means it also works through whatever
 * backlog the display already left in the ring before this call's own bytes
 * can even be queued -- which is correct, not just a side effect: content
 * already committed to the wire has to leave before a clear-screen means
 * anything. Bounded the same way everything else touching this ring is, so a
 * client that has stopped reading cannot wedge this task here forever. */
void net_ssh_write_now(const char *data, size_t n)
{
    if (!s_ring || !s_active_channel) return;

    int64_t t0 = esp_timer_get_time();
    while ((n > 0 || ring_used() > 0) && esp_timer_get_time() - t0 < 3000000) {
        size_t did = ring_push((const uint8_t *)data, n);
        data += did;
        n    -= did;
        if (!flush_ring(s_active_channel)) return;
        if (did == 0 && n > 0) vTaskDelay(pdMS_TO_TICKS(5));  /* ring stayed full; give the peer a beat */
    }
}

/* This task is single-threaded, so while it is serving a session it is not in
 * ssh_bind_accept() and a second caller would sit in the listen backlog until
 * its own client timed out (~15 s of silence). Closing it immediately turns
 * that into a prompt "connection reset by peer", which is at least honest.
 * The busy *message* in run_shell() cannot be delivered here -- that would
 * need a full key exchange for a session about to be dropped -- so it only
 * ever reaches a caller who arrives while the serial shell holds the console.
 *
 * This deliberately does NOT go through libssh. Doing it the obvious way --
 * ssh_new() / ssh_bind_accept() / ssh_free() every 500 ms -- silently killed
 * the session already being served: after the first such call the live
 * session stopped producing output entirely, no error anywhere. libssh's
 * session objects are not safe to create and destroy underneath a live one,
 * so this takes the connection off the listening socket directly and never
 * touches libssh state. */
static void reject_extra(void)
{
    socket_t lfd = ssh_bind_get_fd(s_bind);
    if (lfd < 0) return;

    /* select() rather than trusting the listener to be non-blocking:
     * ssh_bind_set_blocking() is libssh's own flag and says nothing about
     * O_NONBLOCK on the fd, and a blocking accept() here would wedge the
     * session for as long as nobody called. */
    fd_set         r;
    struct timeval tv = { 0, 0 };
    FD_ZERO(&r);
    FD_SET(lfd, &r);
    if (select(lfd + 1, &r, NULL, NULL, &tv) <= 0) return;

    struct sockaddr_storage sa;
    socklen_t               sl = sizeof(sa);
    int fd = accept(lfd, (struct sockaddr *)&sa, &sl);
    if (fd < 0) return;

    ESP_LOGW(TAG, "second connection refused: console already in session");
    close(fd);
}

static void run_shell(ssh_channel ch)
{
    /* Three steps in a fixed order, none of them interchangeable (shell.h
     * states the contract). Claim first and unredirected: the serial shell now
     * sits at a prompt permanently, so this takes the console away from it,
     * and until it has let go its own echo printf()s would land in this
     * session. Swap stdout second. Print the banner last -- doing that before
     * the swap sends it to the serial console and the client sees a blank
     * session. */
    if (!shell_remote_claim()) {
        static const char busy[] =
            "\r\nconsole busy: another session has it\r\n";
        ssh_channel_write(ch, busy, sizeof(busy) - 1);
        return;
    }

    ring_reset();
    stdio_save_t saved;
    stdio_redirect(s_out, &saved);   /* see the file header for why this is global */

    s_active_channel = ch;      /* net_ssh_write_now() needs this for the duration */

    shell_remote_open();

    s_state = NET_SSH_SESSION;
    flush_ring(ch);             /* the banner shell_remote_open() just printed */

    uint8_t buf[256];
    bool    live         = true;
    int64_t next_reject  = esp_timer_get_time() + 500000;

    while (live && ssh_channel_is_open(ch) && !ssh_channel_is_eof(ch)) {
        /* Waiting the full READ_TIMEOUT_MS here for a keystroke that never
         * comes is free when the ring is empty -- there is nothing else this
         * task could be doing. It is not free when the ring has backlog: that
         * wait runs *between* every FLUSH_BUDGET_US-sized write burst, so on a
         * slow link it was costing as much dead time as the write itself,
         * roughly halving throughput (this is what made the TUI's SSH refresh
         * visibly slower after flush_ring() got bounded). Polling fast instead
         * while there is something to drain gets almost all of that back,
         * without reopening the starvation the bound exists to prevent --
         * ring_used() is read without the mutex, which is fine for a
         * poll-interval heuristic. */
        int wait_ms = ring_used() ? BUSY_POLL_MS : READ_TIMEOUT_MS;
        int n = ssh_channel_read_timeout(ch, buf, sizeof(buf), 0, wait_ms);
        if (n == SSH_ERROR) break;

        for (int i = 0; i < n && live; i++) live = shell_remote_byte(buf[i]);

        if (!flush_ring(ch)) break;

        /* Wall-clock rather than a loop-iteration count: wait_ms now varies,
         * so a fixed tick count no longer means a fixed elapsed time. */
        int64_t now = esp_timer_get_time();
        if (now >= next_reject) { next_reject = now + 500000; reject_extra(); }
    }

    s_active_channel = NULL;
    stdio_restore(&saved);      /* before anything else can printf */

    shell_remote_close();
    ssh_channel_write(ch, "\r\nbye\r\n", 7);
    ssh_channel_send_eof(ch);
}

static void serve(ssh_session session)
{
    sess_t s = { 0 };

    struct ssh_server_callbacks_struct server_cb = {
        .userdata                              = &s,
        .auth_none_function                    = cb_auth_none,
        .auth_password_function                = cb_auth_password,
        .channel_open_request_session_function = cb_channel_open,
    };
    ssh_callbacks_init(&server_cb);
    ssh_set_server_callbacks(session, &server_cb);

    if (ssh_handle_key_exchange(session) != SSH_OK) {
        ESP_LOGW(TAG, "key exchange failed: %s", ssh_get_error(session));
        return;
    }
    ssh_set_auth_methods(session, SSH_AUTH_METHOD_PASSWORD);

    ssh_event event = ssh_event_new();
    if (event && ssh_event_add_session(event, session) == SSH_OK) {
        /* Bounded so a client that connects and then says nothing cannot hold
         * the single session slot open: 100 x 100 ms = 10 s, which is ample
         * for a key exchange plus auth on a LAN.
         *
         * The ssh_is_connected() test is what keeps that bound from being the
         * common case: a client that is denied (paramiko gives up after one
         * try) or that walks away just closes the socket, and ssh_event_dopoll
         * does NOT report that as an error -- so without this the loop ran its
         * full length and the server accepted nothing for the next 30 s. */
        for (int i = 0; i < 100 && (!s.authed || !s.channel); i++) {
            if (s.tries >= AUTH_TRIES) break;
            if (ssh_event_dopoll(event, 100) == SSH_ERROR) break;
            if (!ssh_is_connected(session)) break;
            if (ssh_get_status(session) & (SSH_CLOSED | SSH_CLOSED_ERROR)) break;

            /* A client that has been denied once and is not coming back gets
             * 2 s, not the full 10: paramiko gives up after a single try and
             * simply stops talking, and neither ssh_is_connected() nor the
             * status bits notice quickly enough on their own. OpenSSH's three
             * attempts arrive well inside this. */
            if (s.tries > 0 && i > 20) break;
        }
        if (s.authed && s.channel) run_shell(s.channel);
    }

    if (s.channel) ssh_channel_free(s.channel);
    if (event)     ssh_event_free(event);
    ssh_disconnect(session);
}

/* ═════════════════════════════════════════════════════════════════════════════
 * Server task
 * ═══════════════════════════════════════════════════════════════════════════ */

static void ssh_task(void *arg)
{
    (void)arg;

    /* ssh_init() first: it brings up libssh's crypto backend, and generating
     * or importing a key before that fails with no useful diagnostic. */
    if (ssh_init() != SSH_OK) {
        ESP_LOGE(TAG, "ssh_init() failed");
        vTaskDelete(NULL);
    }

    char *pem = hostkey_load_or_create();
    if (!pem) { ESP_LOGE(TAG, "no host key, SSH disabled"); vTaskDelete(NULL); }
    hostkey_fingerprint(pem);

    s_bind = ssh_bind_new();
    ssh_bind sshbind = s_bind;      /* short alias; s_bind is what reject_extra() uses */
    char port[8];
    snprintf(port, sizeof(port), "%d", CONFIG_ADSB_SSH_PORT);
    ssh_bind_options_set(sshbind, SSH_BIND_OPTIONS_BINDADDR, "0.0.0.0");
    ssh_bind_options_set(sshbind, SSH_BIND_OPTIONS_BINDPORT_STR, port);
    ssh_bind_options_set(sshbind, SSH_BIND_OPTIONS_LOG_VERBOSITY_STR, "0");

    if (ssh_bind_options_set(sshbind, SSH_BIND_OPTIONS_IMPORT_KEY_STR, pem) != SSH_OK) {
        ESP_LOGE(TAG, "host key rejected: %s", ssh_get_error(sshbind));
        free(pem);
        ssh_bind_free(sshbind);
        vTaskDelete(NULL);
    }
    free(pem);                  /* libssh has parsed and copied it */

    if (ssh_bind_listen(sshbind) != SSH_OK) {
        ESP_LOGE(TAG, "listen on %s failed: %s", port, ssh_get_error(sshbind));
        ssh_bind_free(sshbind);
        vTaskDelete(NULL);
    }

    /* Non-blocking, so clearing the password with `ssh set` takes effect
     * within 100 ms instead of when the next client happens to connect. */
    ssh_bind_set_blocking(sshbind, 0);

    ESP_LOGI(TAG, "listening on :%s, host key %s", port, s_hostkey_fp);

    while (true) {
        if (s_pass[0] == '\0') {
            if (s_state != NET_SSH_NOCRED) {
                ESP_LOGW(TAG, "no password stored -- not accepting connections. "
                              "Set one from the serial console: ssh set <user> <pass>");
            }
            s_state = NET_SSH_NOCRED;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        s_state = NET_SSH_LISTEN;
        strlcpy(s_peer, "---", sizeof(s_peer));
        s_cols = s_rows = 0;    /* the next client's cb_pty fills these in */

        ssh_session session = ssh_new();
        if (!session) { vTaskDelay(pdMS_TO_TICKS(1000)); continue; }

        int rc = ssh_bind_accept(sshbind, session);
        if (rc == SSH_AGAIN) {
            ssh_free(session);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (rc != SSH_OK) {
            ESP_LOGW(TAG, "accept failed: %s", ssh_get_error(sshbind));
            ssh_free(session);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* The session inherits the bind's non-blocking flag; the handshake and
         * the shell loop both want it blocking (with explicit timeouts). */
        ssh_set_blocking(session, 1);

        /* SSH_OPTIONS_HOST is the *configured* host and is empty server-side,
         * so ask the socket instead. */
        struct sockaddr_storage sa;
        socklen_t               salen = sizeof(sa);
        socket_t                fd    = ssh_get_fd(session);
        if (fd >= 0 && getpeername(fd, (struct sockaddr *)&sa, &salen) == 0) {
            char ip[INET6_ADDRSTRLEN] = "?";
            if (sa.ss_family == AF_INET) {
                struct sockaddr_in *s4 = (struct sockaddr_in *)&sa;
                inet_ntop(AF_INET, &s4->sin_addr, ip, sizeof(ip));
                snprintf(s_peer, sizeof(s_peer), "%s:%u", ip, ntohs(s4->sin_port));
            } else {
                struct sockaddr_in6 *s6 = (struct sockaddr_in6 *)&sa;
                inet_ntop(AF_INET6, &s6->sin6_addr, ip, sizeof(ip));
                snprintf(s_peer, sizeof(s_peer), "[%s]:%u", ip, ntohs(s6->sin6_port));
            }
        }
        ESP_LOGI(TAG, "session from %s", s_peer);

        serve(session);
        ssh_free(session);

        ESP_LOGI(TAG, "session closed (%s)", s_peer);
        if (s_ring_dropped) {
            ESP_LOGW(TAG, "%" PRIu32 " output bytes dropped this session", s_ring_dropped);
        }
    }
}

/* ═════════════════════════════════════════════════════════════════════════════
 * Shell command
 * ═══════════════════════════════════════════════════════════════════════════ */

static const char *state_str(net_ssh_state_t s)
{
    switch (s) {
        case NET_SSH_NOCRED:  return "no password set";
        case NET_SSH_LISTEN:  return "listening";
        case NET_SSH_SESSION: return "session active";
        default:              return "off";
    }
}

static int cmd_ssh(int argc, char **argv)
{
    if (argc >= 2 && strcmp(argv[1], "set") == 0) {
        if (argc != 4) { printf("usage: ssh set <user> <password>\n"); return 1; }
        esp_err_t err = net_ssh_set_login(argv[2], argv[3]);
        printf("%s\n", err == ESP_OK ? "stored; takes effect on the next connection"
                                     : esp_err_to_name(err));
        return err == ESP_OK ? 0 : 1;
    }
    if (argc >= 2 && strcmp(argv[1], "off") == 0) {
        esp_err_t err = net_ssh_set_login(NULL, "");
        printf("%s\n", err == ESP_OK ? "password cleared; no longer accepting connections"
                                     : esp_err_to_name(err));
        return err == ESP_OK ? 0 : 1;
    }
    if (argc >= 2 && strcmp(argv[1], "regen") == 0) {
        esp_err_t err = net_ssh_reset_hostkey();
        printf("%s\n", err == ESP_OK
               ? "host key erased; a new one is generated on the next restart\n"
                 "(every client will report a changed host key -- that is expected)"
               : esp_err_to_name(err));
        return err == ESP_OK ? 0 : 1;
    }
    if (argc >= 2) { printf("usage: ssh [set <user> <pass> | off | regen]\n"); return 1; }

    printf("port      %d\n", CONFIG_ADSB_SSH_PORT);
    printf("state     %s\n", state_str(s_state));
    printf("user      %s\n", s_pass[0] ? s_user : "--- (no password stored)");
    printf("host key  %s\n", s_hostkey_fp);
    printf("peer      %s\n", s_peer);
    if (s_cols) printf("term      %dx%d\n", s_cols, s_rows);
    return 0;
}

/* ═════════════════════════════════════════════════════════════════════════════
 * Public entry points
 * ═══════════════════════════════════════════════════════════════════════════ */

net_ssh_state_t net_ssh_state(void) { return s_state; }

void net_ssh_peer_str(char *dst, size_t n) { strlcpy(dst, s_peer, n); }

int net_ssh_pty_cols(void) { return s_cols; }

void net_ssh_user_str(char *dst, size_t n)
{
    strlcpy(dst, s_pass[0] ? s_user : "---", n);
}

void net_ssh_fingerprint_str(char *dst, size_t n) { strlcpy(dst, s_hostkey_fp, n); }

esp_err_t net_ssh_start(void)
{
    load_login();

    s_ring_mux = xSemaphoreCreateMutex();
    if (!s_ring_mux) return ESP_ERR_NO_MEM;

    /* PSRAM: only the CPU touches it, and the P4 guarantees coherence for
     * pointer-only PSRAM access across both cores (same argument as the USB
     * streaming ring in esp_libusb.c). Never freed -- see s_out below. */
#if CONFIG_SPIRAM
    s_ring = heap_caps_malloc(OUT_RING_SZ, MALLOC_CAP_SPIRAM);
    if (!s_ring)
#endif
    s_ring = malloc(OUT_RING_SZ);
    if (!s_ring) { vSemaphoreDelete(s_ring_mux); s_ring_mux = NULL; return ESP_ERR_NO_MEM; }

    s_out = funopen(NULL, out_read, out_write, NULL, NULL);
    if (!s_out) { free(s_ring); s_ring = NULL; return ESP_FAIL; }
    setvbuf(s_out, NULL, _IONBF, 0);    /* the ring is the buffer */

    const esp_console_cmd_t c = {
        .command = "ssh",
        .help    = "SSH console status, or set/clear the login and the host key",
        .hint    = NULL,
        .func    = cmd_ssh,
    };
    esp_console_cmd_register(&c);

    /* core0 at priority 3, matching shell.c and web_config.c's httpd. The key
     * exchange peaks around 5 KB of stack (the component's own footprint.md)
     * and the shell commands run in this task too. Never raise this to
     * adsb_rx_task's 5 or above. */
    if (xTaskCreatePinnedToCore(ssh_task, "ssh_srv", 12288, NULL, 3, NULL, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
