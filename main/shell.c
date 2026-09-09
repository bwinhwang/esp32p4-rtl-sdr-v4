/* ═════════════════════════════════════════════════════════════════════════════
 * shell.c -- esp_console REPL, the serial console's top-level face.
 *
 * The serial port comes up at a prompt exactly as the SSH session does, and
 * the ADS-B display is the `tui` command. See shell.h for why
 * esp_console_new_repl_uart() is still not used and for the ownership rules
 * between the two transports.
 *
 * The console task here owns UART0's reader. That is a change from the
 * original arrangement, where adsb_rx_task on core1 polled the UART between
 * demodulator passes and handed bytes over through a queue: the shell has to
 * work before a dongle is enumerated (that task does not exist yet) and while
 * one is missing entirely, so the reader had to move off the demod hot path
 * for good. It sits on core0 at priority 3 and blocks in uart_read_bytes(),
 * which also means a command may take as long as it likes without stalling
 * anything on core1.
 *
 * The Read half is deliberately small: raw byte handling with backspace,
 * Ctrl-C and Ctrl-U, and no linenoise. linenoise wants to own stdin and block
 * in it, which is exactly the conflict being avoided -- history and Tab
 * completion are not worth a second reader on the one console UART.
 * ═══════════════════════════════════════════════════════════════════════════ */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "driver/uart_vfs.h"

#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_app_desc.h"
#include "esp_idf_version.h"
#include "esp_ota_ops.h"

#include "shell.h"
#include "net_eth.h"
#include "net_ssh.h"
#include "net_wifi.h"
#include "feed_avr.h"
#include "feed_beast.h"
#include "feed_json.h"

#if CONFIG_SPIRAM
#include "esp_psram.h"
#endif

#define SHELL_LINE_MAX   160
#define PROMPT           "p4> "
#define CLS              "\033[2J\033[H"
#define WIPE_LINE        "\r\033[K"
#define CONSOLE_UART     ((uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM)

/* How long shell_remote_claim() waits for the console task to finish whatever
 * it is in the middle of. The longest-blocking command is `tasks`, which
 * samples over a 1 s window. */
#define SHELL_CLAIM_MS   3000

/* Which transport owns the console. There is one line buffer, one command
 * context and -- under picolibc -- one global stdout, so ownership is
 * exclusive; see shell.h and net_ssh.c. */
typedef enum { OWNER_NONE = 0, OWNER_UART, OWNER_SSH } owner_t;

static volatile owner_t  s_owner;
static volatile bool     s_tui_fg;        /* the TUI is the serial foreground */
static volatile bool     s_tui_ssh;       /* ...and/or the session's          */
static volatile bool     s_ready;         /* shell_console_start() has run    */
static volatile bool     s_remote_quit;   /* the SSH user typed exit/Ctrl-D   */
static volatile bool     s_running;       /* a command is producing output    */
static bool              s_clear_pending; /* the TUI left a full screen       */
static TaskHandle_t      s_console_tsk;

/* Held by the console task for as long as it is touching the line buffer or
 * printing, and taken by shell_remote_claim() so an arriving SSH session can
 * never land between the task's ownership check and its printf(). The task
 * does not hold it while blocked in uart_read_bytes(), so a claim is quick in
 * the normal case. */
static SemaphoreHandle_t s_lock;

static char              s_line[SHELL_LINE_MAX];
static int               s_len;

/* The transports the display can be attached to. Both write below stdio: the
 * UART one for the reason in class_driver.c's frame-buffer comment, the SSH
 * one because a frame must not be dropped a piece at a time. This file is
 * where they belong -- it is already the one arbitrating between the two --
 * and it is what keeps the display itself from knowing either exists. */
static void uart_sink(void *ctx, const char *data, size_t n)
{
    (void)ctx;
    uart_write_bytes(CONSOLE_UART, data, n);
}

static void ssh_sink(void *ctx, const char *data, size_t n)
{
    (void)ctx;
    net_ssh_tui_write(data, n);
}

/* The transport holding stdout is the one an echoed log line would land on, so
 * that is the only display state worth testing here. While the display is up
 * on the serial console the console is unowned (see cmd_tui), which leaves
 * s_tui_fg as the answer -- including when an SSH session is at a prompt
 * alongside it, where echoing into the session is exactly right. */
bool shell_tui_foreground(void)
{
    return (s_owner == OWNER_SSH) ? s_tui_ssh : s_tui_fg;
}

/* Writes straight to the UART, bypassing stdout. The console task's own
 * printf() is unusable in exactly the moments this is needed -- while an SSH
 * session holds stdout -- and the TUI's fb_flush() already writes the port
 * this way, so there is no VFS to do the \n -> \r\n translation here. */
static void uart_note(const char *s)
{
    uart_write_bytes(CONSOLE_UART, s, strlen(s));
}

/* ── output that arrives on its own schedule ─────────────────────────────────
 * A WiFi event, a USB stall, an esp_log line from any task in the system: all
 * of them used to land wherever the cursor happened to be, which at a prompt
 * means on top of a half-typed command. The line buffer was never damaged --
 * only what the terminal showed -- so the user was left looking at wreckage
 * and retyping a command that was still perfectly intact.
 *
 * So wipe the prompt line, print, and draw the prompt and the buffer again.
 * Everything goes through stdout, which means whichever transport owns the
 * console gets it, session included.
 *
 * The lock is taken with no wait, and both failure paths fall back to a plain
 * print rather than to a redraw that would be wrong. A wait would be worse
 * than useless: the console task holds this for as long as a command runs, so
 * a `tasks` sampling its 1 s window would park the WiFi event task in here. */
void shell_async_print(const char *line)
{
    if (!line || !*line) return;
    if (!s_lock) { printf("%s\n", line); fflush(stdout); return; }

    /* Reachable from a command's own thread -- anything a command calls may
     * log -- and that thread already holds the lock. */
    bool self = (xTaskGetCurrentTaskHandle() == s_console_tsk);
    bool held = !self && xSemaphoreTake(s_lock, 0) == pdTRUE;

    /* Only redraw when there is an idle prompt to redraw. A running command
     * has the console to itself and its output is mid-line; a display in front
     * of stdout has no prompt at all and would take the escape codes into its
     * frame. */
    bool at_prompt = (self || held) && s_ready && !s_running &&
                     s_owner != OWNER_NONE && !shell_tui_foreground();

    if (at_prompt) printf(WIPE_LINE);
    printf("%s\n", line);
    if (at_prompt) {
        printf(PROMPT);
        if (s_len > 0) printf("%.*s", s_len, s_line);
    }
    fflush(stdout);

    if (held) xSemaphoreGive(s_lock);
}

/* esp_log's whole output, routed through the same path. IDF, the WiFi stack
 * and esp_hosted all log from their own tasks and none of them knows a prompt
 * exists; without this the fix above would cover the receiver's own events and
 * nothing else, which is most of what actually interrupts a session.
 *
 * One call is one line -- esp_log_writev() formats the level, timestamp, tag
 * and message into a single vprintf -- so the trailing newline is stripped and
 * shell_async_print() puts it back where it belongs. Truncation only affects
 * unusually long lines and beats a per-task heap allocation on this path.
 *
 * A line printed while the display is in front still lands in the frame; it is
 * gone by the next repaint, and losing an IDF error entirely is worse. The
 * receiver's own events do not have that problem -- they are already in the
 * panel, so log_put() does not echo them at all in that state.
 *
 * The re-entry test is not theoretical caution: everything below printf() is
 * free to log, and one such line arriving inside this hook is an unbreakable
 * loop on a board nobody is watching. */
static int log_vprintf(const char *fmt, va_list ap)
{
    static volatile TaskHandle_t s_in_log;

    TaskHandle_t me = xTaskGetCurrentTaskHandle();
    if (s_in_log == me) return 0;
    s_in_log = me;

    char buf[160];

    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    if (n <= 0) { s_in_log = NULL; return n; }

    int len = n < (int)sizeof(buf) - 1 ? n : (int)sizeof(buf) - 1;
    while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r')) buf[--len] = '\0';
    if (len > 0) shell_async_print(buf);

    s_in_log = NULL;
    return n;
}

/* ═════════════════════════════════════════════════════════════════════════════
 * Commands
 *
 * Plain argc/argv rather than argtable3: the whole command set fits in a few
 * positional arguments, and argtable's payoff is generated hints that the
 * linenoise-less Read half above cannot show anyway.
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ── free ─────────────────────────────────────────────────────────────────── */
static int cmd_free(int argc, char **argv)
{
    /* Internal and PSRAM are reported separately on purpose. With
     * CONFIG_SPIRAM_USE_MALLOC the generic esp_get_free_heap_size() folds 32 MB
     * of PSRAM into one number and hides the figure that actually runs out. */
    printf("internal  free %7u  min %7u  largest %7u\n",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
#if CONFIG_SPIRAM
    printf("psram     free %7u  min %7u  largest %7u  total %u\n",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM),
           (unsigned)esp_psram_get_size());
#else
    printf("psram     not enabled in this build\n");
#endif
    return ESP_OK;
}

/* ── tasks ────────────────────────────────────────────────────────────────── */
#define SHELL_MAX_TASKS 32

/* Static because the shell task's stack should not carry ~3 KB of snapshots,
 * and there is only ever one shell. */
static TaskStatus_t s_snap_a[SHELL_MAX_TASKS], s_snap_b[SHELL_MAX_TASKS];

static int cmd_tasks(int argc, char **argv)
{
    /* The TUI's cached stats (s_tstat) are refreshed from tui_draw(), which
     * does not run while the TUI is in the background -- so they would be
     * stale here. Sample fresh, twice, and take the delta over the gap. */
    uint32_t    ta, tb;
    UBaseType_t na = uxTaskGetSystemState(s_snap_a, SHELL_MAX_TASKS, &ta);
    if (na == 0) { printf("uxTaskGetSystemState failed (raise SHELL_MAX_TASKS)\n"); return ESP_FAIL; }

    int64_t t0 = esp_timer_get_time();
    vTaskDelay(pdMS_TO_TICKS(1000));
    int64_t span = esp_timer_get_time() - t0;

    UBaseType_t nb = uxTaskGetSystemState(s_snap_b, SHELL_MAX_TASKS, &tb);
    if (nb == 0) { printf("uxTaskGetSystemState failed (raise SHELL_MAX_TASKS)\n"); return ESP_FAIL; }

    printf("%-16s %4s %4s %5s %8s\n", "TASK", "CORE", "PRIO", "CPU%", "STACK");
    for (UBaseType_t i = 0; i < nb; i++) {
        int pct = 0;
        /* Match on the handle: uxTaskGetSystemState() gives no stable ordering
         * and tasks come and go. An unmatched handle just reads 0%. */
        for (UBaseType_t j = 0; j < na; j++) {
            if (s_snap_a[j].xHandle != s_snap_b[i].xHandle) continue;
            uint32_t d = s_snap_b[i].ulRunTimeCounter - s_snap_a[j].ulRunTimeCounter;
            if (span > 0) pct = (int)((int64_t)d * 100 / span);
            if (pct > 100) pct = 100;
            break;
        }
        char core[12] = "-";   /* sized for any int: xCoreID is 0/1 here, but -Werror=format-truncation cannot know that */
#if ( configTASKLIST_INCLUDE_COREID == 1 )
        if (s_snap_b[i].xCoreID != tskNO_AFFINITY)
            snprintf(core, sizeof(core), "%d", (int)s_snap_b[i].xCoreID);
#endif
        printf("%-16s %4s %4u %4d%% %8u\n",
               s_snap_b[i].pcTaskName ? s_snap_b[i].pcTaskName : "?",
               core,
               (unsigned)s_snap_b[i].uxCurrentPriority,
               pct,
               (unsigned)s_snap_b[i].usStackHighWaterMark);
    }
    return ESP_OK;
}

/* ── net ──────────────────────────────────────────────────────────────────── */
static const char *eth_state_str(net_eth_state_t s)
{
    switch (s) {
        case NET_ETH_OFF:    return "off";
        case NET_ETH_NOLINK: return "no-link";
        case NET_ETH_LINK:   return "link (no address)";
        case NET_ETH_READY:  return "ready";
        default:             return "?";
    }
}

static const char *wifi_state_str(net_wifi_state_t s)
{
    /* NET_WIFI_AP only means "not joined upstream" -- whether anything is
     * beaconing depends on the SoftAP switch. */
    bool ap = net_wifi_ap_enabled();
    switch (s) {
        case NET_WIFI_OFF:  return "off";
        case NET_WIFI_INIT: return "bringing up";
        case NET_WIFI_AP:   return ap ? "AP only"         : "idle";
        case NET_WIFI_STA:  return ap ? "AP + STA joined" : "STA joined";
        default:            return "?";
    }
}

static int cmd_net(int argc, char **argv)
{
    char ip[24], ssid[36];

    net_eth_ip_str(ip, sizeof(ip));
    printf("eth    %-18s %s\n", eth_state_str(net_eth_state()), ip);

    net_wifi_sta_ip_str(ip, sizeof(ip));
    net_wifi_sta_ssid(ssid, sizeof(ssid));
    printf("wifi   %-18s %s\n", wifi_state_str(net_wifi_state()), ip);
    printf("  ap  %-3s  %d client(s)\n",
           net_wifi_ap_enabled() ? "on" : "off", net_wifi_ap_clients());
    printf("  sta %-3s  ssid %s\n",
           net_wifi_sta_enabled() ? "on" : "off", ssid[0] ? ssid : "(unset)");
    if (ssid[0] && net_wifi_sta_enabled() && net_wifi_state() != NET_WIFI_STA) {
        int tries, next_s;
        net_wifi_sta_retry(&tries, &next_s);
        printf("           join failed %dx, next attempt in %ds%s\n",
               tries, next_s,
               net_wifi_ap_clients() > 0 ? " (held back: SoftAP in use)" : "");
    }

    /* Both interfaces can hold addresses on the same subnet at home, in which
     * case only the Ethernet one answers: lwIP's ip4_route() picks the first
     * netif whose subnet matches and Ethernet is added first. See CLAUDE.md. */
    printf("feeds  avr %d client(s) sent %u drop %u\n",
           feed_avr_clients(), (unsigned)feed_avr_sent(), (unsigned)feed_avr_dropped());
    printf("       beast %d client(s) sent %u drop %u\n",
           feed_beast_clients(), (unsigned)feed_beast_sent(), (unsigned)feed_beast_dropped());
    printf("       json %d client(s)\n", feed_json_clients());
    return ESP_OK;
}

/* ── wifi ───────────────────────────────────────────────────────────────── */
/* -1 for anything that is not the word on or the word off. */
static int onoff(const char *s)
{
    if (strcmp(s, "on")  == 0) return 1;
    if (strcmp(s, "off") == 0) return 0;
    return -1;
}

/* Whether the board would still be reachable over IP with the SoftAP gone.
 * The STA address is not always reachable from the LAN (see CLAUDE.md on both
 * interfaces landing in one subnet), so this is a floor, not a promise. */
static bool reachable_without_ap(void)
{
    return net_eth_state() == NET_ETH_READY || net_wifi_state() == NET_WIFI_STA;
}

static int cmd_wifi(int argc, char **argv)
{
    if (argc == 1) return cmd_net(argc, argv);

    if (strcmp(argv[1], "sta") == 0) {
        if (argc == 3 && strcmp(argv[2], "clear") == 0) {
            esp_err_t e = net_wifi_set_sta("", "");
            printf("upstream credentials cleared (%s)\n", esp_err_to_name(e));
            return e;
        }
        if (argc == 3 && onoff(argv[2]) >= 0) {
            bool on = onoff(argv[2]);
            esp_err_t e = net_wifi_set_sta_enabled(on);
            printf("sta %s (%s); credentials kept\n",
                   on ? "enabled, joining now" : "disabled, association dropped",
                   esp_err_to_name(e));
            return e;
        }
        if (argc != 4) {
            printf("usage: wifi sta <ssid> <password>\n"
                   "       wifi sta on | off | clear\n");
            return ESP_ERR_INVALID_ARG;
        }
        /* Stored in NVS, never in the image -- sdkconfig is tracked and this
         * repo's remote is a public fork. Note the password is echoed as it is
         * typed and is visible to anyone watching the console. */
        esp_err_t e = net_wifi_set_sta(argv[2], argv[3]);
        printf("upstream set to \"%s\" (%s); join retried in the background\n",
               argv[2], esp_err_to_name(e));
        return e;
    }

    if (strcmp(argv[1], "ap") == 0 && argc >= 3 && onoff(argv[2]) >= 0) {
        bool on    = onoff(argv[2]);
        bool force = argc > 3 && strcmp(argv[3], "force") == 0;

        /* The switch is persisted, so getting this wrong on a headless board
         * costs a serial cable rather than a reboot. */
        if (!on && !force && !reachable_without_ap()) {
            printf("refusing: nothing else is up (eth %s, wifi %s), so turning\n"
                   "the SoftAP off would leave this board reachable only over\n"
                   "the serial port -- and the setting survives a reboot.\n"
                   "Use 'wifi ap off force' if that is what you want.\n",
                   eth_state_str(net_eth_state()),
                   wifi_state_str(net_wifi_state()));
            return ESP_ERR_INVALID_STATE;
        }

        esp_err_t e = net_wifi_set_ap_enabled(on);
        if (e != ESP_OK) {
            printf("ap %s failed: %s\n", on ? "on" : "off", esp_err_to_name(e));
            return e;
        }
        printf("SoftAP %s\n", on ? "on" : "off, associated stations dropped");
        return ESP_OK;
    }

    printf("usage: wifi                       -- status\n"
           "       wifi sta <ssid> <password> -- set upstream credentials\n"
           "       wifi sta on | off          -- STA half; credentials kept\n"
           "       wifi sta clear             -- forget them\n"
           "       wifi ap  on | off          -- SoftAP half\n");
    return ESP_ERR_INVALID_ARG;
}

/* ── log ──────────────────────────────────────────────────────────────────── */
static int cmd_log(int argc, char **argv)
{
    /* Two different logs share this command because they are one thing to the
     * user: esp_log's per-tag levels, and the event ring behind the LOG panel,
     * which is otherwise invisible whenever the display is not in front. */
    static const char *echo_names[] = { "off", "sys", "brief", "all" };

    if (argc == 1 || strcmp(argv[1], "tail") == 0) {
        adsb_log_dump(argc >= 3 ? atoi(argv[2]) : 0);
        printf("(echo %s)\n", echo_names[adsb_log_echo_get()]);
        return ESP_OK;
    }

    if (strcmp(argv[1], "echo") == 0) {
        for (int i = 0; argc == 3 && i < 4; i++) {
            if (strcmp(argv[2], echo_names[i]) != 0) continue;
            adsb_log_echo_set(i);
            printf("event echo %s\n", echo_names[i]);
            return ESP_OK;
        }
        printf("usage: log echo <off|sys|brief|all>\n"
               "  sys   -- the board only: USB, WiFi, Ethernet, OTA (default)\n"
               "  brief -- plus first and lost contacts\n"
               "  all   -- plus every decode; unusable at a prompt with traffic\n");
        return ESP_ERR_INVALID_ARG;
    }

    if (argc != 3) {
        printf("usage: log                      -- recent events\n"
               "       log tail [n]             -- the last n of them\n"
               "       log echo <off|sys|brief|all> -- which reach this prompt\n"
               "       log <tag|*> <level>      -- none|error|warn|info|debug|verbose\n");
        return ESP_ERR_INVALID_ARG;
    }

    static const char *names[] = { "none", "error", "warn", "info", "debug", "verbose" };
    for (int i = 0; i < 6; i++) {
        if (strcmp(argv[2], names[i]) != 0) continue;
        esp_log_level_set(argv[1], (esp_log_level_t)i);
        printf("%s -> %s\n", argv[1], names[i]);
        return ESP_OK;
    }
    printf("unknown level \"%s\"\n", argv[2]);
    return ESP_ERR_INVALID_ARG;
}

/* ── sys ──────────────────────────────────────────────────────────────────── */
static int cmd_sys(int argc, char **argv)
{
    const esp_app_desc_t *app = esp_app_get_description();
    int64_t up = esp_timer_get_time() / 1000000;

    printf("app       %s  %s  %s %s\n", app->project_name, app->version, app->date, app->time);
    printf("idf       %s\n", app->idf_ver);
    printf("uptime    %lldd %02lld:%02lld:%02lld\n",
           up / 86400, (up % 86400) / 3600, (up % 3600) / 60, up % 60);
    printf("reset     %d\n", (int)esp_reset_reason());
    return ESP_OK;
}

/* ── ota ──────────────────────────────────────────────────────────────────── */
static const char *ota_state_name(esp_ota_img_states_t st)
{
    switch (st) {
    case ESP_OTA_IMG_NEW:            return "new";
    case ESP_OTA_IMG_PENDING_VERIFY: return "pending-verify";
    case ESP_OTA_IMG_VALID:          return "valid";
    case ESP_OTA_IMG_INVALID:        return "invalid";
    case ESP_OTA_IMG_ABORTED:        return "aborted";
    default:                         return "undefined";
    }
}

static int cmd_ota(int argc, char **argv)
{
    const esp_partition_t *run   = esp_ota_get_running_partition();
    const esp_partition_t *other = esp_ota_get_next_update_partition(NULL);

    if (argc >= 2 && strcmp(argv[1], "rollback") == 0) {
        esp_ota_img_states_t st = ESP_OTA_IMG_UNDEFINED;
        esp_ota_get_state_partition(run, &st);

        /* Still unconfirmed: this is exactly the rollback path
         * BOOTLOADER_APP_ROLLBACK_ENABLE exists for, done on demand instead
         * of waiting out a crash. Does not return on success. */
        if (st == ESP_OTA_IMG_PENDING_VERIFY) {
            printf("pending-verify -- rolling back to the previous slot now\n");
            fflush(stdout);
            esp_ota_mark_app_invalid_rollback_and_reboot();
            printf("rollback failed: no valid previous image to boot\n");
            return ESP_FAIL;
        }

        /* Already confirmed valid: "rollback" here just means "boot the
         * other slot next time", for reverting a bad OTA push after the
         * 20s auto-confirm window in ota.c has already closed. */
        if (!other) {
            printf("no other OTA slot to roll back to\n");
            return ESP_ERR_INVALID_STATE;
        }
        esp_err_t err = esp_ota_set_boot_partition(other);
        if (err != ESP_OK) {
            printf("esp_ota_set_boot_partition: %s\n", esp_err_to_name(err));
            return ESP_FAIL;
        }
        printf("next boot -> %s, restarting...\n", other->label);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
        return ESP_OK;      /* not reached */
    }

    esp_ota_img_states_t rst = ESP_OTA_IMG_UNDEFINED, ost = ESP_OTA_IMG_UNDEFINED;
    esp_ota_get_state_partition(run, &rst);
    if (other) esp_ota_get_state_partition(other, &ost);

    printf("running   %-8s %s\n", run->label, ota_state_name(rst));
    if (other) printf("other     %-8s %s\n", other->label, ota_state_name(ost));
    printf("app       %s\n", esp_app_get_description()->version);
    printf("push a new image: curl -H \"Expect:\" --data-binary @build/<app>.bin"
           " http://<board-ip>/ota\n");
    return ESP_OK;
}

/* ── restart / tui / exit ─────────────────────────────────────────────────── */
static int cmd_restart(int argc, char **argv)
{
    printf("restarting...\n");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
    return ESP_OK;      /* not reached */
}

static int cmd_tui(int argc, char **argv)
{
    int cols = (s_owner == OWNER_SSH) ? net_ssh_pty_cols() : 0;
    if (cols > 0 && cols < adsb_tui_cols())
        printf("note: the display is %d columns wide and this terminal is %d --"
               " widen it or the frame wraps\n", adsb_tui_cols(), cols);

    printf("entering the display -- 'q', ':' or Ctrl-C returns to this prompt\n");
    fflush(stdout);

    /* Clear before attaching, never after: the draw task starts the moment a
     * sink appears, and a frame begun first would be half-wiped by the clear
     * that was meant to precede it. */
    adsb_tui_resume();

    if (s_owner == OWNER_SSH) {
        /* Ownership deliberately stays: net_ssh.c still has stdout pointed at
         * this session, so letting the serial shell settle() in behind us
         * would send its banner and its prompt down the wire. The display
         * needs no line buffer anyway -- only the hotkey branch in
         * shell_remote_byte(). */
        if (!tui_attach(ssh_sink, NULL)) goto full;
        s_tui_ssh = true;
    } else {
        /* The serial side is the opposite case: releasing the console rather
         * than holding it is what stops run_line() printing a prompt over the
         * first frame, and lets an SSH session in while the display is up. */
        if (!tui_attach(uart_sink, NULL)) goto full;
        s_tui_fg = true;
        s_owner  = OWNER_NONE;
    }

    return ESP_OK;

full:
    /* Unreachable with one slot per transport, but the alternative to saying so
     * is a display that announced itself and then never paints. */
    printf("no free display slot\n");
    return ESP_ERR_NO_MEM;
}

static int cmd_exit(int argc, char **argv)
{
    if (s_owner == OWNER_SSH) {
        /* Not a direct s_owner write: net_ssh.c still has stdout pointed at
         * the session, and handing the console back here would let the serial
         * shell print its banner into the dying SSH channel. The transport
         * releases it from shell_remote_close(), after restoring stdout. */
        s_remote_quit = true;
        return ESP_OK;
    }

    printf("the serial console is the top level -- 'tui' opens the display,\n"
           "'restart' reboots the board\n");
    return ESP_OK;
}

/* ═════════════════════════════════════════════════════════════════════════════
 * Read half
 * ═══════════════════════════════════════════════════════════════════════════ */

static void run_line(void)
{
    s_line[s_len] = '\0';
    printf("\n");

    if (s_len == 0) return;

    int       ret = 0;
    s_running     = true;
    esp_err_t err = esp_console_run(s_line, &ret);
    s_running     = false;

    if (err == ESP_ERR_NOT_FOUND)      printf("unknown command: %s (try 'help')\n", s_line);
    else if (err == ESP_ERR_INVALID_ARG) { /* empty line -- nothing to say */ }
    else if (err != ESP_OK)            printf("console error: %s\n", esp_err_to_name(err));
    else if (ret != ESP_OK && ret != 0) printf("-> %d (%s)\n", ret, esp_err_to_name(ret));
}

static void handle_byte(uint8_t b)
{
    switch (b) {
        case '\r':
        case '\n':
            run_line();
            s_len = 0;
            /* A command may have opened the display ('tui' -- which hands the
             * console on over serial and keeps it over SSH) or asked to close
             * the session ('exit'). None of them wants a prompt after it. */
            if (s_owner != OWNER_NONE && !s_remote_quit && !s_tui_ssh) printf(PROMPT);
            break;

        case 0x7f:          /* DEL */
        case '\b':
            if (s_len > 0) { s_len--; printf("\b \b"); }
            break;

        case 0x03:          /* Ctrl-C -- abandon the line */
            printf("^C\n" PROMPT);
            s_len = 0;
            break;

        case 0x15:          /* Ctrl-U -- clear the line */
            while (s_len > 0) { s_len--; printf("\b \b"); }
            break;

        case 0x04:          /* Ctrl-D on an empty line */
            if (s_len != 0) break;
            if (s_owner == OWNER_SSH) { s_remote_quit = true; printf("\n"); break; }
            /* The serial console has nowhere to exit to; say so rather than
             * leaving a prompt that looks like it ignored the key. */
            printf("\n(top level -- 'tui' opens the display)\n" PROMPT);
            break;

        default:
            if (b < 0x20 || b > 0x7e) break;            /* ignore other control/8-bit */
            if (s_len >= SHELL_LINE_MAX - 1) break;     /* full: drop, no bell */
            s_line[s_len++] = (char)b;
            printf("%c", (char)b);
            break;
    }
    fflush(stdout);
}

/* ═════════════════════════════════════════════════════════════════════════════
 * Console task -- UART0's only reader
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Takes the console back whenever nothing else holds it and the TUI is not in
 * front: after boot, after the TUI is left, and after an SSH session ends.
 * Called with s_lock held, twice per pass -- once before the keystroke is
 * dispatched and once after, so a key that leaves the TUI gets its prompt in
 * the same pass rather than up to a poll later. */
static void settle(void)
{
    if (s_owner != OWNER_NONE || !s_ready || s_tui_fg) return;

    s_owner = OWNER_UART;
    s_len   = 0;

    if (s_clear_pending) { printf(CLS); s_clear_pending = false; }
    printf("\nADS-B console -- 'help' lists commands, 'tui' opens the display.\n"
           PROMPT);
    fflush(stdout);
}

/* True when the key means "leave the display"; anything else has already been
 * handed to it. The leave keys are decided here rather than in class_driver.c
 * so that ownership stays in one file, and both transports route through this
 * so they cannot drift apart. */
static bool tui_hotkey(uint8_t b)
{
    switch (b) {
        case 'q': case 'Q':
        case ':':
        case 0x03:      /* Ctrl-C */
        case 0x04:      /* Ctrl-D */
            return true;
        default:
            adsb_tui_key(b);
            return false;
    }
}

static void leave_tui(void)
{
    /* An SSH session holds the console, so there is no prompt to come back to.
     * Leaving anyway would strand the serial port on a blank screen that also
     * takes no keys -- the console task drops them while it owns nothing --
     * until that session happens to end. Stay in the display instead, and say
     * why through the LOG panel, which is the one thing on this screen the
     * user can still see. */
    if (s_owner != OWNER_NONE) {
        ui_log(4, "CONSOLE  held by an SSH session -- staying in the display");
        return;
    }

    tui_detach(uart_sink, NULL);
    s_tui_fg        = false;
    s_clear_pending = true;     /* settle() wipes the frame the TUI left */
    adsb_tui_hold();            /* but let the run in flight finish first */
}

/* The SSH half of leave_tui(). It prints rather than leaving that to settle():
 * the session never gave the console up, so there is nothing for settle() to
 * take back -- and it must not, with stdout still pointed here. */
static void leave_tui_ssh(void)
{
    tui_detach(ssh_sink, NULL);
    s_tui_ssh = false;
    adsb_tui_hold();            /* let the run in flight finish first */
    s_len = 0;
    printf(CLS PROMPT);
    fflush(stdout);
}

static void console_task(void *arg)
{
    for (;;) {
        uint8_t b;
        int     n = uart_read_bytes(CONSOLE_UART, &b, 1, pdMS_TO_TICKS(100));

        xSemaphoreTake(s_lock, portMAX_DELAY);

        settle();

        if (n > 0) {
            if (s_owner == OWNER_UART) {
                handle_byte(b);
            } else if (s_tui_fg) {
                if (tui_hotkey(b)) leave_tui();
            }
            /* else: an SSH session holds the console and the display is not up
             * -- there is nothing safe to do with the byte, so drop it. */
        }

        settle();

        xSemaphoreGive(s_lock);
    }
}

/* ═════════════════════════════════════════════════════════════════════════════
 * Remote sessions
 *
 * No task and no queue of its own: the transport calls these from the task it
 * already has. See the call-order contract in shell.h.
 * ═══════════════════════════════════════════════════════════════════════════ */

bool shell_remote_claim(void)
{
    if (!s_lock) return false;                  /* shell_init() never ran */
    if (s_owner == OWNER_SSH) return false;     /* one remote session at a time */

    /* Waiting on the lock is what makes the preemption safe: the console task
     * only ever prints with it held, so once it is ours the serial shell is
     * guaranteed not to be mid-printf when stdout is swapped out from under
     * it. A timeout means a command is still running; refuse rather than
     * interleave two writers. */
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(SHELL_CLAIM_MS)) != pdTRUE) return false;

    if (s_owner == OWNER_SSH) { xSemaphoreGive(s_lock); return false; }

    if (s_owner == OWNER_UART)
        uart_note("\r\n[console taken by an SSH session]\r\n");

    s_owner       = OWNER_SSH;
    s_remote_quit = false;
    s_len         = 0;
    xSemaphoreGive(s_lock);
    return true;
}

void shell_remote_open(void)
{
    printf("ADS-B console. 'help' lists commands, 'exit' closes the session.\n\n"
           PROMPT);
    fflush(stdout);
}

bool shell_remote_byte(uint8_t byte)
{
    if (s_owner != OWNER_SSH) return false;

    if (s_tui_ssh) {
        if (tui_hotkey(byte)) leave_tui_ssh();
        return true;            /* the display has no way to end the session */
    }

    handle_byte(byte);
    return !s_remote_quit;
}

void shell_remote_close(void)
{
    if (s_owner != OWNER_SSH) return;

    /* Dropped without printing anything: stdout has already been restored to
     * the serial console by now, and the ring this frame would go into is
     * about to stop being drained. */
    if (s_tui_ssh) {
        tui_detach(ssh_sink, NULL);
        s_tui_ssh = false;
        adsb_tui_hold();
    }

    s_owner       = OWNER_NONE;
    s_remote_quit = false;
    /* settle() reprints the serial prompt on the console task's next pass. */
}

/* ═════════════════════════════════════════════════════════════════════════════
 * Setup
 * ═══════════════════════════════════════════════════════════════════════════ */

static void reg(const char *cmd, const char *help, esp_console_cmd_func_t fn)
{
    const esp_console_cmd_t c = { .command = cmd, .help = help, .hint = NULL, .func = fn };
    ESP_ERROR_CHECK(esp_console_cmd_register(&c));
}

/* The console UART, set up here rather than in the receiver: this is the file
 * that reads it, and everything printed after this call -- including boot
 * logging from the rest of app_main -- benefits from the driver's buffered,
 * interrupt-driven TX. Without it stdout goes through uart_vfs's default
 * tx_func, which busy-spins on the TX FIFO one byte at a time and never
 * yields, so a heavy repaint can starve IDLE long enough to trip the task
 * watchdog. The baud MUST stay at the configured console rate: this is the
 * same UART stdout goes out on. */
static void console_uart_init(void)
{
    const uart_config_t cfg = {
        .baud_rate  = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(CONSOLE_UART, &cfg);
    /* TX ring sized for the TUI's ~13 KB repaint arriving in fb_flush()-sized
     * chunks; RX only ever holds a typed line. */
    uart_driver_install(CONSOLE_UART, 256, 4096, 0, NULL, 0);
    uart_vfs_dev_use_driver(CONSOLE_UART);
}

void shell_init(void)
{
    console_uart_init();

    esp_console_config_t cfg = ESP_CONSOLE_CONFIG_DEFAULT();
    cfg.max_cmdline_length = SHELL_LINE_MAX;
    cfg.max_cmdline_args   = 8;
    ESP_ERROR_CHECK(esp_console_init(&cfg));
    ESP_ERROR_CHECK(esp_console_register_help_command());

    reg("free",    "internal and PSRAM heap, free / minimum-ever / largest block", cmd_free);
    reg("tasks",   "per-task core, priority, CPU% over 1 s, and stack headroom",   cmd_tasks);
    reg("net",     "interface addresses and per-feed client counts",               cmd_net);
    reg("wifi",    "WiFi status, upstream credentials, or switch either half off",  cmd_wifi);
    reg("log",     "recent receiver events, echo control, esp_log levels",         cmd_log);
    reg("sys",     "firmware build, IDF version, uptime, reset reason",            cmd_sys);
    reg("ota",     "OTA slot/version status, or 'ota rollback' to revert",         cmd_ota);
    reg("tui",     "open the radar display on this console; 'q' returns",          cmd_tui);
    reg("restart", "reboot the board",                                             cmd_restart);
    reg("exit",    "close this session (SSH); the serial console is top level",    cmd_exit);
    reg("quit",    "alias for exit",                                               cmd_exit);

    adsb_register_shell_cmds();     /* ac / usb / sdr -- see class_driver.c */

    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) { ESP_LOGE("shell", "mutex alloc failed"); return; }

    /* core0 at priority 3, matching web_config.c's httpd: a command may block
     * for seconds, and core1 is the demod loop. Never raise this to
     * adsb_rx_task's 5 or above. */
    xTaskCreatePinnedToCore(console_task, "shell", 6144, NULL, 3, &s_console_tsk, 0);

    /* Last, so nothing above can log through a half-built console. */
    esp_log_set_vprintf(log_vprintf);
}

void shell_console_start(void)
{
    /* Boot is over, so stop mirroring the sky to the console: with an antenna
     * connected those arrive several times a second and are on screen in the
     * display anyway. The board keeps talking -- USB, WiFi, Ethernet, OTA --
     * which is what a prompt is for. `log echo brief` adds first and lost
     * contacts back, `log echo all` the rest, `log tail` shows it regardless. */
    adsb_log_echo_set(ADSB_ECHO_SYS);

    s_ready = true;     /* the console task's settle() prints the first prompt */
}
