/* ═════════════════════════════════════════════════════════════════════════════
 * shell.c -- esp_console REPL over the console UART the TUI already owns.
 *
 * See shell.h for why esp_console_new_repl_uart() is not used and why input is
 * queued to another core instead of being executed where it is read.
 *
 * The Read half here is deliberately small: raw byte handling with backspace,
 * Ctrl-C and Ctrl-U, and no linenoise. linenoise wants to own stdin and block
 * in it, which is exactly the conflict being avoided -- history and Tab
 * completion are not worth a second reader on the one console UART.
 * ═══════════════════════════════════════════════════════════════════════════ */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_app_desc.h"
#include "esp_idf_version.h"

#include "shell.h"
#include "net_eth.h"
#include "net_wifi.h"
#include "feed_avr.h"
#include "feed_beast.h"
#include "feed_json.h"

#if CONFIG_SPIRAM
#include "esp_psram.h"
#endif

#define SHELL_LINE_MAX   160
#define SHELL_QUEUE_LEN  256
#define PROMPT           "p4> "

/* Which transport owns the console. There is one line buffer, one command
 * context and -- under picolibc -- one global stdout, so ownership is
 * exclusive; see net_ssh.h. */
typedef enum { OWNER_NONE = 0, OWNER_UART, OWNER_SSH } owner_t;

static volatile owner_t s_owner;
static QueueHandle_t    s_q;
static char             s_line[SHELL_LINE_MAX];
static int              s_len;

/* Only the UART session suspends the TUI. A remote session writes to a socket,
 * so the two never touch the same UART and the radar can keep running. */
bool shell_active(void) { return s_owner == OWNER_UART; }

void shell_feed(uint8_t byte)
{
    if (s_owner == OWNER_UART && s_q) xQueueSend(s_q, &byte, 0);
}

void shell_enter(void)
{
    if (!s_q) return;                     /* shell_init() never ran */
    if (s_owner != OWNER_NONE) return;    /* a remote session has the console */
    xQueueReset(s_q);
    s_len   = 0;
    s_owner = OWNER_UART;     /* the task prints the banner; see shell_task() */
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
    /* The TUI's cached stats (s_tstat) are refreshed from tui_draw(), which is
     * suspended while the shell has the console -- so they would be stale here.
     * Sample fresh, twice, and take the delta over the gap. */
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
    switch (s) {
        case NET_WIFI_OFF:  return "off";
        case NET_WIFI_INIT: return "bringing up";
        case NET_WIFI_AP:   return "AP only";
        case NET_WIFI_STA:  return "AP + STA joined";
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
    printf("  ap clients %d   sta ssid %s\n",
           net_wifi_ap_clients(), ssid[0] ? ssid : "(unset)");

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

/* ── wifi ─────────────────────────────────────────────────────────────────── */
static int cmd_wifi(int argc, char **argv)
{
    if (argc == 1) return cmd_net(argc, argv);

    if (strcmp(argv[1], "sta") == 0) {
        if (argc == 3 && strcmp(argv[2], "clear") == 0) {
            esp_err_t e = net_wifi_set_sta("", "");
            printf("upstream credentials cleared (%s)\n", esp_err_to_name(e));
            return e;
        }
        if (argc != 4) {
            printf("usage: wifi sta <ssid> <password>\n"
                   "       wifi sta clear\n");
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

    printf("usage: wifi                       -- status\n"
           "       wifi sta <ssid> <password> -- set upstream credentials\n"
           "       wifi sta clear             -- forget them\n");
    return ESP_ERR_INVALID_ARG;
}

/* ── log ──────────────────────────────────────────────────────────────────── */
static int cmd_log(int argc, char **argv)
{
    if (argc != 3) {
        printf("usage: log <tag|*> <none|error|warn|info|debug|verbose>\n");
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

/* ── restart / exit ───────────────────────────────────────────────────────── */
static int cmd_restart(int argc, char **argv)
{
    printf("restarting...\n");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
    return ESP_OK;      /* not reached */
}

static int cmd_exit(int argc, char **argv)
{
    s_owner = OWNER_NONE;   /* the session loop sees this and hands back */
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
    esp_err_t err = esp_console_run(s_line, &ret);

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
            if (s_owner != OWNER_NONE) printf(PROMPT);
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

        case 0x04:          /* Ctrl-D on an empty line -- leave the shell */
            if (s_len == 0) { s_owner = OWNER_NONE; break; }
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

static void shell_task(void *arg)
{
    while (true) {
        if (s_owner != OWNER_UART) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

        printf("\033[2J\033[H"                        /* the TUI left a full screen */
               "ADS-B console. 'help' lists commands, 'exit' returns to the TUI.\n\n"
               PROMPT);
        fflush(stdout);

        while (s_owner == OWNER_UART) {
            uint8_t b;
            /* Timed rather than blocking so 'exit' (which runs on this task,
             * inside handle_byte) is not what has to wake the loop. */
            if (xQueueReceive(s_q, &b, pdMS_TO_TICKS(200)) == pdTRUE) handle_byte(b);
        }

        printf("\nreturning to the TUI\n");
        fflush(stdout);
        adsb_tui_resume();
    }
}

/* ═════════════════════════════════════════════════════════════════════════════
 * Remote sessions
 *
 * No task and no queue of its own: the transport calls these from the task it
 * already has. See the comment in shell.h.
 * ═══════════════════════════════════════════════════════════════════════════ */

bool shell_remote_open(void)
{
    if (!s_q) return false;                  /* shell_init() never ran */
    if (s_owner != OWNER_NONE) return false; /* console already claimed */
    s_len   = 0;
    s_owner = OWNER_SSH;

    printf("ADS-B console. 'help' lists commands, 'exit' closes the session.\n\n"
           PROMPT);
    fflush(stdout);
    return true;
}

bool shell_remote_byte(uint8_t byte)
{
    if (s_owner != OWNER_SSH) return false;
    handle_byte(byte);
    return s_owner == OWNER_SSH;
}

void shell_remote_close(void)
{
    if (s_owner == OWNER_SSH) s_owner = OWNER_NONE;
}

/* ═════════════════════════════════════════════════════════════════════════════
 * Setup
 * ═══════════════════════════════════════════════════════════════════════════ */

static void reg(const char *cmd, const char *help, esp_console_cmd_func_t fn)
{
    const esp_console_cmd_t c = { .command = cmd, .help = help, .hint = NULL, .func = fn };
    ESP_ERROR_CHECK(esp_console_cmd_register(&c));
}

void shell_init(void)
{
    esp_console_config_t cfg = ESP_CONSOLE_CONFIG_DEFAULT();
    cfg.max_cmdline_length = SHELL_LINE_MAX;
    cfg.max_cmdline_args   = 8;
    ESP_ERROR_CHECK(esp_console_init(&cfg));
    ESP_ERROR_CHECK(esp_console_register_help_command());

    reg("free",    "internal and PSRAM heap, free / minimum-ever / largest block", cmd_free);
    reg("tasks",   "per-task core, priority, CPU% over 1 s, and stack headroom",   cmd_tasks);
    reg("net",     "interface addresses and per-feed client counts",               cmd_net);
    reg("wifi",    "WiFi status, or set the upstream credentials in NVS",          cmd_wifi);
    reg("log",     "set a log tag's level at runtime",                             cmd_log);
    reg("sys",     "firmware build, IDF version, uptime, reset reason",            cmd_sys);
    reg("restart", "reboot the board",                                             cmd_restart);
    reg("exit",    "leave the console and return to the TUI",                      cmd_exit);
    reg("quit",    "alias for exit",                                               cmd_exit);

    adsb_register_shell_cmds();     /* ac / usb / sdr -- see class_driver.c */

    s_q = xQueueCreate(SHELL_QUEUE_LEN, sizeof(uint8_t));
    if (!s_q) { ESP_LOGE("shell", "queue alloc failed"); return; }

    /* core0 at priority 3, matching web_config.c's httpd: a command may block
     * for seconds, and core1 is the demod loop. Never raise this to
     * adsb_rx_task's 5 or above. */
    xTaskCreatePinnedToCore(shell_task, "shell", 6144, NULL, 3, NULL, 0);
}
