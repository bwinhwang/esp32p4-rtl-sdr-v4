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
#define PROMPT           "p4> "
#define CLS              "\033[2J\033[H"
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
static volatile bool     s_ready;         /* shell_console_start() has run    */
static volatile bool     s_remote_quit;   /* the SSH user typed exit/Ctrl-D   */
static bool              s_clear_pending; /* the TUI left a full screen       */

/* Held by the console task for as long as it is touching the line buffer or
 * printing, and taken by shell_remote_claim() so an arriving SSH session can
 * never land between the task's ownership check and its printf(). The task
 * does not hold it while blocked in uart_read_bytes(), so a claim is quick in
 * the normal case. */
static SemaphoreHandle_t s_lock;

static char              s_line[SHELL_LINE_MAX];
static int               s_len;

bool shell_tui_foreground(void) { return s_tui_fg; }

/* Writes straight to the UART, bypassing stdout. The console task's own
 * printf() is unusable in exactly the moments this is needed -- while an SSH
 * session holds stdout -- and the TUI's fb_flush() already writes the port
 * this way, so there is no VFS to do the \n -> \r\n translation here. */
static void uart_note(const char *s)
{
    uart_write_bytes(CONSOLE_UART, s, strlen(s));
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
    /* Two different logs share this command because they are one thing to the
     * user: esp_log's per-tag levels, and the TUI's own event ring (the LOG
     * panel), which is otherwise invisible whenever the TUI is not in front. */
    static const char *echo_names[] = { "off", "brief", "all" };

    if (argc == 1 || strcmp(argv[1], "tail") == 0) {
        adsb_log_dump(argc >= 3 ? atoi(argv[2]) : 0);
        printf("(echo %s)\n", echo_names[adsb_log_echo_get()]);
        return ESP_OK;
    }

    if (strcmp(argv[1], "echo") == 0) {
        for (int i = 0; argc == 3 && i < 3; i++) {
            if (strcmp(argv[2], echo_names[i]) != 0) continue;
            adsb_log_echo_set(i);
            printf("receiver log echo %s\n", echo_names[i]);
            return ESP_OK;
        }
        printf("usage: log echo <off|brief|all>\n"
               "  brief -- first contacts, bring-up and failures (default)\n"
               "  all   -- every decode too; unusable at a prompt with traffic\n");
        return ESP_ERR_INVALID_ARG;
    }

    if (argc != 3) {
        printf("usage: log                    -- recent receiver events\n"
               "       log tail [n]           -- the last n of them\n"
               "       log echo <off|brief|all> -- mirror new ones to this console\n"
               "       log <tag|*> <level>    -- none|error|warn|info|debug|verbose\n");
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
    /* The TUI paints UART0 directly (fb_flush() -> uart_write_bytes), so it
     * has nowhere to go in a session that is not the serial one. */
    if (s_owner != OWNER_UART) {
        printf("the display paints on the serial console; run 'tui' from there\n");
        return ESP_ERR_INVALID_STATE;
    }

    printf("entering the display -- 'q', ':' or Ctrl-C returns to this prompt\n");
    fflush(stdout);

    /* Releasing the console rather than holding it is what stops run_line()
     * printing a prompt over the first frame, and lets an SSH session in while
     * the display is up. */
    s_tui_fg = true;
    s_owner  = OWNER_NONE;
    adsb_tui_resume();          /* clears the screen, forces a frame now */
    return ESP_OK;
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
            /* A command may have handed the console on ('tui') or asked to
             * close the session ('exit') -- neither wants a prompt after it. */
            if (s_owner != OWNER_NONE && !s_remote_quit) printf(PROMPT);
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

static void leave_tui(void)
{
    s_tui_fg        = false;
    s_clear_pending = true;     /* settle() wipes the frame the TUI left */
    adsb_tui_hold();            /* but let the frame in flight finish first */

    if (s_owner == OWNER_NONE) return;

    /* An SSH session took the console while the display was up, so there is no
     * prompt to come back to yet. Say so on the wire rather than through
     * stdout, which is pointed at that session. */
    uart_note("\r\n[console held by an SSH session -- the serial shell returns "
              "when it ends]\r\n");
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
                /* Display hotkeys. The keys that leave it are handled here so
                 * that ownership stays in one file; the rest go to the TUI. */
                switch (b) {
                    case 'q': case 'Q':
                    case ':':
                    case 0x03:      /* Ctrl-C */
                    case 0x04:      /* Ctrl-D */
                        leave_tui();
                        break;
                    default:
                        adsb_tui_key(b);
                        break;
                }
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
    handle_byte(byte);
    return !s_remote_quit;
}

void shell_remote_close(void)
{
    if (s_owner != OWNER_SSH) return;
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
    reg("wifi",    "WiFi status, or set the upstream credentials in NVS",          cmd_wifi);
    reg("log",     "recent receiver events, echo control, esp_log levels",         cmd_log);
    reg("sys",     "firmware build, IDF version, uptime, reset reason",            cmd_sys);
    reg("tui",     "open the radar display on the serial console",                 cmd_tui);
    reg("restart", "reboot the board",                                             cmd_restart);
    reg("exit",    "close this session (SSH); the serial console is top level",    cmd_exit);
    reg("quit",    "alias for exit",                                               cmd_exit);

    adsb_register_shell_cmds();     /* ac / usb / sdr -- see class_driver.c */

    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) { ESP_LOGE("shell", "mutex alloc failed"); return; }

    /* core0 at priority 3, matching web_config.c's httpd: a command may block
     * for seconds, and core1 is the demod loop. Never raise this to
     * adsb_rx_task's 5 or above. */
    xTaskCreatePinnedToCore(console_task, "shell", 6144, NULL, 3, NULL, 0);
}

void shell_console_start(void)
{
    /* Boot is over, so stop mirroring every decode to the console: with an
     * antenna connected they arrive several times a second and the prompt
     * becomes unusable. First contacts, bring-up and failures still come
     * through; `log tail` shows the rest, `log echo all` puts it all back. */
    adsb_log_echo_set(ADSB_ECHO_BRIEF);

    s_ready = true;     /* the console task's settle() prints the first prompt */
}
