/*
 * WiFi on the ESP32-C6 co-processor, reached over 4-bit SDIO by esp_hosted.
 * The P4 has no radio of its own; esp_wifi_remote transparently remotes the
 * ordinary esp_wifi_* API below to the C6, which is why there is no
 * esp_hosted_init() call here -- esp_wifi_init() brings the transport up.
 *
 * Runs permanently in APSTA so both deployment scenarios are served without
 * ever changing mode at runtime:
 *   - the SoftAP always beacons, for the in-car case where a phone talks
 *     straight to the board;
 *   - the STA side reconnects in the background, for the fixed-at-home case
 *     where the feeds should reach a readsb box across the house network.
 * Runtime STA<->AP switching is deliberately not done: esp_hosted-mcu has
 * known SDIO lockups and a netif_add assert on WiFi re-init, and nothing here
 * needs the radio to change mode. Leaving both interfaces up costs one extra
 * netif and the AP's beacons.
 *
 * The three feeds bind INADDR_ANY, so they are reachable over whichever of
 * Ethernet / SoftAP / STA is up with no per-interface code at all.
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "shell.h"       /* sys_log() */
#include "net_wifi.h"

#define AP_SSID     CONFIG_ADSB_WIFI_AP_SSID
#define AP_PASS     CONFIG_ADSB_WIFI_AP_PASS
#define AP_CHANNEL  CONFIG_ADSB_WIFI_AP_CHANNEL
#define AP_MAX_CONN CONFIG_ADSB_WIFI_AP_MAX_CONN

/* Where the upstream credentials live. Deliberately NOT a Kconfig default:
 * sdkconfig is tracked in a repo whose remote is a public fork, so a password
 * put there would be published. The config page writes these at runtime. */
#define CRED_NS     "netcfg"
#define CRED_SSID   "sta_ssid"
#define CRED_PASS   "sta_pass"

/* Backoff between STA join attempts. The AP half is what matters in the car,
 * so a missing home network must stay cheap: retrying every few seconds would
 * put a scan on the SDIO link forever for no benefit. */
#define STA_RETRY_MS  30000

/* esp_netif's ESP_NETIF_DEFAULT_ETH() uses 50 and DEFAULT_WIFI_STA() uses 100,
 * so out of the box a WiFi lease would steal the default route from the cable.
 * Wired is the better path whenever it exists, so the STA netif is demoted
 * below Ethernet instead. */
#define STA_ROUTE_PRIO  40

static const char *TAG = "wifi";

static volatile net_wifi_state_t s_state;
static volatile int      s_ap_clients;
static volatile uint32_t s_sta_ip4;      /* raw, so a reader can't catch a
                                          * half-written string */
static volatile bool     s_sta_want;     /* an upstream SSID is configured */
static volatile int      s_join_fail = -1;   /* last reported join failure   */
static esp_netif_t      *s_sta_netif;
static TaskHandle_t      s_task;         /* woken to retry a join at once */

/* Guards the two buffers below, which the config page writes from the HTTP
 * task while net_wifi_task and the status page read them. */
static SemaphoreHandle_t s_cred_lock;
static char              s_ssid[33];
static char              s_pass[65];

net_wifi_state_t net_wifi_state(void)  { return s_state; }
int              net_wifi_ap_clients(void) { return s_ap_clients; }

void net_wifi_sta_ip_str(char *dst, size_t n)
{
    uint32_t a = s_sta_ip4;
    if (a == 0) {
        snprintf(dst, n, "---");
        return;
    }
    snprintf(dst, n, "%u.%u.%u.%u",
             (unsigned)(a & 0xff),         (unsigned)((a >> 8) & 0xff),
             (unsigned)((a >> 16) & 0xff), (unsigned)((a >> 24) & 0xff));
}

void net_wifi_sta_ssid(char *dst, size_t n)
{
    xSemaphoreTake(s_cred_lock, portMAX_DELAY);
    strlcpy(dst, s_ssid, n);
    xSemaphoreGive(s_cred_lock);
}

/* Pushes whatever is in s_ssid/s_pass at the driver. Caller must NOT hold
 * s_cred_lock; esp_wifi_set_config() goes over SDIO to the C6 and is far too
 * slow to run under a lock the TUI's status page also wants. */
static esp_err_t sta_config_apply(void)
{
    wifi_config_t cfg = { 0 };
    /* Zeroing the struct leaves pmf_cfg.capable false, and an AP with
     * protected management frames set to required will then refuse the
     * association outright. Capable-but-not-required is the compatible
     * setting: it works both with APs that demand PMF and with those without. */
    cfg.sta.pmf_cfg.capable = true;
    xSemaphoreTake(s_cred_lock, portMAX_DELAY);
    strlcpy((char *)cfg.sta.ssid,     s_ssid, sizeof(cfg.sta.ssid));
    strlcpy((char *)cfg.sta.password, s_pass, sizeof(cfg.sta.password));
    s_sta_want = s_ssid[0] != '\0';
    xSemaphoreGive(s_cred_lock);

    return esp_wifi_set_config(WIFI_IF_STA, &cfg);
}

static void sta_creds_load(void)
{
    nvs_handle_t h;
    xSemaphoreTake(s_cred_lock, portMAX_DELAY);
    s_ssid[0] = s_pass[0] = '\0';
    if (nvs_open(CRED_NS, NVS_READONLY, &h) == ESP_OK) {
        size_t n = sizeof(s_ssid);
        if (nvs_get_str(h, CRED_SSID, s_ssid, &n) != ESP_OK) s_ssid[0] = '\0';
        n = sizeof(s_pass);
        if (nvs_get_str(h, CRED_PASS, s_pass, &n) != ESP_OK) s_pass[0] = '\0';
        nvs_close(h);
    }
    bool have = s_ssid[0] != '\0';
    xSemaphoreGive(s_cred_lock);

    ESP_LOGI(TAG, "%s", have ? "upstream SSID loaded from NVS"
                             : "no upstream SSID stored; STA idle until configured");
}

esp_err_t net_wifi_set_sta(const char *ssid, const char *pass)
{
    if (!ssid) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t err = nvs_open(CRED_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_str(h, CRED_SSID, ssid);
    if (err == ESP_OK) err = nvs_set_str(h, CRED_PASS, pass ? pass : "");
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) return err;

    xSemaphoreTake(s_cred_lock, portMAX_DELAY);
    strlcpy(s_ssid, ssid,          sizeof(s_ssid));
    strlcpy(s_pass, pass ? pass : "", sizeof(s_pass));
    xSemaphoreGive(s_cred_lock);

    /* Drop any current association before adopting the new SSID, then wake the
     * retry loop so the join happens now instead of up to STA_RETRY_MS later. */
    esp_wifi_disconnect();
    err = sta_config_apply();
    if (s_task) xTaskNotifyGive(s_task);
    return err;
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
    case WIFI_EVENT_AP_START:
        if (s_state != NET_WIFI_STA) s_state = NET_WIFI_AP;
        sys_log(1, "WIFI     SoftAP \"%s\" up on ch %d", AP_SSID, AP_CHANNEL);
        break;

    /* Counted from events rather than esp_wifi_ap_get_sta_list(): the TUI
     * reads this every frame, and each list call would be a round trip over
     * SDIO to the C6. */
    case WIFI_EVENT_AP_STACONNECTED: {
        const wifi_event_ap_staconnected_t *e = data;
        s_ap_clients++;
        sys_log(1, "WIFI     station joined " MACSTR " (%d on AP)",
                MAC2STR(e->mac), s_ap_clients);
        break;
    }
    case WIFI_EVENT_AP_STADISCONNECTED: {
        const wifi_event_ap_stadisconnected_t *e = data;
        if (s_ap_clients > 0) s_ap_clients--;
        sys_log(4, "WIFI     station left " MACSTR " (%d on AP)",
                MAC2STR(e->mac), s_ap_clients);
        break;
    }

    case WIFI_EVENT_STA_DISCONNECTED: {
        const wifi_event_sta_disconnected_t *e = data;
        s_sta_ip4 = 0;
        if (s_state == NET_WIFI_STA) {
            s_state = NET_WIFI_AP;
            sys_log(4, "WIFI     upstream lost (reason %d)", e->reason);
        } else if (s_sta_want && e->reason != s_join_fail) {
            /* A join that never succeeded has to be reported too. Without
             * this, a wrong password, a 5GHz-only SSID and a typo all look
             * identical from outside -- the board just sits at "AP only"
             * forever. 201=NO_AP_FOUND, 202=AUTH_FAIL, 15=handshake timeout.
             *
             * Once per *reason*, not once per attempt: the retry loop tries
             * every 30 s and an out-of-range home AP never stops failing, so
             * repeating it says nothing new and buries everything else on the
             * console. A changed reason is news -- the AP came back and the
             * password is wrong -- and so is the next failure after a join
             * that worked, which clears this. */
            s_join_fail = e->reason;
            sys_log(4, "WIFI     join failed, reason %d (201=no AP 202=auth "
                       "15=bad key)", e->reason);
        }
        /* No reconnect from here: the retry loop in net_wifi_task owns the
         * backoff. Calling esp_wifi_connect() in the event handler is the
         * usual pattern but turns a missing home AP into a hot loop on the
         * SDIO link. */
        break;
    }
    default:
        break;
    }
}

static void on_sta_got_ip(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    const ip_event_got_ip_t *e = data;
    s_sta_ip4   = e->ip_info.ip.addr;
    s_state     = NET_WIFI_STA;
    s_join_fail = -1;
    sys_log(1, "WIFI     upstream " IPSTR "  gw " IPSTR,
            IP2STR(&e->ip_info.ip), IP2STR(&e->ip_info.gw));
}

static esp_err_t wifi_bringup(void)
{
    /* WiFi keeps calibration data and the country/protocol settings in NVS.
     * Nothing else in this project uses it yet, so it is initialised here
     * rather than in app_main. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_flash_init: %s", esp_err_to_name(err));
        return err;
    }

    /* net_eth_start() normally did both of these already, but it returns early
     * on its own failures, so neither can be assumed. Both are idempotent
     * except for the second call's return code. */
    err = esp_netif_init();
    if (err != ESP_OK) return err;
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    esp_netif_create_default_wifi_ap();
    s_sta_netif = esp_netif_create_default_wifi_sta();
    if (s_sta_netif) esp_netif_set_route_prio(s_sta_netif, STA_ROUTE_PRIO);

    err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL);
    if (err != ESP_OK) return err;
    err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_sta_got_ip, NULL);
    if (err != ESP_OK) return err;

    /* This is the call that actually probes SDIO, resets the C6 over GPIO54
     * and waits for the slave to answer -- it is the slow, failure-prone step,
     * and why all of this runs off app_main's thread. */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init: %s -- is the C6 slave firmware alive and "
                      "in packet mode? (see ../c6-notes.md)", esp_err_to_name(err));
        return err;
    }

    wifi_config_t ap_cfg = {
        .ap = {
            .ssid_len       = strlen(AP_SSID),
            .channel        = AP_CHANNEL,
            .max_connection = AP_MAX_CONN,
            /* A password shorter than 8 characters is not a weak key, it is a
             * config esp_wifi_set_config() rejects outright -- treat anything
             * too short as "no password wanted" and stay open. */
            .authmode       = strlen(AP_PASS) >= 8 ? WIFI_AUTH_WPA2_PSK
                                                   : WIFI_AUTH_OPEN,
        },
    };
    strlcpy((char *)ap_cfg.ap.ssid,     AP_SSID, sizeof(ap_cfg.ap.ssid));
    strlcpy((char *)ap_cfg.ap.password, AP_PASS, sizeof(ap_cfg.ap.password));

    err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (err != ESP_OK) return err;
    err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    if (err != ESP_OK) return err;

    sta_creds_load();
    err = sta_config_apply();
    if (err != ESP_OK) return err;

    err = esp_wifi_start();
    if (err != ESP_OK) return err;

    /* Turn modem sleep off. ESP-IDF defaults to WIFI_PS_MIN_MODEM, which lets
     * the radio doze between the *upstream* AP's DTIM beacons -- and the C6 has
     * one radio serving both halves of APSTA, so every nap is also a nap on the
     * SoftAP. Associated stations miss frames and time out: measured here as a
     * phone dropping off roughly every 20-40 s and rejoining by itself, with
     * the AP beaconing normally throughout, which makes it look like a client
     * problem rather than a radio one. Costs idle current the C6 would
     * otherwise save; this board is USB-powered, so that is not a trade.
     * Non-fatal if an older slave does not implement the call. */
    esp_err_t pserr = esp_wifi_set_ps(WIFI_PS_NONE);
    if (pserr != ESP_OK)
        ESP_LOGW(TAG, "power save not disabled: %s", esp_err_to_name(pserr));

    /* ESP-IDF's default AP bitmap is 11B|11G|11N, so clients report "WiFi 4"
     * on a board sold as WIFI6-DEV-KIT. The C6 has HE (SOC_WIFI_HE_SUPPORT),
     * it is only off by default. Non-fatal: an older slave that does not
     * implement the call just stays on 11n. */
    const uint8_t proto = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                          WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX;
    esp_err_t perr = esp_wifi_set_protocol(WIFI_IF_AP, proto);
    if (perr == ESP_OK) perr = esp_wifi_set_protocol(WIFI_IF_STA, proto);
    if (perr != ESP_OK)
        ESP_LOGW(TAG, "11ax not enabled: %s", esp_err_to_name(perr));

    return ESP_OK;
}

static void net_wifi_task(void *arg)
{
    (void)arg;

    if (wifi_bringup() != ESP_OK) {
        s_state = NET_WIFI_OFF;
        sys_log(4, "WIFI     bring-up failed, radio unavailable");
        vTaskDelete(NULL);
        return;
    }

    /* STA join retries. The AP half is already serving by this point, so a
     * home network that never appears costs one scan every STA_RETRY_MS and
     * nothing else. */
    while (1) {
        if (s_sta_want && s_state != NET_WIFI_STA) {
            esp_err_t err = esp_wifi_connect();
            if (err != ESP_OK && err != ESP_ERR_WIFI_CONN)
                ESP_LOGD(TAG, "esp_wifi_connect: %s", esp_err_to_name(err));
        }
        /* Notify-aware instead of a plain delay: net_wifi_set_sta() wakes this
         * so a freshly configured SSID is tried at once. */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(STA_RETRY_MS));
    }
}

esp_err_t net_wifi_start(void)
{
    s_state = NET_WIFI_INIT;

    /* Off app_main's thread on purpose: esp_wifi_init() below blocks for the
     * SDIO probe and the C6's boot, and app_main still has USB host
     * enumeration to set up. Core0 for the same reason as the feeds -- core1
     * is the demod loop. Priority 3 matches them too; this task is asleep
     * except during bring-up and once per retry interval. */
    s_cred_lock = xSemaphoreCreateMutex();
    if (!s_cred_lock) {
        s_state = NET_WIFI_OFF;
        return ESP_ERR_NO_MEM;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(net_wifi_task, "wifi_mgr", 4096,
                                            NULL, 3, &s_task, 0);
    if (ok != pdTRUE) {
        s_state = NET_WIFI_OFF;
        return ESP_FAIL;
    }
    return ESP_OK;
}
