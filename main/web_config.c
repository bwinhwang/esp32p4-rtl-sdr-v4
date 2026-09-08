/*
 * Config page and JSON API over HTTP.
 *
 * Exists mainly so the upstream WiFi credentials never have to be compiled in:
 * sdkconfig is tracked and this repo's remote is a public fork, so a password
 * in Kconfig would be published. The page is served on the SoftAP the board
 * already runs, so first-time setup needs nothing but a phone.
 *
 * /aircraft.json is a second reason to have this: the :8888 feed is a raw
 * NDJSON stream with no HTTP framing, so pointing a browser at it gets an
 * "invalid response" -- correct behaviour, useless for debugging. This gives
 * the same data with a status line in front of it.
 */

#include <stdlib.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "shell.h"       /* sys_log() */
#include "net_eth.h"
#include "net_wifi.h"
#include "ota.h"
#include "web_config.h"

/* Defined in class_driver.c; same extern-rather-than-header approach the feed
 * modules already use, to keep a dependency edge out of the TUI internals. */
extern size_t aircraft_export_ndjson(char *buf, size_t bufsize);

#define SNAPSHOT_MAX  4096

static const char *TAG = "web";

/* ── tiny helpers ──────────────────────────────────────────────────────── */

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* In-place percent-decode of one form field. httpd_query_key_value() splits
 * the body but does not decode, and a WiFi password is exactly the kind of
 * string that is full of characters the browser escapes. */
static void url_decode(char *s)
{
    char *w = s;
    for (char *r = s; *r; r++) {
        if (*r == '+') {
            *w++ = ' ';
        } else if (*r == '%' && hexval(r[1]) >= 0 && hexval(r[2]) >= 0) {
            *w++ = (char)(hexval(r[1]) * 16 + hexval(r[2]));
            r += 2;
        } else {
            *w++ = *r;
        }
    }
    *w = '\0';
}

/* Escapes the few characters that would break out of the HTML this page
 * builds. SSIDs are attacker-controlled in the sense that anyone can name
 * their network anything. */
static void html_escape(const char *in, char *out, size_t n)
{
    size_t o = 0;
    for (const char *p = in; *p && o + 7 < n; p++) {
        const char *rep = *p == '<' ? "&lt;"  : *p == '>' ? "&gt;"
                        : *p == '&' ? "&amp;" : *p == '"' ? "&quot;" : NULL;
        if (rep) { size_t l = strlen(rep); memcpy(out + o, rep, l); o += l; }
        else     { out[o++] = *p; }
    }
    out[o] = '\0';
}

/* ── GET / ─────────────────────────────────────────────────────────────── */

static const char PAGE_HEAD[] =
    "<!doctype html><meta charset=utf-8>"
    "<meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>ESP32-P4 ADS-B</title><style>"
    "body{font:15px system-ui,sans-serif;margin:0;padding:1.2rem;"
    "background:#0d1117;color:#c9d1d9;max-width:34rem}"
    "h1{font-size:1.1rem;margin:0 0 1rem;color:#58a6ff}"
    "table{border-collapse:collapse;width:100%;margin-bottom:1.5rem}"
    "td{padding:.35rem .5rem;border-bottom:1px solid #21262d}"
    "td:first-child{color:#8b949e;width:9rem}"
    "label{display:block;margin:.7rem 0 .25rem;color:#8b949e}"
    "input{width:100%;padding:.5rem;border:1px solid #30363d;border-radius:5px;"
    "background:#010409;color:#c9d1d9;box-sizing:border-box}"
    "button{margin-top:1rem;padding:.55rem 1.1rem;border:0;border-radius:5px;"
    "background:#238636;color:#fff;font-size:1rem}"
    "a{color:#58a6ff}.n{color:#8b949e;font-size:.85rem;margin-top:1.5rem}"
    "</style><h1>ESP32-P4 ADS-B Receiver</h1>";

static esp_err_t root_get(httpd_req_t *req)
{
    /* row has to hold the widest template plus a fully escaped SSID: 32 chars
     * of "&quot;" expand to 192. */
    char eth_ip[16], sta_ip[16], ssid[33], esc[224], row[640];

    net_eth_ip_str(eth_ip, sizeof(eth_ip));
    net_wifi_sta_ip_str(sta_ip, sizeof(sta_ip));
    net_wifi_sta_ssid(ssid, sizeof(ssid));
    html_escape(ssid, esc, sizeof(esc));

    net_wifi_state_t ws = net_wifi_state();
    const char *wtxt = ws == NET_WIFI_STA  ? "AP + joined upstream"
                     : ws == NET_WIFI_AP   ? "AP only"
                     : ws == NET_WIFI_INIT ? "starting"
                                           : "off";

    const esp_app_desc_t *app  = esp_app_get_description();
    const esp_partition_t *run = esp_ota_get_running_partition();

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send_chunk(req, PAGE_HEAD, HTTPD_RESP_USE_STRLEN);

    snprintf(row, sizeof(row),
        "<table>"
        "<tr><td>Firmware</td><td>%s (%s)</td></tr>"
        "<tr><td>Ethernet</td><td>%s</td></tr>"
        "<tr><td>WiFi</td><td>%s</td></tr>"
        "<tr><td>SoftAP clients</td><td>%d</td></tr>"
        "<tr><td>Upstream SSID</td><td>%s</td></tr>"
        "<tr><td>Upstream IP</td><td>%s</td></tr>"
        "</table>",
        app->version, run->label,
        eth_ip, wtxt, net_wifi_ap_clients(),
        esc[0] ? esc : "<i>not configured</i>", sta_ip);
    httpd_resp_send_chunk(req, row, HTTPD_RESP_USE_STRLEN);

    snprintf(row, sizeof(row),
        "<form method=post action=/wifi>"
        "<label>Upstream WiFi SSID</label>"
        "<input name=ssid value=\"%s\" autocapitalize=off autocorrect=off>"
        "<label>Password</label>"
        "<input name=pass type=password placeholder=\"unchanged if left blank\">"
        "<button type=submit>Save &amp; connect</button></form>", esc);
    httpd_resp_send_chunk(req, row, HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req,
        "<p class=n>Stored in NVS on the board, never in the firmware image.<br>"
        "Feeds: AVR :30001 &middot; Beast :30005 &middot; JSON :8888 "
        "(raw TCP, not HTTP) &middot; <a href=/aircraft.json>/aircraft.json</a><br>"
        "OTA: <code>curl -H 'Expect:' --data-binary "
        "@build/usb_host_lib_example.bin http://&lt;this-ip&gt;/ota</code></p>",
        HTTPD_RESP_USE_STRLEN);
    return httpd_resp_send_chunk(req, NULL, 0);
}

/* ── POST /wifi ────────────────────────────────────────────────────────── */

static esp_err_t wifi_post(httpd_req_t *req)
{
    char body[256];
    int  len = req->content_len < (int)sizeof(body) - 1
             ? req->content_len : (int)sizeof(body) - 1;
    int  got = httpd_req_recv(req, body, len);
    if (got <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty body");
        return ESP_FAIL;
    }
    body[got] = '\0';

    char ssid[33] = {0}, pass[65] = {0};
    httpd_query_key_value(body, "ssid", ssid, sizeof(ssid));
    httpd_query_key_value(body, "pass", pass, sizeof(pass));
    url_decode(ssid);
    url_decode(pass);

    /* A blank password field means "keep what is stored" -- the form cannot
     * pre-fill a password box, so submitting the page to change only the SSID
     * must not wipe the key. Clearing the SSID clears both. */
    esp_err_t err;
    if (ssid[0] == '\0') {
        err = net_wifi_set_sta("", "");
    } else if (pass[0] == '\0') {
        char cur[33];
        net_wifi_sta_ssid(cur, sizeof(cur));
        err = strcmp(cur, ssid) == 0 ? ESP_OK   /* nothing changed at all */
                                     : net_wifi_set_sta(ssid, "");
    } else {
        err = net_wifi_set_sta(ssid, pass);
    }

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "set_sta failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            esp_err_to_name(err));
        return ESP_FAIL;
    }
    sys_log(1, "WEB      upstream SSID set to \"%s\"", ssid[0] ? ssid : "(none)");

    /* 303 so a reload of the result page is a GET, not a re-POST. */
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    return httpd_resp_send(req, NULL, 0);
}

/* ── GET /aircraft.json ────────────────────────────────────────────────── */

static esp_err_t aircraft_get(httpd_req_t *req)
{
    char *buf = malloc(SNAPSHOT_MAX);
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no mem");
        return ESP_FAIL;
    }
    size_t n = aircraft_export_ndjson(buf, SNAPSHOT_MAX);

    httpd_resp_set_type(req, "application/json");
    /* The snapshot is one NDJSON line; strip the newline so this is valid
     * JSON for a browser or fetch(), which the streaming feed is not. */
    while (n && (buf[n - 1] == '\n' || buf[n - 1] == '\r')) n--;
    esp_err_t err = httpd_resp_send(req, buf, n);
    free(buf);
    return err;
}

/* ── start ─────────────────────────────────────────────────────────────── */

esp_err_t web_config_start(void)
{
    /* The TUI repaints by cursor addressing, so any stray stdout line corrupts
     * it. esp_http_server logs a line per request at INFO. */
    esp_log_level_set("httpd", ESP_LOG_WARN);
    esp_log_level_set("httpd_uri", ESP_LOG_WARN);
    esp_log_level_set("httpd_txrx", ESP_LOG_WARN);
    esp_log_level_set("httpd_parse", ESP_LOG_WARN);
    esp_log_level_set("httpd_sess", ESP_LOG_WARN);

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port     = CONFIG_ADSB_WEB_PORT;
    /* Both of these matter here and both defaults are wrong for this project:
     * core_id defaults to tskNO_AFFINITY, which lets the server land on core1
     * and preempt the demod loop, and task_priority defaults to
     * tskIDLE_PRIORITY+5 -- exactly adsb_rx_task's 5. Core0 at 3 puts it with
     * the feeds instead. */
    cfg.core_id         = 0;
    cfg.task_priority   = 3;
    cfg.stack_size      = 5120;
    cfg.max_uri_handlers = 5;   /* root, wifi, aircraft.json, ota, +1 spare */
    cfg.lru_purge_enable = true;   /* a phone that walks away must not wedge it */
    /* A handler that only ever reads gives TCP nothing to probe with, so a
     * peer that disappears without a FIN is invisible to it -- and POST /ota
     * reads for seconds at a time. Keepalive is what eventually errors that
     * recv out; ota.c's own idle bound covers the case anyway. */
    cfg.keep_alive_enable   = true;
    cfg.keep_alive_idle     = 10;
    cfg.keep_alive_interval = 5;
    cfg.keep_alive_count    = 3;

    httpd_handle_t srv = NULL;
    esp_err_t err = httpd_start(&srv, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start: %s", esp_err_to_name(err));
        return err;
    }

    static const httpd_uri_t uris[] = {
        { .uri = "/",              .method = HTTP_GET,  .handler = root_get     },
        { .uri = "/wifi",          .method = HTTP_POST, .handler = wifi_post    },
        { .uri = "/aircraft.json", .method = HTTP_GET,  .handler = aircraft_get },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++)
        httpd_register_uri_handler(srv, &uris[i]);
    ota_register_http(srv);

    ESP_LOGI(TAG, "config page on :%d", CONFIG_ADSB_WEB_PORT);
    return ESP_OK;
}
