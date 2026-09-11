/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 *
 * ESP32-P4 ADS-B Receiver — ATC Terminal Edition
 */

#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "usb/usb_host.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "es8311_codec.h"
#include "rtl-sdr.h"
#include "mode-s.h"
#include "esp_task_wdt.h"
#include "feed_avr.h"
#include "feed_beast.h"
#include "feed_json.h"
#include "plane_cat.h"
#include "adsb.h"
#include "shell.h"
#include "esp_console.h"

/* ── build config ────────────────────────────────────────────────────────── */
#define CLIENT_NUM_EVENT_MSG  5
#define MAX_PACKET_SIZE       16384
#define DEFAULT_BUF_LENGTH    (MAX_PACKET_SIZE * 2)
/* A contact with no frame for this long is dropped from the table. */
#define CONTACT_TTL_US        60000000LL

/* ── audio pins (Waveshare ESP32-P4-WIFI6-DEV-KIT + ES8311) ──────────────── */
#define I2C_SCL_PIN     8
#define I2C_SDA_PIN     7
#define I2S_MCLK_PIN    13
#define I2S_BCK_PIN     12
#define I2S_WS_PIN      10
#define I2S_DOUT_PIN    9
#define I2S_DIN_PIN     11
#define PA_EN_PIN       53
#define AUDIO_RATE_HZ   16000
#define MCLK_MULTIPLE   384

/* ── USB driver types ────────────────────────────────────────────────────── */
typedef enum {
    ACTION_OPEN_DEV        = (1 << 0),
    ACTION_GET_DEV_INFO    = (1 << 1),
    ACTION_GET_DEV_DESC    = (1 << 2),
    ACTION_GET_CONFIG_DESC = (1 << 3),
    ACTION_GET_STR_DESC    = (1 << 4),
    ACTION_CLOSE_DEV       = (1 << 5),
} action_t;

#define DEV_MAX_COUNT 128

typedef struct {
    usb_host_client_handle_t client_hdl;
    uint8_t                  dev_addr;
    usb_device_handle_t      dev_hdl;
    action_t                 actions;
} usb_device_t;

typedef struct {
    struct {
        union {
            struct {
                uint8_t unhandled_devices : 1;
                uint8_t shutdown          : 1;
                uint8_t reserved6         : 6;
            };
            uint8_t val;
        } flags;
        usb_device_t device[DEV_MAX_COUNT];
    } mux_protected;
    struct {
        usb_host_client_handle_t client_hdl;
        SemaphoreHandle_t        mux_lock;
    } constant;
} class_driver_t;

/* ── audio events ────────────────────────────────────────────────────────── */
typedef enum {
    AUDIO_EVT_NONE = 0,
    AUDIO_EVT_BOOT,
    AUDIO_EVT_NEW_CONTACT,
    AUDIO_EVT_LOST_CONTACT,
    AUDIO_EVT_POSITION,
} audio_evt_t;

/* ── globals ─────────────────────────────────────────────────────────────── */
static const char *TAG = "CLASS";
static rtlsdr_dev_t   *rtldev       = NULL;
static class_driver_t *s_driver_obj;
static EXT_RAM_BSS_ATTR mode_s_t state;   /* 8 KB, mostly the ICAO cache */

/* EXT_RAM_BSS_ATTR here and on the other big statics in main/: PSRAM is
 * cache-backed and therefore unreachable from an ISR and while the cache is
 * off for a flash write, so the rule is task-context-only, CPU-only (no DMA
 * source/sink) buffers. Everything tagged is written and read by plain tasks
 * -- s_mag is the one on the demod hot path, but its access is sequential and
 * the IQ ring it is computed from already lives in PSRAM. Left internal on
 * purpose: s_log (any path may log, including ones that must stay IRAM-safe)
 * and mode-s.c's maglut (2M random lookups/s; untested from PSRAM). */
static EXT_RAM_BSS_ATTR aircraft_t s_aircraft[MAX_TRACKED];
static log_entry_t s_log[ADSB_LOG_LINES];
static int         s_log_head    = 0;
static int         s_msg_count   = 0;
static int         s_msg_bucket  = 0;
static volatile bool s_rx_running  = false;  /* adsb_rx_task is up          */
static volatile bool s_inject_req  = false;  /* the 't' hotkey, see below   */

/* Closed once a second by tracker_tick(); read by adsb_stats_get(). */
static int           s_msg_rate;
static float         s_dec_pct, s_fix_pct;
static float         s_max_range_km;
static int64_t       s_tick_us;
static unsigned long s_tick_ok, s_tick_bad, s_tick_fix;

/* audio */
static i2s_chan_handle_t  s_i2s_tx  = NULL;
static volatile int       s_volume  = 70;
static volatile bool      s_muted   = false;
static QueueHandle_t      s_audio_q = NULL;

/* ═══════════════════════════════════════════════════════════════════════════
 * AUDIO
 * ═══════════════════════════════════════════════════════════════════════════ */

static void audio_write_mono(const int16_t *samples, int n)
{
    if (!s_i2s_tx || s_muted) return;
    static int16_t stereo[512];
    int written = 0;
    while (written < n) {
        int chunk = n - written;
        if (chunk > 256) chunk = 256;
        float vol = s_volume / 100.0f;
        for (int i = 0; i < chunk; i++) {
            int16_t s = (int16_t)(samples[written + i] * vol);
            stereo[i*2]   = s;
            stereo[i*2+1] = s;
        }
        size_t bytes_out = 0;
        i2s_channel_write(s_i2s_tx, stereo, chunk * 4,
                          &bytes_out, pdMS_TO_TICKS(20));
        written += chunk;
    }
}

static void audio_tone(float freq, float dur_s, float amp)
{
    if (!s_i2s_tx || s_muted) return;
    int total = (int)(AUDIO_RATE_HZ * dur_s);
    static int16_t buf[256];
    for (int i = 0; i < total; i += 256) {
        int chunk = total - i;
        if (chunk > 256) chunk = 256;
        for (int j = 0; j < chunk; j++) {
            float t = (float)(i + j) / AUDIO_RATE_HZ;
            buf[j] = (int16_t)(sinf(2.0f * (float)M_PI * freq * t) * amp);
        }
        audio_write_mono(buf, chunk);
    }
}

static void snd_boot(void)
{
    audio_tone(440.0f, 0.06f, 7000.0f);
    vTaskDelay(pdMS_TO_TICKS(20));
    audio_tone(660.0f, 0.06f, 7000.0f);
    vTaskDelay(pdMS_TO_TICKS(20));
    audio_tone(880.0f, 0.10f, 7000.0f);
}

static void snd_new_contact(void)
{
    audio_tone(1200.0f, 0.03f, 5000.0f);
    vTaskDelay(pdMS_TO_TICKS(40));
    audio_tone(1200.0f, 0.03f, 5000.0f);
}

static void snd_lost_contact(void)
{
    audio_tone(800.0f, 0.05f, 4000.0f);
    vTaskDelay(pdMS_TO_TICKS(15));
    audio_tone(600.0f, 0.08f, 3000.0f);
}

static void snd_position_fix(void)
{
    audio_tone(1800.0f, 0.02f, 3000.0f);
}

static void audio_task(void *arg)
{
    audio_evt_t evt;
    while (1) {
        if (xQueueReceive(s_audio_q, &evt, portMAX_DELAY) == pdTRUE) {
            switch (evt) {
                case AUDIO_EVT_BOOT:        snd_boot();         break;
                case AUDIO_EVT_NEW_CONTACT: snd_new_contact();  break;
                case AUDIO_EVT_LOST_CONTACT:snd_lost_contact(); break;
                case AUDIO_EVT_POSITION:    snd_position_fix(); break;
                default: break;
            }
        }
    }
}

void audio_play(audio_evt_t evt)
{
    if (s_audio_q) xQueueSend(s_audio_q, &evt, 0);
}

esp_err_t audio_init(void)
{
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = I2C_SDA_PIN,
        .scl_io_num        = I2C_SCL_PIN,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus;
    if (i2c_new_master_bus(&bus_cfg, &i2c_bus) != ESP_OK) {
        ESP_LOGW(TAG, "I2C bus init failed — audio disabled");
        return ESP_FAIL;
    }

    /* Channel gets its pin map here but is deliberately left disabled:
     * esp_codec_dev_open() below reconfigures the clock for mclk_multiple and
     * does the enable itself. Enabling twice fails. */
    i2s_chan_config_t chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    i2s_chan_handle_t tx, rx_tmp;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx, &rx_tmp));

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCLK_PIN, .bclk = I2S_BCK_PIN,
            .ws   = I2S_WS_PIN,   .dout = I2S_DOUT_PIN,
            .din  = I2S_DIN_PIN,
            .invert_flags = {0},
        },
    };
    std_cfg.clk_cfg.mclk_multiple = MCLK_MULTIPLE;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx, &std_cfg));

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port           = I2C_NUM_0,
        .addr           = ES8311_CODEC_DEFAULT_ADDR,  /* 8-bit form, halved by the driver */
        .bus_handle     = i2c_bus,
        .clock_speed_hz = 100000,
    };
    audio_codec_i2s_cfg_t i2s_cfg = { .port = I2S_NUM_0, .tx_handle = tx };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    if (!ctrl_if || !data_if) {
        ESP_LOGW(TAG, "Codec interfaces unavailable — audio disabled");
        return ESP_FAIL;
    }

    es8311_codec_cfg_t es_cfg = {
        .ctrl_if    = ctrl_if,
        .gpio_if    = audio_codec_new_gpio(),
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC,
        .pa_pin     = PA_EN_PIN,      /* PA now follows codec open/close */
        .use_mclk   = true,
        .mclk_div   = MCLK_MULTIPLE,  /* default is 256; the I2S side runs 384 */
    };
    const audio_codec_if_t *codec_if = es8311_codec_new(&es_cfg);
    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = codec_if,
        .data_if  = data_if,
    };
    esp_codec_dev_handle_t codec = codec_if ? esp_codec_dev_new(&dev_cfg) : NULL;
    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = 16,
        .channel         = 2,
        .sample_rate     = AUDIO_RATE_HZ,
        .mclk_multiple   = MCLK_MULTIPLE,
    };
    if (!codec || esp_codec_dev_open(codec, &fs) != ESP_CODEC_DEV_OK) {
        ESP_LOGW(TAG, "ES8311 not found — audio disabled");
        return ESP_FAIL;
    }
    esp_codec_dev_set_out_vol(codec, s_volume);

    /* Tones keep going out through the channel directly rather than
     * esp_codec_dev_write(): that path waits 1 s on a full DMA queue, and a
     * beep is worth dropping, not worth blocking for. */
    s_i2s_tx = tx;

    s_audio_q = xQueueCreate(8, sizeof(audio_evt_t));
    /* core0, not core1: at prio 6 this sits above adsb_rx_task's 5, so on core1
     * every tone would preempt the demod loop mid-buffer. core0 has the room. */
    xTaskCreatePinnedToCore(audio_task, "audio", 4096, NULL, 6, NULL, 0);
    ESP_LOGI(TAG, "Audio OK  vol=%d", s_volume);
    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * CPR DECODE
 * ═══════════════════════════════════════════════════════════════════════════ */

static int cpr_nl(double lat)
{
    if (lat < 0) lat = -lat;
    if (lat < 10.47047130) return 59;
    if (lat < 14.82817437) return 58;
    if (lat < 18.18626357) return 57;
    if (lat < 21.02939493) return 56;
    if (lat < 23.54504487) return 55;
    if (lat < 25.82924707) return 54;
    if (lat < 27.93898710) return 53;
    if (lat < 29.91135686) return 52;
    if (lat < 31.77209708) return 51;
    if (lat < 33.53993436) return 50;
    if (lat < 35.22899598) return 49;
    if (lat < 36.85025108) return 48;
    if (lat < 38.41241892) return 47;
    if (lat < 39.92256684) return 46;
    if (lat < 41.38651832) return 45;
    if (lat < 42.80914012) return 44;
    if (lat < 44.19454951) return 43;
    if (lat < 45.54626723) return 42;
    if (lat < 46.86733252) return 41;
    if (lat < 48.16039128) return 40;
    if (lat < 49.42776439) return 39;
    if (lat < 50.67150166) return 38;
    if (lat < 51.89342469) return 37;
    if (lat < 53.09516153) return 36;
    if (lat < 54.27817472) return 35;
    if (lat < 55.44378444) return 34;
    if (lat < 56.59318756) return 33;
    if (lat < 57.72747354) return 32;
    if (lat < 58.84763776) return 31;
    if (lat < 59.95459277) return 30;
    if (lat < 61.04917774) return 29;
    if (lat < 62.13216659) return 28;
    if (lat < 63.20427479) return 27;
    if (lat < 64.26616523) return 26;
    if (lat < 65.31845310) return 25;
    if (lat < 66.36171008) return 24;
    if (lat < 67.39646774) return 23;
    if (lat < 68.42322022) return 22;
    if (lat < 69.44242631) return 21;
    if (lat < 70.45451075) return 20;
    if (lat < 71.45986473) return 19;
    if (lat < 72.45884545) return 18;
    if (lat < 73.45177442) return 17;
    if (lat < 74.43893416) return 16;
    if (lat < 75.42056257) return 15;
    if (lat < 76.39684391) return 14;
    if (lat < 77.36789461) return 13;
    if (lat < 78.33374083) return 12;
    if (lat < 79.29428225) return 11;
    if (lat < 80.24923213) return 10;
    if (lat < 81.19801349) return 9;
    if (lat < 82.13956981) return 8;
    if (lat < 83.07199445) return 7;
    if (lat < 83.99173563) return 6;
    if (lat < 84.89166191) return 5;
    if (lat < 85.75541621) return 4;
    if (lat < 86.53536998) return 3;
    if (lat < 87.00000000) return 2;
    return 1;
}

/* The CPR latitude/longitude indices j and m are routinely negative, and C's
 * fmod() keeps the sign of the dividend -- dump1090 uses its own modulo that
 * wraps into [0,b) for exactly this reason. With fmod() a j of -55 yields a
 * latitude ~360 degrees off (and cpr_nl() then rejects the pair, or worse,
 * doesn't). */
static double cpr_mod(double a, double b)
{
    double r = fmod(a, b);
    return r < 0.0 ? r + b : r;
}

static bool cpr_decode(aircraft_t *a)
{
    if (!a->cpr_even.valid || !a->cpr_odd.valid) return false;
    int64_t dt = a->cpr_even.ts_us - a->cpr_odd.ts_us;
    if (dt < 0) dt = -dt;
    if (dt > 10000000LL) return false;

    double rlat0 = a->cpr_even.raw_lat / 131072.0;
    double rlat1 = a->cpr_odd.raw_lat  / 131072.0;
    double rlon0 = a->cpr_even.raw_lon / 131072.0;
    double rlon1 = a->cpr_odd.raw_lon  / 131072.0;

    double dlat0 = 360.0 / 60.0;
    double dlat1 = 360.0 / 59.0;
    double j     = floor(59.0 * rlat0 - 60.0 * rlat1 + 0.5);

    double lat0 = dlat0 * (cpr_mod(j, 60.0) + rlat0);
    double lat1 = dlat1 * (cpr_mod(j, 59.0) + rlat1);
    if (lat0 >= 270.0) lat0 -= 360.0;
    if (lat1 >= 270.0) lat1 -= 360.0;

    if (cpr_nl(lat0) != cpr_nl(lat1)) return false;

    double lat, rlon, dlon;
    int nl;
    if (a->cpr_even.ts_us >= a->cpr_odd.ts_us) {
        lat  = lat0; nl = cpr_nl(lat0); rlon = rlon0;
    } else {
        lat  = lat1; nl = cpr_nl(lat1); if (nl > 0) nl--; rlon = rlon1;
    }
    dlon = 360.0 / (nl > 0 ? nl : 1);

    double m   = floor(rlon0 * (cpr_nl(lat) - 1) -
                       rlon1 *  cpr_nl(lat)       + 0.5);
    double lon = dlon * (cpr_mod(m, (nl > 0 ? nl : 1)) + rlon);
    if (lon >= 180.0) lon -= 360.0;

    a->lat = (float)lat;
    a->lon = (float)lon;
    a->pos_valid = true;
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * LOG / AIRCRAFT HELPERS
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Every event goes to the LOG panel; this decides which of them also reach a
 * console at a prompt. The ladder is by facility rather than by severity,
 * because that is the distinction that matters at a prompt: the board's own
 * events are rare and are what the user is waiting for, while the sky's arrive
 * several a second with an antenna connected -- on_msg() logs an ALT or VEL
 * line per decoded frame -- and are already on screen in the display.
 *
 * BRIEF is the middle ground: board events plus the two aircraft lines that
 * mark an event rather than a measurement, colours 1 (first contact) and 4
 * (lost). See shell.h for the levels themselves.
 *
 * It starts at ALL so a failure during boot -- USB enumeration, tuner
 * bring-up -- is not silent, and shell_console_start() drops it to the default
 * once the prompt is up. `log tail` shows the ring either way. */
static volatile int s_log_echo = ADSB_ECHO_ALL;

void adsb_log_echo_set(int level) { s_log_echo = level; }
int  adsb_log_echo_get(void)      { return s_log_echo; }

void adsb_log_dump(int n)
{
    if (n <= 0 || n > ADSB_LOG_LINES) n = ADSB_LOG_LINES;
    if (n > s_log_head)          n = s_log_head;

    if (n == 0) { printf("(no receiver events yet)\n"); return; }

    for (int i = n - 1; i >= 0; i--) {
        const log_entry_t *e = &s_log[(s_log_head - 1 - i + ADSB_LOG_LINES * 2) % ADSB_LOG_LINES];
        if (e->text[0]) printf("%s\n", e->text);
    }
}

static bool echo_wanted(uint8_t facility, uint8_t color)
{
    switch (s_log_echo) {
        case ADSB_ECHO_ALL:   return true;
        case ADSB_ECHO_BRIEF: return facility == LOG_AIR ? (color == 1 || color == 4)
                                                        : facility == LOG_SYS;
        case ADSB_ECHO_SYS:   return facility == LOG_SYS;
        default:              return false;
    }
}

static void log_put(uint8_t facility, uint8_t color, const char *fmt, va_list ap)
{
    /* Measurements (an ALT or VEL line per decoded frame, colour 0) are not
     * kept: at a few per second they would turn the ring over in seconds and
     * the events -- the reason the panel exists -- with it. They still reach a
     * prompt at `log echo all`. */
    bool keep = !(facility == LOG_AIR && color == 0);
    log_entry_t  scratch;
    log_entry_t *e = keep ? &s_log[s_log_head % ADSB_LOG_LINES] : &scratch;

    vsnprintf(e->text, sizeof(e->text), fmt, ap);
    e->color    = color;
    e->facility = facility;
    if (keep) s_log_head++;

    /* Not echoed while a screen owns stdout: the line would land inside a
     * half-painted frame, and the viewer is already looking at the panel it
     * went into. */
    if (echo_wanted(facility, color) && !shell_screen_foreground())
        shell_async_print(e->text);
}

void sys_log(uint8_t color, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    log_put(LOG_SYS, color, fmt, ap);
    va_end(ap);
}

void air_log(uint8_t color, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    log_put(LOG_AIR, color, fmt, ap);
    va_end(ap);
}

void ui_log(uint8_t color, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    log_put(LOG_UI, color, fmt, ap);
    va_end(ap);
}

static aircraft_t *find_or_create(uint32_t icao)
{
    aircraft_t *empty = NULL;
    for (int i = 0; i < MAX_TRACKED; i++) {
        if (s_aircraft[i].active && s_aircraft[i].icao == icao)
            return &s_aircraft[i];
        if (!s_aircraft[i].active && !empty)
            empty = &s_aircraft[i];
    }
    if (empty) {
        memset(empty, 0, sizeof(aircraft_t));
        empty->icao   = icao;
        empty->active = true;
        /* No callsign this early, but an address inside a military block is
         * already enough to classify -- and that is the one worth flagging. */
        empty->category = plane_classify(icao, NULL);
        air_log(1, "CONTACT  %06lX  first squawk  %s", (unsigned long)icao,
                plane_cat_label(empty->category));
        audio_play(AUDIO_EVT_NEW_CONTACT);
    }
    return empty;
}

int adsb_log_recent(const log_entry_t **out, int n)
{
    int found = 0;
    for (int back = 0; back < ADSB_LOG_LINES && found < n; back++) {
        const log_entry_t *e = &s_log[(s_log_head - 1 - back + ADSB_LOG_LINES * 2) % ADSB_LOG_LINES];
        if (!e->text[0] || e->facility == LOG_SYS) continue;
        out[found++] = e;
    }
    return found;
}

/* Kconfig has no float type, so the antenna position arrives as strings. */
static void antenna_pos(float *lat, float *lon)
{
    static float clat, clon;
    static bool  parsed;
    if (!parsed) {
        clat = strtof(CONFIG_ADSB_RX_LAT, NULL);
        clon = strtof(CONFIG_ADSB_RX_LON, NULL);
        parsed = true;
    }
    *lat = clat;
    *lon = clon;
}

/* Equirectangular is plenty at ADS-B ranges: under 0.5% error at 400 km. */
static void update_range(aircraft_t *a)
{
    float clat, clon;
    antenna_pos(&clat, &clon);
    float dy = (a->lat - clat) * 111.32f;
    float dx = (a->lon - clon) * 111.32f * cosf(clat * (float)M_PI / 180.0f);
    a->dist_km = sqrtf(dx * dx + dy * dy);
    float brg  = atan2f(dx, dy) * 180.0f / (float)M_PI;
    a->brg_deg = brg < 0 ? brg + 360.0f : brg;
}

/* Once a second, in adsb_rx_task: expires contacts and closes the rate
 * window. The percentages are over that window rather than since boot, so
 * they follow the antenna and the gain setting rather than averaging them
 * away; the EMA keeps a quiet sky from flipping them frame to frame. */
static void tracker_tick(int64_t now)
{
    if (now - s_tick_us < 1000000LL) return;
    s_tick_us = now;

    s_msg_rate   = s_msg_bucket;
    s_msg_bucket = 0;

    unsigned long ok = state.stat_goodcrc, bad = state.stat_badcrc, fix = state.stat_fixed;
    unsigned long d_ok = ok - s_tick_ok, d_bad = bad - s_tick_bad, d_fix = fix - s_tick_fix;
    s_tick_ok = ok; s_tick_bad = bad; s_tick_fix = fix;
    if (d_ok + d_bad) {
        float dec = 100.0f * (float)d_ok / (float)(d_ok + d_bad);
        float fx  = d_ok ? 100.0f * (float)d_fix / (float)d_ok : 0.0f;
        s_dec_pct += (dec - s_dec_pct) * 0.3f;
        s_fix_pct += (fx  - s_fix_pct) * 0.3f;
    }

    for (int i = 0; i < MAX_TRACKED; i++) {
        aircraft_t *a = &s_aircraft[i];
        if (!a->active || now - a->last_seen_us <= CONTACT_TTL_US) continue;
        air_log(4, "LOST     %06lX  (%s)", (unsigned long)a->icao,
                a->callsign[0] ? a->callsign : "--------");
        a->active = false;
        audio_play(AUDIO_EVT_LOST_CONTACT);
    }
}

void adsb_tick(void)
{
    if (!s_rx_running) tracker_tick(esp_timer_get_time());
}

const aircraft_t *adsb_aircraft(void) { return s_aircraft; }

void adsb_stats_get(adsb_stats_t *s)
{
    s->msg_rate     = s_msg_rate;
    s->msg_total    = s_msg_count;
    s->dec_pct      = s_dec_pct;
    s->fix_pct      = s_fix_pct;
    s->max_range_km = s_max_range_km;
}

int  adsb_volume(void)          { return s_volume; }
bool adsb_muted(void)           { return s_muted; }
void adsb_set_muted(bool muted) { s_muted = muted; }
void adsb_set_volume(int pct)   { s_volume = pct < 0 ? 0 : pct > 100 ? 100 : pct; }

/* ── JSON snapshot export (feed_json.c) ──────────────────────────────────────
 * Periodic full-table export, not one line per decode -- feed_json.c pushes
 * this same buffer to every client on a timer, so there is no per-client
 * queue and a slow client just misses ticks instead of needing backpressure
 * handling. Staleness window matches tracker_tick()'s, but read-only: this
 * runs from feed_json's own task (core0), a second reader of s_aircraft[]
 * alongside the draw task, so it must never be the one to flip `active` off. */
/* Returns the new length, or `bufsize` to mean "full, and what you asked for
 * did not fit". That sentinel is why the clamp matters: vsnprintf() returns
 * what it WOULD have written, so the old `n + w` ran past the end of the
 * buffer on truncation and handed the caller a length nothing had ever
 * written to -- a buffer overread the moment it reached send(). It could not
 * be hit at MAX_TRACKED=64 (~10.7 KB against a 16 KB buffer), but it was one
 * changed constant or one added field away. */
static size_t json_append(char *buf, size_t bufsize, size_t n, const char *fmt, ...)
{
    if (n >= bufsize) return bufsize;
    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(buf + n, bufsize - n, fmt, ap);
    va_end(ap);
    if (w <= 0) return n;
    /* >= and not >: vsnprintf() spends one byte on the NUL, so a run that
     * lands exactly on bufsize lost its last character. */
    return (n + (size_t)w >= bufsize) ? bufsize : n + (size_t)w;
}

size_t aircraft_export_ndjson(char *buf, size_t bufsize)
{
    int64_t now = esp_timer_get_time();
    size_t  n   = json_append(buf, bufsize, 0,
                    "{\"now\":%.3f,\"messages\":%d,\"aircraft\":[",
                    now / 1e6, s_msg_count);

    bool first = true;
    for (int i = 0; i < MAX_TRACKED; i++) {
        aircraft_t *a = &s_aircraft[i];
        if (!a->active || now - a->last_seen_us > 60000000LL) continue;

        char latbuf[16] = "null", lonbuf[16] = "null";
        if (a->pos_valid) {
            snprintf(latbuf, sizeof(latbuf), "%.5f", a->lat);
            snprintf(lonbuf, sizeof(lonbuf), "%.5f", a->lon);
        }

        n = json_append(buf, bufsize, n,
                "%s{\"hex\":\"%06lx\",\"flight\":\"%s\",\"alt_baro\":%d,"
                "\"gs\":%d,\"track\":%d,\"vert_rate\":%d,\"lat\":%s,\"lon\":%s,"
                "\"category\":\"%s\",\"messages\":%d,\"seen\":%.1f}",
                first ? "" : ",", (unsigned long)a->icao, a->callsign,
                a->altitude, a->velocity, a->heading, a->vert_rate,
                latbuf, lonbuf, plane_cat_label(a->category), a->msg_count,
                (double)(now - a->last_seen_us) / 1e6);
        first = false;
    }
    n = json_append(buf, bufsize, n, "]}\n");
    /* Nothing downstream can use half a line: :8888's terminating '\n' is the
     * last byte, so a truncated snapshot costs the peer the line boundary
     * rather than a few aircraft, and /aircraft.json would serve invalid JSON.
     * Skipping the tick is the honest answer; feed_json.c logs it. */
    return (n >= bufsize) ? 0 : n;
}


/* ═══════════════════════════════════════════════════════════════════════════
 * ON_MSG CALLBACK
 * ═══════════════════════════════════════════════════════════════════════════ */

static void on_msg(mode_s_t *self, struct mode_s_msg *mm)
{
    s_msg_count++;
    s_msg_bucket++;

    uint32_t icao = ((uint32_t)mm->aa1 << 16) |
                    ((uint32_t)mm->aa2 <<  8) |
                     (uint32_t)mm->aa3;

    /* Unreachable while check_crc is set, but the ICAO of a bad-CRC frame is
     * itself garbage -- it must never reach find_or_create(). */
    if (!mm->crcok) return;

    /* Ahead of find_or_create(): the table caps at MAX_TRACKED, and a full
     * table must not quietly stop the feed. */
    feed_avr_push(mm->msg, mm->msgbits);
    feed_beast_push(mm->msg, mm->msgbits, mm->timestamp_12mhz, mm->signal_level);

    aircraft_t *a = find_or_create(icao);
    if (!a) return;

    a->last_seen_us = esp_timer_get_time();
    a->msg_count++;
    a->sig = (uint8_t)mm->signal_level;

    if (mm->flight[0]) {
        strncpy(a->callsign, mm->flight, 8);
        a->callsign[8] = '\0';
        for (int i = 7; i >= 0 && a->callsign[i] == ' '; i--)
            a->callsign[i] = '\0';
        a->category = plane_classify(icao, a->callsign);
        air_log(2, "IDENT    %06lX  %s  %s", (unsigned long)icao, a->callsign,
                plane_cat_label(a->category));
    }

    /* mode-s.c fills `identity` for every frame; it is the squawk only in the
     * two identity replies. */
    if ((mm->msgtype == 5 || mm->msgtype == 21) && mm->identity) {
        if (a->squawk != mm->identity)
            air_log(mm->identity == 7500 || mm->identity == 7600 || mm->identity == 7700 ? 4 : 2,
                    "SQUAWK   %06lX  %04d", (unsigned long)icao, mm->identity);
        a->squawk = mm->identity;
    }

    if (mm->altitude)         a->altitude  = mm->altitude;
    if (mm->heading_is_valid) a->heading   = mm->heading;
    if (mm->velocity)         a->velocity  = mm->velocity;
    if (mm->ew_velocity)
        a->ew_velocity = mm->ew_dir ? -mm->ew_velocity : mm->ew_velocity;
    if (mm->ns_velocity)
        a->ns_velocity = mm->ns_dir ? -mm->ns_velocity : mm->ns_velocity;
    /* mm->vert_rate is the raw 9-bit field as dump1090 leaves it: 0 means "no
     * information", otherwise it's 64 ft/min steps biased by one. The table's
     * climb/descent thresholds are in ft/min, so convert here or every
     * aircraft reads as level. */
    if (mm->vert_rate) {
        int fpm = (mm->vert_rate - 1) * 64;
        a->vert_rate = mm->vert_rate_sign ? -fpm : fpm;
    }

    /* CPR position */
    if (mm->msgtype == 17 && mm->metype >= 9 && mm->metype <= 18
        && mm->raw_latitude != 0) {
        int64_t ts = esp_timer_get_time();
        if (mm->fflag == 0)
            a->cpr_even = (cpr_frame_t){ mm->raw_latitude, mm->raw_longitude, ts, true };
        else
            a->cpr_odd  = (cpr_frame_t){ mm->raw_latitude, mm->raw_longitude, ts, true };
        bool was_valid = a->pos_valid;
        if (cpr_decode(a)) {
            update_range(a);
            /* A CPR pair straddling a zone boundary decodes to somewhere
             * absurd; no real 1090 MHz reception reaches this far. */
            if (a->dist_km > s_max_range_km && a->dist_km < 600.0f)
                s_max_range_km = a->dist_km;
            if (!was_valid) {
                air_log(3, "FIX      %06lX  %+.4f  %+.4f  %.0f km",
                        (unsigned long)icao, a->lat, a->lon, a->dist_km);
                audio_play(AUDIO_EVT_POSITION);
            }
        }
    }

    if (mm->msgtype == 17) {
        if (mm->metype >= 9 && mm->metype <= 18 && mm->altitude)
            air_log(0, "ALT      %06lX  %d ft", (unsigned long)icao, a->altitude);
        else if (mm->metype >= 19 && mm->metype <= 22 && mm->velocity)
            air_log(0, "VEL      %06lX  %d kt  hdg=%d  vs=%d",
                    (unsigned long)icao, a->velocity, a->heading, a->vert_rate);
    }
}

/* ── synthetic contacts ('t' key) ─────────────────────────────────────────
 * Drives the table, the radar and the log with no antenna and no dongle, so
 * display and feed work can be tested indoors. s_aircraft[] has no lock, so
 * this must run in adsb_rx_task -- the task on_msg() runs in -- whenever that
 * task exists; adsb_inject_test() defers to it through s_inject_req for
 * exactly that reason. */
static void inject_fake_aircraft(void)
{
    /* One entry per plane_classify() bucket, so four presses put one of each
     * on screen. 0xAE1234 is inside the US military block *and* carries a
     * military prefix; 0x780ABC deliberately never gets a callsign. */
    static const struct { uint32_t icao; const char *callsign; } FAKE[] = {
        { 0x4CA1FA, "RYR1234" },
        { 0xAE1234, "RCH567"  },
        { 0xA12345, "N172SP"  },
        { 0x780ABC, ""        },
    };
    static int seq = 0;

    const int n   = seq % (int)(sizeof(FAKE) / sizeof(FAKE[0]));
    const int rev = seq / (int)(sizeof(FAKE) / sizeof(FAKE[0]));

    aircraft_t *a = find_or_create(FAKE[n].icao);
    if (!a) return;

    a->last_seen_us = esp_timer_get_time();
    a->msg_count++;
    if (FAKE[n].callsign[0]) {
        strncpy(a->callsign, FAKE[n].callsign, 8);
        a->callsign[8] = '\0';
    }
    a->category = plane_classify(a->icao, a->callsign);

    /* Placed around the configured antenna, not a fixed lat/lon, so the blips
     * land on the radar whatever CONFIG_ADSB_RX_* is set to; 20..190 km spans
     * the three range settings. Not counted towards max range. */
    float clat, clon;
    antenna_pos(&clat, &clon);
    float ang  = (float)(n * 90 + rev * 17) * (float)M_PI / 180.0f;
    float km   = 20.0f + (float)((rev * 37) % 170);
    a->lat = clat + km * cosf(ang) / 111.32f;
    a->lon = clon + km * sinf(ang) / (111.32f * cosf(clat * (float)M_PI / 180.0f));
    a->pos_valid = true;
    update_range(a);

    a->altitude  = 3000 + ((seq * 2500) % 36000);
    a->velocity  = 180  + ((seq *   37) % 320);
    a->heading   =        ((seq *   53) % 360);
    a->vert_rate = -1600 + ((seq *  448) % 3200);
    int oct = (seq * 2467) % 4096;      /* four octal digits, as a real squawk */
    a->squawk    = n == 1 && rev == 1 ? 7700
                 : ((oct >> 9) & 7) * 1000 + ((oct >> 6) & 7) * 100 + ((oct >> 3) & 7) * 10 + (oct & 7);
    a->sig       = (uint8_t)(40 + (seq * 61) % 200);

    air_log(3, "FAKE     %06lX  %s  %s", (unsigned long)a->icao,
            a->callsign[0] ? a->callsign : "--------",
            plane_cat_label(a->category));

    seq++;
}

void adsb_inject_test(void)
{
    /* With no dongle there is no rx task and no writer to race, so inline. */
    if (s_rx_running) s_inject_req = true;
    else              inject_fake_aircraft();
}

/* Sized for the one caller's fixed DEFAULT_BUF_LENGTH chunk. Static rather than
 * malloc'd because this sits on the 4 MB/s IQ hot path, and safe only because
 * adsb_rx_task is the sole caller -- a second demodulating task needs its own. */
static EXT_RAM_BSS_ATTR uint16_t s_mag[DEFAULT_BUF_LENGTH / 2];

void demodulate(uint8_t *source, int length)
{
    if (!source || length <= 0) return;
    if (length > DEFAULT_BUF_LENGTH) length = DEFAULT_BUF_LENGTH;
    int mag_len = length / 2;
    mode_s_compute_magnitude_vector(source, s_mag, length);

    /* mag_len samples at the fixed 2 MSPS rate span mag_len/2 us; the clock
     * read here lands at the *last* sample, so back it up by that span to get
     * sample 0's timestamp -- mode_s_detect() adds each message's own sample
     * offset on top. Receiver-local monotonic clock, not GPS/PPS-disciplined:
     * fine for Beast-format feeder compatibility, not for real MLAT. */
    uint64_t base_ts_us = (uint64_t)esp_timer_get_time() - (uint64_t)(mag_len / 2);
    mode_s_detect(&state, s_mag, mag_len, base_ts_us, on_msg);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ADSB RX TASK
 * ═══════════════════════════════════════════════════════════════════════════ */

void adsb_rx_task(void *arg)
{
    s_tick_us = esp_timer_get_time();

    sys_log(1, "INIT     adsb_rx running on CPU1");

    {
        enum rtlsdr_tuner tt = rtlsdr_get_tuner_type(rtldev);
        int      lock = rtlsdr_get_tuner_pll_locked(rtldev);
        uint32_t fc   = rtlsdr_get_center_freq(rtldev);
        uint32_t sr   = rtlsdr_get_sample_rate(rtldev);
        uint32_t ref  = rtlsdr_get_tuner_xtal(rtldev);

        sys_log(lock == 1 ? 1 : 4,
                "INIT     %s %s  %lu.%03lu MHz  %lu.%03lu MSPS  ref %lu.%03lu MHz",
                tt == RTLSDR_TUNER_R828D ? "R828D"
                    : tt == RTLSDR_TUNER_R820T ? "R820T" : "tuner",
                lock == 1 ? "locked" : lock == 0 ? "PLL UNLOCKED" : "lock unknown",
                (unsigned long)(fc / 1000000), (unsigned long)(fc % 1000000 / 1000),
                (unsigned long)(sr / 1000000), (unsigned long)(sr % 1000000 / 1000),
                (unsigned long)(ref / 1000000), (unsigned long)(ref % 1000000 / 1000));
    }

    uint8_t *buffer = malloc(DEFAULT_BUF_LENGTH);
    if (!buffer) { sys_log(4, "OOM rx buffer"); vTaskDelete(NULL); return; }

    bool full_buffer = false;
    mode_s_init(&state);

    /* UART0 is set up and read by shell.c's console task -- see the note there
     * on why the reader had to leave this task. The display's own draw task is
     * started from app_main and runs whether or not a dongle ever appears. */
    s_rx_running = true;

    bool stream_started = (rtlsdr_stream_start(rtldev) == 0);
    if (!stream_started) sys_log(4, "STREAM   start failed, retrying");

    int64_t last_yield = esp_timer_get_time();

    while (true) {
        /* The 't' hotkey is served here rather than where the key is read:
         * inject_fake_aircraft() writes s_aircraft[] and this is the task
         * on_msg() runs in, which is the only thing that makes the synthetic
         * contacts race-free (there is no lock on that table). */
        if (s_inject_req) { s_inject_req = false; inject_fake_aircraft(); }

        if (!stream_started) {
            vTaskDelay(pdMS_TO_TICKS(100));
            stream_started = (rtlsdr_stream_start(rtldev) == 0);
        } else {
            /* Drain the pump task's IQ ring straight into the demod buffer.
             * The USB side runs decoupled in stream_pump_task -- this loop
             * never blocks on a USB transfer, only on the ring having data. */
            int got = 0, stall_ticks = 0;
            while (got < DEFAULT_BUF_LENGTH) {
                int r = rtlsdr_stream_read(&buffer[got], DEFAULT_BUF_LENGTH - got);
                if (r > 0) { got += r; stall_ticks = 0; continue; }
                if (++stall_ticks > 200) break;   /* ~2s with no data */
                vTaskDelay(1);
            }

            full_buffer = (got >= DEFAULT_BUF_LENGTH);
            if (full_buffer) {
                demodulate(buffer, DEFAULT_BUF_LENGTH);
            } else if (got > 0) {
                sys_log(3, "SHORT READ  got=%d", got);
            }
        }

        /* While the ring has data this loop never blocks, so IDLE1 -- and with
         * it the task watchdog and deleted-task cleanup -- only runs if core1
         * is handed back deliberately. One tick per half second is ~2% of the
         * IQ budget; taskYIELD() would not do, IDLE is lower priority. */
        int64_t now = esp_timer_get_time();
        tracker_tick(now);
        if (now - last_yield >= 500000LL) {
            last_yield = now;
            vTaskDelay(1);
        }
    }

    free(buffer);
    vTaskDelete(NULL);
}


/* ═══════════════════════════════════════════════════════════════════════════
 * USB RECOVERY TASK
 *
 * esp_libusb's bulk/stream pipeline calls adsb_request_recover() when the
 * endpoint is wedged beyond its own halt/flush/clear + teardown escalation.
 * Runs on its own task (rather than doing the reset inline in the pump/bulk
 * caller) so it never resets the interface out from under the code that's
 * still mid-transfer on it.
 * ═══════════════════════════════════════════════════════════════════════════ */

static TaskHandle_t s_recover_task_hdl = NULL;

static void usb_recover_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        sys_log(4, "USB      pipe wedged, resetting RTL interface");
        if (rtldev && rtlsdr_reset_interface(rtldev) == 0) {
            sys_log(2, "USB      interface reset OK");
        } else {
            sys_log(4, "USB      interface reset FAILED - replug dongle");
        }
    }
}

void adsb_request_recover(void)
{
    if (s_recover_task_hdl) xTaskNotifyGive(s_recover_task_hdl);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SHELL COMMANDS
 *
 * The receiver's own state lives in statics here (s_aircraft, rtldev), so
 * these three commands are registered from this file rather than shell.c.
 * They run on the shell task on core0.
 *
 * s_aircraft is read without a lock, exactly as the display reads it from
 * the draw task while adsb_rx_task writes it on core1 -- a torn field shows a
 * wrong number for one listing and nothing worse.
 * ═══════════════════════════════════════════════════════════════════════════ */

static int cmd_ac(int argc, char **argv)
{
    int64_t now = esp_timer_get_time();
    int     n   = 0;

    printf("%-6s %-8s %-4s %4s %6s %5s %4s %10s %10s %4s %3s %3s %5s %4s\n",
           "ICAO", "CALLSIGN", "CAT", "SQK", "ALT", "SPD", "HDG", "LAT", "LON",
           "DIST", "BRG", "SIG", "MSGS", "AGE");

    for (int i = 0; i < MAX_TRACKED; i++) {
        const aircraft_t *a = &s_aircraft[i];
        if (!a->active) continue;
        n++;

        char lat[12] = "---", lon[12] = "---", dist[8] = "---", brg[8] = "---", sqk[8] = "----";
        if (a->pos_valid) {
            snprintf(lat,  sizeof(lat),  "%.4f", a->lat);
            snprintf(lon,  sizeof(lon),  "%.4f", a->lon);
            snprintf(dist, sizeof(dist), "%.0f", a->dist_km);
            snprintf(brg,  sizeof(brg),  "%03.0f", a->brg_deg);
        }
        if (a->squawk) snprintf(sqk, sizeof(sqk), "%04d", a->squawk);
        printf("%06lX %-8s %-4s %4s %6d %5d %4d %10s %10s %4s %3s %3u %5d %3llds\n",
               (unsigned long)a->icao,
               a->callsign[0] ? a->callsign : "-",
               plane_cat_label(a->category), sqk,
               a->altitude, a->velocity, a->heading,
               lat, lon, dist, brg, a->sig, a->msg_count,
               (long long)((now - a->last_seen_us) / 1000000));
    }
    printf("%d of %d slots active\n", n, MAX_TRACKED);
    return ESP_OK;
}

static int cmd_usb(int argc, char **argv)
{
    /* Declared here rather than included: esp_libusb.h carries its own
     * unrelated class_driver_t. */
    extern uint32_t esp_libusb_stream_avail(void);
    extern uint64_t esp_libusb_stream_dropped(void);
    extern int      esp_libusb_stream_slots(void);

    if (argc > 1 && strcmp(argv[1], "reset") == 0) {
        printf("requesting an RTL interface reset\n");
        adsb_request_recover();
        return ESP_OK;
    }
    if (argc > 1) {
        printf("usage: usb          -- stream statistics\n"
               "       usb reset    -- reset the RTL interface\n");
        return ESP_ERR_INVALID_ARG;
    }

    printf("dongle    %s\n", rtldev ? "open" : "not enumerated");
    printf("ring      %u byte(s) queued across %d slot(s)\n",
           (unsigned)esp_libusb_stream_avail(), esp_libusb_stream_slots());
    /* The number that matters: anything but 0 means IQ samples were lost
     * because the demod loop did not drain the ring in time. */
    printf("dropped   %llu\n", (unsigned long long)esp_libusb_stream_dropped());
    return ESP_OK;
}

/* The dongle has no gain-mode getter, so mirror what was last set. Starts
 * false because rtlsdr_setup_task() puts the tuner in auto gain at init. */
static bool s_gain_manual = false;

static int cmd_sdr(int argc, char **argv)
{
    if (!rtldev) { printf("no dongle enumerated\n"); return ESP_ERR_INVALID_STATE; }

    if (argc == 1) {
        /* 1 = locked, 0 = not, -1 = this tuner cannot report it. */
        int pll = rtlsdr_get_tuner_pll_locked(rtldev);
        printf("tuner     type %d, PLL %s\n",
               (int)rtlsdr_get_tuner_type(rtldev),
               pll > 0 ? "locked" : pll == 0 ? "UNLOCKED" : "n/a");
        printf("freq      %u Hz\n",  (unsigned)rtlsdr_get_center_freq(rtldev));
        printf("rate      %u Hz\n",  (unsigned)rtlsdr_get_sample_rate(rtldev));
        printf("gain      %.1f dB (%s)\n", rtlsdr_get_tuner_gain(rtldev) / 10.0,
               s_gain_manual ? "manual"
                             : "auto -- the AGC overrides this; the number is only what was last set");
        printf("ppm       %d\n",      rtlsdr_get_freq_correction(rtldev));

        int gains[32];
        int ng = rtlsdr_get_tuner_gains(rtldev, gains);
        if (ng > 0) {
            printf("available ");
            for (int i = 0; i < ng && i < 32; i++) printf("%.1f ", gains[i] / 10.0);
            printf("dB\n");
        }
        return ESP_OK;
    }

    if (strcmp(argv[1], "gain") == 0 && argc == 3) {
        if (strcmp(argv[2], "auto") == 0) {
            int r = rtlsdr_set_tuner_gain_mode(rtldev, 0);
            if (r == 0) s_gain_manual = false;
            printf("gain mode auto (%d)\n", r);
            return r == 0 ? ESP_OK : ESP_FAIL;
        }
        int tenths = (int)(atof(argv[2]) * 10);
        rtlsdr_set_tuner_gain_mode(rtldev, 1);
        int r = rtlsdr_set_tuner_gain(rtldev, tenths);
        if (r == 0) s_gain_manual = true;
        printf("gain %.1f dB (%d)\n", tenths / 10.0, r);
        return r == 0 ? ESP_OK : ESP_FAIL;
    }

    if (strcmp(argv[1], "freq") == 0 && argc == 3) {
        /* Deliberately unguarded: retuning away from 1090 MHz stops ADS-B
         * decoding until it is set back. Useful for proving the tuner works. */
        uint32_t hz = (uint32_t)strtoul(argv[2], NULL, 10);
        int      r  = rtlsdr_set_center_freq(rtldev, hz);
        printf("freq %u Hz (%d)%s\n", (unsigned)hz, r,
               (hz < 1089000000u || hz > 1091000000u) ? "  -- ADS-B decoding will stop" : "");
        return r == 0 ? ESP_OK : ESP_FAIL;
    }

    printf("usage: sdr                  -- tuner status\n"
           "       sdr gain <dB|auto>   -- set tuner gain\n"
           "       sdr freq <Hz>        -- retune (1090000000 for ADS-B)\n");
    return ESP_ERR_INVALID_ARG;
}

void adsb_register_shell_cmds(void)
{
    const esp_console_cmd_t cmds[] = {
        { .command = "ac",  .help = "list the tracked aircraft",              .func = cmd_ac  },
        { .command = "usb", .help = "IQ stream statistics, or reset the RTL", .func = cmd_usb },
        { .command = "sdr", .help = "tuner status, gain and frequency",       .func = cmd_sdr },
    };
    for (int i = 0; i < (int)(sizeof(cmds) / sizeof(cmds[0])); i++)
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
}

/* ═══════════════════════════════════════════════════════════════════════════
 * RTLSDR SETUP TASK
 * ═══════════════════════════════════════════════════════════════════════════ */

static void rtlsdr_setup_task(void *arg)
{
    uint8_t dev_addr = (uint8_t)(uint32_t)arg;

    int r = rtlsdr_open(&rtldev, dev_addr, s_driver_obj->constant.client_hdl);
    if (r < 0) { ESP_LOGE(TAG, "rtlsdr_open failed"); vTaskDelete(NULL); return; }

    rtlsdr_set_center_freq(rtldev, 1090000000);
    rtlsdr_set_sample_rate(rtldev, 2000000);
    rtlsdr_set_tuner_gain_mode(rtldev, 0);
    rtlsdr_reset_buffer(rtldev);

    if (!s_recover_task_hdl)
        xTaskCreatePinnedToCore(usb_recover_task, "usb_recover", 4096, NULL,
                                4, &s_recover_task_hdl, 0);

    /* dongle found — audio already init'd in app_main, just play the sound */
    audio_play(AUDIO_EVT_NEW_CONTACT);

    /* 16K was inherited guesswork; measured peak use is ~1.9K (the TASKS panel's
     * STACK column is the free margin, watch it if this file grows). */
    xTaskCreatePinnedToCore(adsb_rx_task, "adsb_rx", 4096, NULL, 5, NULL, 1);
    vTaskDelete(NULL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * CLIENT EVENT CALLBACK
 * ═══════════════════════════════════════════════════════════════════════════ */

static void client_event_cb(const usb_host_client_event_msg_t *event_msg,
                             void *arg)
{
    class_driver_t *driver_obj = (class_driver_t *)arg;
    switch (event_msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        xSemaphoreTake(driver_obj->constant.mux_lock, portMAX_DELAY);
        driver_obj->mux_protected.device[event_msg->new_dev.address].dev_addr =
            event_msg->new_dev.address;
        xTaskCreatePinnedToCore(rtlsdr_setup_task, "rtlsdr_setup", 8192,
                                (void *)(uint32_t)event_msg->new_dev.address,
                                4, NULL, 0);
        xSemaphoreGive(driver_obj->constant.mux_lock);
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        xSemaphoreTake(driver_obj->constant.mux_lock, portMAX_DELAY);
        for (uint8_t i = 0; i < DEV_MAX_COUNT; i++) {
            if (driver_obj->mux_protected.device[i].dev_hdl ==
                event_msg->dev_gone.dev_hdl) {
                driver_obj->mux_protected.device[i].actions = ACTION_CLOSE_DEV;
                driver_obj->mux_protected.flags.unhandled_devices = 1;
            }
        }
        xSemaphoreGive(driver_obj->constant.mux_lock);
        break;
    default: abort();
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ACTION HANDLERS
 * ═══════════════════════════════════════════════════════════════════════════ */

static void action_open_dev(usb_device_t *d)
{
    assert(d->dev_addr != 0);
    ESP_ERROR_CHECK(usb_host_device_open(d->client_hdl, d->dev_addr,
                                          &d->dev_hdl));
    d->actions |= ACTION_GET_DEV_INFO;
}
static void action_get_info(usb_device_t *d)
{
    usb_device_info_t i;
    ESP_ERROR_CHECK(usb_host_device_info(d->dev_hdl, &i));
    if (i.parent.dev_hdl) {
        usb_device_info_t p;
        ESP_ERROR_CHECK(usb_host_device_info(i.parent.dev_hdl, &p));
    }
    d->actions |= ACTION_GET_DEV_DESC;
}
static void action_get_dev_desc(usb_device_t *d)
{
    const usb_device_desc_t *dd;
    ESP_ERROR_CHECK(usb_host_get_device_descriptor(d->dev_hdl, &dd));
    d->actions |= ACTION_GET_CONFIG_DESC;
}
static void action_get_config_desc(usb_device_t *d)
{
    const usb_config_desc_t *cd;
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(d->dev_hdl, &cd));
    d->actions |= ACTION_GET_STR_DESC;
}
static void action_get_str_desc(usb_device_t *d)
{
    usb_device_info_t i;
    ESP_ERROR_CHECK(usb_host_device_info(d->dev_hdl, &i));
}
static void action_close_dev(usb_device_t *d)
{
    ESP_ERROR_CHECK(usb_host_device_close(d->client_hdl, d->dev_hdl));
    d->dev_hdl = NULL; d->dev_addr = 0;
}

static void class_driver_device_handle(usb_device_t *d)
{
    uint8_t actions = d->actions;
    d->actions = 0;
    while (actions) {
        if (actions & ACTION_OPEN_DEV)        action_open_dev(d);
        if (actions & ACTION_GET_DEV_INFO)    action_get_info(d);
        if (actions & ACTION_GET_DEV_DESC)    action_get_dev_desc(d);
        if (actions & ACTION_GET_CONFIG_DESC) action_get_config_desc(d);
        if (actions & ACTION_GET_STR_DESC)    action_get_str_desc(d);
        if (actions & ACTION_CLOSE_DEV)       action_close_dev(d);
        actions = d->actions; d->actions = 0;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * CLASS DRIVER TASK
 * ═══════════════════════════════════════════════════════════════════════════ */

void class_driver_task(void *arg)
{
    class_driver_t           obj = {0};
    usb_host_client_handle_t hdl = NULL;

    ESP_LOGI(TAG, "Registering Client");
    SemaphoreHandle_t mux = xSemaphoreCreateMutex();
    if (!mux) { ESP_LOGE(TAG, "mutex fail"); vTaskSuspend(NULL); return; }

    usb_host_client_config_t cfg = {
        .is_synchronous    = false,
        .max_num_event_msg = CLIENT_NUM_EVENT_MSG,
        .async = { .client_event_callback = client_event_cb,
                   .callback_arg          = (void *)&obj },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&cfg, &hdl));

    obj.constant.mux_lock   = mux;
    obj.constant.client_hdl = hdl;
    for (uint8_t i = 0; i < DEV_MAX_COUNT; i++)
        obj.mux_protected.device[i].client_hdl = hdl;
    s_driver_obj = &obj;

    while (1) {
        if (obj.mux_protected.flags.unhandled_devices) {
            xSemaphoreTake(obj.constant.mux_lock, portMAX_DELAY);
            for (uint8_t i = 0; i < DEV_MAX_COUNT; i++)
                if (obj.mux_protected.device[i].actions)
                    class_driver_device_handle(&obj.mux_protected.device[i]);
            obj.mux_protected.flags.unhandled_devices = 0;
            xSemaphoreGive(obj.constant.mux_lock);
        } else {
            if (!obj.mux_protected.flags.shutdown)
                usb_host_client_handle_events(hdl, portMAX_DELAY);
            else break;
        }
    }

    ESP_LOGI(TAG, "Deregistering Class Client");
    ESP_ERROR_CHECK(usb_host_client_deregister(hdl));
    if (mux) vSemaphoreDelete(mux);
    vTaskSuspend(NULL);
}

void class_driver_client_deregister(void)
{
    xSemaphoreTake(s_driver_obj->constant.mux_lock, portMAX_DELAY);
    for (uint8_t i = 0; i < DEV_MAX_COUNT; i++) {
        if (s_driver_obj->mux_protected.device[i].dev_hdl != NULL) {
            s_driver_obj->mux_protected.device[i].actions |= ACTION_CLOSE_DEV;
            s_driver_obj->mux_protected.flags.unhandled_devices = 1;
        }
    }
    s_driver_obj->mux_protected.flags.shutdown = 1;
    xSemaphoreGive(s_driver_obj->constant.mux_lock);
    ESP_ERROR_CHECK(usb_host_client_unblock(s_driver_obj->constant.client_hdl));
}
