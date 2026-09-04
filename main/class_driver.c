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
#include "freertos/idf_additions.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "usb/usb_host.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "es8311_codec.h"
#include "rtl-sdr.h"
#include "mode-s.h"
#include "esp_task_wdt.h"

/* ── build config ────────────────────────────────────────────────────────── */
#define CLIENT_NUM_EVENT_MSG  5
#define MAX_PACKET_SIZE       16384
#define DEFAULT_BUF_LENGTH    (MAX_PACKET_SIZE * 2)
#define MAX_TRACKED           16
#define LOG_LINES             7
/* A full repaint is ~13 KB, and pushing bytes at the console UART costs ~7 us
 * of CPU each, so this constant sets core0 load directly: at 150 ms it was
 * 90 KB/s, which is 98% of the 921600-baud line rate and 63% of the core.
 * Raise it before blaming anything else for core0 being busy. */
#define TUI_REFRESH_MS        500

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

/* ── CPR frame ───────────────────────────────────────────────────────────── */
typedef struct {
    int     raw_lat;
    int     raw_lon;
    int64_t ts_us;
    bool    valid;
} cpr_frame_t;

/* ── aircraft record ─────────────────────────────────────────────────────── */
typedef struct {
    uint32_t    icao;
    char        callsign[9];
    int         altitude;
    int         velocity;
    int         heading;
    float       lat;
    float       lon;
    bool        pos_valid;
    int         ew_velocity;
    int         ns_velocity;
    int         vert_rate;
    int         msg_count;
    int64_t     last_seen_us;
    cpr_frame_t cpr_even;
    cpr_frame_t cpr_odd;
    bool        active;
} aircraft_t;

/* ── event log ───────────────────────────────────────────────────────────── */
typedef struct {
    char    text[80];
    uint8_t color;
} log_entry_t;

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
static mode_s_t        state;

static aircraft_t  s_aircraft[MAX_TRACKED];
static log_entry_t s_log[LOG_LINES];
static int         s_log_head    = 0;
static int         s_msg_count   = 0;
static int         s_msg_rate    = 0;
static int         s_msg_bucket  = 0;
static int64_t     s_rate_ts     = 0;
static int64_t     s_start_us    = 0;
static int64_t     s_last_draw   = 0;
static bool        s_dirty       = false;
static float       s_decode_smooth = 0.0f;
static float       s_crc_smooth    = 0.0f;
static float       s_fix_smooth    = 0.0f;

/* audio */
static i2s_chan_handle_t  s_i2s_tx  = NULL;
static volatile int       s_volume  = 70;
static volatile bool      s_muted   = false;
static QueueHandle_t      s_audio_q = NULL;

/* ── ANSI / phosphor-green ATC palette ───────────────────────────────────── */
#define CLS      "\033[2J\033[H"
#define RESET    "\033[0m"
#define BOLD     "\033[1m"
#define DIM      "\033[2m"

/* 24-bit RGB phosphor green palette */
#define PH_HI    "\033[38;2;0;255;80m"     /* bright phosphor    */
#define PH_MID   "\033[38;2;0;200;60m"     /* mid phosphor       */
#define PH_DIM   "\033[38;2;0;100;30m"     /* dim phosphor       */
#define PH_SCAN  "\033[38;2;140;255;140m"  /* scan highlight     */
#define PH_GRID  "\033[38;2;0;60;20m"      /* grid lines         */

/* accent colours */
#define AC_AMBER "\033[38;2;255;180;0m"    /* warning amber      */
#define AC_RED   "\033[38;2;255;60;60m"    /* alert red          */
#define AC_CYAN  "\033[38;2;0;220;220m"    /* info cyan          */
#define AC_WHITE "\033[38;2;220;255;220m"  /* near-white         */

/* box drawing UTF-8 */
#define HL   "\xe2\x94\x80"
#define VL   "\xe2\x94\x82"
#define TL   "\xe2\x94\x8c"
#define TR   "\xe2\x94\x90"
#define BL   "\xe2\x94\x94"
#define BR   "\xe2\x94\x98"
#define TR_  "\xe2\x94\x9c"
#define TL_  "\xe2\x94\xa4"
#define T_UP "\xe2\x94\xb4"
#define CROSS "\xe2\x94\xbc"
#define BLK  "\xe2\x96\x88"
#define BBLK "\xe2\x96\x91"

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

/* Before the TUI's first frame there's no box on screen to protect, but
 * there's also no tui_draw() call yet to ever surface a queued s_log[]
 * entry -- so anything logged during device setup (USB enumeration, tuner
 * bring-up) needs to also go out raw, or a setup-time hang/error becomes
 * completely silent instead of just ugly. */
static bool s_tui_active = false;

void tui_log(uint8_t color, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_log[s_log_head % LOG_LINES].text,
              sizeof(s_log[0].text), fmt, ap);
    va_end(ap);
    s_log[s_log_head % LOG_LINES].color = color;
    s_log_head++;
    s_dirty = true;

    if (!s_tui_active) {
        printf("%s\n", s_log[(s_log_head - 1) % LOG_LINES].text);
        fflush(stdout);
    }
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
        tui_log(1, "CONTACT  %06lX  first squawk", (unsigned long)icao);
        audio_play(AUDIO_EVT_NEW_CONTACT);
    }
    return empty;
}

static int active_count(void)
{
    int n = 0;
    int64_t now = esp_timer_get_time();
    for (int i = 0; i < MAX_TRACKED; i++) {
        if (!s_aircraft[i].active) continue;
        if (now - s_aircraft[i].last_seen_us > 60000000LL) {
            tui_log(4, "LOST     %06lX  (%s)",
                    (unsigned long)s_aircraft[i].icao,
                    s_aircraft[i].callsign[0] ?
                        s_aircraft[i].callsign : "--------");
            s_aircraft[i].active = false;
            audio_play(AUDIO_EVT_LOST_CONTACT);
        } else {
            n++;
        }
    }
    return n;
}


/* ═══════════════════════════════════════════════════════════════════════════
 * TUI LAYOUT CONFIG
 * TERM_W = visible chars INSIDE the two border │ characters.
 * Your terminal window should be at least TERM_W+2 columns wide.
 * ═══════════════════════════════════════════════════════════════════════════ */
#define TERM_W      154
#define RADAR_COLS   41   /* must be odd; right panel visible width          */
#define RADAR_ROWS   20
#define TABLE_ROWS   20
#define LOG_SHOW      7
/* inner border │ takes 1 char, space before panel takes 1 char = 2 overhead */
#define LEFT_W       (TERM_W - RADAR_COLS - 2)
#define EL           "\033[K"

/* ── frame buffer ─────────────────────────────────────────────────────────
 *
 * A frame is ~20 KB, and both costs sitting on top of the bytes are per-call,
 * not per-byte: stdio's stream lock (taken ~4000x per frame, bottoming out in
 * a cross-core spinlock core1's demod loop contends for), and uart_vfs's
 * write(), which loops PER CHARACTER to do the \n -> \r\n translation and
 * calls uart_write_bytes(&c, 1) for each -- a mutex take/give and a ringbuf
 * send per byte, ~20000 per frame.
 *
 * So the frame is assembled here with memcpy and handed to uart_write_bytes()
 * in whole runs, CRLF expanded by hand: stdio and the VFS are both out of the
 * path. This is why neither -O2 nor setvbuf ever moved the number -- the cost
 * was inside prebuilt libc and IDF, and it was never the write count.
 * printf() elsewhere (ESP_LOG, boot) still goes the normal way.
 * ───────────────────────────────────────────────────────────────────────── */

#define CONSOLE_UART  ((uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM)

static char s_fb[4096];
static int  s_fb_len;

/* Frame-cost probe. Reasoning about where the frame time goes has been wrong
 * three times running, so these numbers get measured and shown instead:
 * per-second totals of frames, bytes, time assembling and time writing. */
static uint32_t s_pf_bytes, s_pf_out_us;                        /* this frame */
static uint32_t s_pr_fps, s_pr_bytes, s_pr_out_ms, s_pr_asm_ms; /* last second*/

static void fb_flush(void)
{
    if (s_fb_len <= 0) return;

    int64_t     t0  = esp_timer_get_time();
    const char *p   = s_fb;
    int         rem = s_fb_len;
    s_pf_bytes += (uint32_t)s_fb_len;
    s_fb_len = 0;

    while (rem > 0) {
        const char *nl  = memchr(p, '\n', (size_t)rem);
        int         run = nl ? (int)(nl - p) : rem;
        if (run > 0) uart_write_bytes(CONSOLE_UART, p, (size_t)run);
        if (!nl) break;
        uart_write_bytes(CONSOLE_UART, "\r\n", 2);
        p    = nl + 1;
        rem -= run + 1;
    }
    s_pf_out_us += (uint32_t)(esp_timer_get_time() - t0);
}

static void probe_frame(int64_t now, uint32_t frame_us)
{
    static int64_t  win;
    static uint32_t n, bytes, out_us, tot_us;

    n++;
    bytes  += s_pf_bytes;
    out_us += s_pf_out_us;
    tot_us += frame_us;

    if (!win) { win = now; return; }
    if (now - win < 1000000LL) return;

    s_pr_fps    = n;
    s_pr_bytes  = bytes / n;
    s_pr_out_ms = out_us / 1000;
    s_pr_asm_ms = (tot_us - out_us) / 1000;
    win = now;
    n = bytes = out_us = tot_us = 0;
}

/* Flushing mid-frame when full keeps the buffer small without ever
 * truncating a row -- the flush count was never what cost anything. */
static void fb_room(int need)
{
    if (s_fb_len + need > (int)sizeof(s_fb)) fb_flush();
}

static void fb_puts(const char *s)
{
    int n = (int)strlen(s);
    fb_room(n);
    if (n > (int)sizeof(s_fb)) n = (int)sizeof(s_fb);
    memcpy(s_fb + s_fb_len, s, (size_t)n);
    s_fb_len += n;
}

static void fb_putc(char c)
{
    fb_room(1);
    s_fb[s_fb_len++] = c;
}

static void fb_rep(char c, int n)
{
    while (n > 0) {
        fb_room(1);
        int chunk = (int)sizeof(s_fb) - s_fb_len;
        if (chunk > n) chunk = n;
        memset(s_fb + s_fb_len, c, (size_t)chunk);
        s_fb_len += chunk;
        n        -= chunk;
    }
}

__attribute__((format(printf, 1, 2)))
static void fb_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    fb_room(256);
    int room = (int)sizeof(s_fb) - s_fb_len;
    int n = vsnprintf(s_fb + s_fb_len, (size_t)room, fmt, ap);
    va_end(ap);
    if (n > 0) s_fb_len += (n < room ? n : room - 1);
}

/* ── draw helpers ─────────────────────────────────────────────────────────*/

static void print_bar(const char *color, float value, float max, int width)
{
    int filled = (max > 0.0f) ? (int)(value * width / max) : 0;
    if (filled > width) filled = width;
    if (filled < 0)     filled = 0;
    fb_puts(color);
    for (int i = 0; i < width; i++)
        fb_puts(i < filled ? BLK : BBLK);
    fb_puts(RESET);
}

static void hline(int w)
{
    fb_puts(PH_GRID);
    for (int i = 0; i < w; i++) fb_puts(HL);
    fb_puts(RESET);
}

/* Every content row is:  │  <LEFT_W chars>  │  <RADAR_COLS chars>  │  \n
 * row_begin/end wrap the outer borders only.
 * The inner │ is printed manually between the two panels.               */
static void row_begin(void) { fb_puts(PH_GRID VL RESET); }
static void row_end(void)   { fb_puts(PH_GRID VL EL "\n" RESET); }

static void sep_full(void)
{
    /* ├────────────────────────────────────────────────┤ */
    fb_puts(PH_GRID TR_); hline(TERM_W); fb_puts(TL_ EL "\n" RESET);
}

static void sep_split(void)
{
    /* ├─── left ───┼─── right ───┤ */
    fb_puts(PH_GRID TR_);
    hline(LEFT_W);
    fb_puts(CROSS);
    hline(RADAR_COLS);
    fb_puts(TL_ EL "\n" RESET);
}

static void sp(int n) { fb_rep(' ', n); }

static void fmt_uptime(char *buf, size_t len)
{
    int64_t s = (esp_timer_get_time() - s_start_us) / 1000000LL;
    snprintf(buf, len, "%02d:%02d:%02d",
             (int)(s/3600), (int)((s%3600)/60), (int)(s%60));
}

/* ═══════════════════════════════════════════════════════════════════════════
 * RADAR — real aircraft positions, tight sweep line, two range rings
 * ═══════════════════════════════════════════════════════════════════════════ */

#define RADAR_RANGE_KM  400.0f

/* Terminal cells are roughly this much taller than wide; rings, sweep and blip
 * placement all have to use the same figure or a blip lands off the ring that
 * marks its own range. */
#define RADAR_ASPECT    2.2f
#define RADAR_CX        (RADAR_COLS / 2)
#define RADAR_CY        (RADAR_ROWS / 2)

static int  s_sweep_angle = 0;

/* panel mode: 0 = radar, 1 = waterfall */
static int  s_panel_mode  = 0;

/* waterfall: store last RADAR_ROWS signal-strength rows */
#define WF_BINS  RADAR_COLS
static uint8_t s_wf[RADAR_ROWS][WF_BINS];   /* 0-255 intensity per bin     */
static int     s_wf_row = 0;                 /* next row to write (ring buf)*/

static void latlon_to_xy(float clat, float clon,
                          float alat, float alon,
                          int *ox, int *oy)
{
    float dlat  = alat - clat;
    float dlon  = (alon - clon) * cosf(clat * (float)M_PI / 180.0f);
    float dx_km =  dlon * 111.0f;
    float dy_km = -dlat * 111.0f;
    float rx = (float)RADAR_CX * 0.90f;
    float ry = rx / RADAR_ASPECT;
    *ox = RADAR_CX + (int)(dx_km / RADAR_RANGE_KM * rx);
    *oy = RADAR_CY + (int)(dy_km / RADAR_RANGE_KM * ry);
}

static void render_radar(char panel[RADAR_ROWS][RADAR_COLS + 1])
{
    for (int r = 0; r < RADAR_ROWS; r++) {
        memset(panel[r], ' ', RADAR_COLS);
        panel[r][RADAR_COLS] = '\0';
    }

    /* three range rings — keep outermost inside the panel boundary */
    for (int r = 0; r < RADAR_ROWS; r++) {
        for (int c = 0; c < RADAR_COLS; c++) {
            float dx = (float)(c - RADAR_CX);
            float dy = (float)(r - RADAR_CY) * RADAR_ASPECT;
            float d  = sqrtf(dx*dx + dy*dy);
            float r1 = (float)RADAR_CX * 0.30f;
            float r2 = (float)RADAR_CX * 0.60f;
            float r3 = (float)RADAR_CX * 0.90f;  /* pulled in from border */
            if (fabsf(d - r1) < 0.55f) panel[r][c] = '.';
            if (fabsf(d - r2) < 0.55f) panel[r][c] = '.';
            if (fabsf(d - r3) < 0.55f) panel[r][c] = ':';
        }
    }

    /* crosshair */
    panel[RADAR_CY][RADAR_CX] = '+';
    if (RADAR_CY > 0)             panel[RADAR_CY-1][RADAR_CX] = '|';
    if (RADAR_CY+1 < RADAR_ROWS)  panel[RADAR_CY+1][RADAR_CX] = '|';
    if (RADAR_CX > 1)             panel[RADAR_CY][RADAR_CX-1] = '-';
    if (RADAR_CX+1 < RADAR_COLS)  panel[RADAR_CY][RADAR_CX+1] = '-';

    /* cardinals */
    panel[0][RADAR_CX]            = 'N';
    panel[RADAR_ROWS-1][RADAR_CX] = 'S';
    panel[RADAR_CY][0]            = 'W';
    panel[RADAR_CY][RADAR_COLS-1] = 'E';

    /* tight sweep line — threshold 0.07 rad ≈ 4° */
    float sa = s_sweep_angle * (float)M_PI / 180.0f;
    float sweep_limit = (float)RADAR_CX * 0.90f;
    for (int r = 0; r < RADAR_ROWS; r++) {
        for (int c = 0; c < RADAR_COLS; c++) {
            float dx = (float)(c - RADAR_CX);
            float dy = (float)(r - RADAR_CY) * RADAR_ASPECT;
            float d  = sqrtf(dx*dx + dy*dy);
            if (d > sweep_limit || d < 1.0f) continue;
            float angle = atan2f(dy, dx);
            float diff  = angle - sa;
            while (diff >  (float)M_PI) diff -= 2.0f * (float)M_PI;
            while (diff < -(float)M_PI) diff += 2.0f * (float)M_PI;
            if (fabsf(diff) < 0.07f)
                panel[r][c] = '/';          /* tight bright line */
            else if (fabsf(diff) < 0.18f && panel[r][c] == ' ')
                panel[r][c] = ',';          /* narrow fade */
        }
    }

    /* Aircraft blips, plotted relative to the antenna (menuconfig -> ADS-B
     * Receiver). Kconfig has no float type, so the position arrives as
     * strings. */
    float clat = strtof(CONFIG_ADSB_RX_LAT, NULL);
    float clon = strtof(CONFIG_ADSB_RX_LON, NULL);

    for (int i = 0; i < MAX_TRACKED; i++) {
        if (!s_aircraft[i].active || !s_aircraft[i].pos_valid) continue;
        int px, py;
        latlon_to_xy(clat, clon, s_aircraft[i].lat, s_aircraft[i].lon, &px, &py);
        if (px < 0 || px >= RADAR_COLS || py < 0 || py >= RADAR_ROWS) continue;
        panel[py][px] = '*';
        /* show up to 3 callsign chars after blip */
        if (s_aircraft[i].callsign[0]) {
            for (int ci = 0; ci < 3 && (px + 1 + ci) < RADAR_COLS; ci++) {
                if (!s_aircraft[i].callsign[ci]) break;
                panel[py][px + 1 + ci] = s_aircraft[i].callsign[ci];
            }
        }
    }
}

/* ── waterfall ─────────────────────────────────────────────────────────────
 * Updated from the IQ magnitude data each demodulate() call.
 * We bucket the magnitude vector into WF_BINS bins and push a new row.
 * Uses running peak tracking for auto-gain so narrow signals are visible. */
static uint32_t s_wf_peak = 256;  /* running peak for auto-scale */

void waterfall_push(const uint16_t *mag, int mag_len)
{
    if (!mag || mag_len <= 0) return;

    /* demodulate() runs ~122x/s but the panel only repaints every
     * TUI_REFRESH_MS, so all but one push per frame went straight back out
     * unseen -- and the RADAR_ROWS visible rows spanned 0.16s, which is not a
     * history. One row per repaint costs ~1/60th as much and makes the panel
     * span RADAR_ROWS frames of real time. Kept running even when the
     * waterfall is not the visible panel so switching to it shows data. */
    static int64_t last_push;
    int64_t now = esp_timer_get_time();
    if (now - last_push < TUI_REFRESH_MS * 1000LL) return;
    last_push = now;

    int bin_size = mag_len / WF_BINS;
    if (bin_size < 1) bin_size = 1;
    int row = s_wf_row % RADAR_ROWS;

    /* first pass: compute bins and track peak */
    uint32_t bins[WF_BINS];
    uint32_t frame_peak = 1;
    for (int b = 0; b < WF_BINS; b++) {
        int start = b * bin_size;
        int end   = start + bin_size;
        if (end > mag_len) end = mag_len;
        uint32_t sum = 0;
        for (int j = start; j < end; j++) sum += mag[j];
        bins[b] = sum / (uint32_t)(end - start);
        if (bins[b] > frame_peak) frame_peak = bins[b];
    }

    /* IIR track peak with slow decay so display stays responsive */
    if (frame_peak > s_wf_peak)
        s_wf_peak = frame_peak;
    else
        s_wf_peak = s_wf_peak - (s_wf_peak >> 6) + (frame_peak >> 6);
    if (s_wf_peak < 256) s_wf_peak = 256;

    /* second pass: normalize to 0-255 using tracked peak */
    for (int b = 0; b < WF_BINS; b++) {
        uint32_t v = bins[b] * 255 / s_wf_peak;
        if (v > 255) v = 255;
        s_wf[row][b] = (uint8_t)v;
    }
    s_wf_row++;
}

static void render_waterfall(char panel[RADAR_ROWS][RADAR_COLS + 1])
{
    /* We still fill the text panel with density chars for the non-coloured path,
     * but the real display comes from the coloured render in tui_draw().       */
    static const char dens[] = " .,:;=+*#%@";
    for (int r = 0; r < RADAR_ROWS; r++) {
        int src = ((s_wf_row - 1 - r) % RADAR_ROWS + RADAR_ROWS) % RADAR_ROWS;
        panel[r][RADAR_COLS] = '\0';
        for (int c = 0; c < RADAR_COLS; c++) {
            int v = s_wf[src][c];
            int idx = v * (int)(sizeof(dens) - 2) / 255;
            if (idx < 0) idx = 0;
            if (idx > (int)(sizeof(dens) - 2)) idx = (int)(sizeof(dens) - 2);
            panel[r][c] = dens[idx];
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * CPU LOAD / HEAP
 *
 * Per-core busy percentage, from how much of each interval that core's IDLE
 * task got. IDF's FreeRTOS has no per-task run-time getter, so the whole task
 * list has to be walked, and uxTaskGetSystemState() holds a cross-core
 * spinlock for the walk -- hence 1 Hz rather than once per frame.
 * ═══════════════════════════════════════════════════════════════════════════ */

#define CPU_STAT_MAX_TASKS  32

typedef struct {
    TaskHandle_t hdl;
    char         name[12];
    int8_t       core;          /* -1 = no affinity */
    uint32_t     last_rt;
    int          pct;           /* share of ONE core, not of both */
    uint32_t     stack_hwm;
} task_stat_t;

static int         s_cpu_busy[2] = { -1, -1 };  /* -1 until first interval */
static uint32_t    s_heap_free, s_heap_min, s_heap_big;
static task_stat_t s_tstat[CPU_STAT_MAX_TASKS];
static int         s_tstat_n;

static void stats_sample(int64_t now)
{
    static TaskStatus_t st[CPU_STAT_MAX_TASKS];
    static TaskHandle_t idle_hdl[2];
    static uint32_t     last_idle[2];
    static int64_t      last_us;

    if (last_us && (now - last_us) < 1000000LL) return;

    if (!idle_hdl[0]) {
        idle_hdl[0] = xTaskGetIdleTaskHandleForCore(0);
        idle_hdl[1] = xTaskGetIdleTaskHandleForCore(1);
    }

    uint32_t    total;
    UBaseType_t n = uxTaskGetSystemState(st, CPU_STAT_MAX_TASKS, &total);
    if (n == 0) return;     /* array too small -- raise CPU_STAT_MAX_TASKS */

    int64_t span = now - last_us;

    uint32_t idle[2] = { 0, 0 };
    for (UBaseType_t i = 0; i < n; i++)
        for (int c = 0; c < 2; c++)
            if (st[i].xHandle == idle_hdl[c])
                idle[c] = st[i].ulRunTimeCounter;

    if (last_us && span > 0) {
        for (int c = 0; c < 2; c++) {
            /* The counter is esp_timer microseconds truncated to 32 bits, so
             * elapsed wall time is exactly one core's budget and the unsigned
             * delta stays correct across the ~71 min wrap. */
            int64_t busy = span - (int64_t)(idle[c] - last_idle[c]);
            if (busy < 0)    busy = 0;
            if (busy > span) busy = span;
            s_cpu_busy[c] = (int)(busy * 100 / span);
        }
    }
    last_idle[0] = idle[0];
    last_idle[1] = idle[1];

    /* Per-task deltas. Matching on the handle rather than the slot index --
     * uxTaskGetSystemState() gives no stable ordering, and tasks come and go
     * (rtlsdr_setup_task is transient). An unmatched handle just reads 0% for
     * one interval. */
    task_stat_t prev[CPU_STAT_MAX_TASKS];
    int         prev_n = s_tstat_n;
    memcpy(prev, s_tstat, sizeof(prev));

    for (UBaseType_t i = 0; i < n; i++) {
        task_stat_t *t = &s_tstat[i];
        t->hdl = st[i].xHandle;
        snprintf(t->name, sizeof(t->name), "%s",
                 st[i].pcTaskName ? st[i].pcTaskName : "?");
#if ( configTASKLIST_INCLUDE_COREID == 1 )
        t->core = (st[i].xCoreID == tskNO_AFFINITY) ? -1 : (int8_t)st[i].xCoreID;
#else
        t->core = -1;
#endif
        t->stack_hwm = st[i].usStackHighWaterMark;
        t->pct       = 0;
        for (int p = 0; p < prev_n; p++) {
            if (prev[p].hdl != t->hdl) continue;
            if (last_us && span > 0) {
                uint32_t d = st[i].ulRunTimeCounter - prev[p].last_rt;
                t->pct = (int)((int64_t)d * 100 / span);
                if (t->pct > 100) t->pct = 100;
            }
            break;
        }
        t->last_rt = st[i].ulRunTimeCounter;
    }
    s_tstat_n = (int)n;

    /* insertion sort, busiest first -- n is ~16 */
    for (int i = 1; i < s_tstat_n; i++) {
        task_stat_t k = s_tstat[i];
        int j = i - 1;
        while (j >= 0 && s_tstat[j].pct < k.pct) { s_tstat[j + 1] = s_tstat[j]; j--; }
        s_tstat[j + 1] = k;
    }

    last_us = now;

    s_heap_free = (uint32_t)esp_get_free_heap_size();
    s_heap_min  = (uint32_t)esp_get_minimum_free_heap_size();
    s_heap_big  = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
}

/* Right-hand panel, mode 2: who is actually eating each core.
 * STACK is the unused-stack high-water margin, in bytes (ESP-IDF's
 * usStackHighWaterMark is bytes, unlike upstream FreeRTOS's words). */
static void render_tasks(char panel[RADAR_ROWS][RADAR_COLS + 1])
{
    for (int r = 0; r < RADAR_ROWS; r++) {
        memset(panel[r], ' ', RADAR_COLS);
        panel[r][RADAR_COLS] = '\0';
    }
    snprintf(panel[0], RADAR_COLS + 1, " %-11s %4s %4s %8s", "TASK", "CPU", "CORE", "STACK");

    /* last 3 rows belong to the frame probe below */
    for (int r = 1; r < RADAR_ROWS - 3 && r - 1 < s_tstat_n; r++) {
        task_stat_t *t = &s_tstat[r - 1];
        char core[4];
        if (t->core < 0) snprintf(core, sizeof(core), "-");
        else             snprintf(core, sizeof(core), "%d", t->core);
        snprintf(panel[r], RADAR_COLS + 1, " %-11s %3d%% %4s %8lu",
                 t->name, t->pct, core, (unsigned long)t->stack_hwm);
    }

    /* Declared here rather than included: esp_libusb.h carries its own,
     * unrelated struct class_driver_t that clashes with this file's. */
    extern uint64_t esp_libusb_stream_dropped(void);

    snprintf(panel[RADAR_ROWS - 3], RADAR_COLS + 1, " FRAME %5luB  %2lu/s",
             (unsigned long)s_pr_bytes, (unsigned long)s_pr_fps);
    snprintf(panel[RADAR_ROWS - 2], RADAR_COLS + 1, " asm %3lums/s   out %3lums/s",
             (unsigned long)s_pr_asm_ms, (unsigned long)s_pr_out_ms);
    snprintf(panel[RADAR_ROWS - 1], RADAR_COLS + 1, " USB drop %llu",
             (unsigned long long)esp_libusb_stream_dropped());

    /* snprintf NUL-terminates early; repaint the tail as spaces so the panel
     * stays a fixed-width block (the TUI never clears, it overwrites). */
    for (int r = 0; r < RADAR_ROWS; r++) {
        int len = (int)strlen(panel[r]);
        for (int c = len; c < RADAR_COLS; c++) panel[r][c] = ' ';
        panel[r][RADAR_COLS] = '\0';
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TUI DRAW
 * ═══════════════════════════════════════════════════════════════════════════ */

static void tui_draw(void)
{
    int64_t now = esp_timer_get_time();
    if ((now - s_last_draw) < (TUI_REFRESH_MS * 1000LL)) return;
    s_last_draw = now;

    /* The frame goes straight to the UART, so anything another task left in
     * stdout's buffer has to get out first or it lands mid-frame. */
    fflush(stdout);

    s_pf_bytes = s_pf_out_us = 0;

    stats_sample(now);
    s_dirty     = false;

    if (now - s_rate_ts >= 1000000LL) {
        s_msg_rate   = s_msg_bucket;
        s_msg_bucket = 0;
        s_rate_ts    = now;
    }

    s_sweep_angle = (s_sweep_angle + 4) % 360;

    int ac = active_count();

    /* Demodulator-side counters, not callback-side: on_msg only ever sees
     * frames whose CRC already passed, so anything derived there reads 100%
     * by construction. FIX is the share of good frames that needed single-bit
     * correction -- it climbs before ERR does as the signal degrades. */
    unsigned long good  = state.stat_goodcrc;
    unsigned long total = good + state.stat_badcrc;
    float dp = total > 0 ? (good * 100.0f / total) : 0.0f;
    float cp = total > 0 ? (state.stat_badcrc * 100.0f / total) : 0.0f;
    float fp = good  > 0 ? (state.stat_fixed  * 100.0f / good)  : 0.0f;
    s_decode_smooth += (dp - s_decode_smooth) * 0.12f;
    s_crc_smooth    += (cp - s_crc_smooth)    * 0.12f;
    s_fix_smooth    += (fp - s_fix_smooth)    * 0.12f;

    char uptime[12];
    fmt_uptime(uptime, sizeof(uptime));

    fb_printf("\033[H");

    /* ┌─────────────────────────────────────────────────┐ */
    fb_printf(PH_GRID TL); hline(TERM_W); fb_printf(TR EL "\n" RESET);

    /* header — single full-width row */
    row_begin();
    {
        int n = (int)strlen("  ATC TERMINAL  //  ESP32-P4 ADS-B RECEIVER"
                            "  //  1090.000 MHz  //  2 MSPS");
        fb_printf(PH_HI BOLD "  ATC TERMINAL" RESET
               PH_GRID "  //  " RESET PH_SCAN "ESP32-P4 ADS-B RECEIVER" RESET
               PH_GRID "  //  " RESET PH_HI "1090.000 MHz" RESET
               PH_GRID "  //  " RESET PH_MID "2 MSPS" RESET);
        sp(TERM_W - n);
    }
    row_end();
    sep_full();

    /* status */
    row_begin();
    {
        char vol_str[8];
        snprintf(vol_str, sizeof(vol_str), "%3d%%", s_muted ? 0 : s_volume);
        char cpu0[12], cpu1[12];   /* %3d of an int can still be 11 chars */
        if (s_cpu_busy[0] < 0) { strcpy(cpu0, " --"); strcpy(cpu1, " --"); }
        else {
            snprintf(cpu0, sizeof(cpu0), "%3d", s_cpu_busy[0]);
            snprintf(cpu1, sizeof(cpu1), "%3d", s_cpu_busy[1]);
        }
        /* compute visible width by snprintf to scratch buffer */
        char scratch[256];
        int n = snprintf(scratch, sizeof(scratch),
            "  UP %-9s  ACFT %-3d  MSG/S %-5d  TOTAL %-8d"
            "  DEC %5.1f%%  ERR %5.1f%%  FIX %5.1f%%  VOL %s"
            "  CPU0 %s%%  CPU1 %s%%  HEAP %3luK/%3luK",
            uptime, ac, s_msg_rate, s_msg_count,
            s_decode_smooth, s_crc_smooth, s_fix_smooth, vol_str,
            cpu0, cpu1,
            (unsigned long)(s_heap_free / 1024), (unsigned long)(s_heap_min / 1024));
        if (n > TERM_W) n = TERM_W;
        fb_printf(PH_DIM "  UP " RESET PH_HI "%-9s" RESET
               PH_DIM "  ACFT " RESET PH_HI BOLD "%-3d" RESET
               PH_DIM "  MSG/S " RESET PH_SCAN "%-5d" RESET
               PH_DIM "  TOTAL " RESET PH_MID "%-8d" RESET
               PH_DIM "  DEC " RESET PH_HI "%5.1f%%" RESET
               PH_DIM "  ERR " RESET AC_AMBER "%5.1f%%" RESET
               PH_DIM "  FIX " RESET PH_MID "%5.1f%%" RESET
               PH_DIM "  VOL " RESET "%s%s" RESET
               PH_DIM "  CPU0 " RESET "%s%s%%" RESET
               PH_DIM "  CPU1 " RESET "%s%s%%" RESET
               PH_DIM "  HEAP " RESET PH_MID "%3luK" RESET
               PH_DIM "/" RESET "%s%3luK" RESET,
               uptime, ac, s_msg_rate, s_msg_count,
               s_decode_smooth, s_crc_smooth, s_fix_smooth,
               s_muted ? AC_RED : PH_HI, vol_str,
               s_cpu_busy[0] > 80 ? AC_AMBER : PH_HI, cpu0,
               s_cpu_busy[1] > 80 ? AC_AMBER : PH_HI, cpu1,
               (unsigned long)(s_heap_free / 1024),
               s_heap_big < 32768 ? AC_AMBER : PH_DIM,
               (unsigned long)(s_heap_min / 1024));
        sp(TERM_W - n);
    }
    row_end();

    /* bars */
    row_begin();
    {
        /* overhead: "  DECODE [" = 10, "] ERR [" = 7, "]" = 1 → 18 total */
        int bar_w = (TERM_W - 18) / 2;
        if (bar_w < 10) bar_w = 10;
        int n = 10 + bar_w + 7 + bar_w + 1;
        fb_printf(PH_DIM "  DECODE [" RESET);
        print_bar(PH_HI, s_decode_smooth, 100.0f, bar_w);
        fb_printf(PH_DIM "] ERR [" RESET);
        print_bar(AC_AMBER, s_crc_smooth, 100.0f, bar_w);
        fb_printf(PH_DIM "]" RESET);
        sp(TERM_W - n);
    }
    row_end();

    /* ├─── left ───┼─── right ───┤ */
    sep_split();

    /* panel header row */
    row_begin();
    {
        /* left: column labels */
        char hdr[256];
        int n = snprintf(hdr, sizeof(hdr),
            "  %-8s  %-9s  %8s  %7s  %-5s  %9s  %9s  %5s  %4s ",
            "ICAO","CALLSIGN","ALT ft","SPD kt","HDG","LAT","LON","V/S","MSGS");
        fb_printf(PH_DIM "%s" RESET, hdr);
        sp(LEFT_W - n);
        /* inner border */
        fb_printf(PH_GRID VL RESET);
        /* right: panel title */
        const char *title = s_panel_mode == 0 ? " RADAR  [R]"
                          : s_panel_mode == 1 ? " WFALL  [R]" : " TASKS  [R]";
        fb_printf(PH_DIM " %-*s" RESET, RADAR_COLS - 1, title);
    }
    row_end();
    sep_split();

    /* render panel */
    char panel[RADAR_ROWS][RADAR_COLS + 1];
    if (s_panel_mode == 0)      render_radar(panel);
    else if (s_panel_mode == 1) render_waterfall(panel);
    else                        render_tasks(panel);

    /* aircraft rows + panel */
    int ac_idx = 0;
    for (int row = 0; row < TABLE_ROWS; row++) {
        aircraft_t *a = NULL;
        for (; ac_idx < MAX_TRACKED; ac_idx++) {
            if (s_aircraft[ac_idx].active) { a = &s_aircraft[ac_idx++]; break; }
        }

        row_begin();

        if (a) {
            const char *alt_col = PH_MID;
            char vs_plain[16] = "  --";
            char vs_col[64]   = "  --";
            if (a->vert_rate > 200) {
                alt_col = PH_HI;
                snprintf(vs_plain, sizeof(vs_plain), "+%d", a->vert_rate);
                snprintf(vs_col,   sizeof(vs_col),   PH_HI "+%d" RESET, a->vert_rate);
            } else if (a->vert_rate < -200) {
                alt_col = AC_AMBER;
                snprintf(vs_plain, sizeof(vs_plain), "%d", a->vert_rate);
                snprintf(vs_col,   sizeof(vs_col),   AC_AMBER "%d" RESET, a->vert_rate);
            }
            char lat_s[12] = "   ------";
            char lon_s[12] = "   ------";
            if (a->pos_valid) {
                snprintf(lat_s, sizeof(lat_s), "%+9.4f", a->lat);
                snprintf(lon_s, sizeof(lon_s), "%+9.4f", a->lon);
            }
            const char *dirs[] = {"N","NE","E","SE","S","SW","W","NW"};
            int didx = (int)(((float)a->heading + 22.5f) / 45.0f) % 8;
            const char *cs = a->callsign[0] ? a->callsign : "--------";

            /* measure visible width of left panel content */
            int vis = 2+8+2+9+2+8+2+7+2+(int)strlen(dirs[didx])+1+3
                      +2+9+2+9+2+(int)strlen(vs_plain)+2+4+1;

            fb_printf("  " AC_CYAN "%-8lX" RESET
                   "  " PH_HI   "%-9s" RESET
                   "  " "%s%8d" RESET
                   "  " PH_MID  "%7d" RESET
                   "  " PH_MID  "%s" RESET PH_DIM "-" RESET PH_HI "%03d" RESET
                   "  " PH_DIM  "%9s" RESET
                   "  " PH_DIM  "%9s" RESET
                   "  " "%s"
                   "  " PH_DIM  "%4d" RESET " ",
                   (unsigned long)a->icao, cs,
                   alt_col, a->altitude,
                   a->velocity,
                   dirs[didx], a->heading,
                   lat_s, lon_s,
                   vs_col,
                   a->msg_count);
            sp(LEFT_W - vis);
        } else {
            fb_printf(PH_DIM "  ---" RESET);
            sp(LEFT_W - 5);
        }

        /* inner border + panel line */
        fb_printf(PH_GRID VL RESET);
        if (row < RADAR_ROWS) {
            if (s_panel_mode == 0) {
                /* radar: colour each char individually */
                for (int c = 0; c < RADAR_COLS; c++) {
                    char ch = panel[row][c];
                    if      (ch == '*')                                        fb_printf(PH_HI BOLD "%c" RESET, ch);
                    else if (ch == '/')                                        fb_printf(PH_SCAN "%c" RESET, ch);
                    else if (ch == ',')                                        fb_printf(PH_DIM "%c" RESET, ch);
                    else if (ch=='N'||ch=='S'||ch=='E'||ch=='W'||ch=='+'||ch=='|') fb_printf(PH_MID "%c" RESET, ch);
                    else if (ch == ':')                                        fb_printf(PH_GRID "%c" RESET, ch);
                    else if (ch == '-' || ch == '.')                           fb_printf(PH_DIM "%c" RESET, ch);
                    else if (ch != ' ')                                        fb_printf(PH_MID "%c" RESET, ch);
                    else fb_putc(' ');
                }
            } else if (s_panel_mode == 2) {
                /* tasks: plain fixed-width text, one write for the whole row */
                fb_printf("%s%s" RESET, row == 0 ? PH_DIM : PH_MID, panel[row]);
            } else {
                /* waterfall: heat-map colour gradient green→amber→white */
                int src = ((s_wf_row - 1 - row) % RADAR_ROWS + RADAR_ROWS) % RADAR_ROWS;
                for (int c = 0; c < RADAR_COLS; c++) {
                    int v = s_wf[src][c];
                    int rr, gg, bb;
                    if (v < 85) {
                        /* black → dark green */
                        rr = 0; gg = 20 + v * 2; bb = 0;
                    } else if (v < 170) {
                        /* dark green → bright green/amber */
                        int t = v - 85;
                        rr = t * 2; gg = 190 + t; bb = 0;
                    } else {
                        /* amber → white-hot */
                        int t = v - 170;
                        rr = 170 + t; gg = 255; bb = t * 3;
                        if (bb > 255) bb = 255;
                        if (rr > 255) rr = 255;
                    }
                    /* use block char for filled look */
                    char ch = panel[row][c];
                    if (ch == ' ' || ch == '.') ch = ' ';
                    fb_printf("\033[38;2;%d;%d;%dm%c" RESET, rr, gg, bb, ch);
                }
            }
        } else {
            sp(RADAR_COLS);
        }
        row_end();
    }

    sep_split();

    /* event log */
    row_begin();
    fb_printf(PH_DIM "  EVENT LOG"); sp(LEFT_W - 11);
    fb_printf(PH_GRID VL RESET); sp(RADAR_COLS);
    row_end();

    static const char *log_cols[] = { PH_DIM, PH_HI, AC_AMBER, AC_CYAN, AC_RED };
    for (int i = LOG_SHOW - 1; i >= 0; i--) {
        int idx = (s_log_head - 1 - i + LOG_LINES * 2) % LOG_LINES;
        row_begin();
        fb_printf("  ");
        if (s_log[idx].text[0]) {
            int tlen = (int)strnlen(s_log[idx].text, sizeof(s_log[0].text));
            fb_printf("%s%s" RESET, log_cols[s_log[idx].color], s_log[idx].text);
            sp(LEFT_W - 2 - tlen);
        } else {
            fb_printf(PH_DIM "~" RESET); sp(LEFT_W - 3);
        }
        /* right side of log rows: blank panel column */
        fb_printf(PH_GRID VL RESET); sp(RADAR_COLS);
        row_end();
    }

    /* └─────────────────────────────────────────────────┘ */
    fb_printf(PH_GRID BL); hline(LEFT_W); fb_printf(T_UP); hline(RADAR_COLS);
    fb_printf(BR EL "\n" RESET);
    fb_printf(PH_DIM "  R828D  " PH_GRID "|" RESET
           PH_DIM "  RAFAEL MICRO  " PH_GRID "|" RESET
           PH_DIM "  ctrl+] EXIT  " PH_GRID "|" RESET
           PH_DIM "  [M]UTE  " PH_GRID "|" RESET
           PH_DIM "  [+/-] VOL  " PH_GRID "|" RESET
           PH_DIM "  [R] RADAR/WFALL/TASKS" EL "\n" RESET);

    fb_flush();
    probe_frame(now, (uint32_t)(esp_timer_get_time() - now));
}


/* ═══════════════════════════════════════════════════════════════════════════
 * DEMODULATE  (also feeds waterfall)
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ═══════════════════════════════════════════════════════════════════════════
 * ON_MSG CALLBACK
 * ═══════════════════════════════════════════════════════════════════════════ */

static void on_msg(mode_s_t *self, struct mode_s_msg *mm)
{
    s_msg_count++;
    s_msg_bucket++;
    s_dirty = true;

    uint32_t icao = ((uint32_t)mm->aa1 << 16) |
                    ((uint32_t)mm->aa2 <<  8) |
                     (uint32_t)mm->aa3;

    /* Unreachable while check_crc is set, but the ICAO of a bad-CRC frame is
     * itself garbage -- it must never reach find_or_create(). */
    if (!mm->crcok) return;

    aircraft_t *a = find_or_create(icao);
    if (!a) return;

    a->last_seen_us = esp_timer_get_time();
    a->msg_count++;

    if (mm->flight[0]) {
        strncpy(a->callsign, mm->flight, 8);
        a->callsign[8] = '\0';
        for (int i = 7; i >= 0 && a->callsign[i] == ' '; i--)
            a->callsign[i] = '\0';
        tui_log(2, "IDENT    %06lX  %s", (unsigned long)icao, a->callsign);
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
        if (cpr_decode(a) && !was_valid) {
            tui_log(3, "FIX      %06lX  %+.4f  %+.4f",
                    (unsigned long)icao, a->lat, a->lon);
            audio_play(AUDIO_EVT_POSITION);
        }
    }

    if (mm->msgtype == 17) {
        if (mm->metype >= 9 && mm->metype <= 18 && mm->altitude)
            tui_log(0, "ALT      %06lX  %d ft", (unsigned long)icao, a->altitude);
        else if (mm->metype >= 19 && mm->metype <= 22 && mm->velocity)
            tui_log(0, "VEL      %06lX  %d kt  hdg=%d  vs=%d",
                    (unsigned long)icao, a->velocity, a->heading, a->vert_rate);
    }
}

/* forward declaration satisfied above — no duplicate needed */

/* Sized for the one caller's fixed DEFAULT_BUF_LENGTH chunk. Static rather than
 * malloc'd because this sits on the 4 MB/s IQ hot path, and safe only because
 * adsb_rx_task is the sole caller -- a second demodulating task needs its own. */
static uint16_t s_mag[DEFAULT_BUF_LENGTH / 2];

void demodulate(uint8_t *source, int length)
{
    if (!source || length <= 0) return;
    if (length > DEFAULT_BUF_LENGTH) length = DEFAULT_BUF_LENGTH;
    int mag_len = length / 2;
    mode_s_compute_magnitude_vector(source, s_mag, length);
    waterfall_push(s_mag, mag_len);     /* feed real IQ energy into wfall  */
    mode_s_detect(&state, s_mag, mag_len, on_msg);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TUI TASK
 *
 * tui_draw() blocks in printf until the entire frame has clocked out of the
 * console UART -- a full 154-column frame is ~20 KB of UTF-8 box drawing and
 * ANSI colour. Drawing it from adsb_rx_task meant the IQ ring overflowed for
 * the whole duration of every frame, which at the old hardcoded 115200 baud
 * was over a second. It reads the aircraft table without a lock: a torn frame
 * is cosmetic, a starved demod loop is not.
 * ═══════════════════════════════════════════════════════════════════════════ */

static void tui_task(void *arg)
{
    for (;;) {
        tui_draw();     /* self-rate-limits to TUI_REFRESH_MS */
        vTaskDelay(pdMS_TO_TICKS(TUI_REFRESH_MS / 3));
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ADSB RX TASK  — also polls UART0 for keystrokes
 * ═══════════════════════════════════════════════════════════════════════════ */

void adsb_rx_task(void *arg)
{
    s_start_us = esp_timer_get_time();
    s_rate_ts  = s_start_us;

    tui_log(1, "INIT     adsb_rx running on CPU1");

    {
        enum rtlsdr_tuner tt = rtlsdr_get_tuner_type(rtldev);
        int      lock = rtlsdr_get_tuner_pll_locked(rtldev);
        uint32_t fc   = rtlsdr_get_center_freq(rtldev);
        uint32_t sr   = rtlsdr_get_sample_rate(rtldev);
        uint32_t ref  = rtlsdr_get_tuner_xtal(rtldev);

        tui_log(lock == 1 ? 1 : 4,
                "INIT     %s %s  %lu.%03lu MHz  %lu.%03lu MSPS  ref %lu.%03lu MHz",
                tt == RTLSDR_TUNER_R828D ? "R828D"
                    : tt == RTLSDR_TUNER_R820T ? "R820T" : "tuner",
                lock == 1 ? "locked" : lock == 0 ? "PLL UNLOCKED" : "lock unknown",
                (unsigned long)(fc / 1000000), (unsigned long)(fc % 1000000 / 1000),
                (unsigned long)(sr / 1000000), (unsigned long)(sr % 1000000 / 1000),
                (unsigned long)(ref / 1000000), (unsigned long)(ref % 1000000 / 1000));
    }

    uint8_t *buffer = malloc(DEFAULT_BUF_LENGTH);
    if (!buffer) { tui_log(4, "OOM rx buffer"); vTaskDelete(NULL); return; }

    bool full_buffer = false;
    mode_s_init(&state);

    /* configure UART0 for non-blocking key reads. The baud MUST stay at the
     * console's configured rate -- this is the same UART stdout goes out on,
     * so hardcoding a slower one both garbles the monitor and throttles
     * tui_draw() to a crawl (a full 154-col frame is ~20 KB of UTF-8+ANSI). */
    uart_config_t uart_cfg = {
        .baud_rate  = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_0, &uart_cfg);
    /* Console stdout otherwise goes through uart_vfs's default tx_func, which
     * busy-spins on the TX FIFO one byte at a time -- it never yields, so a
     * slow/heavy tui_draw() can starve IDLE1 long enough to trip the task
     * watchdog. Routing stdout through this driver's buffered+interrupt TX
     * makes printf() a semaphore-blocked (yielding) call instead. */
    uart_driver_install(UART_NUM_0, 256, 4096, 0, NULL, 0);
    uart_vfs_dev_use_driver(UART_NUM_0);

    printf(CLS);
    s_tui_active = true;
    xTaskCreatePinnedToCore(tui_task, "tui", 6144, NULL, 2, NULL, 0);

    bool stream_started = (rtlsdr_stream_start(rtldev) == 0);
    if (!stream_started) tui_log(4, "STREAM   start failed, retrying");

    int64_t last_yield = esp_timer_get_time();

    while (true) {
        /* ── non-blocking keyread ── */
        uint8_t key = 0;
        if (uart_read_bytes(UART_NUM_0, &key, 1, 0) > 0) {
            switch (key) {
                case 'r': case 'R':
                    s_panel_mode = (s_panel_mode + 1) % 3;
                    tui_log(3, "PANEL    switched to %s",
                            s_panel_mode == 0 ? "RADAR"
                          : s_panel_mode == 1 ? "WATERFALL" : "TASKS");
                    s_dirty = true;
                    break;
                case 'm': case 'M':
                    s_muted = !s_muted;
                    tui_log(2, "AUDIO    %s", s_muted ? "muted" : "unmuted");
                    s_dirty = true;
                    break;
                case '+': case '=':
                    s_volume += 10;
                    if (s_volume > 100) s_volume = 100;
                    s_dirty = true;
                    break;
                case '-':
                    s_volume -= 10;
                    if (s_volume < 0) s_volume = 0;
                    s_dirty = true;
                    break;
                default: break;
            }
        }

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
                tui_log(3, "SHORT READ  got=%d", got);
            }
        }

        /* While the ring has data this loop never blocks, so IDLE1 -- and with
         * it the task watchdog and deleted-task cleanup -- only runs if core1
         * is handed back deliberately. One tick per half second is ~2% of the
         * IQ budget; taskYIELD() would not do, IDLE is lower priority. */
        int64_t now = esp_timer_get_time();
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
        tui_log(4, "USB      pipe wedged, resetting RTL interface");
        if (rtldev && rtlsdr_reset_interface(rtldev) == 0) {
            tui_log(2, "USB      interface reset OK");
        } else {
            tui_log(4, "USB      interface reset FAILED - replug dongle");
        }
    }
}

void adsb_request_recover(void)
{
    if (s_recover_task_hdl) xTaskNotifyGive(s_recover_task_hdl);
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
