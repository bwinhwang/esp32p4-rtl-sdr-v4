/*
 * OTA over the config-page httpd. This project's OTA use case is narrow and
 * that is deliberate: a developer builds a new image on the host and pushes
 * it to a board already reachable on a trusted LAN/SoftAP --
 *
 *   curl -H "Expect:" --data-binary @build/usb_host_lib_example.bin http://<ip>/ota
 *
 * -- not a public update service. So there is no auth, no signed images, no
 * HTTPS client (which would drag in a second TLS stack alongside esp_hosted's
 * and libssh's). If this board ever leaves a hobbyist's own network, that
 * calculus changes and this file needs a token check before anything else.
 *
 * The `-H "Expect:"` is not cosmetic. curl attaches `Expect: 100-continue` to
 * any body over 1KB, esp_http_server has no 100-continue handling whatsoever,
 * and curl therefore sits out its own one-second expect timeout before sending
 * a byte of the image.
 */

#include <stdlib.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "shell.h"       /* sys_log() */
#include "ota.h"

static const char *TAG = "ota";

#define OTA_CONFIRM_DELAY_MS  20000
#define OTA_RECV_BUF          4096
/* Consecutive httpd_req_recv() timeouts to tolerate before giving up on an
 * upload. web_config.c leaves recv_wait_timeout at HTTPD_DEFAULT_CONFIG()'s
 * 5 s, so this is ~30 s of total silence. */
#define OTA_RECV_IDLE_MAX     6

/* Runs once, OTA_CONFIRM_DELAY_MS after boot -- long enough to be past USB
 * enumeration and network bring-up, the two things most likely to wedge a
 * bad build. Anything that panics or watchdogs before this fires never
 * confirms, and BOOTLOADER_APP_ROLLBACK_ENABLE reverts to the previous slot
 * on the very next boot with no operator action. If the running image is not
 * pending (a direct `idf.py flash`, or a slot already confirmed earlier),
 * esp_ota_get_state_partition() will not report PENDING_VERIFY and this is a
 * no-op. */
static void confirm_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(OTA_CONFIRM_DELAY_MS));

    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(run, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
        sys_log(1, "OTA      %s confirmed valid, rollback cancelled", run->label);
    }
    vTaskDelete(NULL);
}

void ota_init(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    const esp_app_desc_t *app  = esp_app_get_description();
    ESP_LOGI(TAG, "running from %s, app version %s", run->label, app->version);

    /* Priority 2: below every network/console task, this has nowhere to be
     * in a hurry to get to. */
    xTaskCreate(confirm_task, "ota_confirm", 3072, NULL, 2, NULL);
}

/* ── POST /ota ────────────────────────────────────────────────────────────
 * Raw image bytes, no multipart -- see the file header for why. OTA_SIZE_
 * UNKNOWN tells esp_ota_begin() to erase the whole target partition up front
 * rather than sector-by-sector as data arrives: a couple of seconds slower
 * against a 2MB slot, but immune to a Content-Length that undercounts, which
 * with the exact-size path silently truncates the image instead of failing
 * esp_ota_end()'s validation. */
static esp_err_t ota_post(httpd_req_t *req)
{
    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (!target) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no OTA partition");
        return ESP_FAIL;
    }

    esp_ota_handle_t handle;
    esp_err_t err = esp_ota_begin(target, OTA_SIZE_UNKNOWN, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(err));
        /* Under BOOTLOADER_APP_ROLLBACK_ENABLE, esp_ota_begin() refuses
         * outright while the running image is still PENDING_VERIFY -- which is
         * exactly the window the "wrong binary, push it again" reflex lands
         * in, and the bare error code name says nothing about waiting. */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            err == ESP_ERR_OTA_ROLLBACK_INVALID_STATE
                                ? "running image not confirmed yet, retry in a few seconds"
                                : esp_err_to_name(err));
        return ESP_FAIL;
    }

    char *buf = malloc(OTA_RECV_BUF);
    if (!buf) {
        esp_ota_abort(handle);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no mem");
        return ESP_FAIL;
    }

    int    remaining = req->content_len;
    size_t total     = 0;
    int    idle      = 0;
    sys_log(1, "OTA      receiving %d bytes -> %s", remaining, target->label);

    while (remaining > 0) {
        int want = remaining < OTA_RECV_BUF ? remaining : OTA_RECV_BUF;
        int got  = httpd_req_recv(req, buf, want);
        if (got == HTTPD_SOCK_ERR_TIMEOUT) {
            /* A client that vanishes without a FIN -- a laptop carrying the
             * push out of SoftAP range -- leaves this socket open forever:
             * nothing here ever sends, so TCP never probes. Retrying without a
             * bound would park the single httpd task in this handler for good,
             * taking the config page down with it and, worse, taking down the
             * one remote path back into a headless board. */
            if (++idle >= OTA_RECV_IDLE_MAX) {
                free(buf);
                esp_ota_abort(handle);
                sys_log(4, "OTA      upload stalled after %u bytes, aborted", (unsigned)total);
                httpd_resp_send_err(req, HTTPD_408_REQ_TIMEOUT, "upload stalled");
                return ESP_FAIL;
            }
            continue;
        }
        idle = 0;
        if (got <= 0) {
            free(buf);
            esp_ota_abort(handle);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "recv failed");
            return ESP_FAIL;
        }
        err = esp_ota_write(handle, buf, got);
        if (err != ESP_OK) {
            free(buf);
            esp_ota_abort(handle);
            ESP_LOGE(TAG, "esp_ota_write: %s", esp_err_to_name(err));
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
            return ESP_FAIL;
        }
        total     += got;
        remaining -= got;
    }
    free(buf);

    err = esp_ota_end(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            err == ESP_ERR_OTA_VALIDATE_FAILED ? "image validation failed"
                                                                : esp_err_to_name(err));
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(target);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
        return ESP_FAIL;
    }

    char msg[80];
    int  n = snprintf(msg, sizeof(msg), "ok, %u bytes -> %s, rebooting\n",
                       (unsigned)total, target->label);
    httpd_resp_send(req, msg, n);
    sys_log(1, "OTA      %u bytes flashed to %s, rebooting", (unsigned)total, target->label);

    vTaskDelay(pdMS_TO_TICKS(300));   /* let the response leave the socket first */
    esp_restart();
    return ESP_OK;   /* not reached */
}

void ota_register_http(httpd_handle_t srv)
{
    static const httpd_uri_t uri = {
        .uri = "/ota", .method = HTTP_POST, .handler = ota_post,
    };
    httpd_register_uri_handler(srv, &uri);
}
