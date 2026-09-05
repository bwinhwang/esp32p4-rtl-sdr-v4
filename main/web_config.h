#pragma once

#include "esp_err.h"

/* HTTP config page and JSON API, port CONFIG_ADSB_WEB_PORT.
 *
 *   GET  /              status + the upstream-WiFi form
 *   POST /wifi          store SSID/password in NVS and join immediately
 *   GET  /aircraft.json one snapshot, same schema as the :8888 feed
 *
 * Non-fatal like the rest of the network bring-up: the receiver works with no
 * server running. */
esp_err_t web_config_start(void);
