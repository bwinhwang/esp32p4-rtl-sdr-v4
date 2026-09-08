#pragma once

#include "esp_http_server.h"

/* Logs the running OTA slot + app version, and spawns the task that confirms
 * (or lets roll back) a pending image -- see ota.c. Call once from app_main,
 * order does not matter relative to networking bring-up. */
void ota_init(void);

/* Registers POST /ota onto an already-started httpd instance (web_config.c's
 * server). Not a standalone listener: reusing that httpd keeps this on the
 * one port already open on every interface (Ethernet, SoftAP, STA). */
void ota_register_http(httpd_handle_t srv);
