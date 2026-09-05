#pragma once

#include <stddef.h>
#include "esp_err.h"

typedef enum {
    NET_WIFI_OFF = 0,   /* not started, or the C6 transport never came up */
    NET_WIFI_INIT,      /* bring-up in progress (SDIO probe + C6 boot) */
    NET_WIFI_AP,        /* SoftAP beaconing; not joined to an upstream AP */
    NET_WIFI_STA,       /* SoftAP up AND joined upstream with a lease */
} net_wifi_state_t;

esp_err_t        net_wifi_start(void);
net_wifi_state_t net_wifi_state(void);

/* Stations currently associated to our own SoftAP. */
int  net_wifi_ap_clients(void);

/* Address the STA side got from the upstream AP, or "---" when there isn't
 * one. The SoftAP's own address is fixed by esp_netif and never interesting. */
void net_wifi_sta_ip_str(char *dst, size_t n);

/* Upstream credentials live in NVS, never in the image: this repo's sdkconfig
 * is tracked and its remote is a public fork, so a Kconfig default would
 * publish the password. They are set at runtime from the config page.
 *
 * net_wifi_set_sta() stores them and retries the join immediately; an empty
 * ssid clears them and stops trying. */
esp_err_t net_wifi_set_sta(const char *ssid, const char *pass);
void      net_wifi_sta_ssid(char *dst, size_t n);
