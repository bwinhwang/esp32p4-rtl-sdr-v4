#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

typedef enum {
    NET_WIFI_OFF = 0,   /* not started, or the C6 transport never came up */
    NET_WIFI_INIT,      /* bring-up in progress (SDIO probe + C6 boot) */
    NET_WIFI_AP,        /* not joined upstream. The SoftAP is beaconing unless
                         * it has been switched off -- ask net_wifi_ap_enabled()
                         * before calling this state "AP only" to a user. */
    NET_WIFI_STA,       /* joined upstream with a lease */
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

/* Join-retry state: consecutive failed attempts, and seconds until the next
 * one. A join sweeps the band on the C6's single radio and takes the SoftAP
 * with it, so the interval backs off and is held down further while a station
 * is associated -- which makes "why has it not joined yet" a fair question and
 * this the answer to it. Both fields are stale once the link is up. */
void      net_wifi_sta_retry(int *tries, int *next_s);

/* Either half of the radio can be switched off at runtime, and the choice is
 * kept in NVS so a headless board resumes the way it was left.
 *
 * The two are not symmetric under the hood, which is why they are separate
 * calls rather than one mode setter. Switching the STA half off only stops the
 * join loop and drops the association -- no mode change, nothing to go wrong,
 * and the stored credentials are kept so switching it back on rejoins with no
 * retyping. Switching the SoftAP off is a real esp_wifi_set_mode() between
 * APSTA and STA; see the comment on net_wifi_set_ap_enabled() for why that
 * particular mode change is not the one esp_hosted-mcu is known to break on.
 *
 * The STA switch is the one that matters in the car: a configured-but-absent
 * upstream SSID makes the radio sweep the band periodically, and the SoftAP
 * goes off its own channel for the duration. */
esp_err_t net_wifi_set_sta_enabled(bool on);
esp_err_t net_wifi_set_ap_enabled(bool on);
bool      net_wifi_sta_enabled(void);
bool      net_wifi_ap_enabled(void);
