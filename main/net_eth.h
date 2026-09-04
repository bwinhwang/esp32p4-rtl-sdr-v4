#pragma once

#include <stddef.h>
#include "esp_err.h"

typedef enum {
    NET_ETH_OFF = 0,   /* driver never came up */
    NET_ETH_NOLINK,    /* PHY answered on MDIO, cable/peer down */
    NET_ETH_LINK,      /* link up, no address yet */
    NET_ETH_READY,     /* DHCP done */
} net_eth_state_t;

esp_err_t       net_eth_start(void);
net_eth_state_t net_eth_state(void);

/* Writes the current IPv4 address into dst, or "---" when there isn't one. */
void net_eth_ip_str(char *dst, size_t n);
