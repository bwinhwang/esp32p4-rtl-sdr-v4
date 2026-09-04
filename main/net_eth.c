/*
 * On-board wired Ethernet: IP101GRI over RMII, driven by the P4's own EMAC.
 * Deliberately independent of esp_hosted and of PSRAM -- both of those are
 * requirements of the C6 WiFi path, not of this one.
 */

#include <stdio.h>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_eth.h"
#include "esp_eth_mac_esp.h"
#include "esp_eth_phy_ip101.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_libusb.h"   /* tui_log() */
#include "net_eth.h"

/* The only pin ETH_ESP32_EMAC_DEFAULT_CONFIG() does not already cover: the
 * PHY's active-low reset (U2 pin 32). Its P4 defaults -- MDC 31 / MDIO 52,
 * REF_CLK in on 50, TX_EN 49, TXD0/1 34/35, CRS_DV 28, RXD0/1 29/30 -- match
 * this board's schematic pin for pin, so nothing else is overridden. */
#define PHY_RST_PIN  51

static const char *TAG = "eth";

static volatile net_eth_state_t s_state;
/* Written by the event task, read by tui_task. Kept as the raw 32-bit address
 * rather than a formatted string so a reader cannot catch a half-written one. */
static volatile uint32_t s_ip4;

net_eth_state_t net_eth_state(void) { return s_state; }

void net_eth_ip_str(char *dst, size_t n)
{
    uint32_t a = s_ip4;
    if (a == 0) {
        snprintf(dst, n, "---");
        return;
    }
    snprintf(dst, n, "%u.%u.%u.%u",
             (unsigned)(a & 0xff),         (unsigned)((a >> 8) & 0xff),
             (unsigned)((a >> 16) & 0xff), (unsigned)((a >> 24) & 0xff));
}

static void on_eth_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
    case ETHERNET_EVENT_CONNECTED: {
        esp_eth_handle_t eth    = *(esp_eth_handle_t *)data;
        eth_speed_t      speed  = ETH_SPEED_10M;
        eth_duplex_t     duplex = ETH_DUPLEX_HALF;
        esp_eth_ioctl(eth, ETH_CMD_G_SPEED, &speed);
        esp_eth_ioctl(eth, ETH_CMD_G_DUPLEX_MODE, &duplex);
        s_state = NET_ETH_LINK;
        tui_log(1, "ETH      link up  %s Mbps %s-duplex",
                speed  == ETH_SPEED_100M  ? "100"  : "10",
                duplex == ETH_DUPLEX_FULL ? "full" : "half");
        break;
    }
    case ETHERNET_EVENT_DISCONNECTED:
        s_ip4   = 0;
        s_state = NET_ETH_NOLINK;
        tui_log(4, "ETH      link down");
        break;
    default:
        break;
    }
}

static void on_got_ip(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    const ip_event_got_ip_t *e = data;
    s_ip4   = e->ip_info.ip.addr;
    s_state = NET_ETH_READY;
    tui_log(1, "ETH      " IPSTR "  gw " IPSTR "  mask " IPSTR,
            IP2STR(&e->ip_info.ip), IP2STR(&e->ip_info.gw),
            IP2STR(&e->ip_info.netmask));
}

esp_err_t net_eth_start(void)
{
    /* The TUI repaints by cursor addressing with no per-frame clear, so a raw
     * log line landing on stdout after it starts corrupts the layout.
     * esp_netif_handlers prints the address at INFO on every DHCP bind and
     * renewal, and those arrive long after the TUI is up. */
    esp_log_level_set("esp_netif_handlers", ESP_LOG_WARN);

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init failed");
    /* Creates the sys_evt task (prio 20, no affinity). It is above
     * adsb_rx_task, but it only ever runs on a link/DHCP transition. */
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop failed");

    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t       *netif     = esp_netif_new(&netif_cfg);
    ESP_RETURN_ON_FALSE(netif, ESP_ERR_NO_MEM, TAG, "netif alloc failed");

    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    /* emac_rx runs at prio 15, well above adsb_rx_task's 5. Left unpinned it
     * is free to land on core1 and preempt the demod loop mid-buffer; this
     * flag pins it to whichever core installs the driver, and that is
     * app_main on core0. */
    mac_cfg.flags |= ETH_MAC_FLAG_PIN_TO_CORE;

    eth_esp32_emac_config_t esp_cfg = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp_cfg, &mac_cfg);
    ESP_RETURN_ON_FALSE(mac, ESP_FAIL, TAG, "MAC alloc failed");

    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.reset_gpio_num = PHY_RST_PIN;
    /* phy_addr stays at ETH_PHY_DEFAULT_CONFIG()'s ESP_ETH_PHY_ADDR_AUTO: the
     * board straps the address through resistors that were not traced, so let
     * the driver scan MDIO for it. A scan that finds nothing is also the
     * cheapest evidence that MDC/MDIO are miswired. */
    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_cfg);
    ESP_RETURN_ON_FALSE(phy, ESP_FAIL, TAG, "PHY alloc failed");

    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth     = NULL;
    ESP_RETURN_ON_ERROR(esp_eth_driver_install(&eth_cfg, &eth), TAG, "driver install failed");
    ESP_RETURN_ON_ERROR(esp_netif_attach(netif, esp_eth_new_netif_glue(eth)), TAG, "netif attach failed");

    ESP_RETURN_ON_ERROR(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                                   on_eth_event, NULL),
                        TAG, "ETH_EVENT register failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                                   on_got_ip, NULL),
                        TAG, "IP_EVENT register failed");

    ESP_RETURN_ON_ERROR(esp_eth_start(eth), TAG, "start failed");
    s_state = NET_ETH_NOLINK;
    return ESP_OK;
}
