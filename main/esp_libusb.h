#include "usb/usb_host.h"
#include "freertos/semphr.h"

#ifndef portMAX_DELAY
#define portMAX_DELAY (TickType_t)0xffffffffUL
#endif

#define CTRL_OUT (USB_BM_REQUEST_TYPE_TYPE_VENDOR | USB_BM_REQUEST_TYPE_DIR_OUT)
#define CTRL_IN (USB_BM_REQUEST_TYPE_TYPE_VENDOR | USB_BM_REQUEST_TYPE_DIR_IN)

#define USB_SETUP_PACKET_INIT_CONTROL(setup_pkt_ptr, bm_reqtype, b_request, w_value, w_index, w_length) ({ \
    (setup_pkt_ptr)->bmRequestType = bm_reqtype;                                                           \
    (setup_pkt_ptr)->bRequest = b_request;                                                                 \
    (setup_pkt_ptr)->wValue = w_value;                                                                     \
    (setup_pkt_ptr)->wIndex = w_index;                                                                     \
    (setup_pkt_ptr)->wLength = w_length;                                                                   \
})

typedef struct
{
    usb_host_client_handle_t client_hdl;
    uint8_t dev_addr;
    usb_device_handle_t dev_hdl;
    uint32_t actions;
} class_driver_t;

typedef struct
{
    bool is_adsb;
    uint8_t *response_buf;
    bool is_success;
    int bytes_transferred;
    usb_transfer_t *transfer;
    SemaphoreHandle_t done_sem;
    // transfer/response_buf/is_success/bytes_transferred are a single shared
    // slot, so a control transfer is not re-entrant. usb_recover_task's
    // rtlsdr_reset_interface() can land on top of a tuner register access from
    // rtlsdr_setup_task or a TUI-driven retune -- both would then read each
    // other's reply.
    SemaphoreHandle_t ctrl_mux;
} class_adsb_dev;

/* Defined in class_driver.c. The TUI repaints via cursor addressing with no
 * per-frame clear, so any raw ESP_LOG/printf output that lands on stdout
 * while it's running corrupts the layout -- route USB/RTL diagnostics here
 * instead so they land in the on-screen event log. */
void tui_log(uint8_t color, const char *fmt, ...);

void init_adsb_dev();
void transfer_read_cb(usb_transfer_t *transfer);
int esp_libusb_bulk_transfer(class_driver_t *driver_obj, unsigned char endpoint, unsigned char *data, int length, int *transferred, unsigned int timeout);
void esp_libusb_bulk_teardown(void);

/* Decoupled streaming: a self-resubmitting transfer pool fills a RAM IQ ring
 * (USB pump runs in its own task), drained by esp_libusb_stream_read(). */
int      esp_libusb_stream_start(class_driver_t *driver_obj, unsigned char endpoint);
void     esp_libusb_stream_stop(void);
int      esp_libusb_stream_read(unsigned char *dst, int max);
void     esp_libusb_stream_reset(void);
uint32_t esp_libusb_stream_avail(void);
uint64_t esp_libusb_stream_dropped(void);
int      esp_libusb_stream_slots(void);

int esp_libusb_control_transfer(class_driver_t *driver_obj, uint8_t bm_req_type, uint8_t b_request, uint16_t wValue, uint16_t wIndex, unsigned char *data, uint16_t wLength, unsigned int timeout);
void esp_libusb_get_string_descriptor_ascii(const usb_str_desc_t *str_desc, char *str);