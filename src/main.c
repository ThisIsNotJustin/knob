#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "host/ble_sm.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *tag = "BLE_CLIENT";
static uint16_t power_handle = 0;
static uint16_t brightness_handle = 0;

// For "932c32bd-0002-47a2-835a-a8d455b859dd"
static const ble_uuid128_t power_uuid128 = BLE_UUID128_INIT(
    0xdd, 0x59, 0xb8, 0x55,
    0xd4, 0xa8, 0x5a, 0x83,
    0xa2, 0x47, 0x02, 0x00,
    0xbd, 0x32, 0x2c, 0x93
);
// For "932c32bd-0003-47a2-835a-a8d455b859dd"
static const ble_uuid128_t brightness_uuid128 = BLE_UUID128_INIT(
    0xdd, 0x59, 0xb8, 0x55,
    0xd4, 0xa8, 0x5a, 0x83,
    0xa2, 0x47, 0x03, 0x00,
    0xbd, 0x32, 0x2c, 0x93
);

// Replace with the MAC address of your bulb device.
// C7:46:23:94:4A:14
// C9:41:94:FB:C9:61
static const ble_addr_t bulb_addr = {
    .type = BLE_ADDR_RANDOM,
    // .val = {0xC9, 0x41, 0x94, 0xFB, 0xC9, 0x61}
    .val = {0x61, 0xC9, 0xFB, 0x94, 0x41, 0xC9}
};

static void print_device_name(const uint8_t *adv_data, uint8_t adv_data_len) {
    int index = 0;
    while (index < adv_data_len) {
        uint8_t length = adv_data[index];
        if (length == 0) {
            break;
        }

        uint8_t type = adv_data[index + 1]; 
        if (type == 0x09 || type == 0x08) {
            printf("Device Name: ");
            for (int i = 0; i < length - 1; i++) {
                printf("%c", adv_data[index + 2 + i]);
            }
            printf("\n");
            return;
        }
        index += length + 1;
    }
}

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg);

// Add security parameters
static void ble_set_security_params(void) {
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;       // No input/output capability
    ble_hs_cfg.sm_bonding = 1;                        // Enable bonding
    ble_hs_cfg.sm_mitm = 0;                           // No man-in-the-middle protection
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC;  // Key distribution
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC;
}

/*
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg) {
    if (event->type == BLE_GAP_EVENT_DISC) {  // Device discovered
        ESP_LOGI(tag, "Device found: %02X:%02X:%02X:%02X:%02X:%02X, RSSI: %d",
            event->disc.addr.val[5], event->disc.addr.val[4], event->disc.addr.val[3],
            event->disc.addr.val[2], event->disc.addr.val[1], event->disc.addr.val[0],
            event->disc.rssi);

        print_device_name(event->disc.data, event->disc.length_data);
    }
    return 0;
}
*/

static int gatt_chr_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg) {
    if (error != NULL) {
        ESP_LOGE(tag, "Characteristic discovery error; status=%d, handle=0x%04x", error->status, error->att_handle);
        return error->status;
    }

    if (chr == NULL) {
        ESP_LOGI(tag, "Characteristic discovery complete");

        if (power_handle != 0 && brightness_handle != 0) {
            uint8_t data;
            int rc;

            // turn on bulb test
            data = 0x01;
            rc = ble_gattc_write_no_rsp_flat(conn_handle, power_handle, &data, sizeof(data));
            if (rc == 0) {
                ESP_LOGI(tag, "Sent power on command");
            } else {
                ESP_LOGE(tag, "Failed to send power on command, rc=%d", rc);
            }
            vTaskDelay(100 / portTICK_PERIOD_MS);

            // brightness 100% test
            data = 0xfe;
            rc = ble_gattc_write_no_rsp_flat(conn_handle, brightness_handle, &data, sizeof(data));
            if (rc == 0) {
                ESP_LOGI(tag, "Sent brightness command");
            } else {
                ESP_LOGE(tag, "Failed to send brightness command, rc=%d", rc);
            }
            vTaskDelay(100 / portTICK_PERIOD_MS);

            // turn off bulb test
            data=0x00;
            rc = ble_gattc_write_no_rsp_flat(conn_handle, power_handle, &data, sizeof(data));
            if (rc == 0) {
                ESP_LOGI(tag, "Sent power off command");
            } else {
                ESP_LOGE(tag, "Failed to send power off command, rc=%d", rc);
            }
        } else {
            ESP_LOGE(tag, "Either power_handle or brightness_handle were not found");
        }

        return 0;
    }

    char uuid_str[BLE_UUID_STR_LEN];
    ble_uuid_to_str(&chr->uuid.u, uuid_str);
    ESP_LOGI(tag, "Discovered characteristic: def_handle=0x%04x, val_handle=0x%04x, uuid=%s",
        chr->def_handle, chr->val_handle, uuid_str);
    
    if (ble_uuid_cmp(&chr->uuid.u, (const ble_uuid_t *)&power_uuid128) == 0) {
        power_handle = chr->val_handle;
        ESP_LOGI(tag, "Found power characteristic at handle: 0x%04x", power_handle);
    } else if (ble_uuid_cmp(&chr->uuid.u, (const ble_uuid_t *)&brightness_uuid128) == 0) {
        brightness_handle = chr->val_handle;
        ESP_LOGI(tag, "Found brightness characteristic at handle: 0x%04x", brightness_handle);
    }

    return 0;
}

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(tag, "Connected, handle=%d", event->connect.conn_handle);
                // Initiate pairing process
                int rc = ble_gap_security_initiate(event->connect.conn_handle);
                if (rc != 0) {
                    ESP_LOGE(tag, "Failed to initiate pairing: %d", rc);
                }
            } else {
                ESP_LOGE(tag, "Connection failed; status=%d", event->connect.status);
            }
            break;

        case BLE_GAP_EVENT_ENC_CHANGE:
            if (event->enc_change.status == 0) {
                ESP_LOGI(tag, "Encryption enabled, starting service discovery");
                // Start characteristic discovery after successful pairing
                int rc = ble_gattc_disc_all_chrs(event->enc_change.conn_handle,
                    0x0001, 0xFFFF, gatt_chr_disc_cb, NULL);
                if (rc != 0) {
                    ESP_LOGE(tag, "Characteristic discovery failed; rc=%d", rc);
                }
            } else {
                ESP_LOGE(tag, "Encryption failed; status=%d", event->enc_change.status);
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(tag, "Disconnected, reason=%d", event->disconnect.reason);
            break;

        default:
            break;
    }
    return 0;
}

static void ble_scan_task(void *param) {
    struct ble_gap_disc_params disc_params = {
        .filter_duplicates = 1,
        .passive = 0,
        .itvl = 0x0060,
        .window = 0x0030,
        .limited = 0,
        .filter_policy = 0
    };

    ESP_LOGI(tag, "Starting BLE scan...");
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params, ble_gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(tag, "Failed to start scan; rc=%d", rc);
    }

    vTaskDelete(NULL);
}

static void ble_client_task(void *param) {
    struct ble_gap_conn_params conn_params;
    memset(&conn_params, 0, sizeof(conn_params));
    conn_params.scan_itvl = 0x0060;
    conn_params.scan_window = 0x0030;
    conn_params.itvl_min = 0x0018;
    conn_params.itvl_max = 0x0028;
    conn_params.latency = 0;
    conn_params.supervision_timeout = 0x0200;
    conn_params.min_ce_len = 0x0010;
    conn_params.max_ce_len = 0x0300;

    ESP_LOGI(tag, "Attempting to connect to bulb...");
    int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &bulb_addr, 30000, &conn_params, ble_gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(tag, "Error initiating connection; rc=%d", rc);
    }

    vTaskDelete(NULL);
}

static void ble_on_sync(void) {
    ESP_LOGI(tag, "BLE Host synchronized");
    // xTaskCreate(ble_scan_task, "ble_scan_task", 4096, NULL, 5, NULL);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    xTaskCreate(ble_client_task, "ble_client_task", 4096, NULL, 5, NULL);
}

void host_task(void *param) {
    nimble_port_run();
}

void app_main() {
    ESP_LOGI(tag, "Initializing NimBLE");
    nvs_flash_init();
    esp_nimble_hci_init();
    nimble_port_init();
    ble_svc_gap_init(); 
    ble_svc_gatt_init();
    ble_hs_init();
    ble_set_security_params();
    
    ble_svc_gap_device_name_set("Hue-Controller");
    ble_hs_cfg.sync_cb = ble_on_sync;

    nimble_port_freertos_init(host_task);
}