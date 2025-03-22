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
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "host/ble_store.h"
#include "store/config/ble_store_config.h"
#include "store/ram/ble_store_ram.h"

static const char *tag = "BLE_CLIENT";
static uint16_t power_handle = 0;
static uint16_t brightness_handle = 0;
static uint8_t own_addr_type;

// For "932c32bd-0002-47a2-835a-a8d455b859dd"
static const ble_uuid128_t power_uuid128 = BLE_UUID128_INIT(
   0x93, 0x2c, 0x32, 0xbd,
   0x00, 0x02, 0x47, 0xa2,
   0x83, 0x5a, 0xa8, 0xd4,
   0x55, 0xb8, 0x59, 0xdd
);

// For "932c32bd-0003-47a2-835a-a8d455b859dd"
static const ble_uuid128_t brightness_uuid128 = BLE_UUID128_INIT(
   0x93, 0x2c, 0x32, 0xbd,
   0x00, 0x03, 0x47, 0xa2,
   0x83, 0x5a, 0xa8, 0xd4,
   0x55, 0xb8, 0x59, 0xdd
);

// Bulb MAC address C9:41:94:FB:C9:61
static const ble_addr_t bulb_addr = {
    .type = BLE_ADDR_RANDOM,
    .val = {0x61, 0xC9, 0xFB, 0x94, 0x41, 0xC9}
};

static void control_bulb(uint16_t conn_handle) {
    if (power_handle == 0 || brightness_handle == 0) {
        ESP_LOGE(tag, "Characteristic handles not found");
        return;
    }
    
    uint8_t data;
    int rc;

    // Turn on bulb
    ESP_LOGI(tag, "Turning bulb ON");
    data = 0x01;
    rc = ble_gattc_write_no_rsp_flat(conn_handle, power_handle, &data, sizeof(data));
    if (rc == 0) {
        ESP_LOGI(tag, "Sent power ON command successfully");
    } else {
        ESP_LOGE(tag, "Failed to send power ON command, rc=%d", rc);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Set brightness to 100%
    ESP_LOGI(tag, "Setting brightness to 100");
    data = 0xfe;
    rc = ble_gattc_write_no_rsp_flat(conn_handle, brightness_handle, &data, sizeof(data));
    if (rc == 0) {
        ESP_LOGI(tag, "Sent brightness command successfully");
    } else {
        ESP_LOGE(tag, "Failed to send brightness command, rc=%d", rc);
    }
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    // Turn off bulb
    ESP_LOGI(tag, "Turning bulb OFF");
    data = 0x00;
    rc = ble_gattc_write_no_rsp_flat(conn_handle, power_handle, &data, sizeof(data));
    if (rc == 0) {
        ESP_LOGI(tag, "Sent power OFF command successfully");
    } else {
        ESP_LOGE(tag, "Failed to send power OFF command, rc=%d", rc);
    }
}

static int gatt_chr_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg) {
    if (error != NULL && error->status != 0) {
        ESP_LOGE(tag, "Characteristic discovery error, status=%d", error->status);
        return error->status;
    }

    if (chr == NULL) {
        ESP_LOGI(tag, "Characteristic discovery complete");
        if (power_handle != 0 && brightness_handle != 0) {
            ESP_LOGI(tag, "Both characteristics found. Starting bulb control sequence.");
            control_bulb(conn_handle);
        } else {
            ESP_LOGE(tag, "Failed to find all required characteristics. Power: 0x%04x, Brightness: 0x%04x", 
                     power_handle, brightness_handle);
        }
        return 0;
    }

    char uuid_str[BLE_UUID_STR_LEN];
    ble_uuid_to_str(&chr->uuid.u, uuid_str);
    ESP_LOGI(tag, "Discovered characteristic: val_handle=0x%04x, uuid=%s", chr->val_handle, uuid_str);
    
    if (ble_uuid_cmp(&chr->uuid.u, (const ble_uuid_t *)&power_uuid128) == 0) {
        power_handle = chr->val_handle;
        ESP_LOGI(tag, "Found power characteristic at handle: 0x%04x", power_handle);
    } else if (ble_uuid_cmp(&chr->uuid.u, (const ble_uuid_t *)&brightness_uuid128) == 0) {
        brightness_handle = chr->val_handle;
        ESP_LOGI(tag, "Found brightness characteristic at handle: 0x%04x", brightness_handle);
    }

    return 0;
}

static int gatt_svc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *service, void *arg) {
    if (error != NULL && error->status != 0) {
        ESP_LOGE(tag, "Service discovery error: status=%d", error->status);
        return error->status;
    }

    if (service == NULL) {
        ESP_LOGI(tag, "Service discovery complete");
        return 0;
    }

    char uuid_str[BLE_UUID_STR_LEN];
    ble_uuid_to_str(&service->uuid.u, uuid_str);
    ESP_LOGI(tag, "Discovered service: handle=0x%04x to 0x%04x, uuid=%s", 
             service->start_handle, service->end_handle, uuid_str);
    
    int rc = ble_gattc_disc_all_chrs(conn_handle, service->start_handle, service->end_handle, gatt_chr_disc_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(tag, "Failed to discover characteristics, rc=%d", rc);
    }

    return 0;
}

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(tag, "Connected to bulb, handle=%d", event->connect.conn_handle);
                int rc = ble_gap_security_initiate(event->connect.conn_handle);
                if (rc != 0) {
                    ESP_LOGE(tag, "Failed to initiate pairing: %d", rc);
                    rc = ble_gattc_disc_all_svcs(event->connect.conn_handle, gatt_svc_disc_cb, NULL);
                    if (rc != 0) {
                        ESP_LOGE(tag, "Service discovery failed; rc=%d", rc);
                    }
                }
            } else {
                ESP_LOGE(tag, "Connection failed; status=%d", event->connect.status);
            }
            break;
            
        case BLE_GAP_EVENT_ENC_CHANGE:
            if (event->enc_change.status == 0) {
                ESP_LOGI(tag, "Encryption enabled, starting service discovery");
                int rc = ble_gattc_disc_all_svcs(event->enc_change.conn_handle, gatt_svc_disc_cb, NULL);
                if (rc != 0) {
                    ESP_LOGE(tag, "Service discovery failed; rc=%d", rc);
                }
            } else {
                ESP_LOGE(tag, "Encryption failed; status=%d", event->enc_change.status);
                int rc = ble_gattc_disc_all_svcs(event->enc_change.conn_handle, gatt_svc_disc_cb, NULL);
                if (rc != 0) {
                    ESP_LOGE(tag, "Service discovery failed; rc=%d", rc);
                }
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

static void ble_client_task(void *param) {
    struct ble_gap_conn_params conn_params = {
        .scan_itvl = 0x0030,
        .scan_window = BLE_GAP_SCAN_FAST_WINDOW,
        .itvl_min = BLE_GAP_INITIAL_CONN_ITVL_MIN,
        .itvl_max = BLE_GAP_INITIAL_CONN_ITVL_MAX,
        .latency = BLE_GAP_INITIAL_CONN_LATENCY,
        .supervision_timeout = BLE_GAP_INITIAL_SUPERVISION_TIMEOUT,
        .min_ce_len = BLE_GAP_INITIAL_CONN_MIN_CE_LEN,
        .max_ce_len = BLE_GAP_INITIAL_CONN_MAX_CE_LEN
    };

    while (1) {
        ESP_LOGI(tag, "Attempting to connect to bulb at address type %d", bulb_addr.type);
        int rc = ble_gap_connect(own_addr_type, &bulb_addr, 10000, &conn_params, ble_gap_event_cb, NULL);
        if (rc != 0) {
            ESP_LOGE(tag, "Error initiating connection; rc=%d. Retrying in 5 seconds...", rc);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            continue;
        }

        vTaskDelay(portMAX_DELAY);
    }
}

static void ble_set_security_params(void) {
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
}

static void on_sync(void) {
    ESP_LOGI(tag, "BLE Host synchronized");
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(tag, "Device does not have any available BT address!");
        return;
    }

    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(tag, "Failed to infer address type, error code: %d", rc);
        return;
    }
    ESP_LOGI(tag, "Local device address type: %d", own_addr_type);
    xTaskCreate(ble_client_task, "ble_client_task", 4096, NULL, 5, NULL);
}

void host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void) {
    nvs_flash_erase();
    ESP_LOGI(tag, "Initializing NimBLE client");
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    esp_nimble_hci_init();
    nimble_port_init();

    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_set_security_params();

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set("Hue-Controller");

    nimble_port_freertos_init(host_task);
}