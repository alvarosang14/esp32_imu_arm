#include <stdbool.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "nvs_flash.h"
#include "bt_wrapper.h"

static uint32_t handle = 0;
static bool congested = false;  // link full: drop instead of queueing old data

static void spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    if (event == ESP_SPP_INIT_EVT) {
        esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, "SPP");
    } else if (event == ESP_SPP_SRV_OPEN_EVT) {
        congested = false;
        handle = param->srv_open.handle;
    } else if (event == ESP_SPP_CLOSE_EVT) {
        handle = 0;
    } else if (event == ESP_SPP_CONG_EVT) {
        congested = param->cong.cong;
    } else if (event == ESP_SPP_WRITE_EVT) {
        congested = param->write.cong;
    }
}

void bt_setup(void) {
    if (nvs_flash_init() != ESP_OK) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&cfg);
    esp_bt_controller_enable(cfg.mode);
    esp_bluedroid_init();
    esp_bluedroid_enable();

    // Generic Access Profile
    esp_bt_gap_set_device_name("BT_IMU_ARM");
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    // Serial Profile
    esp_spp_register_callback(spp_cb);
    esp_spp_cfg_t spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
    };
    esp_spp_enhanced_init(&spp_cfg);
}

void bt_send(const char *msg) {
    if (handle && !congested) {
        esp_spp_write(handle, strlen(msg), (uint8_t *)msg);
    }
}
