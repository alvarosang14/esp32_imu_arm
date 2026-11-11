#include "adc_wrapper.h"

#include "esp_log.h"

static const char *TAG = "adc";

static adc_continuous_handle_t handle = NULL;
static uint8_t raw[ADC_FRAME_SIZE];

esp_err_t adc_setup(const adc_channel_t *channels, uint8_t num, uint32_t freq_hz) {
    if (num == 0 || num > SOC_ADC_PATT_LEN_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = 1024,
        .conv_frame_size = ADC_FRAME_SIZE,
    };
    esp_err_t err = adc_continuous_new_handle(&handle_cfg, &handle);
    if (err != ESP_OK) {
        return err;
    }

    adc_digi_pattern_config_t pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    for (int i = 0; i < num; i++) {
        pattern[i].atten = ADC_ATTEN_DB_12;  // full range ~0-3.1 V
        pattern[i].channel = channels[i] & 0x7;
        pattern[i].unit = ADC_UNIT_1;
        pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    }

    adc_continuous_config_t cfg = {
        .pattern_num = num,
        .adc_pattern = pattern,
        .sample_freq_hz = freq_hz,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    };
    err = adc_continuous_config(handle, &cfg);
    if (err == ESP_OK) {
        err = adc_continuous_start(handle);
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "setup: %s", esp_err_to_name(err));
        adc_continuous_deinit(handle);
        handle = NULL;
    }
    return err;
}

esp_err_t adc_read(adc_continuous_data_t *out, uint32_t *num, uint32_t timeout_ms) {
    *num = 0;
    if (!handle) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t len = 0;
    esp_err_t err = adc_continuous_read(handle, raw, sizeof(raw), &len, timeout_ms);
    if (err != ESP_OK) {
        return err;
    }

    // Parse the raw bytes into {unit, channel, raw_data, valid}
    return adc_continuous_parse_data(handle, raw, len, out, num);
}

esp_err_t adc_stop(void) {
    if (!handle) {
        return ESP_ERR_INVALID_STATE;
    }

    adc_continuous_stop(handle);
    esp_err_t err = adc_continuous_deinit(handle);
    handle = NULL;
    return err;
}
