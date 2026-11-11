#ifndef ADC_WRAPPER_H
#define ADC_WRAPPER_H

#include "esp_adc/adc_continuous.h"
#include "soc/soc_caps.h"

#define ADC_FRAME_SIZE  256
#define ADC_MAX_SAMPLES (ADC_FRAME_SIZE / SOC_ADC_DIGI_RESULT_BYTES)

esp_err_t adc_setup(const adc_channel_t *channels, uint8_t num, uint32_t freq_hz);
esp_err_t adc_read(adc_continuous_data_t *out, uint32_t *num, uint32_t timeout_ms);
esp_err_t adc_stop(void);

#endif // ADC_WRAPPER_H
