#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

static const char *TAG = "MoistureCalibration";
#define ADC_MAX 4095.0f
float moisture_map(int adc_r) {
    float adc_value=(float)adc_r / ADC_MAX;
    // Replace with your actual coefficients from Python calibration
    const float a0 = -1.07575758e-04;         // intercept
    const float a1 = 3.24428905e-02;          // coefficient for x
    const float a2 = -3.58779721;      // coefficient for x^2
    const float a3 = 171.881818;    // coefficient for x^3

    float moisture = a0 
                   + a1 * adc_value 
                   + a2 * adc_value * adc_value
                   + a3 * adc_value * adc_value * adc_value;

    // Clamp to 0–100 %
    if (moisture < 0)   moisture = 0;
    if (moisture > 100) moisture = 100;

    return moisture;
}

void app_main(void)
{
    // Inicialización del ADC en modo one-shot (ADC1)
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // Canal y atenuación: 11 dB ≈ hasta ~3.3V
    // ADC_CHANNEL_6 = GPIO34 en ESP32 clásico
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config));

    // Calibración para convertir RAW -> mV
    adc_cali_handle_t adc1_cali_handle = NULL;
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &adc1_cali_handle));

    int adc_raw = 0;
    int voltage = 0;

    while (1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &adc_raw));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &voltage));
	float moisture = moisture_map(adc_raw);
        ESP_LOGI(TAG, "ADC Raw: %d\tVoltage: %d mV\tMoisture: %.2f %%", adc_raw, voltage, moisture);

        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 Hz
    }
}
