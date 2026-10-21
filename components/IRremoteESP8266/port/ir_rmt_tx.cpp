// RMT TX backend of IRsend for ESP-IDF.
// Part of the ESP-IDF port of IRremoteESP8266, see README_ESP_IDF.md.

#include "ir_rmt_tx.h"

#include <driver/rmt_encoder.h>
#include <driver/rmt_tx.h>
#include <esp_log.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <soc/soc_caps.h>

static const char *TAG = "ir_rmt_tx";

/* The whole message is sent in one transaction, waiting longer means the channel hangs */
#define IR_RMT_TX_TIMEOUT_MS 5000

static rmt_channel_handle_t ir_channel = nullptr;
static rmt_encoder_handle_t ir_encoder = nullptr;
static int ir_channel_gpio = -1;
static bool ir_channel_inverted = false;
static uint32_t ir_carrier_frequency = UINT32_MAX;
static uint8_t ir_carrier_duty = 0;
static SemaphoreHandle_t ir_mutex = nullptr;

static void ir_rmt_tx_delete()
{
    if (ir_channel) {
        rmt_disable(ir_channel);
        rmt_del_channel(ir_channel);
        ir_channel = nullptr;
    }
    ir_channel_gpio = -1;
    ir_carrier_frequency = UINT32_MAX;
}

static bool ir_rmt_tx_create(uint16_t gpio, bool inverted)
{
    rmt_tx_channel_config_t config = {};
    config.gpio_num = static_cast<gpio_num_t>(gpio);
    config.clk_src = RMT_CLK_SRC_DEFAULT;
    config.resolution_hz = IR_RMT_TX_RESOLUTION_HZ;
    config.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL;
    config.trans_queue_depth = 4;
    config.flags.invert_out = inverted;
    esp_err_t err = rmt_new_tx_channel(&config, &ir_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create the RMT TX channel on GPIO%u: %s", gpio, esp_err_to_name(err));
        ir_channel = nullptr;
        return false;
    }
    if (!ir_encoder) {
        rmt_copy_encoder_config_t encoder_config = {};
        err = rmt_new_copy_encoder(&encoder_config, &ir_encoder);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create the RMT copy encoder: %s", esp_err_to_name(err));
            ir_encoder = nullptr;
            ir_rmt_tx_delete();
            return false;
        }
    }
    err = rmt_enable(ir_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable the RMT TX channel: %s", esp_err_to_name(err));
        ir_rmt_tx_delete();
        return false;
    }
    ir_channel_gpio = gpio;
    ir_channel_inverted = inverted;
    ESP_LOGI(TAG, "IR transmitter on GPIO%u", gpio);
    return true;
}

static bool ir_rmt_tx_set_carrier(uint32_t frequency_hz, uint8_t duty, bool modulation)
{
    uint32_t frequency = modulation && duty < 100 ? frequency_hz : 0;
    if (frequency == ir_carrier_frequency && (frequency == 0 || duty == ir_carrier_duty)) {
        return true;
    }
    esp_err_t err;
    if (frequency == 0) {
        err = rmt_apply_carrier(ir_channel, nullptr);    /* No modulation: plain level */
    } else {
        rmt_carrier_config_t carrier = {};
        carrier.frequency_hz = frequency;
        carrier.duty_cycle = duty / 100.0f;
        err = rmt_apply_carrier(ir_channel, &carrier);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set the carrier (%" PRIu32 " Hz, %u %%): %s", frequency, duty, esp_err_to_name(err));
        ir_carrier_frequency = UINT32_MAX;
        return false;
    }
    ir_carrier_frequency = frequency;
    ir_carrier_duty = duty;
    return true;
}

bool ir_rmt_tx_send(uint16_t gpio, bool inverted, const uint32_t *symbols, size_t count, uint32_t frequency_hz,
                    uint8_t duty, bool modulation)
{
    if (count == 0) {
        return true;
    }
    if (!ir_mutex) {
        /* Created by the first caller; the application sends from one task, the mutex only guards misuse */
        ir_mutex = xSemaphoreCreateMutex();
        if (!ir_mutex) {
            return false;
        }
    }
    xSemaphoreTake(ir_mutex, portMAX_DELAY);

    bool ok = true;
    if (ir_channel && (ir_channel_gpio != gpio || ir_channel_inverted != inverted)) {
        ir_rmt_tx_delete();
    }
    if (!ir_channel) {
        ok = ir_rmt_tx_create(gpio, inverted);
    }
    if (ok) {
        ok = ir_rmt_tx_set_carrier(frequency_hz, duty, modulation);
    }
    if (ok) {
        rmt_transmit_config_t transmit_config = {};
        transmit_config.loop_count = 0;
        esp_err_t err = rmt_transmit(ir_channel, ir_encoder, symbols, count * sizeof(uint32_t), &transmit_config);
        if (err == ESP_OK) {
            err = rmt_tx_wait_all_done(ir_channel, IR_RMT_TX_TIMEOUT_MS);
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send %u symbols: %s", (unsigned)count, esp_err_to_name(err));
            ok = false;
        }
    }

    xSemaphoreGive(ir_mutex);
    return ok;
}
