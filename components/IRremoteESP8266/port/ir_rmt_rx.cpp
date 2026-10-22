// RMT RX backend for IRrecv on ESP-IDF.
// Part of the ESP-IDF port of IRremoteESP8266, see README_ESP_IDF.md.

#include "ir_rmt_rx.h"

#include <string.h>

#include <driver/rmt_rx.h>
#include <esp_log.h>
#include <freertos/queue.h>
#include <soc/soc_caps.h>

static const char *TAG = "ir_rmt_rx";

/* 1 tick = 1 us */
#define IR_RMT_RX_RESOLUTION_HZ 1000000
/* Shorter pulses are noise. The filter counts in the group clock (max 255 ticks), 1 us is safe for all of them */
#define IR_RMT_RX_GLITCH_NS 1000
/* Longest message kept: 512 symbols = 1024 durations (Fujitsu: 130 symbols). RAM is short next to Matter. */
#define IR_RMT_RX_FRAME_SYMBOLS 512
/* Ping-pong: the driver fills this buffer and calls the callback for every part */
#define IR_RMT_RX_CHUNK_SYMBOLS 64

static rmt_channel_handle_t rx_channel = nullptr;
static rmt_receive_config_t rx_config = {};
static rmt_symbol_word_t rx_chunk[IR_RMT_RX_CHUNK_SYMBOLS];
static rmt_symbol_word_t rx_frame[IR_RMT_RX_FRAME_SYMBOLS];
static size_t rx_frame_symbols = 0;
static QueueHandle_t rx_queue = nullptr;   /* Nr. of symbols of a complete message */

static bool ir_rmt_rx_done_cb(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    size_t count = edata->num_symbols;
    if (rx_frame_symbols + count > IR_RMT_RX_FRAME_SYMBOLS) {
        count = IR_RMT_RX_FRAME_SYMBOLS - rx_frame_symbols;
    }
    memcpy(&rx_frame[rx_frame_symbols], edata->received_symbols, count * sizeof(rmt_symbol_word_t));
    rx_frame_symbols += count;

    BaseType_t woken = pdFALSE;
    if (edata->flags.is_last) {
        size_t symbols = rx_frame_symbols;
        rx_frame_symbols = 0;
        xQueueSendFromISR(rx_queue, &symbols, &woken);
    }
    return woken == pdTRUE;
}

static bool ir_rmt_rx_arm()
{
    rx_frame_symbols = 0;
    esp_err_t err = rmt_receive(rx_channel, rx_chunk, sizeof(rx_chunk), &rx_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start receiving: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

bool ir_rmt_rx_start(uint16_t gpio, uint32_t idle_us)
{
    if (rx_channel) {
        return true;
    }
    rx_queue = xQueueCreate(1, sizeof(size_t));
    if (!rx_queue) {
        return false;
    }

    rmt_rx_channel_config_t config = {};
    config.gpio_num = static_cast<gpio_num_t>(gpio);
    config.clk_src = RMT_CLK_SRC_DEFAULT;
    config.resolution_hz = IR_RMT_RX_RESOLUTION_HZ;
    config.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL;
    /* The receiver module output is LOW during a mark: invert it, so a mark is level 1 */
    config.flags.invert_in = true;
    esp_err_t err = rmt_new_rx_channel(&config, &rx_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create the RMT RX channel on GPIO%u: %s", gpio, esp_err_to_name(err));
        rx_channel = nullptr;
        return false;
    }

    rmt_rx_event_callbacks_t callbacks = {};
    callbacks.on_recv_done = ir_rmt_rx_done_cb;
    err = rmt_rx_register_event_callbacks(rx_channel, &callbacks, nullptr);
    if (err == ESP_OK) {
        err = rmt_enable(rx_channel);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable the RMT RX channel: %s", esp_err_to_name(err));
        rmt_del_channel(rx_channel);
        rx_channel = nullptr;
        return false;
    }

    rx_config.signal_range_min_ns = IR_RMT_RX_GLITCH_NS;
    rx_config.signal_range_max_ns = (idle_us > 32000 ? 32000 : idle_us) * 1000;
    rx_config.flags.en_partial_rx = true;
    if (!ir_rmt_rx_arm()) {
        return false;
    }
    ESP_LOGI(TAG, "IR receiver on GPIO%u", gpio);
    return true;
}

/* Add a level of the given duration, merged with the previous one if it has the same level */
static void ir_rmt_rx_add(uint16_t *usecs, size_t max, size_t *count, bool *last_level, bool level, uint32_t duration)
{
    if (duration == 0) {
        return;
    }
    if (*count == 0) {
        if (!level) {
            return;     /* A message starts with a mark */
        }
    } else if (level == *last_level) {
        uint32_t sum = usecs[*count - 1] + duration;
        usecs[*count - 1] = sum > UINT16_MAX ? UINT16_MAX : sum;
        return;
    }
    if (*count < max) {
        usecs[(*count)++] = duration > UINT16_MAX ? UINT16_MAX : duration;
        *last_level = level;
    }
}

size_t ir_rmt_rx_wait(uint16_t *usecs, size_t max, TickType_t timeout)
{
    size_t symbols = 0;
    if (!rx_queue || xQueueReceive(rx_queue, &symbols, timeout) != pdTRUE) {
        return 0;
    }

    size_t count = 0;
    bool last_level = false;
    for (size_t i = 0; i < symbols; i++) {
        ir_rmt_rx_add(usecs, max, &count, &last_level, rx_frame[i].level0, rx_frame[i].duration0);
        ir_rmt_rx_add(usecs, max, &count, &last_level, rx_frame[i].level1, rx_frame[i].duration1);
    }
    /* The idle space at the end is not a part of the message */
    if (count > 0 && !last_level) {
        count--;
    }

    ir_rmt_rx_arm();
    return count;
}
