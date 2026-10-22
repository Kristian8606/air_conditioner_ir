// RMT TX backend of IRsend for ESP-IDF.
// Part of the ESP-IDF port of IRremoteESP8266, see README_ESP_IDF.md.

#ifndef IRREMOTE_ESP_IDF_IR_RMT_TX_H_
#define IRREMOTE_ESP_IDF_IR_RMT_TX_H_

#include <stddef.h>
#include <stdint.h>

/// Resolution of the RMT channel: 1 tick = 1 us.
#define IR_RMT_TX_RESOLUTION_HZ 1000000
/// Longest duration of one half of an RMT symbol, in ticks (15 bits).
#define IR_RMT_TX_MAX_HALF_DURATION 32767

/// Pack one RMT symbol (same layout as rmt_symbol_word_t): duration0/level0 in the low 16 bits,
/// duration1/level1 in the high 16 bits. Durations are in us and must not be 0.
inline uint32_t ir_rmt_tx_symbol(uint16_t duration0, bool level0, uint16_t duration1, bool level1)
{
    return (uint32_t)(duration0 & 0x7FFF) | ((uint32_t)level0 << 15) | ((uint32_t)(duration1 & 0x7FFF) << 16) |
           ((uint32_t)level1 << 31);
}

/// Send the symbols on the GPIO and wait until they have been sent.
/// The RMT channel is created on the first call (and again if the GPIO changes).
/// Level 1 is modulated with the carrier (frequency_hz, duty in %), or sent as a plain level when
/// modulation is false.
/// @return true on success
bool ir_rmt_tx_send(uint16_t gpio, bool inverted, const uint32_t *symbols, size_t count, uint32_t frequency_hz,
                    uint8_t duty, bool modulation);

/// Called with every message before it is sent (in the task which sends it), e.g. to decode and log it.
typedef void (*ir_rmt_tx_monitor_t)(const uint32_t *symbols, size_t count);
void ir_rmt_tx_set_monitor(ir_rmt_tx_monitor_t monitor);

/// esp_timer_get_time() when the last message had been sent (0: nothing sent yet).
/// An IR receiver next to the transmitter also receives the own messages.
int64_t ir_rmt_tx_last_end_us(void);

/// true while a message is being sent and for echo_us after its expected end. Set before the message is
/// sent, so it does not depend on when the sending task runs again.
bool ir_rmt_tx_echo_window(int64_t echo_us);

#endif  // IRREMOTE_ESP_IDF_IR_RMT_TX_H_
