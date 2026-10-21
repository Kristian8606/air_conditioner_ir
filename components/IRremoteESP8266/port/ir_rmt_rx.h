// RMT RX backend for IRrecv on ESP-IDF.
// Part of the ESP-IDF port of IRremoteESP8266, see README_ESP_IDF.md.

#ifndef IRREMOTE_ESP_IDF_IR_RMT_RX_H_
#define IRREMOTE_ESP_IDF_IR_RMT_RX_H_

#include <stddef.h>
#include <stdint.h>

#include <freertos/FreeRTOS.h>

/// Start receiving on the GPIO of an IR receiver module (output LOW while it sees the carrier).
/// @param[in] gpio GPIO of the receiver output.
/// @param[in] idle_us A space longer than this ends a message (max. 32000 us).
/// @return true on success
bool ir_rmt_rx_start(uint16_t gpio, uint32_t idle_us);

/// Wait for the next message.
/// @param[out] usecs Durations in us: mark, space, mark, ... (the message starts and ends with a mark).
/// @param[in] max Size of usecs.
/// @param[in] timeout Ticks to wait.
/// @return Nr. of durations, 0 on timeout. A message longer than max is cut.
size_t ir_rmt_rx_wait(uint16_t *usecs, size_t max, TickType_t timeout);

#endif  // IRREMOTE_ESP_IDF_IR_RMT_RX_H_
