// Minimal Arduino API for IRremoteESP8266 on ESP-IDF (no Arduino framework).
// Part of the ESP-IDF port of IRremoteESP8266, see README_ESP_IDF.md.
//
// Only what the library uses outside of its UNIT_TEST mode: String, F(), timing, GPIO and ESP.restart().
// The IR output does not use digitalWrite()/delayMicroseconds(): IRsend collects the marks and spaces
// and sends them with the RMT peripheral (ir_rmt_tx.h).

#ifndef IRREMOTE_ESP_IDF_ARDUINO_H_
#define IRREMOTE_ESP_IDF_ARDUINO_H_

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <string>

#include <driver/gpio.h>
#include <esp_rom_sys.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

typedef std::string String;

#ifndef F
#define F(x) x
#endif

#ifndef HIGH
#define HIGH 0x1
#endif
#ifndef LOW
#define LOW 0x0
#endif
#ifndef INPUT
#define INPUT 0x01
#endif
#ifndef OUTPUT
#define OUTPUT 0x03
#endif
#ifndef INPUT_PULLUP
#define INPUT_PULLUP 0x05
#endif

inline uint32_t micros() { return static_cast<uint32_t>(esp_timer_get_time()); }
inline uint32_t millis() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }
inline void delayMicroseconds(uint32_t usec) { esp_rom_delay_us(usec); }
inline void delay(uint32_t msec) { vTaskDelay(pdMS_TO_TICKS(msec) > 0 ? pdMS_TO_TICKS(msec) : 1); }

inline void pinMode(uint16_t pin, uint8_t mode)
{
    gpio_config_t io_config = {};
    io_config.pin_bit_mask = 1ULL << pin;
    io_config.mode = mode == OUTPUT ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT;
    io_config.pull_up_en = mode == INPUT_PULLUP ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    gpio_config(&io_config);
}

inline void digitalWrite(uint16_t pin, uint8_t level) { gpio_set_level(static_cast<gpio_num_t>(pin), level); }
inline int digitalRead(uint16_t pin) { return gpio_get_level(static_cast<gpio_num_t>(pin)); }

class IRremoteEspClass {
public:
    void restart() { esp_restart(); }
};
inline IRremoteEspClass ESP;

#endif  // IRREMOTE_ESP_IDF_ARDUINO_H_
