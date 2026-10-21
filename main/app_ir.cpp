/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <stdlib.h>

/* The application headers (and with them the CHIP headers) come first: the Arduino shim of IRremoteESP8266
 * defines macros like F(), HIGH and LOW, which must not reach them. */
#include <app_ir.h>
#include <app_priv.h>

#include <IRac.h>
#include <IRremoteESP8266.h>

static const char *TAG = "app_ir";

/* A bool option which is off is not defined at all in sdkconfig.h */
#if CONFIG_IR_TX_INVERTED
#define APP_IR_TX_INVERTED true
#else
#define APP_IR_TX_INVERTED false
#endif

#define APP_IR_TASK_STACK_SIZE 6144
#define APP_IR_TASK_PRIORITY 5
/* Moving a slider writes several values in a row: only the last one is sent, after this pause */
#define APP_IR_SETTLE_MS 500

/* One entry: a newer state replaces the one which was not sent yet */
static QueueHandle_t app_ir_queue = nullptr;

static stdAc::opmode_t app_ir_opmode(app_ir_mode_t mode)
{
    switch (mode) {
    case APP_IR_MODE_COOL:
        return stdAc::opmode_t::kCool;
    case APP_IR_MODE_HEAT:
        return stdAc::opmode_t::kHeat;
    case APP_IR_MODE_DRY:
        return stdAc::opmode_t::kDry;
    case APP_IR_MODE_FAN:
        return stdAc::opmode_t::kFan;
    case APP_IR_MODE_AUTO:
    default:
        return stdAc::opmode_t::kAuto;
    }
}

static stdAc::fanspeed_t app_ir_fanspeed(app_ir_fan_t fan)
{
    switch (fan) {
    case APP_IR_FAN_LOW:
        return stdAc::fanspeed_t::kLow;
    case APP_IR_FAN_MEDIUM:
        return stdAc::fanspeed_t::kMedium;
    case APP_IR_FAN_HIGH:
        return stdAc::fanspeed_t::kHigh;
    case APP_IR_FAN_AUTO:
    default:
        return stdAc::fanspeed_t::kAuto;
    }
}

static void app_ir_task(void *arg)
{
    /* IRac keeps the previous state, protocols with toggle bits (e.g. swing, light) need it */
    IRac ac(CONFIG_IR_TX_GPIO, APP_IR_TX_INVERTED);
    app_ir_state_t state;

    while (true) {
        if (xQueueReceive(app_ir_queue, &state, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        while (xQueueReceive(app_ir_queue, &state, pdMS_TO_TICKS(APP_IR_SETTLE_MS)) == pdTRUE) {
            /* A newer state arrived, wait until the values settle */
        }

        stdAc::state_t next;
        IRac::initState(&next);
        next.protocol = decode_type_t::IR_AC_PROTOCOL_ENUM;
        next.model = CONFIG_IR_AC_MODEL;
        next.power = state.power;
        next.mode = app_ir_opmode(state.mode);
        next.degrees = state.setpoint / 100.0f;
        next.celsius = true;
        next.fanspeed = app_ir_fanspeed(state.fan);
        ac.next = next;

        static const char *const mode_names[] = {"Auto", "Cool", "Heat", "Dry", "Fan"};
        static const char *const fan_names[] = {"Auto", "Low", "Medium", "High"};
        bool sent = ac.sendAc();
        APP_LOG("[IR]       %s %s, power %s, mode %s, %d.%02d C, fan %s (GPIO%d)", sent ? "sent" : "NOT sent",
                CONFIG_IR_AC_PROTOCOL, state.power ? "ON" : "OFF", mode_names[state.mode], state.setpoint / 100,
                abs(state.setpoint % 100), fan_names[state.fan], CONFIG_IR_TX_GPIO);
    }
}

void app_ir_init()
{
    app_ir_queue = xQueueCreate(1, sizeof(app_ir_state_t));
    if (!app_ir_queue ||
            xTaskCreate(app_ir_task, "ir", APP_IR_TASK_STACK_SIZE, nullptr, APP_IR_TASK_PRIORITY, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start the IR task");
        return;
    }
    if (!IRac::isProtocolSupported(decode_type_t::IR_AC_PROTOCOL_ENUM)) {
        ESP_LOGE(TAG, "Protocol %s is not compiled in", CONFIG_IR_AC_PROTOCOL);
    }
    ESP_LOGI(TAG, "IR air conditioner: %s, model %d, transmitter GPIO%d", CONFIG_IR_AC_PROTOCOL, CONFIG_IR_AC_MODEL,
             CONFIG_IR_TX_GPIO);
}

void app_ir_send(const app_ir_state_t *state)
{
    if (app_ir_queue) {
        xQueueOverwrite(app_ir_queue, state);
    }
}
