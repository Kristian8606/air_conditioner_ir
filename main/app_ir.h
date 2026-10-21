/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once

#include <stdint.h>

/* Air conditioner modes, independent of IRremoteESP8266 (stdAc::opmode_t is mapped in app_ir.cpp) */
typedef enum {
    APP_IR_MODE_AUTO,
    APP_IR_MODE_COOL,
    APP_IR_MODE_HEAT,
    APP_IR_MODE_DRY,
    APP_IR_MODE_FAN,
} app_ir_mode_t;

typedef enum {
    APP_IR_FAN_AUTO,
    APP_IR_FAN_LOW,
    APP_IR_FAN_MEDIUM,
    APP_IR_FAN_HIGH,
} app_ir_fan_t;

/* The whole state of the air conditioner: IR remotes of air conditioners always send all of it in one message */
typedef struct {
    bool power;
    app_ir_mode_t mode;
    int16_t setpoint;       /* 0.01 degC */
    app_ir_fan_t fan;
} app_ir_state_t;

/** Start the IR task. The IR transmitter (menuconfig: IR air conditioner) is set up with the first message. */
void app_ir_init();

/** Send the state to the air conditioner (from the IR task, the call does not wait).
 *
 * If several states are queued before the IR task gets to them, only the newest one is sent.
 */
void app_ir_send(const app_ir_state_t *state);

/** Select the next model of the remote (1 .. IR_AC_MODEL_MAX, then 1 again) and keep it in NVS.
 *
 * The model is used from the next message on. To try the variants of a protocol without rebuilding.
 * @return The new model.
 */
int16_t app_ir_next_model();
