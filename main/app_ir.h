/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once

#include <stddef.h>
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
    bool swing_v;           /* Louvers swing up and down */
    bool swing_h;           /* Louvers swing left and right */
} app_ir_state_t;

/** Start the IR task. The IR transmitter (menuconfig: IR air conditioner) is set up with the first message. */
void app_ir_init();

/** Tell the IR TX task that power, mode, a setpoint or the fan changed in Matter (any task, does not wait).
 *
 * This is all Matter does for IR: the IR TX task reads the attributes itself (with the Matter stack locked),
 * builds the message and sends it when the state changed. Several notifications in a row are sent once.
 * @param force send even if nothing changed (BOOT button)
 */
void app_ir_notify(bool force);

/** Select the next model of the remote (1 .. IR_AC_MODEL_MAX, then 1 again) and keep it in NVS.
 *
 * The model is used from the next message on. To try the variants of a protocol without rebuilding.
 * @return The new model.
 */
int16_t app_ir_next_model();

/** The protocol and the model of the remote for the controllers, e.g. "FUJITSU_AC ARRAH2E" or "COOLIX model 1".
 *
 * Shown in the Hardware field of Apple Home and Home Assistant (HardwareVersionString).
 */
void app_ir_describe(char *buf, size_t size);
