/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once

#include <esp_err.h>
#include <esp_log.h>
#include <esp_matter.h>
#include <app_ir.h>

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include "esp_openthread_types.h"
#endif

/* All temperatures in this example use the Matter unit: 0.01 degC (2150 == 21.50 degC). */

/* MinSetpointDeadBand is in 0.1 degC units (menuconfig: Auto dead band, 20 == 2.0 degC). In Auto, the
 * cooling setpoint is always kept at least this much above the heating setpoint. */
#if CONFIG_AC_ENABLE_AUTO
#define THERMOSTAT_MIN_SETPOINT_DEAD_BAND CONFIG_AC_DEAD_BAND
#else
#define THERMOSTAT_MIN_SETPOINT_DEAD_BAND 0
#endif

/* Setpoint limits (menuconfig: Lowest/Highest setpoint). The Thermostat cluster rejects writes outside of
 * these ranges. With Auto the heating range ends one dead band below the highest setpoint and the cooling
 * range starts one dead band above the lowest one. Otherwise the Thermostat server would reject a heating
 * setpoint close to the maximum even in Heat mode, because it could not keep the dead band to the cooling
 * setpoint (the controller then shows the old value again). */
#define THERMOSTAT_DEAD_BAND_CENTI (THERMOSTAT_MIN_SETPOINT_DEAD_BAND * 10)
#if CONFIG_AC_ENABLE_AUTO
#define THERMOSTAT_ABS_MIN_HEAT_SETPOINT (CONFIG_AC_MIN_SETPOINT * 100)
#define THERMOSTAT_ABS_MAX_HEAT_SETPOINT (CONFIG_AC_MAX_SETPOINT * 100 - THERMOSTAT_DEAD_BAND_CENTI)
#define THERMOSTAT_ABS_MIN_COOL_SETPOINT (CONFIG_AC_MIN_SETPOINT * 100 + THERMOSTAT_DEAD_BAND_CENTI)
#define THERMOSTAT_ABS_MAX_COOL_SETPOINT (CONFIG_AC_MAX_SETPOINT * 100)
#else
#define THERMOSTAT_ABS_MIN_HEAT_SETPOINT (CONFIG_AC_MIN_SETPOINT * 100)
#define THERMOSTAT_ABS_MAX_HEAT_SETPOINT (CONFIG_AC_MAX_SETPOINT * 100)
#define THERMOSTAT_ABS_MIN_COOL_SETPOINT (CONFIG_AC_MIN_SETPOINT * 100)
#define THERMOSTAT_ABS_MAX_COOL_SETPOINT (CONFIG_AC_MAX_SETPOINT * 100)
#endif

/* LocalTemperatureCalibration range in 0.1 degC (-2.5 .. +2.5 degC, as in the Matter specification) */
#define THERMOSTAT_MIN_CALIBRATION (-25)
#define THERMOSTAT_MAX_CALIBRATION 25

/* Default setpoints, used on the very first boot only. Afterwards they are restored from NVS. */
#define THERMOSTAT_DEFAULT_HEAT_SETPOINT 2100  /* 21 degC */
#define THERMOSTAT_DEFAULT_COOL_SETPOINT 2500  /* 25 degC */

/* Temporary debug logs of the attribute writes (menuconfig: Trace attribute writes) */
#if CONFIG_AC_TRACE_LOGS
#define APP_TRACE(tag, format, ...) ESP_LOGW(tag, "[TRACE] " format, ##__VA_ARGS__)
#else
#define APP_TRACE(tag, format, ...) do {} while (0)
#endif

/* Distinctive log lines for the values that matter: sensor values, power, mode, setpoints and fan.
 * They all use the THERMOSTAT tag, so they can also be filtered (e.g. `idf.py monitor | grep THERMOSTAT`). */
#define APP_LOG_TAG "THERMOSTAT"
#define APP_LOG(format, ...) ESP_LOGI(APP_LOG_TAG, "========== " format, ##__VA_ARGS__)

/** Name of a Thermostat SystemMode value for the logs */
static inline const char *app_system_mode_name(uint8_t mode)
{
    static const char *const names[] = {"Off", "Auto", "Reserved", "Cool", "Heat", "EmergencyHeat",
                                        "Precooling", "FanOnly", "Dry", "Sleep"};
    return mode < sizeof(names) / sizeof(names[0]) ? names[mode] : "Unknown";
}

/** Value of a numeric attribute as int32 for the trace logs (INT32_MIN for null) */
int32_t app_trace_value(const esp_matter_attr_val_t *val);

typedef void *app_driver_handle_t;

/** Initialize the thermostat driver
 *
 * This initializes the status LED and the button.
 *
 * @return Handle on success.
 * @return NULL in case of failure.
 */
app_driver_handle_t app_driver_thermostat_init();

/** Start the thermostat control logic
 *
 * This must be called after esp_matter::start(). It subscribes to the bound remote sensor(s)
 * and starts the periodic sensor timeout check.
 *
 * @param[in] endpoint_id Thermostat endpoint id.
 */
void app_driver_thermostat_start(uint16_t endpoint_id);

/** Re-evaluate the thermostat
 *
 * Reads the system mode and the setpoints from the data model, compares them with the last
 * temperature received from the remote sensor and updates the running state and the LED.
 * It is scheduled on the Matter thread, so it is safe to call from any task.
 */
void app_driver_thermostat_schedule_update();

/** LocalTemperatureCalibration was changed: recalculate LocalTemperature from the last sensor value
 *
 * It is scheduled on the Matter thread, so it is safe to call from any task.
 */
void app_driver_calibration_changed();

/** The whole IR state of the air conditioner from the Matter attributes (OnOff, SystemMode, setpoints, FanMode).
 *
 * Called by the IR TX task. The caller must hold the Matter stack lock (PlatformMgr().LockChipStack()).
 * @return false before the endpoint exists
 */
bool app_driver_ir_read_state(app_ir_state_t *state);

/** Write the state of the original remote into Matter (OnOff, SystemMode, setpoint, FanMode), changed values only.
 *
 * Called by the IR RX task. The caller must hold the Matter stack lock (PlatformMgr().LockChipStack()).
 */
void app_driver_ir_apply_remote(const app_ir_state_t *state);

/** Delegate of the FanControl server (FanControl::Delegate), for cluster::fan_control::config_t::delegate */
void *app_driver_fan_control_delegate();

/** Handle a change of the binding table
 *
 * (Re)subscribes to the TemperatureMeasurement and RelativeHumidityMeasurement servers
 * that are bound to the thermostat endpoint.
 */
void app_driver_bindings_changed();

/** Start the subscriptions after the binding table has been loaded from NVS
 *
 * Call it on kDnssdInitialized, it waits until esp-matter has loaded the binding table.
 */
void app_driver_bindings_loaded();

/** Identify: blink the status LED until app_driver_identify_stop(). Call it on the Matter thread. */
void app_driver_identify_start();

/** Identify: stop blinking and show the mode again. Call it on the Matter thread. */
void app_driver_identify_stop();

/** Identify TriggerEffect: run a short LED pattern for the effect. Call it on the Matter thread. */
void app_driver_identify_effect(uint8_t effect_id);

/** Set the BasicInformation values (vendor/product name, serial number, hardware version) from menuconfig
 *
 * This should be called before esp_matter::start().
 */
void app_device_info_provider_init();

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#define ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG()                                           \
    {                                                                                   \
        .radio_mode = RADIO_MODE_NATIVE,                                                \
    }

#define ESP_OPENTHREAD_DEFAULT_HOST_CONFIG()                                            \
    {                                                                                   \
        .host_connection_mode = HOST_CONNECTION_MODE_NONE,                              \
    }

#define ESP_OPENTHREAD_DEFAULT_PORT_CONFIG()                                            \
    {                                                                                   \
        .storage_partition_name = "nvs", .netif_queue_size = 10, .task_queue_size = 10, \
    }
#endif
