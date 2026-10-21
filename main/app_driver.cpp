/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_log.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <stdlib.h>

#include <esp_matter.h>
#include "bsp/esp-bsp.h"
#include <led_strip.h>

#include <app_priv.h>

#include <app/AttributePathParams.h>
#include <app/ConcreteAttributePath.h>
#include <app/InteractionModelEngine.h>
#include <app/ReadClient.h>
#include <app/clusters/bindings/binding-table.h>
#include <app/data-model/Decode.h>
#include <app/server/Server.h>
#include <clusters/relative_humidity_measurement/integration.h>
#include <app/clusters/power-source-server/power-source-server.h>
#include <app/clusters/fan-control-server/fan-control-delegate.h>
#include <inttypes.h>
#include <lib/core/TLVReader.h>

using namespace chip::app::Clusters;
using namespace esp_matter;
using chip::app::ReadClient;

static const char *TAG = "app_driver";

/* attribute::update() with a trace log of the value and the reason (Trace attribute writes in menuconfig) */
static esp_err_t app_driver_update_attribute(uint32_t cluster_id, uint32_t attribute_id, esp_matter_attr_val_t *val,
                                             const char *reason);

static uint16_t thermostat_endpoint_id = chip::kInvalidEndpointId;

/* Last values received from the bound remote sensor. The thermostat compares these variables with the
 * setpoints, they are updated on every new report from the sensor. */
static int16_t remote_temperature = 0;          /* 0.01 degC, calibrated and rounded */
static int16_t remote_sensor_temperature = 0;   /* 0.01 degC, as reported by the sensor */
static bool remote_temperature_valid = false;
static int64_t remote_temperature_last_seen_us = 0;
static uint16_t remote_temperature_max_interval = 0;  /* negotiated with the sensor, seconds */
static uint16_t remote_humidity = 0;            /* 0.01 % */
static bool remote_humidity_valid = false;

/* The button is held long enough for a factory reset, released it resets the device */
static bool factory_reset_pending = false;

/* Identify: the LED blinks in identify_color. identify_toggles_left is the number of LED changes left,
 * -1 blinks until the Identify server stops it (IdentifyTime reached 0). */
#define IDENTIFY_BLINK_PERIOD_MS 500
static bool identify_active = false;
static bool identify_led_on = false;
static int identify_toggles_left = 0;
static uint8_t identify_color[3] = {0, 0, 0};

/* Current state of the outputs */
static bool heating_on = false;
static bool cooling_on = false;

static led_strip_handle_t status_led = nullptr;

#define RELAY_LEVEL(on) ((on) == CONFIG_THERMOSTAT_RELAY_ACTIVE_HIGH ? 1 : 0)
#define SENSOR_CHECK_PERIOD_SEC 10
#define SUBSCRIBE_RETRY_PERIOD_SEC 30

/*******************************************************************************
 * Relays and status LED
 ******************************************************************************/

static void app_driver_relay_init(int gpio_num)
{
    /* Set the inactive level before switching the pin to output, so the relay never clicks on boot. */
    gpio_set_level((gpio_num_t)gpio_num, RELAY_LEVEL(false));
    gpio_config_t io_config = {};
    io_config.pin_bit_mask = 1ULL << gpio_num;
    io_config.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&io_config));
}

static void app_driver_set_heating(bool on)
{
    if (heating_on != on) {
        APP_LOG("[RELAY]    HEAT %s (GPIO%d)", on ? "ON" : "OFF", CONFIG_THERMOSTAT_HEAT_RELAY_GPIO);
    }
    heating_on = on;
    gpio_set_level((gpio_num_t)CONFIG_THERMOSTAT_HEAT_RELAY_GPIO, RELAY_LEVEL(on));
}

static void app_driver_set_cooling(bool on)
{
#if CONFIG_THERMOSTAT_ENABLE_COOLING
    if (cooling_on != on) {
        APP_LOG("[RELAY]    COOL %s (GPIO%d)", on ? "ON" : "OFF", CONFIG_THERMOSTAT_COOL_RELAY_GPIO);
    }
#endif
    cooling_on = on;
#if CONFIG_THERMOSTAT_ENABLE_COOLING
    gpio_set_level((gpio_num_t)CONFIG_THERMOSTAT_COOL_RELAY_GPIO, RELAY_LEVEL(on));
#endif
}

static void app_driver_status_led_init()
{
#if CONFIG_THERMOSTAT_STATUS_LED_GPIO >= 0
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = CONFIG_THERMOSTAT_STATUS_LED_GPIO;
    strip_config.max_leds = 1;
    strip_config.led_model = LED_MODEL_WS2812;
    strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rmt_config.resolution_hz = 10 * 1000 * 1000;

    if (led_strip_new_rmt_device(&strip_config, &rmt_config, &status_led) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize the status LED");
        status_led = nullptr;
        return;
    }
    led_strip_clear(status_led);
#endif
}

static void app_driver_status_led_set(uint8_t red, uint8_t green, uint8_t blue)
{
    if (!status_led) {
        return;
    }
    if (red == 0 && green == 0 && blue == 0) {
        led_strip_clear(status_led);
        return;
    }
    led_strip_set_pixel(status_led, 0, red, green, blue);
    led_strip_refresh(status_led);
}

static esp_err_t app_driver_update_attribute(uint32_t cluster_id, uint32_t attribute_id, esp_matter_attr_val_t *val,
                                             const char *reason)
{
    APP_TRACE(TAG, "app update cluster 0x%04" PRIx32 " attr 0x%04" PRIx32 " = %" PRId32 " (%s)", cluster_id,
              attribute_id, app_trace_value(val), reason);
    esp_err_t err = attribute::update(thermostat_endpoint_id, cluster_id, attribute_id, val);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update cluster 0x%04" PRIx32 " attr 0x%04" PRIx32 ": %d", cluster_id, attribute_id,
                 err);
    }
    return err;
}

/*******************************************************************************
 * Thermostat control
 ******************************************************************************/

static uint8_t app_driver_get_system_mode()
{
    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    if (attribute::get_val(thermostat_endpoint_id, Thermostat::Id, Thermostat::Attributes::SystemMode::Id, &val) !=
            ESP_OK) {
        return chip::to_underlying(Thermostat::SystemModeEnum::kOff);
    }
    return val.val.u8;
}

static int16_t app_driver_get_setpoint(uint32_t attribute_id, int16_t default_value)
{
    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    if (attribute::get_val(thermostat_endpoint_id, Thermostat::Id, attribute_id, &val) != ESP_OK) {
        return default_value;
    }
    return val.val.i16;
}

/* Heating demand. ON at (setpoint - hysteresis), OFF when the setpoint is reached. The values are compared as
 * Matter reports them, in 0.01 degC. */
static bool app_driver_heating_demand(int16_t temperature, int16_t heating_setpoint)
{
    if (heating_on) {
        return temperature < heating_setpoint;
    }
    return temperature < heating_setpoint && temperature <= heating_setpoint - CONFIG_THERMOSTAT_HYSTERESIS;
}

#if CONFIG_THERMOSTAT_ENABLE_COOLING
/* Cooling demand. ON at (setpoint + hysteresis), OFF when the setpoint is reached. */
static bool app_driver_cooling_demand(int16_t temperature, int16_t cooling_setpoint)
{
    if (cooling_on) {
        return temperature > cooling_setpoint;
    }
    return temperature > cooling_setpoint && temperature >= cooling_setpoint + CONFIG_THERMOSTAT_HYSTERESIS;
}
#endif

static void app_driver_update_running_state()
{
    /* Only report a change, not every evaluation */
    static int32_t last_running_state = -1;
    uint16_t running_state = 0;
    if (heating_on) {
        running_state |= chip::to_underlying(Thermostat::RelayStateBitmap::kHeat);
    }
    if (cooling_on) {
        running_state |= chip::to_underlying(Thermostat::RelayStateBitmap::kCool);
    }
    if (running_state == last_running_state) {
        return;
    }
    last_running_state = running_state;

    esp_matter_attr_val_t val = esp_matter_bitmap16(running_state);
    app_driver_update_attribute(Thermostat::Id, Thermostat::Attributes::ThermostatRunningState::Id, &val,
                                "relay changed");

#if CONFIG_THERMOSTAT_ENABLE_AUTO
    Thermostat::ThermostatRunningModeEnum running_mode = Thermostat::ThermostatRunningModeEnum::kOff;
    if (heating_on) {
        running_mode = Thermostat::ThermostatRunningModeEnum::kHeat;
    } else if (cooling_on) {
        running_mode = Thermostat::ThermostatRunningModeEnum::kCool;
    }
    val = esp_matter_enum8(chip::to_underlying(running_mode));
    app_driver_update_attribute(Thermostat::Id, Thermostat::Attributes::ThermostatRunningMode::Id, &val,
                                "relay changed");
#endif
}

/* The LED shows the state of the relays */
static void app_driver_update_status_led()
{
    if (factory_reset_pending) {
        app_driver_status_led_set(50, 0, 50);       /* Purple: release the button for a factory reset */
    } else if (identify_active) {
        if (identify_led_on) {                      /* Blinking: Identify */
            app_driver_status_led_set(identify_color[0], identify_color[1], identify_color[2]);
        } else {
            app_driver_status_led_set(0, 0, 0);
        }
    } else if (heating_on) {
        app_driver_status_led_set(60, 20, 0);       /* Orange: heating relay on */
    } else if (cooling_on) {
        app_driver_status_led_set(0, 0, 50);        /* Blue: cooling relay on */
    } else {
        app_driver_status_led_set(25, 25, 25);      /* White: both relays off */
    }
}

/*******************************************************************************
 * Identify
 ******************************************************************************/

static void app_driver_identify_timer_cb(chip::System::Layer *system_layer, void *app_state);

static void app_driver_identify_end()
{
    chip::DeviceLayer::SystemLayer().CancelTimer(app_driver_identify_timer_cb, nullptr);
    if (identify_active) {
        APP_LOG("[IDENTIFY] stop");
    }
    identify_active = false;
    identify_led_on = false;
    app_driver_update_status_led();
}

static void app_driver_identify_timer_cb(chip::System::Layer *system_layer, void *app_state)
{
    if (identify_toggles_left > 0) {
        identify_toggles_left--;
    }
    if (identify_toggles_left == 0) {
        app_driver_identify_end();
        return;
    }
    identify_led_on = !identify_led_on;
    app_driver_update_status_led();
    if (chip::DeviceLayer::SystemLayer().StartTimer(chip::System::Clock::Milliseconds32(IDENTIFY_BLINK_PERIOD_MS),
                                                    app_driver_identify_timer_cb, nullptr) != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "Failed to start the identify timer");
        app_driver_identify_end();
    }
}

/* Start blinking: toggles is the number of LED changes (2 per blink), -1 until app_driver_identify_stop() */
static void app_driver_identify_blink(uint8_t red, uint8_t green, uint8_t blue, int toggles)
{
    identify_color[0] = red;
    identify_color[1] = green;
    identify_color[2] = blue;
    identify_toggles_left = toggles;
    identify_active = true;
    identify_led_on = true;
    app_driver_update_status_led();
    /* StartTimer() replaces a timer that is already running with the same callback */
    if (chip::DeviceLayer::SystemLayer().StartTimer(chip::System::Clock::Milliseconds32(IDENTIFY_BLINK_PERIOD_MS),
                                                    app_driver_identify_timer_cb, nullptr) != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "Failed to start the identify timer");
        app_driver_identify_end();
    }
}

void app_driver_identify_start()
{
    APP_LOG("[IDENTIFY] start");
    app_driver_identify_blink(25, 25, 25, -1);      /* White, until IdentifyTime reaches 0 */
}

void app_driver_identify_stop()
{
    app_driver_identify_end();
}

void app_driver_identify_effect(uint8_t effect_id)
{
    using chip::app::Clusters::Identify::EffectIdentifierEnum;
    switch (static_cast<EffectIdentifierEnum>(effect_id)) {
    case EffectIdentifierEnum::kBlink:              /* One blink */
        APP_LOG("[IDENTIFY] effect Blink");
        app_driver_identify_blink(25, 25, 25, 2);
        break;
    case EffectIdentifierEnum::kBreathe:            /* 15 blinks */
        APP_LOG("[IDENTIFY] effect Breathe");
        app_driver_identify_blink(25, 25, 25, 30);
        break;
    case EffectIdentifierEnum::kOkay:               /* Two green blinks */
        APP_LOG("[IDENTIFY] effect Okay");
        app_driver_identify_blink(0, 50, 0, 4);
        break;
    case EffectIdentifierEnum::kChannelChange:      /* Orange for 8 s */
        APP_LOG("[IDENTIFY] effect ChannelChange");
        app_driver_identify_blink(60, 20, 0, 16);
        break;
    case EffectIdentifierEnum::kFinishEffect:       /* Finish the current blink, then stop */
        APP_LOG("[IDENTIFY] effect FinishEffect");
        if (identify_active) {
            identify_toggles_left = identify_led_on ? 1 : 2;
        }
        break;
    case EffectIdentifierEnum::kStopEffect:
    default:
        APP_LOG("[IDENTIFY] effect StopEffect (%u)", effect_id);
        app_driver_identify_end();
        break;
    }
}

/* Compare the remote temperature with the setpoints and drive the relays.
 *
 * The heating and the cooling setpoints are read into separate variables, they are never mixed:
 *  - Heat: only the heating setpoint is used, cooling is always off.
 *  - Cool: only the cooling setpoint is used, heating is always off.
 *  - Auto: heating is controlled by the heating setpoint and cooling by the cooling setpoint. The
 *          Thermostat server keeps cooling setpoint >= heating setpoint + MinSetpointDeadBand, so
 *          both demands can never be true at the same time. */
/* OnOff of the Room Air Conditioner (false if it cannot be read) */
static bool app_driver_get_power()
{
    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    if (attribute::get_val(thermostat_endpoint_id, OnOff::Id, OnOff::Attributes::OnOff::Id, &val) != ESP_OK) {
        return false;
    }
    return val.val.b;
}

static void app_driver_thermostat_update(intptr_t arg)
{
    if (thermostat_endpoint_id == chip::kInvalidEndpointId) {
        return;
    }

    /* Room Air Conditioner: with OnOff off the unit is off, whatever SystemMode is */
    bool power = app_driver_get_power();
    Thermostat::SystemModeEnum system_mode = power ? static_cast<Thermostat::SystemModeEnum>(app_driver_get_system_mode())
                                                   : Thermostat::SystemModeEnum::kOff;
    int16_t heating_setpoint = app_driver_get_setpoint(Thermostat::Attributes::OccupiedHeatingSetpoint::Id,
                                                       THERMOSTAT_DEFAULT_HEAT_SETPOINT);
#if CONFIG_THERMOSTAT_ENABLE_COOLING
    int16_t cooling_setpoint = app_driver_get_setpoint(Thermostat::Attributes::OccupiedCoolingSetpoint::Id,
                                                       THERMOSTAT_DEFAULT_COOL_SETPOINT);
#endif

#if CONFIG_THERMOSTAT_ENABLE_COOLING
    APP_TRACE(TAG, "control reads mode %u, heat %d, cool %d", chip::to_underlying(system_mode), heating_setpoint,
              cooling_setpoint);
#else
    APP_TRACE(TAG, "control reads mode %u, heat %d", chip::to_underlying(system_mode), heating_setpoint);
#endif

    bool heat = false;
    bool cool = false;

    if (remote_temperature_valid) {
        switch (system_mode) {
        case Thermostat::SystemModeEnum::kHeat:
        case Thermostat::SystemModeEnum::kEmergencyHeat:
            heat = app_driver_heating_demand(remote_temperature, heating_setpoint);
            break;
#if CONFIG_THERMOSTAT_ENABLE_COOLING
        case Thermostat::SystemModeEnum::kCool:
        case Thermostat::SystemModeEnum::kPrecooling:
            cool = app_driver_cooling_demand(remote_temperature, cooling_setpoint);
            break;
#endif
#if CONFIG_THERMOSTAT_ENABLE_AUTO
        case Thermostat::SystemModeEnum::kAuto:
            heat = app_driver_heating_demand(remote_temperature, heating_setpoint);
            cool = app_driver_cooling_demand(remote_temperature, cooling_setpoint);
            break;
#endif
        default:
            /* Off, FanOnly, Dry, Sleep and the unsupported modes: no heating and no cooling. */
            break;
        }
    }

    if (heat && cool) {
        /* Should never happen, but never run both relays at the same time. */
        ESP_LOGW(TAG, "Heating and cooling demand at the same time, switching both off");
        heat = false;
        cool = false;
    }

    /* Always switch the active relay off before the other one is switched on. */
    if (!heat) {
        app_driver_set_heating(false);
    }
    if (!cool) {
        app_driver_set_cooling(false);
    }
    app_driver_set_heating(heat);
    app_driver_set_cooling(cool);

#if CONFIG_THERMOSTAT_ENABLE_COOLING
    APP_LOG("[STATE]    power %s, mode %s, temperature %s%d.%02d C, heat %d.%02d C, cool %d.%02d C -> %s",
             power ? "ON" : "OFF", app_system_mode_name(chip::to_underlying(system_mode)),
             remote_temperature < 0 ? "-" : "",
             abs(remote_temperature) / 100, abs(remote_temperature) % 100,
             heating_setpoint / 100, heating_setpoint % 100, cooling_setpoint / 100, cooling_setpoint % 100,
             !remote_temperature_valid ? "no sensor data" : heat ? "heating" : cool ? "cooling" : "idle");
#else
    APP_LOG("[STATE]    power %s, mode %s, temperature %s%d.%02d C, heat %d.%02d C -> %s",
             power ? "ON" : "OFF", app_system_mode_name(chip::to_underlying(system_mode)),
             remote_temperature < 0 ? "-" : "",
             abs(remote_temperature) / 100, abs(remote_temperature) % 100,
             heating_setpoint / 100, heating_setpoint % 100,
             !remote_temperature_valid ? "no sensor data" : heat ? "heating" : "idle");
#endif

    app_driver_update_running_state();
    app_driver_update_status_led();
}

void app_driver_thermostat_schedule_update()
{
    if (chip::DeviceLayer::PlatformMgr().ScheduleWork(app_driver_thermostat_update, 0) != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "Failed to schedule the thermostat update");
    }
}

/*******************************************************************************
 * Fan Control
 ******************************************************************************/

/* FanControl is a code-driven cluster: its attributes are not in the esp-matter attribute storage and a write
 * does not call app_attribute_update_cb(). The FanControl server keeps the values (FanMode also in NVS) and tells
 * the application through this delegate. */
class AppFanControlDelegate : public FanControl::Delegate {
public:
    chip::Protocols::InteractionModel::Status HandleStep(FanControl::StepDirectionEnum direction, bool wrap,
                                                         bool lowest_off) override
    {
        /* The Step feature is not enabled */
        return chip::Protocols::InteractionModel::Status::UnsupportedCommand;
    }

    void OnFanDriveStateChanged(const FanControl::FanDriveState &state) override
    {
        static const char *const mode_names[] = {"Off", "Low", "Medium", "High", "On", "Auto", "Smart"};
        uint8_t mode = chip::to_underlying(state.mode);
        APP_LOG("[FAN]      %s, percent %d (setting %d), speed %d (setting %d) (EP%u FanMode 0x0202/0x0000 = %u)",
                mode < 7 ? mode_names[mode] : "Unknown", state.percentCurrent,
                state.percentSetting.IsNull() ? -1 : state.percentSetting.Value(), state.speedCurrent,
                state.speedSetting.IsNull() ? -1 : state.speedSetting.Value(), thermostat_endpoint_id, mode);
        /* The IR command with the whole state is sent from here in a later version */
    }
};

static AppFanControlDelegate fan_control_delegate;

void *app_driver_fan_control_delegate()
{
    return &fan_control_delegate;
}

/*******************************************************************************
 * Values from the remote sensor
 ******************************************************************************/

/* LocalTemperatureCalibration in 0.1 degC (0 if it cannot be read) */
static int8_t app_driver_get_calibration()
{
    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    if (attribute::get_val(thermostat_endpoint_id, Thermostat::Id, Thermostat::Attributes::LocalTemperatureCalibration::Id,
                           &val) != ESP_OK) {
        return 0;
    }
    return val.val.i8;
}

/* Store a new temperature from the sensor, show it as LocalTemperature (CurrentTemperature in Apple Home)
 * and re-evaluate the thermostat. The Thermostat server does not apply LocalTemperatureCalibration itself:
 * the calibration is added first, then the value is rounded to the temperature step. */
static void app_driver_remote_temperature_set(bool valid, int16_t sensor_temperature)
{
    int8_t calibration = app_driver_get_calibration();

    /* Calibration (0.1 degC) and rounding to the nearest temperature step (menuconfig), symmetric around 0 */
    int calibrated = sensor_temperature + calibration * 10;
    int step = CONFIG_THERMOSTAT_TEMPERATURE_STEP;
    int rounded = (abs(calibrated) + step / 2) / step * step;
    rounded = calibrated < 0 ? -rounded : rounded;
    int16_t temperature = (int16_t)(rounded > INT16_MAX ? INT16_MAX : rounded < -27315 ? -27315 : rounded);

    remote_temperature_valid = valid;
    remote_sensor_temperature = valid ? sensor_temperature : 0;
    remote_temperature = valid ? temperature : 0;

    nullable<int16_t> local_temperature;
    if (valid) {
        local_temperature = nullable<int16_t>(temperature);
        APP_LOG("[TEMP]     %s%d.%02d C (sensor %d, calibration %s%d.%d C, EP%u LocalTemperature 0x0201/0x0000 = %d)",
                temperature < 0 ? "-" : "", abs(temperature) / 100, abs(temperature) % 100, sensor_temperature,
                calibration < 0 ? "-" : "+", abs(calibration) / 10, abs(calibration) % 10, thermostat_endpoint_id,
                temperature);
    } else {
        APP_LOG("[TEMP]     unknown (EP%u LocalTemperature 0x0201/0x0000 = null)", thermostat_endpoint_id);
    }
    esp_matter_attr_val_t val = esp_matter_nullable_int16(local_temperature);
    app_driver_update_attribute(Thermostat::Id, Thermostat::Attributes::LocalTemperature::Id, &val, "sensor report");

    app_driver_thermostat_update(0);
}

static void app_driver_calibration_apply(intptr_t arg)
{
    int8_t calibration = app_driver_get_calibration();
    APP_LOG("[CALIBR]   %s%d.%d C (EP%u LocalTemperatureCalibration 0x0201/0x0010 = %d)", calibration < 0 ? "-" : "+",
            abs(calibration) / 10, abs(calibration) % 10, thermostat_endpoint_id, calibration);
    if (remote_temperature_valid) {
        /* Recalculate LocalTemperature from the last value of the sensor */
        app_driver_remote_temperature_set(true, remote_sensor_temperature);
    }
}

void app_driver_calibration_changed()
{
    if (chip::DeviceLayer::PlatformMgr().ScheduleWork(app_driver_calibration_apply, 0) != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "Failed to schedule the calibration update");
    }
}

static void app_driver_remote_humidity_set(bool valid, uint16_t humidity)
{
    remote_humidity_valid = valid;
    remote_humidity = valid ? humidity : 0;

    chip::app::DataModel::Nullable<uint16_t> measured_value;
    if (valid) {
        measured_value.SetNonNull(humidity);
        APP_LOG("[HUMIDITY] %d.%02d %% (EP%u MeasuredValue 0x0405/0x0000 = %u)", humidity / 100, humidity % 100,
                thermostat_endpoint_id, humidity);
    } else {
        APP_LOG("[HUMIDITY] unknown (EP%u MeasuredValue 0x0405/0x0000 = null)", thermostat_endpoint_id);
    }
    /* RelativeHumidityMeasurement is a code-driven cluster, its MeasuredValue is not kept in the esp-matter
     * attribute storage, so it is set through the cluster instead of attribute::update(). */
    CHIP_ERROR err = RelativeHumidityMeasurement::SetMeasuredValue(thermostat_endpoint_id, measured_value);
    if (err != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "Failed to set humidity: %" CHIP_ERROR_FORMAT, err.Format());
    }
}

/* Store the battery of the sensor and show it in the PowerSource cluster of the thermostat endpoint.
 * percent is in 0.5 % units (200 == 100 %), as in the BatPercentRemaining attribute. */
static void app_driver_remote_battery_set(bool valid, uint8_t percent)
{
    nullable<uint8_t> bat_percent_remaining;
    if (valid) {
        bat_percent_remaining = nullable<uint8_t>(percent);
        APP_LOG("[BATTERY]  %d%s %% (EP%u BatPercentRemaining 0x002F/0x000C = %u)", percent / 2,
                percent % 2 ? ".5" : "", thermostat_endpoint_id, percent);
    } else {
        APP_LOG("[BATTERY]  unknown (EP%u BatPercentRemaining 0x002F/0x000C = null)", thermostat_endpoint_id);
    }
    esp_matter_attr_val_t val = esp_matter_nullable_uint8(bat_percent_remaining);
    app_driver_update_attribute(PowerSource::Id, PowerSource::Attributes::BatPercentRemaining::Id, &val, "sensor report");
}

static void app_driver_remote_battery_level_set(uint8_t charge_level)
{
    static const char *const level_names[] = {"OK", "Warning", "Critical"};
    APP_LOG("[BATTERY]  level %s (EP%u BatChargeLevel 0x002F/0x000E = %u)",
            charge_level < 3 ? level_names[charge_level] : "unknown", thermostat_endpoint_id, charge_level);
    esp_matter_attr_val_t val = esp_matter_enum8(charge_level);
    app_driver_update_attribute(PowerSource::Id, PowerSource::Attributes::BatChargeLevel::Id, &val, "sensor report");
}

/*******************************************************************************
 * Subscription to the bound sensor(s)
 ******************************************************************************/

#define SENSOR_MAX_PATHS 4

/* One subscription to one remote sensor node. All the values bound to the same node (temperature, humidity
 * and battery, also from different endpoints of the node) come with a single subscription, so a battery
 * powered sensor has to send only one report. Sensors on different nodes get their own subscription. */
class sensor_subscription : public ReadClient::Callback {
public:
    void start(const chip::ScopedNodeId &peer, const chip::app::AttributePathParams *paths, size_t path_count)
    {
        if (m_client && m_peer == peer && same_paths(paths, path_count)) {
            /* Already subscribed to the same sensor */
            return;
        }
        stop();

        m_peer = peer;
        m_path_count = path_count;
        for (size_t i = 0; i < path_count; i++) {
            m_paths[i] = paths[i];
        }
        subscribe();
    }

    void stop()
    {
        if (m_client) {
            chip::Platform::Delete(m_client);
            m_client = nullptr;
        }
        m_path_count = 0;
    }

    /* Called periodically, retries a subscription which could not be sent. */
    void retry()
    {
        if (!m_client && m_path_count > 0) {
            subscribe();
        }
    }

    void OnAttributeData(const chip::app::ConcreteDataAttributePath &path, chip::TLV::TLVReader *data,
                         const chip::app::StatusIB &status) override
    {
        if (!has_path(path.mEndpointId, path.mClusterId, path.mAttributeId)) {
            return;
        }
        if (path.mClusterId == TemperatureMeasurement::Id) {
            chip::app::DataModel::Nullable<int16_t> value;
            if (!status.IsSuccess() || !data || chip::app::DataModel::Decode(*data, value) != CHIP_NO_ERROR ||
                    value.IsNull()) {
                app_driver_remote_temperature_set(false, 0);
                return;
            }
            remote_temperature_last_seen_us = esp_timer_get_time();
            app_driver_remote_temperature_set(true, value.Value());
        } else if (path.mClusterId == RelativeHumidityMeasurement::Id) {
            chip::app::DataModel::Nullable<uint16_t> value;
            if (!status.IsSuccess() || !data || chip::app::DataModel::Decode(*data, value) != CHIP_NO_ERROR ||
                    value.IsNull() || value.Value() > 10000) {
                app_driver_remote_humidity_set(false, 0);
                return;
            }
            app_driver_remote_humidity_set(true, value.Value());
        } else if (path.mAttributeId == PowerSource::Attributes::BatPercentRemaining::Id) {
            chip::app::DataModel::Nullable<uint8_t> value;
            if (!status.IsSuccess() || !data || chip::app::DataModel::Decode(*data, value) != CHIP_NO_ERROR ||
                    value.IsNull() || value.Value() > 200) {
                app_driver_remote_battery_set(false, 0);
                return;
            }
            app_driver_remote_battery_set(true, value.Value());
        } else if (path.mAttributeId == PowerSource::Attributes::BatChargeLevel::Id) {
            uint8_t value = 0;
            if (status.IsSuccess() && data && chip::app::DataModel::Decode(*data, value) == CHIP_NO_ERROR) {
                app_driver_remote_battery_level_set(value);
            }
        }
    }

    void OnReportEnd() override
    {
        /* Also called for the empty keep-alive reports sent at max interval when the value did not change,
         * so the last known temperature stays valid while the subscription is alive. */
        if (has_temperature() && remote_temperature_valid) {
            remote_temperature_last_seen_us = esp_timer_get_time();
        }
    }

    void OnSubscriptionEstablished(chip::SubscriptionId subscription_id) override
    {
        uint16_t min_interval = 0, max_interval = 0;
        if (m_client && m_client->GetReportingIntervals(min_interval, max_interval) != CHIP_NO_ERROR) {
            min_interval = max_interval = 0;
        }
        ESP_LOGI(TAG, "Subscribed to sensor node 0x" ChipLogFormatX64 " (%u values, interval %u..%u s)",
                 ChipLogValueX64(m_peer.GetNodeId()), (unsigned)m_path_count, min_interval, max_interval);
        if (has_temperature()) {
            /* A sleepy (ICD) sensor may choose a max interval longer than the requested one */
            remote_temperature_max_interval = max_interval;
        }
    }

    CHIP_ERROR OnResubscriptionNeeded(ReadClient *client, CHIP_ERROR termination_cause) override
    {
        /* The last value is kept until CONFIG_THERMOSTAT_SENSOR_TIMEOUT expires. */
        ESP_LOGW(TAG, "Sensor subscription lost: %" CHIP_ERROR_FORMAT ", resubscribing", termination_cause.Format());
        return client->DefaultResubscribePolicy(termination_cause);
    }

    void OnError(CHIP_ERROR error) override
    {
        ESP_LOGW(TAG, "Sensor subscription error: %" CHIP_ERROR_FORMAT, error.Format());
    }

    void OnDeallocatePaths(chip::app::ReadPrepareParams &&params) override
    {
        /* The attribute paths are owned by this object */
    }

    void OnDone(ReadClient *client) override
    {
        /* Auto resubscribe gave up. The periodic retry() starts a new subscription. */
        ESP_LOGW(TAG, "Sensor subscription done");
        if (m_client == client) {
            m_client = nullptr;
        }
        chip::Platform::Delete(client);
    }

private:
    bool has_path(chip::EndpointId endpoint, chip::ClusterId cluster, chip::AttributeId attribute) const
    {
        for (size_t i = 0; i < m_path_count; i++) {
            /* A path with a wildcard endpoint (the battery) matches the data from any endpoint */
            if ((m_paths[i].mEndpointId == endpoint || m_paths[i].HasWildcardEndpointId()) &&
                    m_paths[i].mClusterId == cluster && m_paths[i].mAttributeId == attribute) {
                return true;
            }
        }
        return false;
    }

    bool has_temperature() const
    {
        for (size_t i = 0; i < m_path_count; i++) {
            if (m_paths[i].mClusterId == TemperatureMeasurement::Id) {
                return true;
            }
        }
        return false;
    }

    bool same_paths(const chip::app::AttributePathParams *paths, size_t path_count) const
    {
        if (path_count != m_path_count) {
            return false;
        }
        for (size_t i = 0; i < path_count; i++) {
            if (!has_path(paths[i].mEndpointId, paths[i].mClusterId, paths[i].mAttributeId)) {
                return false;
            }
        }
        return true;
    }

    void subscribe()
    {
        m_client = chip::Platform::New<ReadClient>(chip::app::InteractionModelEngine::GetInstance(), nullptr, *this,
                                                    ReadClient::InteractionType::Subscribe);
        if (!m_client) {
            ESP_LOGE(TAG, "Failed to allocate the ReadClient");
            return;
        }

        chip::app::ReadPrepareParams params;
        params.mpAttributePathParamsList = m_paths;
        params.mAttributePathParamsListSize = m_path_count;
        params.mMinIntervalFloorSeconds = CONFIG_THERMOSTAT_SENSOR_MIN_INTERVAL;
        params.mMaxIntervalCeilingSeconds = CONFIG_THERMOSTAT_SENSOR_MAX_INTERVAL;
        /* Keep the other subscriptions when several sensors are bound */
        params.mKeepSubscriptions = true;
        params.mIsFabricFiltered = false;

        ESP_LOGI(TAG, "Subscribing to sensor node 0x" ChipLogFormatX64 " (%u values)",
                 ChipLogValueX64(m_peer.GetNodeId()), (unsigned)m_path_count);
        for (size_t i = 0; i < m_path_count; i++) {
            ESP_LOGI(TAG, "  endpoint %u, cluster 0x%04" PRIx32 ", attribute 0x%04" PRIx32, m_paths[i].mEndpointId,
                     m_paths[i].mClusterId, m_paths[i].mAttributeId);
        }
        /* Establishes the CASE session and resubscribes automatically when the sensor goes away */
        CHIP_ERROR err = m_client->SendAutoResubscribeRequest(m_peer, std::move(params));
        if (err != CHIP_NO_ERROR) {
            /* OnDone() is not called in this case, the ReadClient is deleted here */
            ESP_LOGE(TAG, "Failed to subscribe: %" CHIP_ERROR_FORMAT, err.Format());
            chip::Platform::Delete(m_client);
            m_client = nullptr;
        }
    }

    ReadClient *m_client = nullptr;
    chip::ScopedNodeId m_peer;
    chip::app::AttributePathParams m_paths[SENSOR_MAX_PATHS];
    size_t m_path_count = 0;
};

/* Temperature, humidity and battery can come from up to 3 different nodes */
#define SENSOR_MAX_NODES 3
static sensor_subscription sensor_subscriptions[SENSOR_MAX_NODES];

/* Find the unicast binding of the thermostat endpoint for the given cluster. If endpoint_binding_allowed, a
 * binding without a cluster id (it binds the whole remote endpoint) is also accepted. */
static const Binding::TableEntry *app_driver_find_binding(chip::ClusterId cluster_id, bool endpoint_binding_allowed)
{
    const Binding::TableEntry *endpoint_binding = nullptr;
    for (const auto &binding : Binding::Table::GetInstance()) {
        if (binding.type != Binding::MATTER_UNICAST_BINDING || binding.local != thermostat_endpoint_id) {
            continue;
        }
        if (binding.clusterId.has_value() && binding.clusterId.value() == cluster_id) {
            return &binding;
        }
        if (endpoint_binding_allowed && !binding.clusterId.has_value() && !endpoint_binding) {
            endpoint_binding = &binding;
        }
    }
    return endpoint_binding;
}

/* Attribute paths of one sensor node */
struct sensor_node {
    chip::ScopedNodeId peer;
    chip::app::AttributePathParams paths[SENSOR_MAX_PATHS];
    size_t path_count;
};

static void app_driver_add_sensor_path(sensor_node *nodes, size_t *node_count, const Binding::TableEntry *binding,
                                       chip::ClusterId cluster_id, chip::AttributeId attribute_id,
                                       chip::EndpointId endpoint_id)
{
    chip::ScopedNodeId peer(binding->nodeId, binding->fabricIndex);
    size_t i = 0;
    while (i < *node_count && !(nodes[i].peer == peer)) {
        i++;
    }
    if (i == *node_count) {
        nodes[i].peer = peer;
        nodes[i].path_count = 0;
        (*node_count)++;
    }
    if (nodes[i].path_count < SENSOR_MAX_PATHS) {
        nodes[i].paths[nodes[i].path_count++] = chip::app::AttributePathParams(endpoint_id, cluster_id, attribute_id);
    }
}

static void app_driver_update_subscriptions(intptr_t arg)
{
    /* A binding without a cluster id is used for the temperature and the humidity. The battery (PowerSource)
     * can be bound explicitly, otherwise it is read from the node of the temperature sensor. */
    const Binding::TableEntry *temperature_binding = app_driver_find_binding(TemperatureMeasurement::Id, true);
    const Binding::TableEntry *humidity_binding = app_driver_find_binding(RelativeHumidityMeasurement::Id, true);
    const Binding::TableEntry *battery_binding = app_driver_find_binding(PowerSource::Id, false);

    sensor_node nodes[SENSOR_MAX_NODES];
    size_t node_count = 0;
    if (temperature_binding) {
        app_driver_add_sensor_path(nodes, &node_count, temperature_binding, TemperatureMeasurement::Id,
                                   TemperatureMeasurement::Attributes::MeasuredValue::Id, temperature_binding->remote);
    }
    if (humidity_binding) {
        app_driver_add_sensor_path(nodes, &node_count, humidity_binding, RelativeHumidityMeasurement::Id,
                                   RelativeHumidityMeasurement::Attributes::MeasuredValue::Id, humidity_binding->remote);
    }
    /* The PowerSource of a sensor is usually on its root endpoint (0), which the binding UI of the controllers
     * may not offer. Without an explicit binding, the battery is subscribed with a wildcard endpoint on the
     * node of the temperature sensor: the sensor reports it from whichever endpoint has the PowerSource. */
    const Binding::TableEntry *battery_node = battery_binding ? battery_binding : temperature_binding;
    if (battery_node) {
        chip::EndpointId battery_endpoint = battery_binding ? battery_binding->remote : chip::kInvalidEndpointId;
        app_driver_add_sensor_path(nodes, &node_count, battery_node, PowerSource::Id,
                                   PowerSource::Attributes::BatPercentRemaining::Id, battery_endpoint);
        app_driver_add_sensor_path(nodes, &node_count, battery_node, PowerSource::Id,
                                   PowerSource::Attributes::BatChargeLevel::Id, battery_endpoint);
    }

    for (size_t i = 0; i < SENSOR_MAX_NODES; i++) {
        if (i < node_count) {
            sensor_subscriptions[i].start(nodes[i].peer, nodes[i].paths, nodes[i].path_count);
        } else {
            sensor_subscriptions[i].stop();
        }
    }

    if (!temperature_binding && remote_temperature_valid) {
        ESP_LOGW(TAG, "No temperature sensor bound");
        app_driver_remote_temperature_set(false, 0);
    }
    if (!humidity_binding && remote_humidity_valid) {
        app_driver_remote_humidity_set(false, 0);
    }
}

void app_driver_bindings_changed()
{
    if (chip::DeviceLayer::PlatformMgr().ScheduleWork(app_driver_update_subscriptions, 0) != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "Failed to schedule the subscription update");
    }
}

static void app_driver_bindings_loaded_timer_cb(chip::System::Layer *system_layer, void *app_state)
{
    app_driver_update_subscriptions(0);
}

void app_driver_bindings_loaded()
{
    /* esp-matter loads the binding table on kDnssdInitialized, in work scheduled from its own event handler.
     * The event handlers are called in the reverse order of their registration, so the application handler runs
     * first and work scheduled from it would still see an empty binding table. Subscribe a moment later. */
    if (chip::DeviceLayer::SystemLayer().StartTimer(chip::System::Clock::Milliseconds32(1000),
                                                    app_driver_bindings_loaded_timer_cb, nullptr) != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "Failed to start the binding table timer");
    }
}

/* The sensor sends a report at least every max interval. Allow two missed reports before the
 * temperature is treated as unknown, but never less than CONFIG_THERMOSTAT_SENSOR_TIMEOUT. */
static int64_t app_driver_sensor_timeout_us()
{
    int64_t timeout_sec = CONFIG_THERMOSTAT_SENSOR_TIMEOUT;
    int64_t report_timeout_sec = 2 * (int64_t)remote_temperature_max_interval + 60;
    if (report_timeout_sec > timeout_sec) {
        timeout_sec = report_timeout_sec;
    }
    return timeout_sec * 1000 * 1000;
}

/* Periodic check: retries the subscriptions which could not be sent and invalidates the temperature when the
 * sensor stopped reporting, so the relays are never left on with an old reading. */
static void app_driver_sensor_check_timer_cb(chip::System::Layer *system_layer, void *app_state)
{
    static uint32_t elapsed_sec = 0;
    elapsed_sec += SENSOR_CHECK_PERIOD_SEC;
    if (elapsed_sec >= SUBSCRIBE_RETRY_PERIOD_SEC) {
        elapsed_sec = 0;
        /* Picks up the binding table if it was not loaded yet, does nothing when already subscribed */
        app_driver_update_subscriptions(0);
        for (size_t i = 0; i < SENSOR_MAX_NODES; i++) {
            sensor_subscriptions[i].retry();
        }
    }

    int64_t timeout_us = app_driver_sensor_timeout_us();
    if (remote_temperature_valid && esp_timer_get_time() - remote_temperature_last_seen_us > timeout_us) {
        ESP_LOGW(TAG, "No report from the temperature sensor for %d s", (int)(timeout_us / 1000000));
        app_driver_remote_temperature_set(false, 0);
    }

    if (system_layer->StartTimer(chip::System::Clock::Seconds32(SENSOR_CHECK_PERIOD_SEC),
                                 app_driver_sensor_check_timer_cb, nullptr) != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "Failed to restart the sensor check timer");
    }
}

/* Min/MaxHeat/CoolSetpointLimit are non-volatile: after an update of the firmware, the values stored by the
 * older firmware are restored from NVS instead of the ones in menuconfig. Write the current limits back,
 * otherwise the Thermostat server keeps rejecting setpoints between the old and the new limit. */
static void app_driver_restore_limit(uint32_t attribute_id, int16_t limit)
{
    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    if (attribute::get_val(thermostat_endpoint_id, Thermostat::Id, attribute_id, &val) != ESP_OK ||
            val.val.i16 == limit) {
        return;
    }
    ESP_LOGW(TAG, "Setpoint limit 0x%04" PRIx32 " stored as %d, using %d from menuconfig", attribute_id, val.val.i16,
             limit);
    val = esp_matter_int16(limit);
    app_driver_update_attribute(Thermostat::Id, attribute_id, &val, "boot: limit from menuconfig");
}

/* The setpoints are restored from NVS. If they were set with older (wider) limits, move them into the
 * current limits, otherwise the controllers show a value which cannot be set. */
static void app_driver_clamp_setpoint(uint32_t attribute_id, int16_t min, int16_t max)
{
    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    if (attribute::get_val(thermostat_endpoint_id, Thermostat::Id, attribute_id, &val) != ESP_OK) {
        return;
    }
    int16_t setpoint = val.val.i16;
    int16_t clamped = setpoint < min ? min : (setpoint > max ? max : setpoint);
    if (clamped != setpoint) {
        ESP_LOGW(TAG, "Stored setpoint %d.%02d C is out of the limits, using %d.%02d C", setpoint / 100,
                 abs(setpoint % 100), clamped / 100, abs(clamped % 100));
        val = esp_matter_int16(clamped);
        app_driver_update_attribute(Thermostat::Id, attribute_id, &val, "boot: setpoint out of limits");
    }
}

static void app_driver_thermostat_start_work(intptr_t arg)
{
#if CONFIG_THERMOSTAT_ENABLE_AUTO
    /* MinSetpointDeadBand is non-volatile: apply the value from menuconfig over the one stored by an older
     * firmware, before the setpoints are checked against the limits. */
    esp_matter_attr_val_t dead_band = esp_matter_invalid(NULL);
    if (attribute::get_val(thermostat_endpoint_id, Thermostat::Id, Thermostat::Attributes::MinSetpointDeadBand::Id,
                           &dead_band) == ESP_OK && dead_band.val.i8 != THERMOSTAT_MIN_SETPOINT_DEAD_BAND) {
        ESP_LOGI(TAG, "Auto dead band: %d.%d C", THERMOSTAT_MIN_SETPOINT_DEAD_BAND / 10,
                 THERMOSTAT_MIN_SETPOINT_DEAD_BAND % 10);
        dead_band = esp_matter_int8(THERMOSTAT_MIN_SETPOINT_DEAD_BAND);
        app_driver_update_attribute(Thermostat::Id, Thermostat::Attributes::MinSetpointDeadBand::Id, &dead_band,
                                    "boot: dead band from menuconfig");
    }
#endif
    app_driver_restore_limit(Thermostat::Attributes::MinHeatSetpointLimit::Id, THERMOSTAT_ABS_MIN_HEAT_SETPOINT);
    app_driver_restore_limit(Thermostat::Attributes::MaxHeatSetpointLimit::Id, THERMOSTAT_ABS_MAX_HEAT_SETPOINT);
#if CONFIG_THERMOSTAT_ENABLE_COOLING
    app_driver_restore_limit(Thermostat::Attributes::MinCoolSetpointLimit::Id, THERMOSTAT_ABS_MIN_COOL_SETPOINT);
    app_driver_restore_limit(Thermostat::Attributes::MaxCoolSetpointLimit::Id, THERMOSTAT_ABS_MAX_COOL_SETPOINT);
#endif

#if CONFIG_THERMOSTAT_ENABLE_COOLING
    /* Cooling first: in Auto, the heating setpoint is then checked against the final cooling setpoint */
    app_driver_clamp_setpoint(Thermostat::Attributes::OccupiedCoolingSetpoint::Id, THERMOSTAT_ABS_MIN_COOL_SETPOINT,
                              THERMOSTAT_ABS_MAX_COOL_SETPOINT);
#endif
    app_driver_clamp_setpoint(Thermostat::Attributes::OccupiedHeatingSetpoint::Id, THERMOSTAT_ABS_MIN_HEAT_SETPOINT,
                              THERMOSTAT_ABS_MAX_HEAT_SETPOINT);

    /* Apply the mode and setpoints restored from NVS. The relays stay off until the sensor reports.
     * The subscriptions are started after kDnssdInitialized, when the binding table has been loaded. */
    app_driver_thermostat_update(0);

    /* The mirrored battery belongs to the thermostat endpoint */
    static chip::EndpointId battery_endpoint_list[1];
    battery_endpoint_list[0] = thermostat_endpoint_id;
    if (PowerSourceServer::Instance().SetEndpointList(thermostat_endpoint_id,
            chip::Span<chip::EndpointId>(battery_endpoint_list)) != CHIP_NO_ERROR) {
        ESP_LOGW(TAG, "Failed to set the PowerSource EndpointList");
    }

    if (chip::DeviceLayer::SystemLayer().StartTimer(chip::System::Clock::Seconds32(SENSOR_CHECK_PERIOD_SEC),
                                                    app_driver_sensor_check_timer_cb, nullptr) != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "Failed to start the sensor check timer");
    }
}

void app_driver_thermostat_start(uint16_t endpoint_id)
{
    thermostat_endpoint_id = endpoint_id;
    if (chip::DeviceLayer::PlatformMgr().ScheduleWork(app_driver_thermostat_start_work, 0) != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "Failed to start the thermostat");
    }
}

static void app_driver_factory_reset_indication(intptr_t arg)
{
    factory_reset_pending = true;
    app_driver_update_status_led();
}

/* Called when the button has been held for CONFIG_BUTTON_LONG_PRESS_TIME_MS. app_reset (examples/common)
 * does the factory reset when the button is released, the LED shows that it can be released now. */
static void app_driver_button_long_press_cb(void *arg, void *data)
{
    ESP_LOGW(TAG, "Release the button for a factory reset");
    if (chip::DeviceLayer::PlatformMgr().ScheduleWork(app_driver_factory_reset_indication, 0) != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "Failed to schedule the factory reset indication");
    }
}

app_driver_handle_t app_driver_thermostat_init()
{
    /* Initialize relays, both are off */
    app_driver_relay_init(CONFIG_THERMOSTAT_HEAT_RELAY_GPIO);
#if CONFIG_THERMOSTAT_ENABLE_COOLING
    app_driver_relay_init(CONFIG_THERMOSTAT_COOL_RELAY_GPIO);
#endif

    /* Initialize the WS2812 status LED */
    app_driver_status_led_init();

    /* Initialize button. Held for 5 s (CONFIG_BUTTON_LONG_PRESS_TIME_MS) and released: factory reset */
    button_handle_t btns[BSP_BUTTON_NUM];
    ESP_ERROR_CHECK(bsp_iot_button_create(btns, NULL, BSP_BUTTON_NUM));
    ESP_ERROR_CHECK(iot_button_register_cb(btns[0], BUTTON_LONG_PRESS_START, NULL, app_driver_button_long_press_cb,
                                           NULL));

    return (app_driver_handle_t)btns[0];
}
