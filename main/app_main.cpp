/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <inttypes.h>
#include <string.h>

#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>
#include <esp_matter_providers.h>

#include <common_macros.h>
#include <app_priv.h>
#include <app_reset.h>
#include <app-common/zap-generated/callback.h>
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif
#if CONFIG_DYNAMIC_PASSCODE_COMMISSIONABLE_DATA_PROVIDER
#include <custom_provider/dynamic_commissionable_data_provider.h>
#endif

static const char *TAG = "app_main";
uint16_t thermostat_endpoint_id = 0;

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

#if CONFIG_DYNAMIC_PASSCODE_COMMISSIONABLE_DATA_PROVIDER
dynamic_commissionable_data_provider g_dynamic_passcode_provider;
#endif

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address Changed");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;

    case chip::DeviceLayer::DeviceEventType::kDnssdInitialized:
        /* esp-matter loads the binding table from NVS (Binding Manager init) on this event */
        app_driver_bindings_loaded();
        break;

    case chip::DeviceLayer::DeviceEventType::kBindingsChangedViaCluster:
        ESP_LOGI(TAG, "Binding entry changed");
        /* The remote temperature/humidity sensor is selected through the binding table. */
        app_driver_bindings_changed();
        break;

    default:
        break;
    }
}

// This callback is invoked when clients interact with the Identify Cluster.
// In the callback implementation, an endpoint can identify itself. (e.g., by flashing an LED or light).
static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    /* Called on the Matter thread by the Identify server */
    switch (type) {
    case identification::START:
        app_driver_identify_start();
        break;
    case identification::STOP:
        app_driver_identify_stop();
        break;
    case identification::EFFECT:
        app_driver_identify_effect(effect_id);
        break;
    }
    return ESP_OK;
}

int32_t app_trace_value(const esp_matter_attr_val_t *val)
{
    switch (val->type) {
    case ESP_MATTER_VAL_TYPE_INT8:
        return val->val.i8;
    case ESP_MATTER_VAL_TYPE_INT16:
        return val->val.i16;
    case ESP_MATTER_VAL_TYPE_NULLABLE_INT16:
        return val->val.i16 == INT16_MIN ? INT32_MIN : val->val.i16;
    case ESP_MATTER_VAL_TYPE_UINT8:
    case ESP_MATTER_VAL_TYPE_ENUM8:
    case ESP_MATTER_VAL_TYPE_BITMAP8:
        return val->val.u8;
    case ESP_MATTER_VAL_TYPE_NULLABLE_UINT8:
        return val->val.u8 == UINT8_MAX ? INT32_MIN : val->val.u8;
    case ESP_MATTER_VAL_TYPE_UINT16:
    case ESP_MATTER_VAL_TYPE_BITMAP16:
        return val->val.u16;
    default:
        return INT32_MIN;
    }
}

// This callback is called for every attribute update. The callback implementation shall
// handle the desired attributes and return an appropriate error code. If the attribute
// is not of your interest, please do not return an error code and strictly return ESP_OK.
static esp_err_t app_attribute_update_cb(callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    if (endpoint_id == thermostat_endpoint_id && cluster_id == Thermostat::Id &&
            attribute_id != Thermostat::Attributes::LocalTemperature::Id &&
            attribute_id != Thermostat::Attributes::ThermostatRunningState::Id &&
            attribute_id != Thermostat::Attributes::ThermostatRunningMode::Id) {
        /* Writes by a controller come with the chip[EM] "WriteRequest" line of that controller just before */
        APP_TRACE(TAG, "%s Thermostat attr 0x%04" PRIx32 " = %" PRId32,
                  type == PRE_UPDATE ? "PRE_UPDATE " : "POST_UPDATE", attribute_id, app_trace_value(val));
    }

    if (type == POST_UPDATE && endpoint_id == thermostat_endpoint_id && cluster_id == Thermostat::Id) {
        /* A controller (e.g. Apple Home) changed the mode or one of the setpoints. The new values are
         * read back from the data model in the driver, after the Thermostat server has finished the
         * write (including the dead band adjustment of the other setpoint in Auto). */
        if (attribute_id == Thermostat::Attributes::SystemMode::Id) {
            APP_LOG("[MODE]     %s (EP%u SystemMode 0x0201/0x001C = %u)", app_system_mode_name(val->val.u8),
                    endpoint_id, val->val.u8);
            app_driver_thermostat_schedule_update();
            app_ir_notify(false);   /* The IR TX task reads the attributes and sends itself */
        } else if (attribute_id == Thermostat::Attributes::OccupiedHeatingSetpoint::Id ||
                   attribute_id == Thermostat::Attributes::OccupiedCoolingSetpoint::Id) {
            bool heating = attribute_id == Thermostat::Attributes::OccupiedHeatingSetpoint::Id;
            APP_LOG("[SETPOINT] %s %d.%02d C (EP%u %s 0x0201/0x%04" PRIx32 " = %d)", heating ? "heating" : "cooling",
                    val->val.i16 / 100, abs(val->val.i16 % 100), endpoint_id,
                    heating ? "OccupiedHeatingSetpoint" : "OccupiedCoolingSetpoint", attribute_id, val->val.i16);
            app_driver_thermostat_schedule_update();
            app_ir_notify(false);
        } else if (attribute_id == Thermostat::Attributes::LocalTemperatureCalibration::Id) {
            app_driver_calibration_changed();
        }
    }

    if (type == POST_UPDATE && endpoint_id == thermostat_endpoint_id && cluster_id == OnOff::Id &&
            attribute_id == OnOff::Attributes::OnOff::Id) {
        APP_LOG("[POWER]    %s (EP%u OnOff 0x0006/0x0000 = %u)", val->val.b ? "ON" : "OFF", endpoint_id, val->val.b);
        app_driver_thermostat_schedule_update();
        app_ir_notify(false);
    }

    return ESP_OK;
}

/* Create the attributes which are optional in esp-matter, but needed by the controllers to show a
 * proper thermostat (setpoint limits, running state and the remote sensing flag). */
static void app_thermostat_add_optional_attributes(cluster_t *cluster)
{
    namespace thermostat_attr = cluster::thermostat::attribute;

    thermostat_attr::create_abs_min_heat_setpoint_limit(cluster, THERMOSTAT_ABS_MIN_HEAT_SETPOINT);
    thermostat_attr::create_abs_max_heat_setpoint_limit(cluster, THERMOSTAT_ABS_MAX_HEAT_SETPOINT);
    thermostat_attr::create_min_heat_setpoint_limit(cluster, THERMOSTAT_ABS_MIN_HEAT_SETPOINT);
    thermostat_attr::create_max_heat_setpoint_limit(cluster, THERMOSTAT_ABS_MAX_HEAT_SETPOINT);
    thermostat_attr::create_abs_min_cool_setpoint_limit(cluster, THERMOSTAT_ABS_MIN_COOL_SETPOINT);
    thermostat_attr::create_abs_max_cool_setpoint_limit(cluster, THERMOSTAT_ABS_MAX_COOL_SETPOINT);
    thermostat_attr::create_min_cool_setpoint_limit(cluster, THERMOSTAT_ABS_MIN_COOL_SETPOINT);
    thermostat_attr::create_max_cool_setpoint_limit(cluster, THERMOSTAT_ABS_MAX_COOL_SETPOINT);
#if CONFIG_THERMOSTAT_ENABLE_AUTO
    /* ThermostatRunningMode is mandatory with the AutoMode feature. */
    thermostat_attr::create_thermostat_running_mode(cluster,
        chip::to_underlying(Thermostat::ThermostatRunningModeEnum::kOff));
#endif
    /* Heat/Cool/Fan, derived from the mode of the air conditioner (app_driver.cpp). */
    thermostat_attr::create_thermostat_running_state(cluster, 0);
    /* LocalTemperature is not measured on this device, it comes from the bound remote sensor. */
    thermostat_attr::create_remote_sensing(cluster,
        chip::to_underlying(Thermostat::RemoteSensingBitmap::kLocalTemperature));
    /* Offset added to the sensor temperature (0.1 degC). The default from menuconfig is used on the very first
     * boot only, afterwards the value is restored from NVS. */
    attribute_t *calibration = thermostat_attr::create_local_temperature_calibration(cluster,
        CONFIG_THERMOSTAT_TEMPERATURE_CALIBRATION);
    if (calibration) {
        esp_matter::attribute::add_bounds(calibration, esp_matter_int8(THERMOSTAT_MIN_CALIBRATION),
                                          esp_matter_int8(THERMOSTAT_MAX_CALIBRATION));
    }
}

extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    /* Initialize the ESP NVS layer */
    nvs_flash_init();

    /* Initialize driver */
    app_driver_handle_t thermostat_handle = app_driver_thermostat_init();
    app_reset_button_register(thermostat_handle);

    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    /* SerialNumber and ProductLabel are optional BasicInformation attributes, which are not created by default.
     * Without them the controllers do not show the serial number of app_device_info.cpp. The values are read
     * from the device instance info provider. */
    cluster_t *basic_info_cluster = cluster::get(endpoint::get(node, chip::kRootEndpointId), BasicInformation::Id);
    ABORT_APP_ON_FAILURE(basic_info_cluster != nullptr, ESP_LOGE(TAG, "Failed to get basic information cluster"));
    cluster::basic_information::attribute::create_serial_number(basic_info_cluster, NULL, 0);
    cluster::basic_information::attribute::create_product_label(basic_info_cluster, NULL, 0);

#if CONFIG_AC_DEVICE_ROOM_AIR_CONDITIONER
    /* Room Air Conditioner (0x0072): Identify, OnOff (with the DeadFrontBehavior feature) and Thermostat (Cooling is
     * always added by esp-matter). FanControl is added below. */
    room_air_conditioner::config_t thermostat_config;
    /* Off on the very first boot, OnOff is non-volatile afterwards */
    thermostat_config.on_off.on_off = false;
#else
    /* Thermostat (0x0301): Identify, Groups, ScenesManagement and Thermostat, no OnOff: SystemMode Off switches the
     * unit off. FanControl (optional on a Thermostat) is added below. */
    endpoint::thermostat::config_t thermostat_config;
#endif
    /* Unknown until the first report from the remote sensor. */
    thermostat_config.thermostat.local_temperature = nullable<int16_t>();
    /* Off on the very first boot. SystemMode and the setpoints are non-volatile, so after a reboot the
     * values selected by the user are restored from NVS. */
    thermostat_config.thermostat.system_mode = chip::to_underlying(Thermostat::SystemModeEnum::kOff);
    thermostat_config.thermostat.feature_flags = cluster::thermostat::feature::heating::get_id();
    /* Heating and cooling setpoints are two independent attributes. Heat mode only uses the heating
     * setpoint, Cool mode only uses the cooling setpoint and Auto uses both of them as a range. */
    thermostat_config.thermostat.features.heating.occupied_heating_setpoint = THERMOSTAT_DEFAULT_HEAT_SETPOINT;
    thermostat_config.thermostat.control_sequence_of_operation =
        chip::to_underlying(Thermostat::ControlSequenceOfOperationEnum::kCoolingAndHeating);
    thermostat_config.thermostat.feature_flags |= cluster::thermostat::feature::cooling::get_id();
    thermostat_config.thermostat.features.cooling.occupied_cooling_setpoint = THERMOSTAT_DEFAULT_COOL_SETPOINT;
#if CONFIG_THERMOSTAT_ENABLE_AUTO
    thermostat_config.thermostat.feature_flags |= cluster::thermostat::feature::auto_mode::get_id();
    thermostat_config.thermostat.features.auto_mode.min_setpoint_dead_band = THERMOSTAT_MIN_SETPOINT_DEAD_BAND;
#endif

#if CONFIG_AC_DEVICE_ROOM_AIR_CONDITIONER
    endpoint_t *endpoint = room_air_conditioner::create(node, &thermostat_config, ENDPOINT_FLAG_NONE, thermostat_handle);
#else
    endpoint_t *endpoint = endpoint::thermostat::create(node, &thermostat_config, ENDPOINT_FLAG_NONE, thermostat_handle);
#endif
    ABORT_APP_ON_FAILURE(endpoint != nullptr, ESP_LOGE(TAG, "Failed to create the air conditioner endpoint"));

    /* Fan of the air conditioner: Off, Low, Medium, High and Auto (FanModeSequence OffLowMedHighAuto), three speeds
     * (MultiSpeed, SpeedMax 3) and PercentSetting. FanControl is code-driven, the application gets the changes
     * through the delegate (app_driver.cpp), not through app_attribute_update_cb(). */
    cluster::fan_control::config_t fan_config;
    fan_config.fan_mode = chip::to_underlying(FanControl::FanModeEnum::kAuto);
    fan_config.fan_mode_sequence = chip::to_underlying(FanControl::FanModeSequenceEnum::kOffLowMedHighAuto);
    fan_config.delegate = app_driver_fan_control_delegate();
    cluster_t *fan_cluster = cluster::fan_control::create(endpoint, &fan_config, CLUSTER_FLAG_SERVER);
    ABORT_APP_ON_FAILURE(fan_cluster != nullptr, ESP_LOGE(TAG, "Failed to create fan control cluster"));
    cluster::fan_control::feature::fan_auto::add(fan_cluster);
    cluster::fan_control::feature::multi_speed::config_t fan_speed_config;
    fan_speed_config.speed_max = 3;
    cluster::fan_control::feature::multi_speed::add(fan_cluster, &fan_speed_config);

    /* TriggerEffect is optional in the Identify cluster, the status LED supports the effects */
    cluster_t *identify_cluster = cluster::get(endpoint, Identify::Id);
    ABORT_APP_ON_FAILURE(identify_cluster != nullptr, ESP_LOGE(TAG, "Failed to get identify cluster"));
    cluster::identify::command::create_trigger_effect(identify_cluster);

    cluster_t *thermostat_cluster = cluster::get(endpoint, Thermostat::Id);
    ABORT_APP_ON_FAILURE(thermostat_cluster != nullptr, ESP_LOGE(TAG, "Failed to get thermostat cluster"));
    app_thermostat_add_optional_attributes(thermostat_cluster);

    /* esp-matter main (connectedhomeip v1.7) does not integrate the code-driven Thermostat server yet: its init and
     * PreAttributeChanged callbacks are empty stubs, so there is no dead band adjustment and no Presets. The setpoint
     * limits are still enforced by the attribute bounds of esp-matter. */

    /* TemperatureDisplayMode is exposed to Apple Home as the TemperatureDisplayUnits characteristic. */
    cluster::thermostat_user_interface_configuration::config_t ui_config;
    ui_config.temperature_display_mode =
        chip::to_underlying(ThermostatUserInterfaceConfiguration::TemperatureDisplayModeEnum::kCelsius);
    /* In esp-matter TemperatureDisplayMode and KeypadLockout are not non-volatile: the unit selected in a controller
     * (e.g. Fahrenheit) would go back to Celsius on every reboot. They are created here as non-volatile first, the
     * cluster create below then keeps them ("already exists" warnings) and adds the rest of the cluster. */
    cluster_t *ui_cluster = cluster::create(endpoint, ThermostatUserInterfaceConfiguration::Id, CLUSTER_FLAG_SERVER);
    ABORT_APP_ON_FAILURE(ui_cluster != nullptr, ESP_LOGE(TAG, "Failed to create thermostat UI configuration cluster"));
    esp_matter::attribute::create(ui_cluster, ThermostatUserInterfaceConfiguration::Attributes::TemperatureDisplayMode::Id,
                                  ATTRIBUTE_FLAG_WRITABLE | ATTRIBUTE_FLAG_NONVOLATILE,
                                  esp_matter_enum8(ui_config.temperature_display_mode));
    esp_matter::attribute::create(ui_cluster, ThermostatUserInterfaceConfiguration::Attributes::KeypadLockout::Id,
                                  ATTRIBUTE_FLAG_WRITABLE | ATTRIBUTE_FLAG_NONVOLATILE,
                                  esp_matter_enum8(ui_config.keypad_lockout));
    cluster::thermostat_user_interface_configuration::create(endpoint, &ui_config, CLUSTER_FLAG_SERVER);

    /* Client clusters. A controller binds them to the remote sensor, the thermostat then subscribes to
     * MeasuredValue of the bound TemperatureMeasurement and RelativeHumidityMeasurement servers and to the
     * battery in the PowerSource server of the sensor. */
    cluster::temperature_measurement::create(endpoint, nullptr, CLUSTER_FLAG_CLIENT);

    /* Everything is on the thermostat endpoint, like on commercial Matter thermostats (e.g. Meross MTS300 has
     * Thermostat and RelativeHumidityMeasurement on its only endpoint). Apple Home then shows the humidity
     * inside the thermostat accessory (iOS 18.5 or later). A separate Humidity Sensor endpoint would be shown
     * as a separate accessory tile instead. */
    cluster::relative_humidity_measurement::config_t humidity_config;
    humidity_config.min_measured_value = nullable<uint16_t>(0);
    humidity_config.max_measured_value = nullable<uint16_t>(10000);
    cluster::relative_humidity_measurement::create(endpoint, &humidity_config, CLUSTER_FLAG_SERVER | CLUSTER_FLAG_CLIENT);

    /* Battery of the bound sensor, shown as the battery of the thermostat accessory (like a battery powered
     * Matter thermostat, e.g. Eve Thermo). */
    cluster::power_source::config_t battery_config;
    battery_config.status = chip::to_underlying(PowerSource::PowerSourceStatusEnum::kActive);
    strncpy(battery_config.description, "Sensor battery", sizeof(battery_config.description) - 1);
    battery_config.feature_flags = cluster::power_source::feature::battery::get_id();
    battery_config.features.battery.bat_charge_level = chip::to_underlying(PowerSource::BatChargeLevelEnum::kOk);
    battery_config.features.battery.bat_replaceability =
        chip::to_underlying(PowerSource::BatReplaceabilityEnum::kUserReplaceable);
    cluster_t *battery_cluster = cluster::power_source::create(endpoint, &battery_config,
                                                               CLUSTER_FLAG_SERVER | CLUSTER_FLAG_CLIENT);
    ABORT_APP_ON_FAILURE(battery_cluster != nullptr, ESP_LOGE(TAG, "Failed to create battery power source"));
    /* 0.5 % units. Never null: Apple Home reads it as the Battery Level of the accessory, and a read of a null value
     * failed on the home hub (HMErrorDomain 4), which marked the whole accessory "No Response" after every command.
     * 100 % until the first report of the sensor, then the last reported value, kept in NVS over reboots and while
     * the sensor is not reporting (not bound, out of range). */
    attribute_t *bat_percent = esp_matter::attribute::create(battery_cluster,
        PowerSource::Attributes::BatPercentRemaining::Id, ATTRIBUTE_FLAG_NULLABLE | ATTRIBUTE_FLAG_NONVOLATILE,
        esp_matter_nullable_uint8(nullable<uint8_t>(200)));
    ABORT_APP_ON_FAILURE(bat_percent != nullptr, ESP_LOGE(TAG, "Failed to create the battery percentage"));
    esp_matter::attribute::add_bounds(bat_percent, esp_matter_nullable_uint8(nullable<uint8_t>(0)),
                                      esp_matter_nullable_uint8(nullable<uint8_t>(200)));

    thermostat_endpoint_id = endpoint::get_id(endpoint);
    ESP_LOGI(TAG, "Thermostat created with endpoint_id %d", thermostat_endpoint_id);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

#if CONFIG_DYNAMIC_PASSCODE_COMMISSIONABLE_DATA_PROVIDER
    /* This should be called before esp_matter::start() */
    esp_matter::set_custom_commissionable_data_provider(&g_dynamic_passcode_provider);
#endif

    /* Vendor/product name, serial number and hardware version from menuconfig */
    app_device_info_provider_init();

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

    /* Starting driver: subscribe to the bound sensor and apply the restored mode and setpoints */
    app_driver_thermostat_start(thermostat_endpoint_id);

#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
#if CONFIG_ENABLE_WIFI_STATION
    esp_matter::console::wifi_register_commands();
#endif
    esp_matter::console::factoryreset_register_commands();
    esp_matter::console::init();
#endif
}
