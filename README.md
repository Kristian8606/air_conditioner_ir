# Matter Air Conditioner with IR (ESP32-C6, Matter over Thread)

Work in progress, see [ROADMAP.md](ROADMAP.md). Based on
[thermostat_remote_sensor](https://github.com/Kristian8606/thermostat_remote_sensor) v2.3.0, with its own
identity: product ID 0x8001 (the thermostat is 0x8000, so the Home Assistant Matter Server never offers one
device the firmware of the other), product name "Air Conditioner", serial number prefix "ACIR-".

Since 0.2.0 endpoint 1 is a Room Air Conditioner (0x0072) device:

-   OnOff (DeadFrontBehavior): power of the unit. With OnOff off the unit is off whatever SystemMode is.
-   Thermostat: SystemMode and the heating/cooling setpoints, as in the thermostat.
-   FanControl: FanMode Off/Low/Medium/High/Auto (FanModeSequence OffLowMedHighAuto), three speeds
    (MultiSpeed, SpeedMax 3) and PercentSetting. FanControl is a code-driven cluster in esp-matter: the values
    are kept by the FanControl server (FanMode also in NVS) and the application gets the changes through
    `FanControl::Delegate::OnFanDriveStateChanged()` (log `[FAN]`), not through the attribute callback.
-   The remote sensor (temperature, humidity, battery), Identify, calibration and rounding are unchanged.

The relays still follow the thermostat logic until the IR part replaces them. The rest of the description below
is still the one of the thermostat.

# Thermostat with Remote Sensor

This example creates a Thermostat device using the data model.

The thermostat does not have its own temperature sensor. A remote Matter temperature sensor
(and optionally a humidity sensor) is bound to it, the thermostat subscribes to the sensor and uses
the reported temperature to control a heating relay and a cooling relay.

See the [docs](https://docs.espressif.com/projects/esp-matter/en/latest/esp32/developing.html) for more information about building and flashing the firmware.

## 1. Features

-   System modes: Off, Heat, Cool and Auto (Cool and Auto can be disabled in menuconfig).
-   Separate heating and cooling setpoints (`OccupiedHeatingSetpoint` and `OccupiedCoolingSetpoint`).
    - Heat uses only the heating setpoint, Cool uses only the cooling setpoint.
    - Auto uses both of them as a range. The cooling setpoint is always kept at least
      `MinSetpointDeadBand` (menuconfig, default 0) above the heating setpoint.
-   Heating relay and cooling relay with hysteresis. The two relays are never on at the same time.
-   WS2812 status LED.
-   Temperature and humidity from a bound remote sensor, from the same sensor or from two different sensors.
-   Mode and setpoints are stored in NVS and restored after a reboot.

### 1.1 Data model

Endpoint 1 is a Thermostat (0x0301) device with:

| Cluster                               | Role            | Purpose                                                           |
|---------------------------------------|-----------------|-------------------------------------------------------------------|
| Thermostat                            | Server          | Heat, Cool and AutoMode features                                  |
| Thermostat User Interface Configuration | Server        | Temperature display unit (Celsius)                                |
| Relative Humidity Measurement         | Server + Client | Client: bound to the sensor. Server: humidity inside the thermostat accessory |
| Temperature Measurement               | Client          | Bound to the remote temperature sensor                            |
| Power Source                          | Server + Client | Client: bound to the battery of the sensor. Server: shows it      |
| Binding                               | Server          | Holds the binding to the remote sensor(s)                         |
| Identify, Groups, Scenes              | Server          | Mandatory/optional clusters of the Thermostat device type         |

There is no separate Humidity Sensor endpoint: like commercial Matter thermostats (e.g. Meross MTS300 has
Thermostat and RelativeHumidityMeasurement on one endpoint), the humidity and the battery of the bound sensor
are mirrored on the thermostat endpoint, so Apple Home shows them inside the thermostat accessory (humidity
needs iOS 18.5 or later and the accessory has to be added again after the change).

### 1.2 Apple Home (HAP) mapping

The HAP Thermostat service requires the `CurrentHeatingCoolingState`, `TargetHeatingCoolingState`,
`CurrentTemperature`, `TargetTemperature` and `TemperatureDisplayUnits` characteristics.
`HeatingThresholdTemperature`, `CoolingThresholdTemperature` and `CurrentRelativeHumidity` are optional.
They come from these Matter attributes:

| HAP characteristic            | Matter attribute                                                  |
|-------------------------------|-------------------------------------------------------------------|
| CurrentTemperature            | Thermostat `LocalTemperature` (temperature of the bound sensor)    |
| TargetHeatingCoolingState     | Thermostat `SystemMode`                                           |
| CurrentHeatingCoolingState    | Thermostat `ThermostatRunningState` / `ThermostatRunningMode`      |
| TargetTemperature             | `OccupiedHeatingSetpoint` (Heat) or `OccupiedCoolingSetpoint` (Cool) |
| HeatingThresholdTemperature   | `OccupiedHeatingSetpoint` (Auto)                                  |
| CoolingThresholdTemperature   | `OccupiedCoolingSetpoint` (Auto)                                  |
| TemperatureDisplayUnits       | Thermostat UI Configuration `TemperatureDisplayMode`              |
| CurrentRelativeHumidity       | Relative Humidity Measurement `MeasuredValue` on the thermostat endpoint (iOS 18.5+) |

Notes:

-   The setpoint limit attributes (`AbsMin/AbsMax/Min/MaxHeatSetpointLimit` and the cool ones) are
    optional in esp-matter. They are created in `app_main.cpp`, otherwise the controller does not know the
    valid range of the setpoints.
-   The default function list of the Thermostat cluster does not register the AttributeChanged callback,
    so the Thermostat server never moves the other setpoint to keep the dead band. In Auto, this lets the
    heating and the cooling setpoint get closer than `MinSetpointDeadBand`. The example registers
    `MatterThermostatClusterServerAttributeChangedCallback`.
-   The setpoint range is set in menuconfig (`THERMOSTAT_MIN_SETPOINT` / `THERMOSTAT_MAX_SETPOINT`, default
    16..30 °C). The Auto dead band (`THERMOSTAT_DEAD_BAND`) is 0 by default, so heating and cooling both use
    the full range. With a dead band the heating range ends that much below the maximum (the Thermostat server
    rejects a heating setpoint it cannot keep apart from the cooling one, even in Heat mode), but Apple Home
    shows one slider range for all modes and jumps back when such a value is rejected. Heating and cooling
    never run together anyway: separate setpoints, hysteresis and a relay interlock. Stored setpoints outside
    of the range are moved into it on boot.
-   `MinSetpointDeadBand` is in 0.1 °C units (20 = 2.0 °C), while the setpoints are in 0.01 °C units.
-   `LocalTemperature` is null until the first report from the sensor, and again after the sensor timeout.

## 2. Hardware

Default pins for ESP32-C6-DevKitC-1/M-1 (all of them can be changed in `menuconfig`,
`Thermostat Configuration`):

| Function        | GPIO | Kconfig                              |
|-----------------|------|--------------------------------------|
| Heating relay   | 2    | `THERMOSTAT_HEAT_RELAY_GPIO`          |
| Cooling relay   | 3    | `THERMOSTAT_COOL_RELAY_GPIO`          |
| WS2812 LED      | 8    | `THERMOSTAT_STATUS_LED_GPIO`          |
| Boot button     | 9    | Factory reset (long press)            |

Relays are active HIGH by default, disable `THERMOSTAT_RELAY_ACTIVE_HIGH` for active LOW relay modules.

Status LED:

| Color  | Meaning                                                        |
|--------|----------------------------------------------------------------|
| Orange | Heating relay is on                                            |
| Blue   | Cooling relay is on                                            |
| White  | Both relays are off (idle, system mode Off or no sensor data)  |
| Purple | Factory reset pending (release the button)                     |
| Blinking | Identify (white), TriggerEffect Okay (green), ChannelChange (orange) |

Identify: the Identify command (e.g. "Identify" in Home Assistant, or Apple Home while adding the
accessory) blinks the LED white every 0.5 s until `IdentifyTime` reaches 0. The TriggerEffect command
runs a short pattern: Blink 1 blink, Breathe 15 blinks, Okay 2 green blinks, ChannelChange orange for 8 s,
FinishEffect ends after the current blink, StopEffect stops at once. The relays keep working meanwhile,
and the factory reset indication has priority over the blinking.

Factory reset: hold the BOOT button (GPIO 9) for 5 s (`CONFIG_BUTTON_LONG_PRESS_TIME_MS`), the LED turns
purple, release it. All fabrics, the Thread network, the bindings and the setpoints are erased and the device
is ready to be commissioned again. Remove it from the controllers (and its ACL entry from the sensor) first.

### 2.1 Device information

`menuconfig` -> `Thermostat Device Information` sets the BasicInformation values shown by the controllers
instead of TEST_VENDOR / TEST_PRODUCT: vendor name, product name, hardware version and the serial number
prefix (the serial number is the prefix + the MAC address). It uses `CONFIG_CUSTOM_DEVICE_INSTANCE_INFO_PROVIDER`.
Apple Home reads them when the accessory is added.

## 3. Control logic

The last temperature received from the sensor is stored in a variable and updated on every report.
Each new report, each change of the mode or of a setpoint re-evaluates the thermostat:

-   Heat: the heating relay turns ON when `temperature <= heating setpoint - hysteresis` and OFF when
    `temperature >= heating setpoint`.
-   Cool: the cooling relay turns ON when `temperature >= cooling setpoint + hysteresis` and OFF when
    `temperature <= cooling setpoint`.
-   Auto: both rules above, each with its own setpoint.
-   Off (and FanOnly/Dry/Sleep): both relays are OFF.

The hysteresis is `THERMOSTAT_HYSTERESIS` (default 0.5 °C). The sensor temperature is first rounded to the
nearest `THERMOSTAT_TEMPERATURE_STEP` (default 0.5 °C: 24.20 -> 24.0, 24.30 -> 24.5). The rounded value is
shown as `LocalTemperature` and compared with the setpoints, so the relays follow the temperature the
controllers show. Set the step to 1 to use the exact 0.01 °C value from the sensor.

Calibration: `LocalTemperatureCalibration` (0x0010, 0.1 °C, -2.5 .. +2.5 °C) is added to the sensor
temperature before the rounding: LocalTemperature = round(sensor + calibration). The Thermostat server does
not apply it, the application does. The first value comes from `THERMOSTAT_TEMPERATURE_CALIBRATION`,
a value written by a controller is kept in NVS. After a change the temperature is recalculated at once from
the last sensor value. Apple Home does not show it; with chip-tool, for -0.5 °C:
`chip-tool thermostat write local-temperature-calibration -5 <node-id> 1`.
If the sensor does not report for `THERMOSTAT_SENSOR_TIMEOUT` seconds (default 300), the temperature
becomes unknown and both relays are switched OFF. Sleepy (ICD) Thread sensors may choose a max reporting
interval longer than the requested one, then the timeout is extended to two max intervals + 60 s.

## 4. Post Commissioning Setup

### 4.1 With Home Assistant (easiest)

Home Assistant (Matter integration with the Matter Server add-on) can commission the thermostat, bind the
sensor and update the firmware, without any command line.

**Commissioning**

1.  Settings -> Devices & services -> Add device -> Matter, scan the QR code or enter the manual pairing
    code (default test credentials: `34970112332`, passcode 20202021, discriminator 3840).
2.  Commission the thermostat into Home Assistant first, so it joins the same Thread network as the sensor.
3.  Share it to other controllers (e.g. Apple Home) from the device page: Share device.

**Binding the sensor**

1.  Open the thermostat device page. Its client clusters are listed: TemperatureMeasurement (0x0402),
    RelativeHumidityMeasurement (0x0405) and PowerSource (0x002F).
2.  Bind TemperatureMeasurement to the temperature endpoint of the sensor (e.g. EP 1 Temperature Sensor) and
    RelativeHumidityMeasurement to its humidity endpoint (e.g. EP 2 Humidity Sensor).
3.  Home Assistant writes the binding table of the thermostat and adds the ACL entry on the sensor, which
    allows the thermostat to subscribe ("ACL present").
4.  The battery needs no binding (see 4.4). PowerSource can still be bound to the sensor endpoint which has
    it (usually EP 0, shown as Root Node / OTA Requestor).

The thermostat log shows `Subscribing to sensor node ...` and then `Remote temperature`, `Remote humidity`
and `Remote battery`.

Every commissioning gives the thermostat a new node id and the sensor keeps the ACL entries of the old ones.
Matter guarantees only 4 ACL entries per fabric, so "access control list is full" can appear after a few
re-commissionings: remove the old entries or factory reset the sensor.

**Firmware update (OTA)**

1.  Increase `PROJECT_VER_NUMBER` (and `PROJECT_VER`) in `CMakeLists.txt`, then `idf.py build`.
2.  Open the Matter Server panel in the Home Assistant sidebar, select the thermostat node and press
    Upload firmware. Choose `build/air_conditioner_ir-ota.bin` (the file with `-ota`, it has the Matter
    OTA header; the plain `.bin` is only for flashing over USB).
3.  Confirm the update (it is shown as found on "local"; the warning about an unverified source is expected
    for a self-built firmware). The image is transferred over Thread in a few minutes and the thermostat
    restarts with the new version. Fabrics, bindings and settings are kept.

Instead of the button, the image can be copied to `addon_configs/core_matter_server/updates/` (e.g. with the
Samba share add-on) with the add-on option `enable_test_net_dcl` enabled, then restart the Matter Server add-on.
The file is imported and removed from the folder.

### 4.2 Bind the sensor with chip-tool

Using the chip-tool, commission 2 devices, the thermostat and a temperature sensor
(for example the [sensors](../sensors) example).
If you are having trouble, try commissioning them one at a time (by powering off the other device) as
the default discriminator and passcode are same for both of them.

For the commands below:

-   Node Id of thermostat used during commissioning is 0x7283 (29315 in decimal)
-   Node Id of sensor used during commissioning is 0x5164 (20836 in decimal)
-   The temperature sensor is on endpoint 1 of the sensor
-   Cluster Id for TemperatureMeasurement cluster is 0x0402 (1026 in decimal)
-   Cluster Id for RelativeHumidityMeasurement cluster is 0x0405 (1029 in decimal)
-   Binding cluster is present on endpoint 1 on the thermostat

Update the sensor's acl attribute to allow the thermostat to read and subscribe (View privilege).
Writing the acl replaces the whole list, so keep the administrator entry:
```
accesscontrol write acl '[{"privilege": 5, "authMode": 2, "subjects": [ 112233 ], "targets": null}, {"privilege": 1, "authMode": 2, "subjects": [ 29315 ], "targets": null}]' 0x5164 0x0
```

Update the thermostat's binding attribute to add the entry of the sensor in the binding table:
```
binding write binding '[{"node":20836, "endpoint":1, "cluster":1026}]' 0x7283 0x1
```

### 4.3 Bind a humidity sensor with chip-tool

If the humidity comes from the same sensor endpoint, add a second entry with the same node and endpoint,
one subscription is used for both values:
```
binding write binding '[{"node":20836, "endpoint":1, "cluster":1026}, {"node":20836, "endpoint":1, "cluster":1029}]' 0x7283 0x1
```

If the humidity is on another endpoint or on another node (for example node 0x5165 (20837), endpoint 2),
the thermostat uses a separate subscription for it (do not forget the acl on that node as well):
```
binding write binding '[{"node":20836, "endpoint":1, "cluster":1026}, {"node":20837, "endpoint":2, "cluster":1029}]' 0x7283 0x1
```

A binding entry without the `cluster` field binds the whole remote endpoint, it is used for both the
temperature and the humidity.

### 4.4 Battery of the sensor

No binding is needed for the battery: the thermostat reads the PowerSource cluster 0x002F (47) from the node
of the bound temperature sensor, with a wildcard endpoint (the battery is usually on endpoint 0, which the
binding UI of the controllers may not offer). If the battery is on another node, bind it explicitly:
```
binding write binding '[{"node":20836, "endpoint":1, "cluster":1026}, {"node":20836, "endpoint":1, "cluster":1029}, {"node":20836, "endpoint":0, "cluster":47}]' 0x7283 0x1
```

All the values bound to the same sensor node come with one subscription, also from different endpoints,
so a battery powered sensor sends one report for all of them.

### 4.5 Control the thermostat with chip-tool

Set Heat mode and a heating setpoint of 22 °C:
```
thermostat write system-mode 4 0x7283 0x1
thermostat write occupied-heating-setpoint 2200 0x7283 0x1
```

Set Auto mode with a range of 21 °C .. 25 °C:
```
thermostat write system-mode 1 0x7283 0x1
thermostat write occupied-heating-setpoint 2100 0x7283 0x1
thermostat write occupied-cooling-setpoint 2500 0x7283 0x1
```

Read the running state (bit 0: heat, bit 1: cool):
```
thermostat read thermostat-running-state 0x7283 0x1
```

## 5. Configuration

`menuconfig` -> `Thermostat Configuration`:

| Option                               | Default | Description                                          |
|--------------------------------------|---------|------------------------------------------------------|
| `THERMOSTAT_ENABLE_COOLING`          | y       | Cooling feature and cooling relay                    |
| `THERMOSTAT_ENABLE_AUTO`             | y       | AutoMode feature                                     |
| `THERMOSTAT_MIN_SETPOINT`            | 16      | Lowest setpoint (°C)                                 |
| `THERMOSTAT_MAX_SETPOINT`            | 30      | Highest setpoint (°C)                                |
| `THERMOSTAT_DEAD_BAND`               | 0       | Auto dead band (0.1 °C)                              |
| `THERMOSTAT_HYSTERESIS`              | 50      | Hysteresis in 0.01 °C                                |
| `THERMOSTAT_TEMPERATURE_STEP`        | 50      | Sensor temperature rounding step in 0.01 °C          |
| `THERMOSTAT_TEMPERATURE_CALIBRATION` | 0       | Initial temperature calibration in 0.1 °C (-25..25)  |
| `THERMOSTAT_SENSOR_MIN_INTERVAL`     | 5       | Subscription min interval (s)                        |
| `THERMOSTAT_SENSOR_MAX_INTERVAL`     | 60      | Subscription max interval (s)                        |
| `THERMOSTAT_SENSOR_TIMEOUT`          | 300     | Relays off when the sensor is silent this long (s)   |
| `THERMOSTAT_TRACE_LOGS`              | n       | Debug logs of every attribute write                  |

The default setpoints are in `main/app_priv.h`. The setpoint limits and the dead band are written back on
every boot, because the old values are kept in NVS after a firmware update.

### 5.1 OTA update

`CONFIG_CHIP_OTA_IMAGE_BUILD` is enabled, so `idf.py build` also creates `build/air_conditioner_ir-ota.bin`
with the Matter OTA header. An OTA Provider (e.g. Home Assistant, see 4.1) only offers an image whose software
version is higher than the one running on the device. Increase `PROJECT_VER_NUMBER` (and `PROJECT_VER`) in
`CMakeLists.txt` for every new image.

### 5.2 Thread on ESP32-C6

The default configuration uses Wi-Fi. To build for Thread on ESP32-C6:
```
idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults.c6_thread" set-target esp32c6 build
```
