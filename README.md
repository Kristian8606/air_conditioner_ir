# Matter Air Conditioner with IR (ESP32-C6, Matter over Thread)

A Matter Room Air Conditioner for an existing split air conditioner. The ESP32-C6 controls the unit with an
IR transmitter (like its original remote) and follows changes from the original remote with an IR receiver.
A bound remote Matter sensor provides the room temperature, humidity and battery.

Work in progress, see [ROADMAP.md](ROADMAP.md).

## IR control

The IR transmitter uses [IRremoteESP8266 for ESP-IDF](components/IRremoteESP8266/README_ESP_IDF.md) (RMT with
a 38 kHz carrier). Every change from Matter sends the whole state in one message, like the remote of the air
conditioner does (log `[IR]`):

| IR state  | From Matter                                                                         |
|-----------|-------------------------------------------------------------------------------------|
| Power     | On/Off on and SystemMode not Off                                                    |
| Mode      | SystemMode: Heat, Cool, Auto (Dry and FanOnly if a controller sets them)            |
| Setpoint  | Heat: heating setpoint, Cool/Dry/Fan: cooling setpoint, Auto: middle of the two. Sent in whole degrees (22.5 → 23), Matter keeps the value as written (`IR_AC_HALF_DEGREES` for remotes with 0.5 °C) |
| Fan       | FanMode Low/Medium/High/Auto (Off is sent as Low: a running unit always runs its fan) |

-   Only a change is sent: the sensor reports do not send anything. While the unit is off, changes of the mode,
    setpoint or fan are sent with the next power on.
-   Moving a slider writes several values: the IR task waits 0.5 s and sends only the last one.
-   Nothing is sent on boot (reboot, OTA update): the air conditioner keeps its own state.
-   A short press of the BOOT button sends the current state again (e.g. when the air conditioner did not
    receive it).
-   The original remote: a complete state message of the selected protocol received by the IR receiver updates
    Matter (On/Off, SystemMode, the setpoint of the mode, FanMode; log `[REMOTE]`), so Apple Home and Home
    Assistant show the change. Nothing is sent back over IR: the state is remembered as the state of the unit and
    no IR is sent for 2 s. Short commands (e.g. Fujitsu swing step) are ignored, "off" switches On/Off off.
    Messages right after our own transmission are not taken as the remote.
-   Tasks: Matter only notifies the IR TX task (`app_ir_notify()`, a task notification) when OnOff, SystemMode,
    a setpoint or FanMode changed. The IR TX task reads the attributes itself with the Matter stack locked, builds
    the message and sends it. The IR RX task decodes the receiver and writes the remote's state into Matter itself,
    also with the stack locked. Both IR tasks run below the Matter task; Matter and OpenThread keep the priorities
    of the thermostat (1 and 5). Long IR logs are off by default, `IR_VERBOSE_LOGS` (menu `IR air conditioner`).
-   With `IR_VERBOSE_LOGS`, every message we send is also decoded by the library and logged (`[IR TX]`: protocol, bytes, description),
    to check the bytes without a receiver.
-   IR receiver (TSOP38238, VS1838B, ... 38 kHz module: VCC 3.3 V, GND, OUT to GPIO 5, internal pull-up): every
    received message is decoded and logged in one line (`[IR RX]`: protocol, bits, bytes). The decoder tolerance
    is 40 % (receiver modules shorten the marks). It decodes the selected protocol and NEC; `IR_RX_DECODE_ALL`
    compiles every decoder of the library to find the protocol of an unknown remote (much more flash). Our own
    echo (until 0.5 s after a transmission) is not taken as the remote. With `IR_VERBOSE_LOGS` also the
    description of the message, the raw durations of unknown messages and the own echo are logged.
-   A double press of the BOOT button selects the next model of the remote (1 .. `IR_AC_MODEL_MAX`, Fujitsu: 1
    ARRAH2E, 2 ARDB1, 3 ARREB1E, 4 ARJW2, 5 ARRY4, 6 ARREW4E, then 1 again), keeps it in NVS and sends the
    current state with it. To find the model without rebuilding: switch the unit on in the app (e.g. Cool), then
    double press until the air conditioner reacts. The log shows `[IR] model N of 6`. A factory reset goes back
    to `IR_AC_MODEL` from menuconfig.

Based on [thermostat_remote_sensor](https://github.com/Kristian8606/thermostat_remote_sensor) v2.3.0.
Own identity: product ID 0x8001 (the thermostat is 0x8000, so the Home Assistant Matter Server never offers
one device the firmware of the other), product name "Air Conditioner", serial number prefix "ACIR-".

See the [docs](https://docs.espressif.com/projects/esp-matter/en/latest/esp32/developing.html) for more
information about building and flashing the firmware.

## 1. Data model

Endpoint 1 is a Room Air Conditioner (0x0072) device:

| Cluster                               | Role            | Purpose                                                           |
|---------------------------------------|-----------------|-------------------------------------------------------------------|
| On/Off                                | Server          | Power of the unit (DeadFrontBehavior)                             |
| Thermostat                            | Server          | SystemMode (Off/Heat/Cool/Auto), heating and cooling setpoints    |
| Fan Control                           | Server          | FanMode Off/Low/Medium/High/Auto, 3 speeds, PercentSetting        |
| Thermostat User Interface Configuration | Server        | Temperature display unit                                          |
| Relative Humidity Measurement         | Server + Client | Client: bound to the sensor. Server: humidity of the room         |
| Temperature Measurement               | Client          | Bound to the remote temperature sensor                            |
| Power Source                          | Server + Client | Battery of the sensor                                             |
| Binding                               | Server          | Holds the binding to the remote sensor(s)                         |
| Identify                              | Server          | Identify and TriggerEffect blink the status LED                   |

-   With On/Off off the unit is off, whatever SystemMode is. SystemMode, the setpoints and On/Off are stored
    in NVS and restored after a reboot.
-   Fan Control is a code-driven cluster in esp-matter: the FanControl server keeps the values (FanMode also
    in NVS) and the application gets the changes through `FanControl::Delegate::OnFanDriveStateChanged()`
    (log `[FAN]`), not through the attribute callback.
-   `LocalTemperature` is the temperature of the bound sensor, not the one of the air conditioner. It is null
    until the first report and again after the sensor timeout.
-   The air conditioner controls the temperature itself and gives no feedback over IR, so
    `ThermostatRunningState` / `ThermostatRunningMode` are derived from the mode: Heat heats, Cool cools,
    FanOnly and Dry run the fan, in Auto the sensor temperature tells which way it works.
-   The setpoint limit attributes are optional in esp-matter and are created in `app_main.cpp`. The range is
    set in menuconfig (default 16..30 °C), stored setpoints outside of it are moved into it on boot.
-   The AttributeChanged callback of the Thermostat cluster is registered in `app_main.cpp`, so in Auto the
    server keeps the cooling setpoint at least `MinSetpointDeadBand` (default 0) above the heating one.

Apple Home and Home Assistant: see [ROADMAP.md](ROADMAP.md), step 1 (known Apple issue: Auto on a Room
Air Conditioner is sent as Cool).

## 2. Hardware

Default pins for ESP32-C6-DevKitC-1/M-1 (menuconfig, `Air Conditioner Configuration`):

| Function        | GPIO | Kconfig                              |
|-----------------|------|--------------------------------------|
| WS2812 LED      | 8    | `THERMOSTAT_STATUS_LED_GPIO`          |
| Boot button     | 9    | Factory reset (long press)            |
| IR transmitter  | 4    | `IR_TX_GPIO` (menu `IR air conditioner`) |
| IR receiver     | 5    | `IR_RX_GPIO` (menu `IR air conditioner`), -1 to disable |

Status LED:

| Color    | Meaning                                                        |
|----------|----------------------------------------------------------------|
| Orange   | Heat                                                           |
| Blue     | Cool                                                           |
| Green    | Auto                                                           |
| Cyan     | FanOnly or Dry                                                 |
| White    | Off (On/Off off or SystemMode Off)                             |
| Purple   | Factory reset pending (release the button)                     |
| Blinking | Identify (white), TriggerEffect Okay (green), ChannelChange (orange) |

Identify blinks the LED white every 0.5 s until `IdentifyTime` reaches 0. TriggerEffect: Blink 1 blink,
Breathe 15 blinks, Okay 2 green blinks, ChannelChange orange for 8 s, FinishEffect ends after the current
blink, StopEffect stops at once. The factory reset indication has priority over the blinking.

IR air conditioner (menuconfig, `IR air conditioner`): protocol `FUJITSU_AC` (default, all air conditioner
protocols of IRremoteESP8266 can be selected, only the selected one is compiled), model `1` (Fujitsu: 1 ARRAH2E,
2 ARDB1, 3 ARREB1E, 4 ARJW2, 5 ARRY4, 6 ARREW4E), transmitter GPIO 4. The IR LED needs a transistor driver,
a GPIO cannot drive it directly.

Factory reset: hold the BOOT button (GPIO 9) for 5 s, the LED turns purple, release it. All fabrics, the
Thread network, the bindings and the settings are erased. Remove the device from the controllers (and its
ACL entry from the sensor) first.

`menuconfig` -> `Air Conditioner Device Information` sets the vendor name, product name, hardware version
and the serial number prefix (serial number = prefix + MAC address).

## 3. Remote sensor temperature

-   The sensor temperature is calibrated first: `LocalTemperatureCalibration` (0x0010, 0.1 °C,
    -2.5 .. +2.5 °C) is added. The Thermostat server does not apply it, the application does. The first value
    comes from `THERMOSTAT_TEMPERATURE_CALIBRATION`, a value written by a controller is kept in NVS, a change
    recalculates the temperature at once. With chip-tool, for -0.5 °C:
    `chip-tool thermostat write local-temperature-calibration -5 <node-id> 1`.
-   Then it is rounded to the nearest `THERMOSTAT_TEMPERATURE_STEP` (default 0.5 °C: 24.20 -> 24.0,
    24.30 -> 24.5) and shown as `LocalTemperature`. A step of 1 keeps the exact 0.01 °C value.
-   If the sensor does not report for `THERMOSTAT_SENSOR_TIMEOUT` seconds (default 300), the temperature
    becomes unknown. Sleepy (ICD) Thread sensors may choose a longer max reporting interval, then the timeout
    is extended to two max intervals + 60 s.

Logs: every value has its own line with the `THERMOSTAT` tag (`idf.py monitor | grep THERMOSTAT`):
`[POWER]`, `[MODE]`, `[SETPOINT]`, `[FAN]`, `[STATE]`, `[TEMP]`, `[CALIBR]`, `[HUMIDITY]`, `[BATTERY]`,
`[IDENTIFY]`, `[IR]`.

## 4. Home Assistant

**Commissioning**

1.  Flash it over USB the first time: `idf.py build flash monitor`.
2.  Settings -> Devices & services -> Add device -> Matter, scan the QR code or enter the manual pairing
    code (default test credentials: `34970112332`, passcode 20202021, discriminator 3840).
3.  Commission it into Home Assistant first, so it joins the same Thread network as the sensor. Share it to
    other controllers (e.g. Apple Home) from the device page: Share device.

**Binding the sensor**

1.  On the device page bind TemperatureMeasurement (0x0402) to the temperature endpoint of the sensor and
    RelativeHumidityMeasurement (0x0405) to its humidity endpoint.
2.  Home Assistant writes the binding table and adds the ACL entry on the sensor ("ACL present").
3.  The battery needs no binding: PowerSource 0x002F is read from the node of the bound temperature sensor
    with a wildcard endpoint.

Every commissioning gives the device a new node id and the sensor keeps the ACL entries of the old ones.
Matter guarantees only 4 ACL entries per fabric: remove the old entries or factory reset the sensor when
"access control list is full" appears.

**Firmware update (OTA)**

1.  Increase `PROJECT_VER_NUMBER` (and `PROJECT_VER`) in `CMakeLists.txt`, then `idf.py build`.
2.  Matter Server panel -> the node -> Upload firmware -> `build/air_conditioner_ir-ota.bin` (the file with
    `-ota`; the plain `.bin` is only for flashing over USB).
3.  Confirm the update. Fabrics, bindings and settings are kept.

## 5. chip-tool

Node ids in the examples: device 0x7283 (29315), sensor 0x5164 (20836), sensor temperature on endpoint 1.

ACL on the sensor (keep the administrator entry) and the binding on the device:
```
accesscontrol write acl '[{"privilege": 5, "authMode": 2, "subjects": [ 112233 ], "targets": null}, {"privilege": 1, "authMode": 2, "subjects": [ 29315 ], "targets": null}]' 0x5164 0x0
binding write binding '[{"node":20836, "endpoint":1, "cluster":1026}, {"node":20836, "endpoint":1, "cluster":1029}]' 0x7283 0x1
```

Power on, Cool at 24 °C, fan Medium:
```
onoff on 0x7283 0x1
thermostat write system-mode 3 0x7283 0x1
thermostat write occupied-cooling-setpoint 2400 0x7283 0x1
fancontrol write fan-mode 2 0x7283 0x1
```

## 6. Configuration

`menuconfig` -> `Air Conditioner Configuration`:

| Option                               | Default | Description                                          |
|--------------------------------------|---------|------------------------------------------------------|
| `THERMOSTAT_ENABLE_AUTO`             | y       | AutoMode feature                                     |
| `THERMOSTAT_MIN_SETPOINT`            | 16      | Lowest setpoint (°C)                                 |
| `THERMOSTAT_MAX_SETPOINT`            | 30      | Highest setpoint (°C)                                |
| `THERMOSTAT_DEAD_BAND`               | 0       | Auto dead band (0.1 °C)                              |
| `THERMOSTAT_STATUS_LED_GPIO`         | 8       | WS2812 status LED, -1 to disable                     |
| `THERMOSTAT_TEMPERATURE_STEP`        | 50      | Sensor temperature rounding step in 0.01 °C          |
| `THERMOSTAT_TEMPERATURE_CALIBRATION` | 0       | Initial temperature calibration in 0.1 °C (-25..25)  |
| `THERMOSTAT_SENSOR_MIN_INTERVAL`     | 5       | Subscription min interval (s)                        |
| `THERMOSTAT_SENSOR_MAX_INTERVAL`     | 60      | Subscription max interval (s)                        |
| `THERMOSTAT_SENSOR_TIMEOUT`          | 300     | Temperature unknown when the sensor is silent this long (s) |
| `THERMOSTAT_TRACE_LOGS`              | n       | Debug logs of every attribute write                  |

The default setpoints are in `main/app_priv.h`. The setpoint limits and the dead band are written back on
every boot, because the old values are kept in NVS after a firmware update.

Thread on ESP32-C6 (the committed `sdkconfig` already uses Thread):
```
idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults.c6_thread" set-target esp32c6 build
```
