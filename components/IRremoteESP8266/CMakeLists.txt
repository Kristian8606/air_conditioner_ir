# IRremoteESP8266 v2.9.0 as an ESP-IDF component, see README_ESP_IDF.md.
file(GLOB IR_SRCS "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp")
list(APPEND IR_SRCS "${CMAKE_CURRENT_LIST_DIR}/port/ir_rmt_tx.cpp")

idf_component_register(SRCS ${IR_SRCS}
                       INCLUDE_DIRS "src" "port"
                       REQUIRES esp_driver_gpio esp_timer esp_rom esp_system freertos
                       PRIV_REQUIRES esp_driver_rmt log)

# Only the protocol selected in menuconfig ("IR air conditioner") is compiled: the whole library does not fit
# into the OTA partition next to Matter. The macros change the classes, so they are PUBLIC: every source that
# includes the headers must see the same values.
if(CONFIG_IR_AC_PROTOCOL_MACRO AND CONFIG_IR_AC_PROTOCOL)
    set(IR_PROTOCOL_MACRO ${CONFIG_IR_AC_PROTOCOL_MACRO})
    set(IR_PROTOCOL ${CONFIG_IR_AC_PROTOCOL})
else()
    set(IR_PROTOCOL_MACRO FUJITSU_AC)
    set(IR_PROTOCOL FUJITSU_AC)
endif()
# IR_AC_PROTOCOL_ENUM: the decode_type_t value of the selected protocol, e.g. decode_type_t::IR_AC_PROTOCOL_ENUM
target_compile_definitions(${COMPONENT_LIB} PUBLIC
                           IRREMOTE_ESP_IDF=1
                           IR_AC_PROTOCOL_ENUM=${IR_PROTOCOL}
                           _IR_ENABLE_DEFAULT_=false
                           SEND_${IR_PROTOCOL_MACRO}=true
                           DECODE_${IR_PROTOCOL_MACRO}=true)

# Third party code: its warnings are not ours to fix
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -w)
