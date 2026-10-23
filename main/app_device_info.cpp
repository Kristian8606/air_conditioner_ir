/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_mac.h>
#include <stdio.h>
#include <string.h>

#include <esp_matter_providers.h>
#include <platform/CHIPDeviceLayer.h>
#include <platform/internal/GenericDeviceInstanceInfoProvider.ipp>

#include <app_priv.h>

using namespace chip;
using namespace chip::DeviceLayer;

/* Basic Information of the accessory: vendor name, product name, serial number and hardware version are
 * shown by the controllers (Manufacturer, Model, Serial Number, Firmware/Hardware in Apple Home) instead of
 * TEST_VENDOR / TEST_PRODUCT. All the other values (vendor id, product id, ...) come from the default
 * implementation. */
class thermostat_device_info_provider : public Internal::GenericDeviceInstanceInfoProvider<Internal::ESP32Config> {
public:
    thermostat_device_info_provider() :
        Internal::GenericDeviceInstanceInfoProvider<Internal::ESP32Config>(ConfigurationManagerImpl::GetDefaultInstance())
    {
    }

    CHIP_ERROR GetVendorName(char *buf, size_t bufSize) override { return copy_string(CONFIG_AC_VENDOR_NAME, buf, bufSize); }

    CHIP_ERROR GetProductName(char *buf, size_t bufSize) override
    {
        return copy_string(CONFIG_AC_PRODUCT_NAME, buf, bufSize);
    }

    CHIP_ERROR GetProductLabel(char *buf, size_t bufSize) override
    {
        return copy_string(CONFIG_AC_PRODUCT_NAME, buf, bufSize);
    }

    /* The board and the IR remote, e.g. "ESP32-C6, FUJITSU_AC ARRAH2E" (the model follows a double click on BOOT) */
    CHIP_ERROR GetHardwareVersionString(char *buf, size_t bufSize) override
    {
        char remote[48];
        app_ir_describe(remote, sizeof(remote));
        int len = snprintf(buf, bufSize, "%s, %s", CONFIG_AC_HARDWARE_VERSION_STRING, remote);
        VerifyOrReturnError(len > 0 && (size_t)len < bufSize, CHIP_ERROR_BUFFER_TOO_SMALL);
        return CHIP_NO_ERROR;
    }

    /* Unique per board: the factory programmed MAC address of the chip */
    CHIP_ERROR GetSerialNumber(char *buf, size_t bufSize) override
    {
        uint8_t mac[6];
        VerifyOrReturnError(esp_read_mac(mac, ESP_MAC_BASE) == ESP_OK, CHIP_ERROR_INTERNAL);
        int len = snprintf(buf, bufSize, "%s%02X%02X%02X%02X%02X%02X", CONFIG_AC_SERIAL_NUMBER_PREFIX, mac[0],
                           mac[1], mac[2], mac[3], mac[4], mac[5]);
        VerifyOrReturnError(len > 0 && (size_t)len < bufSize, CHIP_ERROR_BUFFER_TOO_SMALL);
        return CHIP_NO_ERROR;
    }

private:
    static CHIP_ERROR copy_string(const char *value, char *buf, size_t bufSize)
    {
        VerifyOrReturnError(strlen(value) < bufSize, CHIP_ERROR_BUFFER_TOO_SMALL);
        strcpy(buf, value);
        return CHIP_NO_ERROR;
    }
};

void app_device_info_provider_init()
{
    static thermostat_device_info_provider provider;
    /* This should be called before esp_matter::start() */
    esp_matter::set_custom_device_instance_info_provider(&provider);
}
