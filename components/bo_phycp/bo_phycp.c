#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <sys/param.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_phy_init.h"
#include "phy.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_attr.h"

#include "phy_init_data.h"

#include "sdkconfig.h"

#if CONFIG_BO_PHYCP_ENABLED

static const char *TAG = "bo_phycp";

static esp_err_t load_cal_data(esp_phy_calibration_data_t* out_cal_data)
{
    ESP_LOGD(TAG, "%s", __func__);

    const esp_partition_t *cal_data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, CONFIG_BO_PHYCP_PARTITION_SUBTYPE, NULL);
    assert(cal_data_partition && cal_data_partition->size >= sizeof(esp_phy_calibration_data_t));

    spi_flash_mmap_handle_t cal_data_map;
    const void *data_ptr = NULL;
    esp_err_t err = esp_partition_mmap(cal_data_partition, 0, cal_data_partition->size, SPI_FLASH_MMAP_DATA, &data_ptr, &cal_data_map);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mmap err (0x%x)", err);
        return ESP_FAIL;
    }

    memcpy(out_cal_data, data_ptr, sizeof(esp_phy_calibration_data_t));
    spi_flash_munmap(cal_data_map);

    uint32_t cal_format_version = phy_get_rf_cal_version() & (~BIT(16));
    uint32_t cal_data_version = *(uint32_t *)out_cal_data->version;
    if(cal_format_version != cal_data_version)
    {
        ESP_LOGW(TAG, "invalid data version (%u != %u), calibrating", cal_data_version, cal_format_version);
        return ESP_ERR_INVALID_VERSION;
    }

    return ESP_OK;
}

static esp_err_t store_cal_data(const esp_phy_calibration_data_t* cal_data)
{
    ESP_LOGD(TAG, "%s", __func__);

    const esp_partition_t *cal_data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, CONFIG_BO_PHYCP_PARTITION_SUBTYPE, NULL);
    assert(cal_data_partition && cal_data_partition->size >= sizeof(esp_phy_calibration_data_t));

    esp_err_t err = esp_partition_erase_range(cal_data_partition, 0, cal_data_partition->size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "erase err (0x%x)", err);
        return err;
    }

    err = esp_partition_write(cal_data_partition, 0, cal_data, sizeof(*cal_data));
    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "write err (0x%x)", err);
        return err;
    }

    return ESP_OK;
}

#if CONFIG_ESP32_REDUCE_PHY_TX_POWER
// TODO: fix the esp_phy_reduce_tx_power unused warning for esp32s2 - IDF-759
static void __attribute((unused)) esp_phy_reduce_tx_power(esp_phy_init_data_t* init_data)
{
    uint8_t i;

    for(i = 0; i < PHY_TX_POWER_NUM; i++) {
        // LOWEST_PHY_TX_POWER is the lowest tx power
        init_data->params[PHY_TX_POWER_OFFSET+i] = PHY_TX_POWER_LOWEST;
    }
}
#endif

void esp_phy_load_cal_and_init(void)
{
    esp_phy_calibration_data_t* cal_data =
            (esp_phy_calibration_data_t*) calloc(sizeof(esp_phy_calibration_data_t), 1);
    if (cal_data == NULL) {
        ESP_LOGE(TAG, "failed to allocate memory for RF calibration data");
        abort();
    }

#if CONFIG_ESP32_REDUCE_PHY_TX_POWER
    const esp_phy_init_data_t* phy_init_data = esp_phy_get_init_data();
    if (phy_init_data == NULL) {
        ESP_LOGE(TAG, "failed to obtain PHY init data");
        abort();
    }

    esp_phy_init_data_t* init_data = (esp_phy_init_data_t*) malloc(sizeof(esp_phy_init_data_t));
    if (init_data == NULL) {
        ESP_LOGE(TAG, "failed to allocate memory for phy init data");
        abort();
    }

    memcpy(init_data, phy_init_data, sizeof(esp_phy_init_data_t));
    if (esp_reset_reason() == ESP_RST_BROWNOUT) {
        esp_phy_reduce_tx_power(init_data);
    }
#else
    const esp_phy_init_data_t* init_data = esp_phy_get_init_data();
    if (init_data == NULL) {
        ESP_LOGE(TAG, "failed to obtain PHY init data");
        abort();
    }
#endif

    esp_err_t err = load_cal_data(cal_data);
    if(err != ESP_OK)
    {
        register_chipv7_phy(init_data, cal_data, PHY_RF_CAL_FULL);
        ESP_LOGI(TAG, "saving new calibration data");
        err = store_cal_data(cal_data);
    }
    else
    {
        esp_err_t ret = register_chipv7_phy(init_data, cal_data, PHY_RF_CAL_NONE);
        if (ret == ESP_CAL_DATA_CHECK_FAIL) {
            ESP_LOGW(TAG, "saving new calibration data because of failure");
            err = store_cal_data(cal_data);
        }
    }

#if CONFIG_ESP32_REDUCE_PHY_TX_POWER
    esp_phy_release_init_data(phy_init_data);
    free(init_data);
#else
    esp_phy_release_init_data(init_data);
#endif

    free(cal_data); // PHY maintains a copy of calibration data, so we can free this
}

void bo_phycp_include(){}

#endif // CONFIG_BO_PHYCP_ENABLED