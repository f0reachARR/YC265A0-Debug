/*
 * tlv320aic32x4.c - TLV320AIC32x4 Audio Codec Driver for ESP-IDF
 *
 * Ported from Linux kernel driver
 * Original: Copyright 2011 Vista Silicon S.L.
 * ESP-IDF Port: 2026
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "tlv320aic32x4.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "AIC32X4";

/* Rate dividers table - supports common sample rates with different MCLKs */
static const aic32x4_rate_divs_t aic32x4_divs[] = {
    /* 8k rate */
    {AIC32X4_FREQ_12000000, 8000, 1, 7, 6800, 768, 5, 3, 128, 5, 18, 24},
    {AIC32X4_FREQ_24000000, 8000, 2, 7, 6800, 768, 15, 1, 64, 45, 4, 24},
    {AIC32X4_FREQ_25000000, 8000, 2, 7, 3728, 768, 15, 1, 64, 45, 4, 24},
    /* 11.025k rate */
    {AIC32X4_FREQ_12000000, 11025, 1, 7, 5264, 512, 8, 2, 128, 8, 8, 16},
    {AIC32X4_FREQ_24000000, 11025, 2, 7, 5264, 512, 16, 1, 64, 32, 4, 16},
    /* 16k rate */
    {AIC32X4_FREQ_12000000, 16000, 1, 7, 6800, 384, 5, 3, 128, 5, 9, 12},
    {AIC32X4_FREQ_24000000, 16000, 2, 7, 6800, 384, 15, 1, 64, 18, 5, 12},
    {AIC32X4_FREQ_25000000, 16000, 2, 7, 3728, 384, 15, 1, 64, 18, 5, 12},
    /* 22.05k rate */
    {AIC32X4_FREQ_12000000, 22050, 1, 7, 5264, 256, 4, 4, 128, 4, 8, 8},
    {AIC32X4_FREQ_24000000, 22050, 2, 7, 5264, 256, 16, 1, 64, 16, 4, 8},
    {AIC32X4_FREQ_25000000, 22050, 2, 7, 2253, 256, 16, 1, 64, 16, 4, 8},
    /* 32k rate */
    {AIC32X4_FREQ_12000000, 32000, 1, 7, 1680, 192, 2, 7, 64, 2, 21, 6},
    {AIC32X4_FREQ_24000000, 32000, 2, 7, 1680, 192, 7, 2, 64, 7, 6, 6},
    /* 44.1k rate */
    {AIC32X4_FREQ_12000000, 44100, 1, 7, 5264, 128, 2, 8, 128, 2, 8, 4},
    {AIC32X4_FREQ_24000000, 44100, 2, 7, 5264, 128, 8, 2, 64, 8, 4, 4},
    {AIC32X4_FREQ_25000000, 44100, 2, 7, 2253, 128, 8, 2, 64, 8, 4, 4},
    /* 48k rate */
    {AIC32X4_FREQ_12000000, 48000, 1, 8, 1920, 128, 2, 8, 128, 2, 8, 4},
    {AIC32X4_FREQ_24000000, 48000, 2, 8, 1920, 128, 8, 2, 64, 8, 4, 4},
    {AIC32X4_FREQ_25000000, 48000, 2, 7, 8643, 128, 8, 2, 64, 8, 4, 4}
};

#define AIC32X4_NUM_DIVS (sizeof(aic32x4_divs) / sizeof(aic32x4_divs[0]))

/**
 * @brief Find divider configuration for given MCLK and sample rate
 */
static int aic32x4_get_divs(uint32_t mclk, uint32_t rate)
{
    for (int i = 0; i < AIC32X4_NUM_DIVS; i++) {
        if ((aic32x4_divs[i].rate == rate) && (aic32x4_divs[i].mclk == mclk)) {
            return i;
        }
    }
    ESP_LOGE(TAG, "Master clock %lu and sample rate %lu is not supported", mclk, rate);
    return -1;
}

/**
 * @brief Select register page
 */
static esp_err_t aic32x4_select_page(aic32x4_handle_t *handle, uint8_t page)
{
    if (handle->current_page == page) {
        return ESP_OK;
    }

    uint8_t data[2] = {AIC32X4_PSEL, page};
    esp_err_t ret = i2c_master_transmit(handle->i2c_dev_handle, data, 2, 1000);
    if (ret == ESP_OK) {
        handle->current_page = page;
    }
    return ret;
}

esp_err_t aic32x4_write_reg(aic32x4_handle_t *handle, uint8_t reg, uint8_t value)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Determine which page this register is on */
    uint8_t page = (reg >= AIC32X4_PAGE1) ? 1 : 0;
    uint8_t actual_reg = (page == 1) ? (reg - AIC32X4_PAGE1) : reg;

    /* Select page if needed */
    esp_err_t ret = aic32x4_select_page(handle, page);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select page %d", page);
        return ret;
    }

    /* Write register */
    uint8_t data[2] = {actual_reg, value};
    ret = i2c_master_transmit(handle->i2c_dev_handle, data, 2, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write reg 0x%02x = 0x%02x", reg, value);
    }
    return ret;
}

esp_err_t aic32x4_read_reg(aic32x4_handle_t *handle, uint8_t reg, uint8_t *value)
{
    if (handle == NULL || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Determine which page this register is on */
    uint8_t page = (reg >= AIC32X4_PAGE1) ? 1 : 0;
    uint8_t actual_reg = (page == 1) ? (reg - AIC32X4_PAGE1) : reg;

    /* Select page if needed */
    esp_err_t ret = aic32x4_select_page(handle, page);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select page %d", page);
        return ret;
    }

    /* Read register */
    ret = i2c_master_transmit_receive(handle->i2c_dev_handle, &actual_reg, 1, value, 1, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read reg 0x%02x", reg);
    }
    return ret;
}

esp_err_t aic32x4_update_bits(aic32x4_handle_t *handle, uint8_t reg, uint8_t mask, uint8_t value)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t old_val, new_val;
    esp_err_t ret = aic32x4_read_reg(handle, reg, &old_val);
    if (ret != ESP_OK) {
        return ret;
    }

    new_val = (old_val & ~mask) | (value & mask);
    if (new_val == old_val) {
        return ESP_OK;
    }

    return aic32x4_write_reg(handle, reg, new_val);
}

esp_err_t aic32x4_reset(aic32x4_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Hardware reset if GPIO is configured */
    if (handle->reset_gpio >= 0) {
        gpio_set_level(handle->reset_gpio, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(handle->reset_gpio, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* Software reset */
    handle->current_page = 0xFF; /* Invalidate page cache */
    return aic32x4_write_reg(handle, AIC32X4_RESET, 0x01);
}

esp_err_t aic32x4_set_sample_rate(aic32x4_handle_t *handle, uint32_t sample_rate, uint8_t word_length)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int i = aic32x4_get_divs(handle->sysclk, sample_rate);
    if (i < 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    esp_err_t ret;
    uint8_t data;

    /* Use PLL as CODEC_CLKIN and DAC_MOD_CLK as BDIV_CLKIN */
    ret = aic32x4_write_reg(handle, AIC32X4_CLKMUX, AIC32X4_PLLCLKIN);
    if (ret != ESP_OK) return ret;

    ret = aic32x4_write_reg(handle, AIC32X4_IFACE3, AIC32X4_DACMOD2BCLK);
    if (ret != ESP_OK) return ret;

    /* Configure PLL: R=1, P and J=K.D as variable */
    ret = aic32x4_read_reg(handle, AIC32X4_PLLPR, &data);
    if (ret != ESP_OK) return ret;
    data &= ~(7 << 4);
    ret = aic32x4_write_reg(handle, AIC32X4_PLLPR, (data | (aic32x4_divs[i].p_val << 4) | 0x01));
    if (ret != ESP_OK) return ret;

    ret = aic32x4_write_reg(handle, AIC32X4_PLLJ, aic32x4_divs[i].pll_j);
    if (ret != ESP_OK) return ret;

    ret = aic32x4_write_reg(handle, AIC32X4_PLLDMSB, (aic32x4_divs[i].pll_d >> 8));
    if (ret != ESP_OK) return ret;

    ret = aic32x4_write_reg(handle, AIC32X4_PLLDLSB, (aic32x4_divs[i].pll_d & 0xff));
    if (ret != ESP_OK) return ret;

    /* NDAC divider value */
    ret = aic32x4_update_bits(handle, AIC32X4_NDAC, 0x7f, aic32x4_divs[i].ndac);
    if (ret != ESP_OK) return ret;

    /* MDAC divider value */
    ret = aic32x4_update_bits(handle, AIC32X4_MDAC, 0x7f, aic32x4_divs[i].mdac);
    if (ret != ESP_OK) return ret;

    /* DOSR MSB & LSB values */
    ret = aic32x4_write_reg(handle, AIC32X4_DOSRMSB, aic32x4_divs[i].dosr >> 8);
    if (ret != ESP_OK) return ret;

    ret = aic32x4_write_reg(handle, AIC32X4_DOSRLSB, (aic32x4_divs[i].dosr & 0xff));
    if (ret != ESP_OK) return ret;

    /* NADC divider value */
    ret = aic32x4_update_bits(handle, AIC32X4_NADC, 0x7f, aic32x4_divs[i].nadc);
    if (ret != ESP_OK) return ret;

    /* MADC divider value */
    ret = aic32x4_update_bits(handle, AIC32X4_MADC, 0x7f, aic32x4_divs[i].madc);
    if (ret != ESP_OK) return ret;

    /* AOSR value */
    ret = aic32x4_write_reg(handle, AIC32X4_AOSR, aic32x4_divs[i].aosr);
    if (ret != ESP_OK) return ret;

    /* BCLK N divider */
    ret = aic32x4_update_bits(handle, AIC32X4_BCLKN, 0x7f, aic32x4_divs[i].blck_n);
    if (ret != ESP_OK) return ret;

    /* Set word length */
    ret = aic32x4_read_reg(handle, AIC32X4_IFACE1, &data);
    if (ret != ESP_OK) return ret;

    data = data & ~(3 << 4);
    switch (word_length) {
        case 16:
            break;
        case 20:
            data |= (AIC32X4_WORD_LEN_20BITS << AIC32X4_DOSRMSB_SHIFT);
            break;
        case 24:
            data |= (AIC32X4_WORD_LEN_24BITS << AIC32X4_DOSRMSB_SHIFT);
            break;
        case 32:
            data |= (AIC32X4_WORD_LEN_32BITS << AIC32X4_DOSRMSB_SHIFT);
            break;
        default:
            ESP_LOGE(TAG, "Unsupported word length: %d", word_length);
            return ESP_ERR_INVALID_ARG;
    }
    ret = aic32x4_write_reg(handle, AIC32X4_IFACE1, data);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "Sample rate set to %lu Hz, word length %d bits", sample_rate, word_length);
    return ESP_OK;
}

esp_err_t aic32x4_set_format(aic32x4_handle_t *handle, uint8_t format, bool master)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    uint8_t iface_reg_1, iface_reg_2, iface_reg_3;

    ret = aic32x4_read_reg(handle, AIC32X4_IFACE1, &iface_reg_1);
    if (ret != ESP_OK) return ret;

    iface_reg_1 = iface_reg_1 & ~(3 << 6 | 3 << 2);
    iface_reg_2 = 0;

    ret = aic32x4_read_reg(handle, AIC32X4_IFACE3, &iface_reg_3);
    if (ret != ESP_OK) return ret;

    iface_reg_3 = iface_reg_3 & ~(1 << 3);

    /* Set master/slave mode */
    if (master) {
        iface_reg_1 |= AIC32X4_BCLKMASTER | AIC32X4_WCLKMASTER;
    }

    /* Set audio format */
    switch (format) {
        case AIC32X4_I2S_MODE:
            break;
        case AIC32X4_DSP_MODE:
            iface_reg_1 |= (AIC32X4_DSP_MODE << AIC32X4_PLLJ_SHIFT);
            iface_reg_3 |= (1 << 3); /* invert bit clock */
            iface_reg_2 = 0x01;      /* add offset 1 */
            break;
        case AIC32X4_RIGHT_JUSTIFIED_MODE:
            iface_reg_1 |= (AIC32X4_RIGHT_JUSTIFIED_MODE << AIC32X4_PLLJ_SHIFT);
            break;
        case AIC32X4_LEFT_JUSTIFIED_MODE:
            iface_reg_1 |= (AIC32X4_LEFT_JUSTIFIED_MODE << AIC32X4_PLLJ_SHIFT);
            break;
        default:
            ESP_LOGE(TAG, "Invalid audio format: %d", format);
            return ESP_ERR_INVALID_ARG;
    }

    ret = aic32x4_write_reg(handle, AIC32X4_IFACE1, iface_reg_1);
    if (ret != ESP_OK) return ret;

    ret = aic32x4_write_reg(handle, AIC32X4_IFACE2, iface_reg_2);
    if (ret != ESP_OK) return ret;

    ret = aic32x4_write_reg(handle, AIC32X4_IFACE3, iface_reg_3);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "Audio format set to %d, %s mode", format, master ? "master" : "slave");
    return ESP_OK;
}

esp_err_t aic32x4_set_dac_volume(aic32x4_handle_t *handle, uint8_t left_vol, uint8_t right_vol)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Volume range is 0-63 (0x00-0x3F), values are negative dB */
    if (left_vol > 63) left_vol = 63;
    if (right_vol > 63) right_vol = 63;

    esp_err_t ret;
    ret = aic32x4_write_reg(handle, AIC32X4_LDACVOL, left_vol);
    if (ret != ESP_OK) return ret;

    ret = aic32x4_write_reg(handle, AIC32X4_RDACVOL, right_vol);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "DAC volume set to L:%d R:%d", left_vol, right_vol);
    return ESP_OK;
}

esp_err_t aic32x4_set_mute(aic32x4_handle_t *handle, bool mute)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t dac_reg;
    esp_err_t ret = aic32x4_read_reg(handle, AIC32X4_DACMUTE, &dac_reg);
    if (ret != ESP_OK) return ret;

    dac_reg &= ~AIC32X4_MUTEON;
    if (mute) {
        dac_reg |= AIC32X4_MUTEON;
    }

    ret = aic32x4_write_reg(handle, AIC32X4_DACMUTE, dac_reg);
    ESP_LOGI(TAG, "DAC %s", mute ? "muted" : "unmuted");
    return ret;
}

esp_err_t aic32x4_set_adc_volume(aic32x4_handle_t *handle, uint8_t left_vol, uint8_t right_vol)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Volume range is 0-40 (0x00-0x28) */
    if (left_vol > 40) left_vol = 40;
    if (right_vol > 40) right_vol = 40;

    esp_err_t ret;
    ret = aic32x4_write_reg(handle, AIC32X4_LADCVOL, left_vol);
    if (ret != ESP_OK) return ret;

    ret = aic32x4_write_reg(handle, AIC32X4_RADCVOL, right_vol);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "ADC volume set to L:%d R:%d", left_vol, right_vol);
    return ESP_OK;
}

/**
 * @brief Power on/off codec blocks
 */
static esp_err_t aic32x4_set_power(aic32x4_handle_t *handle, bool power_on)
{
    esp_err_t ret;

    if (power_on) {
        /* Switch on PLL */
        ret = aic32x4_update_bits(handle, AIC32X4_PLLPR, AIC32X4_PLLEN, AIC32X4_PLLEN);
        if (ret != ESP_OK) return ret;

        /* Switch on NDAC Divider */
        ret = aic32x4_update_bits(handle, AIC32X4_NDAC, AIC32X4_NDACEN, AIC32X4_NDACEN);
        if (ret != ESP_OK) return ret;

        /* Switch on MDAC Divider */
        ret = aic32x4_update_bits(handle, AIC32X4_MDAC, AIC32X4_MDACEN, AIC32X4_MDACEN);
        if (ret != ESP_OK) return ret;

        /* Switch on NADC Divider */
        ret = aic32x4_update_bits(handle, AIC32X4_NADC, AIC32X4_NADCEN, AIC32X4_NADCEN);
        if (ret != ESP_OK) return ret;

        /* Switch on MADC Divider */
        ret = aic32x4_update_bits(handle, AIC32X4_MADC, AIC32X4_MADCEN, AIC32X4_MADCEN);
        if (ret != ESP_OK) return ret;

        /* Switch on BCLK_N Divider */
        ret = aic32x4_update_bits(handle, AIC32X4_BCLKN, AIC32X4_BCLKEN, AIC32X4_BCLKEN);
        if (ret != ESP_OK) return ret;

        ESP_LOGI(TAG, "Codec powered on");
    } else {
        /* Switch off in reverse order */
        ret = aic32x4_update_bits(handle, AIC32X4_BCLKN, AIC32X4_BCLKEN, 0);
        if (ret != ESP_OK) return ret;

        ret = aic32x4_update_bits(handle, AIC32X4_MADC, AIC32X4_MADCEN, 0);
        if (ret != ESP_OK) return ret;

        ret = aic32x4_update_bits(handle, AIC32X4_NADC, AIC32X4_NADCEN, 0);
        if (ret != ESP_OK) return ret;

        ret = aic32x4_update_bits(handle, AIC32X4_MDAC, AIC32X4_MDACEN, 0);
        if (ret != ESP_OK) return ret;

        ret = aic32x4_update_bits(handle, AIC32X4_NDAC, AIC32X4_NDACEN, 0);
        if (ret != ESP_OK) return ret;

        ret = aic32x4_update_bits(handle, AIC32X4_PLLPR, AIC32X4_PLLEN, 0);
        if (ret != ESP_OK) return ret;

        ESP_LOGI(TAG, "Codec powered off");
    }

    return ESP_OK;
}

esp_err_t aic32x4_init(const aic32x4_config_t *config, aic32x4_handle_t **handle)
{
    if (config == NULL || handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Allocate handle */
    aic32x4_handle_t *dev = (aic32x4_handle_t *)calloc(1, sizeof(aic32x4_handle_t));
    if (dev == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for device handle");
        return ESP_ERR_NO_MEM;
    }

    /* Initialize handle */
    dev->i2c_addr = config->i2c_addr;
    dev->sysclk = config->mclk_freq;
    dev->power_cfg = config->power_cfg;
    dev->micpga_routing = config->micpga_routing;
    dev->swap_dacs = config->swap_dacs;
    dev->reset_gpio = config->reset_gpio;
    dev->current_page = 0xFF;

    /* Add device to I2C bus */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->i2c_addr,
        .scl_speed_hz = 100000,
    };
    esp_err_t ret = i2c_master_bus_add_device(config->i2c_bus_handle, &dev_cfg, &dev->i2c_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device");
        free(dev);
        return ret;
    }

    /* Configure reset GPIO if provided */
    if (dev->reset_gpio >= 0) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << dev->reset_gpio),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure reset GPIO");
            i2c_master_bus_rm_device(dev->i2c_dev_handle);
            free(dev);
            return ret;
        }
    }

    /* Reset codec */
    ret = aic32x4_reset(dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset codec");
        i2c_master_bus_rm_device(dev->i2c_dev_handle);
        free(dev);
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    /* Power platform configuration */
    if (dev->power_cfg & AIC32X4_PWR_MICBIAS_2075_LDOIN) {
        ret = aic32x4_write_reg(dev, AIC32X4_MICBIAS,
                                AIC32X4_MICBIAS_LDOIN | AIC32X4_MICBIAS_2075V);
        if (ret != ESP_OK) goto error;
    }

    if (dev->power_cfg & AIC32X4_PWR_AVDD_DVDD_WEAK_DISABLE) {
        ret = aic32x4_write_reg(dev, AIC32X4_PWRCFG, AIC32X4_AVDDWEAKDISABLE);
        if (ret != ESP_OK) goto error;
    }

    uint8_t tmp_reg = (dev->power_cfg & AIC32X4_PWR_AIC32X4_LDO_ENABLE) ? AIC32X4_LDOCTLEN : 0;
    ret = aic32x4_write_reg(dev, AIC32X4_LDOCTL, tmp_reg);
    if (ret != ESP_OK) goto error;

    ret = aic32x4_read_reg(dev, AIC32X4_CMMODE, &tmp_reg);
    if (ret != ESP_OK) goto error;

    if (dev->power_cfg & AIC32X4_PWR_CMMODE_LDOIN_RANGE_18_36) {
        tmp_reg |= AIC32X4_LDOIN_18_36;
    }
    if (dev->power_cfg & AIC32X4_PWR_CMMODE_HP_LDOIN_POWERED) {
        tmp_reg |= AIC32X4_LDOIN2HP;
    }
    ret = aic32x4_write_reg(dev, AIC32X4_CMMODE, tmp_reg);
    if (ret != ESP_OK) goto error;

    /* Mic PGA routing */
    if (dev->micpga_routing & AIC32X4_MICPGA_ROUTE_LMIC_IN2R_10K) {
        ret = aic32x4_write_reg(dev, AIC32X4_LMICPGANIN, AIC32X4_LMICPGANIN_IN2R_10K);
    } else {
        ret = aic32x4_write_reg(dev, AIC32X4_LMICPGANIN, AIC32X4_LMICPGANIN_CM1L_10K);
    }
    if (ret != ESP_OK) goto error;

    if (dev->micpga_routing & AIC32X4_MICPGA_ROUTE_RMIC_IN1L_10K) {
        ret = aic32x4_write_reg(dev, AIC32X4_RMICPGANIN, AIC32X4_RMICPGANIN_IN1L_10K);
    } else {
        ret = aic32x4_write_reg(dev, AIC32X4_RMICPGANIN, AIC32X4_RMICPGANIN_CM1R_10K);
    }
    if (ret != ESP_OK) goto error;

    /* Workaround: power up and down ADC for first capture to work properly */
    ret = aic32x4_read_reg(dev, AIC32X4_ADCSETUP, &tmp_reg);
    if (ret != ESP_OK) goto error;

    ret = aic32x4_write_reg(dev, AIC32X4_ADCSETUP, tmp_reg | AIC32X4_LADC_EN | AIC32X4_RADC_EN);
    if (ret != ESP_OK) goto error;

    ret = aic32x4_write_reg(dev, AIC32X4_ADCSETUP, tmp_reg);
    if (ret != ESP_OK) goto error;

    /* Configure sample rate and format */
    ret = aic32x4_set_sample_rate(dev, config->sample_rate, config->word_length);
    if (ret != ESP_OK) goto error;

    ret = aic32x4_set_format(dev, config->audio_format, config->master_mode);
    if (ret != ESP_OK) goto error;

    /* Configure DAC channel routing */
    uint8_t dac_route;
    if (dev->swap_dacs) {
        dac_route = AIC32X4_RDAC2LCHN | AIC32X4_LDAC2RCHN;
    } else {
        dac_route = AIC32X4_LDAC2LCHN | AIC32X4_RDAC2RCHN;
    }
    ret = aic32x4_update_bits(dev, AIC32X4_DACSETUP, AIC32X4_DAC_CHAN_MASK, dac_route);
    if (ret != ESP_OK) goto error;

    /* Power on codec */
    ret = aic32x4_set_power(dev, true);
    if (ret != ESP_OK) goto error;

    *handle = dev;
    ESP_LOGI(TAG, "TLV320AIC32x4 initialized successfully");
    return ESP_OK;

error:
    i2c_master_bus_rm_device(dev->i2c_dev_handle);
    free(dev);
    return ret;
}

esp_err_t aic32x4_deinit(aic32x4_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Power off codec */
    aic32x4_set_power(handle, false);

    /* Remove I2C device */
    i2c_master_bus_rm_device(handle->i2c_dev_handle);

    /* Free handle */
    free(handle);

    ESP_LOGI(TAG, "TLV320AIC32x4 deinitialized");
    return ESP_OK;
}