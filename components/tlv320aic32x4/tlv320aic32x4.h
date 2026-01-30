/*
 * tlv320aic32x4.h - TLV320AIC32x4 Audio Codec Driver for ESP-IDF
 *
 * Ported from Linux kernel driver
 * Original: Copyright 2011 Vista Silicon S.L.
 * ESP-IDF Port: 2026
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _TLV320AIC32X4_H
#define _TLV320AIC32X4_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* tlv320aic32x4 register space (in decimal to match datasheet) */

#define AIC32X4_PAGE1 128

#define AIC32X4_PSEL 0
#define AIC32X4_RESET 1
#define AIC32X4_CLKMUX 4
#define AIC32X4_PLLPR 5
#define AIC32X4_PLLJ 6
#define AIC32X4_PLLDMSB 7
#define AIC32X4_PLLDLSB 8
#define AIC32X4_NDAC 11
#define AIC32X4_MDAC 12
#define AIC32X4_DOSRMSB 13
#define AIC32X4_DOSRLSB 14
#define AIC32X4_NADC 18
#define AIC32X4_MADC 19
#define AIC32X4_AOSR 20
#define AIC32X4_CLKMUX2 25
#define AIC32X4_CLKOUTM 26
#define AIC32X4_IFACE1 27
#define AIC32X4_IFACE2 28
#define AIC32X4_IFACE3 29
#define AIC32X4_BCLKN 30
#define AIC32X4_IFACE4 31
#define AIC32X4_IFACE5 32
#define AIC32X4_IFACE6 33
#define AIC32X4_DOUTCTL 53
#define AIC32X4_DINCTL 54
#define AIC32X4_DACSPB 60
#define AIC32X4_ADCSPB 61
#define AIC32X4_DACSETUP 63
#define AIC32X4_DACMUTE 64
#define AIC32X4_LDACVOL 65
#define AIC32X4_RDACVOL 66
#define AIC32X4_ADCSETUP 81
#define AIC32X4_ADCFGA 82
#define AIC32X4_LADCVOL 83
#define AIC32X4_RADCVOL 84
#define AIC32X4_LAGC1 86
#define AIC32X4_LAGC2 87
#define AIC32X4_LAGC3 88
#define AIC32X4_LAGC4 89
#define AIC32X4_LAGC5 90
#define AIC32X4_LAGC6 91
#define AIC32X4_LAGC7 92
#define AIC32X4_RAGC1 94
#define AIC32X4_RAGC2 95
#define AIC32X4_RAGC3 96
#define AIC32X4_RAGC4 97
#define AIC32X4_RAGC5 98
#define AIC32X4_RAGC6 99
#define AIC32X4_RAGC7 100
#define AIC32X4_PWRCFG (AIC32X4_PAGE1 + 1)
#define AIC32X4_LDOCTL (AIC32X4_PAGE1 + 2)
#define AIC32X4_OUTPWRCTL (AIC32X4_PAGE1 + 9)
#define AIC32X4_CMMODE (AIC32X4_PAGE1 + 10)
#define AIC32X4_HPLROUTE (AIC32X4_PAGE1 + 12)
#define AIC32X4_HPRROUTE (AIC32X4_PAGE1 + 13)
#define AIC32X4_LOLROUTE (AIC32X4_PAGE1 + 14)
#define AIC32X4_LORROUTE (AIC32X4_PAGE1 + 15)
#define AIC32X4_HPLGAIN (AIC32X4_PAGE1 + 16)
#define AIC32X4_HPRGAIN (AIC32X4_PAGE1 + 17)
#define AIC32X4_LOLGAIN (AIC32X4_PAGE1 + 18)
#define AIC32X4_LORGAIN (AIC32X4_PAGE1 + 19)
#define AIC32X4_HEADSTART (AIC32X4_PAGE1 + 20)
#define AIC32X4_MICBIAS (AIC32X4_PAGE1 + 51)
#define AIC32X4_LMICPGAPIN (AIC32X4_PAGE1 + 52)
#define AIC32X4_LMICPGANIN (AIC32X4_PAGE1 + 54)
#define AIC32X4_RMICPGAPIN (AIC32X4_PAGE1 + 55)
#define AIC32X4_RMICPGANIN (AIC32X4_PAGE1 + 57)
#define AIC32X4_FLOATINGINPUT (AIC32X4_PAGE1 + 58)
#define AIC32X4_LMICPGAVOL (AIC32X4_PAGE1 + 59)
#define AIC32X4_RMICPGAVOL (AIC32X4_PAGE1 + 60)

/* Supported clock frequencies */
#define AIC32X4_FREQ_12000000 12000000
#define AIC32X4_FREQ_24000000 24000000
#define AIC32X4_FREQ_25000000 25000000

/* Word length bits */
#define AIC32X4_WORD_LEN_16BITS 0x00
#define AIC32X4_WORD_LEN_20BITS 0x01
#define AIC32X4_WORD_LEN_24BITS 0x02
#define AIC32X4_WORD_LEN_32BITS 0x03

/* ADC enable bits */
#define AIC32X4_LADC_EN (1 << 7)
#define AIC32X4_RADC_EN (1 << 6)

/* Audio interface modes */
#define AIC32X4_I2S_MODE 0x00
#define AIC32X4_DSP_MODE 0x01
#define AIC32X4_RIGHT_JUSTIFIED_MODE 0x02
#define AIC32X4_LEFT_JUSTIFIED_MODE 0x03

/* Power configuration bits */
#define AIC32X4_AVDDWEAKDISABLE 0x08
#define AIC32X4_LDOCTLEN 0x01

#define AIC32X4_LDOIN_18_36 0x01
#define AIC32X4_LDOIN2HP 0x02

/* Signal processing block masks */
#define AIC32X4_DACSPBLOCK_MASK 0x1f
#define AIC32X4_ADCSPBLOCK_MASK 0x1f

/* Shift values */
#define AIC32X4_PLLJ_SHIFT 6
#define AIC32X4_DOSRMSB_SHIFT 4

#define AIC32X4_PLLCLKIN 0x03

/* Microphone bias configuration */
#define AIC32X4_MICBIAS_LDOIN 0x08
#define AIC32X4_MICBIAS_2075V 0x60

/* Microphone PGA routing */
#define AIC32X4_LMICPGANIN_IN2R_10K 0x10
#define AIC32X4_LMICPGANIN_CM1L_10K 0x40
#define AIC32X4_RMICPGANIN_IN1L_10K 0x10
#define AIC32X4_RMICPGANIN_CM1R_10K 0x40

#define AIC32X4_LMICPGAVOL_NOGAIN 0x80
#define AIC32X4_RMICPGAVOL_NOGAIN 0x80

/* Control bits */
#define AIC32X4_BCLKMASTER 0x08
#define AIC32X4_WCLKMASTER 0x04
#define AIC32X4_PLLEN (0x01 << 7)
#define AIC32X4_NDACEN (0x01 << 7)
#define AIC32X4_MDACEN (0x01 << 7)
#define AIC32X4_NADCEN (0x01 << 7)
#define AIC32X4_MADCEN (0x01 << 7)
#define AIC32X4_BCLKEN (0x01 << 7)
#define AIC32X4_DACEN (0x03 << 6)
#define AIC32X4_RDAC2LCHN (0x02 << 2)
#define AIC32X4_LDAC2RCHN (0x02 << 4)
#define AIC32X4_LDAC2LCHN (0x01 << 4)
#define AIC32X4_RDAC2RCHN (0x01 << 2)
#define AIC32X4_DAC_CHAN_MASK 0x3c

#define AIC32X4_SSTEP2WCLK 0x01
#define AIC32X4_MUTEON 0x0C
#define AIC32X4_DACMOD2BCLK 0x01

/* Power configuration flags for ESP-IDF */
#define AIC32X4_PWR_AIC32X4_LDO_ENABLE (1 << 0)
#define AIC32X4_PWR_AVDD_DVDD_WEAK_DISABLE (1 << 1)
#define AIC32X4_PWR_MICBIAS_2075_LDOIN (1 << 2)
#define AIC32X4_PWR_CMMODE_LDOIN_RANGE_18_36 (1 << 3)
#define AIC32X4_PWR_CMMODE_HP_LDOIN_POWERED (1 << 4)

/* Microphone PGA routing flags */
#define AIC32X4_MICPGA_ROUTE_LMIC_IN2R_10K (1 << 0)
#define AIC32X4_MICPGA_ROUTE_RMIC_IN1L_10K (1 << 1)

/**
 * @brief TLV320AIC32x4 rate divider configuration
 */
typedef struct {
    uint32_t mclk;       /**< Master clock frequency */
    uint32_t rate;       /**< Sample rate */
    uint8_t p_val;       /**< PLL P value */
    uint8_t pll_j;       /**< PLL J value */
    uint16_t pll_d;      /**< PLL D value */
    uint16_t dosr;       /**< DAC oversampling rate */
    uint8_t ndac;        /**< NDAC divider */
    uint8_t mdac;        /**< MDAC divider */
    uint8_t aosr;        /**< ADC oversampling rate */
    uint8_t nadc;        /**< NADC divider */
    uint8_t madc;        /**< MADC divider */
    uint8_t blck_n;      /**< Bit clock divider */
} aic32x4_rate_divs_t;

/**
 * @brief TLV320AIC32x4 configuration structure
 */
typedef struct {
    i2c_master_bus_handle_t i2c_bus_handle; /**< I2C master bus handle */
    uint8_t i2c_addr;              /**< I2C device address (default 0x18) */
    uint32_t mclk_freq;            /**< Master clock frequency */
    uint32_t sample_rate;          /**< Audio sample rate */
    uint8_t word_length;           /**< Word length (16/20/24/32 bits) */
    uint8_t audio_format;          /**< Audio format (I2S/DSP/etc) */
    bool master_mode;              /**< true = master, false = slave */
    uint32_t power_cfg;            /**< Power configuration flags */
    uint32_t micpga_routing;       /**< Microphone PGA routing flags */
    bool swap_dacs;                /**< Swap left/right DAC channels */
    int reset_gpio;                /**< Reset GPIO pin (-1 if not used) */
} aic32x4_config_t;

/**
 * @brief TLV320AIC32x4 device handle
 */
typedef struct {
    i2c_master_dev_handle_t i2c_dev_handle;
    uint8_t i2c_addr;
    uint32_t sysclk;
    uint32_t power_cfg;
    uint32_t micpga_routing;
    bool swap_dacs;
    int reset_gpio;
    uint8_t current_page;         /**< Current register page */
} aic32x4_handle_t;

/**
 * @brief Initialize TLV320AIC32x4 codec
 *
 * @param config Pointer to configuration structure
 * @param handle Pointer to store device handle
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t aic32x4_init(const aic32x4_config_t *config, aic32x4_handle_t **handle);

/**
 * @brief Deinitialize TLV320AIC32x4 codec
 *
 * @param handle Device handle
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t aic32x4_deinit(aic32x4_handle_t *handle);

/**
 * @brief Reset the codec
 *
 * @param handle Device handle
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t aic32x4_reset(aic32x4_handle_t *handle);

/**
 * @brief Set sample rate
 *
 * @param handle Device handle
 * @param sample_rate Sample rate in Hz
 * @param word_length Word length (16/20/24/32 bits)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t aic32x4_set_sample_rate(aic32x4_handle_t *handle, uint32_t sample_rate, uint8_t word_length);

/**
 * @brief Set audio format
 *
 * @param handle Device handle
 * @param format Audio format (I2S_MODE/DSP_MODE/etc)
 * @param master true for master mode, false for slave
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t aic32x4_set_format(aic32x4_handle_t *handle, uint8_t format, bool master);

/**
 * @brief Set DAC volume
 *
 * @param handle Device handle
 * @param left_vol Left channel volume (0-63)
 * @param right_vol Right channel volume (0-63)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t aic32x4_set_dac_volume(aic32x4_handle_t *handle, uint8_t left_vol, uint8_t right_vol);

/**
 * @brief Mute/unmute DAC
 *
 * @param handle Device handle
 * @param mute true to mute, false to unmute
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t aic32x4_set_mute(aic32x4_handle_t *handle, bool mute);

/**
 * @brief Set ADC volume
 *
 * @param handle Device handle
 * @param left_vol Left channel volume (0-40)
 * @param right_vol Right channel volume (0-40)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t aic32x4_set_adc_volume(aic32x4_handle_t *handle, uint8_t left_vol, uint8_t right_vol);

/**
 * @brief Write codec register
 *
 * @param handle Device handle
 * @param reg Register address
 * @param value Value to write
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t aic32x4_write_reg(aic32x4_handle_t *handle, uint8_t reg, uint8_t value);

/**
 * @brief Read codec register
 *
 * @param handle Device handle
 * @param reg Register address
 * @param value Pointer to store read value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t aic32x4_read_reg(aic32x4_handle_t *handle, uint8_t reg, uint8_t *value);

/**
 * @brief Update bits in codec register
 *
 * @param handle Device handle
 * @param reg Register address
 * @param mask Bit mask
 * @param value Value to write (after masking)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t aic32x4_update_bits(aic32x4_handle_t *handle, uint8_t reg, uint8_t mask, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* _TLV320AIC32X4_H */
