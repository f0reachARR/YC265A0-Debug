/**
 * @file example_usage.c
 * @brief TLV320AIC32x4 usage example
 *
 * This example demonstrates how to initialize and use the TLV320AIC32x4 audio codec
 * with ESP-IDF and ESP32's I2S peripheral.
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "tlv320aic32x4.h"

static const char *TAG = "AIC32X4_EXAMPLE";

/* GPIO definitions - adjust these for your hardware */
#define I2C_MASTER_SCL_IO           22      /* GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /* GPIO number for I2C master data  */
#define I2C_MASTER_FREQ_HZ          100000  /* I2C master clock frequency */

#define I2S_BCK_IO                  26      /* GPIO for I2S bit clock */
#define I2S_WS_IO                   25      /* GPIO for I2S word select (LRCLK) */
#define I2S_DO_IO                   27      /* GPIO for I2S data out (to codec DIN) */
#define I2S_DI_IO                   35      /* GPIO for I2S data in (from codec DOUT) */
#define I2S_MCLK_IO                 0       /* GPIO for master clock (MCLK) */

#define CODEC_I2C_ADDR              0x18    /* Default I2C address */
#define CODEC_RESET_GPIO            -1      /* Set to actual GPIO if reset is connected */

#define SAMPLE_RATE                 48000   /* Audio sample rate */
#define BITS_PER_SAMPLE             16      /* Bits per sample */

/* I2S channel handle */
static i2s_chan_handle_t tx_chan = NULL;
static i2s_chan_handle_t rx_chan = NULL;

/* I2C bus handle */
static i2c_master_bus_handle_t i2c_bus_handle = NULL;

/**
 * @brief Initialize I2C master
 */
static esp_err_t i2c_master_init(i2c_master_bus_handle_t *bus_handle)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C master bus init failed");
        return ret;
    }

    ESP_LOGI(TAG, "I2C master initialized");
    return ESP_OK;
}

/**
 * @brief Initialize I2S
 */
static esp_err_t i2s_driver_init(void)
{
    /* Create I2S channel */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    esp_err_t ret = i2s_new_channel(&chan_cfg, &tx_chan, &rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel");
        return ret;
    }

    /* Configure I2S standard mode */
    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = SAMPLE_RATE,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCLK_IO,
            .bclk = I2S_BCK_IO,
            .ws = I2S_WS_IO,
            .dout = I2S_DO_IO,
            .din = I2S_DI_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ret = i2s_channel_init_std_mode(tx_chan, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2S TX channel");
        return ret;
    }

    ret = i2s_channel_init_std_mode(rx_chan, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2S RX channel");
        return ret;
    }

    /* Enable channels */
    ret = i2s_channel_enable(tx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S TX channel");
        return ret;
    }

    ret = i2s_channel_enable(rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S RX channel");
        return ret;
    }

    ESP_LOGI(TAG, "I2S driver initialized");
    return ESP_OK;
}

/**
 * @brief Initialize TLV320AIC32x4 codec
 */
static esp_err_t codec_init(aic32x4_handle_t **handle, i2c_master_bus_handle_t bus_handle)
{
    aic32x4_config_t codec_config = {
        .i2c_bus_handle = bus_handle,
        .i2c_addr = CODEC_I2C_ADDR,
        .mclk_freq = AIC32X4_FREQ_12000000,  /* Adjust based on your MCLK */
        .sample_rate = SAMPLE_RATE,
        .word_length = BITS_PER_SAMPLE,
        .audio_format = AIC32X4_I2S_MODE,
        .master_mode = false,                /* ESP32 is master */
        .power_cfg = AIC32X4_PWR_AVDD_DVDD_WEAK_DISABLE,
        .micpga_routing = 0,
        .swap_dacs = false,
        .reset_gpio = CODEC_RESET_GPIO,
    };

    esp_err_t ret = aic32x4_init(&codec_config, handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize codec");
        return ret;
    }

    /* Set DAC volume (0 = max, 63 = min, in 0.5dB steps) */
    ret = aic32x4_set_dac_volume(*handle, 10, 10);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set DAC volume");
        return ret;
    }

    /* Unmute DAC */
    ret = aic32x4_set_mute(*handle, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to unmute DAC");
        return ret;
    }

    /* Set ADC volume (0 = min, 40 = max) */
    ret = aic32x4_set_adc_volume(*handle, 20, 20);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ADC volume");
        return ret;
    }

    ESP_LOGI(TAG, "Codec initialized successfully");
    return ESP_OK;
}

/**
 * @brief Audio playback example task
 */
static void audio_playback_task(void *pvParameters)
{
    aic32x4_handle_t *codec_handle = (aic32x4_handle_t *)pvParameters;

    /* Generate a simple 1kHz sine wave */
    const int sample_count = SAMPLE_RATE / 1000;  /* 1ms worth of samples */
    int16_t *samples = malloc(sample_count * 2 * sizeof(int16_t));  /* Stereo */

    if (samples == NULL) {
        ESP_LOGE(TAG, "Failed to allocate sample buffer");
        vTaskDelete(NULL);
        return;
    }

    /* Generate sine wave */
    for (int i = 0; i < sample_count; i++) {
        float sample = sin(2.0 * M_PI * 1000.0 * i / SAMPLE_RATE) * 16384.0;
        samples[i * 2] = (int16_t)sample;      /* Left channel */
        samples[i * 2 + 1] = (int16_t)sample;  /* Right channel */
    }

    ESP_LOGI(TAG, "Starting audio playback (1kHz sine wave)");

    size_t bytes_written;
    while (1) {
        /* Write samples to I2S */
        esp_err_t ret = i2s_channel_write(tx_chan, samples,
                                          sample_count * 2 * sizeof(int16_t),
                                          &bytes_written, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2S write failed");
            break;
        }
    }

    free(samples);
    vTaskDelete(NULL);
}

/**
 * @brief Audio loopback example task (ADC -> DAC)
 */
static void audio_loopback_task(void *pvParameters)
{
    aic32x4_handle_t *codec_handle = (aic32x4_handle_t *)pvParameters;

    const int buffer_size = 1024;
    int16_t *buffer = malloc(buffer_size * sizeof(int16_t));

    if (buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate buffer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Starting audio loopback (ADC -> DAC)");

    size_t bytes_read, bytes_written;
    while (1) {
        /* Read from ADC */
        esp_err_t ret = i2s_channel_read(rx_chan, buffer,
                                         buffer_size * sizeof(int16_t),
                                         &bytes_read, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2S read failed");
            break;
        }

        /* Write to DAC */
        ret = i2s_channel_write(tx_chan, buffer, bytes_read,
                               &bytes_written, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2S write failed");
            break;
        }
    }

    free(buffer);
    vTaskDelete(NULL);
}

/**
 * @brief Main application
 */
void app_main(void)
{
    ESP_LOGI(TAG, "TLV320AIC32x4 Example Starting");

    /* Initialize I2C */
    ESP_ERROR_CHECK(i2c_master_init(&i2c_bus_handle));

    /* Initialize I2S */
    ESP_ERROR_CHECK(i2s_driver_init());

    /* Initialize codec */
    aic32x4_handle_t *codec_handle = NULL;
    ESP_ERROR_CHECK(codec_init(&codec_handle, i2c_bus_handle));

    /* Choose one of the following tasks */

    /* Option 1: Play 1kHz sine wave */
    // xTaskCreate(audio_playback_task, "audio_playback", 4096, codec_handle, 5, NULL);

    /* Option 2: Loopback (record and playback simultaneously) */
    xTaskCreate(audio_loopback_task, "audio_loopback", 4096, codec_handle, 5, NULL);

    ESP_LOGI(TAG, "Example running");
}
