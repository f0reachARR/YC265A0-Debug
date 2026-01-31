/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "portmacro.h"
#include "rmt_uart.h"
#include "tlv320aic32x4.h"

static const char * TAG = "main";

// Pin definitions from pin_assign.md

// MIDI (制御用)
#define SDSP_TX_PIN GPIO_NUM_17
#define SDSP_RX_PIN GPIO_NUM_18
#define KDSP_TX_PIN GPIO_NUM_46
#define KDSP_RX_PIN GPIO_NUM_9

// MIDI (音源)
#define TG1A_TX_PIN GPIO_NUM_3
#define TG1B_TX_PIN GPIO_NUM_7
#define TG1_RX_PIN GPIO_NUM_15
#define TG2A_TX_PIN GPIO_NUM_16
#define TG2B_TX_PIN GPIO_NUM_5
#define TG2_RX_PIN GPIO_NUM_6

// I2C (TLV320AIC32X4用)
#define I2C_SCL_PIN GPIO_NUM_45
#define I2C_SDA_PIN GPIO_NUM_48
#define I2C_FREQ_HZ 400000

// GPIO (TLV320AIC32X4用)
#define AIC32X4_RESET_PIN GPIO_NUM_38

// UART configuration
#define UART_SDSP_NUM UART_NUM_1
#define UART_KDSP_NUM UART_NUM_2
#define UART_BUF_SIZE 1024
#define RMT_UART_BUF_SIZE 512
#define MIDI_BAUD_RATE 31250

// Audio configuration
#define MCLK_FREQ 12000000

rmt_uart_handle_t * tg1a_uart = NULL;
rmt_uart_handle_t * tg1b_uart = NULL;
rmt_uart_handle_t * tg2a_uart = NULL;
rmt_uart_handle_t * tg2b_uart = NULL;

static esp_err_t init_i2c_and_tlv320(
  i2c_master_bus_handle_t * i2c_bus, aic32x4_handle_t ** aic_handle)
{
  esp_err_t ret;

  // I2Cバスの初期化
  i2c_master_bus_config_t i2c_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = I2C_SCL_PIN,
    .sda_io_num = I2C_SDA_PIN,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
  };

  ret = i2c_new_master_bus(&i2c_bus_config, i2c_bus);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
    return ret;
  }
  ESP_LOGI(TAG, "I2C bus initialized");

  // TLV320AIC32x4の初期化
  aic32x4_config_t aic_config = {
    .i2c_bus_handle = *i2c_bus,
    .i2c_addr = 0x18,
    .mclk_freq = MCLK_FREQ,
    .sample_rate = 44100,
    .word_length = 24,
    .audio_format = AIC32X4_RIGHT_JUSTIFIED_MODE,
    .master_mode = false,  // Slave mode
    .power_cfg = AIC32X4_PWR_AIC32X4_LDO_ENABLE | AIC32X4_PWR_AVDD_DVDD_WEAK_DISABLE |
                 AIC32X4_PWR_MICBIAS_2075_LDOIN | AIC32X4_PWR_CMMODE_LDOIN_RANGE_18_36 |
                 AIC32X4_PWR_CMMODE_HP_LDOIN_POWERED,
    .micpga_routing = 0,  // Default routing
    .swap_dacs = false,
    .reset_gpio = AIC32X4_RESET_PIN,
  };

  ret = aic32x4_init(&aic_config, aic_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize TLV320AIC32x4: %s", esp_err_to_name(ret));
    return ret;
  }
  ESP_LOGI(TAG, "TLV320AIC32x4 initialized");

  return ESP_OK;
}

static esp_err_t init_control_midi_uart(void)
{
  esp_err_t ret;

  // SDSP UART configuration
  uart_config_t sdsp_uart_config = {
    .baud_rate = MIDI_BAUD_RATE,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
  };

  ret = uart_driver_install(UART_SDSP_NUM, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 0, NULL, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to install SDSP UART driver: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = uart_param_config(UART_SDSP_NUM, &sdsp_uart_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure SDSP UART: %s", esp_err_to_name(ret));
    return ret;
  }

  ret =
    uart_set_pin(UART_SDSP_NUM, SDSP_TX_PIN, SDSP_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set SDSP UART pins: %s", esp_err_to_name(ret));
    return ret;
  }
  ESP_LOGI(TAG, "SDSP UART initialized (TX: %d, RX: %d)", SDSP_TX_PIN, SDSP_RX_PIN);

  // KDSP UART configuration
  uart_config_t kdsp_uart_config = {
    .baud_rate = MIDI_BAUD_RATE,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
  };

  ret = uart_driver_install(UART_KDSP_NUM, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 0, NULL, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to install KDSP UART driver: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = uart_param_config(UART_KDSP_NUM, &kdsp_uart_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure KDSP UART: %s", esp_err_to_name(ret));
    return ret;
  }

  ret =
    uart_set_pin(UART_KDSP_NUM, KDSP_TX_PIN, KDSP_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set KDSP UART pins: %s", esp_err_to_name(ret));
    return ret;
  }
  ESP_LOGI(TAG, "KDSP UART initialized (TX: %d, RX: %d)", KDSP_TX_PIN, KDSP_RX_PIN);

  return ESP_OK;
}

static esp_err_t init_sound_source_midi_rmt_uart(void)
{
  esp_err_t ret;

  // TG1A (TX/RX)
  rmt_uart_config_t tg1a_config = {
    .baud_rate = MIDI_BAUD_RATE,
    .data_bits = RMT_UART_DATA_BITS_8,
    .parity = RMT_UART_PARITY_NONE,
    .stop_bits = RMT_UART_STOP_BITS_1,
    .tx_pin = TG1A_TX_PIN,
    .rx_pin = TG1_RX_PIN,
    .rx_buffer_size = RMT_UART_BUF_SIZE,
    .tx_buffer_size = RMT_UART_BUF_SIZE,
    .rmt_rx_symbols = 48,
    .rmt_tx_symbols = 48};

  ret = rmt_uart_init(&tg1a_config, &tg1a_uart);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize TG1A RMT UART: %s", esp_err_to_name(ret));
    return ret;
  }
  ESP_LOGI(TAG, "TG1A RMT UART initialized (TX: %d)", TG1A_TX_PIN);

  // TG1B (TX only)
  rmt_uart_config_t tg1b_config = {
    .baud_rate = MIDI_BAUD_RATE,
    .data_bits = RMT_UART_DATA_BITS_8,
    .parity = RMT_UART_PARITY_NONE,
    .stop_bits = RMT_UART_STOP_BITS_1,
    .tx_pin = TG1B_TX_PIN,
    .rx_pin = GPIO_NUM_NC,
    .tx_buffer_size = RMT_UART_BUF_SIZE,
    .rmt_rx_symbols = 48,
    .rmt_tx_symbols = 48};

  ret = rmt_uart_init(&tg1b_config, &tg1b_uart);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize TG1B RMT UART: %s", esp_err_to_name(ret));
    return ret;
  }
  ESP_LOGI(TAG, "TG1B RMT UART initialized (TX: %d)", TG1B_TX_PIN);

  // TG2A (TX/RX)
  rmt_uart_config_t tg2a_config = {
    .baud_rate = MIDI_BAUD_RATE,
    .data_bits = RMT_UART_DATA_BITS_8,
    .parity = RMT_UART_PARITY_NONE,
    .stop_bits = RMT_UART_STOP_BITS_1,
    .tx_pin = TG2A_TX_PIN,
    .rx_pin = TG2_RX_PIN,
    .rx_buffer_size = RMT_UART_BUF_SIZE,
    .tx_buffer_size = RMT_UART_BUF_SIZE,
    .rmt_rx_symbols = 48,
    .rmt_tx_symbols = 48};

  ret = rmt_uart_init(&tg2a_config, &tg2a_uart);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize TG2A RMT UART: %s", esp_err_to_name(ret));
    return ret;
  }
  ESP_LOGI(TAG, "TG2A RMT UART initialized (TX: %d)", TG2A_TX_PIN);

  // TG2B (TX only)
  rmt_uart_config_t tg2b_config = {
    .baud_rate = MIDI_BAUD_RATE,
    .data_bits = RMT_UART_DATA_BITS_8,
    .parity = RMT_UART_PARITY_NONE,
    .stop_bits = RMT_UART_STOP_BITS_1,
    .tx_pin = TG2B_TX_PIN,
    .rx_pin = GPIO_NUM_NC,
    .tx_buffer_size = RMT_UART_BUF_SIZE,
    .rmt_rx_symbols = 48,
    .rmt_tx_symbols = 48};

  ret = rmt_uart_init(&tg2b_config, &tg2b_uart);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize TG2B RMT UART: %s", esp_err_to_name(ret));
    return ret;
  }
  ESP_LOGI(TAG, "TG2B RMT UART initialized (TX: %d)", TG2B_TX_PIN);

  ESP_LOGI(TAG, "All sound source MIDI RMT UARTs initialized");

  return ESP_OK;
}

void restart()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
  ESP_LOGI(TAG, "Restarting system...");
  esp_restart();
}

void rmt_rx_thread()
{
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1));
    rmt_uart_process_rx(tg1a_uart);
    rmt_uart_process_rx(tg2a_uart);
  }
}

void app_main(void)
{
  ESP_LOGI(TAG, "Starting application...");

  i2c_master_bus_handle_t i2c_bus = NULL;
  aic32x4_handle_t * aic_handle = NULL;

  // 1. Initialize I2C and TLV320AIC32x4
  if (init_i2c_and_tlv320(&i2c_bus, &aic_handle) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize I2C and TLV320AIC32x4");
    restart();
    return;
  }

  // 2. Initialize control MIDI UARTs
  if (init_control_midi_uart() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize control MIDI UARTs");
    restart();
    return;
  }

  // 3. Initialize sound source MIDI RMT UARTs
  if (init_sound_source_midi_rmt_uart() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize sound source MIDI RMT UARTs");
    restart();
    return;
  }

  ESP_LOGI(TAG, "All peripherals initialized successfully");

  // Create RMT RX processing task
  xTaskCreatePinnedToCore(
    (TaskFunction_t)rmt_rx_thread, "rmt_rx_thread", 4096, NULL, tskIDLE_PRIORITY + 1, NULL,
    tskNO_AFFINITY);

  // Main loop
  while (1) {
    unsigned char midi_msg[] = {0xF0, 0x43, 0x30, 0x31, 0x03, 0x00, 0x00,
                                0x00, 0x00, 0x10, 0x10, 0x00, 0xF7};
    // Test: Send a MIDI SysEx message to TG1A
    int bytes_written = rmt_uart_write_array(tg1a_uart, midi_msg, sizeof(midi_msg));
    if (bytes_written < 0) {
      ESP_LOGE(TAG, "Failed to send MIDI message to TG1A");
    } else {
      ESP_LOGI(TAG, "Sent %d bytes MIDI message to TG1A", bytes_written);
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t byte;
    // Example: Read a byte from TG1A
    while (rmt_uart_read_byte(tg1a_uart, &byte)) {
      ESP_LOGI(TAG, "Received byte from TG1A: 0x%02X", byte);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
