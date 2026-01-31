/**
 * @file rmt_uart.h
 * @brief RMT-based software UART for ESP32 (ESP-IDF v6)
 */

#pragma once

#include <esp_err.h>
#include <esp_idf_version.h>
#include <stdbool.h>
#include <stdint.h>

#if ESP_IDF_VERSION_MAJOR >= 5
#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>
#else
#error "This driver requires ESP-IDF v5 or later"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define RMT_UART_DEFAULT_BAUD_RATE 9600
#define RMT_UART_TX_BUFFER_SIZE 256
#define RMT_UART_RX_BUFFER_SIZE 256
#define RMT_UART_RMT_SYMBOLS_PER_BYTE 5

// Clock configuration
#ifdef CONFIG_IDF_TARGET_ESP32H2
#define RMT_UART_CLK_FREQ 32000000
#else
#define RMT_UART_CLK_FREQ 80000000
#endif
#define RMT_UART_CLK_DIV 1

/**
 * @brief Parity mode enumeration
 */
typedef enum {
  RMT_UART_PARITY_NONE = 0,
  RMT_UART_PARITY_EVEN = 1,
  RMT_UART_PARITY_ODD = 2
} rmt_uart_parity_t;

/**
 * @brief Stop bits enumeration
 */
typedef enum { RMT_UART_STOP_BITS_1 = 1, RMT_UART_STOP_BITS_2 = 2 } rmt_uart_stop_bits_t;

/**
 * @brief Data bits enumeration
 */
typedef enum {
  RMT_UART_DATA_BITS_5 = 5,
  RMT_UART_DATA_BITS_6 = 6,
  RMT_UART_DATA_BITS_7 = 7,
  RMT_UART_DATA_BITS_8 = 8
} rmt_uart_data_bits_t;

/**
 * @brief RMT symbol half word structure for bit manipulation
 */
typedef struct
{
  uint16_t duration0 : 15;
  uint16_t level0 : 1;
} rmt_symbol_half_word_t;

/**
 * @brief Receiver component store for ISR data
 */
typedef struct
{
  volatile uint8_t * buffer;
  volatile uint8_t * partial_buffer;
  volatile uint32_t buffer_write;
  volatile uint32_t buffer_read;
  bool overflow;
  uint32_t buffer_size;
  uint32_t receive_size;
  esp_err_t error;
  rmt_receive_config_t config;
} rmt_uart_rx_store_t;

/**
 * @brief RMT UART configuration structure
 */
typedef struct
{
  gpio_num_t tx_pin;
  gpio_num_t rx_pin;
  uint32_t baud_rate;
  rmt_uart_parity_t parity;
  rmt_uart_stop_bits_t stop_bits;
  rmt_uart_data_bits_t data_bits;
  uint32_t tx_buffer_size;
  uint32_t rx_buffer_size;
  uint32_t rmt_tx_symbols;
  uint32_t rmt_rx_symbols;
} rmt_uart_config_t;

/**
 * @brief RMT UART handle structure
 */
typedef struct
{
  // Configuration
  rmt_uart_config_t config;

  // RMT handles
  rmt_channel_handle_t tx_channel;
  rmt_channel_handle_t rx_channel;
  rmt_encoder_handle_t encoder;

  // Buffers
  rmt_symbol_word_t * rmt_tx_buf;
  uint8_t * tx_buffer;
  uint8_t * rx_buffer;

  // Circular buffer indices
  volatile uint32_t tx_head;
  volatile uint32_t tx_tail;
  volatile uint32_t rx_head;
  volatile uint32_t rx_tail;

  // Receiver store
  rmt_uart_rx_store_t rx_store;

  // Baud rate timing array for each bit position
  uint16_t baud_rate_timing_array[10];

  // Error tracking
  esp_err_t last_error;
} rmt_uart_handle_t;

/**
 * @brief Get default RMT UART configuration
 *
 * @param tx_pin GPIO pin for TX
 * @param rx_pin GPIO pin for RX
 * @return Default configuration structure
 */
rmt_uart_config_t rmt_uart_get_default_config(uint8_t tx_pin, uint8_t rx_pin);

/**
 * @brief Initialize RMT UART
 *
 * @param config Configuration structure
 * @param handle Pointer to handle pointer (will be allocated)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t rmt_uart_init(const rmt_uart_config_t * config, rmt_uart_handle_t ** handle);

/**
 * @brief Deinitialize RMT UART and free resources
 *
 * @param handle UART handle
 * @return ESP_OK on success
 */
esp_err_t rmt_uart_deinit(rmt_uart_handle_t * handle);

/**
 * @brief Write a single byte to UART
 *
 * @param handle UART handle
 * @param byte Byte to write
 * @return ESP_OK on success
 */
esp_err_t rmt_uart_write_byte(rmt_uart_handle_t * handle, uint8_t byte);

/**
 * @brief Write an array of bytes to UART
 *
 * @param handle UART handle
 * @param buffer Data buffer
 * @param length Number of bytes to write
 * @return ESP_OK on success
 */
esp_err_t rmt_uart_write_array(rmt_uart_handle_t * handle, const uint8_t * buffer, size_t length);

/**
 * @brief Read a single byte from UART
 *
 * @param handle UART handle
 * @param byte Pointer to store received byte
 * @return true if byte was read, false if no data available
 */
bool rmt_uart_read_byte(rmt_uart_handle_t * handle, uint8_t * byte);

/**
 * @brief Read multiple bytes from UART
 *
 * @param handle UART handle
 * @param buffer Buffer to store data
 * @param length Number of bytes to read
 * @return true if all bytes were read, false otherwise
 */
bool rmt_uart_read_array(rmt_uart_handle_t * handle, uint8_t * buffer, size_t length);

/**
 * @brief Peek at the next byte without removing it from buffer
 *
 * @param handle UART handle
 * @param byte Pointer to store peeked byte
 * @return true if byte was peeked, false if no data available
 */
bool rmt_uart_peek_byte(rmt_uart_handle_t * handle, uint8_t * byte);

/**
 * @brief Get number of bytes available in receive buffer
 *
 * @param handle UART handle
 * @return Number of available bytes
 */
int rmt_uart_available(rmt_uart_handle_t * handle);

/**
 * @brief Flush the transmit buffer (wait for all data to be sent)
 *
 * @param handle UART handle
 * @return ESP_OK on success
 */
esp_err_t rmt_uart_flush(rmt_uart_handle_t * handle);

/**
 * @brief Process received data (should be called periodically from main loop)
 *
 * @param handle UART handle
 * @return ESP_OK on success
 */
esp_err_t rmt_uart_process_rx(rmt_uart_handle_t * handle);

/**
 * @brief Set baud rate dynamically
 *
 * @param handle UART handle
 * @param baud_rate New baud rate
 * @return ESP_OK on success
 */
esp_err_t rmt_uart_set_baud_rate(rmt_uart_handle_t * handle, uint32_t baud_rate);

#ifdef __cplusplus
}
#endif
