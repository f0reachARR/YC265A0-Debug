/**
 * @file rmt_uart.c
 * @brief RMT-based software UART implementation for ESP32 (ESP-IDF v6)
 */

#include "rmt_uart.h"

#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_log.h>
#include <memory.h>
#include <stdlib.h>
#include <string.h>

#include "driver/rmt_types.h"
#include "esp_check.h"

static const char * TAG = "rmt_uart";

#define CLOCK_HZ (RMT_UART_CLK_FREQ / RMT_UART_CLK_DIV)
#define RMT_RX_DONE_EVENT_SIZE (sizeof(rmt_rx_done_event_data_t))

// Forward declarations
static void rmt_uart_load_settings(rmt_uart_handle_t * handle);
static esp_err_t rmt_uart_process_tx_queue(rmt_uart_handle_t * handle);
static void rmt_uart_put_rx_byte(rmt_uart_handle_t * handle, uint8_t byte);
static void rmt_uart_decode_rx_data(
  rmt_uart_handle_t * handle, const rmt_symbol_word_t * symbols, int count);

/**
 * @brief RMT RX callback (called from ISR context)
 */
static bool IRAM_ATTR
rmt_rx_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t * event, void * arg)
{
  rmt_uart_handle_t * handle = (rmt_uart_handle_t *)arg;
  rmt_uart_rx_store_t * store = &handle->rx_store;
  rmt_rx_done_event_data_t * combined_event_data =
    (rmt_rx_done_event_data_t *)(store->buffer + store->buffer_write);
  uint8_t * received_symbols =
    (uint8_t *)(store->buffer + store->buffer_write + RMT_RX_DONE_EVENT_SIZE);

  // Copy event data
  uint8_t * received_symbols_last =
    received_symbols + combined_event_data->num_symbols * sizeof(rmt_symbol_word_t);
  memcpy(
    received_symbols_last, event->received_symbols, event->num_symbols * sizeof(rmt_symbol_word_t));
  combined_event_data->num_symbols += event->num_symbols;
  combined_event_data->received_symbols = (rmt_symbol_word_t *)received_symbols;

  if (event->flags.is_last) {
    uint32_t next_write = store->buffer_write + RMT_RX_DONE_EVENT_SIZE +
                          combined_event_data->num_symbols * sizeof(rmt_symbol_word_t);

    // Handle buffer wraparound
    if (next_write + RMT_RX_DONE_EVENT_SIZE + store->receive_size > store->buffer_size) {
      next_write = 0;
    }

    // Check for buffer overflow
    if (store->buffer_read - next_write < RMT_RX_DONE_EVENT_SIZE + store->receive_size) {
      next_write = store->buffer_write;
      store->overflow = true;
    }
    store->buffer_write = next_write;

    rmt_rx_done_event_data_t * next_event_data =
      (rmt_rx_done_event_data_t *)(store->buffer + store->buffer_write);
    next_event_data->num_symbols = 0;

    // Start next receive operation
    store->error =
      rmt_receive(channel, (uint8_t *)store->partial_buffer, store->receive_size, &store->config);
  }

  return false;
}

/**
 * @brief Get default configuration
 */
rmt_uart_config_t rmt_uart_get_default_config(uint8_t tx_pin, uint8_t rx_pin)
{
  rmt_uart_config_t config = {
    .tx_pin = tx_pin,
    .rx_pin = rx_pin,
    .baud_rate = RMT_UART_DEFAULT_BAUD_RATE,
    .parity = RMT_UART_PARITY_NONE,
    .stop_bits = RMT_UART_STOP_BITS_1,
    .data_bits = RMT_UART_DATA_BITS_8,
    .tx_buffer_size = RMT_UART_TX_BUFFER_SIZE,
    .rx_buffer_size = RMT_UART_RX_BUFFER_SIZE,
    .rmt_tx_symbols = 64,
    .rmt_rx_symbols = 64};
  return config;
}

/**
 * @brief Load baud rate timing settings
 */
static void rmt_uart_load_settings(rmt_uart_handle_t * handle)
{
  // Generate baud rate timing array
  uint32_t base_timing = CLOCK_HZ / handle->config.baud_rate;
  uint32_t rest = (CLOCK_HZ % handle->config.baud_rate) / (handle->config.baud_rate / 10);

  for (int i = 0; i < 10; i++) {
    handle->baud_rate_timing_array[i] = base_timing + ((i < rest) ? 1 : 0);
  }

  // Adjust stop bit timing for 2 stop bits
  if (handle->config.stop_bits == RMT_UART_STOP_BITS_2) {
    handle->baud_rate_timing_array[9] = handle->baud_rate_timing_array[9] * 2;
  }
}

/**
 * @brief Initialize RMT UART
 */
esp_err_t rmt_uart_init(const rmt_uart_config_t * config, rmt_uart_handle_t ** handle)
{
  if (!config || !handle) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t ret;

  // Allocate handle
  *handle = (rmt_uart_handle_t *)calloc(1, sizeof(rmt_uart_handle_t));
  ESP_RETURN_ON_FALSE(*handle, ESP_ERR_NO_MEM, TAG, "Failed to allocate RMT UART handle");

  rmt_uart_handle_t * h = *handle;
  memcpy(&h->config, config, sizeof(rmt_uart_config_t));

  ESP_LOGI(
    TAG, "Initializing RMT UART: TX=%d, RX=%d, Baud=%lu", config->tx_pin, config->rx_pin,
    config->baud_rate);

  if (config->tx_pin != GPIO_NUM_NC) {
    ESP_RETURN_ON_FALSE(
      config->tx_buffer_size > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid TX buffer size");
    ESP_RETURN_ON_FALSE(
      config->rmt_tx_symbols > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid RMT TX symbols");

    // Allocate TX buffer
    h->tx_buffer = (uint8_t *)calloc(config->tx_buffer_size, sizeof(uint8_t));
    if (!h->tx_buffer) {
      ESP_LOGE(TAG, "Failed to allocate TX buffer");
      rmt_uart_deinit(h);
      return ESP_ERR_NO_MEM;
    }

    // Allocate RMT TX buffer (10 bits per byte + 1 for reset)
    h->rmt_tx_buf =
      (rmt_symbol_word_t *)calloc(config->tx_buffer_size * 10 + 1, sizeof(rmt_symbol_word_t));
    if (!h->rmt_tx_buf) {
      ESP_LOGE(TAG, "Failed to allocate RMT TX buffer");
      rmt_uart_deinit(h);
      return ESP_ERR_NO_MEM;
    }

    // Configure TX channel
    rmt_tx_channel_config_t tx_channel = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = RMT_UART_CLK_FREQ / RMT_UART_CLK_DIV,
      .gpio_num = config->tx_pin,
      .mem_block_symbols = config->rmt_tx_symbols,
      .trans_queue_depth = 1,
      .flags =
        {// .io_loop_back = 0,
         // .io_od_mode = 0,
         .invert_out = 0,
         .with_dma = 0},
      .intr_priority = 0};

    ret = rmt_new_tx_channel(&tx_channel, &h->tx_channel);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to create TX channel: %s", esp_err_to_name(ret));
      rmt_uart_deinit(h);
      return ret;
    }

    // Create encoder
    rmt_copy_encoder_config_t encoder = {};
    ret = rmt_new_copy_encoder(&encoder, &h->encoder);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to create encoder: %s", esp_err_to_name(ret));
      rmt_uart_deinit(h);
      return ret;
    }

    // Enable TX channel
    ret = rmt_enable(h->tx_channel);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to enable TX channel: %s", esp_err_to_name(ret));
      rmt_uart_deinit(h);
      return ret;
    }

    // Send initial start bit for correct end-of-transmission level
    rmt_symbol_word_t * symbols = h->rmt_tx_buf;
    symbols[0].val = 0;
    symbols[0].duration0 = 1;
    symbols[0].level0 = 1;
    symbols[0].duration1 = 1;
    symbols[0].level1 = 1;

    rmt_transmit_config_t tx_config = {.loop_count = 0, .flags = {.eot_level = 1}};

    ret = rmt_transmit(h->tx_channel, h->encoder, symbols, sizeof(rmt_symbol_word_t), &tx_config);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Initial TX failed: %s", esp_err_to_name(ret));
      rmt_uart_deinit(h);
      return ret;
    }
  }

  if (config->rx_pin != GPIO_NUM_NC) {
    ESP_RETURN_ON_FALSE(
      config->rx_buffer_size > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid RX buffer size");
    ESP_RETURN_ON_FALSE(
      config->rmt_rx_symbols > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid RMT RX symbols");

    // Allocate RX buffer
    h->rx_buffer = (uint8_t *)calloc(config->rx_buffer_size, sizeof(uint8_t));
    if (!h->rx_buffer) {
      ESP_LOGE(TAG, "Failed to allocate RX buffer");
      rmt_uart_deinit(h);
      return ESP_ERR_NO_MEM;
    }

    // Configure RX channel
    rmt_rx_channel_config_t rx_channel = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = RMT_UART_CLK_FREQ / RMT_UART_CLK_DIV,
      .mem_block_symbols = config->rmt_rx_symbols,
      .gpio_num = config->rx_pin,
      .intr_priority = 0,
      .flags = {.invert_in = 0, .with_dma = 0}};

    ret = rmt_new_rx_channel(&rx_channel, &h->rx_channel);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to create RX channel: %s", esp_err_to_name(ret));
      rmt_uart_deinit(h);
      return ret;
    }

    // Enable pullup on RX pin
    gpio_pullup_en((gpio_num_t)config->rx_pin);

    // Enable RX channel
    ret = rmt_enable(h->rx_channel);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to enable RX channel: %s", esp_err_to_name(ret));
      rmt_uart_deinit(h);
      return ret;
    }

    // Register RX callback
    rmt_rx_event_callbacks_t callbacks = {.on_recv_done = rmt_rx_callback};

    ret = rmt_rx_register_event_callbacks(h->rx_channel, &callbacks, h);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to register RX callbacks: %s", esp_err_to_name(ret));
      rmt_uart_deinit(h);
      return ret;
    }

    // Configure RX store
    uint32_t max_filter_ns = 255u * 1000 / (RMT_UART_CLK_FREQ / 1000000);
    uint32_t max_idle_ns = 65535u * 1000;

    h->rx_store.config.signal_range_min_ns = (9000000000u / config->baud_rate < max_filter_ns)
                                               ? 9000000000u / config->baud_rate
                                               : max_filter_ns;
    h->rx_store.config.signal_range_max_ns = (1000000000u / (config->baud_rate / 10) < max_idle_ns)
                                               ? 1000000000u / (config->baud_rate / 10)
                                               : max_idle_ns;
    h->rx_store.config.flags.en_partial_rx = 1;

    h->rx_store.receive_size = config->rmt_rx_symbols * sizeof(rmt_symbol_word_t);
    h->rx_store.buffer_size = (RMT_RX_DONE_EVENT_SIZE + h->rx_store.receive_size) * 8;
    h->rx_store.buffer = (uint8_t *)calloc(h->rx_store.buffer_size, sizeof(uint8_t));
    h->rx_store.partial_buffer = (uint8_t *)calloc(h->rx_store.receive_size, sizeof(uint8_t));

    if (!h->rx_store.buffer) {
      ESP_LOGE(TAG, "Failed to allocate RX store buffer");
      rmt_uart_deinit(h);
      return ESP_ERR_NO_MEM;
    }

    if (!h->rx_store.partial_buffer) {
      ESP_LOGE(TAG, "Failed to allocate RX store partial buffer");
      rmt_uart_deinit(h);
      return ESP_ERR_NO_MEM;
    }

    // Start initial receive
    ret = rmt_receive(
      h->rx_channel, (uint8_t *)h->rx_store.partial_buffer, h->rx_store.receive_size,
      &h->rx_store.config);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to start receive: %s", esp_err_to_name(ret));
      rmt_uart_deinit(h);
      return ret;
    }
  }

  // Load baud rate settings
  rmt_uart_load_settings(h);

  ESP_LOGI(TAG, "RMT UART initialized successfully");
  return ESP_OK;
}

/**
 * @brief Deinitialize RMT UART
 */
esp_err_t rmt_uart_deinit(rmt_uart_handle_t * handle)
{
  if (!handle) {
    return ESP_ERR_INVALID_ARG;
  }

  if (handle->tx_channel) {
    rmt_disable(handle->tx_channel);
    rmt_del_channel(handle->tx_channel);
  }

  if (handle->rx_channel) {
    rmt_disable(handle->rx_channel);
    rmt_del_channel(handle->rx_channel);
  }

  if (handle->encoder) {
    rmt_del_encoder(handle->encoder);
  }

  if (handle->rmt_tx_buf) {
    free(handle->rmt_tx_buf);
  }

  if (handle->tx_buffer) {
    free(handle->tx_buffer);
  }

  if (handle->rx_buffer) {
    free(handle->rx_buffer);
  }

  if (handle->rx_store.buffer) {
    free((void *)handle->rx_store.buffer);
  }

  free(handle);

  ESP_LOGI(TAG, "RMT UART deinitialized");
  return ESP_OK;
}

/**
 * @brief Process TX queue
 */
static esp_err_t rmt_uart_process_tx_queue(rmt_uart_handle_t * handle)
{
  if (handle->tx_head == handle->tx_tail) {
    return ESP_OK;
  }

  int length = (handle->tx_tail - handle->tx_head + handle->config.tx_buffer_size) %
               handle->config.tx_buffer_size;

  if (!handle->rmt_tx_buf) {
    ESP_LOGE(TAG, "RMT TX buffer is null");
    return ESP_ERR_INVALID_STATE;
  }

  uint8_t rmt_bits_per_byte = 1 +  // start bit
                              handle->config.data_bits +
                              (handle->config.parity != RMT_UART_PARITY_NONE ? 1 : 0) +
                              1;  // stop bit

  // Limit length based on available RMT symbols
  if (handle->config.rmt_tx_symbols <= (length * rmt_bits_per_byte) / 2) {
    length = (handle->config.rmt_tx_symbols / rmt_bits_per_byte) * 2;
  }

  rmt_symbol_half_word_t * half_symbols = (rmt_symbol_half_word_t *)handle->rmt_tx_buf;

  for (int j = 0; j < length; j++) {
    uint8_t byte = handle->tx_buffer[(handle->tx_head + j) % handle->config.tx_buffer_size];
    bool parity_bit = (handle->config.parity != RMT_UART_PARITY_NONE) ? __builtin_parity(byte) : 0;

    // Start bit
    half_symbols[j * rmt_bits_per_byte].duration0 = handle->baud_rate_timing_array[0];
    half_symbols[j * rmt_bits_per_byte].level0 = 0;

    // Data bits
    for (int i = 0; i < handle->config.data_bits; i++) {
      half_symbols[j * rmt_bits_per_byte + i + 1].duration0 = handle->baud_rate_timing_array[i + 1];
      half_symbols[j * rmt_bits_per_byte + i + 1].level0 = (byte >> i) & 1;
    }

    // Parity bit
    if (handle->config.parity != RMT_UART_PARITY_NONE) {
      half_symbols[j * rmt_bits_per_byte + handle->config.data_bits + 1].duration0 =
        handle->baud_rate_timing_array[9];
      half_symbols[j * rmt_bits_per_byte + handle->config.data_bits + 1].level0 =
        (handle->config.parity == RMT_UART_PARITY_EVEN) ? (parity_bit & 1) : ((parity_bit ^ 1) & 1);
    }

    // Stop bit
    half_symbols[j * rmt_bits_per_byte + rmt_bits_per_byte - 1].duration0 =
      handle->config.stop_bits * handle->baud_rate_timing_array[9];
    half_symbols[j * rmt_bits_per_byte + rmt_bits_per_byte - 1].level0 = 1;
  }

  uint8_t rmt_symbols_to_send = (length * rmt_bits_per_byte) / 2;
  if ((length * rmt_bits_per_byte) % 2 != 0) {
    half_symbols[length * rmt_bits_per_byte].duration0 = 1;
    half_symbols[length * rmt_bits_per_byte].level0 = 1;
    rmt_symbols_to_send++;
  }

  rmt_transmit_config_t config = {.loop_count = 0, .flags = {.eot_level = 1}};

  esp_err_t ret = rmt_transmit(
    handle->tx_channel, handle->encoder, half_symbols,
    rmt_symbols_to_send * sizeof(rmt_symbol_word_t), &config);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "RMT TX error: %s", esp_err_to_name(ret));
    return ret;
  }

  handle->tx_head = (handle->tx_head + length) % handle->config.tx_buffer_size;

  if (handle->tx_head != handle->tx_tail) {
    // More data to send
    return rmt_uart_process_tx_queue(handle);
  }
  return ESP_OK;
}

/**
 * @brief Put received byte into RX buffer
 */
static void rmt_uart_put_rx_byte(rmt_uart_handle_t * handle, uint8_t byte)
{
  handle->rx_buffer[handle->rx_tail] = byte;
  handle->rx_tail = (handle->rx_tail + 1) % handle->config.rx_buffer_size;
}

/**
 * @brief Decode RMT RX data
 */
static void rmt_uart_decode_rx_data(
  rmt_uart_handle_t * handle, const rmt_symbol_word_t * symbols, int count)
{
  uint32_t min_time_bit = handle->baud_rate_timing_array[0] * 9 / 10;
  uint16_t data_bytes_mask = (1 << handle->config.data_bits) - 1;
  uint16_t received_byte = 0;
  uint8_t received_bits = 0;
  uint32_t total_received_bits = 0;
  rmt_symbol_half_word_t * half_symbols = (rmt_symbol_half_word_t *)symbols;
  uint32_t count_half = count * 2;

  for (int i = 0; i < count_half; i++) {
    // Look for start bit
    if (total_received_bits == 0 && (half_symbols[i].level0 != 0)) {
      continue;
    }

    if (total_received_bits == 0 && (half_symbols[i].duration0 < min_time_bit)) {
      continue;
    }

    if (half_symbols[i].duration0 == 0) {
      // Last bit before idle
      received_bits = (1 + handle->config.data_bits + handle->config.stop_bits +
                       (handle->config.parity != RMT_UART_PARITY_NONE ? 1 : 0)) -
                      total_received_bits;
    } else {
      received_bits = half_symbols[i].duration0 / min_time_bit;
    }

    // Set bits to received_byte
    if (half_symbols[i].level0 == 1) {
      for (uint8_t j = 0; j < received_bits; j++) {
        received_byte |= (1 << (j + total_received_bits));
      }
    }

    total_received_bits += received_bits;

    // Check if we received a complete byte
    if (
      total_received_bits >= (1 + handle->config.data_bits + handle->config.stop_bits +
                              (handle->config.parity != RMT_UART_PARITY_NONE ? 1 : 0))) {
      if (handle->config.parity != RMT_UART_PARITY_NONE) {
        bool parity_bit = (received_byte >> (handle->config.data_bits + 1)) & 1;
        uint8_t data_byte = (uint8_t)((received_byte >> 1) & data_bytes_mask);
        bool calculated_parity = __builtin_parity(data_byte);

        if (
          (handle->config.parity == RMT_UART_PARITY_EVEN && calculated_parity != parity_bit) ||
          (handle->config.parity == RMT_UART_PARITY_ODD && calculated_parity == parity_bit)) {
          ESP_LOGW(TAG, "Parity error detected");
        } else {
          rmt_uart_put_rx_byte(handle, data_byte);
        }
      } else {
        rmt_uart_put_rx_byte(handle, (uint8_t)((received_byte >> 1) & data_bytes_mask));
      }

      received_byte = 0;
      total_received_bits = 0;
    }
  }
}

/**
 * @brief Write a single byte
 */
esp_err_t rmt_uart_write_byte(rmt_uart_handle_t * handle, uint8_t byte)
{
  if (!handle) {
    return ESP_ERR_INVALID_ARG;
  }

  handle->tx_buffer[handle->tx_tail] = byte;
  handle->tx_tail = (handle->tx_tail + 1) % handle->config.tx_buffer_size;
  return rmt_uart_process_tx_queue(handle);
}

/**
 * @brief Write an array of bytes
 */
esp_err_t rmt_uart_write_array(rmt_uart_handle_t * handle, const uint8_t * buffer, size_t length)
{
  if (!handle || !buffer) {
    return ESP_ERR_INVALID_ARG;
  }

  size_t available_space = handle->config.tx_buffer_size -
                           ((handle->tx_tail - handle->tx_head + handle->config.tx_buffer_size) %
                            handle->config.tx_buffer_size);

  if (length > available_space) {
    ESP_LOGE(TAG, "Not enough space in TX buffer");
    return ESP_ERR_NO_MEM;
  }

  size_t first_chunk = (length < (handle->config.tx_buffer_size - handle->tx_tail))
                         ? length
                         : (handle->config.tx_buffer_size - handle->tx_tail);

  memcpy(&handle->tx_buffer[handle->tx_tail], buffer, first_chunk);
  memcpy(&handle->tx_buffer[0], buffer + first_chunk, length - first_chunk);

  handle->tx_tail = (handle->tx_tail + length) % handle->config.tx_buffer_size;
  return rmt_uart_process_tx_queue(handle);
}

/**
 * @brief Read a single byte
 */
bool rmt_uart_read_byte(rmt_uart_handle_t * handle, uint8_t * byte)
{
  if (!handle || !byte) {
    return false;
  }

  if (handle->rx_head == handle->rx_tail) {
    return false;
  }

  *byte = handle->rx_buffer[handle->rx_head];
  handle->rx_head = (handle->rx_head + 1) % handle->config.rx_buffer_size;
  return true;
}

/**
 * @brief Read multiple bytes
 */
bool rmt_uart_read_array(rmt_uart_handle_t * handle, uint8_t * buffer, size_t length)
{
  if (!handle || !buffer) {
    return false;
  }

  for (size_t i = 0; i < length; i++) {
    if (!rmt_uart_read_byte(handle, &buffer[i])) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Peek at next byte
 */
bool rmt_uart_peek_byte(rmt_uart_handle_t * handle, uint8_t * byte)
{
  if (!handle || !byte) {
    return false;
  }

  if (handle->rx_head == handle->rx_tail) {
    return false;
  }

  *byte = handle->rx_buffer[handle->rx_head];
  return true;
}

/**
 * @brief Get number of available bytes
 */
int rmt_uart_available(rmt_uart_handle_t * handle)
{
  if (!handle) {
    return 0;
  }

  return (handle->rx_tail - handle->rx_head + handle->config.rx_buffer_size) %
         handle->config.rx_buffer_size;
}

/**
 * @brief Flush TX buffer
 */
esp_err_t rmt_uart_flush(rmt_uart_handle_t * handle)
{
  if (!handle) {
    return ESP_ERR_INVALID_ARG;
  }

  while (handle->tx_head != handle->tx_tail) {
    esp_err_t ret = rmt_uart_process_tx_queue(handle);
    if (ret != ESP_OK) {
      return ret;
    }
  }

  return ESP_OK;
}

/**
 * @brief Process RX data (call from main loop)
 */
esp_err_t rmt_uart_process_rx(rmt_uart_handle_t * handle)
{
  if (!handle) {
    return ESP_ERR_INVALID_ARG;
  }

  if (handle->rx_store.error != ESP_OK) {
    ESP_LOGE(TAG, "RX error: %s", esp_err_to_name(handle->rx_store.error));
    return handle->rx_store.error;
  }

  if (handle->rx_store.overflow) {
    ESP_LOGW(
      TAG, "Buffer overflow: read=%lu write=%lu", handle->rx_store.buffer_read,
      handle->rx_store.buffer_write);
    handle->rx_store.overflow = false;
  }

  uint32_t buffer_write = handle->rx_store.buffer_write;
  while (handle->rx_store.buffer_read != buffer_write) {
    rmt_rx_done_event_data_t * event =
      (rmt_rx_done_event_data_t *)(handle->rx_store.buffer + handle->rx_store.buffer_read);
    uint32_t next_read = handle->rx_store.buffer_read + RMT_RX_DONE_EVENT_SIZE +
                         event->num_symbols * sizeof(rmt_symbol_word_t);

    if (
      next_read + RMT_RX_DONE_EVENT_SIZE + handle->rx_store.receive_size >
      handle->rx_store.buffer_size) {
      next_read = 0;
    }

    rmt_uart_decode_rx_data(handle, event->received_symbols, event->num_symbols);
    handle->rx_store.buffer_read = next_read;
  }

  return ESP_OK;
}

/**
 * @brief Set baud rate
 */
esp_err_t rmt_uart_set_baud_rate(rmt_uart_handle_t * handle, uint32_t baud_rate)
{
  if (!handle) {
    return ESP_ERR_INVALID_ARG;
  }

  handle->config.baud_rate = baud_rate;
  rmt_uart_load_settings(handle);
  return ESP_OK;
}
