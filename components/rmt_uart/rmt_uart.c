#include "rmt_uart.h"

#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/queue.h"

static const char * TAG = "rmt-uart";

#define UART_TX_BUFFER_SIZE 128
#define UART_RX_BUFFER_SIZE 128

#ifdef USE_ESP32_VARIANT_ESP32H2
static const uint32_t RMT_CLK_FREQ = 32000000;
static const uint8_t RMT_CLK_DIV = 1;
#else
static const uint32_t RMT_CLK_FREQ = 80000000;
static const uint32_t RMT_CLK_DIV = 1;
#endif
#define CLOCK_HZ (RMT_CLK_FREQ / RMT_CLK_DIV)

typedef struct
{
  rmt_rx_done_event_data_t * received_symbols;
  size_t num_symbols;
} rmt_rx_event_t;

typedef struct
{
  rmt_channel_handle_t channel;
  rmt_encoder_handle_t encoder;
  rmt_symbol_word_t * symbols;
  uint32_t baud_rate_timing_array[10];
  uint8_t * tx_buffer;
  volatile size_t tx_head;
  volatile size_t tx_tail;
  size_t rmt_tx_symbols;
} rmt_uart_context_tx_t;

typedef struct
{
  rmt_channel_handle_t channel;
  rmt_receive_config_t receive_config;
  rmt_symbol_word_t * symbols;
  uint8_t * rx_buffer;
  volatile size_t rx_head;
  volatile size_t rx_tail;
  size_t rmt_rx_symbols;
  QueueHandle_t queue;

  // Ring buffer for events
  uint8_t * event_buffer;
  volatile size_t event_buffer_read;
  volatile size_t event_buffer_write;
  size_t event_buffer_size;
  size_t receive_size;
  volatile bool overflow;
  esp_err_t error;
} rmt_uart_context_rx_t;

typedef struct
{
  rmt_uart_config_t config;
  rmt_uart_context_tx_t tx_ctx;
  rmt_uart_context_rx_t rx_ctx;
  bool configured;
  uint8_t data_bits;
  uint8_t stop_bits;
} rmt_uart_context_t;

static rmt_uart_context_t rmt_uart_contexts[RMT_UART_NUM_MAX] = {0};

static void load_baud_rate_settings(rmt_uart_context_t * ctx)
{
  // Generate baud rate timing array
  uint32_t base_timing = CLOCK_HZ / ctx->config.baud_rate;
  uint32_t rest = (CLOCK_HZ % ctx->config.baud_rate) / (ctx->config.baud_rate / 10);

  for (int i = 0; i < 10; i++) {
    ctx->tx_ctx.baud_rate_timing_array[i] = base_timing + ((i < rest) ? 1 : 0);
  }

  if (ctx->stop_bits == 2) {
    ctx->tx_ctx.baud_rate_timing_array[9] = ctx->tx_ctx.baud_rate_timing_array[9] * 2;
  }
}

static bool IRAM_ATTR rmt_rx_done_callback(
  rmt_channel_handle_t channel, const rmt_rx_done_event_data_t * edata, void * user_data)
{
  BaseType_t higher_prio_woken = pdFALSE;
  rmt_uart_context_rx_t * rx_ctx = (rmt_uart_context_rx_t *)user_data;

  // Store event in ring buffer
  uint32_t event_size = sizeof(size_t) + sizeof(rmt_symbol_word_t *);
  uint32_t next_write =
    rx_ctx->event_buffer_write + event_size + edata->num_symbols * sizeof(rmt_symbol_word_t);

  if (next_write + event_size + rx_ctx->receive_size > rx_ctx->event_buffer_size) {
    next_write = 0;
  }

  if (
    (rx_ctx->event_buffer_read > rx_ctx->event_buffer_write &&
     next_write >= rx_ctx->event_buffer_read) ||
    (rx_ctx->event_buffer_read <= rx_ctx->event_buffer_write &&
     (next_write >= rx_ctx->event_buffer_read + rx_ctx->event_buffer_size))) {
    rx_ctx->overflow = true;
    return false;
  }

  // Copy event data
  uint8_t * event_ptr = rx_ctx->event_buffer + rx_ctx->event_buffer_write;
  memcpy(event_ptr, &edata->num_symbols, sizeof(size_t));
  memcpy(
    event_ptr + sizeof(size_t), edata->received_symbols,
    edata->num_symbols * sizeof(rmt_symbol_word_t));

  // Restart receive
  rx_ctx->error = rmt_receive(
    channel, (uint8_t *)rx_ctx->event_buffer + next_write + event_size, rx_ctx->receive_size,
    &rx_ctx->receive_config);

  rx_ctx->event_buffer_write = next_write;

  return higher_prio_woken == pdTRUE;
}

static void decode_rmt_rx_data(
  rmt_uart_context_t * ctx, const rmt_symbol_word_t * symbols, size_t count)
{
  rmt_uart_context_rx_t * rx_ctx = &ctx->rx_ctx;
  uint32_t min_time_bit = ctx->tx_ctx.baud_rate_timing_array[0] * 9 / 10;
  uint16_t data_bytes_mask = (1 << ctx->data_bits) - 1;
  uint16_t received_byte = 0;
  uint8_t received_bits = 0;
  uint32_t total_received_bits = 0;

  // Convert to half symbols for easier processing
  rmt_symbol_half_word_t * half_symbols = (rmt_symbol_half_word_t *)symbols;
  uint32_t count_half = count * 2;

  for (int i = 0; i < count_half; i++) {
    // Looking for start bit
    if (total_received_bits == 0) {
      if (half_symbols[i].level0 != 0 || half_symbols[i].duration0 < min_time_bit) {
        continue;  // Not a valid start bit
      }
    }

    if (half_symbols[i].duration0 == 0) {
      // Last bit before idle
      received_bits = (1 + ctx->data_bits + ctx->stop_bits) - total_received_bits;
    } else {
      received_bits = half_symbols[i].duration0 / min_time_bit;
    }

    if (half_symbols[i].level0 == 1) {
      for (uint8_t j = 0; j < received_bits; j++) {
        received_byte |= (1 << (j + total_received_bits));
      }
    }

    total_received_bits += received_bits;

    if (total_received_bits >= (1 + ctx->data_bits + ctx->stop_bits)) {
      // Extract data byte (skip start bit)
      uint8_t data_byte = (uint8_t)((received_byte >> 1) & data_bytes_mask);

      // Store in ring buffer
      rx_ctx->rx_buffer[rx_ctx->rx_tail] = data_byte;
      rx_ctx->rx_tail = (rx_ctx->rx_tail + 1) % UART_RX_BUFFER_SIZE;

      // Reset for next byte
      received_byte = 0;
      total_received_bits = 0;
    }
  }
}

static void process_tx_queue(rmt_uart_context_t * ctx)
{
  rmt_uart_context_tx_t * tx_ctx = &ctx->tx_ctx;

  if (tx_ctx->tx_head == tx_ctx->tx_tail) {
    return;  // Nothing to send
  }

  // Calculate available data
  size_t length = (tx_ctx->tx_tail - tx_ctx->tx_head + UART_TX_BUFFER_SIZE) % UART_TX_BUFFER_SIZE;

  // Calculate bits per byte
  uint8_t bits_per_byte = 1 + ctx->data_bits + ctx->stop_bits;

  // Limit by RMT buffer size
  size_t max_bytes = (tx_ctx->rmt_tx_symbols * 2) / bits_per_byte;
  if (length > max_bytes) {
    length = max_bytes;
  }

  // Build RMT symbols
  rmt_symbol_half_word_t * half_symbols = (rmt_symbol_half_word_t *)tx_ctx->symbols;

  for (size_t j = 0; j < length; j++) {
    uint8_t byte = tx_ctx->tx_buffer[(tx_ctx->tx_head + j) % UART_TX_BUFFER_SIZE];
    size_t offset = j * bits_per_byte;

    // Start bit (0)
    half_symbols[offset].duration0 = tx_ctx->baud_rate_timing_array[0];
    half_symbols[offset].level0 = 0;

    // Data bits
    for (int i = 0; i < ctx->data_bits; i++) {
      half_symbols[offset + i + 1].duration0 = tx_ctx->baud_rate_timing_array[i + 1];
      half_symbols[offset + i + 1].level0 = (byte >> i) & 1;
    }

    // Stop bit(s) (1)
    half_symbols[offset + ctx->data_bits + 1].duration0 =
      ctx->stop_bits * tx_ctx->baud_rate_timing_array[9];
    half_symbols[offset + ctx->data_bits + 1].level0 = 1;
  }

  // Calculate total symbols
  size_t total_half_symbols = length * bits_per_byte;
  size_t rmt_symbols_to_send = total_half_symbols / 2;

  // If odd number of half symbols, add padding
  if (total_half_symbols % 2 != 0) {
    half_symbols[total_half_symbols].duration0 = 1;
    half_symbols[total_half_symbols].level0 = 1;
    rmt_symbols_to_send++;
  }

  // Transmit
  rmt_transmit_config_t config = {
    .loop_count = 0,
    .flags =
      {
        .eot_level = 1,
      },
  };

  esp_err_t err = rmt_transmit(
    tx_ctx->channel, tx_ctx->encoder, tx_ctx->symbols,
    rmt_symbols_to_send * sizeof(rmt_symbol_word_t), &config);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "RMT transmit failed: %d", err);
    return;
  }

  // Update head
  tx_ctx->tx_head = (tx_ctx->tx_head + length) % UART_TX_BUFFER_SIZE;
}

esp_err_t rmt_uart_init(rmt_uart_port_t uart_num, const rmt_uart_config_t * uart_config)
{
  ESP_RETURN_ON_FALSE(uart_num < RMT_UART_NUM_MAX, ESP_ERR_INVALID_ARG, TAG, "Invalid uart_num");
  ESP_RETURN_ON_FALSE(uart_config != NULL, ESP_ERR_INVALID_ARG, TAG, "uart_config is NULL");
  ESP_RETURN_ON_FALSE(
    uart_config->baud_rate >= 1200 && uart_config->baud_rate <= 460800, ESP_ERR_INVALID_ARG, TAG,
    "Invalid baud_rate");

  rmt_uart_context_t * ctx = &rmt_uart_contexts[uart_num];

  // Copy config
  memcpy(&ctx->config, uart_config, sizeof(rmt_uart_config_t));

  // Set data bits and stop bits
  ctx->data_bits = (uart_config->data_bits == RMT_UART_DATA_8_BITS) ? 8 : 9;
  ctx->stop_bits = 1;

  // Load baud rate settings
  load_baud_rate_settings(ctx);

  const uint32_t RMT_RES_HZ = CLOCK_HZ;
  const uint32_t RMT_NS_PER_TICK = 1000000000UL / RMT_RES_HZ;

  // Initialize TX
  if (uart_config->mode == RMT_UART_MODE_TX_ONLY || uart_config->mode == RMT_UART_MODE_TX_RX) {
    rmt_uart_context_tx_t * tx_ctx = &ctx->tx_ctx;

    // Allocate TX buffer
    tx_ctx->tx_buffer = calloc(UART_TX_BUFFER_SIZE, sizeof(uint8_t));
    ESP_RETURN_ON_FALSE(tx_ctx->tx_buffer != NULL, ESP_ERR_NO_MEM, TAG, "TX buffer alloc failed");

    tx_ctx->tx_head = 0;
    tx_ctx->tx_tail = 0;

    // Calculate RMT symbols needed
    tx_ctx->rmt_tx_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL;

    // Allocate RMT symbol buffer
    tx_ctx->symbols = calloc(tx_ctx->rmt_tx_symbols, sizeof(rmt_symbol_word_t));
    ESP_RETURN_ON_FALSE(tx_ctx->symbols != NULL, ESP_ERR_NO_MEM, TAG, "TX symbols alloc failed");

    // Configure TX channel
    rmt_tx_channel_config_t tx_chan_cfg = {
      .gpio_num = uart_config->tx_io_num,
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = RMT_RES_HZ,
      .mem_block_symbols = tx_ctx->rmt_tx_symbols,
      .trans_queue_depth = 1,
      .flags =
        {
          .invert_out = 0,
        },
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_cfg, &tx_ctx->channel));

    // Create copy encoder
    rmt_copy_encoder_config_t encoder_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&encoder_cfg, &tx_ctx->encoder));

    // Enable channel
    ESP_ERROR_CHECK(rmt_enable(tx_ctx->channel));

    // Send initial idle level
    rmt_symbol_word_t idle_symbol = {
      .duration0 = 1,
      .level0 = 1,
      .duration1 = 1,
      .level1 = 1,
    };

    rmt_transmit_config_t idle_config = {
      .loop_count = 0,
      .flags.eot_level = 1,
    };

    ESP_ERROR_CHECK(rmt_transmit(
      tx_ctx->channel, tx_ctx->encoder, &idle_symbol, sizeof(idle_symbol), &idle_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(tx_ctx->channel, portMAX_DELAY));
  }

  // Initialize RX
  if (uart_config->mode == RMT_UART_MODE_RX_ONLY || uart_config->mode == RMT_UART_MODE_TX_RX) {
    rmt_uart_context_rx_t * rx_ctx = &ctx->rx_ctx;

    // Allocate RX buffer
    rx_ctx->rx_buffer = calloc(UART_RX_BUFFER_SIZE, sizeof(uint8_t));
    ESP_RETURN_ON_FALSE(rx_ctx->rx_buffer != NULL, ESP_ERR_NO_MEM, TAG, "RX buffer alloc failed");

    rx_ctx->rx_head = 0;
    rx_ctx->rx_tail = 0;

    // Calculate RMT symbols needed
    rx_ctx->rmt_rx_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL;

    // Configure RX channel
    rmt_rx_channel_config_t rx_chan_cfg = {
      .gpio_num = uart_config->rx_io_num,
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = RMT_RES_HZ,
      .mem_block_symbols = rx_ctx->rmt_rx_symbols,
      .flags =
        {
          .invert_in = 0,
        },
    };

    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_cfg, &rx_ctx->channel));

    // Enable pull-up on RX pin
    gpio_pullup_en(uart_config->rx_io_num);

    // Enable channel
    ESP_ERROR_CHECK(rmt_enable(rx_ctx->channel));

    // Configure receive settings
    uint32_t max_filter_ns = 255u * 1000 / (RMT_CLK_FREQ / 1000000);
    uint32_t max_idle_ns = 65535u * 1000;

    rx_ctx->receive_config.signal_range_min_ns =
      (9000000000u / uart_config->baud_rate < max_filter_ns) ? 9000000000u / uart_config->baud_rate
                                                             : max_filter_ns;
    rx_ctx->receive_config.signal_range_max_ns =
      (1000000000u / (uart_config->baud_rate / 10) < max_idle_ns)
        ? 1000000000u / (uart_config->baud_rate / 10)
        : max_idle_ns;

    // Allocate event buffer (ring buffer for received events)
    uint32_t event_size = sizeof(size_t);
    rx_ctx->receive_size = rx_ctx->rmt_rx_symbols * sizeof(rmt_symbol_word_t);
    rx_ctx->event_buffer_size = (event_size + rx_ctx->receive_size) * 3;
    rx_ctx->event_buffer = calloc(rx_ctx->event_buffer_size, sizeof(uint8_t));
    ESP_RETURN_ON_FALSE(
      rx_ctx->event_buffer != NULL, ESP_ERR_NO_MEM, TAG, "Event buffer alloc failed");

    rx_ctx->event_buffer_read = 0;
    rx_ctx->event_buffer_write = 0;
    rx_ctx->overflow = false;
    rx_ctx->error = ESP_OK;

    // Register callback
    rmt_rx_event_callbacks_t cbs = {
      .on_recv_done = rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_ctx->channel, &cbs, rx_ctx));

    // Start receiving
    ESP_ERROR_CHECK(rmt_receive(
      rx_ctx->channel, rx_ctx->event_buffer + event_size, rx_ctx->receive_size,
      &rx_ctx->receive_config));
  }

  ctx->configured = true;

  ESP_LOGI(
    TAG, "RMT UART %d initialized: TX=%d, RX=%d, baud=%d", uart_num, uart_config->tx_io_num,
    uart_config->rx_io_num, uart_config->baud_rate);

  return ESP_OK;
}

esp_err_t rmt_uart_write_bytes(rmt_uart_port_t uart_num, const uint8_t * data, size_t size)
{
  ESP_RETURN_ON_FALSE(uart_num < RMT_UART_NUM_MAX, ESP_ERR_INVALID_ARG, TAG, "Invalid uart_num");

  rmt_uart_context_t * ctx = &rmt_uart_contexts[uart_num];
  ESP_RETURN_ON_FALSE(ctx->configured, ESP_ERR_INVALID_STATE, TAG, "UART not configured");
  ESP_RETURN_ON_FALSE(
    ctx->config.mode != RMT_UART_MODE_RX_ONLY, ESP_ERR_INVALID_STATE, TAG, "UART is RX only");

  rmt_uart_context_tx_t * tx_ctx = &ctx->tx_ctx;

  // Check available space
  size_t available =
    UART_TX_BUFFER_SIZE -
    ((tx_ctx->tx_tail - tx_ctx->tx_head + UART_TX_BUFFER_SIZE) % UART_TX_BUFFER_SIZE);

  if (size > available - 1) {  // Keep one byte free to distinguish full from empty
    ESP_LOGW(TAG, "TX buffer full");
    return ESP_ERR_NO_MEM;
  }

  // Copy data to ring buffer
  for (size_t i = 0; i < size; i++) {
    tx_ctx->tx_buffer[tx_ctx->tx_tail] = data[i];
    tx_ctx->tx_tail = (tx_ctx->tx_tail + 1) % UART_TX_BUFFER_SIZE;
  }

  // Process TX queue
  process_tx_queue(ctx);

  return ESP_OK;
}

int rmt_uart_read_bytes(
  rmt_uart_port_t uart_num, uint8_t * buf, size_t buf_size, TickType_t timeout)
{
  ESP_RETURN_ON_FALSE(uart_num < RMT_UART_NUM_MAX, -1, TAG, "Invalid uart_num");

  rmt_uart_context_t * ctx = &rmt_uart_contexts[uart_num];
  ESP_RETURN_ON_FALSE(ctx->configured, -1, TAG, "UART not configured");
  ESP_RETURN_ON_FALSE(ctx->config.mode != RMT_UART_MODE_TX_ONLY, -1, TAG, "UART is TX only");

  rmt_uart_context_rx_t * rx_ctx = &ctx->rx_ctx;

  // Process any pending RX events
  uint32_t event_size = sizeof(size_t);
  while (rx_ctx->event_buffer_read != rx_ctx->event_buffer_write) {
    uint8_t * event_ptr = rx_ctx->event_buffer + rx_ctx->event_buffer_read;
    size_t num_symbols;
    memcpy(&num_symbols, event_ptr, sizeof(size_t));

    rmt_symbol_word_t * symbols = (rmt_symbol_word_t *)(event_ptr + sizeof(size_t));
    decode_rmt_rx_data(ctx, symbols, num_symbols);

    uint32_t next_read =
      rx_ctx->event_buffer_read + event_size + num_symbols * sizeof(rmt_symbol_word_t);
    if (next_read + event_size + rx_ctx->receive_size > rx_ctx->event_buffer_size) {
      next_read = 0;
    }
    rx_ctx->event_buffer_read = next_read;
  }

  // Read from RX buffer
  size_t bytes_read = 0;
  TickType_t start_time = xTaskGetTickCount();

  while (bytes_read < buf_size) {
    if (rx_ctx->rx_head != rx_ctx->rx_tail) {
      buf[bytes_read] = rx_ctx->rx_buffer[rx_ctx->rx_head];
      rx_ctx->rx_head = (rx_ctx->rx_head + 1) % UART_RX_BUFFER_SIZE;
      bytes_read++;
    } else {
      // Check timeout
      if (timeout != portMAX_DELAY && (xTaskGetTickCount() - start_time) >= timeout) {
        break;
      }
      vTaskDelay(1);
    }
  }

  return bytes_read;
}

esp_err_t rmt_uart_deinit(rmt_uart_port_t uart_num)
{
  ESP_RETURN_ON_FALSE(uart_num < RMT_UART_NUM_MAX, ESP_ERR_INVALID_ARG, TAG, "Invalid uart_num");

  rmt_uart_context_t * ctx = &rmt_uart_contexts[uart_num];
  ESP_RETURN_ON_FALSE(ctx->configured, ESP_ERR_INVALID_STATE, TAG, "UART not configured");

  // Cleanup TX
  if (ctx->config.mode != RMT_UART_MODE_RX_ONLY) {
    rmt_uart_context_tx_t * tx_ctx = &ctx->tx_ctx;

    if (tx_ctx->channel != NULL) {
      rmt_disable(tx_ctx->channel);
      rmt_del_channel(tx_ctx->channel);
      tx_ctx->channel = NULL;
    }

    if (tx_ctx->encoder != NULL) {
      rmt_del_encoder(tx_ctx->encoder);
      tx_ctx->encoder = NULL;
    }

    if (tx_ctx->symbols != NULL) {
      free(tx_ctx->symbols);
      tx_ctx->symbols = NULL;
    }

    if (tx_ctx->tx_buffer != NULL) {
      free(tx_ctx->tx_buffer);
      tx_ctx->tx_buffer = NULL;
    }
  }

  // Cleanup RX
  if (ctx->config.mode != RMT_UART_MODE_TX_ONLY) {
    rmt_uart_context_rx_t * rx_ctx = &ctx->rx_ctx;

    if (rx_ctx->channel != NULL) {
      rmt_disable(rx_ctx->channel);
      rmt_del_channel(rx_ctx->channel);
      rx_ctx->channel = NULL;
    }

    if (rx_ctx->event_buffer != NULL) {
      free(rx_ctx->event_buffer);
      rx_ctx->event_buffer = NULL;
    }

    if (rx_ctx->rx_buffer != NULL) {
      free(rx_ctx->rx_buffer);
      rx_ctx->rx_buffer = NULL;
    }
  }

  ctx->configured = false;

  ESP_LOGI(TAG, "RMT UART %d deinitialized", uart_num);

  return ESP_OK;
}
