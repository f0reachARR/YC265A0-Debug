# ESP32 RMT UART

Component for Espressif ESP32 ESP-IDF framework.

This component uses ESP32's RMT peripheral as an UART port. It can send and receive UART frames as well.

## Supported versions of frameworks and devices

| Chip      | Framework          | Versions |   Number of UART
|-----------|--------------------|----------|----------
| ESP32-C6  | ESP-IDF            | v5.x     |   2
| ESP32-S2* | ESP-IDF            | v5.x     |   2
| ESP32-C3* | ESP-IDF            | v5.x     |   2
| ESP32-S3* | ESP-IDF            | v5.x     |   4

* Untested.

## How to Use

Clone this repository to your project components directory.

## Configuration

```c
typedef struct {
    int baud_rate;                        /*!< UART baud rate*/
    rmt_uart_mode_t mode;                 /*!< UART mode*/  
    rmt_uart_word_length_t data_bits;     /*!< UART byte size*/
    rmt_uart_parity_t parity;             /*!< UART parity mode*/
    rmt_uart_stop_bits_t stop_bits;       /*!< UART stop bits*/
    gpio_num_t tx_io_num;                 /*!< Transmit I/O pin number*/
    gpio_num_t rx_io_num;                 /*!< Receive I/O pin number*/
    size_t buffer_size;                   /*!< Size of buffer for symbols*/
} rmt_uart_config_t;
```

Mode can be TX only, RX only or both TX and RX.  
Buffer size must be 5 times of the length of transmit/receive data.
If you want to send 10 bytes maximum then buffer_size = 50.
This is because every symbol has information about a low to high transistion.
1 byte with 8N1 configuration can have 5 such transistions.

## Restrictions

Baud rate is limited to range 4800 .. 460800. Data format is limited to 8N1. Preparations for parity bit support are done, but support is not fully implemented. Currently only 9600 baud and 8N1 are tested. Patches are very welcome!

Due to hardware limitations ESP32-S2 can only receive 12 bytes at once. In RX only mode this limit is 24 bytes. Transmit has no restriction.
