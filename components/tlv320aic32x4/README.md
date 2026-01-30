# TLV320AIC32x4 Audio Codec Driver for ESP-IDF

TI TLV320AIC32x4オーディオコーデック用のESP-IDFドライバ

## 概要

このドライバは、LinuxカーネルのTLV320AIC32x4ドライバをESP-IDF環境に移植したものです。

## 機能

- I2C通信によるレジスタアクセス
- PLLとクロック設定の完全サポート
- DAC/ADC設定
- 複数のオーディオフォーマットサポート (I2S, DSP, Left/Right Justified)
- サンプルレート設定 (8kHz - 48kHz)
- ビット深度設定 (16/20/24/32ビット)
- マスター/スレーブモード
- 音量制御とミュート機能

## サポートされているサンプルレート

- 8 kHz
- 11.025 kHz
- 16 kHz
- 22.05 kHz
- 32 kHz
- 44.1 kHz
- 48 kHz

## サポートされているマスタークロック

- 12 MHz
- 24 MHz
- 25 MHz

## 使用例

```c
#include "tlv320aic32x4.h"
#include "driver/i2c_master.h"

// I2Cマスターバスを初期化 (ESP-IDF v6)
i2c_master_bus_handle_t i2c_bus_handle;
i2c_master_bus_config_t i2c_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = GPIO_NUM_22,
    .sda_io_num = GPIO_NUM_21,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};
i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);

// コーデックの設定
aic32x4_config_t codec_config = {
    .i2c_bus_handle = i2c_bus_handle,
    .i2c_addr = 0x18,              // デフォルトI2Cアドレス
    .mclk_freq = AIC32X4_FREQ_12000000,
    .sample_rate = 48000,
    .word_length = 16,
    .audio_format = AIC32X4_I2S_MODE,
    .master_mode = false,          // ESP32がマスター
    .power_cfg = 0,
    .micpga_routing = 0,
    .swap_dacs = false,
    .reset_gpio = -1,              // リセットGPIOを使用しない場合
};

aic32x4_handle_t *codec_handle;
esp_err_t ret = aic32x4_init(&codec_config, &codec_handle);
if (ret != ESP_OK) {
    // エラー処理
}

// 音量設定 (0-63, 低いほど大きい音量)
aic32x4_set_dac_volume(codec_handle, 10, 10);

// ミュート解除
aic32x4_set_mute(codec_handle, false);
```

## I2Cアドレス

デフォルトのI2Cアドレスは `0x18` です。ハードウェア設定により異なる場合があります。

## ハードウェア接続

| ESP32 | TLV320AIC32x4 |
|-------|---------------|
| SDA   | SDA           |
| SCL   | SCL           |
| MCLK  | MCLK          |
| BCLK  | BCLK          |
| WCLK  | WCLK (LRCLK)  |
| DOUT  | DIN (DAC)     |
| DIN   | DOUT (ADC)    |
| GPIO  | RESET (optional) |

## ライセンス

GPL v2 (元のLinuxドライバと同じ)

## 参考資料

- [TLV320AIC32x4 データシート](https://www.ti.com/product/TLV320AIC3254)
- 元のLinuxカーネルドライバ: sound/soc/codecs/tlv320aic32x4.c
