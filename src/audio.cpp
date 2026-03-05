#include "audio.h"
#include <driver/i2s.h>

// ─── Double Buffer ────────────────────────────────────────────────────────────
// Buffer A and B — DMA fills one while FFT reads the other
static int16_t dmaFillBuf[DOUBLE_BUF_SIZE];   // DMA writes here
int16_t        audioProcessBuffer[DOUBLE_BUF_SIZE]; // FFT reads here
volatile bool  audioBufferReady = false;

static size_t  fillPos = 0; // Current fill position in dmaFillBuf

// ─── I2S Init ─────────────────────────────────────────────────────────────────
bool audioInit() {
    i2s_config_t i2s_config = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate          = SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT, // most MEMS mics output 32-bit frames
        .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = DMA_BUF_COUNT,
        .dma_buf_len          = DMA_BUF_LEN,
        .use_apll             = false,
        .tx_desc_auto_clear   = false,
        .fixed_mclk           = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num   = I2S_SCK_PIN,
        .ws_io_num    = I2S_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num  = I2S_SD_PIN
    };

    esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    if (err != ESP_OK) return false;

    err = i2s_set_pin(I2S_NUM_0, &pin_config);
    if (err != ESP_OK) return false;

    i2s_zero_dma_buffer(I2S_NUM_0);
    return true;
}

// ─── Audio Update — call from loop() ─────────────────────────────────────────
// Reads available DMA samples into the fill buffer.
// When DOUBLE_BUF_SIZE samples are collected, swaps to the process buffer
// and raises audioBufferReady.
void audioUpdate() {
    if (audioBufferReady) return; // FFT hasn't consumed last buffer yet

    // Temporary raw DMA read buffer (32-bit samples from I2S)
    static int32_t rawBuf[DMA_BUF_LEN];
    size_t bytesRead = 0;

    // Non-blocking read — take whatever DMA has available
    esp_err_t err = i2s_read(I2S_NUM_0,
                              rawBuf,
                              sizeof(rawBuf),
                              &bytesRead,
                              0);                // 0 ticks = non-blocking

    if (err != ESP_OK || bytesRead == 0) return;

    size_t samplesRead = bytesRead / sizeof(int32_t);

    for (size_t i = 0; i < samplesRead; i++) {
        // Most MEMS I2S mics place audio in the top 18 bits of a 32-bit frame.
        // Shift down to 16-bit signed PCM.
        dmaFillBuf[fillPos] = (int16_t)(rawBuf[i] >> 14);
        fillPos++;

        if (fillPos >= DOUBLE_BUF_SIZE) {
            // Buffer is full — flip double buffer
            memcpy(audioProcessBuffer, dmaFillBuf, sizeof(dmaFillBuf));
            fillPos = 0;
            audioBufferReady = true;
            break; // Remaining samples go into next fill cycle
        }
    }
}
