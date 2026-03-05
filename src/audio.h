#pragma once
#include <Arduino.h>

// Connections to INMP441 I2S microphone
#define I2S_WS_PIN  D6    // INMP441 pin: WS  (also called LRCK)
#define I2S_SCK_PIN D7    // INMP441 pin: SCK (bit clock)
#define I2S_SD_PIN  D2    // INMP441 pin: SD  (data out from mic)

// Audio config
#define SAMPLE_RATE       16000
#define FFT_SIZE          512         // Must be power of 2
#define DMA_BUF_COUNT     4           // Number of DMA buffers
#define DMA_BUF_LEN       256         // Samples per DMA buffer
#define DOUBLE_BUF_SIZE   FFT_SIZE    // Samples per processing buffer

extern volatile bool audioBufferReady;
extern int16_t audioProcessBuffer[DOUBLE_BUF_SIZE];  // Ready for FFT

bool audioInit();
void audioUpdate();   // Call in loop() - checks DMA and flips double buffer
