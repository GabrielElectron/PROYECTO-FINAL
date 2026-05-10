#pragma once

#include <stdint.h>

// =====================================================
// CONFIGURACIÓN GENERAL DEL PROYECTO
// Raspberry Pi Pico 2 W + AD7606
// =====================================================

// ===== Pines SPI0 =====
#define PIN_MISO 16
#define PIN_SCK  18
#define PIN_MOSI 19
#define PIN_CS   17

// ===== Pines de control AD7606 =====
#define BUSY    15
#define RESET   14
#define CONVST  13
#define RANGE   12

// ===== Parámetros de adquisición =====
static const uint32_t SPI_HZ   = 48000000;
static const uint32_t FS_HZ    = 32000;
static const uint32_t PULSE_US = 2;

// ===== Datos AD7606 =====
static const int AD7606_CHANNELS = 8;
static const int BYTES_PER_CHANNEL = 2;
static const int NBYTES = AD7606_CHANNELS * BYTES_PER_CHANNEL;

// ===== Comunicación serie =====
static const uint16_t FRAMES_PER_PKT = 128;

// ===== Buffer circular =====
static const int RING_FRAMES = 1024;