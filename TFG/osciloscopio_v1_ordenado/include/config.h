#pragma once
#include <stdint.h>

#define PIN_MISO 16
#define PIN_SCK  18
#define PIN_MOSI 19
#define PIN_CS   17

#define BUSY    15
#define RESET   14
#define CONVST  13
#define RANGE   12

static const uint32_t SPI_HZ = 48000000;
static const uint32_t FS_HZ = 32000;
static const uint32_t PULSE_US = 2;