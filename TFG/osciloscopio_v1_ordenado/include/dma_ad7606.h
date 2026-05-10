#pragma once

#include <stdint.h>
#include "config.h"

// =====================================================
// MÓDULO DMA AD7606
// Raspberry Pi Pico 2 W + AD7606
// =====================================================
//
// Este módulo se encarga de configurar y manejar los canales
// DMA utilizados para leer el AD7606 a través de SPI0.
//
// En la versión funcional v0 se usan dos canales DMA:
// - DMA RX: mueve datos desde SPI0 hacia memoria.
// - DMA TX: envía bytes dummy para generar clock SPI.
//
// Cada lectura del AD7606 entrega 16 bytes:
// 8 canales x 2 bytes por canal.
//

// Inicializa los canales DMA necesarios para SPI0.
void dma_ad7606_init();

// Inicia una transferencia DMA de 16 bytes.
// El parámetro dst16 debe apuntar a un buffer de al menos NBYTES bytes.
void dma_ad7606_start_read(uint8_t *dst16);

// Devuelve true si alguno de los canales DMA está ocupado.
bool dma_ad7606_is_busy();

// Devuelve el número de canal DMA usado para RX.
int dma_ad7606_get_rx_channel();

// Devuelve el número de canal DMA usado para TX.
int dma_ad7606_get_tx_channel();