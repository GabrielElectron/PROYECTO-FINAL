#pragma once

#include "config.h"

// =====================================================
// MÓDULO SPI AD7606
// Raspberry Pi Pico 2 W + AD7606
// =====================================================
//
// Este archivo declara las funciones relacionadas con la
// configuración del bus SPI utilizado para leer el AD7606.
//
// Responsabilidades de este módulo:
// - Inicializar SPI0
// - Configurar pines SPI
// - Configurar formato SPI
// - Habilitar solicitud DMA del periférico SPI
// - Limpiar el FIFO RX antes de comenzar
//
// Este módulo NO se encarga de:
// - Configurar canales DMA
// - Generar CONVST
// - Manejar la interrupción BUSY
// - Enviar paquetes por Serial
//

// Inicializa SPI0 con la frecuencia, formato y pines definidos en config.h.
void spi_ad7606_init();

// Limpia el FIFO de recepción del SPI.
// Se usa para descartar datos viejos antes de comenzar la adquisición.
void spi_ad7606_flush_rx_fifo();