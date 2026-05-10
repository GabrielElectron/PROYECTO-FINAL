#pragma once

#include <stdint.h>
#include "config.h"

// =====================================================
// MÓDULO DE ADQUISICIÓN
// Raspberry Pi Pico 2 W + AD7606
// =====================================================
//
// Este módulo se encarga de la adquisición de muestras:
// - Genera la señal CONVST mediante PWM.
// - Atiende la interrupción de BUSY.
// - Inicia la lectura SPI + DMA.
// - Guarda los frames recibidos en un buffer circular.
//
// Este módulo NO se encarga de:
// - Enviar datos por Serial.
// - Calcular RMS, frecuencia o Vpp.
// - Dibujar gráficos.
//


// Inicializa la adquisición:
// - Configura PWM para CONVST.
// - Limpia flags de BUSY.
// - Habilita interrupción por flanco de bajada en BUSY.
void adquisicion_init();


// Devuelve el índice actual de escritura del buffer circular.
// Este valor aumenta cada vez que se captura una muestra.
uint32_t adquisicion_get_write_index();


// Devuelve un puntero a un frame del buffer circular.
// Cada frame contiene NBYTES bytes.
const uint8_t* adquisicion_get_frame(uint32_t index);


// Devuelve la cantidad total de muestras capturadas por la ISR.
uint32_t adquisicion_get_sample_count();


// Devuelve la cantidad de overruns detectados.
// Un overrun ocurre cuando llega una muestra nueva pero el DMA anterior
// todavía no terminó.
uint32_t adquisicion_get_overruns();