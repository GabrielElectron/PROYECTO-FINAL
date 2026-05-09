#pragma once

#include "config.h"

// =====================================================
// MÓDULO AD7606
// Raspberry Pi Pico 2 W + AD7606
// =====================================================
//
// Este archivo declara las funciones básicas para controlar
// el conversor AD7606.
//
// Responsabilidades de este módulo:
// - Configurar pines de control del AD7606
// - Realizar reset del conversor
// - Configurar el rango de entrada
// - Controlar la señal CS
//
// Este módulo NO se encarga de:
// - Configurar DMA
// - Configurar PWM de CONVST
// - Calcular RMS, frecuencia o Vpp
// - Enviar datos por Serial
//

// Inicializa los pines de control del AD7606:
// BUSY, RESET, RANGE y CS.
void ad7606_init_pins();

// Realiza la secuencia de reset del AD7606.
void ad7606_reset();

// Selecciona el AD7606 bajando CS.
void ad7606_select();

// Deselecciona el AD7606 subiendo CS.
void ad7606_deselect();

// Configura el rango de entrada del AD7606.
// false -> RANGE = 0
// true  -> RANGE = 1
void ad7606_set_range(bool high_range);