#pragma once

// =====================================================
// MÓDULO DE COMUNICACIÓN
// Raspberry Pi Pico 2 W + AD7606
// =====================================================
//
// Este módulo se encarga de enviar por USB Serial los datos
// adquiridos por el sistema.
//
// Formato del paquete:
// - SYNC:   4 bytes
// - seq:    4 bytes
// - frames: 2 bytes
// - data:   FRAMES_PER_PKT frames de NBYTES bytes cada uno
//
// Este módulo NO se encarga de:
// - Leer el AD7606
// - Configurar DMA
// - Generar CONVST
// - Calcular Vrms, Vpp o frecuencia
//

// Inicializa variables internas del módulo de comunicación.
void comunicacion_init();

// Envía un paquete si hay suficientes muestras disponibles.
// Si no hay suficientes muestras, no hace nada.
void comunicacion_update();