#include "comunicacion.h"

#include <Arduino.h>
#include <stdint.h>

#include "config.h"
#include "adquisicion.h"

// =====================================================
// IMPLEMENTACIÓN DEL MÓDULO DE COMUNICACIÓN
// =====================================================
//
// Este archivo toma las muestras disponibles en el buffer circular
// de adquisición y las envía por USB Serial en paquetes binarios.
//
// La estructura del paquete se mantiene compatible con la v0:
// SYNC(4) + seq(4) + frames(2) + payload
//

// Palabra de sincronismo para que el software de PC pueda detectar
// el inicio de cada paquete.
static const uint8_t SYNC[4] = {0xA5, 0x5A, 0xC3, 0x3C};

// Número de paquete enviado.
static uint32_t seq = 0;

// Índice de lectura dentro del buffer circular.
static uint32_t rd_idx = 0;

void comunicacion_init() {
  seq = 0;
  rd_idx = 0;
}

void comunicacion_update() {
  uint32_t wr_idx = adquisicion_get_write_index();

  // Si todavía no hay suficientes frames nuevos, no enviamos nada.
  if ((wr_idx - rd_idx) < FRAMES_PER_PKT) {
    return;
  }

  // Header del paquete:
  // SYNC(4) + seq(4) + frames(2)
  Serial.write(SYNC, 4);
  Serial.write((uint8_t*)&seq, 4);

  uint16_t frames = FRAMES_PER_PKT;
  Serial.write((uint8_t*)&frames, 2);

  // Payload:
  // FRAMES_PER_PKT frames, cada uno de NBYTES bytes.
  for (uint16_t i = 0; i < FRAMES_PER_PKT; i++) {
    uint32_t idx = rd_idx + i;
    const uint8_t *frame = adquisicion_get_frame(idx);

    Serial.write(frame, NBYTES);
  }

  rd_idx += FRAMES_PER_PKT;
  seq++;
}