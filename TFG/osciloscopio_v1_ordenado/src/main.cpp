#include <Arduino.h>
#include "ad7606.h"

// =====================================================
// PROGRAMA PRINCIPAL - OSCILOSCOPIO V1 ORDENADO
// Raspberry Pi Pico 2 W + AD7606
// =====================================================
//
// Esta versión inicial solo prueba la estructura modular.
// Todavía no realiza adquisición por DMA ni envío de datos.
//

void setup() {
  Serial.begin(2000000);

  delay(300);

  ad7606_init_pins();
  ad7606_reset();
  ad7606_select();

  Serial.println("Osciloscopio v1 - Inicio correcto");
}

void loop() {
  delay(1000);
}