#include <Arduino.h>

#include "ad7606.h"
#include "spi_ad7606.h"
#include "dma_ad7606.h"
#include "adquisicion.h"
#include "comunicacion.h"

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
  spi_ad7606_init();
  dma_ad7606_init();
  ad7606_select();
  adquisicion_init();
  comunicacion_init();

  //Serial.println("Osciloscopio v1 - Inicio correcto");
}

void loop() {
  comunicacion_update();
}