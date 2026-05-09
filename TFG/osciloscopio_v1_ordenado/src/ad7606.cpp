#include <Arduino.h>
#include "hardware/gpio.h"
#include "ad7606.h"

// =====================================================
// IMPLEMENTACIÓN DEL MÓDULO AD7606
// =====================================================
//
// Este archivo contiene la implementación de las funciones
// declaradas en ad7606.h.
//
// Acá se concentran las operaciones básicas de control
// del AD7606, para evitar tener todo mezclado en main.cpp.
//

void ad7606_init_pins() {
  // BUSY es una entrada del AD7606.
  // Indica si el conversor está realizando una conversión.
  gpio_init(BUSY);
  gpio_set_dir(BUSY, GPIO_IN);
  gpio_pull_down(BUSY);

  // RESET es una salida desde la Pico hacia el AD7606.
  gpio_init(RESET);
  gpio_set_dir(RESET, GPIO_OUT);

  // RANGE permite seleccionar el rango de entrada del AD7606.
  gpio_init(RANGE);
  gpio_set_dir(RANGE, GPIO_OUT);

  // CS selecciona el AD7606 para la comunicación SPI.
  gpio_init(PIN_CS);
  gpio_set_dir(PIN_CS, GPIO_OUT);

  // Estado inicial seguro:
  // - AD7606 deseleccionado
  // - RANGE en bajo, igual que en la versión v0 funcionando
  ad7606_deselect();
  ad7606_set_range(false);
}

void ad7606_reset() {
  // Secuencia de reset utilizada en la versión funcional v0.
  gpio_put(RESET, 1);
  delayMicroseconds(5);

  gpio_put(RESET, 0);
  delayMicroseconds(5);
}

void ad7606_select() {
  // CS activo en bajo.
  gpio_put(PIN_CS, 0);
}

void ad7606_deselect() {
  // CS inactivo en alto.
  gpio_put(PIN_CS, 1);
}

void ad7606_set_range(bool high_range) {
  // En la versión v0 se usa RANGE = 0.
  // Dejamos la función preparada por si más adelante queremos cambiarlo.
  gpio_put(RANGE, high_range ? 1 : 0);
}