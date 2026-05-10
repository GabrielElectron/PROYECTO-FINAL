#include "spi_ad7606.h"

#include "hardware/gpio.h"
#include "hardware/spi.h"

// =====================================================
// IMPLEMENTACIÓN DEL MÓDULO SPI AD7606
// =====================================================
//
// Este archivo contiene la configuración del periférico SPI0
// usado para leer los datos entregados por el AD7606.
//
// En la versión v0 funcional, el AD7606 se lee por SPI usando
// transferencias de 8 bits. Cada conversión entrega 16 bytes:
// 8 canales x 2 bytes por canal.
//

void spi_ad7606_init() {
  // Inicializa SPI0 con la frecuencia definida en config.h.
  spi_init(spi0, SPI_HZ);

  // Configuración SPI utilizada en la versión funcional v0:
  // - 8 bits por transferencia
  // - CPOL = 0
  // - CPHA = 0
  // - MSB primero
  spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

  // Asigna los pines físicos al periférico SPI0.
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

  // Habilita solicitudes DMA del periférico SPI0.
  // Esto permite que luego el módulo DMA pueda mover datos
  // sin intervención constante del procesador.
  spi_get_hw(spi0)->dmacr =
      SPI_SSPDMACR_TXDMAE_BITS |
      SPI_SSPDMACR_RXDMAE_BITS;

  // Limpieza inicial del FIFO RX.
  spi_ad7606_flush_rx_fifo();
}

void spi_ad7606_flush_rx_fifo() {
  spi_hw_t *hw = spi_get_hw(spi0);

  // Mientras haya datos pendientes en el FIFO RX,
  // se leen y se descartan.
  while (hw->sr & SPI_SSPSR_RNE_BITS) {
    (void)hw->dr;
  }
}