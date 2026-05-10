#include "dma_ad7606.h"

#include <string.h>
#include "hardware/spi.h"
#include "hardware/dma.h"

// =====================================================
// IMPLEMENTACIÓN DEL MÓDULO DMA AD7606
// =====================================================
//
// Este archivo contiene la configuración de los canales DMA
// utilizados para realizar la lectura SPI del AD7606.
//
// La lectura se hace usando:
// - Un canal DMA TX que escribe bytes dummy en el registro SPI.
// - Un canal DMA RX que copia los bytes recibidos desde SPI a memoria.
//
// Esto permite leer los 16 bytes de una conversión sin bloquear
// manualmente al procesador byte por byte.
//

static int dma_rx = -1;
static int dma_tx = -1;

// Buffer dummy usado para generar los pulsos de clock SPI.
// El AD7606 entrega datos por MISO, pero SPI necesita transmitir
// algo por MOSI para generar el clock.
static uint8_t tx_dummy[NBYTES];

void dma_ad7606_init() {
  // Reservamos dos canales DMA libres.
  dma_rx = dma_claim_unused_channel(true);
  dma_tx = dma_claim_unused_channel(true);

  // Inicializamos el buffer dummy en cero.
  memset(tx_dummy, 0, sizeof(tx_dummy));

  // -------------------------
  // Configuración DMA RX
  // -------------------------
  {
    dma_channel_config cfg = dma_channel_get_default_config(dma_rx);

    // Transferencia de 8 bits porque SPI trabaja en bytes.
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);

    // El registro SPI DR siempre es la misma dirección.
    channel_config_set_read_increment(&cfg, false);

    // El destino en memoria sí debe avanzar byte a byte.
    channel_config_set_write_increment(&cfg, true);

    // El canal RX se dispara cuando SPI0 tiene datos recibidos.
    channel_config_set_dreq(&cfg, DREQ_SPI0_RX);

    dma_channel_set_config(dma_rx, &cfg, false);
  }

  // -------------------------
  // Configuración DMA TX
  // -------------------------
  {
    dma_channel_config cfg = dma_channel_get_default_config(dma_tx);

    // Transferencia de 8 bits.
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);

    // Leemos desde el buffer dummy avanzando byte a byte.
    channel_config_set_read_increment(&cfg, true);

    // Escribimos siempre en el registro SPI DR.
    channel_config_set_write_increment(&cfg, false);

    // El canal TX se dispara cuando SPI0 puede transmitir.
    channel_config_set_dreq(&cfg, DREQ_SPI0_TX);

    dma_channel_set_config(dma_tx, &cfg, false);
  }
}

void dma_ad7606_start_read(uint8_t *dst16) {
  spi_hw_t *hw = spi_get_hw(spi0);

  // RX: SPI0 DR -> buffer destino
  dma_channel_set_write_addr(dma_rx, dst16, false);
  dma_channel_set_read_addr(dma_rx, (void *)&hw->dr, false);
  dma_channel_set_trans_count(dma_rx, NBYTES, false);

  // TX: buffer dummy -> SPI0 DR
  dma_channel_set_read_addr(dma_tx, tx_dummy, false);
  dma_channel_set_write_addr(dma_tx, (void *)&hw->dr, false);
  dma_channel_set_trans_count(dma_tx, NBYTES, false);

  // Arrancamos ambos canales al mismo tiempo.
  uint32_t mask = (1u << dma_rx) | (1u << dma_tx);
  dma_start_channel_mask(mask);
}

bool dma_ad7606_is_busy() {
  return dma_channel_is_busy(dma_rx) || dma_channel_is_busy(dma_tx);
}

int dma_ad7606_get_rx_channel() {
  return dma_rx;
}

int dma_ad7606_get_tx_channel() {
  return dma_tx;
}