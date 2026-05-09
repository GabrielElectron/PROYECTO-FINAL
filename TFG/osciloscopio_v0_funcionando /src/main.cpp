/*
  Proyecto: Osciloscopio con Raspberry Pi Pico 2 W + AD7606
  Versión: v0 funcionando
  Archivo original: AD_DMA_PWM_OK1.cpp

  Descripción:
  Esta versión corresponde al código funcional actual del proyecto.
  Se conserva como referencia estable antes de comenzar la refactorización.
*/

#include <Arduino.h>
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

// ===== Pines (SPI0) =====
#define PIN_MISO 16
#define PIN_SCK  18
#define PIN_MOSI 19     // dummy TX (no hace falta cablearlo)
#define PIN_CS   17     // CS manual

#define BUSY    15
#define RESET   14
#define CONVST  13
#define RANGE   12



// ===== Parámetros =====
static const uint32_t SPI_HZ   = 48000000;   // ya lo probaste estable
static const uint32_t FS_HZ    = 32000;     // 100 kS/s exactos
static const uint32_t PULSE_US = 2;          // pulso CONVST activo-bajo ~1us

static const int NBYTES = 16;

static const uint32_t PERIOD_US = (1000000u / FS_HZ);   // 10 us si FS_HZ=100000
static const uint32_t MIN_DT_US = (PERIOD_US * 7) / 10; // 70% del período (7 us)
static volatile uint32_t last_irq_us = 0;

// BINARIOS
static const uint8_t SYNC[4] = {0xA5, 0x5A, 0xC3, 0x3C};
static const uint16_t FRAMES_PER_PKT = 128;

static uint32_t seq = 0;


// Buffer circular (solo para tener un “último frame” y stats)
// (Si querés guardar muchos, subí a 1024/2048; cada frame son 16 bytes)
static const int RING_FRAMES = 1024;
static uint8_t ring[RING_FRAMES][NBYTES];

static volatile uint32_t wr_idx = 0;
static volatile uint32_t isr_samples = 0;
static volatile uint32_t overruns = 0;
static volatile uint32_t last_frame_idx = 0;

// Dummy TX para clockear SPI
static uint8_t tx_dummy[NBYTES];

// Canales DMA
static int dma_rx = -1;
static int dma_tx = -1;

// ====== Parse ======
static int16_t A[8];
static inline void parse8words_be_s16(const uint8_t *raw16, int16_t *out8) {
  for (int i = 0; i < 8; i++) {
    uint16_t u = ((uint16_t)raw16[2*i] << 8) | raw16[2*i + 1];
    out8[i] = (int16_t)u;
  }
}

// ===== AD7606 reset =====
static inline void ad7606_reset() {
  gpio_put(RESET, 1); delayMicroseconds(5);
  gpio_put(RESET, 0); delayMicroseconds(5);
}

// ===== DMA start (NO blocking) =====
static inline void spi_dma_start16(uint8_t *dst16) {
  spi_hw_t *hw = spi_get_hw(spi0);

  // RX: DR -> dst16
  dma_channel_set_write_addr(dma_rx, dst16, false);
  dma_channel_set_read_addr(dma_rx, (void *)&hw->dr, false);
  dma_channel_set_trans_count(dma_rx, NBYTES, false);

  // TX: tx_dummy -> DR
  dma_channel_set_read_addr(dma_tx, tx_dummy, false);
  dma_channel_set_write_addr(dma_tx, (void *)&hw->dr, false);
  dma_channel_set_trans_count(dma_tx, NBYTES, false);

  // Start simultáneo
  uint32_t mask = (1u << dma_rx) | (1u << dma_tx);
  dma_start_channel_mask(mask);
}

// ===== IRQ callback BUSY =====
// Dispara en flanco de bajada (fin de conversión) => arrancamos DMA para leer 16 bytes
static void busy_irq(uint gpio, uint32_t events) {
  if (gpio != BUSY) return;
  if ((events & GPIO_IRQ_EDGE_FALL) == 0) return;

  uint32_t now_us = time_us_32();
  uint32_t dt = now_us - last_irq_us;

  if (dt < MIN_DT_US) return;   // filtro anti-ruido/rebote

  last_irq_us = now_us;

  // Si el DMA anterior todavía está ocupado, contamos overrun (perdimos una muestra)
  if (dma_channel_is_busy(dma_rx) || dma_channel_is_busy(dma_tx)) {
    overruns++;
    return;
  }

  uint32_t i = wr_idx % RING_FRAMES;
  uint8_t *dst = ring[i];

  spi_dma_start16(dst);

  last_frame_idx = i;
  wr_idx++;
  isr_samples++;
}

// ===== Configurar CONVST por PWM (pulso activo-bajo) =====
static void setup_convst_pwm(uint32_t fs_hz, uint32_t pulse_us) {
  gpio_set_function(CONVST, GPIO_FUNC_PWM);
  gpio_set_outover(CONVST, GPIO_OVERRIDE_INVERT); // pulso bajo, idle alto

  uint slice = pwm_gpio_to_slice_num(CONVST);
  uint chan  = pwm_gpio_to_channel(CONVST);

  uint32_t sys_hz = clock_get_hz(clk_sys);

  // Buscamos el mejor divisor (int.frac, frac en 1/16) y TOP para aproximar fs_hz
  uint32_t best_err = 0xFFFFFFFFu;
  uint16_t best_div_int = 1, best_div_frac = 0;
  uint16_t best_top = 100;

  for (uint16_t div_int = 1; div_int <= 255; div_int++) {
    for (uint16_t div_frac = 0; div_frac < 16; div_frac++) {
      if (div_int == 1 && div_frac == 0) {
        // permitido, ok
      }
      // divisor en "ticks/16"
      uint32_t div16 = (uint32_t)div_int * 16u + (uint32_t)div_frac;
      if (div16 < 16u) continue; // no debería pasar

      // top+1 = sys_hz * 16 / (div16 * fs)
      uint64_t denom = (uint64_t)div16 * (uint64_t)fs_hz;
      if (denom == 0) continue;

      uint64_t top_plus_1 = ((uint64_t)sys_hz * 16ull) / denom;
      if (top_plus_1 < 2) continue;
      if (top_plus_1 > 65536ull) continue;

      uint16_t top = (uint16_t)(top_plus_1 - 1ull);

      // f_eff = sys_hz*16 / (div16*(top+1))
      uint64_t f_eff_num = (uint64_t)sys_hz * 16ull;
      uint64_t f_eff_den = (uint64_t)div16 * (uint64_t)(top + 1u);
      uint32_t f_eff = (uint32_t)(f_eff_num / f_eff_den);

      uint32_t err = (f_eff > fs_hz) ? (f_eff - fs_hz) : (fs_hz - f_eff);

      if (err < best_err) {
        best_err = err;
        best_div_int = div_int;
        best_div_frac = div_frac;
        best_top = top;
        if (best_err == 0) goto done_search;
      }
    }
  }
done_search:

  // Duty/pulso: necesitamos "level" en ticks del contador (antes de invertir => pulso bajo)
  // ticks_per_us = sys_hz / (div * 1e6)
  // div = div16/16 => ticks_per_us = sys_hz*16/(div16*1e6)
  uint32_t div16 = (uint32_t)best_div_int * 16u + (uint32_t)best_div_frac;
  uint32_t ticks_per_us = (uint32_t)(((uint64_t)sys_hz * 16ull) / ((uint64_t)div16 * 1000000ull));
  uint32_t level = ticks_per_us * pulse_us;
  if (level < 2) level = 2;
  if (level > best_top) level = best_top;

  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_clkdiv_int_frac(&cfg, best_div_int, best_div_frac);
  pwm_config_set_wrap(&cfg, best_top);

  pwm_init(slice, &cfg, false);
  pwm_set_chan_level(slice, chan, (uint16_t)level);
  pwm_set_enabled(slice, true);

  // Print REAL
  uint32_t f_eff = (uint32_t)(((uint64_t)sys_hz * 16ull) / ((uint64_t)div16 * (uint64_t)(best_top + 1u)));

  //Serial.print("PWM: sys_hz="); Serial.print(sys_hz);
  //Serial.print(" div="); Serial.print(best_div_int);
  //Serial.print("."); Serial.print(best_div_frac);
  //Serial.print("/16 top="); Serial.print(best_top);
  //Serial.print(" level="); Serial.print(level);
  //Serial.print(" => f_eff="); Serial.print(f_eff);
  //Serial.print(" err="); Serial.println(best_err);
}

void setup() {
  Serial.begin(3000000);
  delay(300);
  //Serial.println("BOOT OK (100kHz PWM CONVST + BUSY IRQ + SPI DMA)");
  uint32_t t_wait = millis();
  while (!Serial && (millis() - t_wait < 1500)) { }   // esperar hasta 1.5s USB CDC
  delay(200);
  //Serial.println("SERIAL READY");

  // GPIO control
  gpio_init(BUSY);   gpio_set_dir(BUSY, GPIO_IN);  gpio_pull_down(BUSY);
  gpio_init(RESET);  gpio_set_dir(RESET, GPIO_OUT);
  gpio_init(RANGE);  gpio_set_dir(RANGE, GPIO_OUT);

  gpio_init(PIN_CS); gpio_set_dir(PIN_CS, GPIO_OUT);
  gpio_put(PIN_CS, 1);

  gpio_put(RANGE, 0);
  ad7606_reset();

  // SPI0 init
  spi_init(spi0, SPI_HZ);
  spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

  // Habilita DMA en SPI0
  spi_get_hw(spi0)->dmacr = SPI_SSPDMACR_TXDMAE_BITS | SPI_SSPDMACR_RXDMAE_BITS;

  // DMA channels
  dma_rx = dma_claim_unused_channel(true);
  dma_tx = dma_claim_unused_channel(true);

  memset(tx_dummy, 0, sizeof(tx_dummy));

  // Preconfig RX
  {
    dma_channel_config cfg = dma_channel_get_default_config(dma_rx);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, DREQ_SPI0_RX);
    dma_channel_set_config(dma_rx, &cfg, false);
  }

  // Preconfig TX
  {
    dma_channel_config cfg = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_dreq(&cfg, DREQ_SPI0_TX);
    dma_channel_set_config(dma_tx, &cfg, false);
  }

  // Flush RX FIFO una vez
  {
    spi_hw_t *hw = spi_get_hw(spi0);
    while (hw->sr & SPI_SSPSR_RNE_BITS) (void)hw->dr;
  }

  // CS bajo permanente (AD7606 seleccionado siempre)
  gpio_put(PIN_CS, 0);

  // 1) Configurá PWM primero (todavía sin IRQ)
    setup_convst_pwm(FS_HZ, PULSE_US);

    // 2) Limpia posibles flags pendientes en BUSY (por si había ruido)
    gpio_acknowledge_irq(BUSY, GPIO_IRQ_EDGE_FALL);

    //Serial.print("BUSY initial="); Serial.println(gpio_get(BUSY));
    delay(10);
    //Serial.print("BUSY after10ms="); Serial.println(gpio_get(BUSY));

    // 3) Recién ahora habilitá IRQ
    gpio_set_irq_enabled_with_callback(BUSY, GPIO_IRQ_EDGE_FALL, true, &busy_irq);

  //Serial.print("SPI_HZ="); Serial.print(SPI_HZ);
  //Serial.print("  FS_HZ="); Serial.print(FS_HZ);
  //Serial.print("  PULSE_US="); Serial.print(PULSE_US);
  //Serial.print("  dma_rx="); Serial.print(dma_rx);
  //Serial.print("  dma_tx="); Serial.println(dma_tx);
}

void loop() {
  // Espera hasta tener al menos FRAMES_PER_PKT muestras nuevas en ring
  // wr_idx crece en ISR (muestras capturadas)
  static uint32_t rd_idx = 0;

  uint32_t w = wr_idx;  // snapshot (volatile)
  if ((w - rd_idx) < FRAMES_PER_PKT) return;

  // Armamos paquete: header + payload
  // Header: SYNC(4) + seq(4) + frames(2)
  Serial.write(SYNC, 4);
  Serial.write((uint8_t*)&seq, 4);
  Serial.write((uint8_t*)&FRAMES_PER_PKT, 2);

  // Payload: FRAMES_PER_PKT * 16 bytes
  // Mandamos desde ring en orden. Cada frame ya está en big-endian si leés del AD7606 así.
  for (uint16_t i = 0; i < FRAMES_PER_PKT; i++) {
    uint32_t idx = (rd_idx + i) % RING_FRAMES;
    Serial.write(ring[idx], NBYTES);
  }

  rd_idx += FRAMES_PER_PKT;
  seq++;
}
