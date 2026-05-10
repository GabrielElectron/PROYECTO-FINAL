#include "adquisicion.h"

#include <Arduino.h>
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include "dma_ad7606.h"

// =====================================================
// IMPLEMENTACIÓN DEL MÓDULO DE ADQUISICIÓN
// =====================================================
//
// Este archivo concentra la lógica de adquisición:
// - PWM de CONVST.
// - IRQ de BUSY.
// - Buffer circular.
// - Inicio de lectura mediante DMA.
//
// La idea es mantener main.cpp limpio y dejar acá todo lo relacionado
// con la captura de muestras.
//

// Buffer circular de adquisición.
// Cada frame tiene 16 bytes: 8 canales x 2 bytes.
static uint8_t ring[RING_FRAMES][NBYTES];

// Índice de escritura del buffer circular.
// Se incrementa en la interrupción BUSY cada vez que se captura un frame.
static volatile uint32_t wr_idx = 0;

// Contador total de muestras capturadas desde el inicio.
static volatile uint32_t isr_samples = 0;

// Contador de pérdidas por DMA ocupado.
static volatile uint32_t overruns = 0;

// Último índice escrito dentro del buffer circular.
static volatile uint32_t last_frame_idx = 0;

// Tiempo de la última interrupción válida.
// Se usa para filtrar rebotes o ruido en BUSY.
static volatile uint32_t last_irq_us = 0;


// -----------------------------------------------------
// Configuración de PWM para generar CONVST
// -----------------------------------------------------
//
// El AD7606 inicia una conversión con un pulso en CONVST.
// En la versión funcional v0 se genera CONVST usando PWM.
// El pin se configura con salida invertida para obtener un pulso activo-bajo.
//
static void setup_convst_pwm(uint32_t fs_hz, uint32_t pulse_us) {
  gpio_set_function(CONVST, GPIO_FUNC_PWM);
  gpio_set_outover(CONVST, GPIO_OVERRIDE_INVERT);

  uint slice = pwm_gpio_to_slice_num(CONVST);
  uint chan  = pwm_gpio_to_channel(CONVST);

  uint32_t sys_hz = clock_get_hz(clk_sys);

  uint32_t best_err = 0xFFFFFFFFu;
  uint16_t best_div_int = 1;
  uint16_t best_div_frac = 0;
  uint16_t best_top = 100;

  for (uint16_t div_int = 1; div_int <= 255; div_int++) {
    for (uint16_t div_frac = 0; div_frac < 16; div_frac++) {
      uint32_t div16 = (uint32_t)div_int * 16u + (uint32_t)div_frac;
      if (div16 < 16u) continue;

      uint64_t denom = (uint64_t)div16 * (uint64_t)fs_hz;
      if (denom == 0) continue;

      uint64_t top_plus_1 = ((uint64_t)sys_hz * 16ull) / denom;
      if (top_plus_1 < 2) continue;
      if (top_plus_1 > 65536ull) continue;

      uint16_t top = (uint16_t)(top_plus_1 - 1ull);

      uint64_t f_eff_num = (uint64_t)sys_hz * 16ull;
      uint64_t f_eff_den = (uint64_t)div16 * (uint64_t)(top + 1u);
      uint32_t f_eff = (uint32_t)(f_eff_num / f_eff_den);

      uint32_t err = (f_eff > fs_hz) ? (f_eff - fs_hz) : (fs_hz - f_eff);

      if (err < best_err) {
        best_err = err;
        best_div_int = div_int;
        best_div_frac = div_frac;
        best_top = top;

        if (best_err == 0) {
          goto done_search;
        }
      }
    }
  }

done_search:

  uint32_t div16 = (uint32_t)best_div_int * 16u + (uint32_t)best_div_frac;

  uint32_t ticks_per_us =
      (uint32_t)(((uint64_t)sys_hz * 16ull) /
      ((uint64_t)div16 * 1000000ull));

  uint32_t level = ticks_per_us * pulse_us;

  if (level < 2) level = 2;
  if (level > best_top) level = best_top;

  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_clkdiv_int_frac(&cfg, best_div_int, best_div_frac);
  pwm_config_set_wrap(&cfg, best_top);

  pwm_init(slice, &cfg, false);
  pwm_set_chan_level(slice, chan, (uint16_t)level);
  pwm_set_enabled(slice, true);
}


// -----------------------------------------------------
// Interrupción de BUSY
// -----------------------------------------------------
//
// El AD7606 pone BUSY en bajo cuando termina una conversión.
// En ese flanco de bajada arrancamos una lectura SPI + DMA de 16 bytes.
//
static void busy_irq(uint gpio, uint32_t events) {
  if (gpio != BUSY) return;
  if ((events & GPIO_IRQ_EDGE_FALL) == 0) return;

  uint32_t now_us = time_us_32();
  uint32_t dt = now_us - last_irq_us;

  // Filtro anti-ruido/rebote:
  // Si la interrupción llega demasiado cerca de la anterior, se ignora.
  if (dt < MIN_DT_US) return;

  last_irq_us = now_us;

  // Si el DMA anterior todavía está ocupado, se pierde esta muestra.
  if (dma_ad7606_is_busy()) {
    overruns++;
    return;
  }

  uint32_t i = wr_idx % RING_FRAMES;
  uint8_t *dst = ring[i];

  dma_ad7606_start_read(dst);

  last_frame_idx = i;
  wr_idx++;
  isr_samples++;
}


// -----------------------------------------------------
// Inicialización pública del módulo
// -----------------------------------------------------
void adquisicion_init() {
  // Primero configuramos PWM para comenzar a generar CONVST.
  setup_convst_pwm(FS_HZ, PULSE_US);

  // Limpiamos posibles flags pendientes de BUSY.
  gpio_acknowledge_irq(BUSY, GPIO_IRQ_EDGE_FALL);

  delay(10);

  // Habilitamos la interrupción de BUSY por flanco de bajada.
  gpio_set_irq_enabled_with_callback(
      BUSY,
      GPIO_IRQ_EDGE_FALL,
      true,
      &busy_irq
  );
}


// -----------------------------------------------------
// Funciones de consulta
// -----------------------------------------------------
uint32_t adquisicion_get_write_index() {
  return wr_idx;
}

const uint8_t* adquisicion_get_frame(uint32_t index) {
  return ring[index % RING_FRAMES];
}

uint32_t adquisicion_get_sample_count() {
  return isr_samples;
}

uint32_t adquisicion_get_overruns() {
  return overruns;
}