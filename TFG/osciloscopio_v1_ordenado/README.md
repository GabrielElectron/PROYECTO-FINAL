# Osciloscopio v1 - Versión ordenada

Esta carpeta contiene la versión ordenada y modular del firmware para el sistema de adquisición de datos basado en **Raspberry Pi Pico 2 W** y **AD7606**.

La versión v1 fue desarrollada a partir de la versión `v0_funcionando`, manteniendo la misma lógica principal pero separando el código en módulos para facilitar su mantenimiento, documentación y futuras mejoras.

## Estado actual

Versión probada en hardware real.

- Compila correctamente en PlatformIO.
- Carga correctamente en Raspberry Pi Pico 2 W.
- Adquiere datos desde el AD7606.
- Genera CONVST por PWM.
- Detecta fin de conversión mediante BUSY.
- Lee los datos por SPI usando DMA.
- Envía paquetes binarios por USB Serial.
- Probada a 2 Mbps con señal senoidal estable.
- Comunicación compatible con el software de visualización actual.

## Hardware utilizado

- Raspberry Pi Pico 2 W
- Conversor analógico-digital AD7606
- Comunicación SPI
- USB Serial hacia PC
- Generador de funciones para pruebas

## Pines utilizados

| Señal | GPIO Pico 2 W | Descripción |
|---|---:|---|
| MISO | GPIO 16 | Datos desde AD7606 hacia Pico |
| CS | GPIO 17 | Selección del AD7606 |
| SCK | GPIO 18 | Reloj SPI |
| MOSI | GPIO 19 | Salida dummy para generar clock SPI |
| BUSY | GPIO 15 | Indica conversión en proceso |
| RESET | GPIO 14 | Reset del AD7606 |
| CONVST | GPIO 13 | Inicio de conversión |
| RANGE | GPIO 12 | Selección de rango de entrada |

## Parámetros principales

| Parámetro | Valor | Descripción |
|---|---:|---|
| `SPI_HZ` | 48 MHz | Frecuencia SPI |
| `FS_HZ` | 32 kS/s | Frecuencia de muestreo |
| `PULSE_US` | 2 us | Ancho del pulso CONVST |
| `NBYTES` | 16 bytes | 8 canales x 2 bytes |
| `FRAMES_PER_PKT` | 128 | Frames enviados por paquete |
| `RING_FRAMES` | 1024 | Tamaño del buffer circular |
| Serial | 2 Mbps | Comunicación USB Serial |

## Rango de entrada AD7606

El firmware actual configura el pin `RANGE` en alto:


## Estructura del proyecto

```txt
osciloscopio_v1_ordenado/
├── include/
│   ├── config.h
│   ├── ad7606.h
│   ├── spi_ad7606.h
│   ├── dma_ad7606.h
│   ├── adquisicion.h
│   └── comunicacion.h
│
├── src/
│   ├── main.cpp
│   ├── ad7606.cpp
│   ├── spi_ad7606.cpp
│   ├── dma_ad7606.cpp
│   ├── adquisicion.cpp
│   └── comunicacion.cpp
│
├── platformio.ini
└── README.md