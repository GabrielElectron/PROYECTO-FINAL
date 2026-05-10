import struct


NUM_CHANNELS = 8
BYTES_PER_CHANNEL = 2
BYTES_PER_FRAME = NUM_CHANNELS * BYTES_PER_CHANNEL


def parse_payload(payload: bytes, frames: int):
    """
    Convierte el payload binario recibido desde la Pico en 8 canales.

    Cada frame tiene 16 bytes:
    CH1_H CH1_L CH2_H CH2_L ... CH8_H CH8_L

    Los datos del AD7606 vienen en formato signed int16 big-endian.

    Retorna:
        Lista con 8 listas:
        [
            [ch1_muestras],
            [ch2_muestras],
            ...
            [ch8_muestras]
        ]
    """

    expected_size = frames * BYTES_PER_FRAME

    if len(payload) != expected_size:
        raise ValueError(
            f"Payload inválido: esperado {expected_size} bytes, recibido {len(payload)} bytes"
        )

    channels = [[] for _ in range(NUM_CHANNELS)]

    for frame_index in range(frames):
        base = frame_index * BYTES_PER_FRAME

        for ch in range(NUM_CHANNELS):
            offset = base + ch * BYTES_PER_CHANNEL

            raw = struct.unpack(">h", payload[offset:offset + 2])[0]

            channels[ch].append(raw)

    return channels


def raw_to_volts(raw_value: int, full_scale_volts: float = 5.0):
    """
    Convierte una muestra cruda int16 del AD7606 a voltios.

    Para rango ±5 V:
        32768 cuentas ≈ 5 V
    """

    return (raw_value / 32768.0) * full_scale_volts


def channel_to_volts(raw_channel, full_scale_volts: float = 5.0):
    """
    Convierte una lista de muestras crudas a voltios.
    """

    return [raw_to_volts(x, full_scale_volts) for x in raw_channel]