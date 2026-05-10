import serial
import struct
import time

from parser_ad7606 import parse_payload, channel_to_volts

PORT = "COM9"
BAUD = 2000000

SYNC = bytes([0xA5, 0x5A, 0xC3, 0x3C])

FRAMES_PER_PKT = 128
BYTES_PER_FRAME = 16
HEADER_SIZE = 10
PAYLOAD_SIZE = FRAMES_PER_PKT * BYTES_PER_FRAME
PACKET_SIZE = HEADER_SIZE + PAYLOAD_SIZE


def read_exact(ser, n):
    data = bytearray()

    while len(data) < n:
        chunk = ser.read(n - len(data))

        if not chunk:
            return None

        data.extend(chunk)

    return bytes(data)


def find_sync(ser):
    buffer = bytearray()

    while True:
        byte = ser.read(1)

        if not byte:
            continue

        buffer.extend(byte)

        if len(buffer) > 4:
            buffer.pop(0)

        if bytes(buffer) == SYNC:
            return True


def main():
    print("Abriendo puerto:", PORT)
    print("Baudrate:", BAUD)

    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(1)

    # Descarta datos viejos acumulados antes de empezar a leer.
    ser.reset_input_buffer()

    print("Puerto abierto.")
    print("Buscando paquetes...")

    last_seq = None
    lost_total = 0
    packet_count = 0
    warmup_packets = 50

    while True:
        find_sync(ser)

        header_rest = read_exact(ser, 6)

        if header_rest is None:
            print("Timeout leyendo header")
            continue

        seq, frames = struct.unpack("<IH", header_rest)

        payload = read_exact(ser, frames * BYTES_PER_FRAME)

        if payload is None:
            print("Timeout leyendo payload")
            continue

        channels_raw = parse_payload(payload, frames)
        ch1_raw = channels_raw[0]
        ch1_volts = channel_to_volts(ch1_raw, full_scale_volts=5.0)

        # Ignoramos los primeros paquetes porque pueden llegar desfasados
        # al abrir el puerto o al sincronizar por primera vez.
        if packet_count > warmup_packets and last_seq is not None:
            expected = last_seq + 1

            if seq != expected:
                lost = seq - expected

                if lost < 0:
                    lost = 0

                lost_total += lost

                print(
                    "Salto de secuencia.",
                    "Esperado:", expected,
                    "Recibido:", seq,
                    "Lost:", lost
                )

        last_seq = seq
        packet_count += 1

        if packet_count % 20 == 0:
            print(
                "Paquetes:", packet_count,
                "| Seq:", seq,
                "| Frames:", frames,
                "| Payload:", len(payload),
                "| Lost total:", lost_total,
                "| CH1 raw min/max:", min(ch1_raw), max(ch1_raw),
                "| CH1 V min/max:", round(min(ch1_volts), 3), round(max(ch1_volts), 3)
            )


if __name__ == "__main__":
    main()