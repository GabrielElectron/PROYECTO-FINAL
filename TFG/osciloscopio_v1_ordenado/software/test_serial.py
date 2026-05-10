import serial
import time

PORT = "COM9"
BAUD = 2000000

SYNC = bytes([0xA5, 0x5A, 0xC3, 0x3C])

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(1)

print("Puerto abierto:", PORT)
print("Buscando SYNC...")

buffer = bytearray()

while True:
    data = ser.read(1024)

    if data:
        buffer.extend(data)

        pos = buffer.find(SYNC)

        if pos != -1:
            print("SYNC encontrado en posición:", pos)
            print("Primeros bytes:", buffer[pos:pos+20].hex(" "))
            buffer.clear()