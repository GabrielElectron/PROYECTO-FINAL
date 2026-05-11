import sys
import struct
import time

import serial
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
from mediciones import calcular_mediciones


# =====================================================
# CONFIGURACIÓN
# =====================================================

PORT = "COM9"
BAUD = 2000000

SYNC = b"\xA5\x5A\xC3\x3C"
HDR_LEN = 10

CH_PHY = 8
BYTES_PER_FRAME = 16
MAX_FRAMES = 4096

V_RANGE = 10.0
ADC_SCALE = V_RANGE / 32768.0

MAX_POINTS = 2000

FS_HZ = 32000

# =====================================================
# READER SERIAL EN HILO SEPARADO
# =====================================================

class Reader(QtCore.QThread):
    packet = QtCore.Signal(object, int)

    def __init__(self, port, baud):
        super().__init__()

        self.ser = serial.Serial(port, baud, timeout=0.2)

        time.sleep(1.0)

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.buf = bytearray()
        self.running = True

        self.sync_hits = 0
        self.bad_frames = 0

    def stop(self):
        self.running = False

    def run(self):
        while self.running:
            chunk = self.ser.read(65536)

            if chunk:
                self.buf += chunk

            while True:
                idx = self.buf.find(SYNC)

                if idx < 0:
                    if len(self.buf) > 262144:
                        self.buf = self.buf[-64:]
                    break

                if idx > 0:
                    del self.buf[:idx]

                if len(self.buf) < HDR_LEN:
                    break

                try:
                    seq = struct.unpack_from("<I", self.buf, 4)[0]
                    frames = struct.unpack_from("<H", self.buf, 8)[0]
                except struct.error:
                    break

                if frames == 0 or frames > MAX_FRAMES:
                    self.bad_frames += 1
                    del self.buf[:1]
                    continue

                pkt_len = HDR_LEN + frames * BYTES_PER_FRAME

                if len(self.buf) < pkt_len:
                    break

                payload = bytes(self.buf[HDR_LEN:pkt_len])
                del self.buf[:pkt_len]

                data = np.frombuffer(payload, dtype=">i2")

                if data.size != frames * CH_PHY:
                    continue

                data = data.reshape(frames, CH_PHY)

                self.sync_hits += 1
                self.packet.emit(data, seq)

        try:
            self.ser.close()
        except Exception:
            pass


# =====================================================
# VENTANA PRINCIPAL
# =====================================================

class ScopeCH1(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Osciloscopio v1 - CH1")
        self.resize(1200, 700)

        pg.setConfigOptions(useOpenGL=True)

        self.last_seq = None
        self.lost_packets = 0
        self.packets = 0

        self.warmup_packets = 50
        self.warmup_done = False

        self.buffer = np.zeros(MAX_POINTS, dtype=np.float32)
        self.wr = 0
        self.filled = False

        self.plot = pg.PlotWidget()
        self.setCentralWidget(self.plot)

        self.plot.setBackground("k")
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setLabel("left", "Voltaje", units="V")
        self.plot.setLabel("bottom", "Muestras")
        self.plot.setTitle("CH1 - Señal en tiempo real")

        self.plot.setXRange(0, MAX_POINTS)
        self.plot.setYRange(-V_RANGE, V_RANGE)
        self.plot.enableAutoRange(x=False, y=False)

        self.curve = self.plot.plot(
            pen=pg.mkPen((255, 255, 0), width=2)
        )

        self.text = pg.TextItem(color=(255, 255, 0), anchor=(0, 0))
        self.plot.addItem(self.text)
        self.text.setPos(20, 4)

        print("Abriendo puerto:", PORT)
        print("Baudrate:", BAUD)

        self.reader = Reader(PORT, BAUD)
        self.reader.packet.connect(self.on_packet)
        self.reader.start()

    def on_packet(self, data_i16, seq):
        self.packets += 1

        if self.packets <= self.warmup_packets:
            self.last_seq = seq
            self.lost_packets = 0
        else:
            if self.last_seq is not None and seq != self.last_seq + 1:
                self.lost_packets += max(0, seq - self.last_seq - 1)

            self.last_seq = seq

        ch1 = data_i16[:, 0].astype(np.float32) * ADC_SCALE
        n = len(ch1)

        if n >= MAX_POINTS:
            self.buffer[:] = ch1[-MAX_POINTS:]
            self.wr = 0
            self.filled = True
        else:
            end = self.wr + n

            if end <= MAX_POINTS:
                self.buffer[self.wr:end] = ch1
            else:
                first = MAX_POINTS - self.wr
                self.buffer[self.wr:] = ch1[:first]
                self.buffer[:end % MAX_POINTS] = ch1[first:]
                self.filled = True

            self.wr = end % MAX_POINTS

        if self.filled:
            y = np.concatenate((self.buffer[self.wr:], self.buffer[:self.wr]))
        else:
            y = self.buffer[:self.wr]

        if len(y) == 0:
            return

        x = np.arange(len(y))

        self.curve.setData(x, y)

        med = calcular_mediciones(y, FS_HZ)

        vmax = med["vmax"]
        vmin = med["vmin"]
        vpp = med["vpp"]
        vrms = med["vrms_ac"]
        vmedio = med["vmedio"]
        frecuencia = med["frecuencia"]

        self.text.setText(
            f"CH1\n"
            f"Vmax: {vmax:.3f} V\n"
            f"Vmin: {vmin:.3f} V\n"
            f"Vpp: {vpp:.3f} V\n"
            f"Vmedio: {vmedio:.3f} V\n"
            f"Vrms : {vrms:.3f} V\n"
            f"Freq: {frecuencia:.2f} Hz\n"
            f"Seq: {self.last_seq}\n"
            f"Lost: {self.lost_packets}"
        )

        self.text.setPos(20, vmax)

    def closeEvent(self, event):
        self.reader.stop()
        self.reader.wait(1000)
        event.accept()


def main():
    app = pg.mkQApp("Osciloscopio v1 - CH1")

    win = ScopeCH1()
    win.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()