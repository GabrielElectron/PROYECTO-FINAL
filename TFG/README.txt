BANCO DE ENSAYO DE ARRANQUE DE MOTORES CON CARGA VARIABLE

ESTRUCTURA ADQUISICION DE DATOS
                                         comunicacion                comunicacion        comunicacion
I (corriente X3)                             SPI                         SPI                 UART    
V (voltaje X3)   ----------> MODULO ADC ------------> RASPBERRY PICO ------------> FTDI-H ----------> COMPUTADORA
RPM (X1)         ---------->  8 CHAN    ------------> 2W RP2350      ------------> FT232H ----------> PYQTGRAPH
T (temperatura X1)            AD7606                                                                  OSCILOSCOPIO

