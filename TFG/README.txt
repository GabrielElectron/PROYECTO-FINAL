BANCO DE ENSAYO DE ARRANQUE DE MOTORES CON CARGA VARIABLE

ESTRUCTURA ADQUISICION DE DATOS
                                            comunicación              comunicación               comunicación        comunicación
I (corriente X3)                                SPI                      SPI                          SPI                  
V (voltaje X3)   ----------> CONVERSOR      -----------> MODULO ADC ------------> RASPBERRY PICO ------------> FTDI-H ----------> COMPUTADORA
RPM (X1)         ----------> LOGICO 5 A 3.3V-----------> 8 CHAN     ------------> 2W RP2350      ------------> FT232H ----------> PYQTGRAPH
T (temperatura X1)           74LVC245                    AD7606                                                                  OSCILOSCOPIO

