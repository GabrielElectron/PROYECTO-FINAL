/*
 * File:   PROGRAMA1.c
 * Author: wintt
 *
 * Created on September 29, 2025, 2:20 PM
 */

#define FCY 4000000UL
#include <xc.h>

#pragma config FNOSC = FRC      // Este bit indica que el micro arrancará con el oscilador RC interno
#pragma config IESO = ON        // Este bit indica que el micro pasara automaticamente a trabajar con el oscilador elegido
#pragma config POSCMD = XT      // Este bit indica el oscilador elegido para trabajar, en este caso XT es uno externo
#pragma config FWDTEN = 0     // Este bit nos indica que tendremos desactivado el watchdog
#pragma config ICS = PGD3       // Este bit indica que seleccionamos el puerto 3 de programacion
#include <libpic30.h>   // Necesaria para __delay_ms()
// Podemos programar estos bits desde la pestaña de production en set configuration bits



int main(void) {
    // Configurar RB4 como salida digital
    TRISBbits.TRISB4 = 0;  
    LATBbits.LATB4 = 0;    // LED apagado al inicio
    
    while (1) {
        LATBbits.LATB4 = 1;   // Enciende LED
        __delay_ms(500);      // Espera 500 ms

        LATBbits.LATB4 = 0;   // Apaga LED
        __delay_ms(500);      // Espera 500 ms
    }
    
    return 0;
}
