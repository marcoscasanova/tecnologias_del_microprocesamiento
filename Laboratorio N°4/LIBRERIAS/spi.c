#ifndef F_CPU  // Verifica si no se ha definido la frecuencia del microcontrolador
#define F_CPU 16000000UL  // Define la frecuencia del reloj del sistema en 16 MHz
#endif  // Fin de la comprobación de F_CPU

#include <avr/io.h>  // Se incluye la librería para acceder a los registros de hardware del microcontrolador AVR
#include <util/delay.h>  // Se incluye para poder utilizar funciones de retardo como _delay_ms() o _delay_us()
#include <stdio.h>  // Se incluye para soporte de funciones estándar de entrada/salida
#include "spi.h"  // Se incluye el archivo de cabecera con las definiciones y prototipos del módulo SPI

void SPI_INICIAR(void) {  // Inicializa el módulo SPI en modo maestro
    DDRB |= (1 << PB2) | (1 << PB3) | (1 << PB5);  // Configura los pines SS (PB2), MOSI (PB3) y SCK (PB5) como salidas
    DDRB &= ~(1 << PB4);  // Configura el pin MISO (PB4) como entrada
    SPCR = (1 << SPE) | (1 << MSTR);  // Habilita el SPI y selecciona el modo maestro
    SPSR = (1 << SPI2X);  // Habilita la velocidad doble de transferencia (fosc/2)
}

uint8_t SPI_TRANSFERIR(uint8_t data) {  // Envía y recibe un byte de datos a través del bus SPI
    SPDR = data;  // Carga el dato a enviar en el registro del SPI (SPDR)
    while (!(SPSR & (1 << SPIF)));  // Espera hasta que la transmisión y recepción se completen (SPIF = 1)
    return SPDR;  // Retorna el dato recibido desde el esclavo SPI
}
