#ifndef SPI_H  // Se define una directiva de inclusión condicional para evitar múltiples inclusiones del archivo
#define SPI_H  // Marca el inicio del bloque protegido de inclusión

#ifndef F_CPU  // Verifica si no se ha definido la frecuencia del reloj del microcontrolador
#define F_CPU 16000000UL  // Define la frecuencia del CPU en 16 MHz
#endif  // Fin de la comprobación de F_CPU

#include <avr/io.h>  // Se incluye la librería que permite acceder a los registros del periférico SPI del AVR
#include <util/delay.h>  // Se incluye para utilizar funciones de retardo (_delay_ms, _delay_us)
#include <stdio.h>  // Se incluye para permitir operaciones de depuración o impresión si se requiere

void SPI_INICIAR(void);  // Prototipo de función para inicializar el módulo SPI en modo maestro
uint8_t SPI_TRANSFERIR(uint8_t data);  // Prototipo de función para enviar y recibir un byte a través del bus SPI

#endif  // Fin de la protección contra inclusiones múltiples del archivo
