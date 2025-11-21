#ifndef I2C_H  // Se define una directiva de inclusión condicional para evitar que el archivo se incluya múltiples veces
#define I2C_H  // Marca el inicio del bloque protegido de inclusión

#include <avr/io.h>  // Se incluye la librería que permite el acceso a los registros del microcontrolador AVR
#include <stdint.h>  // Se incluye la librería estándar para el uso de tipos de datos con tamaño fijo (uint8_t, uint16_t, etc.)

#ifndef F_CPU  // Verifica si no está definida la frecuencia del CPU
#define F_CPU 16000000UL  // Define la frecuencia del reloj del microcontrolador en 16 MHz
#endif  // Fin de la comprobación de F_CPU

#ifndef I2C_FREC  // Verifica si no está definida la frecuencia de operación del bus I2C
#define I2C_FREC 100000UL  // Define la frecuencia estándar del bus I2C en 100 kHz
#endif  // Fin de la comprobación de I2C_FREC

#define I2C_TWBR_VALUE ((((F_CPU / I2C_FREC) - 16) / 2))  // Cálculo del valor del registro TWBR para obtener la frecuencia I2C deseada

void I2C_INICIAR(void);  // Prototipo de función para inicializar el módulo I2C (TWI)
void I2C_INICIAR_CONDICION(void);  // Prototipo de función para generar una condición de inicio (START) en el bus I2C
void I2C_DETENER_CONDICION(void);  // Prototipo de función para generar una condición de parada (STOP) en el bus I2C
void I2C_ENVIAR_BYTE(uint8_t dato);  // Prototipo de función para enviar un byte a través del bus I2C
uint8_t I2C_RECIBIR_BYTE_ACK(void);  // Prototipo de función para recibir un byte y responder con ACK
uint8_t I2C_RECIBIR_BYTE_NACK(void);  // Prototipo de función para recibir un byte y responder con NACK

#endif  // Fin de la protección contra inclusiones múltiples del archivo
