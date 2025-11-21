#ifndef F_CPU  // Verifica si la frecuencia del CPU no ha sido definida previamente
#define F_CPU 16000000UL  // Define la frecuencia del reloj del microcontrolador en 16 MHz
#endif  // Fin de la verificación de definición de F_CPU

#include <string.h>  // Se incluye la librería estándar de manejo de cadenas de caracteres
#include "i2c.h"  // Se incluye el archivo de cabecera que contiene las definiciones y funciones del módulo I2C

void I2C_INICIAR(void) {
    TWSR = 0x00;  // Se configura el registro de estado del TWI (I2C) sin prescaler
    TWBR = (uint8_t)I2C_TWBR_VALUE;  // Se establece la tasa de transferencia (bit rate) calculada en la macro I2C_TWBR_VALUE
}

void I2C_INICIAR_CONDICION(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);  // Se envía la condición de inicio (START) habilitando el módulo TWI
    while (!(TWCR & (1 << TWINT)));  // Se espera a que la operación finalice (bit TWINT = 1 indica finalización)
}

void I2C_DETENER_CONDICION(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);  // Se envía la condición de parada (STOP)
    while (TWCR & (1 << TWSTO));  // Se espera hasta que el bit STOP se limpie, indicando que la transmisión terminó
}

void I2C_ENVIAR_BYTE(uint8_t dato) {
    TWDR = dato;  // Se carga el dato a enviar en el registro de datos TWI
    TWCR = (1 << TWINT) | (1 << TWEN);  // Se inicia la transmisión habilitando el TWI y limpiando la bandera de interrupción
    while (!(TWCR & (1 << TWINT)));  // Se espera a que la transmisión del byte finalice
}

uint8_t I2C_RECIBIR_BYTE_ACK(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);  // Se configura para recibir un byte y enviar una señal ACK al finalizar
    while (!(TWCR & (1 << TWINT)));  // Se espera a que la recepción finalice
    return TWDR;  // Se retorna el dato recibido desde el bus I2C
}

uint8_t I2C_RECIBIR_BYTE_NACK(void) {
    TWCR = (1 << TWINT) | (1 << TWEN);  // Se configura para recibir un byte sin enviar ACK (NACK)
    while (!(TWCR & (1 << TWINT)));  // Se espera a que la recepción finalice
    return TWDR;  // Se retorna el dato recibido desde el bus I2C
}
