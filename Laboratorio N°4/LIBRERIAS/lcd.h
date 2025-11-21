#ifndef LCD_H  // Se define una directiva de inclusión condicional para evitar múltiples inclusiones del archivo
#define LCD_H  // Marca el inicio del bloque protegido de inclusión

#ifndef F_CPU  // Verifica si no se ha definido la frecuencia del reloj del microcontrolador
#define F_CPU 16000000UL  // Define la frecuencia del CPU en 16 MHz
#endif  // Fin de la comprobación de F_CPU

#include <avr/io.h>  // Se incluye la librería que permite acceder a los registros de E/S del microcontrolador AVR
#include <util/delay.h>  // Se incluye la librería para generar retardos mediante la función _delay_ms() o _delay_us()
#include "i2c.h"  // Se incluye la librería del bus I2C necesaria para la comunicación con el módulo del LCD

#define LCD_DIR          0x27  // Dirección I2C del módulo adaptador del LCD (PCF8574)
#define LCD_BACKLIGHT    0x08  // Bit que activa la retroiluminación del display LCD
#define LCD_ENABLE_BIT   0x04  // Bit que controla la señal de habilitación (EN) del LCD

void LCD_INICIAR(void);  // Prototipo de función para inicializar el LCD en modo 4 bits
void LCD_LIMPIAR(void);  // Prototipo de función para limpiar la pantalla del LCD
void LCD_COMANDO(uint8_t cmd);  // Prototipo de función para enviar comandos al LCD
void LCD_CARACTER(uint8_t data);  // Prototipo de función para enviar un carácter individual al LCD
void LCD_CADENA(const char *s);  // Prototipo de función para mostrar una cadena de texto en el LCD
void LCD_POS(uint8_t fila, uint8_t col);  // Prototipo de función para posicionar el cursor del LCD en una fila y columna específica
void LCD_MOSTRAR(const char *linea1, const char *linea2);  // Prototipo de función para mostrar dos líneas de texto en el LCD

#endif  // Fin de la protección contra inclusiones múltiples del archivo
