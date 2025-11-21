#include "lcd.h"  // Se incluye el archivo de cabecera con las definiciones y prototipos del manejo de la pantalla LCD

static void lcd_enviarNibble(uint8_t nib, uint8_t modo) {  // Función interna para enviar un nibble (4 bits) al LCD con el modo indicado (comando o dato)
    uint8_t data = nib | LCD_BACKLIGHT | modo;  // Se prepara el dato combinando el nibble, la retroiluminación y el modo
    I2C_INICIAR_CONDICION();  // Se genera la condición de inicio I2C
    I2C_ENVIAR_BYTE(LCD_DIR << 1);  // Se envía la dirección del dispositivo LCD desplazada a la izquierda (para indicar escritura)
    I2C_ENVIAR_BYTE(data | LCD_ENABLE_BIT);  // Se activa la señal de enable (EN) para enviar el nibble
    _delay_us(1);  // Pequeña pausa para asegurar el tiempo de activación
    I2C_ENVIAR_BYTE(data & ~LCD_ENABLE_BIT);  // Se desactiva la señal de enable (EN)
    I2C_DETENER_CONDICION();  // Se genera la condición de parada I2C
    _delay_us(50);  // Retardo breve para permitir que el LCD procese el dato
}

void LCD_COMANDO(uint8_t cmd) {  // Envía un comando al LCD
    lcd_enviarNibble(cmd & 0xF0, 0x00);  // Se envían los 4 bits altos con modo comando
    lcd_enviarNibble((cmd << 4) & 0xF0, 0x00);  // Se envían los 4 bits bajos con modo comando
}

void LCD_CARACTER(uint8_t data) {  // Envía un carácter al LCD
    lcd_enviarNibble(data & 0xF0, 0x01);  // Se envían los 4 bits altos en modo datos
    lcd_enviarNibble((data << 4) & 0xF0, 0x01);  // Se envían los 4 bits bajos en modo datos
}

void LCD_CADENA(const char *s) {  // Envía una cadena de caracteres al LCD
    while (*s) LCD_CARACTER(*s++);  // Recorre cada carácter de la cadena y lo envía al LCD
}

void LCD_POS(uint8_t fila, uint8_t col) {  // Posiciona el cursor del LCD en una fila y columna determinada
    uint8_t addr = (fila == 0 ? 0x80 : 0xC0) + col;  // Calcula la dirección DDRAM según la fila y la columna
    LCD_COMANDO(addr);  // Envía el comando para mover el cursor a la posición deseada
}

void LCD_LIMPIAR(void) {  // Limpia la pantalla del LCD
    LCD_COMANDO(0x01);  // Comando de limpieza total del display
    _delay_ms(2);  // Retardo necesario para que el LCD complete la operación
}

void LCD_INICIAR(void) {  // Inicializa el LCD en modo 4 bits
    _delay_ms(15);  // Retardo inicial para permitir que el LCD se estabilice después del encendido
    LCD_COMANDO(0x33);  // Secuencia de inicialización: modo 8 bits (inicio del proceso)
    LCD_COMANDO(0x32);  // Cambio a modo 4 bits
    LCD_COMANDO(0x28);  // Configuración: 2 líneas y formato de 5x8 puntos
    LCD_COMANDO(0x0C);  // Encendido del display sin cursor
    LCD_COMANDO(0x06);  // Movimiento automático del cursor hacia la derecha
    LCD_COMANDO(0x01);  // Limpieza del display
    _delay_ms(2);  // Retardo final para estabilizar
}

void LCD_MOSTRAR(const char *linea1, const char *linea2) {  // Muestra dos líneas de texto en el LCD
    LCD_LIMPIAR();  // Limpia la pantalla antes de mostrar el nuevo contenido
    LCD_POS(0, 0);  // Posiciona el cursor en la primera línea
    LCD_CADENA(linea1);  // Muestra la primera cadena
    LCD_POS(1, 0);  // Posiciona el cursor en la segunda línea
    LCD_CADENA(linea2);  // Muestra la segunda cadena
}
