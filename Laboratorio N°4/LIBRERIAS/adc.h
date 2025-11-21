#ifndef ADC_H  // Se define una directiva de preprocesador para evitar inclusiones múltiples del mismo archivo
#define ADC_H  // Se indica el inicio del bloque de protección de inclusión

#include <avr/io.h>  // Se incluye la librería principal de E/S del AVR que permite el acceso a los registros del microcontrolador
#include <stdint.h>  // Se incluye la librería estándar para el uso de tipos de datos enteros con tamaño definido (uint8_t, uint16_t, etc.)

void ADC_INICIAR(void);  // Prototipo de función para inicializar el módulo ADC
uint16_t ADC_LEER_CANAL(uint8_t canal);  // Prototipo de función para leer un canal analógico específico y retornar el valor convertido

#endif  // Fin de la protección contra inclusión múltiple del archivo
