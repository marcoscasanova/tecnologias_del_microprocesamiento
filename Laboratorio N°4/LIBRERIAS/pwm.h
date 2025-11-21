#ifndef PWM_H  // Se define una directiva de inclusión condicional para evitar múltiples inclusiones del archivo
#define PWM_H  // Marca el inicio del bloque protegido de inclusión

#include <avr/io.h>  // Se incluye la librería que permite acceder a los registros de control del microcontrolador AVR
#include <stdint.h>  // Se incluye la librería estándar que define tipos de datos con tamaño fijo (uint8_t, uint16_t, etc.)

void PWM_INICIAR(unsigned int prescaler);  // Prototipo de función para inicializar el módulo PWM con un prescaler determinado
void PWM_ESTABLECER_DUTY(uint8_t duty);  // Prototipo de función para establecer el ciclo de trabajo (duty cycle) del PWM
void PWM_DETENER(void);  // Prototipo de función para detener la generación de la señal PWM

#endif  // Fin de la protección contra inclusiones múltiples del archivo
