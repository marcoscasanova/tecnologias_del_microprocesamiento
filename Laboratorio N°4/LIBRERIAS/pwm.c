#include "pwm.h"  // Se incluye el archivo de cabecera con las definiciones y prototipos del módulo PWM

void PWM_INICIAR(unsigned int prescaler) {  // Inicializa el PWM con el prescaler especificado
    DDRD |= (1 << PD6);  // Configura el pin PD6 (OC0A) como salida para la señal PWM
    
    TCCR0A = (1 << WGM01) | (1 << WGM00) | (1 << COM0A1);  // Configura el Timer0 en modo Fast PWM y habilita la salida no invertida en OC0A
    TCCR0B = 0;  // Limpia el registro de control B para configurar el prescaler después

    switch (prescaler) {  // Selecciona el valor del prescaler según el argumento recibido
        case 1:    TCCR0B |= (1 << CS00); break;  // Sin prescaler (frecuencia máxima)
        case 8:    TCCR0B |= (1 << CS01); break;  // Prescaler de 8
        case 64:   TCCR0B |= (1 << CS01) | (1 << CS00); break;  // Prescaler de 64
        case 256:  TCCR0B |= (1 << CS02); break;  // Prescaler de 256
        case 1024: TCCR0B |= (1 << CS02) | (1 << CS00); break;  // Prescaler de 1024
        default:   TCCR0B |= (1 << CS01) | (1 << CS00); break;  // Valor por defecto: prescaler de 64
    }

    OCR0A = 0;  // Inicializa el registro de comparación (duty cycle) en 0
}

void PWM_ESTABLECER_DUTY(uint8_t duty) {  // Ajusta el ciclo de trabajo (duty cycle) del PWM
    OCR0A = duty;  // Asigna el valor recibido al registro OCR0A (0–255)
}

void PWM_DETENER(void) {  // Detiene la generación de la señal PWM
    TCCR0A = 0;  // Desactiva el modo PWM limpiando el registro de control A
    TCCR0B = 0;  // Detiene el temporizador desactivando el reloj
    DDRD &= ~(1 << PD6);  // Configura nuevamente el pin PD6 como entrada
}
