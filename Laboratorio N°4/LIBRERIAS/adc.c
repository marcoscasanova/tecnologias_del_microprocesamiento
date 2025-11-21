#include "adc.h"  // Se incluye el archivo de cabecera del ADC con las definiciones y prototipos necesarios

void ADC_INICIAR(void) {
    ADMUX = (1 << REFS0);  // Se selecciona AVCC como tensión de referencia para el ADC (bit REFS0 = 1)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Se habilita el ADC y se configura el prescaler en 128 para obtener una frecuencia adecuada de muestreo
}

uint16_t ADC_LEER_CANAL(uint8_t canal) {
    ADMUX = (ADMUX & 0xF0) | (canal & 0x0F);  // Se limpia la selección anterior de canal y se carga el nuevo canal manteniendo la referencia establecida
    ADCSRA |= (1 << ADSC);  // Se inicia una nueva conversión analógica-digital
    while (ADCSRA & (1 << ADSC));  // Se espera hasta que la conversión finalice (el bit ADSC se limpia automáticamente)
    return ADC;  // Se devuelve el valor digital de 10 bits obtenido de la conversión
}
