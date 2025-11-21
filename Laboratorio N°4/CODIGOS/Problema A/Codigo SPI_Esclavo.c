// esclavo.c - ATmega328P ESCLAVO SPI
// Por cada SS LOW del maestro recibe EXACTAMENTE 3 bytes:
// 1) duty_pwm (0..255) -> PWM motor en PD6 (OC0A / D6)
// 2) flag_temp_mayor (0/1) -> LED temperatura en PD7
// 3) flag_boton (0/1)      -> buzzer bot�n en PD5

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <stdint.h>
#include "pwm.h"

// LED temperatura en PD7
#define LED_TEMP_DDR   DDRD
#define LED_TEMP_PORT  PORTD
#define LED_TEMP_BIT   PD7

// Buzzer bot�n en PD5
#define BUZZER_DDR     DDRD
#define BUZZER_PORT    PORTD
#define BUZZER_BIT     PD5

int main(void)
{
	uint8_t duty_pwm       = 0;
	uint8_t flag_temp      = 0;
	uint8_t flag_boton     = 0;
	uint8_t i;
	uint8_t buffer[3];

	//--- SPI ESCLAVO ---
	DDRB |= (1 << PB4);                               // MISO salida
	DDRB &= ~((1 << PB3) | (1 << PB5) | (1 << PB2));  // MOSI, SCK, SS entradas
	PORTB |= (1 << PB2);                              // pull-up en SS para evitar flotante
	SPCR = (1 << SPE);                                // Habilitar SPI, esclavo, modo 0

	//--- PWM en PD6 (OC0A / D6) ---
	PWM_INICIAR(64);     // prescaler 64, ~1 kHz

	//--- Salidas ---
	LED_TEMP_DDR  |= (1 << LED_TEMP_BIT);
	LED_TEMP_PORT &= ~(1 << LED_TEMP_BIT);

	BUZZER_DDR  |= (1 << BUZZER_BIT);
	BUZZER_PORT &= ~(1 << BUZZER_BIT);

	while (1)
	{
		// Esperar a que el maestro baje SS (activo en 0)
		while (PINB & (1 << PB2))
		{
			// espera hasta que SS sea 0
		}

		// SS est� LOW: el maestro va a mandar 3 bytes
		for (i = 0; i < 3; i++)
		{
			while (!(SPSR & (1 << SPIF)))
			{
				// esperar recepci�n de un byte
			}
			buffer[i] = SPDR;
		}

		// Opcional: esperar hasta que el maestro suelte SS (suba a 1)
		while (!(PINB & (1 << PB2)))
		{
			// espera que SS vuelva a HIGH antes de la pr�xima trama
		}

		// Asignar variables
		duty_pwm  = buffer[0];
		flag_temp = buffer[1];
		flag_boton = buffer[2];

		// Aplicar PWM
		PWM_ESTABLECER_DUTY(duty_pwm);

		// LED temperatura
		if (flag_temp == 1)
		LED_TEMP_PORT |= (1 << LED_TEMP_BIT);
		else
		LED_TEMP_PORT &= ~(1 << LED_TEMP_BIT);

		// Buzzer bot�n
		if (flag_boton == 1)
		BUZZER_PORT |= (1 << BUZZER_BIT);
		else
		BUZZER_PORT &= ~(1 << BUZZER_BIT);
	}

	return 0;
}

