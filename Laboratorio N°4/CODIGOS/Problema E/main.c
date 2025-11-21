#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <avr/interrupt.h>

// =====================================================
// UART – Bluetooth HC-05 (NO BLOQUEANTE)
// =====================================================

void UART_init(void){
	uint16_t ubrr = 103;     // 9600 baudios
	UBRR0H = (ubrr >> 8);
	UBRR0L = ubrr;

	UCSR0B = (1 << RXEN0) | (1 << TXEN0);   // habilitar RX/TX
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8 bits
}

char rx_buffer[32];
uint8_t rx_index = 0;

void UART_process_line(void (*callback)(char*)){
	if (UCSR0A & (1<<RXC0)){
		char c = UDR0;

		if (c=='\n' || c=='\r'){
			rx_buffer[rx_index] = '\0';
			rx_index = 0;
			callback(rx_buffer);
		}
		else if (rx_index < 31){
			rx_buffer[rx_index++] = c;
		}
	}
}

// =====================================================
// PWM MOTORES (Timer1)
// =====================================================

void PWM_init(){
	DDRB |= (1<<PB1) | (1<<PB2);

	TCCR1A = (1<<WGM10) | (1<<COM1A1) | (1<<COM1B1);
	TCCR1B = (1<<WGM12) | (1<<CS11);

	OCR1A = 0;
	OCR1B = 0;
}

// =====================================================
// PUENTE H DIRECCIONES
// =====================================================

void motores_init(){
	DDRD |= (1<<PD2)|(1<<PD3)|(1<<PD4)|(1<<PD5);
}

void motorL(int v){
	if(v > 0){
		PORTD |=  (1<<PD2);
		PORTD &= ~(1<<PD3);
	}
	else if(v < 0){
		PORTD &= ~(1<<PD2);
		PORTD |=  (1<<PD3);
	}
	else {
		PORTD &= ~(1<<PD2);
		PORTD &= ~(1<<PD3);
	}

	v = abs(v);
	if(v > 255) v = 255;
	OCR1A = v;
}

void motorR(int v){
	if(v > 0){
		PORTD |=  (1<<PD4);
		PORTD &= ~(1<<PD5);
	}
	else if(v < 0){
		PORTD &= ~(1<<PD4);
		PORTD |=  (1<<PD5);
	}
	else {
		PORTD &= ~(1<<PD4);
		PORTD &= ~(1<<PD5);
	}

	v = abs(v);
	if(v > 255) v = 255;
	OCR1B = v;
}

// =====================================================
// BUZZER PASIVO – Timer2 (PD7)
// =====================================================

#define BUZ_PORT PORTD
#define BUZ_DDR  DDRD
#define BUZ_PIN  PD7

void buzzer_init(void){
	BUZ_DDR |= (1<<BUZ_PIN);
	BUZ_PORT &= ~(1<<BUZ_PIN);
}

void buzzer_on(void){
	TCCR2A = (1<<WGM21) | (1<<WGM20);
	TCCR2B = (1<<CS21);

	OCR2A = 80;
	TIMSK2 |= (1<<OCIE2A);
}

void buzzer_off(void){
	TIMSK2 &= ~(1<<OCIE2A);
	TCCR2A = 0;
	TCCR2B = 0;
	BUZ_PORT &= ~(1<<BUZ_PIN);
}

ISR(TIMER2_COMPA_vect){
	BUZ_PORT ^= (1<<BUZ_PIN);
}

// =====================================================
// LUZ EN PD6
// =====================================================

#define LUZ_PORT PORTD
#define LUZ_DDR  DDRD
#define LUZ_PIN  PD6

void luz_init(void){
	LUZ_DDR |= (1<<LUZ_PIN);
	LUZ_PORT &= ~(1<<LUZ_PIN);
}

void luz_on(void){
	LUZ_PORT |= (1<<LUZ_PIN);
}

void luz_off(void){
	LUZ_PORT &= ~(1<<LUZ_PIN);
}

// =====================================================
// SERVOS – PB0 (izq) y PB4 (der)
// =====================================================

#define SERVO_L_PORT PORTB
#define SERVO_L_DDR  DDRB
#define SERVO_L_PIN  PB0

#define SERVO_R_PORT PORTB
#define SERVO_R_DDR  DDRB
#define SERVO_R_PIN  PB4

#define SERVO_TICK_US 20
#define SERVO_MIN_US  500
#define SERVO_MAX_US  2500
#define SERVO_FRAME_US 20000
#define SERVO_FRAME_TICKS (SERVO_FRAME_US / SERVO_TICK_US)

volatile uint16_t servoL_ticks = 0;
volatile uint16_t servoR_ticks = 0;
volatile uint16_t servo_frame = 0;
volatile uint8_t servos_busy = 0;

uint16_t servo_angle_to_ticks(uint8_t angle){
	uint32_t pulse = SERVO_MIN_US +
	((uint32_t)angle * (SERVO_MAX_US - SERVO_MIN_US)) / 180UL;

	return pulse / SERVO_TICK_US;
}

ISR(TIMER0_COMPA_vect){
	if(servo_frame == 0){
		SERVO_L_PORT |= (1<<SERVO_L_PIN);
		SERVO_R_PORT |= (1<<SERVO_R_PIN);
	}

	servo_frame++;

	if(servo_frame >= servoL_ticks) SERVO_L_PORT &= ~(1<<SERVO_L_PIN);
	if(servo_frame >= servoR_ticks) SERVO_R_PORT &= ~(1<<SERVO_R_PIN);

	if(servo_frame >= SERVO_FRAME_TICKS)
	servo_frame = 0;
}

void servos_set_L_angle(uint8_t angle){
	cli();
	servoL_ticks = servo_angle_to_ticks(angle);
	sei();
}

void servos_set_R_angle(uint8_t angle){
	cli();
	servoR_ticks = servo_angle_to_ticks(angle);
	sei();
}

#define SERVO_L_REPOSO 50
#define SERVO_L_PATADA 100

#define SERVO_R_REPOSO 50
#define SERVO_R_PATADA 0

void servos_init(void){
	SERVO_L_DDR |= (1<<SERVO_L_PIN);
	SERVO_R_DDR |= (1<<SERVO_R_PIN);

	SERVO_L_PORT &= ~(1<<SERVO_L_PIN);
	SERVO_R_PORT &= ~(1<<SERVO_R_PIN);

	TCCR0A = (1<<WGM01);
	TCCR0B = (1<<CS01);
	OCR0A = 39;

	TIMSK0 |= (1<<OCIE0A);

	servos_set_L_angle(SERVO_L_REPOSO);
	servos_set_R_angle(SERVO_R_REPOSO);
}

void servo_kick_L(void){
	servos_busy = 1;
	servos_set_L_angle(SERVO_L_PATADA);
	_delay_ms(220);
	servos_set_L_angle(SERVO_L_REPOSO);
	_delay_ms(150);
	servos_busy = 0;
}

void servo_kick_R(void){
	servos_busy = 1;
	servos_set_R_angle(SERVO_R_PATADA);
	_delay_ms(220);
	servos_set_R_angle(SERVO_R_REPOSO);
	_delay_ms(150);
	servos_busy = 0;
}

// =====================================================
// PROCESADOR DE COMANDOS
// =====================================================

int L = 0;
int R = 0;

void procesar_comando(char *buffer){

	// ---- LUZ ----
	if (buffer[0]=='L' && buffer[1]=='E' && buffer[2]=='D' &&
	buffer[3]==':' && buffer[4] && buffer[5]=='\0')
	{
		if (buffer[4]=='1') luz_on();
		else luz_off();
		return;
	}

	// ---- BOCINA ----
	if (buffer[0]=='H' && buffer[1]==':' &&
	(buffer[2]=='0' || buffer[2]=='1') &&
	buffer[3]=='\0')
	{
		if (buffer[2]=='1') buzzer_on();
		else buzzer_off();
		return;
	}

	// ---- PATADAS ----
	if (buffer[0]=='K' && buffer[1]==':' && buffer[3]=='\0'){
		if (servos_busy) return;

		if (buffer[2]=='L'){ servo_kick_L(); return; }
		if (buffer[2]=='R'){ servo_kick_R(); return; }
	}

	// ---- MOTORES ----
	char *pL = strstr(buffer, "L:");
	char *pR = strstr(buffer, "R:");

	if (pL) L = atoi(pL + 2);
	if (pR) R = atoi(pR + 2);

	if (L > 255)  L = 255;
	if (L < -255) L = -255;
	if (R > 255)  R = 255;
	if (R < -255) R = -255;

	motorL(L);
	motorR(R);
}

// =====================================================
// MAIN
// =====================================================

int main(void){

	UART_init();
	PWM_init();
	motores_init();
	buzzer_init();
	servos_init();
	luz_init();   // <--- NUEVO

	sei();

	while(1){
		UART_process_line(procesar_comando);
	}
}
