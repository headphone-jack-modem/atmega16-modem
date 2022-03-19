/*
 * Title:    AVR Modem library (avr-modem.c)
 * Author:   Hasan Karimi <hasankarimi.dev@gmail.com>
 */
#include <avr/io.h>
#include <stdint-gcc.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#include "modem.h"

#define BAUD_ICR1                            ((F_CPU / 1.0) / BAUD)

#define LOW_FREQ_ICR1                        ((F_CPU / 1.0) / LOW_FREQ)                         // the ICR1 counted during a low frequency
#define HIGH_FREQ_ICR1                       ((F_CPU / 1.0) / HIGH_FREQ)                        // the ICR1 counted during a high frequency
#define High_PER_LOW                         (1.0 * HIGH_FREQ / LOW_FREQ)                       // high frequency to low frequency ratio
#define LOW_TO_HIGH_ICR1_DIFF                (LOW_FREQ_ICR1 - HIGH_FREQ_ICR1)                   // difference between ICR1 of low frequency and high frequency
#define LOW_FREQ_ICR1_R                      (LOW_TO_HIGH_ICR1_DIFF / (1.0 / High_PER_LOW + 1)) // the allowed difference of ICR1 for the low frequency
#define HIGH_FREQ_ICR1_R                     (LOW_TO_HIGH_ICR1_DIFF - LOW_FREQ_ICR1_R)          // the allowed difference of ICR1 for the high frequency
#define MIN_LOW_FREQ_ICR1                    (LOW_FREQ_ICR1 - LOW_FREQ_ICR1_R)                  // the minimum amount of ICR1 for the low frequency
#define MAX_LOW_FREQ_ICR1                    (LOW_FREQ_ICR1 + LOW_FREQ_ICR1_R)                  // the maximum amount of ICR1 for the low frequency
#define MIN_HIGH_FREQ_ICR1                   (HIGH_FREQ_ICR1 - HIGH_FREQ_ICR1_R)                // the minimum amount of ICR1 for the high frequency
#define MAX_HIGH_FREQ_ICR1                   (HIGH_FREQ_ICR1 + HIGH_FREQ_ICR1_R)                // the maximum amount of ICR1 for the high frequency
#define LOW_FREQ_CNT_PER_BAUD                (LOW_FREQ / BAUD)                                  // the count of the low frequency per each baud
#define HIGH_FREQ_CNT_PER_BAUD               (HIGH_FREQ / BAUD)                                 // the count of the high frequency per each baud

// received bits status
#define RX_BIT_STAT_START                    0
#define RX_BIT_STAT_DATA                     8
#define RX_BIT_STAT_CONTROL                  9
#define RX_BIT_STAT_PARITY                   10
#define RX_BIT_STAT_STOP                     11
#define RX_BIT_STAT_INACTIVE                 0xFF
uint8_t g_rx_bit_status = RX_BIT_STAT_INACTIVE;

// the ICR1 counted during a low or high frequency will be added to this place,
// so at the end of a baud we can find out which bit is received, is it 0 or 1?
uint16_t g_low_count;
uint16_t g_high_count;

uint16_t g_inf = 0;
uint16_t info() {
	return g_inf;
}

// this function be called when a message received
void (*g_listen_callback)(const char* msg, uint8_t size) = 0;

// send queue (FIFO structure)
uint16_t* g_tx_buff[TX_BUFF_SIZE];
uint8_t g_tx_head = 0;
uint8_t g_tx_tail = 0;

void modulate(void) {

	// initial with preamble values (two high bits)
	static uint8_t half_freq_cnt = 12 * HIGH_FREQ_CNT_PER_BAUD * 2;
	static uint16_t half_freq_icr1 = HIGH_FREQ_ICR1 / 2;
	static uint16_t byte = 0;  // a copy of the current byte of the current stream that is sending
	static uint8_t size = 0;  // size of the current stream
	static uint8_t index = 0;  // point to the next byte of the current stream that will be sent

	// if isn't at the end of the baud
	if (--half_freq_cnt) {
		OCR1B += half_freq_icr1;
		return;
	}

	// if the byte was zero
	if (!byte) {

		if (g_tx_head == g_tx_tail) {
			// that's mean the sending buffer is empty,
			// so we switch compare output mode for channel B to normal port operation (OC1B disconnected)
			// and disable the OCR1B and TCNT1 compare match interrupt
			TCCR1A &= ~( 1 << COM1B0);                               // TCCR1B   ---0----
			TCCR1A &= ~( 1 << COM1B1);                               // TCCR1B   --0-----
			TIMSK &= ~(1 << OCIE1B);                                 // TIMSK    ----0---

			// adjust the following variables for the next time that we want to send a message
			// initial with preamble values (two high bits)
			half_freq_cnt = 12 * HIGH_FREQ_CNT_PER_BAUD * 2;
			half_freq_icr1 = HIGH_FREQ_ICR1 / 2;

#if MODEM_DEBUG
			PORTA &= ~(1 << 1);
#endif
			if (g_listen_callback)
				ACSR |= (1 << ACIC);
			return;
		}

		// if the index was zero
		if (!index)
			size = g_tx_buff[g_tx_head][0] >> 1;

		byte = g_tx_buff[g_tx_head][index++];

		if (index > size) {
			free(g_tx_buff[g_tx_head++]);
			g_tx_head &= TX_BUFF_SIZE - 1;
			index = 0;
		}
	}

	auto void adjust_for(uint8_t bit);
	adjust_for(byte & 1);
	OCR1B += half_freq_icr1;
	byte >>= 1;

	// adjust the half frequency counter and the half frequency ICR1 based on the expected bit
	void adjust_for(uint8_t bit) {
		if (bit) {
			half_freq_cnt = HIGH_FREQ_CNT_PER_BAUD * 2;
			half_freq_icr1 = HIGH_FREQ_ICR1 / 2;
		} else {
			half_freq_cnt = LOW_FREQ_CNT_PER_BAUD * 2;
			half_freq_icr1 = LOW_FREQ_ICR1 / 2;
		}
	}
}

// timer 1 compare match interrupt B
ISR(TIMER1_COMPB_vect) {
	modulate();
}

// make a stream of uint16_t from message for sending
uint16_t* mk_stream(const char* msg, uint8_t size) {

	auto uint16_t mk_stream_byte(char byte, uint8_t b_ctrl_bit);

	uint16_t* stream = (uint16_t*) malloc((size + 1) * sizeof(uint16_t));

	stream[0] = mk_stream_byte(size, 1);

	for (int i = 0; i < size; i++)
		stream[i + 1] = mk_stream_byte(msg[i], 0);

	uint16_t mk_stream_byte(char byte, uint8_t b_ctrl_bit) {
		uint16_t pbit = 1;                               // odd parity bit
		uint16_t d = byte;                               // data         --------xxxxxxxx
		d <<= 1;                                         // start bit    -------xxxxxxxx0
		d |= b_ctrl_bit ? 0b0000001000000000 : 0;        // control bit  ------cxxxxxxxx0

		while (byte) {
			pbit ^= byte;
			byte >>= 1;
		}
		pbit ^= b_ctrl_bit;
		pbit <<= 10;
		pbit &= 0b0000010000000000;
		d |= pbit;                                       // parity bit   -----pcxxxxxxxx0
		d |= 0b0000100000000000;                         // stop bit     ----1pcxxxxxxxx0

		return d;
	}

	return stream;
}

uint8_t modem_snd_async(const char* msg, uint8_t size) {

	// check the buffer
	uint8_t new_buff_tail = (g_tx_tail + 1) & (TX_BUFF_SIZE - 1);
	// if the buffer is full, return 1,
	if (new_buff_tail == g_tx_head)
		return 1;

	g_tx_buff[g_tx_tail] = mk_stream(msg, size);
	g_tx_tail = new_buff_tail;

	if (TIMSK & (1 << OCIE1B))
		// that's mean, OCR1B and TCNT1 compare match interrupt is enabled before, so do nothing
		return 0;

	// adjust PD4 port for FSKOUT (connected to the microphone), wave transmitter		<== ~~ |PD4
	DDRD |= (1 << 4);
	PORTD &= ~(1 << 4);

	// adjust timer 1 mode to normal (0000)
	// start from 0 to 0xffff and overflowing
	TCCR1A &= ~((1 << WGM11) | (1 << WGM10));               // TCCR1A   ------00
	TCCR1B &= ~((1 << WGM13) | (1 << WGM12));               // TCCR1B   ---00---

	// adjust timer 1 clock to 1/1 of micro-controller clock ==> 1/1 * 8000000 = 8000000
	TCCR1B |= (1 << CS10);                                  // TCCR1B   -------1
	TCCR1B &= ~((1 << CS12) | (1 << CS11));                 // TCCR1B   -----00-


	// switch compare output mode for channel B to Toggle OC1B (PD4) on compare match
	TCCR1A |= ( 1 << COM1B0);                               // TCCR1B   ---1----
	TCCR1A &= ~( 1 << COM1B1);                              // TCCR1B   --0-----

	// we need aware of at the half of the each frequency,
	// so calculate the time that the "timer 1 compare match" must be interrupted, and set into the OCR1B
	OCR1B = TCNT1 + HIGH_FREQ_ICR1 / 2;

	// timer interrupt flag of OCR1A and TCNT1 compare match
	// will be 0 when try to set it 1
	TIFR |= (1 << OCF1B);                                   // TIFR     ----0---

//	ACSR &= ~(1 << ACIC);
#if MODEM_DEBUG
	PORTA |= (1 << 1);
#endif
	// enable OCR1B and TCNT1 compare match interrupt
	TIMSK |= (1 << OCIE1B);                                 // TIMSK    ----1---

	return 0;
}

void route_msg(const char *msg, uint8_t size) {
	if (g_listen_callback) g_listen_callback(msg, size);
}

void rx_byte(uint8_t data_byte, uint8_t ctrl_bit) {

	static char* msg;
	static uint8_t msg_size = 0;
	static uint8_t msg_index = 0;

	if (ctrl_bit) {
		// catch message size from the data byte
		msg_size = data_byte;
		g_inf = msg_size;
		if (msg) free(msg);
		msg_index = 0;
		msg = 0;
		if (msg_size) msg = (char*) malloc(msg_size * sizeof(char));
		return;
	}

	if (msg_size) {
		*(msg + msg_index++) = data_byte;
		if (msg_index == msg_size) {
			route_msg(msg, msg_size);
			free(msg);
			msg_size = msg_index = 0;
			msg = 0;
		}
	}
}

void rx_bit(void) {

	static uint8_t data_byte;
	static uint8_t b_ctrl_bit;        // control bit
	static uint8_t pbit;              // odd parity bit

	// bit logic determination
	uint8_t bit;

	if (g_high_count < g_low_count) {
		bit = 0b00000000;
		g_low_count = 0;
	} else {
		bit = 0b10000000;
		g_high_count = 0;
	}

	// start bit reception
	if (g_rx_bit_status == RX_BIT_STAT_START && !bit) {
		pbit = 0b10000000;
		g_rx_bit_status++;
		return;
	}

	// data bit reception
	if (g_rx_bit_status <= RX_BIT_STAT_DATA) {
		data_byte >>= 1;
		data_byte |= bit;
		pbit ^= bit;
		g_rx_bit_status++;
		return;
	}

	// control bit reception
	if (g_rx_bit_status == RX_BIT_STAT_CONTROL) {
		b_ctrl_bit = bit;
		pbit ^= bit;
		g_rx_bit_status++;
		return;
	}

	// Parity bit reception
	if (g_rx_bit_status == RX_BIT_STAT_PARITY && pbit == bit) {
		g_rx_bit_status++;
		return;
	}

	if (g_rx_bit_status == RX_BIT_STAT_STOP && bit)
		rx_byte(data_byte, b_ctrl_bit);

	// disable OCR1A and TCNT1 compare match interrupt
	// because we don't expect waiting anymore at the end of the baud to receive the bit
	TIMSK &= ~(1 << OCIE1A);						// TIMSK	---0----
	g_rx_bit_status = RX_BIT_STAT_INACTIVE;
#if MODEM_DEBUG
	PORTA &= ~(1 << 0);
#endif
}

// timer 1 compare match interrupt A
ISR(TIMER1_COMPA_vect) {
	// now we are at the end of a baud
	// first of all, we calculate the time of the next baud
	OCR1A += BAUD_ICR1;
	// in the second step, we must determine the location of the received bit
	// is that start bit, data bit, control bit, parity bit, or none?
	rx_bit();
}

void demod(void) {

	// hold the previous value of the ICR1 (Input Capture Register 1)
	static uint16_t last_icr1 = 0;
	// calculate the difference between ICR1 and the previous value,
	// so we can find out the length of the incoming wave
	uint16_t diff = ICR1 - last_icr1;

	// if the wavelength is too short, then return
	if (diff < MIN_HIGH_FREQ_ICR1)
		return;

	last_icr1 = ICR1;

	// if the wavelength is too long, then return
	if (diff > MAX_LOW_FREQ_ICR1)
		return;

	// zero's wave
	if (diff > MIN_LOW_FREQ_ICR1) {
		g_low_count += diff;
		if (g_rx_bit_status == RX_BIT_STAT_INACTIVE)
			// Start bit detection
			if (g_low_count > (BAUD_ICR1 / 2)) {

				g_high_count = 0;

				// we need aware of at the end of the baud,
				// so calculate the time that the "timer 1 compare match" must be interrupted, and set into OCR1A
				OCR1A = ICR1 + BAUD_ICR1 - g_low_count;

				// timer interrupt flag of OCR1A and TCNT1 compare match
				// will be 0 when try to set it 1
				TIFR |= (1 << OCF1A);               // TIFR     ---0----

				g_rx_bit_status = RX_BIT_STAT_START;
#if MODEM_DEBUG
				PORTA |= (1 << 0);
#endif
				// enable OCR1A and TCNT1 compare match interrupt
				TIMSK |= (1 << OCIE1A);             // TIMSK	---1----
			}
	}
	// one's wave
	else if (g_rx_bit_status == RX_BIT_STAT_INACTIVE)
		g_low_count = g_high_count = 0;
	else
		g_high_count += diff;
}

// timer 1 input capture interrupt
// we connected the analog comparator to the "timer 1 input capture"
// so every time that the analog comparator be matched this interrupt will be activated
ISR(TIMER1_CAPT_vect) {
	// now we know that a wave is received,
	// so we need to find out that this wave is for what bit? 0 or 1
	demod();
}

void modem_lsn(void (*lsn_cb)(const char* msg, uint8_t size)) {

	g_listen_callback = lsn_cb;
	// make PB2 (AIN0) as an input port for FSKIN receive waves (connect it to the speaker)
	DDRB &= ~(1 << 2);    // --0-----
	PORTB &= ~(1 << 2);   // --0-----
	// make PA2 (AIN1) as an input port for a static voltage (connect it to a static voltage)
	// the static voltage will be used in the analog comparator
	DDRA &= ~(1 << 2);    // --0-----
	PORTA &= ~(1 << 2);   // --0-----

	g_rx_bit_status = RX_BIT_STAT_INACTIVE;
	g_low_count = g_high_count = 0;

	// adjust timer 1 mode to normal (0000)
	// start from 0 to 0xffff and overflowing
	TCCR1A &= ~((1 << WGM11) | (1 << WGM10));               // TCCR1A   ------00
	TCCR1B &= ~((1 << WGM13) | (1 << WGM12));               // TCCR1B   ---00---

	// adjust timer 1 clock to 1/1 of micro-controller clock ==> 1/1 * 8000000 = 8000000
	TCCR1B |= (1 << CS10);                                  // TCCR1B   -------1
	TCCR1B &= ~((1 << CS12) | (1 << CS11));                 // TCCR1B   -----00-

	// adjust comparator interrupt on rising output edge
	ACSR |= (1 << ACIS1) | (1 << ACIS0);                    // ACSR     ------11

	// enable analog comparator multiplexer so we can select ADC2 (PA2) as negative input (AIN1)
	SFIOR |= (1 << ACME);                                   // SFIOR    ----1---
	// select ADC2 (PA2) as negative input (AIN1)
	ADMUX |= ~((1 << MUX2) | (1 << MUX0));                  // ADMUX    -----0-0
	ADMUX |= (1 << MUX1);                                   // ADMUX    ------1-

	// enable timer 1 input capture interrupt
	// after the trigger, the amount of TCNT1 is immediately copied to the ICR1
	TIMSK |= (1 << TICIE1);

	// enable analog comparator input capture
	// so instead of the PD6, the "timer 1 input capture" must be triggered by the analog comparator
	// we must enable the "timer 1 input capture interrupt" to be aware of the trigger
	ACSR |= (1 << ACIC);                                    // ACSR     -----1--
}
