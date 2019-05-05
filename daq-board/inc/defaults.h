#ifndef	DEFAULTS_H
#define	DEFAULTS_H

#define	P_MOSI	B,3
#define	P_MISO	B,4
#define	P_SCK	B,5

#define	MCP2515_CS			D,5
#define	MCP2515_INT			D,6

#define PULSE(p, t) DDRB |= (1<<p); PORTB |= (1<<p); _delay_us(t); PORTB &= ~(1<<p); _delay_us(t);

#endif	// DEFAULTS_H
