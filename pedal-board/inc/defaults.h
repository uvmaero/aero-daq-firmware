#ifndef	DEFAULTS_H
#define	DEFAULTS_H

#define	P_MOSI	B,3
#define	P_MISO	B,4
#define	P_SCK	B,5

#define	MCP2515_CS			D,5
#define	MCP2515_INT			D,6

#define PULSE(p, t) DDRC |= (1<<p); PORTC |= (1<<p); _delay_us(t); PORTC &= ~(1<<p); _delay_us(t); DDRC &= ~(1<<p)

#endif	// DEFAULTS_H
