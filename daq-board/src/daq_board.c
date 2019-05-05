/**
 * Copyright 2019 Cullen Jemison, UVM AERO
 * 
 * Data Acquisition Front/Rear board firmware.
 * 
 * Processes data from damper position sensors and wheel speed sensors.
 */

#include <stdbool.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/sfr_defs.h>
#include <avr/wdt.h>
#include "inc/mcp2515.h"
#include "inc/defaults.h"
#include "inc/global.h"
#include "inc/mcp2515_defs.h"
#include "inc/can_ids.h"

// CAN IDs
#define ID_RESET 0x6C

#define NUM_SAMPLES 10

// analog input pin constants
#define PIN_DAMPER_LEFT 0
#define PIN_DAMPER_RIGHT 2

// digital input pin constants
#define PIN_WHEEL_SPEED_LEFT 1  // PORTC
#define PIN_WHEEL_SPEED_RIGHT 3  // PORTC
#define PIN_FR_SELECT 0  // PORTD

// wheel speed sensor constants
#define WHEEL_SPEED_PULSES_PER_REV 1
#define WHEEL_SPEED_SAMPLE_RATE 8000  // sampling rate (Hz). Slightly faster than Nyquist for wheel speed sensing
#define WHEEL_SPEED_SENSE_CIRCUMFERENCE 31415  // in mils (thousandths of an inch)
#define WHEEL_SPEED_DIST_PER_PULSE WHEEL_SPEED_SENSE_CIRCUMFERENCE / WHEEL_SPEED_PULSES_PER_REV
#define WHEEL_SPEED_DEBOUNCE 10  // number of samples used for debouncing
#define MILS_PER_SECOND_TO_MILES_PER_HOUR 17600

// raw ADC values from each sensor
uint16_t damper_left=0, damper_right=0;

// wheel speed
uint8_t wheel_speed_left, wheel_speed_right;

// wheel speed calculation variables
uint16_t wsl_samples_since_last = 0, wsr_samples_since_last = 0;
uint16_t wsl_debounce_samples = 0, wsr_debounce_samples = 0;
bool wsl_detected = false, wsr_detected = false;

uint8_t sample_accumulator = 0;

// stores whether the device is being used as the front or rear datalogger. This simply changes the CAN IDs.
bool front_rear = false;

// Directly from Arduino
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main(void) {
    // set PB2 as an output (so that SS won't get driven low and mess up SPI)
    DDRB |= (1 << PB2);

    // initialize the ADC
    ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADIE) | (7 << ADPS0);  // enable ADC, enable triggering, prescaler F_cpu/128
    ADCSRB = 0;  // freerunning mode
    ADMUX = (1 << REFS0) | (PIN_DAMPER_LEFT);  // AVcc reference, start mux at pedal0
    ADCSRA |= (1 << ADSC);  // start conversionn

    // initialize TIMER0 for 8kHz
    TCCR0A = 0;
    TCCR0B = (0x02 << CS00);  // f_timer = f_cpu/8 => 1MHz
    OCR0A = 125;  // set comparator A
    TIMSK0 = (1 << OCIE0A);  // enable comparator A interrupt

    // initialize TIMER1 for 100Hz
    TCCR1A = 0;
    TCCR1B = (0x02 << CS10);  // f_timer = f_cpu/8 => 1MHz
    OCR1A = 10000;  // set comparator A
    TIMSK1 = (1 << OCIE1A);  // enable comparator A interrupt

    // turn off interrupts while setting up the MCP2515
    cli();

    // wait until the MCP2515 is initialized
    while (!mcp2515_init()) _delay_ms(100);

    // enable interrupts
    sei();

    // Determine if the device is in front or rear mode
    front_rear = bit_is_set(PORTD, PIN_FR_SELECT);

    // Enable the watchdog timer
    //wdt_enable(WDTO_1S);

    tCAN message;
    while (1) {
        if (mcp2515_get_message(&message)) {
            if (message.id == ID_RESET) {
                // reset the device by failing to reset the watchdog
                cli();
                wdt_enable(WDTO_15MS);
                while (1) continue;
            }
        }
        _delay_ms(10);
        // reset the watchdog
        wdt_reset();
    }
}

ISR(ADC_vect) {
    // check the mux to see which sensor was sampled
    if (sample_accumulator == NUM_SAMPLES) {
        switch (ADMUX & 0x0F) {
            case PIN_DAMPER_LEFT: {
                damper_left = ADC;  // read the ADC
                ADMUX = (ADMUX & 0xF0) | PIN_DAMPER_RIGHT;  // switch the mux to the next pin to read
                break;
            }
            case PIN_DAMPER_RIGHT: {
                damper_right = ADC;  // read the ADC
                ADMUX = (ADMUX & 0xF0) | PIN_DAMPER_LEFT;  // switch the mux to the next pin to read
                break;
            }
            default: {
                ADMUX = (ADMUX & 0xF0) | PIN_DAMPER_LEFT;
            }
        }
        sample_accumulator = 0;
    } else {
        sample_accumulator++;
    }
}

// Consistent interrupt for sampling wheel speed sensors
ISR(TIMER0_COMPA_vect) {
    // increment sample counters
    wsl_samples_since_last++;
    wsr_samples_since_last++;
    wsl_debounce_samples++;
    wsr_debounce_samples++;

    // LEFT WHEEL
    if (bit_is_set(PINC, PIN_WHEEL_SPEED_LEFT) && !wsl_detected  // pulse begins
        && wsl_debounce_samples > WHEEL_SPEED_DEBOUNCE) {
        wsl_detected = true;
        uint32_t speed =  // uint32 used because some of the numbers in the calculation get very large
                (WHEEL_SPEED_DIST_PER_PULSE / wsl_samples_since_last)   // wheel speed in mils/sample
            *    WHEEL_SPEED_SAMPLE_RATE                                // converts to mils/second
            /    MILS_PER_SECOND_TO_MILES_PER_HOUR;                     // converts to mph
        wheel_speed_left = speed;  // the final result stored in `speed` should fit into a uint8
        wsl_samples_since_last = 0;
        wsl_debounce_samples = 0;
    } else if (bit_is_clear(PINC, PIN_WHEEL_SPEED_LEFT) && wsl_detected  // pulse ends
               && wsl_debounce_samples > WHEEL_SPEED_DEBOUNCE) {
        wsl_detected = false;
        wsl_debounce_samples = 0;
    }

    // RIGHT WHEEL
    if (bit_is_set(PINC, PIN_WHEEL_SPEED_RIGHT) && !wsr_detected  // pulse begins
        && wsr_debounce_samples > WHEEL_SPEED_DEBOUNCE) {
        wsr_detected = true;
        uint32_t speed =  // uint32 used because some of the numbers in the calculation get very large
                (WHEEL_SPEED_DIST_PER_PULSE / wsr_samples_since_last)   // wheel speed in mils/sample
            *    WHEEL_SPEED_SAMPLE_RATE                                // converts to mils/second
            /    MILS_PER_SECOND_TO_MILES_PER_HOUR;                     // converts to mph
        wheel_speed_right = speed;  // the final result stored in `speed` should fit into a uint8
        wsr_samples_since_last = 0;
        wsr_debounce_samples = 0;
    } else if (bit_is_clear(PINC, PIN_WHEEL_SPEED_LEFT) && wsr_detected  // pulse ends
               && wsr_debounce_samples > WHEEL_SPEED_DEBOUNCE) {
        wsr_detected = false;
        wsr_debounce_samples = 0;
    }
}

// Called 100 times a second to send datalogging values to CAN
ISR(TIMER1_COMPA_vect) {
    // turn off interrupts while sending CAN message
    cli();

    uint16_t damper_left_mapped = map(damper_left, 0, 1024, 0, 3170);
    uint16_t damper_right_mapped = map(damper_right, 0, 1024, 0, 3170);

    // construct the message
    tCAN msg = {
        .id = front_rear ? DAQ_FRONT : DAQ_REAR,
        .header.rtr = false,
        .header.length = 6,
        .data[0] = wheel_speed_left,
        .data[1] = wheel_speed_right,
        .data[2] = damper_left_mapped & 0xFF,
        .data[3] = (damper_left_mapped >> 8) & 0xFF,
        .data[4] = damper_right_mapped & 0xFF,
        .data[5] = (damper_right_mapped >> 8) & 0xFF,
    };

    // send the message
    mcp2515_send_message(&msg);

    // reenable interrupts
    sei();
}
