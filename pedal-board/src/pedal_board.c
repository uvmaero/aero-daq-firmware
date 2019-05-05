/**
 * Copyright 2019 Cullen Jemison, UVM AERO
 * 
 * Pack Manager (PacMan) Firmware
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/sfr_defs.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include "inc/mcp2515.h"
#include "inc/defaults.h"
#include "inc/global.h"
#include "inc/mcp2515_defs.h"
#include <stdbool.h>
#include "inc/can_ids.h"

// Pedal board CAN IDs
#define ID_BASE 0x2D
#define ID_RESET ID_BASE
#define ID_DATA ID_BASE+5
#define ID_GET_PEDAL ID_BASE+6
#define ID_SET_PEDAL_BAND ID_BASE+7
#define ID_SET_TORQUE ID_BASE+8
#define ID_PRECHARGE_STATUS ID_BASE+9

// Dash CAN IDs
#define ID_START 0x6F

// Rinehart CAN IDs
#define ID_RINEHART_COMMAND 0x0C0
#define ID_RINEHART_PARAM_REQUEST 0x0C1
#define ID_RINEHART_PARAM_RESPONSE 0x0C2
#define ID_RINEHART_DIGITAL_IN 0x0A4
#define ID_RINEHART_VOLTAGE 0x0A7

// Maximum number of missed messages before canceling precharge
#define MAX_MISSED_EMUS_MESSAGES 10
#define MAX_MISSED_RINEHART_MESSAGES 100

// Number of ADC samples to wait before sampling. This lets the sensor pull the device
// to the expected voltage in time.
#define NUM_SAMPLES 10

// EMUS CAN IDs
#define ID_EMUS_VOLTAGE 0x2A8

// Rinehart pins
#define RINEHART_PIN_TSMS 4
#define RINEHART_PIN_PRECHARGE 0
#define RINEHART_PIN_MAIN_CONT 1
#define RINEHART_PIN_RTDS 2

// analog input pin constants
#define PIN_PEDAL0  4
#define PIN_PEDAL1  3
#define PIN_BRAKE0  2
#define PIN_BRAKE1  1
#define PIN_STEER   0

// pedal EEPROM address base
#define PEDAL_EEPROM 10

// max torque EEPROM address
#define MAX_TORQUE_EEPROM 40

// maximum pedal mismatch or over/under-travel
#define MAX_PEDAL_SKEW 10

// pedal deadband for converting to torque command
#define PEDAL_DEADBAND 10

// precharge coefficient -- rinehart voltage must be more than emus voltage * coefficient
// in order to finish precharge
#define PRECHARGE_COEFFICIENT 0.9

// RTDS souding period (ms)
#define RTDS_PERIOD 2000

uint16_t brake_pressure_max = 3000;

// pedal ranges -- these are all defaults; they will be updated from the EEPROM on boot
uint16_t pedal0_min = 189;
uint16_t pedal0_max = 849;
uint16_t pedal1_min = 91;
uint16_t pedal1_max = 417;

// raw ADC values from each sensor
uint16_t pedal0=0, pedal1=0, brake0=0, brake1=0, steer=0;

// mapped pedal values
uint8_t pedal0_mapped, pedal1_mapped;

// max torque for Rinehart torque command
int16_t max_torque = 50;  // super low default just to be safe

// regen torque values -- these are updated over CAN from the dash
int16_t brake_regen_torque = 10;
int16_t coast_regen_torque = 10;

uint16_t brake_torque_multiplier = 0;

// brake system params
uint16_t front_brake_pressure = 0;
uint16_t rear_brake_pressure = 0;

// direction -- 0=forward, 1=reverse
uint8_t direction = 0;

// ready to drive state
uint8_t ready_to_drive = false;
uint8_t rtds_on = false;

// rinehart inputs
uint8_t rinehart_inputs = 0;

// precharge state
enum precharge_state_e {
    PRECHARGE_OFF,
    PRECHARGE_ON,
    PRECHARGE_DONE,
    PRECHARGE_ERROR
} precharge_state = PRECHARGE_OFF;

// set whenever a new precharge state is entered
uint8_t precharge_state_enter = 1;

// ADC sample accumulator
uint8_t sample_accumulator;

// RTDS on time (ms)
uint16_t time_since_rtds_start = 0;

// voltages
double emus_voltage = 265.0;  // assume maximum voltage to begin with
double rinehart_voltage = 0;

uint8_t cycles_since_last_emus_message = 0;
uint8_t cycles_since_last_rinehart_message = MAX_MISSED_RINEHART_MESSAGES;  // assume Rinehart connection hasn't been made yet

// Directly from Arduino
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void load_pedal_values(void) {
    uint16_t eeprom_buf[4];
    eeprom_read_block(eeprom_buf, (uint16_t*)PEDAL_EEPROM, 8);
    pedal0_min = eeprom_buf[0];
    pedal0_max = eeprom_buf[1];
    pedal1_min = eeprom_buf[2];
    pedal1_max = eeprom_buf[3];
}

void store_pedal_values(uint16_t p0_min, uint16_t p0_max, uint16_t p1_min, uint16_t p1_max) {
    // update RAM values
    pedal0_min = p0_min;
    pedal0_max = p0_max;
    pedal1_min = p1_min;
    pedal1_max = p1_max;

    uint16_t eeprom_buf[4] = {
        pedal0_min,
        pedal0_max,
        pedal1_min,
        pedal1_max
    };

    // update EEPROM values
    eeprom_update_block(eeprom_buf, (uint16_t*)PEDAL_EEPROM, 8);
}

uint16_t load_max_torque() {
    return eeprom_read_word((uint16_t*)MAX_TORQUE_EEPROM);
}

void store_max_torque(uint16_t torque) {
    eeprom_update_word((uint16_t*)MAX_TORQUE_EEPROM, torque);
}

uint8_t send_rinehart_command(uint16_t param_addr, uint8_t rw, uint16_t data, uint16_t *data_out, uint8_t num_tries) {
    // create the command message
    tCAN command = {
        .id = ID_RINEHART_PARAM_REQUEST,
        .header.rtr = false,
        .header.length = 6,
        .data[0] = param_addr & 0xFF,
        .data[1] = (param_addr >> 8) & 0xFF,
        .data[2] = rw,
        .data[3] = 0x00,
        .data[4] = data & 0xFF,
        .data[5] = (data >> 8) & 0xFF
    };

    cli();
    mcp2515_send_message(&command);
    sei();

    for (int i = 0; i < num_tries; i++) {
        tCAN response;
        cli();
        if (mcp2515_get_message(&response)) {
            // if the CAN id matches, along with the param address and the success message, return 1
            if (response.id == ID_RINEHART_PARAM_RESPONSE &&
                (response.data[0] | (response.data[1] << 8)) == param_addr &&
                response.data[2] == 0x01) {
                if (data_out != NULL) {
                    *data_out = response.data[4] | (response.data[5] << 8);
                }
                return 1;
            }
        }
        sei();
    }
    return 0;
}

uint8_t set_rinehart_output(uint8_t value, uint8_t num_tries) {
    return send_rinehart_command(0x01, 0x01, 0x5500 | value, NULL, num_tries);
}

void set_precharge_state(enum precharge_state_e state) {
    precharge_state = state;
    precharge_state_enter = 1;
}

void control_precharge() {
    if (cycles_since_last_rinehart_message > MAX_MISSED_RINEHART_MESSAGES) {
        precharge_state = PRECHARGE_ERROR;
    }
    // if (cycles_since_last_emus_message > MAX_MISSED_EMUS_MESSAGES) {
    //     precharge_state = PRECHARGE_ERROR;
    // }
    // TODO: make sure outputs were set properly before doing a state change
    switch (precharge_state) {
        case PRECHARGE_OFF: {
            // if the state was just entered, try to turn off both rinehart outputs
            if (precharge_state_enter) {
                if (set_rinehart_output(0, 10)) {
                    precharge_state_enter = 0;
                }
            }

            // vehicle is no ready to drive
            ready_to_drive = false;

            // if TSMS is on, switch to PRECHARGE_ON
            if (rinehart_inputs & (1 << RINEHART_PIN_TSMS)) {
                set_precharge_state(PRECHARGE_ON);
            }
            break;
        }
        case PRECHARGE_ON: {
            if (precharge_state_enter) {
                if (set_rinehart_output(1 << RINEHART_PIN_PRECHARGE, 10)) {
                    precharge_state_enter = 0;
                }
            }

            // if TSMS is no longer on, switch back to PRECHARGE_OFF
            if (!(rinehart_inputs & (1 << RINEHART_PIN_TSMS))) {
                set_precharge_state(PRECHARGE_OFF);
            }

            // vehicle is not ready to drive
            ready_to_drive = false;

            if (rinehart_voltage > emus_voltage * PRECHARGE_COEFFICIENT) {
                set_precharge_state(PRECHARGE_DONE);
            }
            break;
        }
        case PRECHARGE_DONE: {
            if (precharge_state_enter) {
                if (set_rinehart_output((1 << RINEHART_PIN_MAIN_CONT) | (1 << RINEHART_PIN_PRECHARGE), 10)) {
                    precharge_state_enter = 0;
                }
            }

            // if TSMS is no longer on, switch back to PRECHARGE_OFF
            if (!(rinehart_inputs & (1 << RINEHART_PIN_TSMS))) {
                set_precharge_state(PRECHARGE_OFF);
            }
            break;
        }
        case PRECHARGE_ERROR: {
            // turn off Rinehart outputs
            set_rinehart_output(0, 10);
            
            // vehicle is no longer ready to drive
            ready_to_drive = false;

            // if both devices have been read from recently, return to PRECHARGE_OFF
            if (cycles_since_last_emus_message < MAX_MISSED_EMUS_MESSAGES && cycles_since_last_rinehart_message < MAX_MISSED_RINEHART_MESSAGES) {
                precharge_state = PRECHARGE_OFF;
            }
        }
    }

    // incrememnt misssed emus messages counter
    // if (cycles_since_last_emus_message <= MAX_MISSED_EMUS_MESSAGES) {
    //     cycles_since_last_emus_message++;
    // }

    // increment missed rinehart messages counter
    if (cycles_since_last_rinehart_message <= MAX_MISSED_RINEHART_MESSAGES) {
        cycles_since_last_rinehart_message++;
    }

    // send precharge message
    tCAN precharge_msg = {
        .id = ID_PRECHARGE_STATUS,
        .header.rtr = false,
        .header.length = 1,
        .data[0] = precharge_state
    };

    cli();
    mcp2515_send_message(&precharge_msg);
    sei();
}

int main(void) {
    // set PB2 as an output (so that SS won't get driven low and mess up SPI)
    DDRB |= (1<<PB2);

    // initialize the ADC
    ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADIE) | (7 << ADPS0);  // enable ADC, enable triggering, prescaler F_cpu/128
    ADCSRB = 0;  // freerunning mode
    ADMUX = (1 << REFS0) | (PIN_PEDAL0);  // AVcc reference, start mux at pedal0
    ADCSRA |= (1 << ADSC);  // start conversion

    // initialize TIMER0 for 100Hz -- used for datalogging
    TCCR0A = (1 << WGM01);  // CTC mode
    TCCR0B = (0x05 << CS00);  // f_timer = f_cpu/1024 => 7812Hz
    OCR0A = 78;  // set comparator A => f_out = 100.2Hz
    TIMSK0 = (1 << OCIE0A);  // enable comparator A interrupt

    // initialize TIMER1 for 10Hz -- used for Rinehart messaging
    TCCR1A = (1 << WGM01);  // CTC mode
    TCCR1B = (0x05 << CS10);  // f_timer = f_cpu/1024 => 7812Hz
    OCR1A = 781;  // set comparator A => f_out=10Hz
    TIMSK1 = (1 << OCIE1A);  // enable comparator A interrupt

    // load pedal high/low values from EEPROM
    load_pedal_values();

    // load torque value
    max_torque = load_max_torque();

    // turn off interrupts while setting up the MCP2515
    cli();

    // wait until the MCP2515 is initialized
    while (!mcp2515_init()) _delay_ms(100);

    // enable interrupts
    sei();

    // main application loop
    while (1) {
        // clear interrupts when processing received messages
        cli();

        // process received messages
        tCAN message;
        if (mcp2515_get_message(&message)) {
            switch (message.id) {
                case ID_RESET: {
                    // reset the device by failing to reset the watchdog
                    cli();
                    wdt_enable(WDTO_15MS);
                    while (1) continue;
                }
                case ID_GET_PEDAL: {
                    if (message.header.rtr) {
                        tCAN resp = {
                            .id = ID_GET_PEDAL,
                            .header.rtr = false,
                            .header.length = 4,
                            .data[0] = pedal0,
                            .data[1] = pedal0 >> 8,
                            .data[2] = pedal1,
                            .data[3] = pedal1 >> 8
                        };
                        mcp2515_send_message(&resp);
                    }
                    break;
                }
                case ID_SET_PEDAL_BAND: {
                    store_pedal_values(message.data[0] + (message.data[1] << 8),
                                       message.data[2] + (message.data[3] << 8),
                                       message.data[4] + (message.data[5] << 8),
                                       message.data[6] + (message.data[7] << 8));
                    break;
                }
                case ID_SET_TORQUE: {
                    max_torque = message.data[0] + (message.data[1] << 8);
                    store_max_torque(max_torque);
                    break;
                }
                // case ID_START: {
                //     if (precharge_state == PRECHARGE_DONE) {
                //         ready_to_drive = true;
                //     }
                //     break;
                // }
                case ID_RINEHART_DIGITAL_IN: {
                    rinehart_inputs = (message.data[0]) |
                                      (message.data[1] << 1) |
                                      (message.data[2] << 2) |
                                      (message.data[3] << 3) |
                                      (message.data[4] << 4) |
                                      (message.data[5] << 5) |
                                      (message.data[6] << 6) |
                                      (message.data[7] << 7);
                    // candebug(ID_DEBUG, 5, (uint8_t)rinehart_inputs);
                    break;
                }
                case ID_RINEHART_VOLTAGE: {
                    int16_t rinehart_voltage_raw = (int16_t)((message.data[1] << 8) | message.data[0]);
                    rinehart_voltage = (double)rinehart_voltage_raw / 10.0;
                    cycles_since_last_rinehart_message = 0;
                    // candebug(ID_DEBUG, 6, (uint8_t)rinehart_voltage);
                    break;
                }
                case ID_EMUS_VOLTAGE: {
                    // emus overall voltage is coded as a 32-bit integer in 0.01V intervals
                    // uint32_t emus_voltage_raw = ((uint32_t)message.data[5] << 24) | ((uint32_t)message.data[6] << 16) | ((uint32_t)message.data[3] << 8) | (uint32_t)message.data[4];
                    // emus_voltage = (double)emus_voltage_raw * 0.01;
                    break;
                }
            }
        }

        // reenable interrupts
        sei();
    }
}

ISR(ADC_vect) {
    // Record sample after waiting for the desired number of samples
    if (sample_accumulator == NUM_SAMPLES) {
        // check the mux to see which sensor was sampled
        switch (ADMUX & 0x0F) {
            case PIN_PEDAL0: {
                pedal0 = ADC;  // read the ADC
                ADMUX = (ADMUX & 0xF0) | PIN_PEDAL1;  // switch the mux to the next pin to read
                break;
            }
            case PIN_PEDAL1: {
                pedal1 = ADC;  // read the ADC
                ADMUX = (ADMUX & 0xF0) | PIN_BRAKE0;  // switch the mux to the next pin to read
                break;
            }
            case PIN_BRAKE0: {
                brake0 = ADC;  // read the ADC
                ADMUX = (ADMUX & 0xF0) | PIN_BRAKE1;  // switch the mux to the next pin to read
                break;
            }
            case PIN_BRAKE1: {
                brake1 = ADC;  // read the ADC
                ADMUX = (ADMUX & 0xF0) | PIN_STEER;  // switch the mux to the next pin to read
                break;
            }
            case PIN_STEER: {
                // steer = ADC;  // read the ADC
                if (ADC < 256 && !ready_to_drive) {
                    if (precharge_state == PRECHARGE_DONE) {
                        ready_to_drive = true;
                        rtds_on = true;

                        time_since_rtds_start = 0;

                        set_rinehart_output((1 << RINEHART_PIN_MAIN_CONT) | (1 << RINEHART_PIN_PRECHARGE | (1 << RINEHART_PIN_RTDS)), 10);
                   }
                }
                ADMUX = (ADMUX & 0xF0) | PIN_PEDAL0;  // switch the mux to the next pin to read
                break;
            }
            default: {
                ADMUX = (ADMUX & 0xF0) | PIN_PEDAL0;
            }
        }
        sample_accumulator = 0;
    } else {
        sample_accumulator++;
    }
}

// 10Hz timer interrupt
ISR(TIMER1_COMPA_vect) {
    // Control precharge
    control_precharge();

    // map the pedal
    pedal0_mapped = map(pedal0, pedal0_min, pedal0_max, 0, 255);
    pedal1_mapped = map(pedal1, pedal1_min, pedal1_max, 0, 255);

    // map brake pressure sensors
    uint16_t brake0_mapped = map(brake0, 0, 1024, 0, brake_pressure_max);
    uint16_t brake1_mapped = map(brake1, 0, 1024, 0, brake_pressure_max);

    // check for over/under-travel
    if (pedal0 > pedal0_max + 4*MAX_PEDAL_SKEW ||
        pedal0 < pedal0_min ||
        pedal1 > pedal1_max + 4*MAX_PEDAL_SKEW ||
        pedal1 < pedal1_min) {
            pedal0_mapped = 0;
            pedal1_mapped = 0;
    }

    // check for mismatch
    if (pedal0_mapped > pedal1_mapped + MAX_PEDAL_SKEW || pedal1_mapped > pedal0_mapped + MAX_PEDAL_SKEW) {
        pedal0_mapped = 0;
        pedal1_mapped = 0;
    }

    // construct the message
    tCAN msg = {
        .id = ID_DATA,
        .header.rtr = false,
        .header.length = 7,
        .data[0] = pedal0_mapped,
        .data[1] = pedal1_mapped,
        .data[2] = (uint8_t)brake0_mapped & 0xFF,
        .data[3] = (uint8_t)(brake0_mapped >> 8) & 0xFF,
        .data[4] = (uint8_t)brake1_mapped & 0xFF,
        .data[5] = (uint8_t)(brake1_mapped >> 8) & 0xFF,
        .data[6] = (uint8_t)(steer / 4)
    };

    // turn off interrupts while sending CAN message
    cli();

    // send the message
    mcp2515_send_message(&msg);

    // reenable interrupts
    sei();

    if (ready_to_drive && precharge_state == PRECHARGE_DONE && time_since_rtds_start > RTDS_PERIOD && rtds_on) {
        set_rinehart_output((1 << RINEHART_PIN_MAIN_CONT) | (1 << RINEHART_PIN_PRECHARGE), 10);
        rtds_on = false;
    }

    if (ready_to_drive && time_since_rtds_start <= RTDS_PERIOD && rtds_on) {
        time_since_rtds_start += 100;
    }
}

// 100Hz timer interrupt
ISR(TIMER0_COMPA_vect) {
    // take an average of the pedal values
    uint8_t pedal_avg = pedal0_mapped/2 + pedal1_mapped/2;

    // compute commanded torque (N-m * 10 [10x factor is how Rinehart expects it])
    // TODO(cullen): should this be a linear map?
    int16_t commanded_torque = map(pedal_avg, PEDAL_DEADBAND, 255, 0, 10*max_torque);

    if (pedal_avg < PEDAL_DEADBAND) {
        // enable coasting regen if pedal is below deadband
        commanded_torque = -10*coast_regen_torque;

        // also apply brake regen proportionally to brake pressure
        uint16_t brake_torque = brake_torque_multiplier * front_brake_pressure;
        commanded_torque -= brake_torque;
    }

    // if the vehicle is not ready to drive, set torque command to 0
    if (!ready_to_drive) {
        commanded_torque = 0;
    }

    // construct the rinehart message
    tCAN msg = {
        .id = ID_RINEHART_COMMAND,
        .header.rtr = false,
        .header.length = 8,
        .data[0] = commanded_torque & 0xFF,
        .data[1] = commanded_torque >> 8,
        .data[2] = 0,
        .data[3] = 0,
        .data[4] = 0,  // 0 - "reverse", 1 - "forward" (CS4 drives forward with motor in "reverse" direction)
        .data[5] = ready_to_drive ? 1 : 0,
        .data[6] = 0, //(max_torque*10) & 0xFF,
        .data[7] = 0 //(max_torque*10) >> 8
    };

    // turn off interrupts while sending CAN message
    cli();

    // send the message
    mcp2515_send_message(&msg);

    // reenable interrupts
    sei();
}
