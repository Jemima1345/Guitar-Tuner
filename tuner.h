/* 
 * File:   tuner.h
 * Author: Valued Customer
 *
 * Created on August 1, 2022, 4:05 AM
 */

#ifndef TUNER_H
#define	TUNER_H

#ifdef	__cplusplus
extern "C" {
#endif


#define F_CPU 1000000UL

#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define NUM_SAMPLE_PERIODS  10
#define TIM1_CLK_FREQ       1000000 // 8MHZ/8 / 1
#define NOTES_ARRAY_LEN     49

#define SER               PORTD0
#define NO_OUTPUT_EN      PORTD1
#define RCLK              PORTD2
#define SRCLK             PORTD3
#define NO_SRCLR          PORTD4

void init_gpios(void);
void init_analog_comparator(void);
void init_input_capture(void);
uint16_t get_timer_count(void);
uint16_t measure_frequency(void);
uint16_t find_closest_note(uint16_t, uint16_t);
void set_closest_note_led(uint8_t, uint16_t);
void update_leds(uint16_t);


const uint16_t notes_array[] = {27.5,29.14,30.87,32.7,34.65,36.71,38.89,41.2,43.65,
                       46.25,49,51.91,55,58.27,61.74,65.41,69.3,73.42,77.78,
                       82.41,87.31,92.5,98,103.83,110,116.54,123.47,130.81,
                       138.59,146.83,155.56,164.81,174.61,185,196,207.65,220,
                       233.08,246.94,261.63,277.18,293.66,311.13,329.63,349.23,
                       369.99,392,415.3,440}; //TODO: change to float

typedef enum{ //lsb to msb
    /* 16 LEDs controlled by the shift register */
    A, //lsbit
    A_SHARP,
    B,
    C,
    C_SHARP,
    D,
    D_SHARP,
    E,
    F,
    F_SHARP,
    G,
    G_SHARP, 
    FLAT_X2,
    FLAT_X1,
    SHARP_X1,
    SHARP_X2 //msbit
}Leds_t;


#ifdef	__cplusplus
}
#endif

#endif	/* TUNER_H */
