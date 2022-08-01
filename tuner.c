/* 
 * File:   tuner.c
 * Author: Valued Customer
 *
 * Created on July 27, 2022, 11:30 PM
 */

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

uint16_t get_timer_count(void);
uint16_t measure_frequency(void);
uint16_t find_closest_note(uint16_t, uint16_t led_statuses);
void set_closest_note_led(uint8_t, uint16_t);
void update_leds(uint16_t);
/*
 * 
 */
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

volatile uint8_t num_overflows = 0;

int main(int argc, char** argv) {
    
    /* set AIN0 and AIN1 as inputs */
    DDRB &= ~(1 << DDB1)&~(1 << DDB0);
    
    /* gpio output for debugging*/
    PORTC |= (1 << PORTC5);
    DDRC |= (1 << DDC5);
    
    /* set pins controlling shift registers as outputs */
    DDRD |= (1 << DDD4)|(1 << DDD3)|(1 << DDD2)|(1 << DDD1)|(1 << DDD0);
    PORTD |= (1 << NO_SRCLR);
    PORTD &= ~(1 << NO_OUTPUT_EN)&~(1 << SRCLK)&~(1 << SER);

    /* enable analog comparator and interrupt */
    ACSR &= (1 << ACD);
    ACSR |= (1 << ACIE); 
    ACSR |= (1 << ACIC); //input capture enable

    /* Initializing input capture.
     * Set TCCR1A to 0 since WGM13:10 must be 0 for normal mode and COM1Xn=0 also.
     * In TCCR1B, clear ICNC1 to turn off the noise canceller, set CS10 since a 
     * prescaler of 1 is used, and clear ICES1 bit to 0 to trigger input 
     * capture on the falling edges of the analog comparator output signal. */
    TCCR1A = 0;
    TCCR1B &= ~(1 << ICNC1)|(1 << ICES1);
    TCCR1B |= (1 << CS10);
    
    /* enable the timer 1 overflow and input capture interrupts */
    TIMSK1 |= (1 << TOIE1);//|(1 << ICIE1);
    
    /* enable global interrupts */
    sei();

    
    uint16_t plucked_note, closest_note;
    uint16_t led_statuses = 0;
    while(1){
        plucked_note = measure_frequency();
        closest_note = find_closest_note(plucked_note, led_statuses);

        /* turn on led for closest note */
    }
    
    return (EXIT_SUCCESS);
}

ISR(ANALOG_COMP_vect){
        
    /* toggle gpio */
    if(PORTC & (1 << PORTC5)){
        PORTC &= ~(1 << PORTC5);
    }else{
        PORTC |= (1 << PORTC5);
    }
}

ISR(TIMER1_OVF_vect){
    num_overflows++;
}

//ISR(TIMER1_CAPT_vect){
//    /* automatically clears ICF1 */
//}

uint16_t get_timer_count(void){
    
    unsigned char sreg;
    uint16_t tim1_count;

    /* Save global interrupt flag */
    sreg = SREG;
    /* disable global interrupts before reading from ICR1 */
    cli();
    tim1_count = ICR1;
    /* Restore global interrupt flag */
    SREG = sreg;
    return tim1_count;
}

uint16_t measure_frequency(void){
    uint16_t t_start, t_end, note_freq;
    uint32_t period_count;
    uint8_t num_periods = 0;
    num_overflows = 0;
    
    /* wait for input capture flag to be set */
    while(!(TIFR1 & (1 << ICF1))){};
    t_start = get_timer_count();
    /* clear the ICF1 flag */
    TIFR1 |= (1 << ICF1);   
    
    while(num_periods < NUM_SAMPLE_PERIODS){
        /* wait again for input capture flag to be set*/
        while(!(TIFR1 & (1 << ICF1))){};
        ++num_periods;
    }
    t_end = get_timer_count();
    TIFR1 |= (1 << ICF1);
    
    period_count = (uint32_t)(65536*num_overflows + t_end - t_start);
    
    //TODO: use float for more accuracy
    return note_freq = (uint16_t)(TIM1_CLK_FREQ/period_count); 

}

uint16_t find_closest_note(uint16_t plucked_note, uint16_t led_statuses){
    
    uint16_t top_note, bottom_note, closest_note;
    uint8_t dist_to_top, dist_to_bottom, closest_note_idx;
    top_note = bottom_note = dist_to_top = dist_to_bottom = 0;
    
    for(uint8_t note_index = 0; note_index < NOTES_ARRAY_LEN; note_index++){
        
        if(notes_array[note_index] > plucked_note){
            top_note = notes_array[note_index];
            bottom_note = notes_array[note_index-1]; //TODO: don't do if idx 0 
            
            dist_to_top = top_note - plucked_note;
            dist_to_bottom = plucked_note - bottom_note;
            
            if(dist_to_bottom < dist_to_top){
                closest_note_idx = note_index-1;
                set_closest_note_led(closest_note_idx, led_statuses);
                closest_note = bottom_note;
                return closest_note;
            }else {
                /* if the plucked note is the same distance from the two nearest
                 * notes in the array, use the higher note as the closest */
                closest_note_idx = note_index;
                set_closest_note_led(closest_note_idx, led_statuses);
                closest_note = top_note;
                return closest_note;
            }   
        }
    }
    // note played is higher than the max note in the array
    closest_note_idx = NOTES_ARRAY_LEN-1;
    set_closest_note_led(closest_note_idx, led_statuses);
    closest_note = notes_array[NOTES_ARRAY_LEN-1]; 
    return closest_note;    
}

void set_closest_note_led(uint8_t closest_note_idx, uint16_t led_statuses){
   
    Leds_t led;
    led = closest_note_idx % 12; // corresponds to notes A to G#
    /* clear the note indicator LEDs but keep the offset indicator LEDs the same */
    led_statuses &= 0xF000;
    /* set the led for the closest note */
    led_statuses |= (1 << led); 
    update_leds(led_statuses);
}

/**
 * updates the relevant LEDs using the shift registers
 */
void update_leds(uint16_t led_statuses){
     /* If using msb to lsb, the LSB of led_statuses outputs to QA on shift register 1. 
     * If using lsb to msb, the LSB of led_statuses outputs to QH on shift register 2. 
     * The storage register is one clock pulse behind the shift register.
     */
    uint16_t led_status = 0;
    
    for(uint8_t led_num=0; led_num < 16; led_num++){
        led_status = led_statuses & (1 << led_num); //lsb to msb
//        led_status = led_statuses & (0x8000 >> led_num); //msb to lsb
        if(led_status != 0){
           PORTD |= (1 << SER);
        }else{
           PORTD &= ~(1 << SER);
        }
        /* generate a pulse on SRCLK to update shift register */
        PORTD |= (1 << SRCLK);
        _delay_ms(1);
        PORTD &= ~(1 << SRCLK);
        _delay_ms(1);
    }
    // extra clock pulse to update storage register
    PORTD |= (1 << RCLK);
    _delay_ms(1);
    PORTD &= ~(1 << RCLK);
    _delay_ms(1);
}