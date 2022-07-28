/* 
 * File:   tuner.c
 * Author: Valued Customer
 *
 * Created on July 27, 2022, 11:30 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define NUM_SAMPLE_PERIODS  1
#define TIM1_CLK_FREQ       1000000 // 8MHZ/8 / 1

uint16_t measure_frequency(void);
uint16_t get_timer_count(void);

/*
 * 
 */

volatile uint8_t num_overflows = 0;

int main(int argc, char** argv) {
    
    /* set AIN0 and AIN1 as inputs */
    DDRB &= (1 << DDB1)|(1 << DDB0);
    
    /* gpio output for debugging*/
    PORTC |= (1 << PORTC5);
    DDRC |= (1 << DDC5);
    
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
    
    uint16_t note = measure_frequency();
    
    
    while(1){
        note = measure_frequency();
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