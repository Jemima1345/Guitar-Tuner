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

/*
 * 
 */
int main(int argc, char** argv) {
    
    /* set AIN0 and AIN1 as inputs */
    DDRB &= (1 << DDB1)|(1 << DDB0);
    
    /* gpio output for debugging*/
    PORTC &= ~(1 << PORTC5);
    DDRC |= (1 << DDC5);
    
    /* enable analog comparator and interrupt */
    ACSR &= (1 << ACD);
    ACSR |= (1 << ACIE); 
//    ACSR |= (1 << ACIE); //input capture enable

    /* enable global interrupts */
    sei();
    
    while(1){};


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