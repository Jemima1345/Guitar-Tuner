/* 
 * File:   tuner.c
 * Author: Valued Customer
 *
 * Created on July 27, 2022, 11:30 PM
 */

#include "tuner.h"
//TODO: make some variables pointers or global instead of passing into functions

volatile uint8_t num_overflows = 0;

void TIM16_WriteTCNT1(unsigned int i)
{
    unsigned char sreg;
    /* Save global interrupt flag */
    sreg = SREG;
    /* Disable interrupts */
    cli();
    /* Set TCNT1 to i */
    TCNT1 = i;
    /* Restore global interrupt flag */
    SREG = sreg;
}

int main(int argc, char** argv) {

    init_gpios();
    init_analog_comparator();
    init_input_capture();

    /* enable global interrupts */
    sei();

    uint16_t plucked_note, closest_note;
    uint16_t led_statuses = 0;

    while(1){
        plucked_note = measure_frequency();
        closest_note = find_closest_note(plucked_note, led_statuses);
        /* find note offset */
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

void init_gpios(void){
    /* set AIN0 and AIN1 as inputs */
    DDRB &= ~(1 << DDB1)&~(1 << DDB0);
    
    /* gpio output for debugging*/
    PORTC |= (1 << PORTC5);
    DDRC |= (1 << DDC5);
    
    /* set pins controlling shift registers as outputs */
    DDRD |= (1 << DDD4)|(1 << DDD3)|(1 << DDD2)|(1 << DDD1)|(1 << DDD0);
    PORTD |= (1 << NO_SRCLR);
    PORTD &= ~(1 << NO_OUTPUT_EN)&~(1 << SRCLK)&~(1 << SER);

    /* make the green led an output */
    GREEN_LED_PORT &= ~(1 << GREEN_LED);
    DDRB |= (1 << DDB6); //TODO: define in tuner.h
}

/**
 * enable the analog comparator and its interrupt 
 */
void init_analog_comparator(void)
{
    ACSR &= (1 << ACD);
    ACSR |= (1 << ACIE); 
    ACSR |= (1 << ACIC); //input capture enable from comparator
    //TODO: select ACIS0 and ACIS1 values to determine interrupt edge
}

void init_input_capture(void)
{
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
}

uint16_t get_timer_count(void)
{
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

uint16_t measure_frequency(void)
{
    uint16_t t_start, t_end, note_freq;
    uint32_t period_count;
    uint8_t num_periods = 0;
    num_overflows = 0;
    
//    TIM16_WriteTCNT1(0);
//    TCNT1 = 0;
    
    /* wait for input capture flag to be set */
    while(!(TIFR1 & (1 << ICF1))){};
    t_start = get_timer_count();
    /* clear the ICF1 flag by setting it to 1 */
    TIFR1 |= (1 << ICF1);   
    
    while(num_periods < NUM_SAMPLE_PERIODS){
        /* wait again for input capture flag to be set*/
        while(!(TIFR1 & (1 << ICF1))){};
        ++num_periods;
    }
    t_end = get_timer_count();
    TIFR1 |= (1 << ICF1);
    
//    if(t_end < t_start){
//        num_overflows++;
//    }
    
    period_count = (uint32_t)(65536*num_overflows + t_end - t_start);
    
    //TODO: use float for more accuracy. hardcoded +1 for now for more accuracy 
    return note_freq = (uint16_t)(TIM1_CLK_FREQ/period_count)+1; 
}

uint16_t find_closest_note(uint16_t plucked_note, uint16_t led_statuses)
{
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
                closest_note = bottom_note;
                set_closest_note_led(closest_note_idx, led_statuses);
                find_note_offset(plucked_note, closest_note, closest_note_idx, led_statuses);
                return closest_note;
            }else {
                /* if the plucked note is the same distance from the two nearest
                 * notes in the array, use the higher note as the closest */
                closest_note_idx = note_index;
                closest_note = top_note;
                set_closest_note_led(closest_note_idx, led_statuses);
                find_note_offset(plucked_note, closest_note, closest_note_idx, led_statuses);
                return closest_note;
            }   
        }
    }
    // note played is higher than the max note in the array
    closest_note_idx = NOTES_ARRAY_LEN-1;
    set_closest_note_led(closest_note_idx, led_statuses);
    find_note_offset(plucked_note, closest_note, closest_note_idx, led_statuses);
    closest_note = notes_array[NOTES_ARRAY_LEN-1]; 
    return closest_note;    
}

void set_closest_note_led(uint8_t closest_note_idx, uint16_t led_statuses)
{
    Leds_t led;
    led = closest_note_idx % 12; // corresponds to notes A to G#
    /* clear the note indicator LEDs but keep the offset indicator LEDs the same */
    led_statuses &= 0xF000;
    /* set the led for the closest note */
    led_statuses |= (1 << led); 
    update_leds(led_statuses);
}

void find_note_offset(uint16_t plucked_note, uint16_t closest_note, uint16_t closest_note_idx, uint16_t led_statuses)
{
    uint8_t above = 0;
    float step_size, offset;
    // above = (plucked_note > closest_note) ? 1 : 0;

    if(plucked_note > closest_note){
        above = 1;
        //TODO: use floats
        step_size = (notes_array[closest_note_idx+1] - closest_note)/3;
        offset = plucked_note - closest_note;
    }else if(plucked_note <= closest_note){
        above = 0;
        step_size = (closest_note - notes_array[closest_note_idx-1])/3;
        offset = closest_note - plucked_note;
    }
    // else{
        // GREEN_LED_PORT |= (1 << GREEN_LED);
    // }

    if(offset < 0.5f*DEADBAND){
        GREEN_LED_PORT |= (1 << GREEN_LED);
    }else{
        GREEN_LED_PORT &= ~(1 << GREEN_LED);
        uint8_t led;
        if(offset < step_size){
            if(above){
                led = SHARP_X1;
            }else{
                led = FLAT_X1;
            }
        }else{
            if(above){
                led = SHARP_X2;
            }else{
                led = FLAT_X2;
            }
        }
        /* clear the offset leds but keep the note indicator leds the same */
        led_statuses &= 0x0FFF;
        led_statuses |= (1 << led);
        update_leds(led_statuses);
    }
}
/**
 * updates the relevant LEDs using the shift registers
 */
void update_leds(uint16_t led_statuses)
{
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