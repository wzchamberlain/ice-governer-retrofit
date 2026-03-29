#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <stdint.h> // actually comes from avr, is override by compiler
#include <stdbool.h> // these booleans are actually 1 byte blocks

#define SERVO_PIN PB1 // servo PWM output pin
#define SPARK_PIN0 PB0 // (+) polar spark detect pin
#define SPARK_PIN1 PB2 // (-) polar spark detect pin
#define POS_POT_PIN PB4 // position constant trim potentiometer pin
#define INT_POT_PIN PB3 // integral constant trim potentiometer pin
#define DER_POT_PIN PB5 // derivative constant trim potentiometer pin
#define AWAKE_PIN POS_POT_PIN // pin for wake from sleep and send to sleep (alias for position constant trim potentiometer pin)

#define MIN_SERVO 7
#define MAX_SERVO 15

#define TARGET_DURATION 4166 // target duration of a spark cycle in ticks
#define DIFFERENCE_MULTIPLIER 1
#define DIFFERENCE_DIVISOR 1
#define DERIVATIVE_MULTIPLIER 1
#define DERIVATIVE_DIVISOR 1
#define INTEGRAL_MULTIPLIER 1
#define INTEGRAL_DIVISOR 1
#define FINAL_BITSHIFT 24

volatile uint8_t timer_rollover = 0; // used for time tracking.
volatile bool timer_double_roll = false;
volatile bool is_sleep_set = false;
volatile uint8_t pos_pot_val = 128; // let pos_pot_val of 128 mark a target of 3600rpm (60Hz)
volatile uint8_t int_pot_val = 128;
volatile uint8_t der_pot_val = 128;
volatile uint16_t last_duration = TARGET_DURATION;
volatile uint16_t last_error = 0;
volatile int32_t last_integral = 0;

void setup_pins() {
    // Set up port and pins
    // Set as output pins
    DDRB |= _BV(SERVO_PIN);

    // Set as input pins
    DDRB &= ~_BV(SPARK_PIN0);
    DDRB &= ~_BV(SPARK_PIN1);
    DDRB &= ~_BV(POS_POT_PIN);
    DDRB &= ~_BV(INT_POT_PIN);
    DDRB &= ~_BV(DER_POT_PIN);
    DDRB &= ~_BV(AWAKE_PIN); // This is duplicate, but placed in case of future modification or chipset change

    // Set pullup resistors (as required by OPTO-schmitt)
    PORTB |= _BV(SPARK_PIN0);
    PORTB |= _BV(SPARK_PIN1);

    // Disable pullup resistors on analog input trim potentiometers
    PORTB &= ~_BV(POS_POT_PIN);
    PORTB &+ ~_BV(INT_POT_PIN);
    PORTB &= ~_BV(DER_POT_PIN);
}

void setup_timer() {
    // set timer interrupt on overflow
    // disable interaction with pins for timer 0
    TCCR0A &= ~_BV(COM0A0);
    TCCR0A &= ~_BV(COM0A1);
    TCCR0A &= ~_BV(COM0B0);
    TCCR0A &= ~_BV(COM0B1);
    // set to "normal" counter mode
    TCCR0A &= ~_BV(WGM00);
    TCCR0A &= ~_BV(WGM01);
    TCCR0B &= ~_BV(WGM02);

    // 8x pre-scaler @ 1 MHz -> 7,500,000 tick/min | 125,000 tick/sec (2083.333333333 tick/60Hz for 2 stroke, 4166.666666667 tick/30Hz for 4 stroke)
    TCCR0B &= ~_BV(CS02);
    TCCR0B |= _BV(CS01);
    TCCR0B &= ~_BV(CS00);

    // disable force output compare registers?
    TCCR0B &= ~_BV(FOC0A);
    TCCR0B &= ~_BV(FOC0B);

    // set interrupt
    TIMSK &= ~_BV(OCIE0A); // no interrupt on OC0A match
    TIMSK &= ~_BV(OCIE0B); // no interrupt on OC0B match
    TIMSK |= _BV(TOIE0); // interrupt on timer 0 overflow
}

void setup_adc() {
    // use prescaler of 8, (1MHz / 8 = 125kHz, >=50kHz, <=200kHz)
    ADCSRA |= _BV(ADPS0);
    ADCSRA |= _BV(ADPS1);
    ADCSRA &= ~_BV(ADPS2);
    // enable ADC
    ADCSRA |= _BV(ADEN);
    // disable conversion completion interrupt (using busy wait, don't want an ISR)
    ADCSRA &= ~_BV(ADIE);
    // do not use auto triggering of ACD, will use a schedule in main loop to run this
    ADCSRA &= ~_BV(ADATE);
    
    // disable bipolar sampling (only reading GND to VCC)
    ADCSRB &= ~_BV(BIN);
    ADCSRB &= ~_BV(IPR);
    
    // set voltage reference to VCC
    ADMUX &= ~_BV(REFS0);
    ADMUX &= ~_BV(REFS1);
    ADMUX &= ~_BV(REFS2);
    // set left shift (only care about 8 MSb, 10bit precision not needed so this makes reading out byte easier)
    ADMUX |= _BV(ADLAR);

    // disable digital inputs for ADC pins NOTE: need to enable this entering sleep?
    DIDR0 |= _BV(ADC0D);
    DIDR0 |= _BV(ADC2D);
    DIDR0 |= _BV(ADC3D);
}

void set_pin_ints_run(){ // set up pin interrupts for running mode
    PCMSK |= _BV(PCINT0);
    PCMSK &= ~_BV(PCINT1);
    PCMSK |= _BV(PCINT2);
    PCMSK &= ~_BV(PCINT3);
    PCMSK &= ~_BV(PCINT4);
    PCMSK &= ~_BV(PCINT5);
}

void set_pin_ints_sleep() { // set up pin interrups for sleep mode (spark + RPM POT)
    PCMSK |= _BV(PCINT0);
    PCMSK &= ~_BV(PCINT1);
    PCMSK |= _BV(PCINT2);
    PCMSK &= ~_BV(PCINT3);
    PCMSK |= _BV(PCINT4);
    PCMSK &= ~_BV(PCINT5);
    DIDR0 &= ~_BV(ADC2D);
}

void init_pwm() {
    // use prescaler x128 to provide maximal resolution within 50Hz @ Main CLK 1MHz [(1Mhz / 128) = 7812.5Hz,  (7812.5Hz / 50Hz) = 156.25 Cycles]
    TCCR1 &= ~_BV(CS10);
    TCCR1 &= ~_BV(CS11);
    TCCR1 &= ~_BV(CS12);
    TCCR1 |= _BV(CS13);

    // set to PWM mode (A output) (similar effect to previous, ORC1C match clears)
    TCCR1 |= _BV(PWM1A);
    TCCR1 &= ~_BV(COM1B0);
    TCCR1 |= _BV(COM1A1);
    // disable PWM mode B output
    GTCCR &= ~_BV(PWM1B);
    GTCCR &= ~_BV(COM1B0);
    GTCCR &= ~_BV(COM1B1);

    // no interrupts for timer 1
    TIMSK &= ~_BV(OCIE1A);
    TIMSK &= ~_BV(OCIE1B);
    TIMSK &= ~_BV(TOIE1);

    // reset timer at 156 cycles (-1 since 0 is compared)
    OCR1C = 155;
    // duty cycle is 5%-10%, setting to min as a default [MIN: (156.25 * 5%) - 1 = 6.8125 ~= 7, MAX: (156.25 * 10%) - 1 = 14.625 ~= 15]
    OCR1A = MIN_SERVO;
}

void set_throttle_by_percent(uint8_t percent) {
    uint8_t new_val = MIN_SERVO;
    new_val += (percent * (MAX_SERVO - MIN_SERVO)) / 100;
    OCR1A = new_val;
}

void set_throttle_by_uint8(uint8_t input) {
    uint8_t new_val = MIN_SERVO;
    new_val += (input * (MAX_SERVO - MIN_SERVO)) / 255;
    OCR1A = new_val;
}

uint8_t get_throttle_as_uint8() {
    return OCR1A;
}

void close_throttle() {
    OCR1A = MIN_SERVO;
}

void wide_open_throttle() {
    OCR1A = MAX_SERVO;
}

// timer 0 (time counter) overflow interrupt
ISR (TIMER0_OVF_vect) {
    timer_rollover++; // just increment the timer
    if (timer_rollover >= 255) { // make note of the double rollover for throttle calculations
        timer_rollover = 0;
        timer_double_roll = true;
    }
}

// PCINT interrupt
ISR (PCINT0_vect) {
    if (is_sleep_set) { // sleep mode, wake
        wide_open_throttle();
        set_pin_ints_run();
        setup_adc();
        setup_timer();
        init_pwm();
        timer_double_roll = false;
        timer_rollover = 0;
        TCNT0 = 0;
        // TODO: reset/restart sub controllers
    } else { // normal running, spark detected
        if (timer_double_roll) {
            // engine is too slow to even track. Just hit the limiter so a reading can even be achieved.
            // if this causes thrashing in impl, then we need to widen the rollover integer
            wide_open_throttle();
            // clear timers
            timer_double_roll = false;
            timer_rollover = 0;
            TCNT0 = 0;
        } else {
            uint16_t rotation_duration = TCNT0; // set low byte to value currently in timer
            rotation_duration += timer_rollover << 8; // set high byte from rollover
            // clear timer so it can continue tracking
            timer_rollover = 0;
            TCNT0 = 0;
            // calculate error
            int32_t error = rotation_duration - TARGET_DURATION;
            // calculate time delta
            int32_t duration_delta = rotation_duration - last_duration;
            // find difference between counted and setpoint
            int32_t time_difference = error * pos_pot_val * DIFFERENCE_MULTIPLIER / (128 * DIFFERENCE_DIVISOR);
            // find derivative
            int32_t time_derivative = (error - last_error) * der_pot_val * DERIVATIVE_MULTIPLIER / (128 * duration_delta * DERIVATIVE_DIVISOR);
            // find integral
            int32_t time_integral = (last_integral + (error * duration_delta)) * int_pot_val * INTEGRAL_MULTIPLIER / (128 * INTEGRAL_DIVISOR);
            int32_t pid_out = time_difference + time_derivative + time_integral;
            uint8_t adjust = (pid_out < 0) ? ((-pid_out) >> FINAL_BITSHIFT) : (pid_out >> FINAL_BITSHIFT);
            uint8_t current_throttle = get_throttle_as_uint8();
            if (pid_out < 0 && adjust >= current_throttle) {
                close_throttle();
            } else if (pid_out < 0) {
                set_throttle_by_uint8(current_throttle - adjust);
            } else if (current_throttle >= current_throttle) {
                wide_open_throttle();
            } else {
                set_throttle_by_uint8(current_throttle + adjust);
            }
        }
    }
}

int main(void) {
    setup_pins(); // set up port B pins
    setup_timer(); // set up timer 0 for rpm measure
    setup_adc(); // set up adc block for measurement of pots
    init_pwm(); // set up timer 1 for PWM servo output

    // the forever run loop
    while(true) {
        // measure position pot (PB4, ADC2)
        ADMUX &= ~_BV(MUX0);
        ADMUX |= _BV(MUX1);
        ADMUX &= ~_BV(MUX2);
        ADMUX &= ~_BV(MUX3);
        ADCSRA |= _BV(ADSC); // start the conversion
        loop_until_bit_is_set(ADCSRA, ADIF); // wait for the conversion to finish
        pos_pot_val = ADCH;
        ADCSRA |= _BV(ADIF); // clear the flag
        // measure integral pot (PB3, ACD3)
        ADMUX |= _BV(MUX0);
        ADMUX |= _BV(MUX1);
        ADMUX &= ~_BV(MUX2);
        ADMUX &= ~_BV(MUX3);
        ADCSRA |= _BV(ADSC); // start the conversion
        loop_until_bit_is_set(ADCSRA, ADIF);
        int_pot_val = ADCH;
        ADCSRA |= _BV(ADSC);
        // measure derivative pot (PB5, ADC0)
        ADMUX &= _BV(MUX0);
        ADMUX &= _BV(MUX1);
        ADMUX &= _BV(MUX2);
        ADMUX &= _BV(MUX3);
        ADCSRA |= _BV(ADSC);
        loop_until_bit_is_set(ADCSRA, ADIF);
        der_pot_val = ADCH;
        ADCSRA |= _BV(ADSC);

        // check if sleep/shutdown commanded
        if (pos_pot_val < 2) {
            close_throttle(); // kill engine
            set_pin_ints_sleep();
            is_sleep_set = true;
            // set for powerdown sleep
            MCUCR |= _BV(SM1);
            MCUCR &= ~_BV(SM0);
            // disable brown out detection on sleep. No point in this app
            MCUCR |= _BV(BODS);
            // power reduction measures (shouldn't apply when using powerdown type sleep)
            PRR |= _BV(PRTIM1);
            PRR |= _BV(PRTIM0);
            PRR |= _BV(PRUSI);
            PRR |= _BV(PRADC);
            // enter sleep!
            MCUCR |= _BV(SE);
        }
    }
}