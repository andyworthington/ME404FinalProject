/*
 * FinalProject.c
 *
 * Created: 11/10/2017
 * Authors: Andy Worthington & Sandy Crammond
 * Course: ME404L, Fall 2017
 * Final Project: Rabbit's Run
 *
 * Description: 
 *              
 *              
 *               
 *              
 *
 * Input:
 * Analog signal from the IR Sensor (PC0)
 * 1-Bit Digital signal from Ultrasonic Sensor (PB0)
 * 2-Bit Signal from the Quadrature Encoder (PB2...PB1)
 *
 * Output:
 * PWM signal to Motor 1 H-Bridge (PD6) 
 * PWM signal to Motor 2 H-Bridge (PD5)
 * 2-Bit Digital Directional input to Motor 1 H-Bridge (PC3...PC2)
 * 2-Bit Digital Directional input to Motor 2 H-Bridge (PC5...PC4)             
 * 4-Bit Digital Output to 7-Segment Display Driver (PD3...PD0)
 *  
 *  
 *  
 *  
 *  
 */ 

#include <avr/io.h>      
#include <inttypes.h>   
#include <avr/interrupt.h>


typedef enum driveState{
	forward, reverse, turnLeft, turnRight,
}driveState;

typedef enum UltrasonicState {
	triggerPulse, waitforRise, distanceTiming, calculating,
} UltrasonicState;





volatile UltrasonicState state = calculating;
volatile uint8_t needsToCalculate = 0;
volatile uint8_t doneConverting = 0;
volatile uint16_t ADCin = 0;
volatile int16_t encCount = 0;


int main(void) {       
	uint16_t pulseWidth = 0;
    // GPIO Setup // 
    DDRB &= ~((1 << PORTB1) | (1 << PORTB2) | (1 << PORTB0));
    DDRC |= ((1 << PORTC5)|(1 << PORTC4)|(1 << PORTC3)|(1 << PORTC2));    
	DDRC &= ~((1 << PORTC0)); 
	DDRD |= ((1 << PORTD5) | (1 << PORTD6) | (1 << PORTD7));
                                           
          

	// Timer 0 Setup (PCPWM)
	TCCR0A |= ((1 << COM0A1)| (1 << WGM00)| (1 << COM0B1)); // Configure TOP = 0xFF, Non-Inverting OC
	TCCR0B |= (1 << CS00);
	OCR0A = 200;
	OCR0B = 200;

	// Timer 1 Setup (CTC, Ultrasonic)
	OCR1A = 49999;              // 0.2s before first pulse
	TCCR1B |= (1 << WGM12);     // Configure Timer 1 for CTC mode
	TIMSK1 |= (1 << OCIE1A);    // Enable Timer 1 compare interrupt

	// Timer 2 Setup (CTC, Ultrasonic)
	/*OCR2A = 160;                // 10us with no scaling
	TCCR2A |= (1 << WGM21);     // Enable Timer 2 compare interrupt
	TIMSK2 |= (1 << OCIE2A);    // Enable Timer 2 compare interrupt
	*/

	// Pin Change Setup
	PCICR  |= (1 << PCIE0);     // Configure Pin Change Interrupt for PB0

	                                  
	


    // ADC Setup 
    ADCSRA |= ((1 << ADEN) | (1 << ADIE));                  // Enable ADC, Enable Interrupts
    ADCSRA |= ((1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2)); // Configure Prescalar for 125kHz
    ADMUX  = (1 << REFS0);                                  // Configure for 5V Vcc Reference
    DIDR0 |= (1 << ADC0D);		                            // Disable digital buffer on PC0
    
	
	
	
    sei();          // ENABLE GLOBAL INTERRUPTS
    //ADCSRA |= (1 << ADSC);  // Start First Conversion
	PORTC &= ~((1 << PORTC2) | (1 << PORTC4));
	PORTC |= ((1 << PORTC3) | (1 << PORTC5));

	TCCR1B |= ((1 << CS11) | (1 << CS10));  // Start timer, waiting 0.2s
											// before first trigger pulse
    while(1){
		if(needsToCalculate){
			needsToCalculate = 0;
			cli();
			pulseWidth = ICR1;
			if((pulseWidth < 75)){ 
				OCR0A = 190;
				OCR0B = 175;
			}else if(pulseWidth > 90){ 
				OCR0A = 190;
				OCR0B = 205;
			}
			else{
				OCR0A = 190;
				OCR0B = 190;
			}
			sei();
			}
        
    }
    return 0;           
}

ISR(ADC_vect){
    doneConverting = 1;
}

//ISR(PCINT1_vect){}

//ISR(PCINT2_vect, ISR_ALIASOF(PCINT1_vect))

ISR(TIMER1_COMPA_vect) {
	// Main Timer Interrupt//
	if(state == calculating){
		state = triggerPulse;
		TCCR1B &= ~((1 << CS11) | (1 << CS10)); // Kill Timer 1
		
		//new stuff
		OCR1A = 160;           // Time for 10us
		TCNT1 = 0;
		PORTD  |=  (1 << PORTD7);			   // Pull PD7 High
		TCCR1B |= (1 << CS10); // Start Timer 1, PS 64

	}
		//new
	else if(state == triggerPulse){
		state = waitforRise;
		PORTD  &= ~(1 << PORTD7);   // Pull PD7 Low
		TCCR1B &= ~(1 << CS10);		// Stop Timer 1
		PCIFR   =  (1 << PCIF0);    // Clear PC flags
		PCMSK0 |=  (1 << PCINT0);   // Enable PC Interrupt
	}

	else if(state == distanceTiming){
		state = calculating;
		OCR1A = 12500;  // Time for 50ms
		TCNT1 = 0;
		TCCR1B |= ((1 << CS11) | (1 << CS10)); // Start Timer 1
		TIMSK1 &=  ~(1 << ICIE1); // Kill Input Capture Interrupt
		needsToCalculate = 1;
	}

}
/*
ISR(TIMER2_COMPA_vect) {
	// Timer 2 Compare Interrupt

}*/


ISR(PCINT0_vect){
	// PIN Change Interrupt
	state = distanceTiming;
	PCMSK0 &= ~(1 << PCINT0);   // Kill PC Interrupt
	OCR1A = 7500;               // 30ms at prescalar 64
	TIMSK1 |=  (1 << OCIE1A);   // Enable CTC
	TIMSK1 |=  (1 << ICIE1);    // Enable Input Capture Interrupt
	TCNT1 = 0;                  // Clear Timer
	TCCR1B |= ((1 << CS11) | (1 << CS10));  // Start timer, prescalar 64
											// timing out after 30ms
}

ISR(TIMER1_CAPT_vect){
	// Input Capture Event Interrupt

	state = calculating;
	TIMSK1 &=  ~(1 << ICIE1); // Kill Input Capture Interrupt
	TIMSK1 |=  (1 << OCIE1A); // Enable CTC
	OCR1A = 12500;           // Time for 50ms
	TCNT1 = 0;               // Clear Timer
	TCCR1B |= ((1 << CS11) | (1 << CS10)); // Start Timer 1, PS 64
	needsToCalculate = 1;
}
