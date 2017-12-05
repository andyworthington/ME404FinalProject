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
#include <stdlib.h>
/*
// This enum. variable describes the current  
// velocity and direction of the robot's movement.
typedef enum robotMotion{
	forward, reverse, turnLeft, turnRight,
}robotMotion;
*/
// This enum. variable describes the current  
// geographical section of Rabbit's Run in which 
// the robot is moving.
typedef enum robotLocation {
	Leg1FirstPass, Leg2FirstPass, Leg3FirstPass, Leg4, 
	Leg5, Leg1SecondPass, Leg2SecondPass, Leg3SecondPass,
	FinalPush,
}robotLocation;

// This enum. variable describes the frequency  
// with which the ADC should measure forward distance,
// based on the distance traveled by the robot since the last turn. 
// This is implemented in an attempt to prevent race conditions based on 
// a continously triggerred ADC, which operates @ 125kHz. 
typedef enum IRSApproach {
	LongApproach, ShortApproach,
}IRSApproach;


// This enum. variable describes the current  
// state of the Ultrasonic Sensor, bridging both 
// input and output processes. This is designed to run
// more or less continuously throughout the main program. 
typedef enum UltrasonicState {
	triggerPulse, waitforRise, distanceTiming, calculating,
} UltrasonicState;

//
// Establishing all the global variables required to make this robot function. 
//
volatile IRSApproach approachState = LongApproach;
//volatile robotMotion driveState = forward;
volatile robotLocation legState = Leg1FirstPass;
volatile UltrasonicState USSstate = calculating;
// Bools for various processes

volatile uint8_t needsToCalculate = 0;
volatile uint8_t doneConverting = 0;
// Storage Variables
volatile uint16_t ADCin = 0;
volatile int16_t encCount = 0;
volatile uint8_t cs = 0;
uint16_t encStamp = 0;
//
//
//


//
// Function prototypes required for this program. 
//
// Functions for Robot Movement
void forwardWithControl();
void forwardWithoutControl();
void approachWall(uint8_t inchesTilCheck);
void approachWallBlindWithCutoff(uint8_t inchesTilCheck);
void forwardUntil(uint16_t stampedEnc);
void brake();
void turnRight();
void turnLeft();
void turnLeftALittleBit();


// Conversion Functions
uint16_t inchesToADC(uint8_t inches); 
uint16_t inchesToENC(uint8_t inches);

//
//
//


int main(void) {       
	
    // GPIO Setup // 
    DDRB &= ~(1 << PINB0);
    DDRC |=  ((1 << PORTC5)|(1 << PORTC4)|(1 << PORTC3)|(1 << PORTC2));    
	DDRC &= ~((1 << PINC1) | (1 << PINC0)); 
	DDRD |=  ((1 << PORTD5) | (1 << PORTD6) | (1 << PORTD7));
	
                                           
          

	// Timer 0 Setup (PCPWM)
	TCCR0A |= ((1 << COM0A1)| (1 << WGM00)| (1 << COM0B1)); // Configure TOP = 0xFF, Non-Inverting OC
	TCCR0B |=  (1 << CS00);
	OCR0A = 180;
	OCR0B = 180;

	// Timer 1 Setup (CTC, Ultrasonic)
	OCR1A = 49999;              // 0.2s before first pulse
	TCCR1B |= (1 << WGM12);     // Configure Timer 1 for CTC mode
	TIMSK1 |= (1 << OCIE1A);    // Enable Timer 1 compare interrupt

	// Timer 2 Setup (CTC, Ultrasonic)
	OCR2A = 157;                // 1ms with PS = 64
	TCCR2A |= (1 << WGM21);     // Enable Timer 2 compare interrupt
	TIMSK2 |= (1 << OCIE2A);    // Enable Timer 2 compare interrupt
	/*
	// External Interrupt Setup
	EICRA  |= (1 << ISC00);    // Configure for Any Edge
	EIMSK  |= (1 << INT0);*/
	// Pin Change Setup
	PCICR  |= ((1 << PCIE0) | (1 << PCIE1));    // Enable Pin Change Interrupt for PB0 & PC0
	PCMSK1 |= (1 << PCINT8);					// Configure pin change interrupt for PC0

    // ADC Setup 
    ADCSRA |= ((1 << ADEN) | (1 << ADIE));                  // Enable ADC, Enable Interrupts
    ADCSRA |= ((1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2)); // Configure Prescalar for 125kHz
    ADMUX  =  (1 << REFS0);                                 // Configure for 5V Vcc Reference
    ADMUX |= ((1<<MUX2)|(1<<MUX1)|(1<<MUX0));				//Set channel to ADC7
										
	//DIDR0 |=  (1 << ADC0D);		                            // Disable digital buffer on PC0

    
    //ADCSRA |= (1 << ADSC);  // Start First Conversion

	TCCR1B |= ((1 << CS11) | (1 << CS10));  // Start timer, waiting 0.2s
											// before first trigger pulse

	sei();          // ENABLE GLOBAL INTERRUPTS								
	uint8_t dicks = 1;
    while(1){
		while(dicks){
			dicks = 0;
			
			// First Lap
			approachWall(36); // 1st Leg
			turnRight();
			approachWall(77); // 2st Leg
			turnRight();
			approachWall(40); // 3st Leg
			turnRight();
			approachWall(40); // 4st Leg
			turnRight();
			approachWall(24); // 5st Leg
			turnLeft();
			approachWall(24); // 6st Leg
			turnRight();

			// Second Lap
			approachWall(36); // 1st Leg
			turnRight();
			approachWall(77); // 2st Leg
			turnRight();
			forwardUntil(inchesToENC(34)); // Get to the Hole
			turnRight();
			forwardUntil(inchesToENC(22)); // Get in the Hole
			turnLeftALittleBit();
			approachWallBlindWithCutoff(3); // Pray, and go down sawtooth section
			turnLeftALittleBit();
			approachWall(3);				// Go home

			
			
			// Testing
// 			approachWall(6);
// 			turnRight();
// 			forwardUntil(inchesToENC(34)); // Get to the Hole
// 			turnRight();
// 			forwardUntil(inchesToENC(22)); // Get in the Hole
// 			turnLeftALittleBit();
// 			approachWallBlindWithCutoff(3); // Pray, and go down sawtooth section
// 			turnLeftALittleBit();
// 			approachWall(3);				// Go home
			//approachWall(8);
			// End Test Code 
		}
		//approachWall(inchesToENC(36));
	}
        
    
    return 0;           
}



//ISR(PCINT2_vect, ISR_ALIASOF(PCINT1_vect));

uint16_t inchesToADC(uint8_t inches){
	uint16_t ADCout = 4565/((inches * 5) >> 1) + 31;
	return ADCout;

}

uint16_t inchesToENC(uint8_t inches){
	uint16_t ENCout = 24 * inches;
	return ENCout;
}



void brake(){
	PORTC &= ~((1 << PORTC2) | (1 << PORTC3)  | (1 << PORTC4) | (1 << PORTC5));

	TCNT2 = 0;
	TCCR2B |= ((1 << CS22) | (1 << CS21) | (1 << CS20));	//Start Timer 2, triggers roughly every 0.01s, PS = 1024
	while(cs < 10){
		//wait
	}
	TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20));	//Stop Timer 2
	cs = 0;
}

void approachWall(uint8_t inchesTilCheck){
// This function moves the robot forward, running parallel to the left wall.
// After the amount of ticks (from the encoder) speicified by longDistInTicks,
// the robot will check the forward IR sensor every 1-2 inches (depending on 
// distance to the wall), and stop when it reaches 4in distance to the wall.  

	uint16_t longDistInTicks = inchesToENC(inchesTilCheck);
	uint16_t minDistInADC = inchesToADC(4);
	uint8_t wallReached = 0;				 // Assuming wall not reached yet.  
	uint8_t ADCFreqInTicks = inchesToADC(2); // Two inches by default per ADC conversion.
	doneConverting = 0; 
	encCount = 0;

	while(encCount < longDistInTicks){ 
		// Continue forward as normal until
		// the encoder shows a certain distance travelled. 
		forwardWithControl();
	}
	ADCSRA |= (1 << ADSC);  // Start First Conversion
	encCount = 0;			// Reset Enc to limit ADC frequency
	

	while(!wallReached){
		// After long distance is reached, 
		// the system checks the IR every 1-2 inches. 
		forwardWithControl();
		if(encCount > ADCFreqInTicks){ 
			encCount = 0;
			ADCSRA |= (1 << ADSC);  // Start Conversion, takes ~8us to settle
		}
		if(doneConverting){
			doneConverting = 0;

			// Verify that the frequency with which the robot
			// checks the fwd. IR sensor is appropriate, and
			// when the robot is 16 in. from the wall, increase 
			// that frequency to 1 conversion per inch travelled. 
			if((ADCFreqInTicks > 24) && ADCin > 143){
				ADCFreqInTicks = 12; // 12 ticks = 1/2 inch 
			}
			// When the robot reads 4in. to wall, function ends. 
			if(ADCin > minDistInADC){ 
				wallReached = 1;
			}
		}
	}
	brake();
}


void approachWallBlindWithCutoff(uint8_t inchesTilCheck){
	// This function moves the robot forward, running parallel to the left wall.
	// After the amount of ticks (from the encoder) speicified by longDistInTicks,
	// the robot will check the forward IR sensor every 1-2 inches (depending on
	// distance to the wall), and stop when it reaches 4in distance to the wall.

	uint16_t longDistInTicks = inchesToENC(inchesTilCheck);
	uint16_t minDistInADC = inchesToADC(6);
	uint8_t wallReached = 0;				 // Assuming wall not reached yet.
	uint8_t ADCFreqInTicks = inchesToADC(2); // Two inches by default per ADC conversion.
	uint16_t cutoffDistInENC = inchesToENC(18);
	uint16_t encStorage = 0;
	doneConverting = 0;
	encCount = 0;

	while(encCount < longDistInTicks){
		// Continue forward as normal until
		// the encoder shows a certain distance travelled.
		forwardWithoutControl();
	}
	ADCSRA |= (1 << ADSC);  // Start First Conversion
	encCount = 0;			// Reset Enc to limit ADC frequency
	

	while((!wallReached) && (encStorage < cutoffDistInENC)){
		// After long distance is reached,
		// the system checks the IR every 1-2 inches.
		forwardWithoutControl();
		if(encCount > ADCFreqInTicks){
		encStorage += encCount;
			encCount = 0;
			ADCSRA |= (1 << ADSC);  // Start Conversion, takes ~8us to settle
		}
		if(doneConverting){
			doneConverting = 0;

			// Verify that the frequency with which the robot
			// checks the fwd. IR sensor is appropriate, and
			// when the robot is 16 in. from the wall, increase
			// that frequency to 1 conversion per inch travelled.
			if((ADCFreqInTicks > 24) && ADCin > 143){
				ADCFreqInTicks = 12; // 12 ticks = 1/2 inch
			}
			// When the robot reads 4in. to wall, function ends.
			if(ADCin > minDistInADC){
				wallReached = 1;
			}
		}
	}
	brake();
}

void turnRight(){
	brake();
	encCount = 0;
	while(encCount < 100){
		PORTC &= ~((1 << PORTC2) | (1 << PORTC5));
		PORTC |=  ((1 << PORTC3) | (1 << PORTC4));
		OCR0A = 190;
		OCR0B = 190;
	}
	brake();
}

void turnLeft(){
	brake();
	encCount = 0;
	while(encCount > -85){
		PORTC &= ~((1 << PORTC3) | (1 << PORTC4));
		PORTC |=  ((1 << PORTC2) | (1 << PORTC5));
		OCR0A = 190;
		OCR0B = 190;
	}
	brake();
}

void turnLeftALittleBit(){
	brake();
	encCount = 0;
	while(encCount > -16){
		PORTC &= ~((1 << PORTC3) | (1 << PORTC4));
		PORTC |=  ((1 << PORTC2) | (1 << PORTC5));
		OCR0A = 190;
		OCR0B = 190;
	}
	brake();
}


void forwardWithoutControl(){
	OCR0A = 208;
	OCR0B = 212;
	PORTC &= ~((1 << PORTC2) | (1 << PORTC4));
	PORTC |=  ((1 << PORTC3) | (1 << PORTC5));

}

void forwardWithControl(){

	// The code that regulates master 
	// and slave motor speeds, mimicking negative
	// feedback from the ultrasonic sensor with 
	// a simple Goldilocks-zone threshold.
	uint16_t pulseWidth = 0;
	static uint8_t firstStamp = 1;
	// H-Bridge Logic for forward movement
	PORTC &= ~((1 << PORTC2) | (1 << PORTC4));
	PORTC |=  ((1 << PORTC3) | (1 << PORTC5));
	 
	if(needsToCalculate){
		needsToCalculate = 0;
		cli();
		pulseWidth = ICR1;
		if((pulseWidth < 75)){			//5cm
			OCR0A = 196;
			OCR0B = 180;
		}else if((pulseWidth > 90)  && (pulseWidth < 247)){		//6cm
			OCR0A = 180;
			OCR0B = 200;
		}
		else{
		if((pulseWidth > 247) && firstStamp){
			firstStamp = 0;
			encStamp = encCount;
			}
			OCR0A = 196;
			OCR0B = 200;
		}
		sei();
	}
}

void forwardUntil(uint16_t stampedEnc){
	encCount = 0;
	while(encCount < stampedEnc){
		forwardWithControl();
	}
	encCount = 0;
	brake();
}

ISR(ADC_vect){
    doneConverting = 1;
    ADCin = ADC;
}

ISR(PCINT1_vect){

	// NEED DOCUMENTATION HERE
	static int8_t encoderTable[] = {0,0,0,-1,0,0,1,0,0,1,0,0,-1,0,0,0};
    static uint8_t encoderValue = 0;
    static uint8_t internalCounter = 0;

	internalCounter++;
	if(internalCounter > 50){
		internalCounter = 0;
		encoderValue <<= 2;
		encoderValue |= ((PINC & 0x03));

		encCount += encoderTable[encoderValue & 0x0F];
		encoderValue &= 0x03;
	}
	
}

ISR(TIMER1_COMPA_vect) {
	// Main Timer Interrupt//
	if(USSstate == calculating){
		USSstate = triggerPulse;
		TCCR1B &= ~((1 << CS11) | (1 << CS10)); // Kill Timer 1
		OCR1A = 160;           // Time for 10us
		TCNT1 = 0;
		PORTD  |=  (1 << PORTD7);			   // Pull PD7 High
		TCCR1B |= (1 << CS10); // Start Timer 1, PS 64

	}
		
	else if(USSstate == triggerPulse){
		USSstate = waitforRise;
		PORTD  &= ~(1 << PORTD7);   // Pull PD7 Low
		TCCR1B &= ~(1 << CS10);		// Stop Timer 1
		PCIFR   =  (1 << PCIF0);    // Clear PC flags
		PCMSK0 |=  (1 << PCINT0);   // Enable PC Interrupt

	}

	else if(USSstate == distanceTiming){
		USSstate = calculating;
		OCR1A = 12500;  // Time for 50ms
		TCNT1 = 0;
		TCCR1B |= ((1 << CS11) | (1 << CS10)); // Start Timer 1
		TIMSK1 &=  ~(1 << ICIE1); // Kill Input Capture Interrupt
		needsToCalculate = 1;
	}

}

ISR(TIMER2_COMPA_vect) {
	// Timer 2 Compare Interrupt
	cs++;
}


ISR(PCINT0_vect){
	// PIN Change Interrupt
	USSstate = distanceTiming;
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

	USSstate = calculating;
	TIMSK1 &=  ~(1 << ICIE1); // Kill Input Capture Interrupt
	TIMSK1 |=  (1 << OCIE1A); // Enable CTC
	OCR1A = 12500-ICR1;           // Time for 50ms - pulseWidth
	TCNT1 = 0;               // Clear Timer
	TCCR1B |= ((1 << CS11) | (1 << CS10)); // Start Timer 1, PS 64
	needsToCalculate = 1;
}

/*
ISR(INT0_vect) {
	// External Start/Stop Pushbutton Interrupt
	static uint8_t firstTrigger = 1;
	
	// Toggle firstTrigger
	firstTrigger = firstTrigger == 1 ? 0 : 1;

}
*/
