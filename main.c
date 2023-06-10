/*
 * RobotCar.c
 *
 * Created: 04/06/2023 13:33:19
 * Author : talha
 */ 

#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>
#define TRIG_PIN PD0
#define ECHO_PIN PD1
#define LEFT_MOTOR_PIN1 PD3
#define LEFT_MOTOR_PIN2 PB1
#define RIGHT_MOTOR_PIN1 PB2
#define RIGHT_MOTOR_PIN2 PB3
#define MOTOR3_PIN1 PB4
#define MOTOR3_PIN2 PB5
#define MOTOR4_PIN1 PB6
#define MOTOR4_PIN2 PB7
#define robotPOT PC5
#define goalPOT PC4

#include "IncFile1.h"






int analogRead(uint8_t pin) {
	// Configure the ADC for single-ended mode with AVcc as the reference
	ADMUX = (1<<REFS0) | (pin & 0x0f);

	// Start the conversion
	ADCSRA |= (1<<ADSC);

	// Wait for the conversion to complete
	while (ADCSRA & (1<<ADSC));

	// Read the result
	return ADC;
}

void initialize_map(){
	// Read the values from the potentiometers
	int startPotValue = analogRead(robotPOT);
	int goalPotValue = analogRead(goalPOT);
	
	if(startPotValue > 800)
	currRow = 1;
	else if(startPotValue > 500 && startPotValue <800)
	currRow = 3;
	else if(startPotValue > 200 && startPotValue <500)
	currRow = 5;
	else if(startPotValue < 200 )
	currRow = 7;
	
	
	if(goalPotValue > 800)
	goalRow = 1;
	else if(goalPotValue > 500 && goalPotValue < 800)
	goalRow = 3;
	else if(goalPotValue > 200 && goalPotValue < 500)
	goalRow = 5;
	else if(goalPotValue < 200)
	goalRow = 7;
	
}



void initialise_motors()
{
	// Set motor pins as output
	DDRB |=  (1<<LEFT_MOTOR_PIN2) | (1<<RIGHT_MOTOR_PIN1) | (1<<RIGHT_MOTOR_PIN2) | (1 << MOTOR3_PIN1) | (1 << MOTOR3_PIN2) | (1 << MOTOR4_PIN1) | (1 << MOTOR4_PIN2);
	DDRD|=  (1<<LEFT_MOTOR_PIN1);
}

void forward()
{
	PORTB = 0x00;
	PORTD &= 0b11110111;
	PORTB |= (1 << RIGHT_MOTOR_PIN1) | (1 << MOTOR3_PIN1) | (1 << MOTOR4_PIN1);
	PORTD |= (1 << LEFT_MOTOR_PIN1);
	_delay_ms(490);
}
void backward()
{
	
	PORTB = 0x00;
	PORTD &= 0b11110111;
	PORTB |= (1 << LEFT_MOTOR_PIN2)|(1 << RIGHT_MOTOR_PIN2) | (1 << MOTOR3_PIN2) | (1 << MOTOR4_PIN2);
	_delay_ms(490);
}
void right()
{
	
	PORTB = 0x00;
	PORTD &= 0b11110111;
	PORTB |= (1 << RIGHT_MOTOR_PIN2) | (1 << MOTOR3_PIN1) | (1 << MOTOR4_PIN2);
	PORTD |= (1 << LEFT_MOTOR_PIN1);
	_delay_ms(640);
}
void left()
{
	
	PORTB = 0x00;
	PORTD &= 0b11110111;
	PORTB |= (1 << LEFT_MOTOR_PIN2) | (1 << RIGHT_MOTOR_PIN1) | (1 << MOTOR3_PIN2) | (1 << MOTOR4_PIN1);
	_delay_ms(640);
}
void initialise_ultrasonic_sensor()
{
	// Set trigger pin as output
	DDRD|= (1<<TRIG_PIN);
	
	// Set echo pin as input
	DDRD &= ~(1<<ECHO_PIN);
}
int get_distance_cm()
{
	int distance;
	long duration;
	
	// Send 10us pulse to trigger pin
	PORTD |= (1<<TRIG_PIN);
	_delay_us(10);
	PORTD &= ~(1<<TRIG_PIN);
	
	// Measure pulse duration on echo pin
	duration = 0;
	while ((PIND & (1<<ECHO_PIN)) == 0 && duration < 10000) {
		duration++;
		_delay_us(1);
	}
	
	while ((PIND & (1<<ECHO_PIN)) && duration < 10000) {
		duration++;
		_delay_us(1);
	}
	
	// Convert duration to distance in cm
	distance = duration / 100;
	return distance;
}


void stop_moving(){
	PORTB = 0x00;
	PORTD &= 0b11110111;
	_delay_ms(400);
}

void avoid_obstacle(){
	if(currRow>0 && currRow<7){
		right();
		stop_moving();
		
		if(get_distance_cm()<0.05){
			avoid_obstacleROW1();
			currCol--;
			
			
		}
		
		else if(get_distance_cm()>0.05){
			forward();
			left();
			if(get_distance_cm()<0.05){
				avoid_obstacle();
				currRow++;
			}
			forward();
			if(get_distance_cm()<0.05){
				avoid_obstacle();
				currRow++;
			}
			forward();
			if(get_distance_cm()<0.05){
				avoid_obstacle();
				currRow++;
			}
		}
		currRow++;
		currCol+=2;
	}
	if(currRow==7){
		left();
		if(get_distance_cm()<0.05){
			avoid_obstacleROW2();
			currCol--;
		}
		else if(get_distance_cm()>0.05){
			forward();
			right();
			if(get_distance_cm()<0.05){
				avoid_obstacle();
				currRow--;
			}
			forward();
			if(get_distance_cm()<0.05){
				avoid_obstacle();
				currRow--;
			}
			forward();
			if(get_distance_cm()<0.05){
				avoid_obstacle();
				currRow--;
			}
		}
		currRow--;
		currCol+=2;
	}
	
}


void avoid_obstacleROW1(){
	stop_moving();
	
	if(currCol==0){
		left();
		stop_moving();
		left();
		stop_moving();
		
		forward();
		stop_moving();
		currRow--;
		if (get_distance_cm()<0.05){
			avoid_obstacleROW2();
		}
		else{
			right();
			stop_moving();
			forward();
			stop_moving();
			if(get_distance_cm()<0.05){
				obstacle_special();
			}
			currCol++;
		}
		
	}
	else if(currCol>0 && currCol<7){
		right();
		stop_moving();
		if(get_distance_cm()<0.05){
			right();
			stop_moving();
			forward();
			stop_moving();
			right();
			stop_moving();
			currRow--;
		}
		else if(get_distance_cm()>0.05){
			forward();
			stop_moving();
			left();
			stop_moving();
			if(get_distance_cm()<0.05){
				avoid_obstacleROW1();
				currCol--;
			}
			forward();
			stop_moving();
			if(get_distance_cm()<0.05){
				avoid_obstacleROW1();
				currCol--;
			}
			forward();
			stop_moving();
			if(get_distance_cm()<0.05){
				avoid_obstacleROW1();
				currCol--;
			}
			right();
			stop_moving();
		}
		currCol--;
		currRow+=2;
		navigate();
	}
	else if (currCol==7)
	{
		right();
		stop_moving();
		if (get_distance_cm()<0.05)
		{
			right();
			stop_moving();
			forward();
			stop_moving();
			left();
			stop_moving();
			forward();
			stop_moving();
			forward();
			stop_moving();
			left();
			stop_moving();
			currCol-=2;
			
			
		}
		forward();
		stop_moving();
		left();
		stop_moving();
		if(get_distance_cm()<0.05){
			currCol--;
			avoid_obstacleROW1();
		}
		forward();
		stop_moving();
		right();
		stop_moving();
		
		
		count--;
	}
}

void avoid_obstacleROW2(){
	stop_moving();
	_delay_ms(500);
	if(currCol==0 && currRow==1){
		right();
		stop_moving();
		forward();
		stop_moving();
		currCol++;
		
		
	}
	
	else if(currCol==0){
		left();
		stop_moving();
		left();
		stop_moving();
		forward();
		stop_moving();
		currRow++;
		
		left();
		stop_moving();
		
	}
	else if(currCol>0 && currCol<7){
		left();
		stop_moving();
		forward();
		stop_moving();
		right();
		stop_moving();
		forward();
		stop_moving();
		forward();
		stop_moving();
		currCol--;
		currRow-=2;
		
		
	}
	else if (currCol==7)
	{
		left();
		stop_moving();
		if (get_distance_cm()<0.05)
		{
			left();
			stop_moving();
			forward();
			stop_moving();
			right();
			stop_moving();
			forward();
			stop_moving();
			forward();
			stop_moving();
			right();
			stop_moving();
			forward();
			stop_moving();
			forward();
			stop_moving();
			right();
			stop_moving();
			currRow--;
			currCol-=2;
			
			
		}
		else {
			forward();
			stop_moving();
			right();
			stop_moving();
			if(get_distance_cm()<0.05){
				currCol--;
				avoid_obstacleROW2();
			}
			forward();
			stop_moving();
			forward();
			stop_moving();
			right();
			stop_moving();
			currRow-=2;
			currCol--;
		}
		count--;
		
		
	}
}


void obstacle_special(){
	left();
	stop_moving();
	forward();
	stop_moving();
	right();
	stop_moving();
	forward();
	stop_moving();
	forward();
	stop_moving();
	right();
	stop_moving();
	forward();
	stop_moving();
	left();
	stop_moving();
	currCol+=2;
	
	navigate();
}

void NextCol(){
	if(get_distance_cm()<0.05){
		avoid_obstacle();
	}
	else {
		forward();
		currCol++;
		
	}
}

void NextRow(){
	if(get_distance_cm()<0.05)
	avoid_obstacleROW1();

	else{
	forward();
	currRow++;
	
}
}
void PrevRow(){
	if(get_distance_cm()<0.05){
		avoid_obstacleROW2();
		
	}
	else{
		forward();
		currRow--;
		
	}
}

void navigate(){
	if(currCol<goalCol){
		NextCol();
	}
	
	while(currCol==goalCol){
		if (currRow<goalRow)
		{
			if(count==0){
				right();
				count++;
			}
			
			stop_moving();
			NextRow();
		}
		else if(currRow>goalRow){
			if(count==0){
				left();
				count++;
			}
			stop_moving();
			PrevRow();
		}
		else if(currRow==goalRow){
			stop_moving();
			_delay_ms(10000);
		}
	}
}



int main()
{
	// Configure ADC settings
	ADMUX = (1 << REFS0);  // Set reference voltage to AVcc (5V)
	ADCSRA = (1 << ADEN);  // Enable ADC
 // Enable ADC and set prescaler to 128 (ADC clock = system clock / 128)
 ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
 
 //Define Map
 pitchRow = 7;
 pitchCol = 7;
  goalCol = 7;
  goalRow = 7;
  startCol = 0;
  currCol = 0;
  count=0;

 

 
  initialize_map();
  initialise_motors();
  initialise_ultrasonic_sensor();
while(1){
 

 
 navigate();
}

	return 0;
	}   
