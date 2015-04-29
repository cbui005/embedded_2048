#include <avr/io.h>
#include <stdlib.h>
#define F_CPU 1000000UL
#include "avr/interrupt.h"
#include "avr/delay.h"
#define true 1
#define false 0
typedef char bool;
unsigned int grid[4][4];
unsigned int grid_prev[4][4];

/* initializing pins on Atmega1284 for fast clock cycle on PWM mode.
Using ports B and D as input
*/
#ifndef TLC5940_GS_PORT
#define TLC5940_GS_PORT PORTD
#endif
#ifndef TLC5940_GS_PIN
#define TLC5940_GS_PIN 5
#endif
// serial clock - PD5 -OCR0B
//OCR0B = pb4
#ifndef TLC5940_SCK_PORT
#define TLC5940_SCK_PORT PORTB
#endif
#ifndef TLC5940_SCK_PIN
#define TLC5940_SCK_PIN 4
#endif
// latch - PD4
#ifndef TLC5940_XLAT_PORT
#define TLC5940_XLAT_PORT PORTD
#endif
#ifndef TLC5940_XLAT_PIN
#define TLC5940_XLAT_PIN 1
#endif
// programming select - PD7
#ifndef TLC5940_VPRG_PORT
#define TLC5940_VPRG_PORT PORTD
#endif
#ifndef TLC5940_VPRG_PIN
#define TLC5940_VPRG_PIN 0
#endif
// blank outputs - pullup resistor + PD3. OCR2B
//OCR2B = PD6
#ifndef TLC5940_BLANK_PORT
#define TLC5940_BLANK_PORT PORTD
#endif
#ifndef TLC5940_BLANK_PIN
#define TLC5940_BLANK_PIN 6
#endif
// serial data master out slave in - PD6. OCR0A
//OCR0B = pb4
#ifndef TLC5940_MOSI_PORT
#define TLC5940_MOSI_PORT PORTB
#endif
#ifndef TLC5940_MOSI_PIN
#define TLC5940_MOSI_PIN 3
#endif

#ifndef TLC5940_N
#define TLC5940_N 2
#endif

#define TLC5940_LED_N 16 * TLC5940_N
uint8_t dc[TLC5940_LED_N];
uint16_t gs[TLC5940_LED_N];
volatile bool newData;

#define DDR(port) (*(&port-1))

// give the variables some default values
TLC5940(void) {
	// initialize variables at all leds off for safety and dot correction to full brightness
	for (uint8_t i=0; i<(16 * TLC5940_N); i++) {
		setDC(i, 63);
	}
	for (uint8_t i=0; i<(16 * TLC5940_N); i++) {
		setGS(i, 0);
	}
	newData = false;
}

void transmit_data(unsigned char data) {
	int i;
	for (i = 0; i < 8 ; ++i) {
		// Sets SRCLR to 1 allowing data to be set
		// Also clears SRCLK in preparation of sending data
		PORTC = 0x08;
		// set SER = next bit of data to be sent.
		PORTC |= ((data >> i) & 0x01);
		// set SRCLK = 1. Rising edge shifts next bit of data into the shift register
		PORTC |= 0x02;
	}
	// set RCLK = 1. Rising edge copies data from “Shift” register to “Storage” register
	PORTC |= 0x04;
	// clears all lines in preparation of a new transmission
	PORTC = 0x00;
}

// initialize the pins and set dot correction
void init(void) {
	// initialize pins
	// gsclk - output set low initially
	DDR(TLC5940_GS_PORT) |= (1 << TLC5940_GS_PIN);
	TLC5940_GS_PORT &= ~(1 << TLC5940_GS_PIN);
	// sclk - output set low
	DDR(TLC5940_SCK_PORT) |= (1 << TLC5940_SCK_PIN);
	TLC5940_SCK_PORT &= ~(1 << TLC5940_SCK_PIN);
	// xlat - output set low
	DDR(TLC5940_XLAT_PORT) |= (1 << TLC5940_XLAT_PIN);
	TLC5940_XLAT_PORT &= ~(1 << TLC5940_XLAT_PIN);
	// blank - output set high (active high pin blanks output when high)
	DDR(TLC5940_BLANK_PORT) |= (1 << TLC5940_BLANK_PIN);
	TLC5940_BLANK_PORT &= ~(1 << TLC5940_BLANK_PIN);
	// serial data MOSI - output set low
	DDR(TLC5940_MOSI_PORT) |= (1 << TLC5940_MOSI_PIN);
	TLC5940_MOSI_PORT &= ~(1 << TLC5940_MOSI_PIN);
	// programming select - output set high
	DDR(TLC5940_VPRG_PORT) |= (1 << TLC5940_VPRG_PIN);
	TLC5940_VPRG_PORT |= (1 << TLC5940_VPRG_PIN);

	// set vprg to 1 (program dc data)
	TLC5940_VPRG_PORT |= (1 << TLC5940_VPRG_PIN);
	// set serial data to high (setting dc to 1)
	TLC5940_MOSI_PORT |= (1 << TLC5940_MOSI_PIN);

	// pulse the serial clock (96 * number-of-drivers) times to write in dc data
	for (uint16_t i=0; i<(96 * TLC5940_N); i++) {
		// get the bit the tlc5940 is expecting from the gs array (tlc expects msb first)
		uint8_t data = (dc[((96 * TLC5940_N) - 1 - i)/6]) & (1 << ((96 * TLC5940_N) - 1 - i)%6);
		// set mosi if bit is high, clear if bit is low
		if (data) {
			TLC5940_MOSI_PORT |= (1 << TLC5940_MOSI_PIN);
		}
		else {
			TLC5940_MOSI_PORT &= ~(1 << TLC5940_MOSI_PIN);
		}
		TLC5940_SCK_PORT |= (1 << TLC5940_SCK_PIN);
		TLC5940_SCK_PORT &= ~(1 << TLC5940_SCK_PIN);
	}

	// pulse xlat to latch the data
	TLC5940_XLAT_PORT |= (1 << TLC5940_XLAT_PIN);
	TLC5940_XLAT_PORT &= ~(1 << TLC5940_XLAT_PIN);

	// enable leds
	TLC5940_BLANK_PORT &= ~(1 << TLC5940_BLANK_PIN);
}

// refresh the led display and data
void refreshGS(void) {
	bool gsFirstCycle = false;
	static bool needLatch = false;

	// disable leds before latching in new data
	TLC5940_BLANK_PORT |= (1 << TLC5940_BLANK_PIN);

	// check if vprg is still high
	if ( TLC5940_VPRG_PORT & (1 << TLC5940_VPRG_PIN) ) {
		// pull VPRG low and set the first cycle flag
		TLC5940_VPRG_PORT &= ~(1 << TLC5940_VPRG_PIN);
		gsFirstCycle = true;
	}

	// check if we need a latch
	if (needLatch) {
		needLatch = false;
		TLC5940_XLAT_PORT |= (1 << TLC5940_XLAT_PIN);
		TLC5940_XLAT_PORT &= ~(1 << TLC5940_XLAT_PIN);
	}

	// check if this was the first gs cycle after a dc cycle
	if (gsFirstCycle) {
		gsFirstCycle = false;
		// pulse serial clock once if it is (because the datasheet tells us to)
		TLC5940_SCK_PORT |= (1 << TLC5940_SCK_PIN);
		TLC5940_SCK_PORT &= ~(1 << TLC5940_SCK_PIN);
	}

	// enable leds
	TLC5940_BLANK_PORT &= ~(1 << TLC5940_BLANK_PIN);
	
	// clock in new gs data
	needLatch = serialCycle();
}


int serialCycle(void) {
	// if there's data to clock in
	if (newData) {
		newData = false;
		for (uint16_t dataCount=0; dataCount<192*TLC5940_N; dataCount++) {
			// get the bit the tlc5940 is expecting from the gs array (tlc expects msb first)
			uint16_t data = (gs[((192 * TLC5940_N) - 1 - dataCount)/12]) & (1 << ((192 * TLC5940_N) - 1 - dataCount)%12);
			// set mosi if bit is high, clear if bit is low
			if (data) {
				TLC5940_MOSI_PORT |= (1 << TLC5940_MOSI_PIN);
			}
			else {
				TLC5940_MOSI_PORT &= ~(1 << TLC5940_MOSI_PIN);
			}
			// pulse the serial clock
			TLC5940_SCK_PORT |= (1 << TLC5940_SCK_PIN);
			TLC5940_SCK_PORT &= ~(1 << TLC5940_SCK_PIN);
		}
		return true;
	}
	return false;
}

// set the new data flag
void update(void) {
	newData = true;
}
int wonGame = 0;
void checkWin()
{
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			if (grid[i][j] == 2048)
			{
				wonGame = 1;
				return;
			}
		}
	}
}
// set the brightness of an individual led
void setDC(uint8_t led, uint8_t val) {
	// basic parameter checking
	// check if led is inbounds
	if (led < (16 * TLC5940_N)) {
		// if value is out of bounds, set to max
		if (val < 64) {
			dc[led] = val;
		}
		else {
			dc[led] = 63;
		}
	}
}

// set the brightness of an individual led
void setGS(uint8_t led, uint16_t val) {
	// basic parameter checking
	// check if led is inbounds
	if (led < (16 * TLC5940_N)) {
		// if value is out of bounds, set to max
		if (val < 4096) {
			gs[led] = val;
		}
		else {
			gs[led] = 4095;
		}
	}
}

// loop counter
uint16_t count;
int8_t dir;

void setup(void) {
	TLC5940();
	count = 200;
	dir = 1;

	// set DC to full
	for (uint8_t i=0; i<TLC5940_LED_N; i++) {
		setDC(i, 63);
	}

	init();

	cli();

	// user timer 1 to toggle the gs clock pin
	TCCR1A = 0;
	TCCR1B = 0;
	TCCR1C = 0;
	TIMSK1 = 0;
	// toggle OC1A (pin B1) on compare match event
	
	TCCR1A |= (1 << COM1A0);
	// set the top of the timer
	// PS = 1, F_CPU = 16 MHz, F_OC = F_CPU/(2 * PS * (OCR1A+1)
	// gs edge gets sent every 32*2=64 clock ticks
	OCR1A = 31;

	// put the timer in CTC mode and start timer with no prescaler

	TCCR1B |= ( (1 << WGM12) | (1 << CS10) );

	// set up an isr for the serial cycle to live in
	// let it live in timer 0
	TCCR0A = 0;
	TCCR0B = 0;
	TIMSK0 = 0;
	// set waveform generation bit to put the timer into CTC mode

	TCCR0A |= (1 << WGM01);
	// set the top of the timer - want this to happen every 4096 * gs clocks = every 8192 clock ticks
	// set top to 255 for an interrupt every 256 * 1024 = 64 * 4096 clock ticks
	OCR0A = 255;
	// start the timer with a 1024 prescaler
	TCCR0B |= ( (1 << CS02) | (1 << CS00) );
	// enable the interrupt of output compare A match
	TIMSK0 |= (1 << OCIE0A);

	sei();
}

void loop(void) {
	/* list of colors:
	
	red: full red, no green, no blue
	orange: red 5000, green 1000, no blue
	yellow: red 5000, green 2300, no blue
	lime green: full red, full green, no blue
	green: 0 red, full green, 0 blue
	aqua: 0 red, full green, full blue
	blue: 0 red, 0 green, full blue
	purple: full red, 0 green, full blue
	violet: red 1000, green 1000, full blue
	pink: red 5000, green 1000, full blue
	all on: white
	*/
	int max = grid[0][0];
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			if (grid[i][j] > max)
			{
				max = grid[i][j];
			}
		}
	}
	if (max == 0)
	{
		for (uint8_t i=0; i<TLC5940_LED_N; i= i+2) {
			setGS(i, 0);

		}
		for (uint8_t i=1; i<TLC5940_LED_N; i=i+2) {
			setGS(i, 0);
		}
		transmit_data(255);
	}
	if (max == 2)
	{
		for (uint8_t i=0; i<TLC5940_LED_N; i= i+2) {
			setGS(i, 5000);

		}
		for (uint8_t i=1; i<TLC5940_LED_N; i=i+2) {
			setGS(i, 1000);
		}
		transmit_data(0);
	}
	if (max == 4)
	{
		for (uint8_t i=0; i<TLC5940_LED_N; i= i+2) {
			setGS(i, 1000);

		}
		for (uint8_t i=1; i<TLC5940_LED_N; i=i+2) {
			setGS(i, 1000);
		}
		transmit_data(0);
	}
	if (max == 8)
	{
		for (uint8_t i=0; i<TLC5940_LED_N; i= i+2) {
			setGS(i, 5000);

		}
		for (uint8_t i=1; i<TLC5940_LED_N; i=i+2) {
			setGS(i, 0);
		}
		transmit_data(0);
	}
	if (max == 16)
	{
		for (uint8_t i=0; i<TLC5940_LED_N; i= i+2) {
			setGS(i, 0);

		}
		for (uint8_t i=1; i<TLC5940_LED_N; i=i+2) {
			setGS(i, 0);
		}
		transmit_data(0);
	}
	if (max == 32)
	{
		for (uint8_t i=0; i<TLC5940_LED_N; i= i+2) {
			setGS(i, 1000);

		}
		for (uint8_t i=1; i<TLC5940_LED_N; i=i+2) {
			setGS(i, 5000);
		}
		transmit_data(0);
	}
	if (max == 64)
	{
		for (uint8_t i=0; i<TLC5940_LED_N; i= i+2) {
			setGS(i, 0);

		}
		for (uint8_t i=1; i<TLC5940_LED_N; i=i+2) {
			setGS(i, 5000);
		}
		transmit_data(255);
	}
	if (max == 128)
	{
		for (uint8_t i=0; i<TLC5940_LED_N; i= i+2) {
			setGS(i, 5000);

		}
		for (uint8_t i=1; i<TLC5940_LED_N; i=i+2) {
			setGS(i, 5000);
		}
		transmit_data(255);
	}
	if (max == 256)
	{
		for (uint8_t i=0; i<TLC5940_LED_N; i= i+2) {
			setGS(i, 5000);

		}
		for (uint8_t i=1; i<TLC5940_LED_N; i=i+2) {
			setGS(i, 2300);
		}
		transmit_data(255);
	}
	if (max == 512)
	{
		for (uint8_t i=0; i<TLC5940_LED_N; i= i+2) {
			setGS(i, 5000);

		}
		for (uint8_t i=1; i<TLC5940_LED_N; i=i+2) {
			setGS(i, 1000);
		}
		transmit_data(255);
	}
	if (max == 1024)
	{
		for (uint8_t i=0; i<TLC5940_LED_N; i= i+2) {
			setGS(i, 5000);

		}
		for (uint8_t i=1; i<TLC5940_LED_N; i=i+2) {
			setGS(i, 0);
		}
		transmit_data(255);
	}
	if (max == 2048)
	{
		for (uint8_t i=0; i<TLC5940_LED_N; i= i+2) {
			setGS(i, 3300);

		}
		for (uint8_t i=1; i<TLC5940_LED_N; i=i+2) {
			setGS(i, 5000);
		}
		transmit_data(0);
	}
	/*
	// give it some new data
	for (uint8_t i=0; i<TLC5940_LED_N; i++) {
		setGS(i, count);
	}
	// tell the driver to update
	update();

	// set loop direction
	if (dir==1 && count>=4000) {
		dir = -1;
	}
	else if (dir==-1 && count<=300) {
		dir = 1;
	}
	// increment counter
	count += dir*100;

	// delay
	_delay_ms(20);
	*/
	
	
}

// ISR for serial data input into TLC5940
ISR(TIMER0_COMPA_vect) {
	refreshGS();
}

#define DIN			PINB5		// data in							->	MOSI
#define CLK			PINB7		// clocking the data to the LCD		->	SCLK
#define CE			PINB0		// chip enable, active low			
#define DC			PINB2		// data (high)/character(low)
#define RST			PINB1		// reset pin
#define LCD_Port	PORTB
#define LCD_Port_Datadir	DDRB

#define Black 1
#define White 0
#define Filled 1
#define NotFilled 0

void SendCommand_LCD (unsigned int CommandOut);								//sends a command to the lcd for control
void SendData_LCD (unsigned int DataOut);									//sends data to the lcd for displaying
void LCDInitialize(void);													//set up the lcd
void LCD_Update (void);														//transfer LCDBuffer to the lcd
void LCD_clear(void);														//clear the lcd, also fills the LCDBuffer with 0's
void LCD_gotoXY ( unsigned char x, unsigned char y );						//used for characters (6 banks of y positions)
void DrawRectangle (int x1, int y1, int x2, int y2,char bw, char fill);		//draw rectangle, specify upr left and lwr right corner, pixel color (Black/White), Filled/NotFilled
void ReplaceBitmap (char ReplacementBitmap[]);								//fills the LCDBuffer with the bytes of a ReplacementBitmap
void AddBitmap (char AddedBitmap[]);										//adds to the LCDBuffer by masking an AddedBitmap
void SetPixel (unsigned char xp, unsigned char yp, char bw);				//draw a single pixel specify x,y location and Black/White 
void DrawLine (char x1, char y1, char x2, char y2, char bw);

 char LCDBuffer[] = {
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 0,0 - 15,7
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 16,0 - 31,7
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 32,0 - 47,7
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 48,0 - 63,7
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 64,0 -	79,7
	0x00,0x00,0x00,0x00,															 //pixel 80,0 - 83,7
	
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 0,7 - 15,15
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 16,7 - 31,15
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 32,7 - 47,15
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 48,7 - 63,15
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 64,7 -	79,15
	0x00,0x00,0x00,0x00,															 //pixel 80,7 - 83,15
	
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 0,16 - 15,23
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 16,16 - 31,23
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 32,16 - 47,23
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 48,16 - 63,23
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 64,16 - 79,23
	0x00,0x00,0x00,0x00,															 //pixel 80,16 - 83,23
	
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 0,24 - 15,31
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 16,24 - 31,31
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 32,24 - 47,31
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 48,24 - 63,31
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 64,24 - 79,31
	0x00,0x00,0x00,0x00,															 //pixel 80,24 - 83,31
	
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 0,32 - 15,39
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 16,32 - 31,39
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 32,32 - 47,39
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 48,32 - 63,39
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 64,32 - 79,39
	0x00,0x00,0x00,0x00,															 //pixel 80,32 - 83,39
	
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 0,40 - 15,47
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 16,40 - 31,47
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 32,40 - 47,47
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 48,40 - 63,47
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //pixel 64,40 - 79,47
	0x00,0x00,0x00,0x00,															 //pixel 80,40 - 83,47
	
};

const unsigned char lose [] = {
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x9F, 0xFF, 0x5F, 0x7F, 0xDF, 0xBF,
	0x7F, 0x7F, 0xBF, 0xDF, 0xFF, 0x5F, 0x5F, 0xDF, 0xBF, 0x7F, 0x7F, 0xDF, 0xDF, 0xBF, 0xBF, 0xDF,
	0xDF, 0x3F, 0x7F, 0x9F, 0xDF, 0xFF, 0x7F, 0x5F, 0x7F, 0x5F, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F,
	0xDF, 0xFF, 0x7F, 0x5F, 0xDF, 0xBF, 0x7F, 0xFF, 0xFF, 0xDF, 0x3F, 0x3F, 0xDF, 0xDF, 0xBF, 0x3F,
	0x3F, 0xDF, 0xDF, 0x7F, 0x7F, 0x5F, 0x5F, 0x3F, 0x7F, 0x9F, 0xDF, 0x7F, 0x7F, 0xDF, 0xDF, 0xBF,
	0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFA, 0xF7, 0xEF,
	0xE8, 0xE9, 0xE7, 0xEF, 0xE0, 0xE0, 0xFF, 0xEF, 0xE6, 0xE2, 0xE6, 0xEF, 0xEF, 0xE0, 0xF0, 0xEF,
	0xEF, 0xE1, 0xE1, 0xEF, 0xEF, 0xE0, 0xE0, 0xF6, 0xEF, 0xEF, 0xE9, 0xEB, 0xE9, 0xE8, 0xE6, 0xFF,
	0xFF, 0xFF, 0xFF, 0xF1, 0xE7, 0xEF, 0xE8, 0xE8, 0xEF, 0xE7, 0xE0, 0xFE, 0xF9, 0xFB, 0xF6, 0xEC,
	0xE7, 0xE3, 0xF1, 0xF8, 0xF0, 0xEF, 0xEF, 0xEB, 0xEB, 0xEB, 0xE8, 0xE2, 0xFF, 0xE5, 0xEF, 0xE7,
	0xE3, 0xE7, 0xEF, 0xE4, 0xE0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
};
const unsigned char test [] = {
	0x00, 0x00, 0xF8, 0xFC, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE,
	0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0x7E, 0x3E, 0x3E, 0x3E, 0x3E, 0x7E, 0xFE, 0xFE, 0xFE, 0xFE,
	0xFE, 0x7E, 0x3E, 0x3E, 0x3E, 0x3E, 0x7E, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0x3E, 0x3E, 0x3E,
	0x3E, 0x3E, 0xFE, 0xFE, 0xFE, 0xFE, 0x7E, 0x3E, 0x3E, 0x3E, 0x3E, 0x7E, 0xFE, 0xFE, 0xFE, 0xFE,
	0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE,
	0xFE, 0xFC, 0xF8, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xF0, 0xF0, 0x3F, 0x0F, 0x00, 0x00,
	0xE0, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x7F, 0x0F,
	0x01, 0xC0, 0xF8, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x20, 0x00, 0x00, 0x9F, 0x9F, 0x00, 0x00,
	0x20, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x9F, 0x87, 0x81,
	0x80, 0x98, 0x9E, 0x9F, 0x9F, 0xFF, 0xFF, 0xFF, 0xE0, 0xC0, 0x80, 0x9F, 0x9F, 0x80, 0xC0, 0xE0,
	0xFF, 0xFF, 0xF0, 0xF0, 0xF0, 0xF1, 0xF1, 0x80, 0x80, 0x80, 0xF1, 0xFF, 0xFF, 0xE0, 0xC0, 0x80,
	0x9F, 0x9F, 0x80, 0xC0, 0xE0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F,
	0xEF, 0xF7, 0xF7, 0xF7, 0xEF, 0xDF, 0xFF, 0xFF, 0x8F, 0x77, 0x77, 0x77, 0xF7, 0xF7, 0xCF, 0xFF,
	0xFF, 0xDF, 0xEF, 0x07, 0xFF, 0xFF, 0xFF, 0xEF, 0xF7, 0xF7, 0xF7, 0xF7, 0x0F, 0xFF, 0x0F, 0xF7,
	0xF7, 0xF7, 0xF7, 0x0F, 0xFF, 0xFF, 0x07, 0x77, 0x77, 0x77, 0x77, 0x77, 0x8F, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
	0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xF8, 0xF7, 0xEF, 0xEF, 0xEF, 0xF7, 0xFB, 0xFF, 0xFF, 0xF3, 0xEF, 0xEF, 0xEF,
	0xEE, 0xEE, 0xF1, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0xFF, 0xFF, 0xFF, 0xEF, 0xE7, 0xEB, 0xED, 0xEE,
	0xEF, 0xFF, 0xF0, 0xEF, 0xEF, 0xEF, 0xEF, 0xF0, 0xFF, 0xFF, 0xE0, 0xEF, 0xEF, 0xEF, 0xEF, 0xEF,
	0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x0F, 0x1F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F,
	0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F,
	0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F,
	0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F,
	0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F,
	0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x1F, 0x0F, 0x00,
};
const unsigned char test2 [] = {
	0x00, 0x00, 0x00, 0x00, 0xF8, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC,
	0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC,
	0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC,
	0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC,
	0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xF8,
	0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0x9F, 0x1F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x1F,
	0x9F, 0x1F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0x1F, 0x9F, 0xFF, 0xFF, 0xFF,
	0x1F, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F, 0x1F, 0x3F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0x1F, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0xC0, 0x03, 0x3F, 0xFF, 0xFF, 0xFF,
	0x3F, 0x03, 0xC0, 0xF8, 0xFF, 0xF8, 0xC0, 0x03, 0x3F, 0xFF, 0xFF, 0xFF, 0x1F, 0x03, 0xC0, 0xFC,
	0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xFE, 0xFC, 0xF1, 0xE3,
	0x8F, 0x1F, 0x7F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE,
	0xE0, 0x81, 0x8F, 0x81, 0xE0, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xE0, 0x81, 0x8F, 0x81,
	0xE0, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x80,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0xF8, 0xE3, 0xC7, 0x80, 0x80, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F,
	0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F,
	0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F,
	0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F,
	0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F,
	0x1F, 0x1F, 0x1F, 0x0F, 0x03, 0x00, 0x00, 0x00,
};

char WB [] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0xFC, 0xFC, 0xFC, 0xF0, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xE0, 0xFC, 0xFC, 0xFC, 0xFC, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0,
	0xFC, 0xFC, 0xFC, 0x1C, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFC, 0xFC, 0xFC, 0x3C, 0x3C, 0x3C, 0x3C,
	0x3C, 0x7C, 0xFC, 0xF8, 0xF8, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	0x1F, 0xFF, 0xFF, 0xFE, 0xE0, 0x00, 0xC0, 0xFE, 0xFF, 0x7F, 0x07, 0x07, 0x7F, 0xFF, 0xFE, 0xE0,
	0x00, 0xC0, 0xFE, 0xFF, 0xFF, 0x1F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
	0x78, 0x78, 0x78, 0x78, 0x78, 0x7C, 0xFF, 0xFF, 0xFF, 0xE3, 0xE0, 0x80, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x3F, 0xFF, 0xFF, 0xFC, 0xFF, 0xFF, 0x0F, 0x00, 0x00, 0x00,
	0x00, 0x0F, 0xFF, 0xFF, 0xFC, 0xFF, 0xFF, 0x3F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xF0, 0xF8, 0xFF, 0xFF, 0x7F, 0x1F,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
void drawZero(unsigned char startPixelX, unsigned char startPixelY)
{
	SetPixel(startPixelX, startPixelY, Black);
	SetPixel(startPixelX+1, startPixelY-1, Black);
	SetPixel(startPixelX+2, startPixelY, Black);
	SetPixel(startPixelX+2, startPixelY+1, Black);
	SetPixel(startPixelX+2, startPixelY+2, Black);
	SetPixel(startPixelX+1, startPixelY+3, Black);
	SetPixel(startPixelX, startPixelY+2, Black);
	SetPixel(startPixelX, startPixelY+1, Black);
}
void drawOne(unsigned char startPixelX, unsigned char startPixelY)
{
	SetPixel(startPixelX, startPixelY, Black);
	SetPixel(startPixelX, startPixelY+1, Black);
	SetPixel(startPixelX, startPixelY+2, Black);
	SetPixel(startPixelX, startPixelY+3, Black);
	SetPixel(startPixelX, startPixelY+4, Black);
}
void drawTwo(unsigned char startPixelX, unsigned char startPixelY)
{
	SetPixel(startPixelX, startPixelY, Black);
	SetPixel(startPixelX+1, startPixelY, Black);
	SetPixel(startPixelX+2, startPixelY+1, Black);
	SetPixel(startPixelX+1, startPixelY+2, Black);
	SetPixel(startPixelX, startPixelY+3, Black);
	SetPixel(startPixelX, startPixelY+4, Black);
	SetPixel(startPixelX+1, startPixelY+4, Black);
	SetPixel(startPixelX+2, startPixelY+4, Black);

	
}
void drawThree(unsigned char startPixelX, unsigned char startPixelY)
{
	SetPixel(startPixelX, startPixelY, Black);
	SetPixel(startPixelX+1, startPixelY, Black);
	SetPixel(startPixelX+2, startPixelY+1, Black);
	SetPixel(startPixelX+1, startPixelY+2, Black);
	SetPixel(startPixelX+2, startPixelY+3, Black);
	SetPixel(startPixelX+1, startPixelY+4, Black);
	SetPixel(startPixelX, startPixelY+4, Black);
}
void drawFour(unsigned char startPixelX, unsigned char startPixelY)
{
	SetPixel(startPixelX, startPixelY, Black);
	SetPixel(startPixelX, startPixelY+1, Black);
	SetPixel(startPixelX, startPixelY+2, Black);
	SetPixel(startPixelX+1, startPixelY+2, Black);
	SetPixel(startPixelX+2, startPixelY+2, Black);
	SetPixel(startPixelX+2, startPixelY+1, Black);
	SetPixel(startPixelX+2, startPixelY, Black);
	SetPixel(startPixelX+2, startPixelY+3, Black);
	SetPixel(startPixelX+2, startPixelY+4, Black);
	
}
void drawFive(unsigned char startPixelX, unsigned char startPixelY)
{
	SetPixel(startPixelX, startPixelY, Black);
	SetPixel(startPixelX+1, startPixelY, Black);
	SetPixel(startPixelX+2, startPixelY, Black);
	SetPixel(startPixelX, startPixelY+1, Black);
	SetPixel(startPixelX, startPixelY+2, Black);
	SetPixel(startPixelX+1, startPixelY+2, Black);
	SetPixel(startPixelX+2, startPixelY+3, Black);
	SetPixel(startPixelX+1, startPixelY+4, Black);
	SetPixel(startPixelX, startPixelY+4, Black);
}
void drawSix(unsigned char startPixelX, unsigned char startPixelY)
{
	SetPixel(startPixelX, startPixelY, Black);
	SetPixel(startPixelX+1, startPixelY-1, Black);
	SetPixel(startPixelX+2, startPixelY-1, Black);
	SetPixel(startPixelX, startPixelY+1, Black);
	SetPixel(startPixelX, startPixelY+2, Black);
	SetPixel(startPixelX, startPixelY+3, Black);
	SetPixel(startPixelX+1, startPixelY+1, Black);
	SetPixel(startPixelX+2, startPixelY+1, Black);
	SetPixel(startPixelX+2, startPixelY+2, Black);
	SetPixel(startPixelX+2, startPixelY+3, Black);
	SetPixel(startPixelX+1, startPixelY+3, Black);
}
void drawEight(unsigned char startPixelX, unsigned char startPixelY)
{
	SetPixel(startPixelX, startPixelY, Black);
	SetPixel(startPixelX+1, startPixelY, Black);
	SetPixel(startPixelX+2, startPixelY, Black);
	SetPixel(startPixelX+2, startPixelY+1, Black);
	SetPixel(startPixelX+2, startPixelY+2, Black);
	SetPixel(startPixelX+1, startPixelY+2, Black);
	SetPixel(startPixelX, startPixelY+2, Black);
	SetPixel(startPixelX, startPixelY+1, Black);
	SetPixel(startPixelX, startPixelY+3, Black);
	SetPixel(startPixelX, startPixelY+4, Black);
	SetPixel(startPixelX+1, startPixelY+4, Black);
	SetPixel(startPixelX+2, startPixelY+4, Black);
	SetPixel(startPixelX+2, startPixelY+3, Black);
	
}

void setTwo (unsigned char box)
{
	if (box == 1)
	{
		drawTwo(8,5);
	}
	else if (box == 2)
	{
		drawTwo(29,5);
	}
	else if (box == 3)
	{
		drawTwo(50,5);
	}
	else if (box == 4)
	{
		drawTwo(71,5);
	}
	
	
	
	else if (box == 5)
	{
		drawTwo(8,17);
	}
	else if (box == 6)
	{
		drawTwo(29,17);
	}
	else if (box == 7)
	{
		drawTwo(50,17);
	}
	else if (box == 8)
	{
		drawTwo(71,17);
	}
	
	
	else if (box == 9)
	{
		drawTwo(8,29);
	}
	else if (box == 10)
	{
		drawTwo(29,29);
	}
	else if (box == 11)
	{
		drawTwo(50,29);
	}
	else if (box == 12)
	{
		drawTwo(71,29);
	}
	
	else if (box == 13)
	{
		drawTwo(8,41);
	}
	else if (box == 14)
	{
		drawTwo(29,41);
	}
	else if (box == 15)
	{
		drawTwo(50,41);
	}
	else if (box == 16)
	{
		drawTwo(71,41);
	}
}
void setFour (unsigned char box)
{
	if (box == 1)
	{
		drawFour(8,5);
	}
	else if (box == 2)
	{
		drawFour(29,5);
	}
	else if (box == 3)
	{
		drawFour(50,5);
	}
	else if (box == 4)
	{
		drawFour(71,5);
	}
	
	
	
	else if (box == 5)
	{
		drawFour(8,17);
	}
	else if (box == 6)
	{
		drawFour(29,17);
	}
	else if (box == 7)
	{
		drawFour(50,17);
	}
	else if (box == 8)
	{
		drawFour(71,17);
	}
	
	
	else if (box == 9)
	{
		drawFour(8,29);
	}
	else if (box == 10)
	{
		drawFour(29,29);
	}
	else if (box == 11)
	{
		drawFour(50,29);
	}
	else if (box == 12)
	{
		drawFour(71,29);
	}
	
	else if (box == 13)
	{
		drawFour(8,41);
	}
	else if (box == 14)
	{
		drawFour(29,41);
	}
	else if (box == 15)
	{
		drawFour(50,41);
	}
	else if (box == 16)
	{
		drawFour(71,41);
	}
}
void setEight (unsigned char box)
{
	if (box == 1)
	{
		drawEight(8,5);
	}
	else if (box == 2)
	{
		drawEight(29,5);
	}
	else if (box == 3)
	{
		drawEight(50,5);
	}
	else if (box == 4)
	{
		drawEight(71,5);
	}
	
	
	
	else if (box == 5)
	{
		drawEight(8,17);
	}
	else if (box == 6)
	{
		drawEight(29,17);
	}
	else if (box == 7)
	{
		drawEight(50,17);
	}
	else if (box == 8)
	{
		drawEight(71,17);
	}
	
	
	else if (box == 9)
	{
		drawEight(8,29);
	}
	else if (box == 10)
	{
		drawEight(29,29);
	}
	else if (box == 11)
	{
		drawEight(50,29);
	}
	else if (box == 12)
	{
		drawEight(71,29);
	}
	
	else if (box == 13)
	{
		drawEight(8,41);
	}
	else if (box == 14)
	{
		drawEight(29,41);
	}
	else if (box == 15)
	{
		drawEight(50,41);
	}
	else if (box == 16)
	{
		drawEight(71,41);
	}
}
void setSixteen (unsigned char box)
{
	if (box == 1)
	{
		drawOne(7,5);
		drawSix(9,6);
	}
	else if (box == 2)
	{
		drawOne(28,5);
		drawSix(30,6);
	}
	else if (box == 3)
	{
		drawOne(49,5);
		drawSix(51,6);
	}
	else if (box == 4)
	{
		drawOne(70,5);
		drawSix(72,6);
	}
	
	
	
	else if (box == 5)
	{
		drawOne(7,17);
		drawSix(9,18);
	}
	else if (box == 6)
	{
		drawOne(28,17);
		drawSix(30,18);
	}
	else if (box == 7)
	{
		drawOne(49,17);
		drawSix(51,18);
	}
	else if (box == 8)
	{
		drawOne(70,17);
		drawSix(72,18);
	}
	
	
	else if (box == 9)
	{
		drawOne(7,29);
		drawSix(9,30);
	}
	else if (box == 10)
	{
		drawOne(28,29);
		drawSix(30,30);
	}
	else if (box == 11)
	{
		drawOne(49,29);
		drawSix(51,30);
	}
	else if (box == 12)
	{
		drawOne(70,29);
		drawSix(72,30);
	}
	
	else if (box == 13)
	{
		drawOne(7,41);
		drawSix(9,42);
	}
	else if (box == 14)
	{
		drawOne(28,41);
		drawSix(30,42);
	}
	else if (box == 15)
	{
		drawOne(49,41);
		drawSix(51,42);
	}
	else if (box == 16)
	{
		drawOne(70,41);
		drawSix(72,42);
	}
}
void setThirtyTwo (unsigned char box)
{
	if (box == 1)
	{
		drawThree(6,5);
		drawTwo(10,5);
	}
	else if (box == 2)
	{
		drawThree(27,5);
		drawTwo(31,5);
	}
	else if (box == 3)
	{
		drawThree(48,5);
		drawTwo(52,5);
	}
	else if (box == 4)
	{
		drawThree(69,5);
		drawTwo(73,5);
	}
	
	
	
	else if (box == 5)
	{
		drawThree(6,17);
		drawTwo(10,17);
	}
	else if (box == 6)
	{
		drawThree(27,17);
		drawTwo(31,17);
	}
	else if (box == 7)
	{
		drawThree(48,17);
		drawTwo(52,17);
	}
	else if (box == 8)
	{
		drawThree(69,17);
		drawTwo(73,17);
	}
	
	
	else if (box == 9)
	{
		drawThree(6,29);
		drawTwo(10,29);
	}
	else if (box == 10)
	{
		drawThree(27,29);
		drawTwo(31,29);
	}
	else if (box == 11)
	{
		drawThree(48,29);
		drawTwo(52,29);
	}
	else if (box == 12)
	{
		drawThree(69,29);
		drawTwo(73,29);
	}
	
	else if (box == 13)
	{
		drawThree(6,41);
		drawTwo(10,41);
	}
	else if (box == 14)
	{
		drawThree(27,41);
		drawTwo(31,41);
	}
	else if (box == 15)
	{
		drawThree(48,41);
		drawTwo(52,41);
	}
	else if (box == 16)
	{
		drawThree(69,41);
		drawTwo(73,41);
	}
}
void setSixtyFour (unsigned char box)
{
	if (box == 1)
	{
		drawSix(6,6);
		drawFour(10,5);
	}
	else if (box == 2)
	{
		drawSix(27,6);
		drawFour(31,5);
	}
	else if (box == 3)
	{
		drawSix(48,6);
		drawFour(52,5);
	}
	else if (box == 4)
	{
		drawSix(69,6);
		drawFour(73,5);
	}
	
	
	
	else if (box == 5)
	{
		drawSix(6,18);
		drawFour(10,17);
	}
	else if (box == 6)
	{
		drawSix(27,18);
		drawFour(31,17);
	}
	else if (box == 7)
	{
		drawSix(48,18);
		drawFour(52,17);
	}
	else if (box == 8)
	{
		drawSix(69,18);
		drawFour(73,17);
	}
	
	
	else if (box == 9)
	{
		drawSix(6,30);
		drawFour(10,29);
	}
	else if (box == 10)
	{
		drawSix(27,30);
		drawFour(31,29);
	}
	else if (box == 11)
	{
		drawSix(48,30);
		drawFour(52,29);
	}
	else if (box == 12)
	{
		drawSix(69,30);
		drawFour(73,29);
	}
	
	else if (box == 13)
	{
		drawSix(6,42);
		drawFour(10,41);
	}
	else if (box == 14)
	{
		drawSix(27,42);
		drawFour(31,41);
	}
	else if (box == 15)
	{
		drawSix(48,42);
		drawFour(52,41);
	}
	else if (box == 16)
	{
		drawSix(69,42);
		drawFour(73,41);
	}
}
void set128 (unsigned char box)
{
	if (box == 1)
	{
		drawOne(4,5);
		drawTwo(7,5);
		drawEight(11,5);
	}
	else if (box == 2)
	{
		drawOne(24,5);
		drawTwo(27,5);
		drawEight(31,5);
	}
	else if (box == 3)
	{
		drawOne(46,5);
		drawTwo(49,5);
		drawEight(54,5);
	}
	else if (box == 4)
	{
		drawOne(67,5);
		drawTwo(70,5);
		drawEight(75,5);
	}
	
	
	
	else if (box == 5)
	{
		drawOne(4,17);
		drawTwo(7,17);
		drawEight(11,17);
	}
	else if (box == 6)
	{
		drawOne(24,17);
		drawTwo(27,17);
		drawEight(31,17);
	}
	else if (box == 7)
	{
		drawOne(46,17);
		drawTwo(49,17);
		drawEight(54,17);
	}
	else if (box == 8)
	{
		drawOne(67,17);
		drawTwo(70,17);
		drawEight(75,17);
	}
	
	
	else if (box == 9)
	{
		drawOne(4,29);
		drawTwo(7,29);
		drawEight(11,29);
	}
	else if (box == 10)
	{
		drawOne(24,29);
		drawTwo(27,29);
		drawEight(31,29);
	}
	else if (box == 11)
	{
		drawOne(46,29);
		drawTwo(49,29);
		drawEight(54,29);
	}
	else if (box == 12)
	{
		drawOne(67,29);
		drawTwo(70,29);
		drawEight(75,29);
	}
	
	else if (box == 13)
	{
		drawOne(4,41);
		drawTwo(7,41);
		drawEight(11,41);
	}
	else if (box == 14)
	{
		drawOne(24,41);
		drawTwo(27,41);
		drawEight(31,41);
	}
	else if (box == 15)
	{
		drawOne(46,41);
		drawTwo(49,41);
		drawEight(54,41);
	}
	else if (box == 16)
	{
		drawOne(67,41);
		drawTwo(70,41);
		drawEight(75,41);
	}
}
void set256 (unsigned char box)
{
	if (box == 1)
	{
		drawTwo(4,5);
		drawFive(8,5);
		drawSix(11,6);
	}
	else if (box == 2)
	{
		drawTwo(24,5);
		drawFive(28,5);
		drawSix(31,6);
	}
	else if (box == 3)
	{
		drawTwo(46,5);
		drawFive(50,5);
		drawSix(54,6);
	}
	else if (box == 4)
	{
		drawTwo(67,5);
		drawFive(71,5);
		drawSix(75,6);
	}
	
	
	
	else if (box == 5)
	{
		drawTwo(4,17);
		drawFive(8,17);
		drawSix(11,18);
	}
	else if (box == 6)
	{
		drawTwo(24,17);
		drawFive(28,17);
		drawSix(31,18);
	}
	else if (box == 7)
	{
		drawTwo(46,17);
		drawFive(50,17);
		drawSix(54,18);
	}
	else if (box == 8)
	{
		drawTwo(67,17);
		drawFive(71,17);
		drawSix(75,18);
	}
	
	
	else if (box == 9)
	{
		drawTwo(4,29);
		drawFive(8,29);
		drawSix(11,30);
	}
	else if (box == 10)
	{
		drawTwo(24,29);
		drawFive(28,29);
		drawSix(31,30);
	}
	else if (box == 11)
	{
		drawTwo(46,29);
		drawFive(50,29);
		drawSix(54,30);
	}
	else if (box == 12)
	{
		drawTwo(67,29);
		drawFive(71,29);
		drawSix(75,30);
	}
	
	else if (box == 13)
	{
		drawTwo(4,41);
		drawFive(8,41);
		drawSix(11,42);
	}
	else if (box == 14)
	{
		drawTwo(24,41);
		drawFive(28,41);
		drawSix(31,42);
	}
	else if (box == 15)
	{
		drawTwo(46,41);
		drawFive(50,41);
		drawSix(54,42);
	}
	else if (box == 16)
	{
		drawTwo(67,41);
		drawFive(71,41);
		drawSix(75,42);
	}
}
void set512 (unsigned char box)
{
	if (box == 1)
	{
		drawFive(4,5);
		drawOne(8,5);
		drawTwo(11,5);
	}
	else if (box == 2)
	{
		drawFive(24,5);
		drawOne(28,5);
		drawTwo(31,5);
	}
	else if (box == 3)
	{
		drawFive(46,5);
		drawOne(50,5);
		drawTwo(54,5);
	}
	else if (box == 4)
	{
		drawFive(67,5);
		drawOne(71,5);
		drawTwo(75,5);
	}
	
	
	
	else if (box == 5)
	{
		drawFive(4,17);
		drawOne(8,17);
		drawTwo(11,17);
	}
	else if (box == 6)
	{
		drawFive(24,17);
		drawOne(28,17);
		drawTwo(31,17);
	}
	else if (box == 7)
	{
		drawFive(46,17);
		drawOne(50,17);
		drawTwo(54,17);
	}
	else if (box == 8)
	{
		drawFive(67,17);
		drawOne(71,17);
		drawTwo(75,17);
	}
	
	
	else if (box == 9)
	{
		drawFive(4,29);
		drawOne(8,29);
		drawTwo(11,29);
	}
	else if (box == 10)
	{
		drawFive(24,29);
		drawOne(28,29);
		drawTwo(31,29);
	}
	else if (box == 11)
	{
		drawFive(46,29);
		drawOne(50,29);
		drawTwo(54,29);
	}
	else if (box == 12)
	{
		drawFive(67,29);
		drawOne(71,29);
		drawTwo(75,29);
	}
	
	else if (box == 13)
	{
		drawFive(4,41);
		drawOne(8,41);
		drawTwo(11,41);
	}
	else if (box == 14)
	{
		drawFive(24,41);
		drawOne(28,41);
		drawTwo(31,41);
	}
	else if (box == 15)
	{
		drawFive(46,41);
		drawOne(50,41);
		drawTwo(54,41);
	}
	else if (box == 16)
	{
		drawFive(67,41);
		drawOne(71,41);
		drawTwo(75,41);
	}
}
void set1024 (unsigned char box)
{
	if (box == 1)
	{
		drawOne(2,5);
		drawZero(4,6);
		drawTwo(8,5);
		drawFour(12,5);
	}
	else if (box == 2)
	{
		drawOne(22,5);
		drawZero(24,6);
		drawTwo(28,5);
		drawFour(32,5);
	}
	else if (box == 3)
	{
		drawOne(44,5);
		drawZero(46,6);
		drawTwo(50,5);
		drawFour(54,5);
	}
	else if (box == 4)
	{
		drawOne(65,5);
		drawZero(67,6);
		drawTwo(71,5);
		drawFour(75,5);
	}
	
	
	
	else if (box == 5)
	{
		drawOne(2,17);
		drawZero(4,18);
		drawTwo(8,17);
		drawFour(12,17);
	}
	else if (box == 6)
	{
		drawOne(22,17);
		drawZero(24,18);
		drawTwo(28,17);
		drawFour(32,17);
	}
	else if (box == 7)
	{
		drawOne(44,17);
		drawZero(46,18);
		drawTwo(50,17);
		drawFour(54,17);
	}
	else if (box == 8)
	{
		drawOne(65,17);
		drawZero(67,18);
		drawTwo(71,17);
		drawFour(75,17);
	}
	
	
	else if (box == 9)
	{
		drawOne(2,29);
		drawZero(4,30);
		drawTwo(8,29);
		drawFour(12,29);
	}
	else if (box == 10)
	{
		drawOne(22,29);
		drawZero(24,29);
		drawTwo(28,30);
		drawFour(32,29);
	}
	else if (box == 11)
	{
		drawOne(44,29);
		drawZero(46,30);
		drawTwo(50,29);
		drawFour(54,29);
	}
	else if (box == 12)
	{
		drawOne(65,29);
		drawZero(67,30);
		drawTwo(71,29);
		drawFour(75,29);
	}
	
	else if (box == 13)
	{
		drawOne(2,41);
		drawZero(4,42);
		drawTwo(8,41);
		drawFour(12,41);
	}
	else if (box == 14)
	{
		drawOne(22,41);
		drawZero(24,42);
		drawTwo(28,41);
		drawFour(32,41);
	}
	else if (box == 15)
	{
		drawOne(44,41);
		drawZero(46,42);
		drawTwo(50,41);
		drawFour(54,41);
	}
	else if (box == 16)
	{
		drawOne(65,41);
		drawZero(67,42);
		drawTwo(71,41);
		drawFour(75,41);
	}
}
void set2048 (unsigned char box)
{
	if (box == 1)
	{
		drawTwo(2,5);
		drawZero(5,6);
		drawFour(9,5);
		drawEight(13,5);
	}
	else if (box == 2)
	{
		drawTwo(22,5);
		drawZero(25,6);
		drawFour(29,5);
		drawEight(33,5);
	}
	else if (box == 3)
	{
		drawTwo(44,5);
		drawZero(47,6);
		drawFour(51,5);
		drawEight(55,5);
	}
	else if (box == 4)
	{
		drawTwo(65,5);
		drawZero(68,6);
		drawFour(72,5);
		drawEight(76,5);
	}
	
	
	
	else if (box == 5)
	{
		drawTwo(2,17);
		drawZero(5,18);
		drawFour(9,17);
		drawEight(13,17);
	}
	else if (box == 6)
	{
		drawTwo(22,17);
		drawZero(25,18);
		drawFour(29,17);
		drawEight(33,17);
	}
	else if (box == 7)
	{
		drawTwo(44,17);
		drawZero(47,18);
		drawFour(51,17);
		drawEight(55,17);
	}
	else if (box == 8)
	{
		drawTwo(65,17);
		drawZero(68,18);
		drawFour(72,17);
		drawEight(76,17);
	}
	
	
	else if (box == 9)
	{
		drawTwo(2,29);
		drawZero(5,30);
		drawFour(9,29);
		drawEight(13,29);
	}
	else if (box == 10)
	{
		drawTwo(22,29);
		drawZero(25,29);
		drawFour(29,30);
		drawEight(33,29);
	}
	else if (box == 11)
	{
		drawTwo(44,29);
		drawZero(47,30);
		drawFour(51,29);
		drawEight(55,29);
	}
	else if (box == 12)
	{
		drawTwo(65,29);
		drawZero(68,30);
		drawFour(72,29);
		drawEight(76,29);
	}
	
	else if (box == 13)
	{
		drawTwo(2,41);
		drawZero(5,42);
		drawFour(9,41);
		drawEight(13,41);
	}
	else if (box == 14)
	{
		drawTwo(22,41);
		drawZero(25,42);
		drawFour(29,41);
		drawEight(33,41);
	}
	else if (box == 15)
	{
		drawTwo(44,41);
		drawZero(47,42);
		drawFour(51,41);
		drawEight(55,41);
	}
	else if (box == 16)
	{
		drawTwo(65,41);
		drawZero(68,42);
		drawFour(72,41);
		drawEight(76,41);
	}
}

int validmove = 0;

void updatePrevGrid()
{
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			grid_prev[i][j] = grid[i][j];
		}
	}
}

void copyGrid()
{
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			grid[i][j] = grid_prev[i][j];
		}
	}
}

void initializeGrid()
{
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			grid[i][j] = 0;
		}
	}
}

void shiftGridRight()
{
	unsigned int trigger = 0;
	do
	{
		trigger = 0;
		for (unsigned int i = 0; i < 4; i++)
		{
			for (unsigned int j = 0; j < 3; j++)
			{
				if ((grid[i][j] != 0) && (grid[i][j+1] == 0))
				{
					grid[i][j+1] = grid[i][j];
					grid[i][j] = 0;
					trigger = 1;
				}
			}
		}
	}
	while (trigger != 0);
}

void combineGridRight()
{
	for (unsigned int i = 0; i < 4; i++)
	{
		for (unsigned int j = 0; j < 3; j++)
		{
			if ((grid[i][j] == grid[i][j+1]) && (grid[i][j] != 0))
			{
				grid[i][j+1] = grid[i][j+1] * 2;
				grid[i][j] = 0;
			}
		}
	}
}

void shiftGridLeft()
{
	unsigned int trigger = 0;
	do
	{
		trigger = 0;
		for (unsigned int i = 0; i < 4; i++)
		{
			for (unsigned int j = 3; j > 0; j--)
			{
				if ((grid[i][j] != 0) && (grid[i][j-1] == 0))
				{
					grid[i][j-1] = grid[i][j];
					grid[i][j] = 0;
					trigger = 1;
				}
			}
		}
	}
	while (trigger != 0);
}

void combineGridLeft()
{
	for (unsigned int i = 0; i < 4; i++)
	{
		for (unsigned int j = 0; j < 3; j++)
		{
			if ((grid[i][j] == grid[i][j+1]) && (grid[i][j] != 0))
			{
				grid[i][j] = grid[i][j] * 2;
				grid[i][j+1] = 0;
			}
		}
	}
}

void shiftGridDown()
{
	unsigned int trigger = 0;
	do
	{
		trigger = 0;
		for (unsigned int i = 0; i < 4; i++)
		{
			for (unsigned int j = 0; j < 3; j++)
			{
				if ((grid[j][i] != 0) && (grid[j+1][i] == 0))
				{
					grid[j+1][i] = grid[j][i];
					grid[j][i] = 0;
					trigger = 1;
				}
			}
		}
	}
	while (trigger != 0);
}

void combineGridDown()
{
	for (unsigned int i = 0; i < 4; i++)
	{
		for (unsigned int j = 3; j > 0; j--)
		{
			if ((grid[j][i] == grid[j-1][i]) && (grid[j][i] != 0))
			{
				grid[j-1][i] = grid[j-1][i] * 2;
				grid[j][i] = 0;
			}
		}
	}
}

void shiftGridUp()
{
	unsigned int trigger = 0;
	do
	{
		trigger = 0;
		for (unsigned int i = 0; i < 4; i++)
		{
			for (unsigned int j = 3; j > 0; j--)
			{
				if ((grid[j][i] != 0) && (grid[j-1][i] == 0))
				{
					grid[j-1][i] = grid[j][i];
					grid[j][i] = 0;
					trigger = 1;
				}
			}
		}
	}
	while (trigger != 0);
}

void combineGridUp()
{
	for (unsigned int i = 0; i < 4; i++)
	{
		for (unsigned int j = 0; j < 3; j++)
		{
			if ((grid[j][i] == grid[j+1][i]) && (grid[j][i] != 0))
			{
				grid[j][i] = grid[j][i] * 2;
				grid[j+1][i] = 0;
			}
		}
	}
}

void pressLeft()
{
	shiftGridLeft();
	combineGridLeft();
	shiftGridLeft();
}

void pressUp()
{
	shiftGridUp();
	combineGridUp();
	shiftGridUp();
}

void pressDown()
{
	shiftGridDown();
	combineGridDown();
	shiftGridDown();
}

void pressRight()
{
	shiftGridRight();
	combineGridRight();
	shiftGridRight();
}

void generateRandom()
{
	int size = 0;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			if (grid[i][j] == 0)
			{
				size++;
			}
		}
	}
	int arr[size];
	size = 0;
	int location = 0;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			if (grid[i][j] == 0)
			{
				arr[size] = location;
				size++;
			}
			location++;
		}
	}
	int read = rand() % size;
	size = arr[read];
	int i = 0;
	int j = 0;
	while (size > 4)
	{
		size = size - 4;
		i++;
	}
	j = size;
	grid[i][j] = 2;
}

void clearBoxLCD(int box)
{
	if (box == 1)
	{
		for (int i = 1; i < 21; i++)
		{
			for (int j = 1; j <12; i++)
			{
				SetPixel(i,j,White);
			}
		}
		return;
	}
	else if (box == 2)
	{
		for (int i = 22; i < 42; i++)
		{
			for (int j = 1; j <12; i++)
			{
				SetPixel(i,j,White);
			}
		}
		return;
	}
	else if (box == 3)
	{
		for (int i = 43; i < 63; i++)
		{
			for (int j = 1; j <12; i++)
			{
				SetPixel(i,j,White);
			}
		}
		return;
	}
	else if (box == 4)
	{
		for (int i = 64; i < 83; i++)
		{
			for (int j = 1; j <12; i++)
			{
				SetPixel(i,j,White);
			}
		}
		return;
	}
	else if (box == 5)
	{
		for (int i = 1; i < 21; i++)
		{
			for (int j = 13; j < 24; i++)
			{
				SetPixel(i,j,White);
			}
		}
		return;
	}
	else if (box == 6)
	{
		for (int i = 22; i < 42; i++)
		{
			for (int j = 13; j < 24; i++)
			{
				SetPixel(i,j,White);
			}
		}
		return;
	}
	else if (box == 7)
	{
		for (int i = 43; i < 63; i++)
		{
			for (int j = 13; j < 24; i++)
			{
				SetPixel(i,j,White);
			}
		}
		return;
	}
	else if (box == 8)
	{
		for (int i = 64; i < 83; i++)
		{
			for (int j = 13; j < 24; i++)
			{
				SetPixel(i,j,White);
			}
		}
		return;
	}
	else if (box == 9)
	{
		for (int i = 1; i < 21; i++)
		{
			for (int j = 25; j < 36; i++)
			{
				SetPixel(i,j,White);
			}
		}
		return;
	}
	else if (box == 10)
	{
		for (int i = 22; i < 42; i++)
		{
			for (int j = 25; j < 36; i++)
			{
				SetPixel(i,j,White);
			}
		}
		return;
	}
	else if (box == 11)
	{
		for (int i = 43; i < 63; i++)
		{
			for (int j = 25; j < 36; i++)
			{
				SetPixel(i,j,White);
			}
		}
		return;
	}
	else if (box == 12)
	{
		for (int i = 63; i < 83; i++)
		{
			for (int j = 25; j < 36; i++)
			{
				SetPixel(i,j,White);
			}
		}
		return;
	}
	else if (box == 13)
	{
		for (int i = 1; i < 21; i++)
		{
			for (int j = 37; j < 47; i++)
			{
				SetPixel(i,j,White);
			}
		}
		return;
	}
	else if (box == 14)
	{
		for (int i = 22; i < 42; i++)
		{
			for (int j = 37; j < 47; i++)
			{
				SetPixel(i,j,White);
			}
		}
		return;
	}
	else if (box == 15)
	{
		for (int i = 43; i < 63; i++)
		{
			for (int j = 37; j < 47; i++)
			{
				SetPixel(i,j,White);
			}
		}
		return;
	}
	else if (box == 16)
	{
		for (int i = 64; i < 83; i++)
		{
			for (int j = 37; j < 47; i++)
			{
				SetPixel(i,j,White);
			}
		}
		return;
	}
}
void Tick_LCD()
{
	int cnt = 0;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			cnt++;
			if (grid[i][j] == 0)
			{
				//clearBoxLCD(cnt);
			}
			if (grid[i][j] == 2)
			{
				setTwo(cnt);
			}
			if (grid[i][j] == 4)
			{
				setFour(cnt);
			}
			if (grid[i][j] == 8)
			{
				setEight(cnt);
			}
			if (grid[i][j] == 16)
			{
				setSixteen(cnt);
			}
			if (grid[i][j] == 32)
			{
				setThirtyTwo(cnt);
			}
			if (grid[i][j] == 64)
			{
				setSixtyFour(cnt);
			}
			if (grid[i][j] == 128)
			{
				set128(cnt);
			}
			if (grid[i][j] == 256)
			{
				set256(cnt);
			}
			if (grid[i][j] == 512)
			{
				set512(cnt);
			}
			if (grid[i][j] == 1024)
			{
				set1024(cnt);
			}
			if (grid[i][j] == 2048)
			{
				set2048(cnt);
			}
		}
	}
}

typedef struct task {
	int state; // Current state of the task
	unsigned long period; // Rate at which the task should tick
	unsigned long elapsedTime; // Time since task's previous tick
	int (*TickFct)(int); // Function to call for task's tick
} task;

task tasks[1];
const unsigned char tasksNum = 1;
const unsigned long tasksPeriodGCD = 1;
const unsigned long run_period = 1;

enum Run_States{Run_start};
int Run (int state)
{
	unsigned char buttonUp, buttonDown, buttonLeft, buttonRight, buttonReset;
	LCDInitialize();
	buttonUp = ~PINA & 0x01;
	buttonDown = ~PINA & 0x02;
	buttonLeft = ~PINA & 0x04;
	buttonRight = ~PINA & 0x08;
	buttonReset = ~PINA & 0x10;
	transmit_data(255);
	while (!buttonUp && !buttonDown && !buttonRight && !buttonLeft && !buttonReset)
	{
		ReplaceBitmap(test);
		LCD_Update();
		buttonUp = ~PINA & 0x01;
		buttonDown = ~PINA & 0x02;
		buttonLeft = ~PINA & 0x04;
		buttonRight = ~PINA & 0x08;
		buttonReset = ~PINA & 0x10;
	}
	_delay_ms(1000);
	LCD_clear();
	setup();
	update();
	DrawRectangle(0,0,83,47,Black,0);
	DrawLine(21,0,21,47,Black);
	DrawLine(42,0,42,47,Black);
	DrawLine(63,0,63,47,Black);
	DrawLine(0,12,83,12,Black);
	DrawLine(0,24,83,24,Black);
	DrawLine(0,36,83,36,Black);
	LCD_Update();
	initializeGrid();
	generateRandom();
	generateRandom();
	updatePrevGrid();
	Tick_LCD();
	LCD_Update();
	while(1)
	{
		_delay_ms(50);
		loop();
		update();
		_delay_ms(50);
		buttonUp = ~PINA & 0x01;
		buttonDown = ~PINA & 0x02;
		buttonLeft = ~PINA & 0x04;
		buttonRight = ~PINA & 0x08;
		buttonReset = ~PINA & 0x10;
		_delay_ms(50);
		int size = 0;
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				if (grid[i][j] == 0)
				{
					size++;
				}
			}
		}
		if (size == 0)
		{
			while(!buttonReset)
			{
				transmit_data(255);
				ReplaceBitmap(lose);
				LCD_Update();
				for (uint8_t i=0; i<TLC5940_LED_N; i++) {
					setGS(i, 0);
				}
				update();
				buttonReset = ~PINA & 0x10;
				
			}
			
		}
		
		if (buttonUp && !buttonDown && !buttonRight && !buttonLeft && !buttonReset)
		{
			pressUp();
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					if (grid[i][j] != grid_prev[i][j])
					{
						validmove = 1;
					}
				}
			}
			if (validmove == 1)
			{
				generateRandom();
				validmove = 0;
			}
			updatePrevGrid();
			LCD_clear();
			DrawRectangle(0,0,83,47,Black,0);
			DrawLine(21,0,21,47,Black);
			DrawLine(42,0,42,47,Black);
			DrawLine(63,0,63,47,Black);
			
			DrawLine(0,12,83,12,Black);
			DrawLine(0,24,83,24,Black);
			DrawLine(0,36,83,36,Black);
			Tick_LCD();
			LCD_Update();
		}
		else if (!buttonUp && buttonDown && !buttonRight && !buttonLeft && !buttonReset)
		{
			pressDown();
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					if (grid[i][j] != grid_prev[i][j])
					{
						validmove = 1;
					}
				}
			}
			if (validmove == 1)
			{
				generateRandom();
				validmove = 0;
			}
			updatePrevGrid();
			LCD_clear();
			DrawRectangle(0,0,83,47,Black,0);
			DrawLine(21,0,21,47,Black);
			DrawLine(42,0,42,47,Black);
			DrawLine(63,0,63,47,Black);
			
			DrawLine(0,12,83,12,Black);
			DrawLine(0,24,83,24,Black);
			DrawLine(0,36,83,36,Black);
			Tick_LCD();
			LCD_Update();
		}
		else if (!buttonUp && !buttonDown && buttonRight && !buttonLeft && !buttonReset)
		{
			pressRight();
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					if (grid[i][j] != grid_prev[i][j])
					{
						validmove = 1;
					}
				}
			}
			if (validmove == 1)
			{
				generateRandom();
				validmove = 0;
			}
			updatePrevGrid();
			LCD_clear();
			DrawRectangle(0,0,83,47,Black,0);
			DrawLine(21,0,21,47,Black);
			DrawLine(42,0,42,47,Black);
			DrawLine(63,0,63,47,Black);
			
			DrawLine(0,12,83,12,Black);
			DrawLine(0,24,83,24,Black);
			DrawLine(0,36,83,36,Black);
			Tick_LCD();
			LCD_Update();
		}
		else if (!buttonUp && !buttonDown && !buttonRight && buttonLeft && !buttonReset)
		{
			pressLeft();
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					if (grid[i][j] != grid_prev[i][j])
					{
						validmove = 1;
					}
				}
			}
			if (validmove == 1)
			{
				generateRandom();
				validmove = 0;
			}
			updatePrevGrid();
			LCD_clear();
			DrawRectangle(0,0,83,47,Black,0);
			DrawLine(21,0,21,47,Black);
			DrawLine(42,0,42,47,Black);
			DrawLine(63,0,63,47,Black);
			
			DrawLine(0,12,83,12,Black);
			DrawLine(0,24,83,24,Black);
			DrawLine(0,36,83,36,Black);
			Tick_LCD();
			LCD_Update();
		}
		else if (!buttonUp && !buttonDown && !buttonRight && !buttonLeft && buttonReset)
		{
			LCD_clear();
			initializeGrid();
			generateRandom();
			generateRandom();
			updatePrevGrid();
			DrawRectangle(0,0,83,47,Black,0);
			DrawLine(21,0,21,47,Black);
			DrawLine(42,0,42,47,Black);
			DrawLine(63,0,63,47,Black);
			
			DrawLine(0,12,83,12,Black);
			DrawLine(0,24,83,24,Black);
			DrawLine(0,36,83,36,Black);
			Tick_LCD();
			LCD_Update();
			_delay_ms(50);
		}
		_delay_ms(50);
		while (buttonUp || buttonLeft || buttonRight || buttonDown || buttonReset)
		{
			buttonUp = ~PINA & 0x01;
			buttonDown = ~PINA & 0x02;
			buttonLeft = ~PINA & 0x04;
			buttonRight = ~PINA & 0x08;
			buttonReset = ~PINA & 0x10;
		}
		checkWin();
		if (wonGame == 1)
		{
			wonGame = 0;
			while (!buttonReset)
			{
				ReplaceBitmap(test2);
				LCD_Update();
				transmit_data(0);
				for (uint8_t i=0; i<TLC5940_LED_N; i= i+2) {
					setGS(i, 3300);

				}
				for (uint8_t i=1; i<TLC5940_LED_N; i=i+2) {
					setGS(i, 5000);
				}
				update();
				_delay_ms(3000);
				buttonReset = ~PINA & 0x10;
			}
		}
		_delay_ms(50);
	}	
}

int main(void)
{
	DDRA = 0x00; PORTA = 0xFF;
	DDRC = 0xFF; PORTC = 0x00;
	unsigned char i=0;
	tasks[i].state = Run_start;
	tasks[i].period = run_period;
	tasks[i].elapsedTime = tasks[i].period;
	tasks[i].TickFct = &Run;
	while(1) {
		for (i = 0; i < tasksNum; ++i) { // Heart of the scheduler code
			if ( tasks[i].elapsedTime >= tasks[i].period ) { // Ready
				tasks[i].state = tasks[i].TickFct(tasks[i].state);
				tasks[i].elapsedTime = 0;
			}
			tasks[i].elapsedTime += tasksPeriodGCD;
		}
	}
	return 0;
    
}


void SendCommand_LCD (unsigned int CommandOut)		// Can also be used for reading the SPI register which gets filled automatically as bits are shifted out
{	
	LCD_Port |= 1<<DC;
	
	LCD_Port |=1<<CE;							//	set chip enable high (has to be high, than low)
	LCD_Port &=~1<<CE;							// set chip enable low
	LCD_Port &=~1<<DC;							//	data/command low -> command	
	
	/* SPDR = CommandOut;						// if spi is used, uncomment this section out

	 //  Wait until Tx register empty.
	 while ( !(SPSR & 0x80) );
	 */
	 for(uint8_t i=0;i<8;i++)					// if spi is used, comment this section out
	 {
		 		
		if((CommandOut>>(7-i)) & 0x01) 
			{
			LCD_Port |=1<<DIN; //lcd pin high
			} 
			else 
			{
			LCD_Port &=~1<<DIN; //lcd pin low
			}

		LCD_Port |=1<<CLK;			//Pulse the Clock line (Shift bit into register)
		LCD_Port &=~1<<CLK;
		

	 }
			
	LCD_Port |=1<<CE;							// set chip enable high again
		
		
}

void SendData_LCD (unsigned int DataOut)
{
	LCD_Port |=1<<DC;							//	data/command high -> data
	LCD_Port |=1<<CE;							//	set chip enable high (has to be high, than low)
	LCD_Port&=(~(1<<CE));						// set chip enable low
	
	/*SPDR = DataOut;							// if spi is used, uncomment this section out

	//  Wait until Tx register empty.
	while ( !(SPSR & 0x80) );
	*/
	for(uint8_t i=0;i<8;i++)					// if spi is used, comment this section out
	{
	if((DataOut>>(7-i)) & 0x01) 
		{
		LCD_Port |=1<<DIN; //lcd pin high
		} 
		else 
		{
		LCD_Port&=(~(1<<DIN)); //lcd pin low
		}

	LCD_Port |=1<<CLK;			//Pulse the Clock line (Shift bit into register)
	LCD_Port&=(~(1<<CLK));
	
	}
	
	LCD_Port |=1<<CE;							// set chip enable high again
}

void LCDInitialize(void)
{
	//SPCR |=1<<SPE | 1<<MSTR;										// if spi is used, uncomment this section out
	//SPSR |=1<<SPI2X;
	LCD_Port_Datadir |=1<<DIN | 1<<CLK | 1<<CE | 1<<DC | 1<< RST;	// set spi pins and data/command and reset pins to output
	LCD_Port |=1<<RST;												// set reset high -> required for start up of display (receives a cycle in lcdinitialize)
	LCD_Port |=1<<PINB4;
	
	_delay_ms(15);
	LCD_Port &=~1<<RST;			//resetpin lcd low
	_delay_ms(64);
	LCD_Port |=1<<RST;			//resetpin lcd high
	LCD_Port &=~1<<CE;			//CE pin low
	
		
	SendCommand_LCD(0x21);		//set lcd control in additional command set (H=1)
	SendCommand_LCD(0xBE);		//set lcd VOP control (contrast)
	SendCommand_LCD(0x06);		//set lcd Temp coefficient
	SendCommand_LCD(0x13);		//set lcd bias mode
	SendCommand_LCD(0x20);		//set lcd control in basic mode set (H=0)
	SendCommand_LCD(0x0C);		//set display configuration control in normal mode (alternative: blank, all on, inverse video)
	//SendCommand_LCD(0x0D);	//set display configuration control in inverse mode
	
	LCD_clear();
			
}

void LCD_clear ( void )
{
    /* First flush the RAM memory of the LCD */
	int i,j;	
	for(i=0; i<8; i++)
	  for(j=0; j<90; j++)
	     SendData_LCD(0x00);   
    LCD_gotoXY (0,0);	//bring the XY position back to (0,0)
	
	/* Clear the LCDBuffer array to remove any existing bitmaps*/
	for (int i=0; i<504;i++)
	{
		LCDBuffer[i] = 0x00;
		SendData_LCD(LCDBuffer[i]);
	}
     
}

void LCD_Update (void)			
{
	for (int i=0; i<504;i++)
	{
		
		SendData_LCD(LCDBuffer[i]);
		
	}
}

/*X max 84, Y max 5*/

  void LCD_gotoXY ( unsigned char x, unsigned char y )
{
    SendCommand_LCD(0x80 | x);   //column
	SendCommand_LCD(0x40 | y);   //row
}

void DrawRectangle (int x1,int y1, int x2, int y2, char bw, char fill)	// LCD_Update required after this function
{
	if (x1<0)	x1=0;	if (x1>83)	x1=83;
	if (x2<0)	x2=0;	if (x2>83)	x2=83;
	if (y1<0)	y1=0;	if (y1>47)	y1=47;
	if (y2<0)	y2=0;	if (y2>47)	y2=47;
	
	/*int RectangleStartByte = ((y1 / 8)*84)+x1;		// determine start byte of upper line -> upper left corner
	int RectangleEndByte = ((y2 /8)*84)+x1;			// determine start byte of lower line -> lower left corner
	char Rectangle_Y1_Bit = y1%8;					// which bit is affected in start byte
	char Rectangle_Y2_Bit = y2%8;					// which bit is affected in end byte
	char Rectangle_X_Delta = x2-x1;					// what is delta of x coordinates
	
	
	for (int i=0; i<Rectangle_X_Delta; i++)			// both horizontal lines of rectangle
	{
		LCDBuffer[i+RectangleStartByte]|=1<<Rectangle_Y1_Bit;
		LCDBuffer[i+RectangleEndByte] |=1<<Rectangle_Y2_Bit;
	}
	
	for (int i=0; i<8-Rectangle_Y1_Bit; i++)			// complement start + end bytes of upper line for vertical line
	{
		LCDBuffer[RectangleStartByte]|=1<<(Rectangle_Y1_Bit+i);
		LCDBuffer[RectangleStartByte+Rectangle_X_Delta] |=1<<(Rectangle_Y1_Bit+i);
	}
	
	for (int i=0; i<=Rectangle_Y2_Bit; i++)			// complement start + end bytes of upper line for vertical line
	{
		LCDBuffer[RectangleEndByte]|=1<<(Rectangle_Y2_Bit-i);
		LCDBuffer[RectangleEndByte+Rectangle_X_Delta] |=1<<(Rectangle_Y2_Bit-i);
	}
	while (RectangleStartByte<RectangleEndByte-84)
	{
		RectangleStartByte +=84;
		LCDBuffer[RectangleStartByte]|=0xFF;
		LCDBuffer[RectangleStartByte+Rectangle_X_Delta]|=0xFF;
	}*/
	if (fill == NotFilled)				//draw a not filled rectangle
	{
		for (int i=0; i<(x2-x1); i++)
		{
			SetPixel(i+x1,y1,bw);		// upper line
			SetPixel(i+x1,y2,bw);		// lower line
		}
		
		for (int i=0; i<=(y2-y1); i++)
		{
			SetPixel(x1,i+y1,bw);		// left line
			SetPixel(x2,i+y1,bw);		// right line
		}
	}
	
	else								//draw a filled rectangle by drawing a vertical line from x1 posn to x2 posn
	{
		for (int j=0; j<(x2-x1); j++)
			{
				for (int i=0; i<=(y2-y1); i++)
				{
					SetPixel(j+x1,i+y1,bw);		
			
				}
			}
	}
	
		
}

void DrawLine (char x1, char y1, char x2, char y2,char bw)
{
	int dy = y2 - y1; // Difference between y1 and y2
	int dx = x2 - x1; // Difference between x2 and x1
	int stepx, stepy;

	if (dy < 0)
	{
		dy = -dy;
		stepy = -1;
	}
	else
	stepy = 1;

	if (dx < 0)
	{
		dx = -dx;
		stepx = -1;
	}
	else
	stepx = 1;

	dy <<= 1; // dy is now 2*dy
	dx <<= 1; // dx is now 2*dx
	SetPixel(x1, y1,bw); // Draw the first pixel.

	if (dx > dy)
	{
		int fraction = dy - (dx >> 1);
		while (x1 != x2)
		{
			if (fraction >= 0)
			{
				y1 += stepy;
				fraction -= dx;
			}
			x1 += stepx;
			fraction += dy;
			SetPixel(x1, y1,bw);
		}
	}
	else
	{
		int fraction = dx - (dy >> 1);
		while (y1 != y2)
		{
			if (fraction >= 0)
			{
				x1 += stepx;
				fraction -= dy;
			}
			y1 += stepy;
			fraction += dx;
			SetPixel(x1, y1,bw);
		}
	}
}

void ReplaceBitmap (char ReplacementBitmap[])	//replaces the bitmap by filling the LCDBuffer with the ReplacementBitmap bytes, LCD_Update required after this function
{
	for (int i=0; i<504; i++)
	{
		LCDBuffer[i] = ReplacementBitmap[i];
	}
	
}
void AddBitmap (char AddedBitmap[])				//adds to the existing bitmap by masking the LCDBuffer with the AddedBitmap bytes, LCD_Update required after this function
{
	for (int i=0; i<504; i++)
	{
		LCDBuffer[i] |= AddedBitmap[i];
	}
}

void SetPixel (unsigned char xp, unsigned char yp, char bw)
{
	int YBit = yp%8;				//determine which bit is affected by using modulo
	int YByte = ((yp/8)*84)+xp;		//determine which byte is affected
	
	if (bw == Black)				//draw a black pixel
		LCDBuffer[YByte] |=1<<YBit;
	else 
		LCDBuffer[YByte] &=~(1<<YBit);	//draw a white pixel		
}
