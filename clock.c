/***************************************************************************
 Retro LED Clock firmware
 (c) 2009 Len Popp
 Includes portions of Ice Tube Clock firmware (c) 2009 Limor Fried / Adafruit Industries
 Includes portions of auto-dimmer mod by Dave Parker
 Includes portions of time correction mod by fat16lib

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
****************************************************************************/

#include <avr/io.h>      
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

// Optional Features - #define these or not as desired.
// Define this if an external 16MHz crystal is used for CPU clock & timer
// Undefine it if a 32KHz watch xtal is used & CPU is running on 8MHz internal osc.
//#define FEATURE_FAST_XTAL
// Time correction - see below for correction factor
//#define FEATURE_CORRECT_TIME
// Loud or soft alarm beeper
#define FEATURE_LOUD_BEEPER

// Figure out if CPU is running at 8 or 16 MHz
#if !defined(FEATURE_FAST_XTAL)
	// 8MHz CPU when using int oscillator & watch crystal
	#undef FAST_CPU
#elif F_CPU == 8000000
	#undef FAST_CPU
#elif F_CPU == 16000000
	#define FAST_CPU
#else
	unknown CPU speed
#endif

#ifdef FEATURE_CORRECT_TIME
// Define CORRECT_TIME_SLOW if the clock is slow or CORRECT_TIME_FAST if it's fast.
//#define CORRECT_TIME_SLOW
#define CORRECT_TIME_FAST
// Estimate how many seconds are gained or lost per day
// and set TIME_CORRECTION to <seconds>*256)/24
#define TIME_CORRECTION		100
#define CORRECT_TIME_REALLY_FAST
#define TIME_CORRECTION_COARSE	120
#endif

// Current Mode
volatile uint8_t bMode = 0;
#define MODE_SHOW_TIME	1
#define MODE_SET_ALARM	2
#define MODE_SET_TIME	3

uint16_t nModeTimeout = 0;
#define MODE_TIMEOUT	(3*100)
uint16_t nHoldTimeout = 0;
#define HOLD_TIMEOUT	(2*100)
#define HOLD_COUNT		(100/4)

// Current Time
volatile uint8_t timeH;
volatile uint8_t timeM;
volatile uint8_t timeS;

// Alarm Time
volatile uint8_t alarmH;
volatile uint8_t alarmM;
volatile uint8_t bAlarmOn;
volatile uint16_t tSnooze;
#define SNOOZE_TIME		(10*60)

// Switches
// Alarm switch is Arduino pin d3
#define SW_ALARM_PIN		_BV(PORTD3)
#define SW_ALARM_PORT		PORTD
#define SW_ALARM_INPORT		PIND
#define SW_ALARM_DIR_REG	DDRD
#define SW_ALARM			0x01
// Mode button is Arduino pin d5
#define SW_MODE_PIN			_BV(PORTD5)
#define SW_MODE_PORT		PORTD
#define SW_MODE_INPORT		PIND
#define SW_MODE_DIR_REG		DDRD
#define SW_MODE				0x02
// Set button is Arduino pin d4
#define SW_SET_PIN			_BV(PORTD4)
#define SW_SET_PORT			PORTD
#define SW_SET_INPORT		PIND
#define SW_SET_DIR_REG		DDRD
#define SW_SET				0x04

// switch flags combined in 1 byte
volatile uint8_t bSwitches = 0;
volatile uint8_t bSwitchesJustPressed = 0;
volatile uint8_t bSwitchesPending = 0;
volatile uint8_t bSetSwitchHeld = 0;

#define isSwitchOn(SW) ((bSwitches & (SW)) != 0)
#define isJustPressed(SW) ((bSwitchesJustPressed & (SW)) != 0)

// Our display buffer, which is updated to show the time/date/etc
// and is latched into the display driver
#define DISPLAYSIZE 4
uint8_t display[DISPLAYSIZE]; // stores segments, not values!
uint8_t displayAM;
uint8_t displayPM;
uint8_t displayColonTop;
uint8_t displayColonBottom;

// Minimum number of segments that can be lit (other than zero)
// This is needed to prevent too much current through one LED segment.
#define MIN_SEGS 10

// LED segments to illuminate to display a given digit
const uint8_t numbertable[] PROGMEM = { 
	/*0*/ 0x6F,
	/*1*/ 0x03,
	/*2*/ 0x5D,
	/*3*/ 0x57,
	/*4*/ 0x33,
	/*5*/ 0x76,
	/*6*/ 0x7E,
	/*7*/ 0x43,
	/*8*/ 0x7F,
	/*9*/ 0x77,
};
PGM_P numbertable_p PROGMEM = numbertable;

// CLOCK is Arduino pin d12
#define CLOCK_PIN		_BV(PORTB4)
#define CLOCK_PORT		PORTB
#define CLOCK_DIR_REG	DDRB

// DATA is Arduino pin d11
#define DATA_PIN		_BV(PORTB3)
#define DATA_PORT		PORTB
#define DATA_DIR_REG	DDRB

// PWM brightness control for LED driver is Arduino pin d6
#define BRIGHT_PIN		_BV(PORTD6)
#define BRIGHT_DIR_REG	DDRD
#define BRIGHTNESS_MAX	255
#define BRIGHTNESS_MIN	65

// Light sensor (photocell) input for auto-dimmer is Arduino pin a4
#define PHOTOCELL_INPUT		_BV(MUX2)
#define PHOTOCELL_PIND		_BV(ADC4D)
#define PHOTOCELL_DARK		10
#define PHOTOCELL_LIGHT		500

// Piezo speaker is on Arduino pins d9 & d10
#define SPKR1_PIN		_BV(PORTB1)
#define SPKR2_PIN		_BV(PORTB2)
#define SPKR_PORT		PORTB
#define SPKR_DIR_REG	DDRB

volatile uint8_t bBeeperOn = 0;
volatile uint8_t bBeeperSwitch = 0;

void initClock(void);
void initSwitches(void);
void checkSwitches(void);
void checkSwitch(uint8_t bInPort, uint8_t bPin, uint8_t bSwFlag);
void displayTime(uint8_t h, uint8_t m, uint8_t s, uint8_t colonTop, uint8_t colonBottom);
void initLEDDisplay(void);
void setBrightness(uint8_t brightness);
void writeDisplay(void);
void initDimmer(void);
void updateDimmer(void);
void initSpeaker(void);
void beeperOn(void);
void beeperOff(void);

//**** Main

int main(void)
{
	uint8_t nHoldCount = 0;
	uint8_t bAutoIncrement;

	initLEDDisplay();
	// seekrit startup message:
	display[0] = 0;
	display[1] = 0x18;
	display[2] = 0x1E;
	display[3] = 0x3E;
	displayAM = 0;
	displayPM = 0;
	displayColonTop = 0;
	displayColonBottom = 0;
	writeDisplay();

	initDimmer();

	initSwitches();

	initClock();

	bMode = MODE_SHOW_TIME;
	//displayTime(timeH, timeM, timeS, 1, 1);
	// no, leave the startup message displayed for the first second

	initSpeaker();

	for (;;)
	{		
		checkSwitches();

		// Handle switches & other stuff

		// Turn alarm on/off
		if (isSwitchOn(SW_ALARM) && !bAlarmOn) {
			bAlarmOn = 1;
			tSnooze = 0;
		} else if (!isSwitchOn(SW_ALARM) && bAlarmOn) {
			bAlarmOn = 0;
			tSnooze = 0;
			beeperOff();
		}

		// Snooze switch (Set switch when not setting the time)
		if (bMode == MODE_SHOW_TIME && bBeeperOn && isJustPressed(SW_SET)) {
			beeperOff();
			tSnooze = SNOOZE_TIME;
		}

		// Mode switch
		if (isJustPressed(SW_MODE)) {
			switch (bMode) {
				case MODE_SHOW_TIME:
					bMode = MODE_SET_ALARM;
					displayTime(alarmH, alarmM, 0, 1, 0);
					break;
				case MODE_SET_ALARM:
					bMode = MODE_SET_TIME;
					displayTime(timeH, timeM, timeS, 0, 1);
					break;
				case MODE_SET_TIME:
					bMode = MODE_SHOW_TIME;
					displayTime(timeH, timeM, timeS, 1, 1);
					break;
			}
		}

		// Check for auto-increment when the Set button is held down.
		bAutoIncrement = 0;
		if (bSetSwitchHeld) {
			if (nHoldCount > 0) {
				nHoldCount--;
			} else {
				nHoldCount = HOLD_COUNT;
				bAutoIncrement = 1;
			}
		}

		// Set alarm or time when Set button is pressed.
		if (bMode == MODE_SET_ALARM) {
			if (bAutoIncrement)
				alarmM = (alarmM / 10) * 10 + 10;
			else if (isJustPressed(SW_SET))
				alarmM++;
			if (alarmM >= 60) {
				alarmM = 0;
				if (++alarmH >= 24) {
					alarmH = 0;
				}
			}
			displayTime(alarmH, alarmM, 0, 1, 0);
		} else if (bMode == MODE_SET_TIME) {
			if (bAutoIncrement) {
				timeM = (timeM / 10) * 10 + 10;
				timeS = 0;
			} else if (isJustPressed(SW_SET)) {
				timeM++;
				timeS = 0;
			}
			if (timeM >= 60) {
				timeM = 0;
				if (++timeH >= 24) {
					timeH = 0;
				}
			}
			displayTime(timeH, timeM, timeS, 0, 1);
		}

		// After a few seconds of inactivity, return to time display.
		if (bMode != MODE_SHOW_TIME && nModeTimeout == 0) {
			bMode = MODE_SHOW_TIME;
			displayTime(timeH, timeM, timeS, 1, 1);
		} else if (nModeTimeout > 0) {
			nModeTimeout--;
		}

		_delay_ms(10);
	}
}

//**** Clock Timekeeping

void initClock(void)
{
	timeH = 12;
	timeM = 34;
	timeS = 56;

	alarmH = 12;
	alarmM = 35;
	bAlarmOn = 0;
	tSnooze = 0;

	// Turn on the RTC by selecting the external 32khz crystal
	// 32.768 / 128 = 256 which is exactly an 8-bit timer overflow
	ASSR |= _BV(AS2); // use crystal
	TCCR2B = _BV(CS22) | _BV(CS20); // div by 128
	// We will overflow once a second, and call an interrupt

	// enable interrupt
	TIMSK2 = _BV(TOIE2);

	// enable all interrupts!
	sei();
}

#ifdef FEATURE_FAST_XTAL
// Timer scaling when using 16MHz crystal instead of watch crystal
volatile uint16_t scaleTimer2 = 0;
#define TIMER2_COUNT  488
#endif

// Clock Interrupt
// this goes off once a second
SIGNAL (TIMER2_OVF_vect)
{
	CLKPR = _BV(CLKPCE);  //MEME
	CLKPR = 0;

#ifdef FEATURE_FAST_XTAL
	// when using 16MHz xtal osc, scale down to 32768 Hz (approx)
	if (++scaleTimer2 < TIMER2_COUNT)
		return;
	scaleTimer2 = 0;
#endif

	timeS++;             // one second has gone by

	// a minute!
	if (timeS >= 60) {
		timeS = 0;
		timeM++;
#ifdef FEATURE_CORRECT_TIME
#if defined(CORRECT_TIME_REALLY_FAST)
		// The clock is fast. Lengthen the first second of the minute.
		uint8_t tmp = TCNT2;
		while ((TCNT2 - tmp) < TIME_CORRECTION_COARSE)
			;
		TCNT2 = tmp;
#endif
#endif
	}

	// an hour...
	if (timeM >= 60) {
		timeM = 0;
		timeH++;
#ifdef FEATURE_CORRECT_TIME
#if defined(CORRECT_TIME_SLOW)
		// The clock is slow. Shorten the first second of the hour.
		TCNT2 += TIME_CORRECTION;
#elif defined(CORRECT_TIME_FAST)
		// The clock is fast. Lengthen the first second of the hour.
		uint8_t tmp = TCNT2;
		while ((uint8_t)(TCNT2 - tmp) < (uint8_t)(TIME_CORRECTION+TIME_CORRECTION_COARSE))
			;
		TCNT2 = tmp;
#endif
#endif
	}

	// a day
	if (timeH >= 24) {
		timeH = 0;
	}

	// Check the alarm
	if (bAlarmOn && timeH == alarmH && timeM == alarmM && timeS == 0) {
		beeperOn();
		tSnooze = 0;
	}
	if (tSnooze > 0) {
		if (--tSnooze == 0) {
			beeperOn();
		}
	}

	if (bMode == MODE_SHOW_TIME)
		displayTime(timeH, timeM, timeS, 1, 1);

	updateDimmer();
}

//**** Switches

void initSwitches(void)
{
	// Initialize switch inputs with pull-up resistors.
	SW_ALARM_DIR_REG &= ~SW_ALARM_PIN;
	SW_ALARM_PORT |= SW_ALARM_PIN;

	SW_MODE_DIR_REG &= ~SW_MODE_PIN;
	SW_MODE_PORT |= SW_MODE_PIN;

	SW_SET_DIR_REG &= ~SW_SET_PIN;
	SW_SET_PORT |= SW_SET_PIN;
}

void checkSwitches(void)
{
	// Check which switches are pressed.
	bSwitchesJustPressed = 0;
	checkSwitch(SW_ALARM_INPORT, SW_ALARM_PIN, SW_ALARM);
	checkSwitch(SW_MODE_INPORT, SW_MODE_PIN, SW_MODE);
	checkSwitch(SW_SET_INPORT, SW_SET_PIN, SW_SET);
	// If any switches are pressed, re-start mode timeout.
	if (bSwitches & (SW_SET | SW_MODE))
		nModeTimeout = MODE_TIMEOUT;
	// Check if the Set switch is held down.
	if (isJustPressed(SW_SET)) {
		nHoldTimeout = HOLD_TIMEOUT;
		bSetSwitchHeld = 0;
	} else if (isSwitchOn(SW_SET)) {
		if (nHoldTimeout > 0)
			nHoldTimeout--;
		else
			bSetSwitchHeld = 1;
	} else {
		nHoldTimeout = 0;
		bSetSwitchHeld = 0;
	}
}

void checkSwitch(uint8_t bInPort, uint8_t bPin, uint8_t bSwFlag)
{
	uint8_t bSw = ((bInPort & bPin) == 0);
	// debounce the switch
	if (bSw != ((bSwitches & bSwFlag) != 0)) {
		if (bSwitchesPending & bSwFlag) {
			// switch the switch!
			if (bSw) {
				bSwitches |= bSwFlag;
				bSwitchesJustPressed |= bSwFlag;
			} else {
				bSwitches &= ~bSwFlag;
			}
			bSwitchesPending &= ~bSwFlag;
		} else {
			bSwitchesPending |= bSwFlag;
		}
	} else {
		bSwitchesPending &= ~bSwFlag;
	}
}

//**** LED Display

void displayTime(uint8_t h, uint8_t m, uint8_t s, uint8_t colonTop, uint8_t colonBottom)
{
	// Display time in 12h am/pm format
	displayAM = (h < 12);
	displayPM = (h >= 12);
	h = ((h + 11) % 12) + 1;
	display[0] =  pgm_read_byte(numbertable_p + h/10);
	display[1] =  pgm_read_byte(numbertable_p + h%10);
	display[2] =  pgm_read_byte(numbertable_p + m/10);
	display[3] =  pgm_read_byte(numbertable_p + m%10);

	// Colons between hours & minutes are optional.
	// (They're used to indicate the current mode.)
	displayColonTop = colonTop;
	displayColonBottom = colonBottom;

	writeDisplay();
}

void initLEDDisplay(void)
{
	int i;
	for (i = 0; i < DISPLAYSIZE; i++)
		display[i] = 0;
	displayAM = displayPM = displayColonTop = displayColonBottom = 0;

	CLOCK_PORT &= ~CLOCK_PIN;
	CLOCK_DIR_REG |= CLOCK_PIN;
	DATA_PORT &= ~DATA_PIN;
	DATA_DIR_REG |= DATA_PIN;

	// Init PWM for brightness control to display driver chip
	setBrightness(255);
	// fast PWM, set OC0A (brightness output pin) on match
	TCCR0A = _BV(WGM00) | _BV(WGM01) | _BV(COM0A1);  
	// Use the fastest clock
	TCCR0B = _BV(CS00);

	TIMSK0 |= _BV(TOIE0); // turn on the interrupt for muxing

	BRIGHT_DIR_REG |= BRIGHT_PIN;
}

void setBrightness(uint8_t brightness)
{
	// Set PWM value to control the LED brightness
	// Not too bright
	if (brightness > BRIGHTNESS_MAX)
		brightness = BRIGHTNESS_MAX;

	// Or so low its not visible
	if (brightness < BRIGHTNESS_MIN)
		brightness = BRIGHTNESS_MIN;

	if (OCR0A != brightness)
		OCR0A = brightness;
}

void writeSegBit(uint8_t fOn)
{
	// set/reset data line
	if (fOn)
		DATA_PORT |= DATA_PIN;
	else
		DATA_PORT &= ~DATA_PIN;    
	_delay_us(1);
	// raise clock for a while
	CLOCK_PORT |= CLOCK_PIN;
	_delay_us(2);
	CLOCK_PORT &= ~CLOCK_PIN;
	_delay_us(1);
}

int countSegs(void)
{
	int iDigit;
	int iBit;
	uint8_t bSegs;
	int cSegs;

	cSegs = 0;
	for (iDigit = 0; iDigit < DISPLAYSIZE; iDigit++) {
		bSegs = display[iDigit];
		for (iBit = 0; iBit < 7; iBit++) {
			cSegs += ((bSegs & _BV(iBit)) != 0);
		}
	}
	cSegs += displayAM + displayPM + displayColonTop + displayColonBottom;

	return cSegs;
}

void writeDisplay(void)
{
	int cSegs;
	uint8_t bSegs;

	// Make sure enough segments are lit so we don't pump too much current
	// through one LED segment.
	cSegs = countSegs();
	if (cSegs < MIN_SEGS && cSegs != 0) {
		// Turn on a few extra segments to give enough load
		display[0] = 0x7F;
		displayAM = 1;
	}

	// Write all segments to the display driver.
	CLOCK_PORT &= ~CLOCK_PIN; // should be redundant
	// First: the start bit
	writeSegBit(1);
	// Next the digits & dots
	// WTF: Each digit's segments are wired in a different order.
	// So each digit must be output differently.
	bSegs = display[3];
	writeSegBit(bSegs & _BV(1));
	writeSegBit(bSegs & _BV(2));
	writeSegBit(bSegs & _BV(3));
	writeSegBit(bSegs & _BV(0));
	writeSegBit(bSegs & _BV(6));
	writeSegBit(bSegs & _BV(4));
	writeSegBit(bSegs & _BV(5));
	bSegs = display[2];
	writeSegBit(bSegs & _BV(1));
	writeSegBit(bSegs & _BV(3));
	writeSegBit(bSegs & _BV(2));
	writeSegBit(bSegs & _BV(0));
	writeSegBit(bSegs & _BV(6));
	writeSegBit(bSegs & _BV(4));
	writeSegBit(bSegs & _BV(5));
	// unused outputs
	writeSegBit(0);
	writeSegBit(0);
	writeSegBit(0);
	// colon is 2 separate LEDs
	writeSegBit(displayColonBottom);
	writeSegBit(displayColonTop);
	bSegs = display[1];
	writeSegBit(bSegs & _BV(1));
	writeSegBit(bSegs & _BV(2));
	writeSegBit(bSegs & _BV(3));
	writeSegBit(bSegs & _BV(0));
	writeSegBit(bSegs & _BV(6));
	writeSegBit(bSegs & _BV(4));
	writeSegBit(bSegs & _BV(5));
	bSegs = display[0];
	writeSegBit(bSegs & _BV(0));
	writeSegBit(bSegs & _BV(1));
	writeSegBit(bSegs & _BV(2));
	writeSegBit(bSegs & _BV(3));
	writeSegBit(bSegs & _BV(4));
	writeSegBit(bSegs & _BV(5));
	writeSegBit(bSegs & _BV(6));
	writeSegBit(displayAM);
	writeSegBit(displayPM);
	// NOTE: This had better add up to 35 LED segments (plus start bit) - else the driver chip gets out of sync
}

// PWM Timer Interrupt
// called @ (F_CPU/256) = ~31.25 khz
SIGNAL (TIMER0_OVF_vect)
{
	// allow other interrupts to go off while we're doing this
	sei();

	// kick the dog
	//kickthedog();

	// Divide down to 10 Hz
	static uint16_t timerCount = 0;
#ifdef FAST_CPU
	#define TIMER_DIVIDER 6250
#else
	#define TIMER_DIVIDER 3125
#endif
	if (++timerCount < TIMER_DIVIDER)
		return;
	timerCount = 0;

	// Beeper - make it beep!
	if (bBeeperOn) {
		if (bBeeperSwitch) {
			bBeeperSwitch = 0;
			TCCR1B &= ~_BV(CS11); // turn beeper off temporarily
		} else {
			bBeeperSwitch = 1;
			TCCR1B |= _BV(CS11); // turn beeper on temporarily
		}
	}
}

//**** Auto-Dimmer

void initDimmer(void)
{
	// Initialize the photocell sensor
	uint8_t bPrescale;
#ifdef FAST_CPU
	// Set ADC prescalar to 128 for 125KHz sample rate with F_CPU = 16MHz
	bPrescale = _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
#else
	// Set ADC prescalar to 64 for 125KHz sample rate with F_CPU = 8MHz
	bPrescale = _BV(ADPS2) | _BV(ADPS1);
#endif
	ADCSRA |= bPrescale;
	ADMUX |= _BV(REFS0);  // Set ADC reference to AVCC
	ADMUX |= PHOTOCELL_INPUT;   // Set ADC input as ADC4 (PC4)
	DIDR0 |= PHOTOCELL_PIND; // Disable the digital input buffer on the sense pin to save power.
	ADCSRA |= _BV(ADEN);  // Enable ADC
	ADCSRA |= _BV(ADIE);  // Enable ADC interrupt
}

void updateDimmer(void)
{
	// Kick off the analog input conversion
	ADCSRA |= _BV(ADSC);
	// When it's complete an interrupt will be signalled.
}

// ADC Complete Interrupt
SIGNAL (ADC_vect)
{
	uint8_t low, high;
	unsigned int val;

	// Read 2-byte value. Must read ADCL first because that locks the value.
	low = ADCL;
	high = ADCH;
	val = (high << 8) | low;

	// Set brightness to a value between min & max based on light reading.
	if (val <= PHOTOCELL_DARK) {
		val = BRIGHTNESS_MIN;
	} else if (val >= PHOTOCELL_LIGHT) {
		val = BRIGHTNESS_MAX;
	} else {
		val = BRIGHTNESS_MIN + (((unsigned long)(BRIGHTNESS_MAX - BRIGHTNESS_MIN)) *
			(val - PHOTOCELL_DARK)) / (PHOTOCELL_LIGHT - PHOTOCELL_DARK);
	}

	setBrightness(val);
}

//**** Speaker

void initSpeaker(void)
{
	// We use the built-in fast PWM, 8 bit timer
	SPKR_PORT |= SPKR1_PIN | SPKR2_PIN;
	SPKR_DIR_REG |= SPKR1_PIN | SPKR2_PIN;

	// Turn on PWM outputs for one or both pins.
	// For loud volume use both PWM with opposite phase.
	TCCR1A = _BV(COM1B1) | _BV(COM1B0) | _BV(WGM11);
#ifdef FEATURE_LOUD_BEEPER
	TCCR1A |= _BV(COM1A1);
#endif
	TCCR1B = _BV(WGM13) | _BV(WGM12);

	// start at 4khz:  250 * 8 prescaler * 4000 = 8mhz
#ifdef FAST_CPU
	ICR1 = 500;
#else
	ICR1 = 250;
#endif
	OCR1A = OCR1B = ICR1 / 2;
}

void beeperOn(void)
{
	// Turn on PWM counter with prescaler = 1/8
	TCCR1B |= _BV(CS11);
	bBeeperOn = 1;
	bBeeperSwitch = 1;
}

void beeperOff(void)
{
	TCCR1B &= ~_BV(CS11);
	bBeeperOn = 0;
}
