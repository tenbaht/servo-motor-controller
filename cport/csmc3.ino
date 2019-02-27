/**
 * SMC4 servo motor controller v4
 *
 */

#include "csmc3.h"
#include "cli.h"
#include "encoder.h"
#include "control.h"
//#include "eeprom.h"
//#include "uart.h"

/* --- global variables --------------------------------------------------- */




//;----------------------------------------------------------;
//; Data memory area

// this could be in control.c
//; Servo / G command parameters
uint16_t Parms[N_PARM];

//; Command/Servo registers
int24_t		CtPos;	// Position 		g/j	mode 3
int16_t		CtSub;	// Sub command   	s	mode 0/1/2
int16_t		PvInt;	// Integration register
int16_t		PvPos;	// Velocity detection register
uint16_t	OvTmr;	// Torque limit timer
uint8_t		Mode;	// Servo Mode		m

// this could be in encoder.c
//; Status registers
int24_t	Pos;		// current position


static const char m_start[] PROGMEM = "\r\nSMC type 3c\r\n";


void setup()
{
	PORTD	= 0b01111111;	//	;Initialize PORTD
	DDRD	= 0b00000010;

#if defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny2313A__)
	PORTB	= 0b10000000;	//	;Initialize PORTB
	DDRB	= 0b11111111;
#elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
	PORTB	= 0b00100000;	//	;Initialize PORTB
	DDRB	= 0b11111111;

	PORTC	= 0;		//	;Initialize PORTC
	DDRC	= 0b00001100;
#else
 #error "unknown CPU type"
#endif

	Serial.begin(BPS);

	// TC1: 8bit PWM mode
	OCR1A	= 128;
	OCR1B	= 128;
	TCCR1A	= 0b10100001;
	TCCR1B	= 0b00000001;

#ifdef USE_TIM2
 #warning "using TIM2 instead of TIM0 for the background task"
	TCCR2A	= (1<<WGM21);		// CTC mode
	TCCR2B	= (1<<CS22);		// prescaler 1/64
	OCR2A	= 2;			// TC2: 83kHz interval timer
	TIMSK2	= (1<<OCIE2A);
#else
	outi	OCR0A, 2		;TC0: 83kHz interval timer
	outi	TCCR0A, 0b00000010	;CTC mode (count up to OCR0A)
	outi	TCCR0B, 0b00000011	;prescaler clk_io/64
	outi	TIMSK, (1<<OCIE0A)	;/
#endif

#if defined(USICR)
	USICR	= 0b00011100;	//	;USI: LED display
#else
//	;FIXME: initialize 3-wire SPI instead
#endif

//FIXME	load_parm(0);		//	;Load servo parms form bank 0
	init_servo(0);		//	;Initial servo mode = 0
	dp_str(m_start);	//	;Start up message
}



// --------



/**
 * Tim0 interrupt routine
 *
 * called 83.3 times per millisecond (divider 192=3*64@16MHz).
 *
 * every time:
 * - Polls the position encoder, update the position counter
 * Every millisecond:
 * - Calls the control loop
 * - updates the position display
 */
//void background ()
#ifdef USE_TIM2
 #if defined(__AVR_ATtiny2313__)
  ISR(TIM2_COMPA_vect)
 #else
  ISR(TIMER2_COMPA_vect)
 #endif
#else
 #if defined(__AVR_ATtiny2313__)
  ISR(TIM0_COMPA_vect)
 #else
  ISR(TIMER0_COMPA_vect)
 #endif
#endif
{
	static uint8_t CtDiv;

	position_capture();
	if (--CtDiv == 0) {
		servo_operation();	// once every ms
//		disp_pos();
		CtDiv = 83;
	}
}

void loop()
{
	task_cli();
}
