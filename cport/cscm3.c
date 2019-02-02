/**
 * SMC4 servo motor controller v4
 *
 */


#include "cscm3.h"
#include "control.h"

/* --- global variables --------------------------------------------------- */


//;----------------------------------------------------------;
//; EEPROM Area

//.eseg
const struct control_s parameter_bank[4] = {
	// Memory bank 0 : Yasukawa Electric UGTMEM-A1SA51
//	{	300, 0x0500, 0x0300, 0x00c0, 240, 0x0340,  0x2600, 128},
	// Mein DeskJet-Motor im XY-Tisch
	{	 80, 0x0500, 0x0300, 0x00c0, 240,      9,     160, 128},

	// Memory bank 1 : Yasukawa Electric UGTMEM-A1SA51
	{	300, 0x0500, 0x0300, 0x00c0, 240, 0x0340,  0x2600, 128},

	// Memory bank 2 : Tamagawa Seiki TS1410N1
	{	300, 0x0800, 0x0300, 0x0060, 180, 0x0550,  0x1a00, 32},

	// Memory bank 3 : Matsushita Electric MCN-14EAEC (6V, 40p/r)
	{	200, 0x0800, 0x0a00, 0x0400, 200, 0x0840,  0x1400, 64}
};





//;----------------------------------------------------------;
//; Data memory area

//; Servo / G command parameters
//Parms:
uint16_t	LimSpd;	// P0,Velocity limit		Integer
uint16_t	GaSpd;	// P1,Velocity feedback gain	8.8 fixed point
uint16_t	GaTqP;	// P2,Proportional gain		8.8 fixed point
uint16_t	GaTqI;	// P3,Integral gain		8.8 fixed point
uint16_t	LimTrq;	// P4,Torque limit		Integer
uint16_t	GaEG;	// P5,EG feedback gain		8.8 fixed point
uint16_t	MvSpd;	// P6,G0 velocity		Integer
uint16_t	MvAcc;	// P7,G0 acceleration		Integer

//; Command/Servo registers
int24_t		CtPos;	// Position 		g/j	mode 3
int16_t		CtSub;	// Sub command   	s	mode 0/1/2
int16_t		PvInt;	// Integration register
int16_t		PvPosM;	// Velocity detection register
uint8_t		Mode;	// Servo Mode		m

//; Status registers
uint24_t	Pos;	// current position
uint8_t		Flags=0;// 1kHz|Sat.F|Sat.R| | | |



void setup()
{
	PORTD	= 0b01111111;	//	;Initialize PORTD
	DDRD	= 0b00000010;

#if defined(__ATtiny2313__) || defined(__ATtiny2313A__)
	PORTB	= 0b10000000;	//	;Initialize PORTB
	DDRB	= 0b11111111;
#elsif defined(__ATmega328P__)
	PORTB	= 0b00100000;	//	;Initialize PORTB
	DDRB	= 0b11111111;

	PORTC	= 0;		//	;Initialize PORTC
	DDRC	= 0b00001100;
#endif

	Serial.begin(BPS);

	// TC1: 8bit PWM mode
	OCR1A	= 128;
	OCR1B	= 128;
	TCCR1A	= 0b10100001;
	TCCR1B	= 0b00000001;

/*FIXME: we need to get into the Tim0 interrupt
	outi	OCR0A, 2		;TC0: 83kHz interval timer
	outi	TCCR0A, 0b00000010	;
	outi	TCCR0B, 0b00000011	;
	outi	TIMSK, (1<<OCIE0A)	;/
*/
#if defined(USICR)
	USICR	= 0b00011100;	//	;USI: LED display
#else
//	;FIXME: initialize 3-wire SPI instead
#endif

	load_parm(0);		//	;Load servo parms form bank 0
	init_servo(0);		//	;Initial servo mode = 0
	dp_str(m_start);	//	;Start up message
}



// --------




/**
 * Tim0 interrupt routine
 *
 * called 83.3 times per millisecond (divider 192@16MHz).
 *
 * every time:
 * - Polls the position encoder, update the position counter
 * Every millisecond:
 * - Calls the control loop
 * - updates the position display
 */
void background ()
{
	static uint8_t CtDiv;

	position_capture();
	if (--CtDiv == 0) {
		CtDiv = 83;
		servo_operation();	// once every ms
		disp_pos();
	}
}
