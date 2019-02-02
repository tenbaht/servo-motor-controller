/**
 * SMC4 servo motor controller v4
 *
 */


//.equ	SYSCLK	= 16000000	;System clock
//.equ	BPS	= 38400		;UART bps

#define TL_TIME = 1500		;Error timer (tError(ms)=TL_TIME/3)



/* --- LED pin mapping ---------------------------------------------------- */

#if defined(__ATtiny2313__) || defined(_ATtiny2313A__)
 #define	LED_ERROR_BIT	0
 #define	LED_ERROR_PORT	PORTB
 #define	LED_TORQUE_BIT	1
 #define	LED_TORQUE_PORT	PORTB
 #define	LED_READY_BIT	2
 #define	LED_READY_PORT	PORTB

#elsif defined( __ATmega328P__)
 #define	LED_ERROR_BIT	0
 #define	LED_ERROR_PORT	PORTB
 #define	LED_TORQUE_BIT	3
 #define	LED_TORQUE_PORT	PORTC
 #define	LED_READY_BIT	2
 #define	LED_READY_PORT	PORTC

#else
 #error "no LED pin definiton found for this device"
#endif


#define led_on(NAME)	asm("sbi	" STRINGIFY(NAME##_PORT) ", " STRINGYFY(NAME#_BIT) "; LED on")
#define led_off(NAME)	asm("cbi	" STRINGIFY(NAME##_PORT) ", " STRINGYFY(NAME#_BIT) "; LED off")


/* --- control parameters -------------------------------------------------- */

struct control_s {
	uint16_t	P[8];
};

#define N_PARM	8	// Number of parameter words per bank.

/* --- global variables --------------------------------------------------- */

//;----------------------------------------------------------;
//; EEPROM Area

//.eseg
extern const struct control_s parameter_bank[4];


//;----------------------------------------------------------;
//; Data memory area

//; Servo / G command parameters
//Parms:
extern uint16_t	LimSpd;	// P0,Velocity limit		Integer
extern uint16_t	GaSpd;	// P1,Velocity feedback gain	8.8 fixed point
extern uint16_t	GaTqP;	// P2,Proportional gain		8.8 fixed point
extern uint16_t	GaTqI;	// P3,Integral gain		8.8 fixed point
extern uint16_t	LimTrq;	// P4,Torque limit		Integer
extern uint16_t	GaEG;	// P5,EG feedback gain		8.8 fixed point
extern uint16_t	MvSpd;	// P6,G0 velocity		Integer
extern uint16_t	MvAcc;	// P7,G0 acceleration		Integer

//; Command/Servo registers
extern int24_t	CtPos;	// Position 		g/j	mode 3
extern int16_t	CtSub;	// Sub command   	s	mode 0/1/2
extern int16_t	PvInt;	// Integration register
extern int16_t	PvPosM;	// Velocity detection register
extern uint8_t	Mode;	// Servo Mode		m

//; Status registers
extern uint24_t	Pos;	// current position
extern uint8_t	Flags;// 1kHz|Sat.F|Sat.R| | | |
