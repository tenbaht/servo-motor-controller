/**
 * SMC4 servo motor controller v4
 *
 */

#ifndef _CSCM3_H_
#define _CSCM3_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define uint24_t	uint16_t	//FIXME: implement 24 bit arithmetics
#define int24_t		int16_t		//FIXME: implement 24 bit arithmetics

/* --- System configuration ----------------------------------------------- */

#define SYSCLK	16000000	// System clock
#define BPS	38400		// UART bps
#define TL_TIME	1500		// Error timer (tError(ms)=TL_TIME/3)



/* --- LED pin mapping ---------------------------------------------------- */

#if defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny2313A__)
 #define LED_ERROR	PORTB,0
 #define LED_TORQUE	PORTB,1
 #define LED_READY	PORTB,2

#elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
 #define LED_ERROR	PORTB,0
 #define LED_TORQUE	PORTC,3
 #define LED_READY	PORTC,2

#else
 #error "no LED pin definition found for this device"
#endif

#define led_on(PORT,BIT)	PORT|=(1<<BIT)
#define led_off(PORT,BIT)	PORT&=~(1<<BIT)

/*
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
*/


/* --- control parameters -------------------------------------------------- */

#define N_PARM	8	// Number of parameter words per bank.

struct control_s {
	uint16_t	P[N_PARM];
};


/* --- global variables --------------------------------------------------- */



//;----------------------------------------------------------;
//; Data memory area

//; Servo / G command parameters
extern uint16_t Parms[N_PARM];

#define LimSpd	Params[0];	//P0,Velocity limit		Integer
#define GaSpd	Params[1];	//:	.fill	2	;P1,Velocity feedback gain	8.8 fixed point
#define GaTqP	Params[2];	//:	.fill	2	;P2,Proportional gain		8.8 fixed point
#define GaTqI	Params[3];	//:	.fill	2	;P3,Integral gain		8.8 fixed point
#define LimTrq	Params[4];	//:	.fill	2	;P4,Torque limit		Integer
#define GaEG	Params[5];	//:	.fill	2	;P5,EG feedback gain		8.8 fixed point
#define MvSpd	Params[6];	//:	.fill	2	;P6,G0 velocity			Integer
#define MvAcc	Params[7];	//:	.fill	2	;P7,G0 acceleration		Integer

//; Command/Servo registers
extern int24_t	CtPos;	// Position 		g/j	mode 3
extern int16_t	CtSub;	// Sub command   	s	mode 0/1/2
extern int16_t	PvInt;	// Integration register
extern int16_t	PvPosM;	// Velocity detection register
extern uint8_t	Mode;	// Servo Mode		m

//; Status registers
extern uint24_t	Pos;	// current position
#define Flags	GPIOR0	// 1kHz|Sat.F|Sat.R| | | |

#ifdef __cplusplus
}
#endif

#endif
