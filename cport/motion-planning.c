
#include "csmc3.h"
#include "cli.h"
#include "motion-planning.h"
#include "uart.h"


/* --- private variables --------------------------------------------------- */

//static s24_8	cur_pos;	// next planned position, u24.8
//static s16_8	velocity;	// currently planned velocity (mostly unsigned)
//static bool	direction;	// 0 = positive, 1 = negative
static int32_t	cur_pos;	// next planned position, u24.8
static int24_t	velocity;	// currently planned velocity (mostly unsigned)
static uint8_t	direction;	// 0 = positive, 1 = negative

static int24_t	start_pos;	// start position of the move
static int24_t	target_pos;	// target position of the move


/* --- private functions --------------------------------------------------- */

static void dg_0();
static void dg_1();
static uint8_t dg_add();


/* --- implementation of public functions ---------------------------------- */

/**
;------------------------------------------------;
; Go at trapezoidal/rectangular velocity profile.
;
; local variables: (24 bit, LSB first in list)
; T0H:T2	current position (kind of loop variable) (s24.8)
; T4:T6L	start position (s24)
; A:BL		target position (s24)
; BH:C		velocity (u16.8)
; D:EL		next important point on ramp (midpoint, start of end ramp)
; Tflag		direction of movement (b1)

; 		T4:T6L = CtPos;	// s24
; cur_pos T0:T2  = CtPos << 8;	// s32 (wirklich?)
; cur_pos T0H:T2 = CtPos;	// s24 (wirklich?)
; if (get_val())
; {
;   if (val==0) goto dg_0();
;   if (val==1) goto dg_1();
; }
; goto cmd_err;
*/
void do_go()
{
	start_pos	= CtPos;
	cur_pos		= start_pos << 8;	// cur_pos is s24.8

	if (get_val()==0) {
		if (val==0) {
			dg_0();
			return;
		} else if (val==1) {
			dg_1();
			return;
		}
	}
	cmd_err();
}


/* --- implementation of private functions --------------------------------- */
	

/**
 * Rectangular velocity profile - G1 command
 *
 */
//void dg_1(uint24_t target, int24_t speed)
static void dg_1()
{
	uint8_t	c;

	if (get_val()) {cmd_err(); return;}
	target_pos = val;

	if (get_val()) {cmd_err(); return;}
	velocity = val;		// this is u16.8

	direction = (target_pos < start_pos);

	while ((c=dg_add())==0);	// loop until target pos is reached

	if (c==1) {
		// if not interrupted by the user, go for the target position
		CtPos = target_pos;
	}
}



/**
 * Trapezoidal velocity profile - G0 command
 *
 */
static void dg_0()
{
	uint8_t	c;
	uint8_t	keep_waiting;
	int24_t	next_mark;

	if (get_val()) {cmd_err(); return;}
	target_pos = val;

	direction = (target_pos < start_pos);

	// calculate the midpoint of the movement
	next_mark = (target_pos - start_pos)/2 + start_pos;

	// ---Up ramp loop
	velocity = 0;
	do {
		velocity += MvAcc;
		c = dg_add();	// do the next step
		if (c==1) goto dg_end;	// end position reached
		if (c==2) return;	// user interruption

		// check if we already passed the midpoint.
		// if yes, go straight for the down ramp.
		if (direction==0) {
			if (cur_pos>>8 >= next_mark) goto dg_de;
		} else {
			if (cur_pos>>8 <= next_mark) goto dg_de;
		}
	} while (velocity < MvSpd);	// FIXME: velocity might exceed MvSpd

	// the speed reached P6 now. Before we keep goint at this speed
	// calculate the down ramp point, assuming that the down ramp will
	// be the same width than the the up ramp:
	// up ramp width was (cur_pos-start_pos)
	// down ramp beginns at target - up_ramp_width = target - (cur-start)
	next_mark = start_pos - (cur_pos>>8) + target_pos;

	// ---Constant velocity loop
	for(;;) {
		if (dg_add()==2) return;	//check for user interruption

		// check if it is time for the down ramp:
		if (direction==0) {
			if (cur_pos>>8 >= next_mark) break;
		} else {
			if (cur_pos>>8 <= next_mark) break;
		}
	}

	// ---Down ramp loop
dg_de:
//	if (dg_add()==2) return;	//check for user interruption
	// the following decrement is the reason why velocity is only
	// "mostly unsigned". We definitly need a signed comparision here.
//	while (( (s16_8)velocity-=MvAcc) >= 0) {
//		if (dg_add()==0) break;
//	}

	c = dg_add();
	while ((velocity >= MvAcc) && (c==0)) {
		velocity -= MvAcc;
		c = dg_add();
	}
	if (c==2) return;	// user interruption

	// End of action
dg_end:
//	uint8_t	keep_waiting;

	BEGIN_CRITICAL;
	CtPos = target_pos;

	do {
		// access to Pos needs to be protected from IRQs, that's why
		// it can't be done inside the while statement.
		BEGIN_CRITICAL
		keep_waiting = (Pos!=target_pos);
		END_CRITICAL
	} while (keep_waiting && !Serial_available());	// allow to end the loop by keypress

/*
		//FIXME: the comparison needs to be protected from IRQs, but
		// that is difficult to express in C. Would be better to
		// define an inline function with some asm code.
		BEGIN_CRITICAL
		if (Pos!=target_pos) {
			END_CRITICAL
			break;
		}
		END_CRITICAL
	} while (!Serial.available());	// allow to end the loop by keypress
*/
}



/**
;
; do one 1ms step with the current velocity
;
; wait for 1kHz time interval without any torque flags set.
; advance cur_pos by (velocity)
; if that does not overrun the target position, advance CtPos by (velocity)
;
 * The return value differs from the assembler version:
 * @returns:
 *	0: all ok, target not reached yet (Assember: S=1, N^V=1)
 *	1: overrun target position (Assembler: N^V=0)
 *	2: user interruption, ESC-key hit (Assembler: Jump back to main)

//assembler: * @returns: TRUE=ok, FALSE=overrun the target position
*/
static uint8_t dg_add()
{
	do {
		// Wait for next 1kHz time interval
		clear_flag(7);
		do {
			if (Serial_available()) {
				if (Serial_read()==27) {
					return 2;
				}
			}
		} while (flag(7)==0);	// wait for 1kHz time interval
//	} while (flag(6)||flag(5));	// wait if torque limit did occur.
	} while (Flags & 0b0110000);	// keep waiting if torque limit did occur

	// Increase commanded point by current velocity
	if (direction == 0) {
		// move in positive direction
		cur_pos += velocity;
		if (cur_pos >> 8 >= target_pos) return 1;
	} else {
		// move in negative direction
		cur_pos -= velocity;
		if (cur_pos >> 8 <= target_pos) return 1;
	}

	BEGIN_CRITICAL
	CtPos = cur_pos >> 8;
	END_CRITICAL;

	return 0;	// target no reached yet
}
