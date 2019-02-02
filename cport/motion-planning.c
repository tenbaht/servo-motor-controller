

#include "csmc3.h"
#include "cli.h"
#include "motion-planning.h"


/* --- private variables --------------------------------------------------- */

static s24_8	cur_pos;	// next planned position, u24.8
static s16_8	velocity;	// currently planned velocity (mostly unsigned)
static bool	direction;	// 0 = positive, 1 = negative

static s24	start_pos;	// start position of the move
static s24	target_pos;	// target position of the move


/* --- private functions --------------------------------------------------- */

static void dg_0()
static void dg_1()
static bool dg_add();


/* --- implementation of public functions ---------------------------------- */

/**
;------------------------------------------------;
; Go at trapezoidal/rectanguler velocity profile.
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
	cur_pos		= CtPos << 8;	// cur_pos is u24.8
	if (get_val()) {
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
 * Rectanguler velocity profile - G1 command
 *
 */
static void dg_1()
{
	if (!get_val()) goto cmd_err;	// FIXME
	target_pos = val;

	if (!get_val()) goto cmd_err;	// FIXME
	velocity = val;		// this is u16.8

	direction = (target_pos < start_pos);

	while (dg_add());	// loop until target pos is reached

	CtPos = target_pos;
}



/**
 * Trapezoidal velocity profile - G0 command
 *
 */
static void dg_0();
{
	if (!get_val()) goto cmd_err;	// FIXME
	target_pos = val;

	direction = (target_pos < start_pos);

	// calculate the midpoint of the movement
	next_mark = (target_pos - start_pos)/2 + start_pos;

	// ---Up ramp loop
	velocity = 0;
	do {
		velocity += MvAcc;
		if (!dg_add()) goto dg_end;


		// check if we already passed the midpoint.
		// if yes, go straight for the down ramp.
		if (direction==0) {
			if (cur_pos>>8 >= next_mark) goto dg_de;
		} else {
			if (cur_pos>>8 <= next_mark) goto dg_de;
		}
	} while (velocity < MvSpd);

	// the speed reached P6 now. Before we keep goint at this speed
	// calculate the down ramp point, assuming that the down ramp will
	// be the same width than the the up ramp:
	// up ramp width was (cur_pos-start_pos)
	// down ramp beginns at target - up_ramp_width = target - (cur-start)
	next_mark = start_pos - cur_pos>>8 + target_pos

	// ---Constant velocity loop
	for(;;) {
		dg_add();

		// check if it is time for the down ramp:
		if (direction==0) {
			if (cur_pos>>8 >= next_mark) break;
		} else {
			if (cur_pos>>8 <= next_mark) break;
		}
	}

	// ---Down ramp loop
dg_de:
	dg_add();
	// the following decrement is the reason why velocity is only
	// "mostly unsigned". We definitly need a signed comparision here.
	while (( (s16_8)velocity-=MvAcc) >= 0) {
		if (dg_add()==0) break;
	}

	// End of action
dg_end:
	BEGIN_CRITICAL;
	CtPos = target_pos;

	do {
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
}



/**
;
; do one 1ms step with the current velocity
;
; wait for 1kHz time interval without any torque flags set.
; advance cur_pos by (velocity)
; if that does not overrun the target position, advance CtPos by (velocity)
;
 * @returns: TRUE=ok, FALSE=overrun the target position
*/
static bool dg_add()
{
	do {
		// Wait for next 1kHz time interval
		clear_flag(7);
		do {
			if (Serial.available())
				if (Serial.read()==0x1b) return;
		} while (flag(7)==0);
	} while (flag & 0b0110000);	// keep waiting if torque limit did occur

	// Increase commanded point by current velocity
	if (direction == 0) {
		cur_pos += velocity;
		if (cur_pos >> 8 >= target_pos) return 0;
	} else {
		cur_pos -= velocity;
		if (cur_pos >> 8 <= target_pos) return 0;
	}

	BEGIN_CRITICAL
	CtPos = cur_pos >> 8;
	END_CRITICAL;

	return 1;
}
