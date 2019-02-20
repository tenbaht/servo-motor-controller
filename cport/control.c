#include <avr/io.h>

#include "csmc3.h"
#include "control.h"


static int16_t T2;	// current velocity

/**
 * Initialize servo system
 *
 * Zero out all status variables and turn off all LEDs.
 */
void init_servo(uint8_t mode)
{
	BEGIN_CRITICAL
	Mode = mode;
//	bzero(CtPos, &Mode-&CtPos);	// more efficient, but a little crude
	CtPos =
	CtSub =
	PvInt =
	PvPos =
	OvTmr = 0;

	Pos = 0;
	END_CRITICAL

	led_off(LED_ERROR);
	led_off(LED_TORQUE);
	led_off(LED_READY);
}


/**
 * postion control loop (mode 3)
 */
static inline int16_t tap_position()
{
	int24_t T0;	// position error

	BEGIN_CRITICAL
	T0 = CtPos - Pos;
	END_CRITICAL

	// clamp position error to LimSpd (P0)
	if (T0 >= LimSpd) T0 = LimSpd;
	else if (T0 <= -LimSpd) T0 = -LimSpd;

	return (int16_t) T0;
}

/**
 * velocity control loop (mode 2)
 *
 * flag 5: lower torque limit active
 * flag 6: lower torque limit active
 *
 * @parms:
 * T0:	desired velocity value. This value is expected to be already clamped
 *	to +/-LimSpd
 *
 * used global variables, side effects:
 * GaSpd (r), GaTqP (r), PvInt (rw), GaTqI (r), LimTrq (r), T2/speed (r)
 *
 * @returns: torque value
 */
static inline int16_t tap_velocity(int16_t T0)
{
	int16_t	Z;

	T0 -= T2 * GaSpd;		// error = desired_value - speed*P1
	Z = T0*GaTqP + PvInt*GaTqI;	// Z = Pval*error + Ival*Isum

	// torque limit (P4)
	clear_flag(5);
	clear_flag(6);
	if (Z >= LimTrq) {
		Z = LimTrq;
		set_flag(6);
	}
	if (Z <= -LimTrq) {
		Z = -LimTrq;
		set_flag(5);
	}

	// calculate integral part, with anti-windup
	if ( ((T0<0)&&!flag(5)) || ((T0>=0)&&!flag(6)) ) {
		PvInt += T0;
	}

	if (flags(5) || flags(6)) {
		led_on(LED_TORQUE);
		OvTmr += 3;
	} else {
		led_off(LED_TORQUE);
		if (OvTmr == 0) return (Z);
		OvTmr--;
	}
	if (OvTmr >= TL_TIME) {
		// trigger a servo error. Go back into save mode 0 (and make
		// servo_operation() skip tap_torque)
		init_servo(0);		// Enter mode 0
		led_on(LED_ERROR);
		return (0);
	}

	return (Z);
}


static inline int16_t tap_torque(int16_t T0)
{
	return (T0 + T2*GaEG);		// T0 + speed*P5
}


/**
 * output a PWM value (mode 0)
 *
 * Clip output voltage between -240 and +240. Limit minimum duty ratio to
 * 15/16 for bootstrap type FET driver.
 */
static inline void tap_voltage(int16_t T0)
{
	// Clip output voltage between -240 and +240.
	if (T0 >= 240) {
		T0=240;
	} else if (T0 <= -240) {
		T0=-240;
	}

	OCR1A = (T0/2) + 120;
	OCR1B = 120 - (T0/2);
}

void servo_operation()
{
	int16_t T0;	// input value to the module

	set_flag(7);		// 1kHz interrupt flag
	BEGIN_CRITICAL
	T2 = Pos - PvPos;	// calculate current velocity
	PvPos = Pos;		// update previous position
	END_CRITICAL

	T0 = CtSub;	// only needed for mode 0,1,2
/*
	switch (Mode) {
		case 3:	T0 = tap_position();
			// intentional fall-through
		case 2:	T0 = tap_velocity(T0);
			// intentional fall-through
		case 1:	// tap_velocity might have triggered a servo error
			// and reset the system into mode 0.
			if (Mode) {
				T0 = tap_torque(T0);
			}
			// intentional fall-through
		default: tap_voltage(T0);
	}
*/
	if (Mode>=3) T0 = tap_position();
	if (Mode>=2) T0 = tap_velocity(T0);
	if (Mode>=1) T0 = tap_torque(T0);
	tap_voltage(T0);
}
