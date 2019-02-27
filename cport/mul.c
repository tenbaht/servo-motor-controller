#include "mul.h"

/*
;--------------------------------------;
; 16bit * 16bit signed multiply
; 
; Multiplier:   A(signed int)
; Multiplicand: B(unsigned, 8.8 fraction)
; Result:       A(signed int)
; Clk:		181(max)
*/
int16_t muls1616(int16_t i16, uint16_t u8_8)
{
	uint8_t		DL;
	int16_t	C;	// temp var

	asm volatile(
"	clt"						"\n\t"
"	tst	%B[A]"					"\n\t"
"	brpl	1f"					"\n\t"
"	set"						"\n\t"
"	com	%B[A]"					"\n\t"
"	neg	%A[A]"					"\n\t"
"	brne	1f"					"\n\t"
"	inc	%B[A]"					"\n\t"
							"\n\t"
"1:	clr	%A[C]	; clear high 16bit."		"\n\t"
"	clr	%B[C]"					"\n\t"
"	ldi	%[DL], 17; DL = loop count"		"\n\t"
"2:	brcc	3f	; ---- calculating loop"	"\n\t"
"	add	%A[C], %A[A];"				"\n\t"
"	adc	%B[C], %B[A];"				"\n\t"
"3:	ror	%B[C]	;"				"\n\t"
"	ror	%A[C]	;"				"\n\t"
"	ror	%B[B]	;"				"\n\t"
"	ror	%A[B]	;"				"\n\t"
"	dec	%[DL]	; if (--DL > 0)"		"\n\t"
"	brne	2b	;  continue loop;"		"\n\t"

"	mov	%A[A], %B[B]"				"\n\t"
"	mov	%B[A], %A[C]"				"\n\t"

"	brtc	4f	; Negate the result if multiplier was negative."	"\n\t"
"	com	%B[A]"					"\n\t"
"	neg	%A[A]"					"\n\t"
"	brne	4f"					"\n\t"
"	inc	%B[A]"					"\n\t"
"4:"							"\n\t"
	""
	: [A] "+r" (i16), [C] "=&r" (C), [DL] "=&d" (DL), [B] "+r" (u8_8)	// output
	:						// input
	: 						// clobber
	);

	return i16;	// return value is in the same register as the first arg
}
