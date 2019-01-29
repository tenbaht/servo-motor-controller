;----------------------------------------------------------------------------;
; AVR SERVOMOTOR CONTROLLER R0.3a                             (C)ChaN, 2005  ;
;----------------------------------------------------------------------------;
; SMC3A: Specified for standalone operation for stepper motor replacement


.include "tn2313def.inc"
.include "avr.inc"

.equ	INI_MODE = 0	;Initial Servo Mode (0-3) for no serial control

.equ	SYSCLK	= 16000000
.equ	BPS	= 38400


.def	_0	= r15	;Permanent zero register

.def	_PvEnc	= r14	;Previous encoder signal A/B
.def	_PvDir	= r13	;Previous direction
.def	_CtDiv	= r12	;1/83 divider

.def	_PosH	= r11	;Current position (circular counter)
.def	_PosL	= r10	;/
.def	_CmdH	= r9	;Commanded position (circular counter)
.def	_CmdL	= r8	;/
.def	_CmdF	= r7	;Gear fraction

.def	_Flags	= r25	; -|Sat.F|Sat.R|-|-|-|Echo


;----------------------------------------------------------;
; EEPROM Area

.eseg
	; Memory bank 0 : Yasukawa Electric UGTMEM-A1SA51
	.dw	300, 0x0500, 0x0300, 0x00c0, 240, 0x0340,  0x0100, 0

	; Memory bank 1 : Yasukawa Electric UGTMEM-A1SA51
	.dw	300, 0x0500, 0x0300, 0x00c0, 240, 0x0340,  0x0100, 0

	; Memory bank 2 : Tamagawa Seiki TS1410N1
	.dw	300, 0x0800, 0x0300, 0x0060, 180, 0x0550,  0x0100, 0

	; Memory bank 3 : Matsushita Electric MCN-14EAEC (6V, 40p/r)
	.dw	200, 0x0800, 0x0a00, 0x0400, 200, 0x0840,  0x0100, 0

.equ	N_PARM = 8	; Number of parameter words par bank.



;----------------------------------------------------------;
; Data memory area

.dseg
	.org	RAMTOP
; Servo / G command parameters
Parms:
LimSpd:	.byte	2	;P0,Velocity limit		Integer
GaSpd:	.byte	2	;P1,Velocity feedback gain	8.8 fixed point
GaTqP:	.byte	2	;P2,Proportional gain		8.8 fixed point
GaTqI:	.byte	2	;P3,Integral gain		8.8 fixed point
LimTrq:	.byte	2	;P4,Torque limit		Integer
GaEG:	.byte	2	;P5,EG feedback gain		8.8 fixed point
Gear:	.byte	2	;P6,Gear ratio for pulsed input	8.8 fixed point
	.byte	2	;P7

; Command/Servo registers
CtSub:	.byte	2	;Sub command   		s	mode 0/1/2
PvInt:	.byte	2	;Integration register
PvPos:	.byte	2	;Velocity detection register
OvTmr:	.byte	2	;Torque limit timer

Mode:	.byte	1	;Servo Mode		m

; Displacements referrd from RAMTOP
.equ	iLimSpd	= LimSpd-RAMTOP
.equ	iGaSpd	= GaSpd-RAMTOP
.equ	iGaTqP	= GaTqP-RAMTOP
.equ	iGaTqI	= GaTqI-RAMTOP
.equ	iLimTrq	= LimTrq-RAMTOP
.equ	iGaEG	= GaEG-RAMTOP
.equ	iGear	= Gear-RAMTOP
.equ	iCtSub	= CtSub-RAMTOP
.equ	iPvInt	= PvInt-RAMTOP
.equ	iPvPos	= PvPos-RAMTOP
.equ	iOvTmr	= OvTmr-RAMTOP
.equ	iMode	= Mode-RAMTOP


; Host command
RxBuf:	.byte	2+16	; Serial receive buffer (Rp, Wp, Buff[16])
LineBuf:.byte	20	; Command line input buffer



;----------------------------------------------------------;
; Program code

.cseg
	; Interrupt Vectors (ATtiny2313)
	rjmp	reset		;Reset
	rjmp	int0_isr	;INT0
	rjmp	0		;INT1
	rjmp	0		;TC1 CAPT
	rjmp	0		;TC1 COMPA
	rjmp	0		;TC1 overflow
	rjmp	0		;TC0 overflow
	rjmp	rxint		;USART0 Rx ready
	rjmp	0		;USART0 Tx UDRE
	rjmp	0		;USART0 Tx empty
	rjmp	0		;Analog comparator
	rjmp	0		;PCINT
	rjmp	0		;TC1 COMPB
	rjmp	background	;TC0 COMPA
;	rjmp	0		;TC0 COMPB
;	rjmp	0		;USI START
;	rjmp	0		;USI OVF
;	rjmp	0		;EEPROM
;	rjmp	0		;WDT


reset:
	outi	SPL, low(RAMEND)	;Stack ptr
	clr	_0			;Clear RAM
	ldiw	Y, RAMTOP		;
	st	Y+, _0			;
	cpi	YL, low(RAMTOP+128)	;
	brne	PC-2			;/

	outi	PORTD, 0b01111111	;Initialize PORTD
	outi	DDRD,  0b00000010	;/

	outi	PORTB, 0b11100000	;Initialize PORTB
	outi	DDRB,  0b00011111	;/

	ldiw	A, SYSCLK/16/BPS-1	;UART
	outw	UBRR, A			;
	outi	UCSRB, 0b10011000	;/

	ldiw	A, 128			;TC1: 8bit PWM mode
	outw	OCR1A, A		;
	outw	OCR1B, A		;
	outi	TCCR1A, 0b10100001	;
	outi	TCCR1B, 0b00000001	;/

	outi	OCR0A, 2		;TC0: 83kHz interval timer
	outi	TCCR0A, 0b00000010	;
	outi	TCCR0B, 0b00000011	;
	outi	TIMSK,  0b00000001	;/

	outi	MCUCR, 0b00000010	;INT0: Falling edge
	outi	GIMSK, 0b01000000	;/

	ldi	_Flags, 0b00000001	;Echo ON

	ldi	AL, 0			;Load servo parms form bank 0
	 rcall	load_parms		;/

	ldi	AL, INI_MODE		;Initial servo mode
	 rcall	init_servo		;/

	ldiw	Z, m_start*2		;Start up message
	 rcall	dp_str			;/


;----------------------------------------------------------;
; Command processing loop

main:
	ldiw	Z, m_prompt*2	;Display command prompt
	 rcall	dp_str		;/
	 rcall	get_line	;Get a command line
	ld	BH,X+		;BH = command char
	cpi	BH,'a'		;CAPS
	brcs	PC+2		;
	subi	BH,0x20		;/
	cpi	BH,' '		;Null line ?
	brlt	main		;
	cpi	BH,'+'		;+Step?
	rjeq	do_step		;
	cpi	BH,'-'		;-Step?
	rjeq	do_step		;
	cpi	BH, '?'		;Help?
	breq	do_help		;
	cpi	BH, 'M'		;Set mode?
	breq	do_mode		;
	cpi	BH, 'S'		;Set sub command?
	breq	do_sub		;
	cpi	BH, 'P'		;Set parms?
	breq	do_parm		;
	cpi	BH, 'E'		;Echo mode?
	breq	do_echo		;
	cpi	BH, 'R'		;Read parms?
	breq	do_reep		;
	cpi	BH, 'W'		;Write parms?
	breq	do_weep		;

cmd_err:ldiw	Z,m_error*2	;Syntax error
	rjmp	PC+3
do_help:ldiw	Z,m_help*2	;Help
	 rcall	dp_str
	rjmp	main


;------------------------------------------------;
; Change parameters, command regs or servo mode.

do_mode:	; Change servo mode
	 rcall	get_val
	breq	cmd_err
	 rcall	init_servo
	rjmp	main

do_echo:	; Change echo mode
	 rcall	get_val
	breq	cmd_err
	bst	AL, 0
	bld	_Flags,0
	rjmp	main

do_sub:		; Set subcommand reg.
	ldiw	Y, CtSub
	rjmp	ds_set

do_parm:	; Set parameters
	 rcall	get_val
	breq	cmd_err
	cpi	AL, N_PARM
	brcc	cmd_err
	lsl	AL
	mov	YL, AL
	clr	YH
	subiw	Y, -Parms
ds_set:	 rcall	get_val
	brcs	cmd_err
	brne	ds_st
	ldi	AL, 0x0a
	 rcall	xmit
	lddw	A, Y+0
	 rcall	dp_dec
	ldi	AL, ':'
	 rcall	xmit
	 rcall	get_line
	 rcall	get_val
	brcs	cmd_err
	breq	PC+5
ds_st:	cli
	stdw	Y+0, A
	sei
	rjmp	main



;------------------------------------------------;
; Load/Save parameters.

do_reep:	; Load parameters from EEPROM
	 rcall	get_val
	breq	cmd_err
	cpi	AL, (EEPROMEND+1)/N_PARM/2
	brcc	cmd_err
	cli
	 rcall	load_parms
	sei
	rjmp	main


do_weep:	; Save parameters into EEPROM
	 rcall	get_val
	breq	cmd_err
	cpi	AL, (EEPROMEND+1)/N_PARM/2
	brcc	cmd_err
	 rcall	get_eeadr
	sbic	EECR, EEWE
	rjmp	PC-1
	out	EEAR, BH
	inc	BH
	ld	AL, Y+
	out	EEDR, AL
	cli
	sbi	EECR, EEMWE
	sbi	EECR, EEWE
	sei
	dec	AH
	brne	PC-11
	rjmp	main


load_parms:
	 rcall	get_eeadr
	out	EEAR, BH
	inc	BH
	sbi	EECR, EERE
	in	AL, EEDR
	st	Y+, AL
	dec	AH
	brne	PC-6
	ret


get_eeadr:
	ldi	AH, N_PARM*2
	clr	BH
	subi	AL, 1
	brcs	PC+3
	add	BH, AH
	rjmp	PC-3
	ldiw	Y, Parms
	ret


;------------------------------------------------;
; Change position command reg. immediataly.

do_step:
	mov	DL, BH
	 rcall	get_val
	rjeq	cmd_err
	cli
	cpi	DL, '-'
	breq	PC+4
	addw	_Cmd, A
	rjmp	PC+3
	subw	_Cmd, A
	sei
	rjmp	main



;----------------------------------------------------------;
; Initialize servo system
;
;Call: AL = servo mode

init_servo:
	cli
	sts	Mode, AL	;Set servo mode
	ldiw	Y, CtSub	;Clear cmmand regs and servo operators.
	st	Y+, _0		;
	cpi	YL, low(CtSub+8);
	brne	PC-2		;/
	clrw	_Pos		;Clear position counter
	clrw	_Cmd		;Clear command counter
	clr	_CmdF		;/
	sei
	cbi	PORTB, 0	;Torque LED on
	cbi	PORTB, 1	;Error LED off
	sbi	PORTB, 2	;Ready LED on
	ret



;----------------------------------------------------------;
; Pulse drive interrupt

int0_isr:
	pushw	A			;Save flag/reg
	in	AH, SREG		;/

	sbis	PIND, 6			;Branch by Dir input
	rjmp	pi_rev

pi_fwd:					;Cmd += Gear
	lds	AL, Gear+0
	add	_CmdF, AL
	lds	AL, Gear+1
	adc	_CmdL, AL
	adc	_CmdH, _0
	rjmp	pi_exit

pi_rev:					;Cmd -= Gear
	lds	AL, Gear+0
	sub	_CmdF, AL
	lds	AL, Gear+1
	sbc	_CmdL, AL
	sbc	_CmdH, _0

pi_exit:
	out	SREG, AH		;Restore flag/reg
	popw	A			;/
	reti



;----------------------------------------------------------;
; 83kHz Position capture and servo operation interrupt

background:
	push	T0L
	pushw	Z
	in	T0L, SREG		;Save flags

	mov	ZL, _PvEnc		;ZL[1:0] = previous A/B signal
	in	_PvEnc, PIND		;Sample A/B signal into _PvEnc[1:0]
	swap	_PvEnc			;/
	ldi	ZH, 1			;Convert it to sequencial number.
	sbrc	_PvEnc, 1		;
	eor	_PvEnc, ZH		;/
	sub	ZL, _PvEnc		;Decode motion
	andi	ZL, 3			;/
	breq	enc_zr			;-> Not moved
	cpi	ZL, 3			;
	breq	enc_rev			;-> -1 count
	cpi	ZL, 1			;
	breq	enc_fwd			;-> +1 count
	mov	ZL, _PvDir		;-> Missing code recovery:
	mov	ZH, _PvDir		; double count for previous direction
	lsl	ZL			;
	asr	ZH			;/
	rjmp	enc_add
enc_rev:ldiw	Z, -1
	rjmp	PC+3
enc_fwd:ldiw	Z, 1
	mov	_PvDir, ZL
enc_add:addw	_Pos, Z
enc_zr:
	dec	_CtDiv		;Decrement 1/83 divider
	rjne	bgnd_exit	;If not overflow, exit interrupt routine.

; End of 83 kHz position captureing process. Follows are 1kHz servo
; operation. It will be interrupted itself, but will not be re-entered.

	ldi	ZL, 83		;Re-initialize 1/83 divider
	mov	_CtDiv, ZL	;/
	sei			;Enable interrupts
	pushw	T0
	pushw	T2
	pushw	A
	pushw	B
	pushw	C
	push	DL
	pushw	Y
	ldiw	Y, Parms	;Work area base pointer

	lddw	T2, Y+iPvPos	;Detect velocity
	cli			;
	subw	T2, _Pos	;
	stdw	Y+iPvPos, _Pos	;
	sei			;
	negw	T2		;/ T2 = velocity

	ldd	AL, Y+iMode	;Branch by servo mode
	cpi	AL, 3		;Mode3?
	breq	tap_position	;/
	lddw	T0, Y+iCtSub	;Get sub command
	cpi	AL, 2		;Mode2?
	breq	tap_velocity	;/
	cpi	AL, 1		;Mode1?
	rjeq	tap_torque	;/
	rjmp	tap_voltage	;Mode0


tap_position:
	cli			;Get position error (= velocity command)
	movw	T0L, _CmdL	;
	subw	T0, _Pos	;
	sei			;T0 = position error

	ldiw	A, 20000	;Check if position error is too much
	cpw	T0, A		;
	rjge	servo_error	;
	ldiw	A, -20000	;
	cpw	T0, A		;
	rjlt	servo_error	;/

	lddw	A, Y+iLimSpd	;Velocity limit (P0)
	cpw	T0, A		;
	brge	PC+8		;
	negw	A		;
	cpw	T0, A		;
	brge	PC+2		;
	movw	T0L, AL		;T0 = velocity command


tap_velocity:
	movw	AL, T2L		;Velocity loop gain (P1)
	lddw	B, Y+iGaSpd	;
	 rcall	muls1616	;B = scaled velocity
	subw	T0, B		;T0 = velocity error

	movw	AL, T0L		;Velocity error P-gain (P2)
	lddw	B, Y+iGaTqP	;
	 rcall	muls1616	;
	movw	ZL, BL		;Z = P term;

	lddw	A, Y+iPvInt	;Velocity error I-gain (P3)
	lddw	B, Y+iGaTqI	;
	 rcall	muls1616	;
	addw	Z, B		;Z += I term;

	cbr	_Flags, bit6+bit5;Torque limit (P4)
	lddw	B, Y+iLimTrq	;
	cpw	Z, B		;
	brlt	PC+3		;
	movw	ZL, BL		;
	sbr	_Flags, bit6	;
	neg	BL		;
	com	BH		;
	cpw	Z, B		;
	brge	PC+3		;
	movw	ZL, BL		;
	sbr	_Flags, bit5	;/

	tst	T0H		;PvInt += T0, with anti-windup
	brmi	PC+4		;
	sbrc	_Flags, 6	;
	rjmp	PC+10		;
	rjmp	PC+3		;
	sbrc	_Flags, 5	;
	rjmp	PC+7		;
	lddw	A, Y+iPvInt	;
	addw	A, T0		;
	stdw	Y+iPvInt, A	;/

	mov	AL, _Flags	;Check torque limiter timer
	andi	AL, bit6+bit5	; OvTmr is increased by 3 when torque limitter
	lddw	A, Y+iOvTmr	; works, decreased by 1 when no torque limit.
	breq	PC+5		; When the value reaches 3000, servo error
	sbi	PORTB, 1	; will beoccured.
	addiw	A, 3		;
	rjmp	PC+5		;
	cbi	PORTB, 1	;
	subiw	A, 1		;
	brcs	PC+8		;
	stdw	Y+iOvTmr, A	;
	ldiw	B, 3000		;
	cpw	A, B		;
	brcc	servo_error	;/

	movw	T0L, ZL		;T0 = torque command


tap_torque:
	movw	AL, T2L		;EG compensation (P5)
	lddw	B, Y+iGaEG	;
	 rcall	muls1616	;
	addw	T0, B		;T0 = voltage command


tap_voltage:
	ldiw	A, 240		;Clip output voltage between -240 and +240.
	cpw	T0, A		; Limit minimum duty ratio to 15/16 for bootstrap
	brge	PC+6		; type FET driver.
	ldiw	A, -240		;
	cpw	T0, A		;
	brge	PC+2		;
	movw	T0L, AL		;T0 = PWM command

	asrw	T0		;Set PWM register (OCR1A and ~OCR1B)
	ldi	AL, 120		;
	adc	AL, T0L		;
	ldi	AH, 120		;
	sub	AH, T0L		;
	out	OCR1AL, AL	;
	out	OCR1BL, AH	;/

	popw	Y
	pop	DL
	popw	C
	popw	B
	popw	A
	popw	T2
	popw	T0


bgnd_exit:			;End of encoder capture and servo operation
	out	SREG, T0L	;Restore flags
	popw	Z
	pop	T0L
	reti


servo_error:
	ldi	AL, 0		;Enter mode 0
	 rcall	init_servo	;/
	sbi	PORTB, 0	;Error LED on
	clrw	T0		;Output = 0V
	rjmp	tap_voltage



;--------------------------------------;
; 16bit * 16bit signed multiply
; 
; Multiplyer:   A(signed int)
; Multiplicand: B(unsigned, 8.8 fraction)
; Result:       B(signed int)
; Clk:		181(max)

muls1616:
	clt
	tst	AH
	brpl	PC+6
	set
	negw	A

	subw	C, C	; clear high 16bit.
	ldi	DL, 17	; DL = loop count
	brcc	PC+3	; ---- calcurating loop
	addw	C, A	;
	rorw	C	;
	rorw	B	;
	dec	DL	; if (--DL > 0)
	brne	PC-8	;  continue loop;

	mov	BL, BH
	mov	BH, CL

	brtc	PC+5	; Negate the result if multiplyer was negative.
	negw	B
	ret



;--------------------------------------;
; Input a command line into LineBuf.

get_line:
	ldiw	X,LineBuf
	ldi	BH,0
rl_lp:	 rcall	receive
	breq	PC-1
	st	X,AL
	cpi	AL,0x0d	; CR
	brne	PC+4
	ldiw	X,LineBuf
	rjmp	echo
	cpi	AL,0x08	; BS
	brne	PC+7
	cpi	BH,0
	breq	rl_lp
	 rcall	echo
	sbiw	XL,1
	dec	BH
	rjmp	rl_lp
	cpi	AL,' '		; SP
	brcs	rl_lp
	cpi	BH,20-1
	breq	rl_lp
	 rcall	echo
	adiw	XL,1
	inc	BH
	rjmp	rl_lp



;--------------------------------------;
; Send ROM string
;
; Call: Z = top of the string (ASCIZ)
; Ret:  Z = next string

dp_str:	lpm	AL, Z+
	tst	AL
	brne	PC+2
	ret
	 rcall	xmit
	rjmp	dp_str



;--------------------------------------;
; Get value of decimal string
;
; Call: X -> ASCII string
; Ret:  X = updated
;         if    C=1: error
;         elsif Z=1: end of line, value=0
;         else:      A = 16bit value
;
;  Positive:   "300"
;  Negative:   "-1250"

get_val:
	clt
	clrw	A
	ld	BH, X+
	cpi	BH, ' '
	brcs	gd_n
	breq	PC-3
	cpi	BH, '-'
	brne	PC+3
	set
gd_l:	ld	BH, X+
	cpi	BH, ' '+1
	brcs	gd_e
	subi	BH, '0'
	brcs	gd_x
	cpi	BH, 10
	brcc	gd_x
	ldi	CL, 17
	clr	BL
	lsr	BL
	rorw	A
	brcc	PC+2
	addi	BL, 10
	dec	CL
	brne	PC-6
	add	AL, BH
	adc	AH, _0
	rjmp	gd_l
gd_x:	sec
	sez
	ret
gd_e:	sbiw	XL, 1
	brtc	PC+5
	negw	A
	clc
	ret
gd_n:	sbiw	XL,1
	clc
	sez
	ret



;--------------------------------------;
; Display a value in decimal string
;
; Call: A = 16bit signed value to be displayed
; Ret:  A = broken

dp_dec:	ldi	CH,' '
	sbrs	AH, 7
	rjmp	PC+6
	negw	A
	ldi	CH,'-'
	clr	T0L		;digit counter
	inc	T0L		;---- decimal string generating loop
	clr	BL		;var1 /= 10;
	ldi	CL,16		;
	lslw	A		;
	rol	BL		;
	cpi	BL,10		;
	brcs	PC+3		;
	subi	BL,10		;
	inc	AL		;
	dec	CL		;
	brne	PC-8		;/
	addi	BL,'0'		;Push the remander (a decimal digit)
	push	BL		;/
	cp	AL,_0		;if(var1 =! 0)
	cpc	AH,_0		; continue digit loop;
	brne	PC-16		;/
	mov	AL, CH		;Sign
	 rcall	xmit		;/
	pop	AL		;Transmit decimal string
	 rcall	xmit		;<-- Put a char into console
	dec	T0L		;
	brne	PC-3		;/
	ret



;--------------------------------------;
; Serial I/O driver

	; Transmit AL.
echo:	sbrs	_Flags, 0
	ret
xmit:	sbis	UCSRA, UDRE
	rjmp	PC-1
	out	UDR, AL
	ret

receive:; Receive a char into AL. (ZR=no data)
	push	AH
	pushw	Y
	ldiw	Y, RxBuf
	cli
	ldd	AH, Y+0
	ldd	AL, Y+1
	cp	AH, AL
	breq	PC+8
	add	YL, AH
	ldd	AL, Y+2
	sub	YL, AH
	inc	AH
	andi	AH, 15
	std	Y+0, AH
	clz
	sei
	popw	Y
	pop	AH
	ret

rxint:	;USART0 Rx ready
	push	AL
	in	AL, SREG
	push	BL
	in	BL, UDR
	cbi	UCSRB, RXCIE
	sei
	pushw	A
	pushw	Y
	ldiw	Y, RxBuf
	ldd	AL, Y+0
	ldd	AH, Y+1
	inc	AH
	andi	AH, 15
	cp	AH, AL
	breq	PC+6
	std	Y+1, AH
	dec	AH
	andi	AH, 15
	add	YL, AH
	std	Y+2, BL
	popw	Y
	popw	A
	pop	BL
	out	SREG, AL
	pop	AL
	cli
	sbi	UCSRB, RXCIE
	reti



;----------------------------------------------------------;
; Strings

m_prompt:	.db	13,10, "%", 0
m_error:	.db	10, "???", 0
m_start:	.db	13,10, "SMC3A - Servo Motor Controller (?:help)", 13,10, 0
m_help:		.db	13,10, "m<mod> - servo mode",13,10, "e<mod> - echo mode",13,10, "s<val> - sub cmd",13,10, "{+|-}<stp> - step cmd",13,10, "p<par> [<val>] - examine/change parms",13,10, "w<bnk> - save parms",13,10, "r<bnk> - load parms", 0

