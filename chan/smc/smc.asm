;----------------------------------------------------------------------------;
; AVR SERVOMOTOR CONTROLLER R0.2                              (C)ChaN, 2004  ;
;----------------------------------------------------------------------------;
;

.include "2313def.inc" ; This file is included in "AVR Family Assembler".
.include "avr.inc"

.def	_0	= r15	;Permanent zero register

.def	_PvEnc	= r14	;Previous encoder data A:B
.def	_PvDir	= r13	;Previous direction
.def	_PosH	= r12	;Current position
.def	_PosM	= r11	;
.def	_PosL	= r10	;/

.def	_CtDiv	= r9	;1/52 divider
.def	_Mode	= r8	;Servo mode

.def	_Flags	= r25	; 1kHz|T.Lim| | | | |Echo


;----------------------------------------------------------;
; SRAM領域定義
; Data memory area

.dseg
	.org	RAMTOP

; Command/Servo registers
CtPos:	.byte	3	;Position 		g/j	mode 3
CtSub:	.byte	2	;Sub command   		s	mode 0/1/2
PvInt:	.byte	2	;Integration register
PvPos:	.byte	2	;Velocity detection register


; Servo / G command parameters
Parms:
LimSpd:	.byte	2	;P0,Velocity limit		Integer
GaSpd:	.byte	2	;P1,Velocity feedback gain	Fixed point
GaTqP:	.byte	2	;P2,Proportional gain		Fixed point
GaTqI:	.byte	2	;P3,Integral gain		Fixed point
LimTrq:	.byte	2	;P4,Torque limit		Integer
GaEG:	.byte	2	;P5,EG feedback gain		Fixed point
MvSpd:	.byte	2	;P6,G0 velocity			Integer
MvAcc:	.byte	2	;P7,G0 acceleration		Integer


; Host command
RxBuf:	.byte	2+16	; Serial receive buffer
LineBuf:.byte	20	; Command line input buffer


;----------------------------------------------------------;
; EEPROM定義

.eseg
	; Memory bank 0 : Tamagawa Seiki TS1410N1
	.dw	300, 0x0800, 0x0300, 0x0060, 180, 0x0550,  0x1a00, 32

	; Memory bank 1 : Tamagawa Seiki TS1410N1
	.dw	300, 0x0800, 0x0300, 0x0060, 180, 0x0550,  0x1a00, 32

	; Memory bank 2 : Yasukawa Electric UGTMEM-A1SA51
	.dw	300, 0x0500, 0x0180, 0x0060, 255, 0x0340,  0x2600, 128

	; Memory bank 3 : Matsushita Electric MCN-14EAEC (6V, 40p/r)
	.dw	200, 0x0800, 0x0300, 0x0080, 255, 0x0840,  0x1400, 64

.equ	N_PARM = 8	; Number of parameters par bank.



;----------------------------------------------------------;
; プログラムコード
; Program code

.cseg
	; Interrupt Vectors (AT90S2313)
	rjmp	reset		; Reset
	rjmp	int0_isr	; Extrenal INT0
	rjmp	0		; External INT1
	rjmp	0		; TC1 capture
	rjmp	0		; TC1 compare
	rjmp	0		; TC1 overflow
	rjmp	background	; TC0 overflow
	rjmp	rxint		; UART Rx ready
;	rjmp	0		; UART Tx ready
;	rjmp	0		; UART Tx empty
;	rjmp	0		; Analog comparator


reset:
	clr	_0
	outi	SPL,low(RAMEND)		;SP 初期化
	ldiw	Y, RAMTOP		;Clear RAM
	ldi	r16, 128		;
	st	Y+, _0			;
	dec	r16			;
	brne	PC-2			;/

	outi	PORTD, -1		;Initialize ports
	outi	DDRB, 0b00011100	;/

	outi	UBRR, 15		;UART: 38.4kbps @10MHz
	outi	UCR, 0b10011000		;/

	outi	TCCR1A, 0b10000001	;TC1 PWM mode
	outi	TCCR1B, 0b00000001	;
	out	OCR1AH, _0		;/

	outi	TCCR0, 0b011		; TC0.ck = CK/64
	outi	TIMSK, 0b00000010	; Enable TO0 Irq.

	outi	MCUCR, 0b00000010	;INT0 = fall edge
	outi	GIMSK, 0b01000000	;/

	ldi	_Flags, 0b00000001	; Echo ON

	clr	_Mode		;Servo mode = 0
	 rcall	init_servo	;/

	ldi	r16, 0		;Load servo parms form bank 0
	 rcall	read_parms	;/

;	ldi	r16,		;Set mode (for without serial I/F)
;	mov	_Mode, r16	;/

	sei

	ldiw	Z,m_start*2	; Display start up message
	 rcall	dp_str		; /



;----------------------------------------------------------;
; コマンド実行ループ
; Command execution loop

main:
	ldiw	Z,m_prompt*2	;Display command prompt
	 rcall	dp_str		;/
	 rcall	get_line	;Get a command line
	ld	r19,X+		;r19 = command char
	 rcall	caps		;/
	cpi	r19,' '		;Null line ?
	brlt	main		;
	cpi	r19,'J'		;Jump?
	rjeq	do_jump		;
	cpi	r19,'G'		;Go?
	rjeq	do_go		;
	cpi	r19,'L'		;Show posision counter?
	rjeq	do_loc		;
	cpi	r19, 'R'	;Read parms?
	rjeq	do_reep		;
	cpi	r19, 'W'	;Write parms?
	breq	do_weep		;
	cpi	r19, 'M'	;Set mode?
	breq	do_mode		;
	cpi	r19, 'S'	;Set sub command?
	breq	do_sub		;
	cpi	r19, 'P'	;Set parms?
	breq	do_parm		;
	cpi	r19, 'E'	;Echo mode?
	breq	do_echo		;

cmd_err:ldiw	Z,m_error*2	;Syntax error
	 rcall	dp_str		;/
	rjmp	main



;------------------------------------------------;
; Change parameters, command regs or servo mode.

do_mode:	; Change servo mode
	 rcall	get_val
	breq	cmd_err
	cli
	mov	_Mode, r16
	 rcall	init_servo
	sei
	rjmp	main

do_echo:	; Change echo mode
	 rcall	get_val
	breq	cmd_err
	bst	r16, 0
	bld	_Flags, 0
	rjmp	main

do_sub:	; Set subcommand reg.
	ldiw	Y, CtSub
	rjmp	ds_set

do_parm:	; Set parameters
	 rcall	get_val
	breq	cmd_err
	cpi	r16, N_PARM
	brcc	cmd_err
	lsl	r16
	mov	YL, r16
	clr	YH
	subi	YL, low(-LimSpd)
	sbci	YH, high(-LimSpd)
ds_set:	 rcall	get_val
	brcs	cmd_err
	brne	PC+15
	ldi	r16, 0x0a
	 rcall	xmit
	ldd	r16, Y+0
	ldd	r17, Y+1
	clr	r18
	sbrc	r17, 7
	dec	r18
	 rcall	dp_dec
	ldi	r16, ':'
	 rcall	xmit
	 rcall	get_line
	 rcall	get_val
	brcs	cmd_err
	breq	PC+5
	cli
	std	Y+0, r16
	std	Y+1, r17
	sei
	rjmp	main



;------------------------------------------------;
; Load/Save parameters.

do_weep:	; Save parameters into EEPROM
	 rcall	get_val
	breq	cmd_err
	cpi	r16, (E2END+1)/N_PARM/2
	brcc	cmd_err
	 rcall	write_parms
	rjmp	main

do_reep:	; Load parameters from EEPROM
	 rcall	get_val
	breq	cmd_err
	cpi	r16, (E2END+1)/N_PARM/2
	brcc	cmd_err
	 rcall	read_parms
	rjmp	main


write_parms:
	ldi	r17, N_PARM*2
	clr	r19
	subi	r16, 1
	brcs	PC+3
	add	r19, r17
	rjmp	PC-3
	ldiw	Y, Parms
	sbic	EECR, EEWE
	rjmp	PC-1
	out	EEAR, r19
	inc	r19
	ld	r16, Y+
	out	EEDR, r16
	cli
	sbi	EECR, EEMWE
	sbi	EECR, EEWE
	sei
	dec	r17
	brne	PC-11
	ret

read_parms:
	ldi	r17, N_PARM*2
	clr	r19
	subi	r16, 1
	brcs	PC+3
	add	r19, r17
	rjmp	PC-3
	ldiw	Y, Parms
	out	EEAR, r19
	inc	r19
	sbi	EECR, EERE
	in	r16, EEDR
	st	Y+, r16
	dec	r17
	brne	PC-6
	ret



;------------------------------------------------;
; Show location counter

do_loc:
	ldi	r16, 0x0a
	 rcall	xmit
dp_p:	ldi	r16, 0x0d
	 rcall	xmit
	cli
	mov	r16, _PosL
	mov	r17, _PosM
	mov	r18, _PosH
	sei
	push	r16
	 rcall	dp_dec
	ldi	r16, ' '
	 rcall	xmit
	pop	r20
	 rcall	receive
	rjne	main
	cp	r20, _PosL
	breq	PC-4
	rjmp	dp_p



;------------------------------------------------;
; Change position command reg. immediataly.

do_jump:
	 rcall	get_val
	rjeq	cmd_err
	ldiw	Z, CtPos
	cli
	std	Z+0, r16
	std	Z+1, r17
	std	Z+2, r18
	sei
	rjmp	main



;------------------------------------------------;
; Go at trapezoidal/rectanguler velocity profile.

do_go:
	ldiw	Z, CtPos	;Z -> Position command reg.
	ldd	r4, Z+0		;r6:r4 = start posision
	ldd	r5, Z+1		;
	ldd	r6, Z+2		;/
	mov	r1, r4		;r3:r0 = commanded posision
	mov	r2, r5		;
	mov	r3, r6		;/
	 rcall	get_val		;sub command
	rjeq	cmd_err		;/
	cpi	r16, 0		;G0?
	breq	dg_0		;/
	cpi	r16, 1		;G1?
	breq	dg_1		;/
	rjmp	cmd_err

; Working action (Constant velocity)
dg_1:	 rcall	get_val		;r18:r16 = target posision
	rjeq	cmd_err		;/
	push	r16
	push	r17
	push	r18
	 rcall	get_val		;r21:r19 = velocity
	mov	r19, r16	;
	mov	r20, r17	;
	mov	r21, _0		;/
	pop	r18
	pop	r17
	pop	r16
	rjeq	cmd_err		;/
	cp	r16, r4		;T = direction
	cpc	r17, r5		;
	cpc	r18, r6		;
	clt			;
	brge	PC+2		;
	set			;/
	clr	r0		;
	 rcall	dg_add		;---Constant velocity loop
	brlt	PC-1		;
	rjmp	dg_end		;/

; Quick positioning action (Trapezoidal profile)
dg_0:	 rcall	get_val		;r18:r16 = target posision
	rjeq	cmd_err		;/
	cp	r16, r4		;T = direction
	cpc	r17, r5		;
	cpc	r18, r6		;
	clt			;
	brge	PC+2		;
	set			;/
	clr	r0		;
	clr	r19		;r20:r19 = start velocity
	clr	r20		;
	clr	r21		;/

dg_ul:	lds	r22, MvAcc+0	;---Up ramp loop
	add	r19, r22	;Increace velocity
	lds	r22, MvAcc+1	;
	adc	r20, r22	;
	adc	r21, _0		;/
	 rcall	dg_add
	brge	dg_end
	mov	r22, r16	;Check current posisiton has passed
	mov	r23, r17	;half of distance.
	mov	r24, r18	;If passed, enter to down ramp.
	sub	r22, r4		;
	sbc	r23, r5		;
	sbc	r24, r6		;
	asr	r24		;
	ror	r23		;
	ror	r22		;
	add	r22, r4		;
	adc	r23, r5		;
	adc	r24, r6		;
	brts	PC+6		;
	cp	r1, r22		;
	cpc	r2, r23		;
	cpc	r3, r24		;
	brge	dg_de		;
	rjmp	PC+5		;
	cp	r22, r1		;
	cpc	r23, r2		;
	cpc	r24, r3		;
	brge	dg_de		;/
	lds	r22, MvSpd+0	;Has current velocity reached P6?
	lds	r23, MvSpd+1	;If reached, enter constant velocity mode.
	cp	r19, r22	;
	cpc	r20, r23	;
	cpc	r21, _0		;
	brcs	dg_ul		;/

	mov	r22, r4		;Calcurate down ramp point
	mov	r23, r5		;
	mov	r24, r6		;
	sub	r22, r1		;
	sbc	r23, r2		;
	sbc	r24, r3		;
	add	r22, r16	;
	adc	r23, r17	;
	adc	r24, r18	;/ r24:r22 = s.p. - c.p. + t.p.
dg_cl:	 rcall	dg_add		;---Constant velocity loop
	brts	PC+6
	cp	r1, r22
	cpc	r2, r23
	cpc	r3, r24
	brge	dg_de
	rjmp	dg_cl
	cp	r22, r1
	cpc	r23, r2
	cpc	r24, r3
	brlt	dg_cl

dg_de:	 rcall	dg_add
dg_dl:	lds	r22, MvAcc+0	;---Down ramp loop
	sub	r19, r22	;Decrese velocity
	lds	r22, MvAcc+1	;
	sbc	r20, r22	;
	sbc	r21, _0		;/
	brcs	dg_end
	 rcall	dg_add
	brlt	dg_dl

dg_end:				; End of action
	cli
	std	Z+0, r16
	std	Z+1, r17
	std	Z+2, r18
dge_lp:	cli			; Wait until position stabled
	cp	_PosL, r16
	cpc	_PosM, r17
	cpc	_PosH, r18
	sei
	breq	PC+5
	push	r16
	 rcall	receive
	pop	r16
	breq	dge_lp
	rjmp	main



dg_add:
	cbr	_Flags, bit7	;Wait for 1kHz time interval
	push	r16		;
	 rcall	receive		;
	pop	r16		;
	brne	dga_stop	;
	sbrs	_Flags, 7	;
	rjmp	PC-5		;/
	sbrc	_Flags, 6	;Skip if torque limit has being occured.
	rjmp	dg_add		;/
	brts	PC+10		;Increase commanded point by current velocity
	add	r0, r19		;
	adc	r1, r20		;
	adc	r2, r21		;
	adc	r3, r21		;
	cp	r1, r16		;
	cpc	r2, r17		;
	cpc	r3, r18		;
	brge	dga_ov		;
	rjmp	PC+9		;
	sub	r0, r19		;
	sbc	r1, r20		;
	sbc	r2, r21		;
	sbc	r3, r21		;
	cp	r16, r1		;
	cpc	r17, r2		;
	cpc	r18, r3		;
	brge	dga_ov		;/
	cli
	std	Z+0, r1
	std	Z+1, r2
	std	Z+2, r3
	sei
	ses
dga_ov:	ret

dga_stop:
	pop	r16
	pop	r16
	rjmp	main



;----------------------------------------------------------;
; 制御系初期化
; Initialize servo system

init_servo:
	ldiw	Y, CtPos	; Clear cmmand regs and servo operators.
	ldi	r16, 9
	st	Y+, _0
	dec	r16
	brne	PC-2
	clr	_PosL
	clr	_PosM
	clr	_PosH
	ret



;----------------------------------------------------------;
; Pulse/Dir input

int0_isr:
	push	r0
	in	r0, SREG		;Save flags
	push	ZL

	sec
	lds	ZL, CtPos+0
	sbic	PIND, 6
	rjmp	PC+13
	sbc	ZL, _0
	sts	CtPos+0, ZL
	lds	ZL, CtPos+1
	sbc	ZL, _0
	sts	CtPos+1, ZL
	lds	ZL, CtPos+2
	sbc	ZL, _0
	rjmp	PC+12
	adc	ZL, _0
	sts	CtPos+0, ZL
	lds	ZL, CtPos+1
	adc	ZL, _0
	sts	CtPos+1, ZL
	lds	ZL, CtPos+2
	adc	ZL, _0
	sts	CtPos+2, ZL

	pop	ZL
	out	SREG, r0
	pop	r0
	reti


;----------------------------------------------------------;
; 52kHz エンコーダ取り込み, 1kHzサーボ演算
; Position capture and servo operation interrupt

background:
	push	r0
	push	ZL
	push	ZH
	in	r0, SREG		;Save flags
	ldi	ZL, -3			;Restore clock divisor
	out	TCNT0, ZL		;/

	mov	ZL, _PvEnc		;ZL[1:0] = previous A:B signal
	in	_PvEnc, PIND		;Sample A:B signal into _PvEnc[1:0]
	swap	_PvEnc			;/
	ldi	ZH, 1			;Convert to sequencial number.
	sbrc	_PvEnc, 1		;
	eor	_PvEnc, ZH		;/
	sub	ZL, _PvEnc		;Decode motion
	andi	ZL, 3			;/
	breq	enc_zr			;->Not moved
	cpi	ZL, 1			;
	breq	enc_inc			;->Incremented
	cpi	ZL, 3			;
	breq	enc_dec			;->Decremented
	mov	ZL, _PvDir		;Missing code:
	mov	ZH, _PvDir		; Count by 2 for previous direction
	lsl	ZL			;
	asr	ZH			;/
	rjmp	enc_add
enc_dec:ldiw	Z, -1
	rjmp	PC+3
enc_inc:ldiw	Z, 1
	mov	_PvDir, ZL
enc_add:add	_PosL, ZL
	adc	_PosM, ZH
	adc	_PosH, ZH
enc_zr:

	dec	_CtDiv		;Decrement 1/52 divider
	rjne	tc0_eoi		;If not overflow, exit interrupt routine.
	ldi	ZL, 52		;Re-initialize 1/52 divider
	mov	_CtDiv, ZL	;/

; End of 52 kHz position captureing process. Follows are 1kHz servo
; operation (924clks). It is interrupted itself, but will not be re-entered. 

	sei			;Enable interrupts
	push	r0
	push	r1
	push	r2
	push	r3
	push	r16
	push	r17
	push	r18
	push	r19
	push	r20
	push	r21
	push	r22

	lds	r2, PvPos+0	;速度検出
	lds	r3, PvPos+1	;Detect velocity
	cli			;
	sub	r2, _PosL	;
	sbc	r3, _PosM	;
	sts	PvPos+0, _PosL	;
	sts	PvPos+1, _PosM	;
	sei			;
	com	r2		;
	com	r3		;
	adc	r2, _0		;
	adc	r3, _0		;/ r3:r2 = velocity

	mov	r16, _Mode	; Branch by servo mode
	cpi	r16, 3
	breq	tap_position
	lds	r0, CtSub+0
	lds	r1, CtSub+1
	mov	ZL, r0
	mov	ZH, r1
	mov	r19, r0
	mov	r20, r1
	cpi	r16, 2
	breq	tap_speed
	cpi	r16, 1
	rjeq	tap_torque
	rjmp	tap_voltage

tap_position:
	cli			;位置エラー(=要求速度)計算
	lds	r16, CtPos+0	;Position error
	lds	r17, CtPos+1	;
	lds	r18, CtPos+2	;
	sub	r16, _PosL	;
	sbc	r17, _PosM	;
	sbc	r18, _PosH	;
	sei			;/ r18:r16 = position error

	lds	r19, LimSpd+0	;速度リミット(P0)
	lds	r20, LimSpd+1	;Velocity limiter
	clr	r21		;
	cp	r16, r19	;
	cpc	r17, r20	;
	cpc	r18, r21	;
	brge	PC+11		;
	com	r19		;
	com	r20		;
	com	r21		;
	adc	r19, _0		;
	adc	r20, _0		;
	adc	r21, _0		;
	cp	r16, r19	;
	cpc	r17, r20	;
	cpc	r18, r21	;
	brge	PC+3		;
	mov	r16, r19	;
	mov	r17, r20	;
	mov	r0, r16		;
	mov	r1, r17		;/ r1:r0 = velocity command

tap_speed:
	mov	r16, r2		;速度ループゲイン(P1)
	mov	r17, r3		;Velocity loop gain
	lds	r18, GaSpd+0	;
	lds	r19, GaSpd+1	;
	 rcall	muls1616	;/ r20:r19 = scaled velocity

	sub	r0, r19		;速度エラー(=要求トルク)計算 
	sbc	r1, r20		;/ r1:r0 = velocity error

	mov	r16, r0		;速度エラー比例ゲイン(P2)
	mov	r17, r1		;Proportional gain
	lds	r18, GaTqP+0	;
	lds	r19, GaTqP+1	;
	 rcall	muls1616	;/
	mov	ZL, r19		;Z = proportional term;
	mov	ZH, r20		;/

	lds	r16, PvInt+0	;速度エラー積分ゲイン(P3)
	lds	r17, PvInt+1	;Integral gain
	add	r0, r16		;
	adc	r1, r17		;
	mov	r16, r0		;
	mov	r17, r1		;
	lds	r18, GaTqI+0	;
	lds	r19, GaTqI+1	;
	 rcall	muls1616	;/
	add	ZL, r19		;Z += Integral term;
	adc	ZH, r20		;/

	lds	r18, LimTrq+0	;トルク制限(P4)
	ldi	r19, 0		;Torque limiter
	cp	ZL, r18		;
	cpc	ZH, r19		;
	brge	PC+6		;
	neg	r18		;
	com	r19		;
	cp	ZL, r18		;
	cpc	ZH, r19		;
	brge	PC+5		;
	mov	ZL, r18		;
	mov	ZH, r19		;
	sbr	_Flags, bit6	;
	rjmp	PC+6		;---トルク飽和時は積分器ホールド
	sts	PvInt+0, r0	; Anti-wind-up.
	sts	PvInt+1, r1	;
	cbr	_Flags, bit6	;/

tap_torque:
	mov	r16, r2		;EG補償(P5)
	mov	r17, r3		;EG compasation
	lds	r18, GaEG+0	;
	lds	r19, GaEG+1	;
	 rcall	muls1616	;
	add	r19, ZL		;
	adc	r20, ZH		;/ r20:r19 = voltage command

tap_voltage:
	ldi	r16, low(255)	;電圧出力(±255でアンプ飽和)
	ldi	r17, high(255)	;Clip output voltage between -255 to +255.
	cp	r19, r16	;
	cpc	r20, r17	;
	brge	PC+6		;
	ldi	r16, low(-255)	;
	ldi	r17, high(-255)	;
	cp	r19, r16	;
	cpc	r20, r17	;
	brge	PC+3		;
	mov	r19, r16	;
	mov	r20, r17	;/ r20:r19
	tst	r20		;Motor output. Set PWM register and polarity
	brmi	PC+5		;
	cbi	PORTB, 4	;
	out	OCR1AL, r19	;
	sbi	PORTB, 2	;
	rjmp	PC+5		;
	cbi	PORTB, 2	;
	neg	r19		;
	out	OCR1AL, r19	;
	sbi	PORTB, 4	;/

	pop	r22
	pop	r21
	pop	r20
	pop	r19
	pop	r18
	pop	r17
	pop	r16
	pop	r3
	pop	r2
	pop	r1
	pop	r0
	sbr	_Flags, bit7	; 1kHz inerrupt flag


tc0_eoi:			;End of encoder capture and servo operation
	out	SREG, r0	;Restore flags
	pop	ZH
	pop	ZL
	pop	r0
	reti


;--------------------------------------;
; 16ビットかけ算
; 16bit * 16bit signed multiply
; 
; Multiplyer:   r17:r16(signed)
; Multiplicand: r19:r18(positive)
; Result:       r21:r18(signed)
; Clk:		183(max)

muls1616:
	clt
	tst	r17
	brpl	PC+6
	set
	com	r16
	com	r17
	adc	r16,_0
	adc	r17,_0

	sub	r20,r20	; clear high 16bit.
	sub	r21,r21	;
	ldi	r22,17	; r22 = loop count
	brcc	PC+3	; ---- calcurating loop
	add	r20,r16	;
	adc	r21,r17	;
	ror	r21	;
	ror	r20	;
	ror	r19	;
	ror	r18	;
	dec	r22	; if (--r22 > 0)
	brne	PC-8	;  continue loop;

	brtc	PC+9	; Negate the result if multiplyer was negative.
	com	r18
	com	r19
	com	r20
	com	r21
	adc	r18,_0
	adc	r19,_0
	adc	r20,_0
	adc	r21,_0
	ret



;--------------------------------------;
; ホストから 1行入力
; Input a command line into LineBuf.

get_line:
	ldiw	X,LineBuf
	ldi	r19,0
rl_lp:	 rcall	receive
	breq	PC-1
	st	X,r16
	cpi	r16,0x0d	; CR
	brne	PC+4
	ldiw	X,LineBuf
	rjmp	xmit2
	cpi	r16,0x08	; BS
	brne	PC+7
	cpi	r19,0
	breq	rl_lp
	 rcall	xmit2
	sbiw	XL,1
	dec	r19
	rjmp	rl_lp
	cpi	r16,' '		; SP
	brcs	rl_lp
	cpi	r19,20-1
	breq	rl_lp
	 rcall	xmit2
	adiw	XL,1
	inc	r19
	rjmp	rl_lp

caps:	cpi	r19,'a'		; CAPS
	brcs	PC+2		; 
	subi	r19,0x20	; /
	ret


;--------------------------------------;
; ROMの文字列を送信
; Send ROM string
;
; Call: Z = top of the string (ASCIZ)
; Ret:  Z = next string

dp_str:	lpm
	adiw	ZL,1
	tst	r0
	brne	PC+2
	ret
	mov	r16,r0
	 rcall	xmit
	rjmp	PC-7


;--------------------------------------;
; 10進数文字列の値を取得
; Get value of decimal string
;
; Call: X -> ASCII string
; Ret:  X = updated
;         if    C=1: error
;         elsif Z=1: end of line, value=0
;         else:      r18:r16 = 24bit value
;
;  Positive:   "300"
;  Negative:   "-125000"

get_val:
	clt
	clr	r16
	clr	r17
	clr	r18
	ld	r19,X+
	cpi	r19,' '
	brcs	gd_n
	breq	PC-3
	cpi	r19,'-'
	brne	PC+3
	set
gd_l:	ld	r19,X+
	cpi	r19,' '+1
	brcs	gd_e
	subi	r19,'0'
	brcs	gd_x
	cpi	r19,10
	brcc	gd_x
	ldi	r20, 25
	ldi	r21, 10
	sub	r0, r0
	lsr	r0
	ror	r18
	ror	r17
	ror	r16
	brcc	PC+2
	add	r0, r21
	dec	r20
	brne	PC-7
	add	r16, r19
	adc	r17, _0
	adc	r18, _0
	rjmp	gd_l
gd_x:	sec
	sez
	ret
gd_e:	sbiw	XL,1
	brtc	PC+7
	com	r16
	com	r17
	com	r18
	subi	r16,-1
	sbci	r17,-1
	sbci	r18,-1
	clc
	ret
gd_n:	sbiw	XL,1
	clc
	sez
	ret



;--------------------------------------;
; 値を10進文字列で表示
; Display a value in decimal string
;
; Call: r18:r16 = 24bit signed value to be displayed
; Ret:  r18:r16 = broken

dp_dec:	ldi	r21,' '
	sbrs	r18, 7
	rjmp	PC+8
	com	r16
	com	r17
	com	r18
	subi	r16,-1
	sbci	r17,-1
	sbci	r18,-1
	ldi	r21,'-'
	clr	r0		;digit counter
	inc	r0		;---- decimal string generating loop
	clr	r19		;var1 /= 10;
	ldi	r20,24		;
	lsl	r16		;
	rol	r17		;
	rol	r18		;
	rol	r19		;
	cpi	r19,10		;
	brcs	PC+3		;
	subi	r19,10		;
	inc	r16		;
	dec	r20		;
	brne	PC-9		;/
	addi	r19,'0'		;Push the remander (a decimal digit)
	push	r19		;/
	cp	r16,_0		;if(var1 =! 0)
	cpc	r17,_0		; continue digit loop;
	cpc	r18,_0		;
	brne	PC-18		;/
	mov	r16, r21	;Sign
	 rcall	xmit		;/
	pop	r16		;Transmit decimal string
	 rcall	xmit		;<-- Put a char to memory, console or any other display device
	dec	r0		;
	brne	PC-3		;/
	ret


;--------------------------------------;
; Serial I/O driver

	; Transmit r16.
xmit2:	sbrs	_Flags, 0
	rjmp	PC+4
xmit:	sbis	USR, UDRE
	rjmp	PC-1
	out	UDR, r16
	ret

receive:; Receive a char into r16. (ZR=no data)
	push	r17
	push	YL
	push	YH
	ldiw	Y, RxBuf
	cli
	ldd	r17, Y+0
	ldd	r16, Y+1
	cp	r17, r16
	breq	PC+8
	add	YL, r17
	ldd	r16, Y+2
	sub	YL, r17
	inc	r17
	andi	r17, 15
	std	Y+0, r17
	clz
	sei
	pop	YH
	pop	YL
	pop	r17
	ret

rxint:	; 受信割り込み
	push	r16
	in	r16, SREG
	push	r16
	in	r16, UDR
	cbi	UCR, RXCIE
	sei
	push	r17
	push	r18
	push	YL
	push	YH
	ldiw	Y, RxBuf
	ldd	r17, Y+0
	ldd	r18, Y+1
	inc	r18
	andi	r18, 15
	cp	r18, r17
	breq	PC+6
	std	Y+1, r18
	dec	r18
	andi	r18, 15
	add	YL, r18
	std	Y+2, r16
	pop	YH
	pop	YL
	pop	r18
	pop	r17
	pop	r16
	out	SREG, r16
	pop	r16
	cli
	sbi	UCR, RXCIE
	reti



;----------------------------------------------------------;
; ROM constants

.equ	_PC	= PC

		.org	_PC+0x00
m_prompt:	.db	0x0d, 0x0a, "%", 0

		.org	_PC+0x02
m_error:	.db	0x0a, "???", 0

		.org	_PC+0x05
m_start:	.db	0x0d, "AVR SERVOMOTOR CONTROLLER  R0.3", 0x0d, 0x0a, 0

	