;----------------------------------------------------------------------------;
; AVR SERVOMOTOR CONTROLLER R0.3                              (C)ChaN, 2004  ;
;----------------------------------------------------------------------------;
;

.nolist
.include "m328Pdef.inc"
.include "avr.inc"
.list


.equ	SYSCLK	= 16000000	;System clock
.equ	BPS	= 38400		;UART bps
.equ	TL_TIME = 1500		;Error timer (tError(ms)=TL_TIME/3)

.equ	USE_DECAY = 1	; let the I value decay after reaching a steady state

.def	_0	= r15	;Permanent zero register
.def	_PvEnc	= r14	;Previous encoder signal A/B
.def	_PvDir	= r13	;Previous direction
.def	_PosX	= r12	;Current position
.def	_PosH	= r11	;
.def	_PosL	= r10	;/

.def	_CtDiv	= r9	;1/83 divider

.def	_Flags	= r25	; 1kHz|Sat.F|Sat.R| | | |


;----------------------------------------------------------;
; compatibility definitions for devices with different register names

; eeprom: bigger devices have 16 bit offset register
.ifndef EEAR
.equ	EEAR	= EEARL
.endif

; uart: use UART0
.ifndef UDR
.equ	UBRRL	= UBRR0L
.equ	UBRRH	= UBRR0H
.equ	UCSRA	= UCSR0A
.equ	UCSRB	= UCSR0B
.equ	UDR	= UDR0
.equ	UDRE	= UDRE0
.equ	RXCIE	= RXCIE0
.endif

; timer
.ifndef TIMSK
.equ	TIMSK	= TIMSK0
.endif


;----------------------------------------------------------;
; define LED pin mappings for different devices

.if (__DEVICE__ == __ATtiny2313__) || (__DEVICE__ == __ATtiny2313A__)
.equ	LED_ERROR_BIT	= 0
.equ	LED_ERROR_PORT	= PORTB
.equ	LED_TORQUE_BIT	= 1
.equ	LED_TORQUE_PORT	= PORTB
.equ	LED_READY_BIT	= 2
.equ	LED_READY_PORT	= PORTB

.elif (__DEVICE__ == __ATmega328P__)
.equ	LED_ERROR_BIT	= 0
.equ	LED_ERROR_PORT	= PORTB
.equ	LED_TORQUE_BIT	= 3
.equ	LED_TORQUE_PORT	= PORTC
.equ	LED_READY_BIT	= 2
.equ	LED_READY_PORT	= PORTC

.else
.error "no LED pin definition found for this device"

.endif

.macro	led_on
	sbi	@0_PORT, @0_BIT	; LED on
.endm

.macro	led_off
	cbi	@0_PORT, @0_BIT	; LED off
.endm



;----------------------------------------------------------;
; EEPROM Area

.eseg
	; Memory bank 0 : Yasukawa Electric UGTMEM-A1SA51
	.dw	300, 0x0500, 0x0300, 0x00c0, 240, 0x0340,  0x2600, 128

	; Memory bank 1 : Yasukawa Electric UGTMEM-A1SA51
	.dw	300, 0x0500, 0x0300, 0x00c0, 240, 0x0340,  0x2600, 128

	; Memory bank 2 : Tamagawa Seiki TS1410N1
	.dw	300, 0x0800, 0x0300, 0x0060, 180, 0x0550,  0x1a00, 32

	; Memory bank 3 : Matsushita Electric MCN-14EAEC (6V, 40p/r)
	.dw	200, 0x0800, 0x0a00, 0x0400, 200, 0x0840,  0x1400, 64

.equ	N_PARM = 8	; Number of parameter words per bank.



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
MvSpd:	.byte	2	;P6,G0 velocity			Integer
MvAcc:	.byte	2	;P7,G0 acceleration		Integer

; Command/Servo registers
CtPos:	.byte	3	;Position 		g/j	mode 3
CtSub:	.byte	2	;Sub command   		s	mode 0/1/2
PvInt:	.byte	2	;Integration register
PvPos:	.byte	2	;Velocity detection register
OvTmr:	.byte	2	;Torque limit timer

Mode:	.byte	1	;Servo Mode		m

; Display buffer
Disp:	.byte	1+8	;Display buffer, pointer


; Displacements referrd from RAMTOP
.equ	iLimSpd	= LimSpd-Parms
.equ	iGaSpd	= GaSpd-Parms
.equ	iGaTqP	= GaTqP-Parms
.equ	iGaTqI	= GaTqI-Parms
.equ	iLimTrq	= LimTrq-Parms
.equ	iGaEG	= GaEG-Parms
.equ	iMvSpd	= MvSpd-Parms
.equ	iMvAcc	= MvAcc-Parms
.equ	iCtPos	= CtPos-Parms
.equ	iCtSub	= CtSub-Parms
.equ	iPvInt	= PvInt-Parms
.equ	iPvPos	= PvPos-Parms
.equ	iOvTmr	= OvTmr-Parms
.equ	iMode	= Mode-Parms


; Host command
RxBuf:	.byte	2+16	; Serial receive buffer (Rp, Wp, Buff[16])
LineBuf:.byte	20	; Command line input buffer



;----------------------------------------------------------;
; Program code

.cseg
.org 0

.if (__DEVICE__ == __ATtiny2313__) || (__DEVICE__ == __ATtiny2313A__)
	; Interrupt Vectors (ATtiny2313)
	rjmp	reset		;Reset
	rjmp	0		;INT0
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


.elif __FLASH_SIZE__ <= 0x1000
	; Interrupt Vectors (for all AVR with up to 4kB flash, use rjmp)
	rjmp	reset		;Reset
.org OC0Aaddr
	rjmp	background	;TC0 COMPA
.org URXCaddr
	rjmp	rxint		;USART0 Rx ready
.org INT_VECTORS_SIZE
				; leave the rest of the table empty

.else
	; Interrupt Vectors (for all AVR with more than 4kB flash, use jmp)
	jmp	reset		;Reset
.org OC0Aaddr
	jmp	background	;TC0 COMPA
.org URXCaddr
	jmp	rxint		;USART0 Rx ready
.org INT_VECTORS_SIZE
				; leave the rest of the table empty

.endif


reset:
	outi	SPL, low(RAMEND)	;Stask ptr
	clr	_0			;Clear RAM
	ldiw	Y, RAMTOP		;
b001:	st	Y+, _0			;
	cpi	YL, low(RAMTOP+128)	;
	brne	b001			;/

	outi	PORTD, 0b01111111	;Initialize PORTD
	outi	DDRD,  0b00000010	;/

.if (__DEVICE__ == __ATtiny2313__) || (__DEVICE__ == __ATtiny2313A__)
	outi	PORTB, 0b10000000	;Initialize PORTB
	outi	DDRB,  0b11111111	;/
.elif (__DEVICE__ == __ATmega328P__)
	outi	PORTB, 0b00100000	;Initialize PORTB
	outi	DDRB,  0b11111111	;/

	out	PORTC, _0		;Initialize PORTC
	outi	DDRC,  0b00001100	;/
.endif

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
	outi	TIMSK, (1<<OCIE0A)	;/
.ifdef USICR
	outi	USICR, 0b00011100	;USI: LED display
.else
	;FIXME: initialize 3-wire SPI instead
.endif

	ldi	_Flags, 0

	ldi	AL, 0			;Load servo parms form bank 0
	 rcall	load_parms		;/

	ldi	AL, 0			;Initial servo mode = 0
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
	brcs	b02		;
	subi	BH,0x20		;/
b02:	cpi	BH,' '		;Null line ?
	brlt	main		;
	cpi	BH,'J'		;Jump?
	rjeq	do_jump		;
	cpi	BH,'G'		;Go?
	rjeq	do_go		;
	cpi	BH,'L'		;Show posision counter?
	rjeq	do_loc		;
	cpi	BH, 'R'		;Read parms?
	rjeq	do_reep		;
	cpi	BH, 'W'		;Write parms?
	rjeq	do_weep		;
	cpi	BH, 'M'		;Set mode?
	breq	do_mode		;
	cpi	BH, 'S'		;Set sub command?
	breq	do_sub		;
	cpi	BH, 'P'		;Set parms?
	breq	do_parm		;

cmd_err:ldiw	Z,m_error*2	;Syntax error
	 rcall	dp_str
	rjmp	main


;------------------------------------------------;
; Change parameters, command regs or servo mode.

do_mode:	; Change servo mode
	 rcall	get_val
	breq	cmd_err
	 rcall	init_servo
	rjmp	main

do_sub:	; Set subcommand reg.
	ldiw	Y, CtSub
	rjmp	ds_set

do_parm:	; Set parameters
; if (get_val()) goto cmd_err;
	 rcall	get_val
	breq	cmd_err
; if (val >= N_PARM) goto cmd_err;
	cpi	AL, N_PARM
	brcc	cmd_err
; Y = &Parms[val];
	lsl	AL
	mov	YL, AL
	clr	YH
	subiw	Y, -Parms
; c = get_val();
ds_set:	 rcall	get_val
; if (c == 2) goto cmd_err;
	brcs	cmd_err
; if (c == 1) {
	brne	ds_st
	ldi	AL, 0x0a
	 rcall	xmit
	lddw	A, Y+0
	clr	BL
	sbrc	AH, 7
	dec	BL
	 rcall	dp_dec
	ldi	AL, ':'
	 rcall	xmit
	 rcall	get_line
	 rcall	get_val
	brcs	cmd_err
	breq	b03
; }
; *Y = A;
ds_st:	cli
	stdw	Y+0, A
	sei
b03:	rjmp	main



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
b04:	sbic	EECR, EEPE
	rjmp	b04
	out	EEAR, BH
	inc	BH
	ld	AL, Y+
	out	EEDR, AL
	cli
	sbi	EECR, EEMPE
	sbi	EECR, EEPE
	sei
	dec	AH
	brne	b04
	rjmp	main


load_parms:
	 rcall	get_eeadr
b05:	out	EEAR, BH
	inc	BH
	sbi	EECR, EERE
	in	AL, EEDR
	st	Y+, AL
	dec	AH
	brne	b05
	ret


get_eeadr:
	ldi	AH, N_PARM*2
	clr	BH
b06:	subi	AL, 1
	brcs	b07
	add	BH, AH
	rjmp	b06
b07:	ldiw	Y, Parms
	ret


;------------------------------------------------;
; Show location counter

do_loc:
	ldi	AL, 0x0a
	 rcall	xmit
dp_p:	ldi	AL, 0x0d	;Show position counter
	 rcall	xmit		;
	cli			;
	movw	AL, _PosL	;
	mov	BL, _PosX	;
	sei			;
	mov	T0H, AL		;
	 rcall	dp_dec		;
	ldi	AL, ' '		;
	 rcall	xmit		;/
b08:	 rcall	receive		;Break if any key was pressed
	rjne	main		;/
	cp	T0H, _PosL	;Continue if not changed
	breq	b08		;/
	rjmp	dp_p



;------------------------------------------------;
; Change position command reg. immediataly.

; if (!get_val()) goto cmd_err;
; CtPos = val;
; goto main;
do_jump:
	 rcall	get_val
	rjeq	cmd_err
	ldiw	Z, CtPos	;Set position command reg.
	cli			;
	stdw	Z+0, A		;
	std	Z+2, BL		;
	sei			;/
	rjmp	main



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
do_go:
	ldiw	Z, CtPos	;Z -> Position command reg.
	lddw	T4, Z+0		;T6L:T4 = start position
	ldd	T6L, Z+2	;/
	mov	T0H, T4L	;r3:r0 = commanded position
	mov	T2L, T4H	;
	mov	T2H, T6L	;/
	 rcall	get_val		;sub command
	rjeq	cmd_err		;/
	cpi	AL, 0		;G0?
	breq	dg_0		;/
	cpi	AL, 1		;G1?
	breq	dg_1		;/
	rjmp	cmd_err

; Rectangular velocity profile
dg_1:	 rcall	get_val		;BL:AL = target position
	rjeq	cmd_err		;/
	pushw	A
	push	BL
	 rcall	get_val		;CH:BH = velocity
	mov	BH, AL	;
	mov	CL, AH	;
	mov	CH, _0		;/
	pop	BL
	popw	A
	rjeq	cmd_err		;/
	cpw	A, T4		;T = direction
	cpc	BL, T6L		;
	clt			;
	brge	b09		;
	set			;/
b09:	clr	r0		;
b010:	 rcall	dg_add		;---Constant velocity loop
	brlt	b010		;
	rjmp	dg_end		;/

; Trapezoidal velocity profile
; if (!get_val()) goto cmd_err;
dg_0:	 rcall	get_val		;BL:A = target posision
	rjeq	cmd_err		;/
; Tflag = (val<start_pos)//(val<T4:T6L)	// Tflag = direction of movement
; // 0 = forward (positive), 1 = backward (negative)
	cpw	A, T4		;T = direction
	cpc	BL, T6L		;
	clt			;
	brge	b011		;
	set			;/
; // BH:C is start velocity (24 bit)
; velocity = 0;
b011:	clr	r0		;
	clr	BH		;CL:BH = start velocity
	clrw	C		;/

; do {
;   velocity += MvAcc;
dg_ul:	lds	DL, MvAcc+0	;---Up ramp loop
	add	BH, DL		;Increase velocity
	lds	DL, MvAcc+1	;
	adc	CL, DL		;
	adc	CH, _0		;/
;   if (!dg_add()) goto dg_end;
	 rcall	dg_add
	brge	dg_end
;   // A:BL is target position
;   // T4:T6L is start position
;   D:EL = (target - start)/2 + start;
	movw	DL, AL		;Check current position has passed half of distance.
	mov	EL, BL		;If passed, enter to down ramp.
	sub	DL, T4L		;
	sbc	DH, T4H		;
	sbc	EL, T6L		;
	asr	EL		;
	rorw	D		;
	addw	D, T4		;
	adc	EL, T6L		;

;   if (direction) {	// if (Tflag) {
	brts	b40		;
;     if (cur_pos >= midpoint) goto dg_de;  //if (T0H:T2 >= EL:D) goto dg_de
	cp	T0H, DL		;
	cpc	T2L, DH		;
	cpc	T2H, EL		;
	brge	dg_de		;
	rjmp	b41		;
;   } else {
;     if (cur_pos <= midpoint) goto dg_de;  //if (T0H:T2 >= EL:D) goto dg_de
b40:	cp	DL, T0H		;
	cpc	DH, T2L		;
	cpc	EL, T2H		;
	brge	dg_de		;/
;   }
; } while (velocity < MvSpd);
b41:	ldsw	D, MvSpd	;Has current velocity reached P6?
	cp	BH, DL		;If reached, enter constant velocity mode.
	cpc	CL, DH		;
	cpc	CH, _0		;
	brlo	dg_ul		;/

; // Calculate down ramp point:
; // up ramp width was (cur_pos-start_pos)
; // down ramp beginns at target - up_ramp_width = target - (cur-start)
; EL:D ramp_point = start_pos - cur_pos + target_pos
	movw	DL, T4L		;Calculate down ramp point
	mov	EL, T6L		;
	sub	DL, T0H		;
	sbc	DH, T2L		;
	sbc	EL, T2H		;
	addw	D, A		;
	adc	EL, BL		;/ EL:DL = s.p. - c.p. + t.p.
; do {
;   dg_add();
dg_cl:	 rcall	dg_add		;---Constant velocity loop
;   if (direction==0) {		//Tflag==0) {
	brts	b42
;     if (cur_pos >= ramp_point) break; {// if (T0H:T2 >= EL:D) break;
	cp	T0H, DL
	cpc	T2L, DH
	cpc	T2H, EL
	brge	dg_de
	rjmp	dg_cl
;   } else {
;     if (ramp_point >= cur_pos) break; {// if (EL:D < T0H:T2) goto dg_cl
b42:	cp	DL, T0H
	cpc	DH, T2L
	cpc	EL, T2H
	brlt	dg_cl
; } // do
; dg_add();
dg_de:	 rcall	dg_add
; while ((velocity-=MvAcc) >= 0) {
;   if (dg_add()==0) break;
; }
dg_dl:	lds	DL, MvAcc+0	;---Down ramp loop
	sub	BH, DL		;Decrese velocity
	lds	DL, MvAcc+1	;
	sbc	CL, DL		;
	sbc	CH, _0		;/
	brcs	dg_end
	 rcall	dg_add
	brlt	dg_dl

; block_irq();
; CtPos = target_pos;
dg_end:				; End of action
	cli
	stdw	Z+0, A
	std	Z+2, BL
; while (block_irq(), Pos!=target_pos) {
;   allow_irq();
;   if (receive()

; // irq blocking is not exactly right in the C translation
; do {
;   block_irq();
;   if (cur_pos == target_pos) break;
;   allow_irq();
; } while (receive()==0);
dge_lp:	cli			; Wait until position stabled
	cpw	_Pos, A
	cpc	_PosX, BL
	sei
	breq	b43
	push	AL
	 rcall	receive
	pop	AL
	breq	dge_lp
; goto main;
b43:	rjmp	main


;
; do one 1ms step with the current velocity
;
; wait for 1kHz time interval without any torque flags set.
; advance cur_pos by (velocity)
; if that does not overrun the target position, advance CtPos by (velocity)
;
; returns: s-flag=1, N^V=1: all ok
;          N^V=0: overun the target position (brge will catch this)
dg_add:
	cbr	_Flags, bit7	;Wait for 1kHz time interval
b44:	push	AL		;
	 rcall	receive		; Break on ESC received
	breq	b45		;
	cpi	AL, 0x1B	;
	breq	dga_stop	; /
b45:	pop	AL		;
	sbrs	_Flags, 7	;
	rjmp	b44		;/
	sbrc	_Flags, 6	;Skip if torque limit did occur.
	rjmp	dg_add		;
	sbrc	_Flags, 5	;
	rjmp	dg_add		;/
; if (direction == 0) {		// if (Tflag==0) {
;   cur_pos += velocity;	//T2:T0 += BH:C
	brts	b46		;Increase commanded point by current velocity
	add	T0L, BH		;
	adc	T0H, CL		;
	adc	T2L, CH		;
	adc	T2H, CH		;
;   if (cur_pos >= target) return 0;
	cp	T0H, AL		;
	cpc	T2L, AH		;
	cpc	T2H, BL		;
	brge	dga_ov		;
	rjmp	b47		;
; } else {
;   cur_pos -= velocity;	//T2:T0 -= C:BH
b46:	sub	T0L, BH		;
	sbc	T0H, CL		;
	sbc	T2L, CH		;
	sbc	T2H, CH		;
;   if (cur_pos <= target) return 0;
	cp	AL, T0H		;
	cpc	AH, T2L		;
	cpc	BL, T2H		;
	brge	dga_ov		;/
; }
; BEGIN_CRITICAL
; CtPos = cur_pos >> 8;
; END_CRITICAL;
b47:	cli
	std	Z+0, T0H
	stdw	Z+1, T2
	sei
; return (1);
	ses
dga_ov:	ret
; }

dga_stop:
	pop	AL
	pop	AL
	pop	AL
	rjmp	main



;----------------------------------------------------------;
; Initialize servo system
;
;Call: AL = servo mode

init_servo:
	cli
	sts	Mode, AL	;Set servo mode
	ldiw	Y, CtPos	;Clear cmmand regs and servo operators.
b012:	st	Y+, _0		;
	cpi	YL, low(CtPos+11);
	brne	b012		;/
	clrw	_Pos		;Clear position counter
	clr	_PosX		;/
	sei
	led_off	LED_ERROR	;Error LED off
	led_off	LED_TORQUE	;Torque LED off
	led_on	LED_READY	;Ready LED on
	ret



;----------------------------------------------------------;
; 83kHz Position capture and servo operation interrupt


; part 1: position capture
;

; input: PIND
; sideeffects/global variables:
; - PvEnc: previous A/B signal (r/w)
; - Pos/PosX: update the current position (r/w)
; - saved registers: T0L, SREG, Z (left on stack)
; modified registers: Z

background:
	push	T0L
	pushw	Z
	in	T0L, SREG		;Save flags

; ZL = PvEnc;
	mov	ZL, _PvEnc		;ZL[1:0] = previous A/B signal
; PvEnc = swap(PIND);
	in	_PvEnc, PIND		;Sample A/B signal into _PvEnc[1:0]
	swap	_PvEnc			;/
; if (PvEnc & 2) PvEnc ^= 1;
	ldi	ZH, 1			;Convert it to sequencial number.
	sbrc	_PvEnc, 1		;
	eor	_PvEnc, ZH		;/
; ZL = (ZL - PvEnc) & 3;
	sub	ZL, _PvEnc		;Decode motion
	andi	ZL, 3			;/
; if (ZL) {
	breq	enc_zr			;-> Not moved
;   if (ZL == 3) {
;     // enc_rev
;     PvDir = -1;
;     Pos--;
;   } else if (ZL == 1) {
;     // enc_fwd
;     PvDir = 1;
;     Pos++;
;   } else {
;     // missing code recovery
;     Pos += 2*PvDir;
;   }
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
	rjmp	b013
;     } // if (ZL!=1)
enc_fwd:ldiw	Z, 1
b013:	mov	_PvDir, ZL
enc_add:addw	_Pos, Z
	adc	_PosX, ZH
; } // if (ZL)

; if (--CtDiv) return;
enc_zr:
	dec	_CtDiv		;Decrement 1/83 divider
	rjne	bgnd_exit	;If not overflow, exit interrupt routine.

; End of 83 kHz position captureing process. Follows are 1kHz servo
; operation. It will be interrupted itself, but will not be re-entered.

; CtDiv = 83;
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

	sbr	_Flags, bit7	; 1kHz interrupt flag

; T2 = Pos - PvPos;
; PvPos = Pos;
	lddw	T2, Y+iPvPos	;Detect velocity
	cli			;
	subw	T2, _Pos	;
	stdw	Y+iPvPos, _Pos	;
	sei			;
	negw	T2		;/ T2 = velocity

;
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
	lddw	T0, Y+iCtPos	;
	ldd	BH, Y+iCtPos+2	;
	subw	T0, _Pos	;
	sbc	BH, _PosX	;
	sei			;BH:T0 = position error

; if (pos_error >= LimSpd) pos_error = LimSpd;
; else if (pos_error <= -LimSpd) pos_error = -LimSpd;
	lddw	A, Y+iLimSpd	;Velocity limit (P0)
	clr	BL		;
	cpw	T0, A		;
	cpc	BH, BL		;
	brge	b10		;
	negw	A		;
	com	BL		;
	cpw	T0, A		;
	cpc	BH, BL		;
	brge	tap_velocity	;
b10:	movw	T0L, AL		;T0 = velocity command

; output a velocity value (mode 2)
;

; input:
; - T0: desired velocity value
; - T2: current velocity (in pules/ms)
; - Y: pointer to motor parameter block
; output:
; - T0: next torque value
; modified registers: A, B, Z
tap_velocity:
; T0 -= speed{T2} * P1
	movw	AL, T2L		;Velocity loop gain (P1)
	lddw	B, Y+iGaSpd	;
	 rcall	muls1616	;B = scaled velocity
	subw	T0, B		;T0 = velocity error

; Z = T0 * GaTqP			; P term P2
	movw	AL, T0L		;Velocity error P-gain (P2)
	lddw	B, Y+iGaTqP	;
	 rcall	muls1616	;
	movw	ZL, BL		;Z = P term;

; Z += PvInt * GaTqI			; I term P3
	lddw	A, Y+iPvInt	;Velocity error I-gain (P3)
	lddw	B, Y+iGaTqI	;
	 rcall	muls1616	;
	addw	Z, B		;Z += I term;

; if (Z>= LimTrq) {Z=LimTrq; set_flag(6);}	;Torque limit P4 (LimTrq)
	cbr	_Flags, bit6+bit5;Torque limit (P4)
	lddw	B, Y+iLimTrq	;
	cpw	Z, B		;
	brlt	b20		;
	movw	ZL, BL		;
	sbr	_Flags, bit6	;
; if (Z< -LimTrq) {Z=-LimTrq; set_flag(5);}
b20:	neg	BL		;
	com	BH		;
	cpw	Z, B		;
	brge	b21		;
	movw	ZL, BL		;
	sbr	_Flags, bit5	;/

; if ( ((T0<0)&&!flag(5)) || ((T0>=0)&&!flag(6)) ) PvInt += T0;
b21:	tst	T0H		;PvInt += T0, with anti-windup
	brmi	b22		;
	sbrc	_Flags, 6	;
	rjmp	b24		;
	rjmp	b23		;
b22:	sbrc	_Flags, 5	;
	rjmp	b24		;
; PvInt += T0
b23:	lddw	A, Y+iPvInt	;
.ifdef USE_DECAY
;-- addition: decay for the integral value after steady state is reached
;   (useful for static positioning applications like XY-tables)
; if (T0 || T2) A+=T0
; else {B=(A>>6); A-=B}

	mov	BL, T0L		; check for (T0==0)&&(T2==0)
	or	BL, T0H
	or	BL, T2L
	or	BL, T2H
	brne	b28
	movw	BL, AL		; B = A>>5
	asr	BH,1
	ror	BL
	asr	BH,1
	ror	BL
	asr	BH,1
	ror	BL
	asr	BH,1
	ror	BL
	asr	BH,1
	ror	BL
	subw	A, B		; A -= B;
	rjmp	b29
.endif
b28:	addw	A, T0		;
b29:	stdw	Y+iPvInt, A	;/
; if (flags(5) || flags(6)) {
;   led_on(LED_TORQUE);
; } else {
;   led_off(LED_TORQUE);
;   if (OvTmr-1)>0) {
;     OvTmr--;
;     if (OvTmr >= TL_TIME) goto servo_error;
;   }
; }
; T0 = Z
;
b24:	mov	AL, _Flags	;Check torque limiter timer
	andi	AL, bit6+bit5	; OvTmr is increased by 3 when torque limitter
;AL is overwritten right away, but the CPU flags survive
	lddw	A, Y+iOvTmr	; works, decreased by 1 when no torque limit.
	breq	b25		; When the value reaches TL_TIME, servo turns
	led_on	LED_TORQUE	; off and goes error state.
	addiw	A, 3		;
	rjmp	b26		;
b25:	led_off	LED_TORQUE	;
	subiw	A, 1		;
	brlo	b27		;
b26:	stdw	Y+iOvTmr, A	;
	ldiw	B, TL_TIME	;
	cpw	A, B		;
	brsh	servo_error	;/

b27:	movw	T0L, ZL		;T0 = torque command

; output a torque/current value (mode 1)
;
; current_PWM{T0} += speed{T2}*P5;
;
; input:
; - T2: current velocity (in pules/ms)
; - Y: pointer to motor parameter block
; - T0: current PWM value (left over from the last round)
; output:
; - T0: next PWM value
; modified registers: A, B
tap_torque:
	movw	AL, T2L		;EG compensation (P5)
	lddw	B, Y+iGaEG	;
	 rcall	muls1616	;
	addw	T0, B		;T0 = voltage command


; output a PWM value (mode 0)
;
; clip T0 to [-240;240]
; OCR1A = (T0/2) + 120;
; OCR1B = 120 - (T0/2);
;
; input:
; - T0: value to output.
;
; valid outputs and side effects:
;
; modified registers: A
tap_voltage:
	ldiw	A, 240		;Clip output voltage between -240 and +240.
	cpw	T0, A		; Limit minimum duty ratio to 15/16 for bootstrap
	brge	b30		; type FET driver.
	ldiw	A, -240		;
	cpw	T0, A		;
	brge	b31		;
b30:	movw	T0L, AL		;T0 = PWM command

b31:	asrw	T0		;Set PWM register (OCR1A and ~OCR1B)
	ldi	AL, 120		;
	adc	AL, T0L		;
	ldi	AH, 120		;
	sub	AH, T0L		;
.if OCR1AL < 0x40
	out	OCR1AL, AL	;
	out	OCR1BL, AH	;/
.else
	sts	OCR1AL, AL	;
	sts	OCR1BL, AH	;/
.endif

	 rcall	disp_pos

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
	clr	AL		;Enter Mode0
	 rcall	init_servo	;/
	led_on	LED_ERROR	;Error LED on
	clrw	T0		;Output off
	rjmp	tap_voltage



;--------------------------------------;
; Display position counter

disp_pos:
	lds	DL,Disp+8
	inc	DL
	cpi	DL,8
	brcs	dp_out

	mov	AL,_PosL
	mov	AH,_PosH
	mov	BL,_PosX
	ldi	CH,0
	sbrs	BL,7
	rjmp	b50
	ldi	CH,0x40
	com	AL
	com	AH
	com	BL
	adc	AL,_0
	adc	AH,_0
	adc	BL,_0
b50:	ldiw	Y,Disp
dp_l1:	ldi	CL,24
	ldi	BH,0
dp_l2:	lsl	AL
	rol	AH
	rol	BL
	rol	BH
	cpi	BH,10
	brcs	b51
	subi	BH,10
	inc	AL
b51:	dec	CL
	brne	dp_l2
	ldiw	Z,t_7seg*2
	add	ZL,BH
	adc	ZH,_0
	lpm	CL,Z
	st	Y+,CL
	cp	AL,_0
	cpc	AH,_0
	cpc	BL,_0
	brne	dp_l1
b52:	st	Y+,CH
	clr	CH
	cpi	YL,low(Disp+8)
	brne	b52
	clr	DL

.ifdef USIDR
; This code uses the USI device of the tiny2313.
; - write data to USIDR
; - double-toggle USCK 8 times to shift out the data
; - write 0x80 to USIDR to set DO to high
; - toggle DI to advance to the next digit.
dp_out:	ldiw	Y,Disp
	std	Y+8,DL
	add	YL,DL
	adc	YH,_0
	ld	AL,Y
	com	AL
	out	USIDR,AL
	ldi	AH,0x80
	ldi	CL,8
b53:	out	PINB,AH
	dec	CL
	out	PINB,AH
	brne	b53
	cpi	DL,0
	brne	b54
	ldi	AH,0
b54:	out	USIDR,AH
	sbi	PORTB,5
	cbi	PORTB,5
	ret
.else
;FIXME: add SPI code for ATmega.
dp_out:
	ret
.endif




;--------------------------------------;
; 16bit * 16bit signed multiply
; 
; Multiplier:   A(signed int)
; Multiplicand: B(unsigned, 8.8 fraction)
; Result:       B(signed int)
; Clk:		181(max)

muls1616:
	clt
	tst	AH
	brpl	b60
	set
	negw	A

b60:	subw	C, C	; clear high 16bit.
	ldi	DL, 17	; DL = loop count
b61:	brcc	b62	; ---- calculating loop
	addw	C, A	;
b62:	rorw	C	;
	rorw	B	;
	dec	DL	; if (--DL > 0)
	brne	b61	;  continue loop;

	mov	BL, BH
	mov	BH, CL

	brtc	b63	; Negate the result if multiplier was negative.
	negw	B
b63:	ret



;--------------------------------------;
; Input a command line into LineBuf.
;
; returns:
; - BH: number of char in buffer
; - X: pointer to LineBuf
get_line:
	ldiw	X,LineBuf
	ldi	BH,0
rl_lp:	 rcall	receive
	breq	rl_lp
	st	X,AL
	cpi	AL,0x0d	; CR
	brne	b80
	ldiw	X,LineBuf
	rjmp	echo
b80:	cpi	AL,0x08	; BS
	brne	b81
	cpi	BH,0
	breq	rl_lp
	 rcall	echo
	sbiw	XL,1
	dec	BH
	rjmp	rl_lp
b81:	cpi	AL,' '		; SP
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

b82:	 rcall	xmit
dp_str:	lpm	AL, Z+
	tst	AL
	brne	b82
	ret


;--------------------------------------;
; Get value of decimal string
;
; Call: X -> ASCII string
; Ret:  X = updated
;         if    C=1: error
;         elsif Z=1: end of line, value=0
;         else:      BL:AL = 24bit value
;
;  Positive:   "300"
;  Negative:   "-125000"

get_val:
; flag_neg = 0;	// use T as flag for negative value
	clt
; val = 0;
	clr	AL
	clr	AH
	clr	BL
; do {
;   BH = *X++;
b83:	ld	BH,X+
;   if (BH < ' ') return (c1z0);	// return empty line
	cpi	BH,' '
	brcs	gd_n
	breq	b83
; } while (BH == ' ');			// ignore spaces
; if (BH == '-') {
;	flag_neg = 1;
;	BH = *X++;
; }
	cpi	BH,'-'
	brne	b84
	set
gd_l:	ld	BH,X+		; in C this would be at the end of loop
; do {
;   if (BH <= ' ') goto gd_e;	// found end of number, return success
b84:	cpi	BH,' '+1
	brcs	gd_e
;   BH -= '0';
	subi	BH,'0'
;   if (BH < 0) goto gd_x;	// return syntax error
	brcs	gd_x	; could be left out, this is covered by the next compare.
;   if (BH >= 10) goto gd_x;	// return syntax error
	cpi	BH,10
	brcc	gd_x
;   val = val*10 + BH;		// this loop is a 24x8 bit multiplication
	ldi	CL, 25
	ldi	CH, 10
	sub	r0, r0
b85:	lsr	r0
	ror	BL
	ror	AH
	ror	AL
	brcc	b86
	add	r0, CH
b86:	dec	CL
	brne	b85
	add	AL, BH
	adc	AH, _0
	adc	BL, _0
	rjmp	gd_l
;   BH = *X++;		// implemented in assembler at the beginning
; } while (1);
; return (z1c1);	// return value for error
gd_x:	sec
	sez
	ret
; X--
; if (flag_neg) {
;	val = -val;
; }
; return (c0);		// return value for success
gd_e:	sbiw	XL,1
	brtc	b87
	com	AL
	com	AH
	com	BL
	subi	AL,-1
	sbci	AH,-1
	sbci	BL,-1
b87:	clc
	ret
; X--
; return (z1c0);	// return value for empty line
gd_n:	sbiw	XL,1
	clc
	sez
	ret



;--------------------------------------;
; Display a value in decimal string
;
; Call: BL:A = 24bit signed value to be displayed
; Ret:  BL:A = broken

; This implements the standard algorithmn for unsigned division Q,R=N/D with
; N and Q both in BL:A, R in BH and D fixed to 10.
;
; Q := 0                  -- Initialize quotient and remainder to zero
; R := 0                     
; for i := n − 1 .. 0 do  -- Where n is number of bits in N
;   R := R << 1           -- Left-shift R by 1 bit
;   R(0) := N(i)          -- Set the least-significant bit of R equal to bit i of the numerator
;   if R ≥ D then
;     R := R − D
;     Q(i) := 1
;   end
; end

dp_dec:	ldi	CH,' '
; CH = ' ';
; if (val < 0) {
	sbrs	BL, 7
	rjmp	b88
;   val = -val;
	com	AL
	com	AH
	com	BL
	adc	AL,_0
	adc	AH,_0
	adc	BL,_0
;   CH = '-'
	ldi	CH,'-'
; }
; // convert the value into an ASCII string on the stack
; T0L = 0;
b88:	clr	T0L		;digit counter
; do {
;   T0L++;
;   BH = 0;
;   CL=24;
b088:	inc	T0L		;---- decimal string generating loop
	clr	BH		;var1 /= 10;
	ldi	CL,24		;
;   do {
;     var1 *= 2;
b89:	lslw	A		;
	rolw	B		;
;     if ((var1 >> 24) >= 10) {
	cpi	BH,10		;
	brcs	b89a		;
;       var1 -= (10<<24);
;       var1++;
	subi	BH,10		;
	inc	AL		;
;   } while (--CL);
b89a:	dec	CL		;
	brne	b89		;/
;   push(BH+'0');
	addi	BH,'0'		;Push the remainder (a decimal digit)
	push	BH		;/
; } while (var1);
	cp	AL,_0		;if(var1 =! 0)
	cpc	AH,_0		; continue digit loop;
	cpc	BL,_0		;
	brne	b088		;/
; // output the stored string in opposite order
; xmit(CH);	// sign
	mov	AL, CH		;Sign
	 rcall	xmit		;/
; do {
;   xmit(pop());
; } while (--T0L);
b89b:	pop	AL		;Transmit decimal string
	 rcall	xmit		;<-- Put a char to memory, console or any other display device
	dec	T0L		;
	brne	b89b		;/
	ret


;--------------------------------------;
; Serial I/O driver

	; Transmit AL.
echo:
xmit:
.if UDR < 0x40
b90:	sbis	UCSRA, UDRE
	rjmp	b90
	out	UDR, AL
	ret
.else
	push	AH		; not sure if AH is really important.
b90:
	lds	AH, UCSRA
	sbrs	AH, UDRE
	rjmp	b90
	sts	UDR, AL
	pop	AH
	ret
.endif

; Receive a char into AL. (ZR=no data)
;
; modified regs: AL
; return status: Z=1: no data, Z=0: data in AL valid
; global variables:
;   read: RxBuf: Rp, Wp, Buff
;   write: Rp
receive:
	push	AH
	pushw	Y
	ldiw	Y, RxBuf
	cli
	ldd	AH, Y+0
	ldd	AL, Y+1
	cp	AH, AL
	breq	b91
	add	YL, AH
	ldd	AL, Y+2
	sub	YL, AH
	inc	AH
	andi	AH, 15
	std	Y+0, AH
	clz
b91:	sei
	popw	Y
	pop	AH
	ret

rxint:	;USART0 Rx ready
	push	AL
	in	AL, SREG
	push	BL
	pushw	A
.if UCSRB < 0x40
	in	BL, UDR
	cbi	UCSRB, RXCIE
.else
	lds	BL, UDR
	lds	AL, UCSRB
	andi	AL, low(~(1<<RXCIE))
	sts	UCSRB, AL
.endif
	sei
;	pushw	A		; moved to an earlier position
	pushw	Y
	ldiw	Y, RxBuf
	ldd	AL, Y+0
	ldd	AH, Y+1
	inc	AH
	andi	AH, 15
	cp	AH, AL
	breq	b92
	std	Y+1, AH
	dec	AH
	andi	AH, 15
	add	YL, AH
	std	Y+2, BL
b92:	popw	Y
	popw	A
	pop	BL
	out	SREG, AL
.if UCSRB < 0x40
	pop	AL
	cli
	sbi	UCSRB, RXCIE
.else
	cli
	lds	AL, UCSRB
	ori	AL, (1<<RXCIE)
	sts	UCSRB, AL
	pop	AL
.endif
	reti


;----------------------------------------------------------;
; Strings

m_prompt:	.db	13,10, "%", 0
m_error:	.db	10, "?", 0
m_start:	.db	13,10, "SMC type 3", 13,10, 0

t_7seg:		.db	0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F

