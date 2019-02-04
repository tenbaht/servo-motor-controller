#define low(X) lo8(X)
#define high(X) hi8(X)

;------------------------------------------------;
; Constants

;RAMTOP = RAMSTART	; SRAM top address
#define RAMTOP 		RAMSTART
#define EEPROMEND	E2END
#define __FLASH_SIZE__	FLASHEND
#define INT_VECTORS_SIZE _VECTORS_SIZE

bit0	= 0b00000001
bit1	= 0b00000010
bit2	= 0b00000100
bit3	= 0b00001000
bit4	= 0b00010000
bit5	= 0b00100000
bit6	= 0b01000000
bit7	= 0b10000000

; this can't be done using assembler assigns
#define r0 0
#define r1 1
#define r2 2
#define r3 3
#define r4 4
#define r5 5
#define r6 6
#define r7 7
#define r8 8
#define r9 9
#define r10 10
#define r11 11
#define r12 12
#define r13 13
#define r14 14
#define r15 15
#define r16 16
#define r17 17
#define r18 18
#define r19 19
#define r20 20
#define r21 21
#define r22 22
#define r23 23
#define r24 24
#define r25 25
#define r26 26
#define r27 27
#define r28 28
#define r29 29
#define r30 30
#define r31 31

; these could use assembler assigns
#define	T0L	r0
#define	T0H	r1
#define	T2L	r2
#define	T2H	r3
#define	T4L	r4
#define	T4H	r5
#define	T6L	r6
#define	T6H	r7
#define	T8L	r8
#define	T8H	r9
#define	T10L	r10
#define	T10H	r11
#define	T12L	r12
#define	T12H	r13
#define	T14L	r14
#define	T14H	r15


#define	AL	r16
#define	AH	r17
#define	BL	r18
#define	BH	r19
#define	CL	r20
#define	CH	r21
#define	DL	r22
#define	DH	r23
#define	EL	r24
#define	EH	r25

#define	A	AL
#define	B	BL
#define	C	CL
#define	D	DL
//#define	X	XL
//#define	Y	YL
//#define	Z	ZL
#define	T0	T0L
#define	T2	T2L
#define	T4	T4L

;------------------------------------------------;
; Push/Pop register pair
;
; 	pushw	Z

.macro	pushw	reg
	push	\reg+1
	push	\reg
.endm

.macro	popw	reg
	pop	\reg
	pop	\reg+1
.endm


;------------------------------------------------;
; Load/store word from/to direct memory/immediate
;
;	ldsw	Z,mem
;	ldiw	Z,imm

.macro	xldiw	par0,par1
	ldi	\par0,16
	ldi	r17,16
.endm

.macro	ldiw	reg,val
	ldi	\reg,lo8(\val)
	ldi	\reg+1,hi8(\val)
.endm

.macro	ldsw	par0,par1		;
	lds	\par0,\par1
	lds	\par0+1,\par1+1
.endm

.macro	stsw	par0,par1
	sts	\par0+1,\par1\()H
	sts	\par0,\par1\()L
.endm

.macro	lddw	reg,val
	ldd	\reg,\val
	ldd	\reg+1,\val+1
.endm

.macro	stdw	disp,reg
	std	\disp+1,\reg+1
	std	\disp,\reg
.endm

.macro	ldw	par0,par1
	ld	\par0\()L,\par1
	ld	\par0\()H,\par1
.endm

.macro	stw	par0,par1
	st	\par0,\par1\()L
	st	\par0,\par1\()H
.endm

.macro	inw	par0,par1
.if \par0\()L < 0x40
	in	\par0\()L,\par1\()L
	in	\par0\()H,\par1\()H
.else
	lds	\par0\()L,\par1\()L
	lds	\par0\()H,\par1\()L
.endif
.endm

.macro	outw	port,reg
.if \port < 0x40
	out	\port+1,\reg+1
	out	\port,\reg
.else
	sts	\port+1,\reg+1
	sts	\port,\reg
.endif
.endm


;------------------------------------------------;
; Store immediate into indirect memory via r16
;
;	sti	Z,imm
;	stdi	Z+d,imm

.macro	sti	par0,par1
	ldi	r16,\par1
	st	\par0,r16
.endm

.macro	stdi	par0,par1
	ldi	r16,\par1
	std	\par0,r16
.endm

.macro	muli	par0,par1
	ldi	r16,\par1
	mul	\par0,r16
.endm


;------------------------------------------------;
; add/sub/subc/cp/cpc/lsl/lsr/rol/ror to register pair
;

; this does not work for addresses, avr-as can't relocate the -\val.
.macro	addiw	reg,val
	subi	\reg,lo8(-\val)
	sbci	\reg+1,hi8(-\val)
.endm

.macro	subiw	reg,val			;
	subi	\reg,lo8(\val)
	sbci	\reg+1,hi8(\val)
.endm

.macro	addw	par0,par1		;
	add	\par0,\par1
	adc	\par0+1,\par1+1
.endm

.macro	adcw	par0,par1
	adc	\par0\()L,\par1\()L
	adc	\par0\()H,\par1\()H
.endm

.macro	subw	par0,par1		;
	sub	\par0,\par1
	sbc	\par0+1,\par1+1
.endm

.macro	sbcw	par0,par1
	sbc	\par0\()L,\par1\()L
	sbc	\par0\()H,\par1\()H
.endm

.macro	cpw	par0,par1		;
	cp	\par0,\par1
	cpc	\par0+1,\par1+1
.endm

.macro	cpcw	par0,par1
	cpc	\par0\()L,\par1\()L
	cpc	\par0\()H,\par1\()H
.endm

.macro	cpiw	par0,par1
	cpi	\par0\()L,low(\par1)
	ldi	r16,high(\par1) 
	cpc	\par0\()H,r16
.endm

.macro	andw	par0,par1
	and	\par0\()L,\par1\()L
	and	\par0\()H,\par1\()H
.endm

.macro	andiw	par0,par1
	andi	\par0\()L,low(\par1)
	andi	\par0\()H,high(\par1)
.endm

.macro	orw	par0,par1
	or	\par0\()L,\par1\()L
	or	\par0\()H,\par1\()H
.endm

.macro	oriw	par0,par1
	ori	\par0\()L,low(\par1)
	ori	\par0\()H,high(\par1)
.endm

.macro	lslw	par0			;
	lsl	\par0
	rol	\par0+1
.endm

.macro	lsrw	par0
	lsr	\par0\()H
	ror	\par0\()L
.endm

.macro	asrw	par0			;
	asr	\par0+1
	ror	\par0
.endm

.macro	rolw	par0			;
	rol	\par0
	rol	\par0+1
.endm

.macro	rorw	par0			;
	ror	\par0+1
	ror	\par0
.endm

.macro	clrw	par0			;
	clr	\par0
	clr	\par0+1
.endm

.macro	comw	par0
	com	\par0\()L
	com	\par0\()H
.endm

.macro	negw	par0			;
	com	\par0+1
	neg	\par0
	brne	end\@
	inc	\par0+1
end\@:
.endm

.macro	movew	par0,par1
	mov	\par0\()L, \par1\()L
	mov	\par0\()H, \par1\()H
.endm

.macro	lpmw	par0,par1
	lpm	\par0\()L, \par1
	lpm	\par0\()H, \par1
.endm


;------------------------------------------------;
; Store immediate into direct memory via r16
;
;	stsi	var,imm

.macro	stsi	par0,par1
	ldi	r16,\par1
	sts	\par0,r16
.endm


;------------------------------------------------;
; Output port immediate via r16
;
;	outi	port,var

.macro	outi	port,var
.if \port < 0x40
	ldi	r16,\var
	out	\port,r16
.else
	ldi	r16,\var
	sts	\port,r16
.endif
.endm


;------------------------------------------------;
; Add immediate to register

.macro	addi	par0,par1
	subi	\par0,-(\par1)
.endm


;------------------------------------------------;
; Long branch

.macro	rjne	par0
	breq	end\@
	rjmp	\par0
end\@:
.endm

.macro	rjeq	par0
	brne	end\@
	rjmp	\par0
end\@:
.endm

#if 0
.macro	rjcc	par0
	brcs	PC+2
	rjmp	\par0
.endm

.macro	rjcs	par0
	brcc	PC+2
	rjmp	\par0
.endm

.macro	rjtc	par0
	brts	PC+2
	rjmp	\par0
.endm

.macro	rjts	par0
	brtc	PC+2
	rjmp	\par0
.endm

.macro	rjge	par0
	brlt	PC+2
	rjmp	\par0
.endm

.macro	rjlt	par0
	brge	PC+2
	rjmp	\par0
.endm


.macro	retcc
	brcs	PC+2
	ret
.endm

.macro	retcs
	brcc	PC+2
	ret
.endm

.macro	reteq
	brne	PC+2
	ret
.endm

.macro	retne
	breq	PC+2
	ret
.endm
#endif
;------------------------------------------------;
; Move single bit between two registers
;
;	bmov	dstreg,dstbit,srcreg.srcbit

.macro	movb	par0,par1,par2,par3
	bst	\par2,\par3
	bld	\par0,\par1
.endm


