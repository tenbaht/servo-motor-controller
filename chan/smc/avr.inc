;------------------------------------------------;
; Constants

.equ	RAMTOP = 0x60



.equ	bit0	= 0b00000001
.equ	bit1	= 0b00000010
.equ	bit2	= 0b00000100
.equ	bit3	= 0b00001000
.equ	bit4	= 0b00010000
.equ	bit5	= 0b00100000
.equ	bit6	= 0b01000000
.equ	bit7	= 0b10000000



;------------------------------------------------;
; Load immediate word into the resister pair.
;
;	ldiw	Z,var

.macro	ldiw
	ldi	@0L,low(@1)
	ldi	@0H,high(@1)
.endm


;------------------------------------------------;
; Load memory word into the resister pair.
;
;	ldsx	Z,mem

.macro	ldsx
	lds	@0L,@1
	lds	@0H,@1+1
.endm


;------------------------------------------------;
; Store resister pair into memory
;
;	stsx	mem,Z

.macro	stsx
	sts	@0,@1L
	sts	@0+1,@1H
.endm


;------------------------------------------------;
; Store immediate into the indirect memory
;
;	sti	Z,imm

.macro	sti
	ldi	r16,@1
	st	@0,r16
.endm


;------------------------------------------------;
; Add immediate to register
;
;	addi	var,imm

.macro	addi
	subi	@0,-(@1)
.endm

.macro	adci
	sbci	@0,-(@1)
.endm


;------------------------------------------------;
; Store immediate into the direct memory
;
;	stsi	var,imm

.macro	stsi
	ldi	r16,@1
	sts	@0,r16
.endm


;------------------------------------------------;
; Output immediate
;
;	outi	port,var

.macro	outi
	ldi	r16,@1
	out	@0,r16
.endm


;------------------------------------------------;
; Long branch


.macro	rjne
	breq	PC+2
	rjmp	@0
.endm

.macro	rjeq
	brne	PC+2
	rjmp	@0
.endm

.macro	rjcc
	brcs	PC+2
	rjmp	@0
.endm

.macro	rjcs
	brcc	PC+2
	rjmp	@0
.endm

