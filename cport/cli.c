//;----------------------------------------------------------;
//; Command processing loop

void loop()
{
	dp_str(m_prompt);	//	;Display command prompt

	// if there's any serial available, read it:
	while (Serial.available() > 0) {
		c = Serial.read();
		if ( c>='a' ) c -= 0x20;// tolower(c)
		switch (c) {
			case ' ':	break;	// empty line
			case 'J':
				do_jump();
				break;
			case 'G':
				do_go();
				break;
			case 'L':	//	;Show posision counter?
				do_loc();
				break;
			case 'R':	//	;Read parms?
				do_reep();
				break;
			case 'W':	//	;Write parms?
				do_weep();
				break;
			case 'M':	//	;Set mode?
				do_mode();
				break;
			case 'S':	//	;Set sub command?
				do_sub();
				break;
			case 'P':	//	;Set parms?
				do_parm();
				break;
			default:	//	;Syntax error
				dp_str(m_error);
		};
	}
/*
	buf = get_line();	//	;Get a command line
	c = *buf++;
	if ( c>='a' ) c -= 0x20;// tolower(c)
	switch (c) {
		case ' ':	break;	// empty line
		case 'J':
			do_jump();
			break;
		case 'G':
			do_go();
			break;
		case 'L':	//	;Show posision counter?
			do_loc();
			break;
		case 'R':	//	;Read parms?
			do_reep();
			break;
		case 'W':	//	;Write parms?
			do_weep();
			break;
		case 'M':	//	;Set mode?
			do_mode();
			break;
		case 'S':	//	;Set sub command?
			do_sub();
			break;
		case 'P':	//	;Set parms?
			do_parm();
			break;
		default:	//	;Syntax error
			dp_str(m_error);
	};
*/
}


//;------------------------------------------------;
//; Change parameters, command regs or servo mode.

static inline void do_mode(void)	//:	; Change servo mode
{
	init_servo(Serial.parseInt());
}

static inline void do_sub(void)		//:	; Set subcommand reg.
{

}
/*
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
	clr	BL
	sbrc	AH, 7
	dec	BL
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
*/

