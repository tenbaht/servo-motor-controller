#include <avr/pgmspace.h>

#include "csmc3.h"
#include "cli.h"
#include "uart.h"
#include "control.h"
#include "eeprom.h"


static const char m_prompt[] PROGMEM = "\r\n%";
static const char m_error[] PROGMEM = "\n?";

static void cmd_err(void);



static void do_jump();
static void do_go();
static void do_loc();
static void do_reep();
static void do_weep();
static void do_mode();
static void do_sub();
static void do_parm();

static void ds_set(int16_t *ptr);

void dp_dec(int16_t v);	// print an integer value (should be 24 bit)
uint8_t get_line(void);


// getval(): 0=kein Wert, 1=Wert ok. Wert ist in val
uint8_t get_val();
int24_t val;


uint8_t LineBuf[20];	// Command line input buffer
uint8_t *buf;		// read pointer into line buffer

//;----------------------------------------------------------;
//; Command processing loop

void task_cli()
{
	uint8_t c;

	dp_str(m_prompt);	//	;Display command prompt

	get_line();	//	;Get a command line
	c = *buf++;
	if ( c>='a' ) c -= 0x20;// tolower(c)

	if (c<32) return;	// empty line
	if (c=='J') {do_jump(); return;}	// Jump?
	if (c=='G') {do_go(); return;}		// Go?
	if (c=='L') {do_loc(); return;}		// Show posision counter?
	if (c=='R') {do_reep(); return;}	// Read parms?
	if (c=='W') {do_weep(); return;}	// Write parms?
	if (c=='M') {do_mode(); return;}	// Set mode?
	if (c=='S') {do_sub(); return;}		// Set sub command?
	if (c=='P') {do_parm(); return;}	// Set parms?
	// Syntax error
	dp_str(m_error);
}
/*
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
*/


void do_jump(){Serial_print_s(__func__);}
void do_go(){Serial_print_s(__func__);}


/**
 * Load parameters from EEPROM
 *
 */
void do_reep()
{
	if (get_val() || ((uint24_t) val>= EEBANKS)) {
		cmd_err();
		return;
	}
	load_parms(val);
}


/**
 * Save parameters into EEPROM
 *
 */
void do_weep()
{
	if (get_val() || ((uint24_t) val>= EEBANKS)) {
		cmd_err();
		return;
	}
	save_parms(val);
}


/**
 * print the current location
 *
 * keep updating the output until a key press is received.
 *
 * It is sufficient to compare only the lower 8 bits of the position counter.
 * To help the compiler avoiding an unneeded cast to int for the comparision
 * the two temp variabes a and b are used. The (nonsense) variable assignments
 * are cut out by the optimizer and never make it into the output.
 */
static void do_loc()
{
	int24_t	p;
	uint8_t a,b;	// temp variables to help the compiler casting the type

	Serial_print_s(__func__);
	xmit(10);
	do {
		xmit(13);
		BEGIN_CRITICAL
		p = Pos;
		END_CRITICAL
		dp_dec(p);
		xmit(32);
		do {
			if (Serial_available()) return;
//		} while ((uint8_t)p == (uint8_t)Pos);	// cast does not work
			a = (uint8_t) p;	// all this is optimized away
			b = (uint8_t) Pos;
		} while (a == b);	// and this results in a 8 bit compare
	} while (1);
}


static void cmd_err(void)
{
	dp_str(m_error);
}



//;------------------------------------------------;
//; Change parameters, command regs or servo mode.

static void do_mode(void)	//:	; Change servo mode
{
	if (get_val()) {
		cmd_err();
		return;
	}
	init_servo(val);
}


static void do_sub(void)		//:	; Set subcommand reg.
{
	ds_set(&CtSub);
}


static void do_parm(void)		// Set parameters
{
	if (get_val() || (val >= N_PARM)) {
		cmd_err();
		return;
	}

	ds_set((int16_t*)&Parms[val]);
}

static void ds_set(int16_t *ptr)
{
	uint8_t c;

	c = get_val();
	if (c == 2) {
		cmd_err();
		return;
	}
	if (c) {	// no value given:
		// show current value
		xmit('\n');
		dp_dec(*ptr);
		xmit(':');

		// allow to enter a new value
		get_line();
		c = get_val();
		if (c == 2) {
			cmd_err();
			return;
		}
		if (c) return;	// don't change anything for empty input
	}
	//FIXME: BEGIN_CRITICAL
	*ptr = val;
	//FIXME: END_CRITICAL
}


/*
 * read one line from serial
 *
 * reads until CR (13) is received or the line buffer is full.
 * This is similar to Arduino Serial.readBytesUntil(), but without any timeout.
 *
 * @returns: number of valid bytes in the line buffer
 */
uint8_t get_line(void)
{
	uint8_t n;	// BH: number of char in buffer
	uint8_t c;
	uint8_t *ptr;		// write pointer in line buffer

	ptr = LineBuf;
	n = 0;
	do {
//		while ((c=receive())<0);	// wait for one char
		while(Serial_available()<=0);
		c = Serial_read();
		*ptr = c;
		if (c==13) break;
		if (c==8) {	// BS
			if (n) {
				echo(c);
				ptr--;
				n--;
			}
			continue;
		}
		if (c<32) continue;
		if (n==19) continue;
		echo(c);
		ptr++;
		n++;
	} while(1);
	echo(c);
	buf = LineBuf;
	return n;
}


/**
 * print a zero-terminated string in flash over uart
 *
 *
 * This could be _much_ more efficient if the parameter would be in Z already:

b82:     rcall  xmit
dp_str: lpm     AL, Z+
        tst     AL
        brne    b82
        ret

 */
void dp_str(const char *pstr)	//	;Display string
{
	uint8_t c;

	while ((c=pgm_read_byte(pstr++))) {	// intentional assign
		xmit(c);
	}
}

/*
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
 *
 * The meaning of the return value is different from the assembler version.
 * In C zero is ok.
 *
 * used/effected global variables:
 *	buf: in: the first byte to be read
 *	     out: the first unprocessed byte
 *
 * @returns:
 *	0: ok, valid value in global variable val
 *	1: empty line, val=0
 *	2: syntax error, val invalid
 */
uint8_t get_val(void)
{
	uint8_t	neg=0;
	uint8_t c;			// unsigned is essential, see below

	val = 0;

	// ignore leading space
	do {
		c = (uint8_t) *buf++;
		if (c < ' ') return (1);	// found empty line
	} while (c == ' ');		// read until non-space character

	// handle minus sign
	if (c == '-') {
		neg = 1;
		c = (uint8_t) *buf++;
	}

	// read all characters up to CR or space
	while (c > ' ') {
		c -= '0';	// c is unsigned, so values below 0x30 wrap over
		if (c>9) {
			buf--;
			return (2);	// syntax error
		}
		val = val*10 + c;
		c = (uint8_t) *buf++;
	}

	// success
	buf--;
	if (neg) val = -val;
	return (0);	// ok
}


#ifdef USE_ARDUINO
// #define dp_dec	Serial_print_i
inline void dp_dec(int16_t v)
{
	Serial_print_i(v);
}
#else
void dp_dec(int16_t v)	// print an integer value (should be 24 bit)
{
	char sign, c;
	uint8_t digits;

	sign = ' ';
	if (v<0) {
		sign = '-';
		v = -v;
	}

	// FIXME: This implementation is inefficient as it does the
	// division two times instead of saving the quotient the first time.
	digits = 0;
	do {
		digits++;
		push((v%10)+'0');
	} while (v /= 10);

	// print the saved string from the stack
	xmit(sign);
	while (digits--) {
		xmit(pop());
	}
}
#endif

/*
 unsigned division Q,R = N/D

Q := 0                  -- Initialize quotient and remainder to zero
R := 0                     
for i := n − 1 .. 0 do  -- Where n is number of bits in N
  R := R << 1           -- Left-shift R by 1 bit
  R(0) := N(i)          -- Set the least-significant bit of R equal to bit i of the numerator
  if R ≥ D then
    R := R − D
    Q(i) := 1
  end
end

BH	R
AL	Q
*/
