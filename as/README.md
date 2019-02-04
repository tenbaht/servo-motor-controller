# Compiling smc3 using avr-as

smc3 was written for the original AVR assembler (compatible to avra).
Unfortunatly, the syntax differs slightly from avr-as.

The main advantage of avr-as over avra is the possibility to generate
linkable .o object files that can be mixed with C files. So I thought
converting the code for avr-as would be a useful intermediate step on the
way to embedd smc3 in other C-based projects.

Unfortunatly, it turns out that the syntax differences are big enough to
make the conversion of source code a non-trivial task and it took way longer
than expected.

I summed up my findings about [migrating an avra project to
avr-as](https://tenbaht.github.io/posts/migrating-from-avra-to-avr-as/) and
hope that can be useful to someone.



## Compile

	make

Or more in detail:

	avr-gcc -mmcu=atmega328 -Wa,-I/usr/lib/avr/include -c smc3.sx
	avr-gcc -mmcu=atmega328 -nostartfiles -nostdlib -nodefaultlibs smc3.o -o smc3.elf
	avr-objcopy -O ihex -R .eeprom smc3.elf smc3.hex
	avr-objcopy -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 -O ihex smc3.elf smc3.eep.hex


## Quality control

The goal was to convert the source code in a way that it results in exactly
the same binary. For checking on this it was neccessary to normalize the
output files of avra and as into a common format that can be diff'ed
efficiently.


avra saves the interrupt vectors separately in individual pseudo-sections:

```
$ objdump -h smc3.hex

smc3.hex:     file format ihex

Sections:
Idx Name          Size      VMA       LMA       File off  Algn
  0 .sec1         00000004  00000000  00000000  00000011  2**0
                  CONTENTS, ALLOC, LOAD
  1 .sec2         00000004  00000038  00000038  00000026  2**0
                  CONTENTS, ALLOC, LOAD
  2 .sec3         00000004  00000048  00000048  0000003b  2**0
                  CONTENTS, ALLOC, LOAD
  3 .sec4         00000718  00000068  00000068  00000050  2**0
                  CONTENTS, ALLOC, LOAD
```

This can be normalized by converting to a binary image and back:

	objcopy  -I ihex -O binary smc3.hex smc3.bin
	objcopy  -O ihex -I binary smc3.bin normal-avra.hex

This can be used to produce a normalized disassembler listing:

	avr-objdump -j .sec1 -m avr5 -d -I ihex normal-avra.hex  > avra.disass

A similar listing can be produced for the as source. Both are normalized and
can be compared using the usual text file comparision tools:

	make disass
	diffuse smc3.disass ../atmega/avra.disass



## Modifing the avra source for use with avr-as

These quick notes summarize the steps to initially convert the source code.



### Modifications to the main file smc3.asm

Remove `.equ`, only keep `var = val`:

	sed -i -e "s/^\.equ[ \t]*//" smc3.asm

Replace `.def name = val` by `#define name val`:

	sed -i -e "/^\.def/ s/=[ \t]*//" -e "s/\.def/#define/" smc3.asm

Make sure all comments in `#define` lines are C comments:

	sed -i -e "/^#define/ s#;#//#"

Optional: replace `.ifdef` by `#ifdef`. This requires hand-editing later.
Sometimes one is better than the other in a particular situation:

	sed -i -e "s/\.if/#if/" -e "s/\.endif/#endif/" -e "s/\.el/#el/" smc3.asm

replace the section specifier:

	.eseg		.section eeprom
	.dseg		.section bss
	.cseg		.text
	.byte		.fill

The easiest is using sed again:

	sed -i 	-e "s/\.eseg/\.section eeprom/" \
		-e "s/\.dseg/\.section bss/" \
		-e "s/\.cseg/\.text/" smc3.asm

The remaining changes can't be automated.



#### Relocate negative values in a different segment

Requires brackets around the variable name to separate it from the minus
sign. Otherwise the assembler wouldn't choose the required relocation type
R_AVR_LO8_LDI_NEG.

	subiw	YL, -Parms	; works only for avra
	subiw	YL, -(Parms)	; works for both

Strange, isn't it? Is this an assembler bug?


#### pointer

Since avr-as calculates flash addresses in bytes rather than word, the
pointer values don't need to be multiplied by two anymore to access the stored
strings.



### Modifications to avr.inc

Massive. And most of it can't be automated. And the result needs to be
included with `#include`, because it contains `#define`s that need to
evaluated at preprocessing time.

Don't change the .if/.endif, because this time they need to evaluated at
assembly time.

At least the .equ and .def replacements work the same way as before:

	sed -e "s/^\.equ[ \t]*//" -e "/^\.def/ s/=[ \t]*//" -e "s/\.def/#define/" ../atmega/avr.inc > avr.inc

Add some compatibility definitions:

	#define low(X)		lo8(X)
	#define high(X) 	hi8(X)
	#define RAMTOP		RAMSTART
	#define EEPROMEND	E2END
	#define __FLASH_SIZE__	FLASHEND
	#define INT_VECTORS_SIZE _VECTORS_SIZE

	#define	A	AL
	#define	B	BL
	#define	C	CL
	#define	D	DL
	#define	T0	T0L
	#define	T2	T2L
	#define	T4	T4L

There might be weird problems with nested expressions like _low(low(val))_.

Adopting the syntax of macro parameters from @0/@0L/@0H to \par0/\par0/\par0+1
like this is more complex, see
[here](view-source:https://tenbaht.github.io/posts/migrating-from-avra-to-avr-as/#macros-and-preprocessor-constants)

	.macro	ldiw	par0,par1
		ldi	\par0,low(\par1)
		ldi	\par0+1,high(\par1)
	.endm

This was a try to convert from @0/@0L to `\par0\()L` and `\par0\()H`:

	sed -e 's/@\([0-9]\)\([HL]\)/@par\1\\\(\)\2/g' -e 's/@\([0-9]\)/@par\1/g' avr.inc

That's quite a regular expression, isn't it? But it doesn't help, the L/H
syntax doesn't work anyway. (Again, it would be required at preprocessing
time)

It would be possible to use C macros. They would require braches around the
arguments, but it could work. The preprocessor outputs the whole macro
content as one line, but '$' can be used to [mark the intentend line
breaks](https://sourceware.org/binutils/docs-2.21/as/AVR_002dChars.html )
for the assembler.
