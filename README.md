This is a port of the very sophisticated [DC Servomotor
Controller](http://elm-chan.org/works/smc/report_e.html) by Elm Chan for the
ATmega328 as used with the popular Arduino Uno and Arduino Nano boards.

It includes the results of a detailed analysis of the inner workings of the
source code and (hopefully) more beginner-friendly explanation of the
meaning of the control parameter and how to choose sensible values for the
desired application.

This analysis includes some almost-C code that I wrote to help me
understanding the inner workings of SMC3. That code won't compile, but it is
much easier to read than the highly optimized hand-crafted assembler
masterpiece by Elm Chan.



## Features of SMC3

- Originally written for the (now obsolete) ATtiny2313 microcontroller and
  now ported to the ATmega328.
- accepts serial command via UART
   - Direct Access to Position Command Register
   - G0/G1 Motion Generation
- Three Control Modes
   - Absolute Positioning Mode
   - Constant Speed Mode
   - Constant Torque Mode
- Software Decoding of Quadrature Encoder Input
   - Count rate over 100,000 counts/sec in quadrature decoding
   - No External Component Required
- Two PWM Outputs for H-bridge Driver
- Supports bootstrap type FET driver (minimum duty ratio 15/16)
- Three Diagnostic Outputs
   - Ready, Torque Limit and Servo Error
- On-the-Fly Servo Tuning



The version *SMC3A* is optimized to be a drop-in replacement for stepper motor
drivers: (but not ported yet). Instead of the UART interface it supports:

- Pulse/Dir Command Input with Pulse Multiplyer
- Pulse rate over 100,000 pulses/sec
- Gear ratio from 1/256 to 255 in 1/256 step


Differences of this version to the original:

- ported to ATmega328, should be easy to adopt for other ATmega CPU
- optional decay function for the integral sum to avoid unnecessary motor
  current after the target position is reached.
- no support for the proprietary serial position display on the SPI pins.



## Compiling

Install avra (The syntax is not compatible with avr-as):

	sudo apt install avra

compile:

	make

upload:

	make upload

Or more in detail:

	avra -I inc smc3.asm
	avrdude -c arduino -p m328p -U flash:w:smc3.hex -P /dev/ttyUSB0

The avra package on Debian-based systems (Ubuntu, Mint etc) is very, very
old and misses the required CPU definitions for the ATmega328. That's why m328Pdef.inc
is included in this repository.

To compile the original files (they were written for the orignal AVR
assembler, but [avra](http://avra.sourceforge.net/) is almost 100%
compatible):

	avra -I /usr/share/avra smc3.asm



## Wiring

Signal		|Pin ATtiny2313	|Pin m328	|Arduino Uno pin number
------		|--------	|-----------	|---------
Enc_A		|PD5/T1		|PD5/T1		|5
Enc_B		|PD4/T0		|PD4/T0		|4
Enc_Z		|PD3/INT1	|PD3/INT1	|3
nSTEP		|PD2/INT0	|PD2/INT0	|2
DIR		|PD6		|PD6/OC0A	|6
TxD		|PD1/TXD	|PD1/TXD	|1
RxD		|PD0/RXD	|PD0/RXD	|0
SCK (Clk)	|PB7/SCK/USCK	| PB5/SCK	|13
MISO (SegData)	|PB6/MISO/DO	| PB3/MOSI	|11 (intentionally swopped)
MOSI (DigitClk)	|PB5/MOSI/DI	| PB4/MISO	|12 (intentionally swopped)
Mot_n		|PB4/OC1B	| PB2/OC1B	|10
Mot_p		|PB3/OC1A	| PB1/OC1A	|9
LED_Ready	|PB2/OC0A	|PC2		|16 (flexible)
LED_TorqueLimit	|PB1		|PC3		|17 (flexible)
LED_ServoError	|PB0		|PB0		|8 (flexible)
RESn		|PA2/RES	|PC6/RES	|
Xtal2		|PA1/X2		|		|
Xtal1		|PA0/X1		|		|

The three LED connections can be easily reconfigured in the source code. All
other connections are very hard-coded and would require major changes.



## Usage

Connect position encoder and motor driver, start a serial terminal with
38400N1. Check if you can turn the motor on a 50% PWM signal:

	SMC type 3

	%m0
	%s128

Turn off the motor:

	%s0

The higher modes require the definition of some motor control parameters.
These values depend on the motor, the supply voltage, the encoder and the
masses involved.

Follow [this guide in the project
wiki](https://github.com/tenbaht/servo-motor-controller/wiki/Meaning-of-the-parameters)
on how to measure and calculate them. Once you have a set of numbers define
and save it:

	%p0 20
	%p1 256
	%p2 2560
	%p3 13
	%p4 240
	%p5 9
	%p6 4480
	%p7 16
	%w0

These values are stored in EEPROM and can be recalled any time by

	%r0

Now let's move! But be careful: This can physically crash your machine if
the speed and torque limits or the feedback values are incorrectly set!

Move to position 1000 following the speed ramp defined by P6
and P7:

	g0 1000

Move to position 100 at a constant speed of 10:

	g1 100 10

Jump to 500 at maximum speed (P0) and acceleration (only limited by the
current/torque limit P4):

	j 500


## Connection to the real world

Positions are in encoder counts and speeds are in encoder counts per
millisecond. A linear encoder strip from an old DeskJet printer with 150 lpi
(lines per inch) gives 600 encoder counts per inch = 42.3um/count.

- Position 1000 is at 42.3mm
- Speed 10 cnt/ms is 10000 cnt/s = 423mm/s.

For rotational encoder it all scales to rounds and rpm or rps instead of
distance and distance/time.



## Position display

The original source supports an optional 7-segment display with 8 digits to
show the current position. The supported serial protocol (see [Chan Elm's
page](http://elm-chan.org/docs/avr/avrisp.html) for details) is unfortunatly
not compatible with the cheap serial LED modules you can find on Aliexpress.

Since this is not an off-the-shelf standard display anyway I didn't bother
to port that code (yet?) and swopped these two serial pins for the Arduino
pin definition in a way that the builtin SPI could be used.



## CPU

The AT90S2313 got replaced by the ATtiny2313 a long time ago. Both are very
similar, only some register names changed slightly and the interrupt table
got a little bigger. See Atmel/Microchip AVR091 for details on migration.

Today I would choose one of the more modern CPUs in the 40 or 50 cent class
like ATtiny404, ATtiny44 or ATtiny45. 4kB flash should even allow for C code
and the hardware multiplier would help as well.

Using an ATmega328 is ridiculous overkill, but that's what I have laying
around in my drawer...




## Files in this repository

_atmega/_: the ATmega port of SMC3

_cport/_: parts of the assembler code translated into something very similar
to C. It won't compile, but it is more readable.

The original archives are unpacked for reference into chan/smc and chan/smc3.

_chan/smc/smc.asm_: v0.2 of the motor controller (AT90S2313)

_chan/smc/smc2.asm_: v0.2 of the motor controller with position display on a
7-segment display connected via SPI to the ISP connector. (AT90S2313)

_chan/smc3/SMC3.ASM_: v0.3, ATtiny2313. Supports serial interface, but not the
stepper-like dir/step input and no LED display. 'E' command not implemented,
echo always on.

_chan/smc3/SMC3A.ASM_: v0.3a, ATtiny2313. Supports the stepper-like dir/step
interface, but no serial interface and no LED display.



## Further reading

- [Internal operations of SCM3](https://github.com/tenbaht/servo-motor-controller/wiki)
  in the project wiki
- [ATtiny2313 data sheet](http://ww1.microchip.com/downloads/en/DeviceDoc/doc8246.pdf)
- AVR091 Replacing AT90S2313 by ATtiny2313.pdf
- [AVR Assember user guide](http://ww1.microchip.com/downloads/en/devicedoc/40001917a.pdf)
  (was known as Atmel doc1022 before the merger with Microchip)
- Theory of sensorless speed control of brushed DC motors: [AB-026 :
  Sensorless Speed Stabiliser for a DC
  Motor](https://www.precisionmicrodrives.com/content/ab-026-sensorless-speed-stabiliser-for-a-dc-motor/)
  by Precision Microdrives.


## Licence

The original work was released under the [Creative Commons Attribution 3.0 Unported]
(https://creativecommons.org/licenses/by/3.0/) (CC-BY 3.0) by Elm Chan.
