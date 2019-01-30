This work is based on the DC Servomotor Controller by Elm Chan:
http://elm-chan.org/works/smc/report_e.html

## Files in this repository

The original archives are unpacked into chan/smc and chan/smc3.

chan/smc/smc.asm: v0.2 of the motor controller (AT90S2313)

chan/smc/smc2.asm: v0.2 of the motor controller with position display on a
7-segment display connected via SPI to the ISP connector. (AT90S2313)

chan/smc3/SMC3.ASM: v0.3, ATtiny2313. Supports serial interface, but not the
stepper-like dir/step input and no LED display. 'E' command not implemented,
echo always on.

chan/smc3/SMC3A.ASM: v0.3a, ATtiny2313. Supports the stepper-like dir/step
interface, but no serial interface and no LED display.


## Compiling

Chan wrote his files for the orignal AVR assembler. The open source
alternative [avra](http://avra.sourceforge.net/) can be used as well:

	avra -I /usr/share/avra smc3.asm



## Features of SMC3

 o Using Only a Cheap ATtiny2313 Microcontroller
 o Controlled in Serial Command via UART
   - Direct Access to Position Command Register
   - G0/G1 Motion Generation
 o Three Control Modes
   - Absolute Positioning Mode
   - Constant Speed Mode
   - Constant Torque Mode
 o Software Decoding of Quadrature Encoder Input
   - Count rate over 100,000 counts/sec in quadrature decoding
   - No External Component Required
 o Two PWM Outputs for H-bridge Driver
 o Supports bootstrap type FET driver (minimum duty ratio 15/16)
 o Three Diagnostic Outputs
   - Ready, Torque Limit and Servo Error
 o On-the-Fly Servo Tuning

Changes from SMC to SMC3

 o Removed Pulsed Input
 o Added Diagnostic Outputs
   - Ready: Servo control is running.
   - Torque Limit: Torque limit indicator for mode 2/3.
   - Servo Error: Servo error occured and enterd to mode 0. M command or
     reset can restart servo.


## Features of SMC3A

 o Using Only a Cheap ATtiny2313 Microcontroller
 o Optimized for Replacement of Stepping Motors
   - Pulse/Dir Command Input with Pulse Multiplyer
   - Pulse rate over 100,000 pulses/sec
   - Gear ratio from 1/256 to 255 in 1/256 step
 o Three Control Modes
   - Relative Positioning Mode
   - Constant Speed Mode
   - Constant Torque Mode
 o Software Decoding of Quadrature Encoder Input
   - Count rate over 100,000 counts/sec in quadrature decoding
   - No External Component is Required
 o Two PWM Outputs for H-bridge Driver
 o Supports bootstrap type FET driver (minimum duty ratio 15/16)
 o Three Diagnostic Outputs
   - Ready, Torque Limit and Servo Error
 o On-the-Fly Servo Tuning


Changes from SMC to SMC3A

 o Removed G0/G1 Commands
 o Added +/- Commands
   - {+|-}<value>: Increase/Decrease command counter.
 o Extened Pulsed Input Function
   - Pulse Multiplier: Gear ratio (8.8 fixed-point) can be set with parameter
     #6, 256 means 1.0. Parameter #7 has no effect.
 o Added Diagnostic Outputs
   - Ready: Servo control is running.
   - Torque Limit: Torque limit indicator for mode 2/3.
   - Servo Error: Servo error occured and enterd to mode 0. M command or
     reset can restart servo.
 


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


## Position display

The current position is displayed on an optional 7-segment display with 8
digits. This display is unfortunatly not compatible with the cheap serial
LED modules you can find on Aliexpress.

It is attached to the SPI pins and uses an unusual serial protocol. SCK is
the clock signal, the segment data is shifted out on MISO. Yes, no typo.
MISO is used as an output. After 8 clock pulses one pulse on MOSI (finally)
advances to the next digit. Checkout the schematic on [Chan Elm's
page](http://elm-chan.org/docs/avr/avrisp.html) for details.

Since this is not an off-the-shelf standard display anyway I swopped these
two pins for the Arduino pin definition. This allows to use the builtin SPI.


## CPU

The AT90S2313 got replaced by the ATtiny2313 a long time ago. Both are very
similar, only some register names changed slightly and the interrupt table
got a little bigger. See Atmel/Microchip AVR091 for details on migration.

Today I would choose one of these AVR CPUs (based on price and
availability):

- ATtiny25: cheapest AVR today, but QFN only
- ATtiny44: SO14, 4kB flash, only slightly more expensive
- ATtiny45: SO20


## Further reading

[ATtiny2313 data sheet](http://ww1.microchip.com/downloads/en/DeviceDoc/doc8246.pdf)

AVR091 Replacing AT90S2313 by ATtiny2313.pdf

[AVR Assember user guide](http://ww1.microchip.com/downloads/en/devicedoc/40001917a.pdf)
(was known as Atmel doc1022 before the merger with Microchip)

Theory of sensorless speed control of brushed DC motors: [AB-026 :
Sensorless Speed Stabiliser for a DC
Motor](https://www.precisionmicrodrives.com/content/ab-026-sensorless-speed-stabiliser-for-a-dc-motor/)
by Precision Microdrives.


## Licence

The original work was released under the [Creative Commons Attribution 3.0 Unported]
(https://creativecommons.org/licenses/by/3.0/) (CC-BY 3.0) by Elm Chan.
