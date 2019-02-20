/*
 * wrapper layer to access the Arduino Serial class from C
 *
 * This is a replacement for the actual UART implementation.
 *
 */

#include "Arduino.h"
#include "HardwareSerial.h"

#include "uart.h"





#ifdef USE_ARDUINO

#ifdef __cplusplus
extern "C" {
#endif

uint8_t Serial_available(){return Serial.available();}
uint8_t Serial_read(){return Serial.read();}
void Serial_write(uint8_t c){Serial.print(c);}
void Serial_print_c(char c){Serial.print(c);}
void Serial_print_i(long v){Serial.print(v);}
void Serial_print_s(const char *s){Serial.print(s);}


// #define xmit Serial_write_c
void xmit(char c)
{
	Serial_print_c(c);
}

#ifdef __cplusplus
}
#endif

#else
 #error "no implementation for xmit()"
#endif
