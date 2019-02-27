#ifndef _UART_H_
#define _UART_H_

#ifdef __cplusplus
extern "C" {
#endif


#define USE_ARDUINO

#ifdef USE_ARDUINO
uint8_t Serial_available();
uint8_t Serial_read();
void Serial_write(uint8_t);
void Serial_print_c(char);
void Serial_print_i(long);
void Serial_print_s(const char *);
#endif



//; Host command
#define echo xmit
void xmit(char);	//	transmit one byte

#ifdef __cplusplus
}
#endif

#endif
