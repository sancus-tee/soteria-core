#include <stdlib.h>
#include <stdio.h>

#include "hardware.h"


//--------------------------------------------------
int putchar(int c) {
	while (UART_STAT & UART_TX_FULL);

	UART_TXD = c;

	return c;
}


//--------------------------------------------------
int getchar(void) {
	int c;

	while (!(UART_STAT & UART_RX_PND));

	c = UART_RXD;

	UART_STAT = UART_RX_PND;

	return c;
}


//--------------------------------------------------
int main(void) {
	int c;

	// disable watchdog timer
	WDTCTL = WDTPW | WDTHOLD;

	// show some light
	P3DIR  = 0x0f;
	P3OUT  = 0x0f;

	// init uart
	UART_BAUD = BAUD;
	UART_CTL  = UART_EN;

	P3OUT  = 0x01;

	printf("UART test running...\n");
	printf("Starting simple echo test...\n");

	P3OUT  = 0x03;

	while (1) {
		printf("> ");
		while (1) {
			c = getchar();
			putchar(c);
			if (c == '\n')
				break;
		}
	}
}
