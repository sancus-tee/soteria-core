#include "hardware.h"
#include "uart.h"
#include <stdio.h>

int putchar(int c)
{
    if (c == '\n')
        putchar('\r');

    uart_write_byte(c);
    return c;
}

int main(void) {

    WDTCTL = WDTPW | WDTHOLD;          // Disable watchdog timer

    uart_init();
    puts("hello world!");


    while(1);
    return 0;
}

