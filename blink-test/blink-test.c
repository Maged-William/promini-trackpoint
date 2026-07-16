#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>

int main(void) {
    DDRB |= _BV(PB5);
    while (1) {
        PORTB |= _BV(PB5);
        _delay_ms(100);
        PORTB &= ~_BV(PB5);
        _delay_ms(100);
        PORTB |= _BV(PB5);
        _delay_ms(100);
        PORTB &= ~_BV(PB5);
        _delay_ms(100);
        _delay_ms(1000);
    }
}
