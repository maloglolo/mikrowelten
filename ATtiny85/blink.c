// blink.c
#include <avr/io.h>
#include <util/delay.h>

#define LED_PIN PB1

int main(void) {
    // Set PB1 as output
    DDRB |= (1 << LED_PIN);

    while (1) {
        // Toggle PB1
        PORTB ^= (1 << LED_PIN);
        _delay_ms(500);
    }
}
