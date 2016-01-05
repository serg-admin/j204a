#include <avr/io.h>
#include <avr/interrupt.h>
//#define F_CPU 8000000UL
//#include <util/delay.h>

#define PORT_DATA PORTD
#define PORT_OTHER PORTC

#define PIN_V0 PORTC1
#define PIN_RS PORTC0
#define PIN_RW PORTC3
#define PIN_E PORTC2

void _delay_ms(uint16_t ms) {
  uint16_t delay;
  for (;ms; ms--) {
    delay = 2000;
    for(; delay; delay--){
      asm("");
    }
  }
}

uint8_t halfchar_to_hex(uint8_t tt) {
  if (tt < 10) return tt + 48;
  switch (tt) {
    case 10 : return 'A'; 
    case 11 : return 'B';
    case 12 : return 'C';
    case 13 : return 'D';
    case 14 : return 'E';
    case 15 : return 'F';
  }
  return 'X';
}

void j204a_applay(uint8_t rs, uint8_t rw, uint8_t data) {
    PORT_OTHER |= _BV(PIN_E);
    if (rs) PORT_OTHER |= _BV(PIN_RS);
    else PORT_OTHER &= ~(_BV(PIN_RS));
    if (rw) PORT_OTHER |= _BV(PIN_RW);
    else PORT_OTHER &= ~(_BV(PIN_RW));
    PORT_DATA = data ;
    _delay_ms(1);
    PORT_DATA &= ~(_BV(PORT7));;
    PORT_OTHER &= ~(_BV(PIN_E));
    _delay_ms(1);
}

int main(void) {
  sei();
  uint8_t tmp = 0;
  DDRD = 0xFF;
  DDRC = 0xFF;
  PORTA = 0xFF;
  _delay_ms(500);
  PORT_DATA = 0;

  j204a_applay(0, 0, 0b00001100); // Настройка 1DCB
  //j204a_applay(0, 0, 0b00111100); // Настройка 1DCB
  //j204a_applay(0, 0, 0b10000000); // set ddram address
  j204a_applay(0, 0, 0b10000000); // set ddram address
  
  j204a_applay(0, 0, 0b01001000); // set CGRAM address
  
  j204a_applay(1, 0, 0b00011111);  
  j204a_applay(1, 0, 0b00010001);  
  j204a_applay(1, 0, 0b00010101);  
  j204a_applay(1, 0, 0b00010101);  
  j204a_applay(1, 0, 0b00010101);  
  j204a_applay(1, 0, 0b00010101);  
  j204a_applay(1, 0, 0b00010001);  
  j204a_applay(1, 0, 0b00011111);  
  
  j204a_applay(0, 0, 0b00000010); // Курсор домой
  for (;;) {
    j204a_applay(1, 0, tmp);
    j204a_applay(1, 0, halfchar_to_hex(tmp >> 4));
    j204a_applay(1, 0, halfchar_to_hex(tmp & 0x0F));
    tmp++;


    if (((~tmp) & 0b00000111) == 0x07) {
      j204a_applay(0, 0, 0b00000010); // Курсор домой
      _delay_ms(500);
      while (PINA == 0xFF); 
      switch(PINA) {
        case 0b11111110 :
          break;
        case 0b11111101 :
          tmp = tmp - 8;
          break;
        case 0b11111011 :
          tmp = tmp - 16;
          break;
      }
      j204a_applay(0, 0, 0b00000001); // set ddram address  
    }
  }
}