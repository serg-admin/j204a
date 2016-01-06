#include <avr/io.h>
#include <avr/interrupt.h>
#include <tools/uart_async.h>

#define PORT_DATA PORTD
#define PORT_OTHER PORTC

#define PIN_V0 PORTC1
#define PIN_RS PORTC0
#define PIN_RW PORTC3
#define PIN_E PORTC2

#define J204A_CLEAR_DISPLAY j204a_applay(0, 0, 0b00000001)
#define J204A_CURSOR_TO_HOME j204a_applay(0, 0, 0b00000010)
#define J204A_PRINT(c) j204a_applay(1, 0, c)
#define J204A_WIDTH 20

// Z- буфер дисплея
uint8_t j204a_ram[4][J204A_WIDTH];

void _delay_ms(uint16_t ms) {
  uint16_t delay;
  for (;ms; ms--) {
    delay = 2000;
    for(; delay; delay--){
      asm("");
    }
  }
}

void _delay_us(uint16_t us) {
  uint16_t delay;
  for (;us; us--) {
    delay = 2;
    for(; delay; delay--){
      asm("");
    }
  }
}

/**
 * @brief Приводит шину данных дисплея в указанное состояние
 * @param rs Бит регистра (регистр команд/регистр данных)
 * @param rw Бит направления (чтение/запись)
 * @param data Команда/Данные
 */
void j204a_applay(uint8_t rs, uint8_t rw, uint8_t data) {
    // Приводим шину данных в требующееся состояние
    // и ждем 25 микросекунд.
    if (rs) PORT_OTHER |= _BV(PIN_RS);
    else PORT_OTHER &= ~(_BV(PIN_RS));
    if (rw) PORT_OTHER |= _BV(PIN_RW);
    else PORT_OTHER &= ~(_BV(PIN_RW));
    PORT_DATA = data ;
    PORT_OTHER |= _BV(PIN_E); // Старт транзакции
    _delay_us(25);
    
    // Отмечаем завершение транзакции и ждем еще 25 микросекунд.
    PORT_DATA &= ~(_BV(PORT7));
    PORT_OTHER &= ~(_BV(PIN_E));
    _delay_us(25);
}

void j204a_refresh(void) {
  uint8_t line = 0;
  uint8_t col;
  J204A_CURSOR_TO_HOME;
  _delay_ms(2);
  while(line < 4) {
    for(col = 0; col < J204A_WIDTH; col++) {
      J204A_PRINT(j204a_ram[line][col]);
    }
    // В имеющемся дисплеи строки отображаются в видеопамять
    // "черезстрочно"  0-2-1-3
    switch(line) { 
      case 0 : line = 2; break;
      case 1 : line = 3; break;
      case 2 : line = 1; break;
      case 3 : line = 4; break; // Выход из цыкла.
    }
  }
}

void j204a_clear(void) {
  uint8_t n;
  J204A_CLEAR_DISPLAY;
  for(n = 0; n < (J204A_WIDTH * 4); n++) {
    ((uint8_t*)j204a_ram)[n] = 0x20; // Забиваем z-буфер пробелами.
  }
}

/**
 * @brief Выводит строку символов в начиная с указанной 
 *        позиции дисплея.
 * @param col Номер колонки дисплея 0-19.
 * @param line Номер строка дисплея 0-3.
 * @param str Выводимая строка символов.
 */
void j204aPrint(uint8_t col, uint8_t line, char* str){
  uint8_t n = 0;
  while(str[n]) {
    j204a_ram[line][col++] = str[n++];
    if (col >= J204A_WIDTH) {
      col = 0;
      line++;
    }
  }
}

void j204aInit(void){
  DDRD = 0xFF;  // Data пины
  DDRC = 0xFF;  // Управляющие пины  
  PORT_DATA = 0;
  _delay_ms(20); // Даем дисплею время включиться
  j204a_applay(0, 0, 0b00001100); // Настройка 1,D,C,B
  _delay_ms(2);
  j204a_applay(0, 0, 0b00010100); 
  _delay_ms(2);
   // Настройка 1,DL,N,F,*,*
   //    DL - 1 : 8-ми проводная шина.
   //         0 : 4-х проводная шина.
   //     N - 1 : Черезстрочный ?
   //         0 : Однострочный
   //     F - 1 : матрица символа 5x10
   //         0 : матрица символа 5x7 
  j204a_applay(0, 0, 0b00111000); 
  _delay_ms(2);
  j204a_clear();
}

int main(void) {
  j204aInit();
  
  j204aPrint(4, 2, "Hello World.");
  
  j204a_refresh();
}