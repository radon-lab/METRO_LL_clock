#include <avr/pgmspace.h>

#define SEGMENT_A 0x80 //номер сегмента A
#define SEGMENT_B 0x40 //номер сегмента B
#define SEGMENT_C 0x20 //номер сегмента C
#define SEGMENT_D 0x10 //номер сегмента D
#define SEGMENT_E 0x08 //номер сегмента E
#define SEGMENT_F 0x04 //номер сегмента F
#define SEGMENT_G 0x02 //номер сегмента G
#define SEGMENT_NULL 0x00 //пустой байт шрифта

/*    A  ---  A
    F  ||   || B
    F  ||   || B
      G  ---  G
    E  ||   || C
    E  ||   || C
      D  ---  D
*/

//--------------Массив шрифтов--------------
const uint8_t indiFont[] PROGMEM =
{
  SEGMENT_NULL,                                                                      // sp
  SEGMENT_B | SEGMENT_C,                                                             // !
  SEGMENT_B | SEGMENT_F,                                                             // "
  SEGMENT_A | SEGMENT_B | SEGMENT_F | SEGMENT_G,                                     // #
  SEGMENT_NULL,                                                                      // $
  SEGMENT_NULL,                                                                      // %
  SEGMENT_NULL,                                                                      // &
  SEGMENT_NULL,                                                                      // '
  SEGMENT_B | SEGMENT_C,                                                             // (
  SEGMENT_F | SEGMENT_G,                                                             // )
  SEGMENT_NULL,                                                                      // *
  SEGMENT_NULL,                                                                      // +
  SEGMENT_C,                                                                         // ,
  SEGMENT_G,                                                                         // -
  SEGMENT_NULL,                                                                      // .
  SEGMENT_B | SEGMENT_E | SEGMENT_G,                                                 // /

  SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F,             // 0
  SEGMENT_B | SEGMENT_C,                                                             // 1
  SEGMENT_A | SEGMENT_B | SEGMENT_D | SEGMENT_E | SEGMENT_G,                         // 2
  SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_G,                         // 3
  SEGMENT_B | SEGMENT_C | SEGMENT_F | SEGMENT_G,                                     // 4
  SEGMENT_A | SEGMENT_C | SEGMENT_D | SEGMENT_F | SEGMENT_G,                         // 5
  SEGMENT_A | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G,             // 6
  SEGMENT_A | SEGMENT_B | SEGMENT_C,                                                 // 7
  SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G, // 8
  SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_F | SEGMENT_G,             // 9
  SEGMENT_NULL,                                                                      // :
  SEGMENT_NULL,                                                                      // ;
  SEGMENT_NULL,                                                                      // <
  SEGMENT_D | SEGMENT_G,                                                             // =
  SEGMENT_NULL,                                                                      // >
  SEGMENT_NULL,                                                                      // ?

  SEGMENT_NULL,                                                                      // @
  SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_E | SEGMENT_F | SEGMENT_G,             // A
  SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G,                         // B
  SEGMENT_A | SEGMENT_D | SEGMENT_E | SEGMENT_F,                                     // C
  SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_G,                         // D
  SEGMENT_A | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G,                         // E
  SEGMENT_A | SEGMENT_E | SEGMENT_F | SEGMENT_G,                                     // F
  SEGMENT_A | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F,                         // G
  SEGMENT_C | SEGMENT_E | SEGMENT_F | SEGMENT_G,                                     // H
  SEGMENT_E | SEGMENT_F,                                                             // I
  SEGMENT_B | SEGMENT_C | SEGMENT_D,                                                 // J
  SEGMENT_A,                                                                         // K
  SEGMENT_D | SEGMENT_E | SEGMENT_F,                                                 // L
  SEGMENT_A,                                                                         // M
  SEGMENT_C | SEGMENT_E | SEGMENT_G,                                                 // N
  SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_G,                                     // O

  SEGMENT_A | SEGMENT_B | SEGMENT_E | SEGMENT_F | SEGMENT_G,                         // P
  SEGMENT_A,                                                                         // Q
  SEGMENT_E | SEGMENT_G,                                                             // R
  SEGMENT_A | SEGMENT_C | SEGMENT_D | SEGMENT_F | SEGMENT_G,                         // S
  SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G,                                     // T
  SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F,                         // U
  SEGMENT_C | SEGMENT_D | SEGMENT_E,                                                 // V
  SEGMENT_A,                                                                         // W
  SEGMENT_B | SEGMENT_C | SEGMENT_E | SEGMENT_F | SEGMENT_G,                         // X
  SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_F | SEGMENT_G,                         // Y
  SEGMENT_A | SEGMENT_B | SEGMENT_D | SEGMENT_E | SEGMENT_G,                         // Z
  SEGMENT_A | SEGMENT_D | SEGMENT_E | SEGMENT_F,                                     // [
  SEGMENT_A,                                                                         // Backslash
  SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D,                                     // ]
  SEGMENT_A | SEGMENT_B | SEGMENT_F,                                                 // ^
  SEGMENT_D                                                                          // _
};

const uint8_t numbersForAnim [10][5] = {
  {0B10000000, 0B01000110, 0B11111100, 0B00101010, 0B10000000},   // 0
  {0B00000000, 0B01000000, 0B01100000, 0B00100000, 0B00000000},   // 1
  {0B10000000, 0B10000110, 0B11011010, 0B00110010, 0B10000000},   // 2
  {0B10000000, 0B11000010, 0B11110010, 0B00110010, 0B10000000},   // 3
  {0B00000000, 0B01000000, 0B01100110, 0B00111000, 0B00000000},   // 4
  {0B10000000, 0B11000010, 0B10110110, 0B00011010, 0B10000000},   // 5
  {0B10000000, 0B11000110, 0B10111110, 0B00011010, 0B10000000},   // 6
  {0B00000000, 0B01000000, 0B11100000, 0B00100010, 0B10000000},   // 7
  {0B10000000, 0B11000110, 0B11111110, 0B00111010, 0B10000000},   // 8
  {0B10000000, 0B11000010, 0B11110110, 0B00111010, 0B10000000}    // 9
};
