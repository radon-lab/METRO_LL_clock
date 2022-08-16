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
