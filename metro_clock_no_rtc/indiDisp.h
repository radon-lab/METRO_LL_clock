#include <Arduino.h>
#include "font.c"

volatile uint8_t* anodePort[] = {&ANODE_A_PORT, &ANODE_B_PORT, &ANODE_C_PORT, &ANODE_D_PORT, &ANODE_E_PORT, &ANODE_F_PORT, &ANODE_G_PORT}; //таблица портов анодов индикатора
const uint8_t anodeBit[] = {0x01 << ANODE_A_BIT, 0x01 << ANODE_B_BIT, 0x01 << ANODE_C_BIT, 0x01 << ANODE_D_BIT, 0x01 << ANODE_E_BIT, 0x01 << ANODE_F_BIT, 0x01 << ANODE_G_BIT}; //таблица бит анодов индикатора

volatile uint8_t* cathodePort[] = {&CATHODE_1_PORT, &CATHODE_2_PORT, &CATHODE_3_PORT, &CATHODE_4_PORT}; //таблица портов катодов индикатора
const uint8_t cathodeBit[] = {0x01 << CATHODE_1_BIT, 0x01 << CATHODE_2_BIT, 0x01 << CATHODE_3_BIT, 0x01 << CATHODE_4_BIT}; //таблица бит катодов индикатора

uint8_t indi_buf[4];
uint8_t indi_dimm[4];
uint8_t flash_dimm[2];
boolean dot_state = 1;
boolean flask_state = 1;
volatile uint8_t indi_state;
volatile uint8_t tick_ms; //счетчик тиков для обработки данных

#define LEFT 0
#define RIGHT 255
#define CENTER 254

#define fontbyte(x) pgm_read_byte(&indiFont[x])

#define _INDI_ON  PRR &= ~(0x01 << PRTIM0); TIMSK0 = 0b00000111
#define _INDI_OFF TCNT0 = 128; TIMSK0 = 0b00000000; PRR |= (0x01 << PRTIM0)


void indiSet(uint8_t st, uint8_t indi, boolean state = 1);
void indiPrintNum(int16_t num, uint8_t indi, uint8_t length = 0, char filler = ' ');
//---------------------------------Генерация символов---------------------------------------
ISR(TIMER0_OVF_vect) //генерация символов
{
  OCR0A = indi_dimm[indi_state]; //устанавливаем яркость индикаторов
  switch (indi_state) {
    case 0:
      OCR0B = flash_dimm[0]; //устанавливаем яркость точек
      if (dot_state) DOT_ON; //включаем точки
      break;
    case 2:
      OCR0B = flash_dimm[1]; //устанавливаем яркость колбы
      if (flask_state) FLASK_ON; //включаем колбу
      break;
  }
  TCNT0 = 128; //сбрасываем счетчик таймера

  uint8_t buff = 0x80;
  for (uint8_t i = 0; i < 7; i++) {
    if (indi_buf[indi_state] & buff) *anodePort[i] |= anodeBit[i]; //включаем сегмент
    else *anodePort[i] &= ~anodeBit[i]; //выключаем сегмент
    buff >>= 0x01;
  }
  *cathodePort[indi_state] &= ~cathodeBit[indi_state]; //включаем индикатор

  tick_ms++;
}
ISR(TIMER0_COMPA_vect) {
  *cathodePort[indi_state] |= cathodeBit[indi_state]; //выключаем индикатор
  if (++indi_state > 3) indi_state = 0; //переходим к следующему индикатору
}
ISR(TIMER0_COMPB_vect) {
  DOT_OFF; //выключаем точки
  FLASK_OFF; //выключаем колбу
}
//-------------------------Инициализация индикаторов----------------------------------------------------
void indiInit(void) //инициализация индикаторов
{
  for (uint8_t i = 0; i < 7; i++) {
    *anodePort[i] &= ~anodeBit[i]; //выключаем сегмент
    *(anodePort[i] - 1) |= anodeBit[i];
  }

  for (uint8_t i = 0; i < 4; i++) {
    *cathodePort[i] |= cathodeBit[i]; //выключаем индикатор
    *(cathodePort[i] - 1) |= cathodeBit[i];
    indi_dimm[i] = 128 + brightDefault[0];
  }

  for (uint8_t i = 0; i < 2; i++) flash_dimm[i] = 128 + brightDefault[0];

  OCR0A = indi_dimm[0];
  OCR0B = flash_dimm[0];

  TCCR0A = 0b00000000; //отключаем OC0A/OC0B
  TCCR0B = 0b00000011; //пределитель 64
  TIMSK0 = 0b00000000; //отключаем прерывания Таймера0

  ASSR = (0x01 << AS2); //включаем асинхронный режим Таймера2

  while (ASSR & (0x01 << TCR2AUB));
  TCCR2A = 0b00000000; //отключаем OC2A/OC2B

  while (ASSR & (0x01 << TCR2BUB));
  TCCR2B = 0b00000101; //пределитель 128
  TIMSK2 = 0b00000001; //включаем прерывания Таймера0

  sei(); //разрешаем прерывания глобально
}
//---------------------------------Включение режима сна---------------------------------------
void indiEnableSleep(void) //включение режима сна
{
  _INDI_OFF; //отключаем генирацию
  for (uint8_t i = 0; i < 7; i++) *anodePort[i] &= ~anodeBit[i]; //выключаем сегмент
  for (uint8_t i = 0; i < 4; i++) *cathodePort[i] |= cathodeBit[i]; //выключаем индикатор
  dot_state = 0; //выключаем точки
  DOT_OFF; //выключаем точки
  FLASK_OFF; //выключаем колбу
}
//---------------------------------Выключение режима сна---------------------------------------
void indiDisableSleep(void) //выключение режима сна
{
  for (uint8_t i = 0; i < 4; i++) indi_buf[i] = 0; //очищаем буфер
  _INDI_ON; //запускаем генерацию
}
//---------------------------------Установка яркости индикатора---------------------------------------
void indiSetBright(uint8_t indi, uint8_t pwm) //установка яркости индикатора
{
  if (!pwm) pwm = 1;
  indi_dimm[indi] = 128 + pwm;
}
//---------------------------------Установка яркости светодиодов---------------------------------------
void flashSetBright(uint8_t flash, uint8_t pwm) //установка яркости светодиодов
{
  if (!pwm) pwm = 1;
  flash_dimm[flash] = 128 + pwm;
}
//---------------------------------Установка общей яркости---------------------------------------
void indiSetBright(uint8_t pwm) //установка общей яркости
{
  if (!pwm) pwm = 1;
  for (uint8_t i = 0; i < 4; i++) {
    indi_dimm[i] = 128 + pwm;
  }
  for (uint8_t i = 0; i < 2; i++) {
    flash_dimm[i] = 128 + pwm;
  }
}
//-------------------------Очистка индикаторов----------------------------------------------------
void indiClr(void) //очистка индикаторов
{
  for (uint8_t i = 0; i < 4; i++) indi_buf[i] = 0;
}
//-------------------------Очистка индикатора----------------------------------------------------
void indiClr(uint8_t indi) //очистка индикатора
{
  indi_buf[indi] = 0;
}
//-------------------------Вывод символов----------------------------------------------------
void indiSet(uint8_t st, uint8_t indi, boolean state) //вывод символов
{
  BIT_WRITE(indi_buf[indi], 7 - st, state);
}
//-------------------------Вывод текста----------------------------------------------------
void indiPrint(const char *st, uint8_t indi) //вывод текста
{
  uint8_t stl = strlen(st);

  switch (indi) {
    case RIGHT: indi = 4 - stl; break;
    case CENTER: indi = 4 - stl / 2; break;
  }

  for (int cnt = 0; cnt < stl; cnt++) indi_buf[indi++] = fontbyte(st[cnt] - 32);
}
//-------------------------Вывод чисел----------------------------------------------------
void indiPrintNum(int16_t num, uint8_t indi, uint8_t length, char filler) //вывод чисел
{
  char buf[5];
  char st[5];
  boolean neg = 0;
  uint8_t c = 0, f = 0;

  if (!num) {
    if (length) {
      for (c = 0; c < (length - 1); c++) st[c] = filler;
      st[c] = 48;
      st[c + 1] = 0;
    }
    else {
      st[0] = 48;
      st[1] = 0;
    }
  }
  else {
    if (num < 0) {
      neg = 1;
      num = -num;
    }
    while (num > 0) {
      buf[c] = 48 + (num % 10);
      c++;
      num = (num - (num % 10)) / 10;
    }
    buf[c] = 0;

    if (neg) st[0] = 45;

    if (length > (c + neg)) {
      for (uint8_t i = 0; i < (length - c - neg); i++) {
        st[i + neg] = filler;
        f++;
      }
    }
    for (uint8_t i = 0; i < c; i++) st[i + neg + f] = buf[c - i - 1];
    st[c + neg + f] = 0;
  }
  indiPrint(st, indi);
}
