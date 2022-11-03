#include <Arduino.h>
#include "font.c"

volatile uint8_t* segPort[] = {&SEG_A_PORT, &SEG_B_PORT, &SEG_C_PORT, &SEG_D_PORT, &SEG_E_PORT, &SEG_F_PORT, &SEG_G_PORT}; //таблица портов сегмента индикатора
const uint8_t segBit[] = {0x01 << SEG_A_BIT, 0x01 << SEG_B_BIT, 0x01 << SEG_C_BIT, 0x01 << SEG_D_BIT, 0x01 << SEG_E_BIT, 0x01 << SEG_F_BIT, 0x01 << SEG_G_BIT}; //таблица бит сегмента индикатора

volatile uint8_t* indiPort[] = {&INDI_1_PORT, &INDI_2_PORT, &INDI_3_PORT, &INDI_4_PORT}; //таблица портов индикаторов
const uint8_t indiBit[] = {0x01 << INDI_1_BIT, 0x01 << INDI_2_BIT, 0x01 << INDI_3_BIT, 0x01 << INDI_4_BIT}; //таблица бит индикаторов

uint8_t indi_buf[4];
uint8_t indi_dimm[4];
uint8_t flash_dimm[3];
boolean pm_state = 1;
boolean dot_state = 1;
boolean flask_state = 1;
volatile uint8_t indi_state;
volatile uint8_t tick_ms; //счетчик тиков для обработки данных

#define fontbyte(x) pgm_read_byte(&indiFont[x])

#define _INDI_START  PRR &= ~(0x01 << PRTIM0); TIMSK0 = (0x01 << OCIE0B) | (0x01 << OCIE0A) | (0x01 << TOIE0)
#define _INDI_STOP TCNT0 = 128; TIMSK0 = 0; PRR |= (0x01 << PRTIM0)

#if INDI_CONNECT_MODE
#define _OUT_SET(port, bit) (*port &= ~bit)
#define _OUT_CLEAR(port, bit) (*port |= bit)
#else
#define _OUT_SET(port, bit) (*port |= bit)
#define _OUT_CLEAR(port, bit) (*port &= ~bit)
#endif
#define _OUT_PORT(port, bit) (*(port - 1) |= bit)

void indiSet(uint8_t st, uint8_t indi, boolean state = 1);
void indiPrint(const char* str, int8_t indi); //отрисовка символов
void indiPrintNum(int16_t num, int8_t indi, uint8_t length = 0, char filler = ' '); //отрисовка чисел
//--------------------------Генерация символов-------------------------------
ISR(TIMER0_OVF_vect) //генерация символов
{
  OCR0A = indi_dimm[indi_state]; //устанавливаем яркость индикаторов
  switch (indi_state) {
    case 0:
      OCR0B = flash_dimm[0]; //устанавливаем яркость точек
      if (dot_state) DOT_ON; //включаем точки
      break;
    case 1:
      OCR0B = flash_dimm[1]; //устанавливаем яркость колбы
      if (flask_state) FLASK_ON; //включаем колбу
      break;
    case 2:
      OCR0B = flash_dimm[2]; //устанавливаем яркость точек
      if (pm_state) DOT_PM_ON; //включаем точки
      break;
  }
  TCNT0 = 128; //сбрасываем счетчик таймера

  uint8_t buff = 0x80;
  for (uint8_t i = 0; i < 7; i++) {
    if (indi_buf[indi_state] & buff) _OUT_SET(segPort[i], segBit[i]); //включаем сегмент
    else _OUT_CLEAR(segPort[i], segBit[i]); //выключаем сегмент
    buff >>= 0x01;
  }
  _OUT_CLEAR(indiPort[indi_state], indiBit[indi_state]); //включаем индикатор

  tick_ms++;
}
ISR(TIMER0_COMPA_vect) {
  _OUT_SET(indiPort[indi_state], indiBit[indi_state]); //выключаем индикатор
  if (++indi_state > 3) indi_state = 0; //переходим к следующему индикатору
}
ISR(TIMER0_COMPB_vect) {
  DOT_OFF; //выключаем точки
  DOT_PM_OFF; //выключаем точку
  FLASK_OFF; //выключаем колбу
}
//-----------------------Инициализация индикаторов------------------------------
void indiInit(void) //инициализация индикаторов
{
  for (uint8_t i = 0; i < 7; i++) {
    _OUT_CLEAR(segPort[i], segBit[i]); //выключаем сегмент
    _OUT_PORT(segPort[i], segBit[i]); //настраиваем как выход
  }

  for (uint8_t i = 0; i < 4; i++) {
    _OUT_SET(indiPort[i], indiBit[i]); //выключаем индикатор
    _OUT_PORT(indiPort[i], indiBit[i]); //настраиваем как выход
    indi_dimm[i] = 128 + brightDefault[0]; //устанавливаем яркость
  }

  for (uint8_t i = 0; i < 3; i++) flash_dimm[i] = 128 + brightDefault[0];

  OCR0A = indi_dimm[0];
  OCR0B = flash_dimm[0];

  TCCR0A = 0; //отключаем OC0A/OC0B
  TCCR0B = (0x01 << CS01) | (0x01 << CS00); //пределитель 64
  TIMSK0 = 0; //отключаем прерывания Таймера0

  ASSR = (0x01 << AS2); //включаем асинхронный режим Таймера2

  while (ASSR & (0x01 << TCR2AUB));
  TCCR2A = 0; //отключаем OC2A/OC2B

  while (ASSR & (0x01 << TCR2BUB));
  TCCR2B = (0x01 << CS22) | (0x01 << CS20); //пределитель 128
  TIMSK2 = (0x01 << TOIE2); //включаем прерывания Таймера0

  sei(); //разрешаем прерывания глобально
}
//-------------------------Включение режима сна-------------------------------
void indiEnableSleep(void) //включение режима сна
{
  _INDI_STOP; //отключаем индикацию
  for (uint8_t i = 0; i < 7; i++) _OUT_CLEAR(segPort[i], segBit[i]); //выключаем сегмент
  for (uint8_t i = 0; i < 4; i++) _OUT_SET(indiPort[i], indiBit[i]); //выключаем индикатор
  dot_state = 0; //выключаем точки
  DOT_OFF; //выключаем точки
  DOT_PM_OFF; //выключаем точку
  FLASK_OFF; //выключаем колбу
}
//------------------------Выключение режима сна--------------------------------
void indiDisableSleep(void) //выключение режима сна
{
  for (uint8_t i = 0; i < 4; i++) indi_buf[i] = 0; //очищаем буфер
  _INDI_START; //запускаем генерацию
}
//-------------------Установка яркости индикатора------------------------------
void indiSetBright(uint8_t indi, uint8_t pwm) //установка яркости индикатора
{
  if (!pwm) pwm = 1;
  indi_dimm[indi] = 128 + pwm;
}
//-------------------Установка яркости светодиодов-----------------------------
void flashSetBright(uint8_t flash, uint8_t pwm) //установка яркости светодиодов
{
  if (!pwm) pwm = 1;
  flash_dimm[flash] = 128 + pwm;
}
//-----------------------Установка общей яркости-------------------------------
void indiSetBright(uint8_t pwm) //установка общей яркости
{
  if (!pwm) pwm = 1;
  for (uint8_t i = 0; i < 4; i++) {
    indi_dimm[i] = 128 + pwm;
  }
  for (uint8_t i = 0; i < 3; i++) {
    flash_dimm[i] = 128 + pwm;
  }
}
//-------------------------Очистка индикаторов---------------------------------
void indiClr(void) //очистка индикаторов
{
  for (uint8_t i = 0; i < 4; i++) indi_buf[i] = 0;
}
//-------------------------Очистка индикатора----------------------------------
void indiClr(uint8_t indi) //очистка индикатора
{
  indi_buf[indi] = 0;
}
//-------------------------Вывод символов--------------------------------------
void indiSet(uint8_t st, uint8_t indi, boolean state) //вывод символов
{
  BIT_WRITE(indi_buf[indi], 7 - st, state);
}
//------------------------Заполнение буфера------------------------------------
void indiPrint(uint8_t* _buff, int8_t _indi, uint8_t _count) //заполнение буфера
{
  while (_count) { //декодирование символов
    _count--; //убавили счетчик символов
    if ((uint8_t)_indi < 4) indi_buf[_indi] = pgm_read_byte(&indiFont[_buff[_count]]); //если число в поле индикатора то устанавливаем его
    _indi++; //сместили номер индикатора
  }
}
//-------------------------Вывод символов--------------------------------------
void indiPrint(const char* str, int8_t _indi) //вывод символов
{
  uint8_t count = 0; //счетчик символов

  while (*str && (count < 6)) { //если есть число
    if ((uint8_t)_indi < 4) indi_buf[_indi] = pgm_read_byte(&indiFont[(*str - 32)]); //если число в поле индикатора то устанавливаем его
    count++; //смещаем счетчик
    str++; //смещаем номер символа
    _indi++; //сместили номер индикатора
  }
}
//--------------------------Вывод чисел----------------------------------------
void indiPrintNum(int16_t _num, int8_t _indi, uint8_t _length, char _filler) //вывод чисел
{
  uint8_t buff[6]; //временный буфер
  uint8_t count = 0; //счетчик символо

  if (!_num) { //если ноль
    buff[0] = 16; //устанавливаем ноль
    count = 1; //прибавляем счетчик
  }
  else { //иначе заполняем буфер числами
    boolean neg = 0; //флаг отрицательного числа

    if (_num < 0) {
      neg = 1;
      _num = -_num;
    }

    while (_num && (count < 6)) { //если есть число
      buff[count++] = (_num % 10) + 16; //забираем младший разряд в буфер
      _num /= 10; //отнимаем младший разряд от числа
    }

    if (neg) buff[count++] = 13; //добавили минус
  }

  while ((_length > count) && (count < 6)) buff[count++] = _filler - 32; //заполняем символами заполнителями

  while (count) { //декодирование символов
    count--; //убавили счетчик символов
    if ((uint8_t)_indi < 4) indi_buf[_indi] = pgm_read_byte(&indiFont[buff[count]]); //если число в поле индикатора то устанавливаем его
    _indi++; //сместили номер индикатора
  }
}
