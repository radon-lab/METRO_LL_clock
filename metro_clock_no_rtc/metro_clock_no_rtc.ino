/*
  Arduino IDE 1.8.13 версия прошивки 2.1.1 от 30.10.22
  Специльно для проекта "Часы METRO LAST LIGHT"
  Версия без DS1307, встроенный кварц 8мГц + внешний 32кГц
  Исходник - https://github.com/radon-lab/METRO_LL_clock
  Автор Radon-lab.
*/

//-----------------Частота------------------
#ifdef FCPU
#undef FCPU
#define FCPU 8000000
#endif

//----------------Библиотеки----------------
#include <util/delay.h>

//---------------Конфигурации---------------
#include "config.h"
#include "connection.h"
#include "indiDisp.h"
#include "EEPROM.h"

//переменные обработки кнопок
uint16_t btn_tmr = 0; //таймер тиков обработки
boolean btn_check = 0; //флаг разрешения опроса кнопки
boolean btn_state = 0; //флаг текущего состояния кнопки

uint8_t bat = 100; //заряд акб
uint8_t bat_tmr = 0; //таймер опроса батареи

uint16_t timer_millis = 0; //таймер отсчета миллисекунд
uint16_t timer_dot = 0; //таймер отсчета миллисекунд для точек

uint8_t _mode = 0; //текущий основной режим
uint8_t _msg_type = 0; //тип оповещения
boolean _animStart = 1; //флаг анимации

uint16_t _timer_sec = 0; //установленное время таймера
boolean _timer_start = 0; //флаг работы таймера

boolean _scr = 0; //флаг обновления индикаторов
boolean _dot = 0; //флаг обновления точек
volatile boolean _sec = 0; //флаг обновления секунды

uint8_t _timer_sleep = 0; //счетчик ухода в сон
boolean _disableSleep = 0; //флаг запрета сна
volatile boolean _sleep = 0; //флаг активного сна

struct time { //структура времени
  uint8_t s = 0;
  uint8_t m = 0;
  uint8_t h = 8;
  uint8_t DD = 1;
  uint8_t MM = 1;
  uint8_t YY = 21;
  uint8_t correct;
  uint8_t correctNow;
  boolean correctDrv;
  boolean timeFormat;
} RTC;

struct Settings1 { //структура основных настроек
  uint8_t timeBright[2] = {DEFAULT_TIME_NIGHT, DEFAULT_TIME_DAY}; //время подсветки по умолчанию(ночь, день)(0..23)(ч)
  uint8_t indiBright[2] = {DEFAULT_BRIGHT_NIGHT, DEFAULT_BRIGHT_DAY}; //яркость подсветки по умолчанию(ночь, день)(0..4)(1..5 в меню)
  uint8_t flask_mode = DEFAULT_FLASK_MODE; //текущий режим свечения колбы
  uint8_t bright_mode = DEFAULT_BRIGHT_MODE; //текущий режим подсветки
  uint8_t bright_levle = DEFAULT_BRIGHT; //текущая яркость подсветки
  uint8_t sleep_time = DEFAULT_SLEEP_TIME; //время ухода в сон
  uint8_t anim_mode = DEFAULT_ANIM_MODE; //текущий режим анимации
  boolean sleep_anim = DEFAULT_SLEEP_ANIM; //флаг анимации ухода в сон
  uint8_t adcMinAuto = DEFAULT_LIGHT_SENS_ADC; //минимальное значение ацп
  uint8_t adcMaxAuto = DEFAULT_LIGHT_SENS_ADC; //максимальное значение ацп
} mainSettings;

struct Settings2 { //структура настроек таймера
  boolean timer_mode = DEFAULT_MODE_TIMER; //режим работы таймера
  uint8_t timer_preset = DEFAULT_PRESET_TIMER; //текущий номер выбранного пресета таймера
  uint8_t timer_blink = DEFAULT_BLINK_TIMER; //время мигания таймера
} timerSettings;

const uint8_t allModes[] = {5, 8, 6}; //всего режимов настроек подсветки
const uint8_t daysInMonth[] PROGMEM = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}; //дней в месяце

//перечисления анимаций перебора цифр
enum {
  FLIP_NULL, //без анимации
  FLIP_REW_NUMBERS_CIRCLE, //перемотка чисел по кругу
  FLIP_REW_NUMBERS_TOP, //перемотка чисел сверху
  FLIP_TRAIN, //поезд
  FLIP_RUBBER_BAND, //резинка
  FLIP_DRUM, //барабан
  FLIP_FALL, //падение
  FLIP_EFFECT_NUM //максимум эффектов перелистывания
};

//перечисления меню настроек времени
enum {
  SET_TIME_HOURS, //настройка часов
  SET_TIME_MINS, //настройка минут
  SET_TIME_DATE, //настройка даты
  SET_TIME_MONTH, //настройка месяца
  SET_TIME_YEAR, //настройка года
  SET_TIME_CORRECT, //настройка коррекции
  SET_TIME_FORMAT, //настройка формата времени
  SET_TIME_MAX_ITEMS //максимум пунктов меню настроек времени
};

//перечисления кнопок
enum {
  KEY_NULL, //кнопка не нажата
  LEFT_KEY_PRESS, //клик левой кнопкой
  LEFT_KEY_HOLD, //удержание левой кнопки
  RIGHT_KEY_PRESS, //клик правой кнопкой
  RIGHT_KEY_HOLD //удержание правой кнопки
};

#define CONVERT_VCC(vcc) (uint8_t)((REFERENCE * 256) / vcc)

#define EEPROM_BLOCK_TIME EEPROM_BLOCK_NULL //блок памяти времени
#define EEPROM_BLOCK_SETTINGS_MAIN (EEPROM_BLOCK_TIME + sizeof(time)) //блок памяти основных настроек
#define EEPROM_BLOCK_SETTINGS_TIMER (EEPROM_BLOCK_SETTINGS_MAIN + sizeof(mainSettings)) //блок памяти настроек таймера

#define EEPROM_BLOCK_CRC_DEFAULT (EEPROM_BLOCK_SETTINGS_TIMER + sizeof(timerSettings)) //блок памяти контрольной суммы настроек
#define EEPROM_BLOCK_CRC_TIME (EEPROM_BLOCK_CRC_DEFAULT + 1) //блок памяти контрольной суммы времени
#define EEPROM_BLOCK_CRC_MAIN (EEPROM_BLOCK_CRC_TIME + 1) //блок памяти контрольной суммы основных настроек
#define EEPROM_BLOCK_CRC_TIMER (EEPROM_BLOCK_CRC_MAIN + 1) //блок памяти контрольной суммы настроек таймера

//----------------------------------Инициализация---------------------------------------------------
int main(void) //инициализация
{
  LEFT_INIT; //инициализация левой кнопки
  RIGHT_INIT; //инициализация правой кнопки
  DOT_INIT; //инициализация точек
  FLASK_INIT; //инициализация колбы
  SENS_INIT; //инициализация сенсора освещения

  PRR = (0x01 << PRTWI) | (0x01 << PRTIM1) | (0x01 << PRSPI) | (0x01 << PRUSART0) | (0x01 << PRADC); //отключаем все лишнее (I2C | TIMER1 | SPI | UART | ADC)
  ACSR |= 1 << ACD; //отключаем компаратор

  indiInit(); //инициализация индикаторов

  if (checkSettingsCRC()) { //проверяем настройки по умолчанию
    updateData((uint8_t*)&RTC, sizeof(RTC), EEPROM_BLOCK_TIME, EEPROM_BLOCK_CRC_TIME); //записываем дату и время в память
    updateData((uint8_t*)&mainSettings, sizeof(mainSettings), EEPROM_BLOCK_SETTINGS_MAIN, EEPROM_BLOCK_CRC_MAIN); //записываем основные настройки в память
    updateData((uint8_t*)&timerSettings, sizeof(timerSettings), EEPROM_BLOCK_SETTINGS_TIMER, EEPROM_BLOCK_CRC_TIMER); //записываем настройки таймера в память
  }
  else {
    if (checkData(sizeof(RTC), EEPROM_BLOCK_TIME, EEPROM_BLOCK_CRC_TIME)) updateData((uint8_t*)&RTC, sizeof(RTC), EEPROM_BLOCK_TIME, EEPROM_BLOCK_CRC_TIME); //записываем дату и время в память
    else EEPROM_ReadBlock((uint16_t)&RTC, EEPROM_BLOCK_TIME, sizeof(RTC)); //считываем время из памяти
    if (checkData(sizeof(mainSettings), EEPROM_BLOCK_SETTINGS_MAIN, EEPROM_BLOCK_CRC_MAIN)) updateData((uint8_t*)&mainSettings, sizeof(mainSettings), EEPROM_BLOCK_SETTINGS_MAIN, EEPROM_BLOCK_CRC_MAIN); //записываем основные настройки в память
    else EEPROM_ReadBlock((uint16_t)&mainSettings, EEPROM_BLOCK_SETTINGS_MAIN, sizeof(mainSettings)); //считываем основные настройки из памяти
    if (checkData(sizeof(timerSettings), EEPROM_BLOCK_SETTINGS_TIMER, EEPROM_BLOCK_CRC_TIMER)) updateData((uint8_t*)&timerSettings, sizeof(timerSettings), EEPROM_BLOCK_SETTINGS_TIMER, EEPROM_BLOCK_CRC_TIMER); //записываем настройки таймера в память
    else EEPROM_ReadBlock((uint16_t)&timerSettings, EEPROM_BLOCK_SETTINGS_TIMER, sizeof(timerSettings)); //считываем настройки таймера из памяти
  }

  powerDown(); //выключаем питание

  indiPrint("INIT", 0); //если пропадало питание
  _wait(TIME_START); //ждем

  flask_state = mainSettings.flask_mode; //обновление стотояния колбы
  _timer_sec = timerDefault[timerSettings.timer_preset] * 60; //установленное время таймера

  main_screen(); //главный экран
  return 0; //конец
}
//---------------------------------Ожидание----------------------------------------
void _wait(uint32_t timer) //ожидание
{
  for (timer_millis = timer; timer_millis && !check_keys();) data_update(); //ждем, обновление данных
}
//------------------------Сверка контрольной суммы---------------------------------
void checkCRC(uint8_t* crc, uint8_t data) //сверка контрольной суммы
{
  for (uint8_t i = 0; i < 8; i++) { //считаем для всех бит
    *crc = ((*crc ^ data) & 0x01) ? (*crc >> 0x01) ^ 0x8C : (*crc >> 0x01); //рассчитываем значение
    data >>= 0x01; //сдвигаем буфер
  }
}
//------------------------Проверка данных в памяти-------------------------------
boolean checkData(uint8_t size, uint8_t cell, uint8_t cellCRC) //проверка данных в памяти
{
  uint8_t crc = 0;
  for (uint8_t n = 0; n < size; n++) checkCRC(&crc, EEPROM_ReadByte(cell + n));
  return (boolean)(crc != EEPROM_ReadByte(cellCRC));
}
//-----------------------Обновление данных в памяти-------------------------------
void updateData(uint8_t* str, uint8_t size, uint8_t cell, uint8_t cellCRC) //обновление данных в памяти
{
  uint8_t crc = 0;
  for (uint8_t n = 0; n < size; n++) checkCRC(&crc, str[n]);
  EEPROM_UpdateBlock((uint16_t)str, cell, size);
  EEPROM_UpdateByte(cellCRC, crc);
}
//--------------------Проверка контрольной суммы настроек--------------------------
boolean checkSettingsCRC(void) //проверка контрольной суммы настроек
{
  uint8_t CRC = 0; //буфер контрольной суммы

  for (uint8_t i = 0; i < sizeof(RTC); i++) checkCRC(&CRC, *((uint8_t*)&RTC + i));
  for (uint8_t i = 0; i < sizeof(mainSettings); i++) checkCRC(&CRC, *((uint8_t*)&mainSettings + i));
  for (uint8_t i = 0; i < sizeof(timerSettings); i++) checkCRC(&CRC, *((uint8_t*)&timerSettings + i));


  if (EEPROM_ReadByte(EEPROM_BLOCK_CRC_DEFAULT) == CRC) return 0;
  else EEPROM_UpdateByte(EEPROM_BLOCK_CRC_DEFAULT, CRC);
  return 1;
}
//-------------------------Получить 12-ти часовой формат------------------------
uint8_t get_12h(uint8_t timeH) //получить 12-ти часовой формат
{
  return (timeH > 12) ? (timeH - 12) : (timeH) ? timeH : 12; //возвращаем результат
}
//-------------------------------Чтение датчика освещённости--------------------------------------------------
uint8_t readLightSens(void) //чтение датчика освещённости
{
  SENS_ON; //включили питание сенсора
  ADC_enable(); //включение ADC

  ADMUX = (0x01 << REFS0) | (0x01 << ADLAR) | SENS_ANALOG_PIN; //настройка мультиплексатора АЦП
  ADCSRA = (0x01 << ADEN) | (0x01 << ADPS0) | (0x01 << ADPS2); //настройка АЦП пределитель 32
  ADCSRA |= (1 << ADSC); //запускаем преобразование

  uint16_t result = 0; //результат опроса АЦП внутреннего опорного напряжения
  for (uint8_t i = 0; i < 10; i++) { //делаем 10 замеров
    while (ADCSRA & (1 << ADSC)); //ждем окончания преобразования
    result += ADCH; //прибавляем замер в буфер
    ADCSRA |= (1 << ADSC); //перезапускаем преобразование
  }
  result /= 10; //находим среднее значение

  ADC_disable(); //выключение ADC
  SENS_OFF; //выключили питание сенсора

  return result; //возвращаем результат
}
//-------------------------------Максимальное количество дней--------------------------------------------------
uint8_t maxDays(void) //максимальное количество дней
{
  return ((RTC.MM == 2 && !(RTC.YY % 4)) ? 1 : 0) + pgm_read_byte(&daysInMonth[RTC.MM - 1]); //возвращаем количество дней в месяце
}
//-------------------------------Чтение датчика освещённости--------------------------------------------------
uint8_t lightSens(void) //чтение датчика освещённости
{
  return map(constrain(readLightSens(), mainSettings.adcMinAuto, mainSettings.adcMaxAuto), mainSettings.adcMinAuto + 1, mainSettings.adcMaxAuto - 1, 0, 4); //возвращаем результат
}
//----------------------------------------------------------------------------------
boolean checkTimeBright(void) //проверка времени на смену яркости
{
  return !((mainSettings.timeBright[0] > mainSettings.timeBright[1] && (RTC.h >= mainSettings.timeBright[0] || RTC.h < mainSettings.timeBright[1])) ||
           (mainSettings.timeBright[0] < mainSettings.timeBright[1] && RTC.h >= mainSettings.timeBright[0] && RTC.h < mainSettings.timeBright[1]));
}
//----------------------------------------------------------------------------------
void changeBright(void) //установка яркости от времени суток
{
  switch (mainSettings.bright_mode) {
    case 0: indiSetBright(brightDefault[mainSettings.bright_levle]); break; //установка яркости индикаторов
    case 1: indiSetBright(brightDefault[mainSettings.indiBright[checkTimeBright()]]); break; //установка яркости индикаторов
    case 2: indiSetBright(brightDefault[lightSens()]); break; //установка яркости индикаторов
  }
  if (RTC.h > 11) DOT_PM_ON; //если больше 12 часов
  else DOT_PM_OFF; //иначе выключаем индикатор
}
//----------------------------------------------------------------------------------
void batCheck(void)
{
  bat = map(constrain(read_VCC(), CONVERT_VCC(BAT_MAX_V), CONVERT_VCC(BAT_MIN_V)), CONVERT_VCC(BAT_MIN_V), CONVERT_VCC(BAT_MAX_V), 0, 100); //состояние батареи
  if (bat < LOW_BAT_P) _msg_type = 3; //если батарея разряжена
  else if (bat < MSG_BAT_P) _msg_type = 2; //если осталось мало заряда
  bat_tmr = 0; //сбрасываем таймер
}
//----------------------------------Чтение напряжения батареи-------------------------------------------------
uint8_t read_VCC(void)  //чтение напряжения батареи
{
  ADC_enable(); //включение ADC
  ADMUX = (0x01 << REFS0) | (0x01 << ADLAR) | (0x01 << MUX3) | (0x01 << MUX2) | (0x01 << MUX1); //выбор внешнего опорного + 1.1в
  ADCSRA = (0x01 << ADEN) | (0x01 << ADPS0) | (0x01 << ADPS1) | (0x01 << ADPS2); //настройка АЦП пределитель 128
  _delay_ms(5); //ждем пока опорное успокоится
  ADCSRA |= (1 << ADSC); //запускаем преобразование
  while (ADCSRA & (1 << ADSC)); //ждем окончания преобразования
  uint8_t resu = ADCH; //результат опроса АЦП
  ADC_disable(); //выключение ADC
  return resu; //возвращаем результат опроса АЦП
}
//-------------------------------------Включение ADC------------------------------------------------
void ADC_enable(void) //включение ADC
{
  PRR &= ~ (1 << PRADC); //включаем питание АЦП
  ADCSRA |= (1 << ADEN); //включаем ацп
}
//-------------------------------------Выключение ADC-----------------------------------------------
void ADC_disable(void) //выключение ADC
{
  ADCSRA &= ~ (1 << ADEN); //выключаем ацп
  PRR |= (1 << PRADC); //выключаем питание ацп
}
//-------------------------------------Ожидание--------------------------------------------------------
void waint_pwr(void) //ожидание
{
  SMCR = (0x01 << SE);  //устанавливаем режим сна idle

  MCUCR = (0x03 << 5); //выкл bod
  MCUCR = (0x02 << 5);

  asm ("sleep");  //с этого момента спим.
}
//-------------------------------------Энергосбережение--------------------------------------------------------
void save_pwr(void) //энергосбережение
{
  while (ASSR & (0x01 << OCR2AUB)); //ждем когда завершиться запись в регистр таймера

  SMCR = (0x01 << SM0) | (0x01 << SM1) | (0x01 << SE);  //устанавливаем режим сна powersave

  MCUCR = (0x03 << 5); //выкл bod
  MCUCR = (0x02 << 5);

  asm ("sleep");  //с этого момента спим.
}
//------------------------------------Включение питания----------------------------------------------
ISR(INT0_vect, ISR_NOBLOCK) //внешнее прерывание на пине INT0 - включение питания
{
  uint16_t startDellay = TIME_PWR_ON; //устанавливаем таймер
  while (!RIGHT_CHK) { //если кнопка не отжата
    if (startDellay) { //если время не истекло
      _delay_ms(1); //ждем 1мс
      startDellay--; //отнимаем от таймера 1 мс
    }
    else { //иначе включаем питание
      batCheck(); //проверяем заряд акб
      if (bat > PWR_BAT_P) { //если батарея не разряжена
        changeBright(); //смена яркости индикаторов

        indiDisableSleep(); //включаем дисплей
        indiPrint("####", 0); //отрисовка сообщения

        while (!RIGHT_CHK); //ждем пока отпустят кнопу
        EIMSK = 0; //запрещаем внешнее прерывание INT0

        tick_ms = 0; //сбросили счетчик мс
        animReset(); //сброс анимаций
      }
      else { //иначе выводим предупреждение об разряженной батарее
        indiSetBright(brightDefault[0]); //устанавливаем минимальную яркость
        indiDisableSleep(); //включаем дисплей
        indiPrint("LO", 1); //отрисовка сообщения разряженной батареи
        _delay_ms(TIME_PWR_DOWN); //ждём
        indiEnableSleep(); //выключаем дисплей
      }
    }
  }
}
//----------------------------------------------------------------------------------
void powerDown(void)
{
  indiEnableSleep(); //выключаем дисплей

  EICRA = (0x01 << ISC01); //настраиваем внешнее прерывание по спаду импульса на INT0
  EIMSK = (0x01 << INT0); //разрешаем внешнее прерывание INT0

  _sleep = 1; //установили флаг отключения питания

  while (_sleep) save_pwr(); //спим
}
//----------------------------------Выход из глубокого сна-------------------------------------------
ISR(PCINT2_vect, ISR_NOBLOCK) //внешнее прерывание PCINT2
{
  PCICR = 0; //запрещаем прерывания PCINT2
  sleepOut(); //выход из сна
}
//------------------------------------Сброс анимаций-------------------------------------------------------
void animReset(void) //сброс анимаций
{
  _scr = 0; //разрешаем обновить индикаторы
  _dot = 1; //сбрасываем флаг точек
  _sleep = 0; //сбрасываем флаг активного сна
  _timer_sleep = 0; //сбрасываем таймер сна

  timer_dot = 0; //сбрасываем таймер точек
  btn_check = 0; //запрещаем проверку кнопок

  if (_mode != 3) { //если таймер не работает
    _mode = 0; //переходим в режим часов
    _animStart = 1; //разрешаем анимацию
  }
}
//-------------------------------------Выход из сна--------------------------------------------------------
void sleepOut(void) //выход из сна
{
  animReset(); //сброс анимаций
  batCheck(); //проверяем заряд акб

  changeBright(); //смена яркости индикаторов
  indiDisableSleep(); //включаем дисплей
}
//-------------------------------Режим сна----------------------------------------------------
boolean sleepMode(void) //режим сна
{
  if (!_disableSleep && mainSettings.sleep_time && _timer_sleep == mainSettings.sleep_time) {
    if (mainSettings.sleep_anim) {
      uint8_t indi[4]; //буфер анимации
      for (uint8_t s = 0; s < 4; s++) indi[s] = readLightSens() % 7; //выбираем рандомный сегмент
      for (uint8_t c = 0; c < 7;) { //отрисовываем анимацию
        data_update(); //обновление данных
        if (check_keys()) return 0; //если нажата кнопка прерываем сон
        if (!timer_millis) { //если таймер истек
          timer_millis = SLEEP_ANIM_TIME; //устанавливаем таймер
          c++; //смещаем анимацию
          for (uint8_t i = 0; i < 4; i++) { //стираем сегменты
            if (indi[i] < 6) indi[i]++; //если не достигнут максимум
            else indi[i] = 0; //иначе сбрасываем в начало
            indiSet(indi[i], i, 0); //очищаем сегменты по очереди
          }
        }
      }
    }
    _sleep = 1; //устанавливаем флаг активного сна
    indiEnableSleep(); //выключаем дисплей
    PCIFR = (0x01 << PCIF2); //сбрасываем флаг прерывания PCINT2
    PCMSK2 = (0x01 << RIGHT_BIT) | (0x01 << LEFT_BIT); //разрешаем прерывания от D0 и D2
    PCICR = (0x01 << PCIE2); //разрешаем внешнее прерывание PCINT2
    return 1;
  }
  return 0;
}
//-----------------------------Проверка кнопок----------------------------------------------------
uint8_t check_keys(void) //проверка кнопок
{
  static uint8_t btn_switch; //флаг мультиопроса кнопок

  switch (btn_switch) { //переключаемся в зависимости от состояния мультиопроса
    case 0:
      if (!LEFT_CHK) { //если нажата левая кл.
        btn_switch = 1; //выбираем клавишу опроса
        btn_state = 0; //обновляем текущее состояние кнопки
      }
      else if (!RIGHT_CHK) { //если нажата правая кл.
        btn_switch = 2; //выбираем клавишу опроса
        btn_state = 0; //обновляем текущее состояние кнопки
      }
      else btn_state = 1; //обновляем текущее состояние кнопки
      break;
    case 1: btn_state = LEFT_CHK; break; //опрашиваем левую клавишу
    case 2: btn_state = RIGHT_CHK; break; //опрашиваем правую клавишу
  }

  switch (btn_state) { //переключаемся в зависимости от состояния клавиши
    case 0:
      if (btn_check) { //если разрешена провекрка кнопки
        if (btn_tmr > BTN_HOLD_TICK) { //если таймер больше длительности удержания кнопки
          btn_tmr = BTN_GIST_TICK; //сбрасываем таймер на антидребезг
          btn_check = 0; //запрещем проврку кнопки
          _timer_sleep = 0; //сбрасываем таймер сна
          _scr = 0; //разрешаем обновить экран
          switch (btn_switch) { //переключаемся в зависимости от состояния мультиопроса
            case 1: return LEFT_KEY_HOLD; //удержание левой кнопки
            case 2: return RIGHT_KEY_HOLD; //удержание правой кнопки
          }
        }
      }
      break;

    case 1:
      if (btn_tmr > BTN_GIST_TICK) { //если таймер больше времени антидребезга
        btn_tmr = BTN_GIST_TICK; //сбрасываем таймер на антидребезг
        btn_check = 0; //запрещем проврку кнопки
        _timer_sleep = 0; //сбрасываем таймер сна
        _scr = 0; //разрешаем обновить экран
        switch (btn_switch) { //переключаемся в зависимости от состояния мультиопроса
          case 1: return LEFT_KEY_PRESS; //клик левой кнопкой
          case 2: return RIGHT_KEY_PRESS; //клик правой кнопкой
        }
      }
      else if (!btn_tmr) {
        btn_check = 1; //разрешаем проврку кнопки
        btn_switch = 0; //сбрасываем мультиопрос кнопок
      }
      break;
  }

  return KEY_NULL;
}
//-----------------------------------Счет времени------------------------------------------
ISR(TIMER2_OVF_vect) //счет времени
{
  OCR2A = 0; //делаем пустую запись в регистр таймера

  //счет времени
  if (++RTC.s > 59) { //секунды
    RTC.s = 0; //сбросили секунды
    if (++RTC.m > 59) { //минуты
      RTC.m = 0; //сбросили минуты

      if (RTC.correct && ++RTC.correctNow >= RTC.correct) { //если пришло время коррекции
        if (RTC.correctDrv) { //если откатываем секунду
          RTC.m = 59; //сбросили минуты
          RTC.s = 59; //сбросили секунды
          RTC.correctNow = 255; //сбросили коррекцию
        }
        else {
          RTC.s = 1; //иначе прибавляем секунду
          RTC.correctNow = 0; //сбросили коррекцию
        }
      }
      else {
        if (++RTC.h > 23) { //часы
          RTC.h = 0; //сбросили часы
          if (++RTC.DD > maxDays()) { //день
            RTC.DD = 1; //сбросили день
            if (++RTC.MM > 12) { //месяц
              RTC.MM = 1; //сбросили месяц
              if (++RTC.YY > 99) { //год
                RTC.YY = 21; //сбросили год
              }
            }
          }
        }
      }
      if (mainSettings.bright_mode == 1) indiSetBright(brightDefault[mainSettings.indiBright[checkTimeBright()]]); //установка яркости индикаторов
      if (RTC.h > 11) DOT_PM_ON; //если больше 12 часов
      else DOT_PM_OFF; //иначе выключаем индикатор
    }
  }

  _sec = 0; //разрешаем обновить данные
}
//----------------------------------Обновление данных-----------------------------------------------
void data_update(void) //обновление данных
{
  if (!_sleep) waint_pwr(); //если не спим то режим ожидания
  else save_pwr(); //иначе режим сна

  if (!_sec) { //если прошла секунда
    _sec = 1; //сбросили флаг секунды

    //таймер часов
    if (_timer_start) {
      switch (timerSettings.timer_mode) {
        case 0:
          if (_timer_sec) {
            _timer_sec--; //уменьшаем таймер на 1сек
            //если осталось мало времени
            if (_timer_sec == timerSettings.timer_blink) {
              _mode = 3; //переходим в режим таймера
              _disableSleep = 1; //запрещаем сон
              if (_sleep) sleepOut(); //выход из сна
            }
            //оповещение окончания таймера
            if (!_timer_sec) _msg_type = 1; //если таймер досчитал до 0
          }
          break;

        case 1:
          if (_timer_sec < 6000) _timer_sec++; //увеличиваем таймер на 1сек
          else _timer_start = 0; //если секундомер переполнен выключаем таймер
          break;
      }
    }

    if (!_sleep) { //если не спим
      if (bat_tmr >= BAT_TIME) batCheck(); //если пришло время опросить акб
      else bat_tmr++; //иначе прибавляем время

      if (mainSettings.flask_mode == 2) flask_state = lightSens(); //автоматическое включение колбы
      if (mainSettings.bright_mode == 2) indiSetBright(brightDefault[lightSens()]); //установка яркости индикаторов

      if (_timer_sleep <= mainSettings.sleep_time) _timer_sleep++; //таймер ухода в сон

      _scr = _dot = 0; //разрешаем обновить индикаторы
    }
  }

  while (tick_ms) { //если был тик, обрабатываем данные
    tick_ms--;

    switch (btn_state) { //таймер опроса кнопок
      case 0: if (btn_check) btn_tmr++; break; //считаем циклы
      case 1: if (btn_tmr > 0) btn_tmr--; break; //убираем дребезг
    }

    if (timer_millis) timer_millis--; //если таймер активен
    if (timer_dot) timer_dot--; //если таймер активен
  }
}
//-------------------------------Проверка оповещений--------------------------------------------
void checkMessage(void) //проверка оповещений
{
  if (_msg_type) {
    dot_state = 0; //выключаем точки
    mainSettings.flask_mode |= 0x80; //запрещаем управление колбой

    switch (_msg_type) {
      case 1: //оповещения таймера
        _timer_start = 0; //сбрасываем режим таймера
        for (timer_millis = TIME_MSG_TMR_OVF; timer_millis && !check_keys();) {
          data_update(); //обновление данных
          if (!timer_dot) { //если таймер отработал
            indiClr(); //очистка индикаторов
            if (!flask_state) indiPrint("TOUT", 0); //если колба не горит
            flask_state = !flask_state; //инвертируем колбу
            timer_dot = 500; //устанавливаем таймер
          }
        }
        _timer_sec = timerDefault[timerSettings.timer_preset] * 60; //устанавливаем таймер в начало
        _mode = 0; //переходим в режим часов
        _disableSleep = 0; //разрешаем сон
        break;
      case 2: { //оповещения батареи
          uint8_t pos = 0; //позиция надписи
          for (timer_millis = TIME_MSG_BAT; timer_millis && !check_keys();) {
            data_update(); //обновление данных
            if (!timer_dot) { //если таймер отработал
              indiClr(); //очистка индикаторов
              indiPrint("LO", (pos > 2) ? 1 : pos); //отрисовка сообщения разряженной батареи
              if (pos < 3) pos++; else pos = 0; //меняем позицию
              flask_state = !flask_state; //инвертируем колбу
              timer_dot = 500; //устанавливаем таймер
            }
          }
        }
        break;
      case 3: //оповещения выключения питания
        indiPrint(" OFF", 0); //отрисовка сообщения разряженной батареи
        for (timer_millis = TIME_MSG_BAT; timer_millis;) { //ждем
          data_update(); //обновление данных
          if (!timer_dot) { //если таймер отработал
            flask_state = !flask_state; //инвертируем колбу
            timer_dot = 500; //устанавливаем таймер
          }
        }
        updateData((uint8_t*)&RTC, sizeof(RTC), EEPROM_BLOCK_TIME, EEPROM_BLOCK_CRC_TIME); //записываем дату и время в память
        powerDown(); //выключаем питание
        break;
    }

    _msg_type = 0; //сбрасываем тип оповещения
    _timer_sleep = 0; //сбрасываем таймер сна
    mainSettings.flask_mode &= 0x7F; //разрешаем управление колбой
    flask_state = mainSettings.flask_mode; //обновление стотояния колбы
  }
}
//----------------------------------------------------------------------------------
void dotFlash(void)
{
  if (!_dot) { //если поднят флаг точек
    if (!dot_state) { //если точки выключены
      dot_state = 1; //включаем точки
      timer_dot = DOT_TIME; //устанавливаем таймер
    }
    else if (!timer_dot) { //иначе если таймер истек
      dot_state = 0; //выключаем точки
      _dot = 1; //сбрасываем флаг точек
    }
  }
}
//----------------------------------------------------------------------------------
void dotFlashTimer(void)
{
  if (!_dot) { //если поднят флаг точек
    if (!dot_state) { //если точки выключены
      dot_state = 1; //включаем точки
      timer_dot = TIMER_DOT_TIME; //устанавливаем таймер
    }
    else if (!timer_dot) { //иначе если таймер истек
#if TIMER_BLINK
      if (!timerSettings.timer_mode && _timer_start && _timer_sec < timerSettings.timer_blink) indiClr(); //если таймер не запущен или время больше утановленного или точки не горят
#endif
      if (_timer_start) dot_state = 0; //если таймер запущен то выключаем точки
      _dot = 1; //сбрасываем флаг точек
    }
  }
}
//-------------------------------Анимция перелистывания----------------------------------------------------
void animFlip(void) //анимция перелистывания
{
  _animStart = 0; //запрещаем анимацию

  if (mainSettings.anim_mode) {
    boolean stopIndi[4]; //буфер анимации
    uint8_t anim_buf[4]; //буфер анимации
    uint8_t indiPos = 0; //позиция индикатора
    uint8_t numState = 0; //фаза числа
    uint8_t stopTick = 0; //фаза числа

    uint8_t timeHour = (RTC.timeFormat) ? get_12h(RTC.h) : RTC.h; //текущий час
    uint8_t timeMins = RTC.m; //текущая минута

    anim_buf[0] = timeHour / 10; //часы
    anim_buf[1] = timeHour % 10; //часы
    anim_buf[2] = timeMins / 10; //минуты
    anim_buf[3] = timeMins % 10; //минуты

    switch (mainSettings.anim_mode) {
      case FLIP_REW_NUMBERS_CIRCLE:
        for (uint8_t i = 0; i < 10;) {
          data_update(); //обновление данных
          dotFlash(); //мигаем точками
          if (check_keys()) return;
          if (!timer_millis) { //если таймер отработал
            for (uint8_t n = 0; n < 4; n++) {
              if (anim_buf[n] < 9) anim_buf[n]++; else anim_buf[n] = 0;
              indiPrintNum(anim_buf[n], n); //вывод часов
            }
            i++; //прибавляем цикл
            timer_millis = FLIP_REW_NUMBERS_CIRCLE_TIME; //устанавливаем таймер
          }
        }
        break;

      case FLIP_REW_NUMBERS_TOP:
        for (uint8_t i = 0; i < 4; i++) anim_buf[i] = 9; //буфер анимации
        for (uint8_t i = 1; i;) {
          data_update(); //обновление данных
          dotFlash(); //мигаем точками
          if (check_keys()) return;
          if (!timer_millis) { //если таймер отработал
            i = 0; //сбрасываем счетчик циклов
            indiPrintNum(anim_buf[0], 0); //вывод часов
            if (anim_buf[0] > timeHour / 10) {
              anim_buf[0]--;
              i++;
            }
            indiPrintNum(anim_buf[1], 1); //вывод часов
            if (anim_buf[1] > timeHour % 10) {
              anim_buf[1]--;
              i++;
            }
            indiPrintNum(anim_buf[2], 2); //вывод минут
            if (anim_buf[2] > timeMins / 10) {
              anim_buf[2]--;
              i++;
            }
            indiPrintNum(anim_buf[3], 3); //вывод минут
            if (anim_buf[3] > timeMins % 10) {
              anim_buf[3]--;
              i++;
            }
            timer_millis = FLIP_REW_NUMBERS_TOP_TIME; //устанавливаем таймер
          }
        }
        break;

      case FLIP_TRAIN:
        for (uint8_t i = 4; i;) {
          data_update(); //обновление данных
          dotFlash(); //мигаем точками
          if (check_keys()) return;
          if (!timer_millis) { //если таймер отработал
            i--; //убавляем цикл
            timer_millis = FLIP_TRAIN_TIME; //устанавливаем таймер
            for (uint8_t b = 0; b < 4; b++) indiPrintNum(anim_buf[b - i], b); //вывод часов
          }
        }
        break;

      case FLIP_RUBBER_BAND:
        for (uint8_t i = 0; i < 4;) {
          data_update(); //обновление данных
          dotFlash(); //мигаем точками
          if (check_keys()) return;
          if (!timer_millis) { //если таймер отработал
            for (uint8_t b = 0; b < 4 - i; b++) {
              if (b == indiPos) indiPrintNum(anim_buf[3 - i], b); //вывод часов
              else indiPrint(" ", b); //вывод пустого символа
            }
            if (indiPos++ >= 3 - i) {
              indiPos = 0; //сбрасываем позицию индикатора
              i++; //прибавляем цикл
            }
            timer_millis = FLIP_RUBBER_BAND_TIME; //устанавливаем таймер
          }
        }
        break;

      case FLIP_DRUM:
        for (uint8_t i = 0; i < 4; i++) anim_buf[i] = stopIndi[i] = 0; //буфер анимации
        for (uint8_t i = 1; i;) {
          data_update(); //обновление данных
          dotFlash(); //мигаем точками
          if (check_keys()) return;
          if (!timer_millis) { //если таймер отработал
            if (numState < 4) numState++;
            else {
              numState = i = stopTick = 0;
              if (anim_buf[0] < timeHour / 10) {
                anim_buf[0]++;
                i++;
              }
              else stopTick += FLIP_DRUM_STOP;
              if (anim_buf[1] < timeHour % 10) {
                anim_buf[1]++;
                i++;
              }
              else stopTick += FLIP_DRUM_STOP;
              if (anim_buf[2] < timeMins / 10) {
                anim_buf[2]++;
                i++;
              }
              else stopTick += FLIP_DRUM_STOP;
              if (anim_buf[3] < timeMins % 10) {
                anim_buf[3]++;
                i++;
              }
              else stopTick += FLIP_DRUM_STOP;
            }
            if (!stopIndi[0] && anim_buf[0] == timeHour / 10 && numState >= 2) stopIndi[0] = 1;
            if (!stopIndi[1] && anim_buf[1] == timeHour % 10 && numState >= 2) stopIndi[1] = 1;
            if (!stopIndi[2] && anim_buf[2] == timeMins / 10 && numState >= 2) stopIndi[2] = 1;
            if (!stopIndi[3] && anim_buf[3] == timeMins % 10 && numState >= 2) stopIndi[3] = 1;

            for (uint8_t c = 0; c < 4; c++) {
              if (!stopIndi[c]) indi_buf[c] = numbersForAnim[anim_buf[c]][numState]; //отрисовываем
              else indiPrintNum(anim_buf[c], c);
            }
            timer_millis = FLIP_DRUM_TIME + stopTick; //устанавливаем таймер
          }
        }
        break;

      case FLIP_FALL:
        for (uint8_t i = 0; i < 4;) {
          data_update(); //обновление данных
          dotFlash(); //мигаем точками
          if (check_keys()) return;
          if (!timer_millis) { //если таймер отработал
            if (numState < 2) numState++;
            else {
              numState = 0; //переходим к следующему индикатору
              i++; //прибавляем цикл
            }
            indi_buf[i] = numbersForAnim[anim_buf[i]][numState]; //отрисовываем
            timer_millis = FLIP_FALL_TIME; //устанавливаем таймер
          }
        }
        break;
    }
  }
}
//----------------------------------------------------------------------------------
void settings_time(void)
{
  boolean blink_data = 0; //мигание сигментами
  uint8_t cur_mode = 0; //текущий режим
  int16_t cur_correct = (RTC.correctDrv) ? -(int16_t)RTC.correct : RTC.correct; //текущая коррекция

  dot_state = 1; //включаем точку
  indiClr(); //очищаем индикаторы
  indiPrint("SET", 0);
  _wait(TIME_MSG); //ждем

  //настройки
  while (1) {
    data_update(); //обновление данных

    if (!_scr) {
      _scr = 1; //сбрасываем флаг
      indiClr(); //очистка индикаторов
      switch (cur_mode) {
        case SET_TIME_HOURS:
        case SET_TIME_MINS:
          if (!blink_data || cur_mode == 1) indiPrintNum(RTC.h, 0, 2, '0'); //вывод часов
          if (!blink_data || cur_mode == 0) indiPrintNum(RTC.m, 2, 2, '0'); //вывод минут
          break;
        case SET_TIME_DATE:
        case SET_TIME_MONTH:
          if (!blink_data || cur_mode == 3) indiPrintNum(RTC.DD, 0, 2, '0'); //вывод даты
          if (!blink_data || cur_mode == 2) indiPrintNum(RTC.MM, 2, 2, '0'); //вывод месяца
          break;
        case SET_TIME_YEAR:
          indiPrint("20", 0); //вывод 2000
          if (!blink_data) indiPrintNum(RTC.YY, 2, 2, '0'); //вывод года
          break;
        case SET_TIME_CORRECT:
          if (!blink_data) indiPrintNum(cur_correct, 0, 4, ' '); //вывод коррекции
          break;
        case SET_TIME_FORMAT:
          if (!blink_data) indiPrintNum((RTC.timeFormat) ? 12 : 24, 0, 4, ' '); //вывод формата времени
          break;
      }
      blink_data = !blink_data; //мигание сигментами
    }

    //+++++++++++++++++++++  опрос кнопок  +++++++++++++++++++++++++++
    switch (check_keys()) {
      case LEFT_KEY_PRESS: //клик левой кнопкой
        switch (cur_mode) {
          //настройка времени
          case SET_TIME_HOURS: if (RTC.h > 0) RTC.h--; else RTC.h = 23; RTC.s = 0; break; //часы
          case SET_TIME_MINS: if (RTC.m > 0) RTC.m--; else RTC.m = 59; RTC.s = 0; break; //минуты

          //настройка даты
          case SET_TIME_DATE: if (RTC.DD > 1) RTC.DD--; else RTC.DD = maxDays(); break; //день
          case SET_TIME_MONTH: //месяц
            if (RTC.MM > 1) RTC.MM--;
            else RTC.MM = 12;
            if (RTC.DD > maxDays()) RTC.DD = maxDays();
            break;

          //настройка года
          case SET_TIME_YEAR: if (RTC.YY > 21) RTC.YY--; else RTC.YY = 99; break; //год

          //настройка коррекции
          case SET_TIME_CORRECT: if (cur_correct > -168) cur_correct--; RTC.correctNow = 255; break; //коррекция

          //настройка формата времени
          case SET_TIME_FORMAT: RTC.timeFormat = !RTC.timeFormat; break; //формат времени
        }
        blink_data = 0; //сбрасываем флаг мигания
        break;

      case RIGHT_KEY_PRESS: //клик правой кнопкой
        switch (cur_mode) {
          //настройка времени
          case SET_TIME_HOURS: if (RTC.h < 23) RTC.h++; else RTC.h = 0; RTC.s = 0; break; //часы
          case SET_TIME_MINS: if (RTC.m < 59) RTC.m++; else RTC.m = 0; RTC.s = 0; break; //минуты

          //настройка даты
          case SET_TIME_DATE: if (RTC.DD < maxDays()) RTC.DD++; else RTC.DD = 1; break; //день
          case SET_TIME_MONTH: //месяц
            if (RTC.MM < 12) RTC.MM++;
            else RTC.MM = 1;
            if (RTC.DD > maxDays()) RTC.DD = maxDays();
            break;

          //настройка года
          case SET_TIME_YEAR: if (RTC.YY < 99) RTC.YY++; else RTC.YY = 21; break; //год

          //настройка коррекции
          case SET_TIME_CORRECT: if (cur_correct < 168) cur_correct++; RTC.correctNow = 255; break; //коррекция

          //настройка формата времени
          case SET_TIME_FORMAT: RTC.timeFormat = !RTC.timeFormat; break; //формат времени
        }
        blink_data = 0; //сбрасываем флаг мигания
        break;

      case LEFT_KEY_HOLD: //удержание левой кнопки
        if (cur_mode < (SET_TIME_MAX_ITEMS - 1)) cur_mode++; else cur_mode = 0;
        switch (cur_mode) {
          case SET_TIME_HOURS:
            indiClr(); //очистка индикаторов
            indiPrint("T", 0);
            _wait(TIME_MSG_PNT); //ждем
            break;

          case SET_TIME_DATE:
            indiClr(); //очистка индикаторов
            indiPrint("D", 0);
            _wait(TIME_MSG_PNT); //ждем
            break;

          case SET_TIME_YEAR:
            indiClr(); //очистка индикаторов
            indiPrint("Y", 0);
            _wait(TIME_MSG_PNT); //ждем
            break;

          case SET_TIME_CORRECT:
            indiClr(); //очистка индикаторов
            indiPrint("C", 0);
            _wait(TIME_MSG_PNT); //ждем
            break;

          case SET_TIME_FORMAT:
            indiClr(); //очистка индикаторов
            indiPrint("C", 0);
            _wait(TIME_MSG_PNT); //ждем
            break;
        }
        blink_data = 0; //сбрасываем флаг мигания
        break;

      case RIGHT_KEY_HOLD: //удержание правой кнопки
        if (cur_correct < 0) { //если коррекция отрицательная
          RTC.correctDrv = 1; //устанавливаем флаг убавления секунды
          RTC.correct = -cur_correct; //устанавливаем время коррекции
        }
        else { //иначе коррекция положительная
          RTC.correctDrv = 0; //устанавливаем флаг прибавления секунды
          RTC.correct = cur_correct; //устанавливаем время коррекции
        }
        updateData((uint8_t*)&RTC, sizeof(RTC), EEPROM_BLOCK_TIME, EEPROM_BLOCK_CRC_TIME); //записываем дату и время в память
        changeBright(); //смена яркости индикаторов
        dot_state = 0; //выключаем точку
        indiClr(); //очистка индикаторов
        indiPrint("OUT", 0);
        _wait(TIME_MSG); //ждем
        if (timerSettings.timer_mode || !_timer_start || _timer_sec > timerSettings.timer_blink) _disableSleep = 0; //разрешаем сон
        if (_mode != 3) _mode = 0; //переходим в режим часов
        return;
    }
  }
}
//----------------------------------------------------------------------------------
void settings_bright(void)
{
  boolean blink_data = 0; //мигание сигментами
  uint8_t cur_mode = 0; //текущий режим
  uint8_t result = 0; //результат опроса сенсора освещенности

  dot_state = 0; //выключаем точку
  indiClr(); //очищаем индикаторы
  indiPrint("BRI", 0);
  _wait(TIME_MSG); //ждем

  //настройки
  while (1) {
    data_update(); //обновление данных

    if (!_scr) {
      _scr = 1; //сбрасываем флаг
      indiClr(); //очистка индикаторов
      switch (cur_mode) {
        case 0:
          indiPrint("FL", 0); //колба
          if (!blink_data) indiPrintNum(mainSettings.flask_mode, 3); //режим колбы
          break;
        case 1:
          indiPrint("SL", 0); //сон
          if (!blink_data) indiPrintNum(mainSettings.sleep_time, 2, 2); //время сна
          break;
        case 2:
          indiPrint("AN", 0); //анимация
          if (!blink_data) indiPrintNum(mainSettings.anim_mode, 3); //режим анимации
          break;
        case 3:
          indiPrint("AS", 0); //анимация сна
          if (!blink_data) indiPrintNum(mainSettings.sleep_anim, 3); //анимация ухода в сон
          break;
        case 4:
          indiPrint("BR", 0); //подсветка
          if (!blink_data) indiPrintNum(mainSettings.bright_mode, 3); //режим подсветки
          break;
        case 5:
          switch (mainSettings.bright_mode) {
            case 0: //ручная подсветка
              indiPrint("L", 0);
              if (!blink_data) indiPrintNum(mainSettings.bright_levle + 1, 2, 2, '0'); //вывод яркости
              break;

            case 1: //подсветка день/ночь
              indiPrint("N", 0);
              if (!blink_data) indiPrintNum(mainSettings.timeBright[0], 2, 2, '0'); //вывод время включения ночной подсветки
              break;

            case 2: //авто-подсветка
              indiPrint("N", 0); //ночь
              result = readLightSens(); //считываем сенсор
              if (result < mainSettings.adcMinAuto) mainSettings.adcMinAuto = result; //находим минимум
              indiPrintNum(mainSettings.adcMinAuto, 1, 3, ' '); //вывод порога ночь
              break;
          }
          break;
        case 6:
          switch (mainSettings.bright_mode) {
            case 1:
              indiPrint("L", 0); //уровень ручной подсветки
              if (!blink_data) indiPrintNum(mainSettings.indiBright[0] + 1, 3); //вывод яркости ночь
              break;
            case 2:
              indiPrint("D", 0); //день
              result = readLightSens(); //считываем сенсор
              if (result > mainSettings.adcMaxAuto) mainSettings.adcMaxAuto = result; //назодим максимум
              indiPrintNum(mainSettings.adcMaxAuto, 1, 3, ' '); //вывод порога день
              break;
          }
          break;
        case 7:
          indiPrint("D", 0);
          if (!blink_data) indiPrintNum(mainSettings.timeBright[1], 2, 2, '0'); //вывод время включения дневной подсветки
          break;
        case 8:
          indiPrint("L", 0); //вывод 2000
          if (!blink_data) indiPrintNum(mainSettings.indiBright[1] + 1, 3); //вывод яркости день
          break;
      }
      blink_data = !blink_data; //мигание сигментами
    }

    //+++++++++++++++++++++  опрос кнопок  +++++++++++++++++++++++++++
    switch (check_keys()) {
      case LEFT_KEY_PRESS: //клик левой кнопкой
        switch (cur_mode) {
          case 0: //настройка колбы
            if (mainSettings.flask_mode > 0) mainSettings.flask_mode--; else mainSettings.flask_mode = 1 + USE_LIGHT_SENS;
            flask_state = mainSettings.flask_mode; //обновление стотояния колбы
            break;
          case 1: //настройка сна
            if (mainSettings.sleep_time > 3) mainSettings.sleep_time--; else mainSettings.sleep_time = 0;
            break;
          case 2: //настройка анимации
            if (mainSettings.anim_mode > 0) mainSettings.anim_mode--; else mainSettings.anim_mode = (FLIP_EFFECT_NUM - 1);
            animFlip(); //анимция перелистывания
            break;
          case 3: //настройка анимации сна
            mainSettings.sleep_anim = !mainSettings.sleep_anim;
            break;
          case 4: //настройка подсветки
            if (mainSettings.bright_mode > 0) mainSettings.bright_mode--; else mainSettings.bright_mode = 1 + USE_LIGHT_SENS;
            switch (mainSettings.bright_mode) {
              case 0: indiSetBright(brightDefault[mainSettings.bright_levle]); break; //установка яркости индикаторов
              case 1: indiSetBright(brightDefault[mainSettings.indiBright[checkTimeBright()]]); break; //установка яркости индикаторов
              case 2: indiSetBright(brightDefault[lightSens()]); break; //установка яркости индикаторов
            }
            break;

          //настройка ночной подсветки
          case 5:
            switch (mainSettings.bright_mode) {
              case 0: //ручная подсветка
                if (mainSettings.bright_levle > 0) mainSettings.bright_levle--; else mainSettings.bright_levle = 4;
                indiSetBright(brightDefault[mainSettings.bright_levle]); //установка яркости индикаторов
                break;

              case 1: //часы ночь
                if (mainSettings.timeBright[0] > 0) mainSettings.timeBright[0]--; else mainSettings.timeBright[0] = 23; //часы
                break;

              case 2: //авто-подсветка
                mainSettings.adcMinAuto = readLightSens();
                break;
            }
            break;
          case 6:
            switch (mainSettings.bright_mode) {
              case 1:
                if (mainSettings.indiBright[0] > 0) mainSettings.indiBright[0]--; else mainSettings.indiBright[0] = 4;
                indiSetBright(brightDefault[mainSettings.indiBright[0]]); //установка яркости индикаторов
                break;

              case 2:
                mainSettings.adcMaxAuto = readLightSens();
                break;
            }
            break;

          //настройка дневной подсветки
          case 7: if (mainSettings.timeBright[1] > 0) mainSettings.timeBright[1]--; else mainSettings.timeBright[1] = 23; break; //часы
          case 8:
            if (mainSettings.indiBright[1] > 0) mainSettings.indiBright[1]--; else mainSettings.indiBright[1] = 4;
            indiSetBright(brightDefault[mainSettings.indiBright[1]]); //установка яркости индикаторов
            break;
        }
        blink_data = 0; //сбрасываем флаг мигания
        break;

      case RIGHT_KEY_PRESS: //клик правой кнопкой
        switch (cur_mode) {
          case 0: //настройка колбы
            if (mainSettings.flask_mode < 1 + USE_LIGHT_SENS) mainSettings.flask_mode++; else mainSettings.flask_mode = 0;
            flask_state = mainSettings.flask_mode; //обновление стотояния колбы
            break;
          case 1: //настройка сна
            if (!mainSettings.sleep_time) mainSettings.sleep_time = 3; else if (mainSettings.sleep_time < 15) mainSettings.sleep_time++; else mainSettings.sleep_time = 3;
            break;
          case 2: //настройка анимации
            if (mainSettings.anim_mode < (FLIP_EFFECT_NUM - 1)) mainSettings.anim_mode++; else mainSettings.anim_mode = 0;
            animFlip(); //анимция перелистывания
            break;
          case 3: //настройка анимации сна
            mainSettings.sleep_anim = !mainSettings.sleep_anim;
            break;
          case 4: //настройка подсветки
            if (mainSettings.bright_mode < 1 + USE_LIGHT_SENS) mainSettings.bright_mode++; else mainSettings.bright_mode = 0;
            switch (mainSettings.bright_mode) {
              case 0: indiSetBright(brightDefault[mainSettings.bright_levle]); break; //установка яркости индикаторов
              case 1: indiSetBright(brightDefault[mainSettings.indiBright[checkTimeBright()]]); break; //установка яркости индикаторов
              case 2: indiSetBright(brightDefault[lightSens()]); break; //установка яркости индикаторов
            }
            break;

          //настройка ночной подсветки
          case 5:
            switch (mainSettings.bright_mode) {
              case 0: //ручная подсветка
                if (mainSettings.bright_levle < 4) mainSettings.bright_levle++; else mainSettings.bright_levle = 0;
                indiSetBright(brightDefault[mainSettings.bright_levle]); //установка яркости индикаторов
                break;

              case 1: //часы ночь
                if (mainSettings.timeBright[0] < 23) mainSettings.timeBright[0]++; else mainSettings.timeBright[0] = 0; //часы
                break;

              case 2: //авто-подсветка
                mainSettings.adcMinAuto = readLightSens();
                break;
            }
            break;
          case 6:
            switch (mainSettings.bright_mode) {
              case 1:
                if (mainSettings.indiBright[0] < 4) mainSettings.indiBright[0]++; else mainSettings.indiBright[0] = 0;
                indiSetBright(brightDefault[mainSettings.indiBright[0]]); //установка яркости индикаторов
                break;

              case 2:
                mainSettings.adcMaxAuto = readLightSens();
                break;
            }
            break;

          //настройка дневной подсветки
          case 7: if (mainSettings.timeBright[1] < 23) mainSettings.timeBright[1]++; else mainSettings.timeBright[1] = 0; break; //часы
          case 8:
            if (mainSettings.indiBright[1] < 4) mainSettings.indiBright[1]++; else mainSettings.indiBright[1] = 0;
            indiSetBright(brightDefault[mainSettings.indiBright[1]]); //установка яркости индикаторов
            break;
        }
        blink_data = 0; //сбрасываем флаг мигания
        break;

      case LEFT_KEY_HOLD: //удержание левой кнопки
        if (cur_mode < allModes[mainSettings.bright_mode & 0x7F]) cur_mode++; else cur_mode = 0;
        switch (cur_mode) {
          case 0:
            indiClr(); //очистка индикаторов
            indiPrint("FLS", 0); //колба
            _wait(TIME_MSG_PNT); //ждем
            if (mainSettings.bright_mode == 1) indiSetBright(brightDefault[mainSettings.indiBright[checkTimeBright()]]); //установка яркости индикаторов
            mainSettings.bright_mode &= 0x7F; //разрешаем управление подсветкой
            break;

          case 1:
            indiClr(); //очистка индикаторов
            indiPrint("SLP", 0); //сон
            _wait(TIME_MSG_PNT); //ждем
            break;

          case 2:
            indiClr(); //очистка индикаторов
            indiPrint("ANI", 0); //анимация
            _wait(TIME_MSG_PNT); //ждем
            break;

          case 3:
            indiClr(); //очистка индикаторов
            indiPrint("ANS", 0); //анимация
            _wait(TIME_MSG_PNT); //ждем
            break;

          case 4:
            indiClr(); //очистка индикаторов
            indiPrint("BRI", 0); //подсветка
            _wait(TIME_MSG_PNT); //ждем
            break;

          case 5:
            indiClr(); //очистка индикаторов
            if (mainSettings.bright_mode) indiPrint("NHT", 0); //ночь
            else indiPrint("L-T", 0); //уровень подсветки
            _wait(TIME_MSG_PNT); //ждем
            break;

          case 6:
            if (mainSettings.bright_mode == 2) {
              indiClr(); //очистка индикаторов
              indiPrint("DAY", 0); //день
              _wait(TIME_MSG_PNT); //ждем
            }
            else {
              indiSetBright(brightDefault[mainSettings.indiBright[0]]);
              mainSettings.bright_mode |= 0x80; //запрещаем управление подсветкой
            }
            break;

          case 7:
            indiClr(); //очистка индикаторов
            indiPrint("DAY", 0); //день
            _wait(TIME_MSG_PNT); //ждем
            indiSetBright(brightDefault[checkTimeBright()]); //установка яркости индикаторов
            mainSettings.bright_mode &= 0x7F; //разрешаем управление подсветкой
            break;

          case 8:
            indiSetBright(brightDefault[mainSettings.indiBright[1]]);
            mainSettings.bright_mode |= 0x80; //запрещаем управление подсветкой
            break;
        }
        blink_data = 0; //сбрасываем флаг мигания
        break;

      case RIGHT_KEY_HOLD: //удержание правой кнопки
        mainSettings.bright_mode &= 0x7F; //разрешаем управление подсветкой
        updateData((uint8_t*)&mainSettings, sizeof(mainSettings), EEPROM_BLOCK_SETTINGS_MAIN, EEPROM_BLOCK_CRC_MAIN); //записываем основные настройки в память
        changeBright(); //смена яркости индикаторов
        dot_state = 0; //выключаем точку
        indiClr(); //очистка индикаторов
        indiPrint("OUT", 0);
        _wait(TIME_MSG); //ждем
        if (timerSettings.timer_mode || !_timer_start || _timer_sec > timerSettings.timer_blink) _disableSleep = 0; //разрешаем сон
        if (_mode != 3) _mode = 0; //переходим в режим часов
        return;
    }
  }
}
//----------------------------------------------------------------------------------
void set_timer(void)
{
  uint8_t cur_mode = 0; //текущий режим
  boolean blink_data = 0; //мигание сигментами

  dot_state = 0; //выключаем точку
  indiClr(); //очищаем индикаторы
  indiPrint("TSET", 0);
  _wait(TIME_MSG); //ждем

  //настройки
  while (1) {
    data_update(); //обновление данных

    if (!_scr) {
      _scr = 1; //сбрасываем флаг
      indiClr(); //очистка индикаторов
      switch (cur_mode) {
        case 0:
          indiPrint("P-", 0);
          if (!blink_data) {
            switch (timerSettings.timer_mode) {
              case 0: indiPrint("T", 3); break; //таймер
              case 1: indiPrint("S", 3); break; //секундомер
            }
          }
          break;
        case 1:
          indiPrint("T", 0);
          if (!blink_data) indiPrintNum(timerDefault[timerSettings.timer_preset], 2, 2, '0'); //вывод времени таймера
          break;

        case 2:
          indiPrint("B", 0);
          if (!blink_data) indiPrintNum(timerSettings.timer_blink, 2, 2, '0'); //вывод времени мигания таймера
          break;
      }
      blink_data = !blink_data; //мигание сигментами
    }

    //+++++++++++++++++++++  опрос кнопок  +++++++++++++++++++++++++++
    switch (check_keys()) {
      case LEFT_KEY_PRESS: //клик левой кнопкой
        switch (cur_mode) {
          case 0: timerSettings.timer_mode = 0; _timer_start = 0; _timer_sec = timerDefault[timerSettings.timer_preset] * 60; break;
          case 1: if (timerSettings.timer_preset > 0) timerSettings.timer_preset--; else timerSettings.timer_preset = 9; break;
          case 2: if (timerSettings.timer_blink > 5) timerSettings.timer_blink--; else timerSettings.timer_blink = 60; break;
        }
        blink_data = 0; //сбрасываем флаг мигания
        break;

      case RIGHT_KEY_PRESS: //клик правой кнопкой
        switch (cur_mode) {
          case 0: timerSettings.timer_mode = 1; _timer_start = 0; _timer_sec = 0; break;
          case 1: if (timerSettings.timer_preset < 9) timerSettings.timer_preset++; else timerSettings.timer_preset = 0; break;
          case 2: if (timerSettings.timer_blink < 60) timerSettings.timer_blink++; else timerSettings.timer_blink = 5; break;
        }
        blink_data = 0; //сбрасываем флаг мигания
        break;

      case LEFT_KEY_HOLD: //удержание левой кнопки
        if (cur_mode < 2) cur_mode++; else cur_mode = 0;
        indiClr(); //очистка индикаторов
        switch (cur_mode) {
          case 0: indiPrint("PAR", 0); break;
          case 1: indiPrint("TPR", 0); break;
          case 2: indiPrint("BLK", 0); break;
        }
        _wait(TIME_MSG_PNT); //ждем
        blink_data = 0; //сбрасываем флаг мигания
        break;

      case RIGHT_KEY_HOLD: //удержание правой кнопки
        updateData((uint8_t*)&timerSettings, sizeof(timerSettings), EEPROM_BLOCK_SETTINGS_TIMER, EEPROM_BLOCK_CRC_TIMER); //записываем настройки таймера в память
        indiClr(); //очистка индикаторов
        indiPrint("OUT", 0);
        _wait(TIME_MSG); //ждем
        if (timerSettings.timer_mode || !_timer_start || _timer_sec > timerSettings.timer_blink) _disableSleep = 0; //разрешаем сон
        return;
    }
  }
}
//-----------------------------Главный экран------------------------------------------------
void main_screen(void) //главный экран
{
  for (;;) //главная
  {
    data_update(); //обновление данных

    if (!_sleep) { //если не спим
      checkMessage(); //проверка оповещений
      if (!sleepMode()) { //если не вошли в режим сна

        if (_animStart) {
          animFlip(); //анимция перелистывания
          _timer_sleep = 0; //сбрасываем таймер сна
        }

        if (!_scr) {
          _scr = 1; //сбрасываем флаг
          switch (_mode) {
            case 0: //режим часов
              indiPrintNum((RTC.timeFormat) ? get_12h(RTC.h) : RTC.h, 0, 2, '0'); //вывод часов
              indiPrintNum(RTC.m, 2, 2, '0'); //вывод минут
              break;
            case 1: //режим заряда акб
              indiPrint("B", 0);
              indiPrintNum(bat, 1, 3, ' '); //вывод заряда акб
              dot_state = 0; //выключаем точки
              break;
            case 2: //режим даты
              indiPrintNum(RTC.DD, 0, 2, '0'); //вывод даты
              indiPrintNum(RTC.MM, 2, 2, '0'); //вывод месяца
              dot_state = 1; //включаем точки
              break;
            case 3: //режим таймера
              indiPrintNum(_timer_sec / 60, 0, 2, '0'); //вывод минут
              indiPrintNum(_timer_sec % 60, 2, 2, '0'); //вывод секунд
              break;
          }
        }

        switch (_mode) {
          case 0: //режим часов
            dotFlash(); //мигаем точками
            break;

          case 3: //режим таймера
            dotFlashTimer(); //мигаем точками
            break;
        }

        switch (check_keys()) {
          case LEFT_KEY_PRESS: //клик левой кнопкой
            if (_mode != 3) _mode = 3; //если не в режиме таймера, переходим в режим таймера
            else { //иначе
              _timer_start = !_timer_start; //запуск - остановка таймера
              switch (timerSettings.timer_mode) {
                case 0: //режим таймера
                  if (_timer_start && _timer_sec <= timerSettings.timer_blink) _disableSleep = 1; //запрещаем сон
                  else if (_disableSleep) _disableSleep = 0; //разрешаем сон
                  break;
              }
            }
            break;

          case RIGHT_KEY_PRESS: //клик правой кнопкой
            switch (timerSettings.timer_mode) {
              case 0: //режим таймера
                if (!_timer_start || _timer_sec > timerSettings.timer_blink) { //если таймер выключен или время таймера больше установлнного
                  if (_mode < 2) _mode++; else _mode = 0; //переключаем режимы времени
                }
                else {
                  _timer_sec = timerDefault[timerSettings.timer_preset] * 60; //иначе перезапускаем таймер
                  if (_disableSleep) _disableSleep = 0; //разрешаем сон
                }
                break;
              case 1: //режим секундомера
                if (_mode < 2) _mode++; else _mode = 0; //переключаем режимы времени
                break;
            }
            break;

          case LEFT_KEY_HOLD: //удержание левой кнопки
            if (_mode != 3) settings_time(); //настройки времени
            else { //сброс таймера
              _timer_start = 0; //выключаем таймер
              switch (timerSettings.timer_mode) {
                case 0: //режим таймера
                  _timer_sec = timerDefault[timerSettings.timer_preset] * 60;
                  if (_disableSleep) _disableSleep = 0; //разрешаем сон
                  break;
                case 1: //режим секундомера
                  _timer_sec = 0; //сброс секундомера
                  break;
              }
            }
            break;

          case RIGHT_KEY_HOLD: //удержание правой кнопки
            if (_mode != 3) settings_bright(); //настройки яркости
            else set_timer(); //настройка таймера
            break;
        }
      }
    }
  }
}
