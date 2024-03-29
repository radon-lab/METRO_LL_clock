/*
  Arduino IDE 1.8.13 версия прошивки 1.4.7 от 10.08.22
  Специльно для проекта "Часы METRO LAST LIGHT"
  Исходник - https://github.com/radon-lab/METRO_LL_clock
  Автор Radon-lab.
*/
//--------------Версия прошивки-------------
#define VERSION_FW 0x92

//----------------Библиотеки----------------
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include <util/delay.h>

//---------------Конфигурации---------------
#include "config.h"
#include "connection.h"
#include "indiDisp.h"
#include "wire.h"
#include "RTC.h"

//переменные обработки кнопок
uint8_t btn_tmr; //таймер тиков обработки
boolean btn_check; //флаг разрешения опроса кнопки
boolean btn_state; //флаг текущего состояния кнопки

uint8_t bat = 100; //заряд акб

uint8_t _mode = 0; //текущий основной режим
uint8_t _msg_type = 0; //тип оповещения
boolean _animStart = 1; //флаг анимации

boolean _timer_start = 0; //флаг работы таймера(0 - выкл | 1 - вкл)
uint16_t _timer_secs = 0; //установленное время таймера

boolean _flask_block = 0; //блокировка управления колбой
boolean _bright_block = 0; //блокировка изменения яркости подсветки яркость подсветки

boolean _scr = 0; //флаг обновления экрана
boolean _disableSleep = 0; //флаг запрета сна

uint8_t _timer_sleep = 0; //счетчик ухода в сон
boolean _sleep = 0; //флаг активного сна

uint8_t time[7]; //массив времени(год, месяц, день, день_недели, часы, минуты, секунды)

volatile uint8_t tick_wdt; //счетчик тиков для обработки данных
uint32_t timer_millis; //таймер отсчета миллисекунд
uint32_t timer_dot; //таймер отсчета миллисекунд для точек

struct Settings1 {
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

struct Settings2 {
  boolean timer_mode = DEFAULT_MODE_TIMER; //режим работы таймера
  uint8_t timer_preset = DEFAULT_PRESET_TIMER; //текущий номер выбранного пресета таймера
  uint8_t timer_blink = DEFAULT_BLINK_TIMER; //время мигания таймера
} timerSettings;

#define EEPROM_BLOCK_TIME EEPROM_BLOCK_NULL //блок памяти времени
#define EEPROM_BLOCK_SETTINGS_MAIN (EEPROM_BLOCK_TIME + sizeof(time)) //блок памяти основных настроек
#define EEPROM_BLOCK_SETTINGS_TIMER (EEPROM_BLOCK_SETTINGS_MAIN + sizeof(mainSettings)) //блок памяти настроек таймера

int atexit(void (* /*func*/ )()) { //инициализация функций
  return 0;
}

int main(void)  //инициализация
{
  LEFT_INIT; //инициализация левой кнопки
  RIGHT_INIT; //инициализация правой кнопки
  DOT_INIT; //инициализация точек
  FLASK_INIT; //инициализация колбы
  RTC_BAT_INIT; //инициализация дополнительного питания RTC
  RTC_INIT; //инициализация питания RTC

  PRR = 0b10101111; //отключаем все лишнее (I2C | TIMER0 | TIMER1 | SPI | UART | ADC)
  ACSR |= 1 << ACD; //отключаем компаратор

  indiInit(); //инициализация индикаторов
  _PowerDown(); //выключаем питание

  if (eeprom_read_byte((uint8_t*)EEPROM_BLOCK_VERSION_FW) != VERSION_FW) { //если первый запуск, восстанавливаем из переменных
    eeprom_update_byte((uint8_t*)EEPROM_BLOCK_VERSION_FW, VERSION_FW); //делаем метку
    eeprom_update_block((void*)&timeDefault, (void*)EEPROM_BLOCK_TIME, sizeof(timeDefault)); //записываем дату по умолчанию в память
    eeprom_update_block((void*)&mainSettings, (void*)EEPROM_BLOCK_SETTINGS_MAIN, sizeof(mainSettings)); //записываем основные настройки в память
    eeprom_update_block((void*)&timerSettings, (void*)EEPROM_BLOCK_SETTINGS_TIMER, sizeof(timerSettings)); //записываем настройки таймера в память

  }
  else {
    eeprom_read_block((void*)&mainSettings, (void*)EEPROM_BLOCK_SETTINGS_MAIN, sizeof(mainSettings)); //считываем основные настройки из памяти
    eeprom_read_block((void*)&timerSettings, (void*)EEPROM_BLOCK_SETTINGS_TIMER, sizeof(timerSettings)); //считываем настройки таймера из памяти
  }

  if (time[0] < 21 || time[0] > 50) { //если пропадало питание
    indiPrint("INIT", 0);
    eeprom_read_block((void*)&timeDefault, EEPROM_BLOCK_TIME, sizeof(timeDefault)); //считываем дату из памяти
    TimeSetDate(timeDefault); //устанавливаем новое время
  }
  for (timer_millis = 2000; timer_millis && !check_keys();) data_convert(); // ждем, преобразование данных

  flask_state = mainSettings.flask_mode; //обновление стотояния колбы
  _timer_secs = timerDefault[timerSettings.timer_preset] * 60; //установленное время таймера

  switch (mainSettings.bright_mode) {
    case 0: indiSetBright(brightDefault[mainSettings.bright_levle]); break; //установка яркости индикаторов
    case 1: indiSetBright(brightDefault[mainSettings.indiBright[changeBright()]]); break; //установка яркости индикаторов
    case 2: indiSetBright(brightDefault[lightSens()]); break; //установка яркости индикаторов
  }

  //----------------------------------Главная-------------------------------------------------------------
  for (;;) //главная
  {
    data_convert(); //преобразование данных
    sleepMode(); //режим сна
    main_screen(); //главный экран
  }
  return 0; //конец
}
//-------------------------Прерывание по переполнению wdt - 17.5мс------------------------------------
ISR(WDT_vect) //прерывание по переполнению wdt - 17.5мс
{
  tick_wdt++; //прибавляем тик
}
//----------------------------------Преобразование данных---------------------------------------------------------
void data_convert(void) //преобразование данных
{
  static uint8_t wdt_time; //счетчик времени
  static uint8_t tmr_bat; //таймер опроса батареи

  if (!_sleep) waint_pwr(); //если не спим - режим ожидания
  else sleep_pwr(); //иначе режим сна

  for (; tick_wdt > 0; tick_wdt--) { //если был тик, обрабатываем данные

    switch (btn_state) { //таймер опроса кнопок
      case 0: if (btn_check) btn_tmr++; break; //считаем циклы
      case 1: if (btn_tmr > 0) btn_tmr--; break; //убираем дребезг
    }

    if (timer_millis > 17) timer_millis -= 17; //если таймер больше 17мс
    else if (timer_millis) timer_millis = 0; //иначе сбрасываем таймер

    if (timer_dot > 17) timer_dot -= 17; //если таймер больше 17мс
    else if (timer_dot) timer_dot = 0; //иначе сбрасываем таймер

    if (++wdt_time >= 57) { //если прошла секунда
      wdt_time = 0; //сбрасываем таймер секунды

      //таймер часов
      switch (timerSettings.timer_mode) {
        case 0:
          if (_timer_start && _timer_secs) {
            _timer_secs--; //уменьшаем таймер на 1сек
            //если осталось мало времени
            if (_timer_secs == timerSettings.timer_blink) {
              _mode = 3; //переходим в режим таймера
              _disableSleep = 1; //запрещаем сон
              if (_sleep) sleepOut(); //выход из сна
            }
            //оповещение окончания таймера
            if (!_timer_secs) _msg_type = 1; //если таймер досчитал до 0
          }
          break;

        case 1:
          if (_timer_start) {
            if (_timer_secs < 6000) _timer_secs++; //увеличиваем таймер на 1сек
            else _timer_start = 0; //если секундомер переполнен выключаем таймер
          }
          break;
      }
      //опрос акб
      if (tmr_bat >= BAT_TIME) { //если пришло время опросит акб
        tmr_bat = 0; //сбрасываем таймер
        _batCheck(); //проверяем заряд акб
        if (bat < LOW_BAT_P) _msg_type = 3; //если батарея разряжена
      }
      else tmr_bat++; //иначе прибавляем время

      if (!_sleep) { //если не спим
        //счет времени
        if (++time[6] > 59) { //секунды
          time[6] = 0;
          if (++time[5] > 59) { //минуты
            time[5] = 0;
            if (++time[4] > 23) { //часы
              time[4] = 0;
            }
            if (!_bright_block && mainSettings.bright_mode == 1) indiSetBright(brightDefault[mainSettings.indiBright[changeBright()]]); //установка яркости индикаторов
          }
          TimeGetDate(time); //синхронизируем время
        }
        if (!_flask_block && mainSettings.flask_mode == 2) flask_state = lightSens(); //автоматическое включение колбы
        if (!_bright_block && mainSettings.bright_mode == 2) indiSetBright(brightDefault[lightSens()]); //установка яркости индикаторов

        //сон
        if (_timer_sleep <= mainSettings.sleep_time) _timer_sleep++; //таймер ухода в сон
        _scr = 0; //разрешаем обновить индикаторы
      }
    }
  }
}
//-------------------------------Режим сна----------------------------------------------------
void sleepMode(void) //режим сна
{
  if (!_sleep) {
    if (!_disableSleep && mainSettings.sleep_time && _timer_sleep == mainSettings.sleep_time) {
      if (mainSettings.sleep_anim) {
        uint8_t indic[4]; //буфер анимации
        for (uint8_t s = 0; s < 4; s++) indic[s] = readLightSens() % 7; //выбираем рндомный сигмент
        for (uint8_t c = 0; c < 7;) { //отрисовываем анимацию
          data_convert(); //преобразование данных
          if (check_keys()) return; //если нажата кнопка прерываем сон
          if (!timer_millis) { //если таймер истек
            timer_millis = SLEEP_ANIM_TIME; //устанавливаем таймер
            c++; //смещаем анимацию
            for (uint8_t i = 0; i < 4; i++) { //стираем сигменты
              if (indic[i] < 6) indic[i]++; //если не достигнут максимум
              else indic[i] = 0; //иначе сбрасываем в начало
              indiSet(indic[i], i, 0); //очищаем сигменты по очереди
            }
          }
        }
      }
      _sleep = 1; //устанавливаем флаг активного сна
      TWI_disable(); //выключение TWI
      indiEnableSleep(); //выключаем дисплей
      if (!_timer_start) sleepDeep(); //глубокий сон
    }
  }
}
//-----------------------------Выход из глубокого сна--------------------------------------------
EMPTY_INTERRUPT(PCINT2_vect); //внешнее прерывание PCINT2
//-------------------------------Глубокий сон----------------------------------------------------
void sleepDeep(void) //глубокий сон
{
  WDT_disable(); //выключение WDT

  PCMSK2 = 0b00000101; //разрешаем прерывания от D0 и D2
  PCICR = 0b00000100; //разрешаем внешнее прерывание PCINT2

  while (1) {
    sleep_pwr(); //глубокий сон

    uint16_t startDellay = BTN_HOLD_TICK * 17; //устанавливаем таймер
    while (!RIGHT_OUT || !LEFT_OUT) { //ждем пока отпустят кнопку
      if (startDellay) { //если время не истекло
        _delay_ms(1); //ждем 1мс
        startDellay--; //отнимаем от таймера 1 мс
      }
    }
    if (startDellay) {
      _timer_sleep = 0; //сбрасываем таймер сна
      PCICR = 0b00000000; //запрещаем прерывания PCINT2
      sleepOut(); //выход из сна
      WDT_enable(); //включение WDT
      return; //выходим
    }
  }
}
//-------------------------------------Выход из сна--------------------------------------------------------
void sleepOut(void) //выход из сна
{
  _sleep = 0; //сбрасываем флаг активного сна
  if (_mode != 3) {
    _mode = 0; //если таймер не работает, переходим в режим часов
    _animStart = 1; //разрешаем анимацию
  }
  TWI_enable(); //включение TWI
  TimeGetDate(time); //синхронизируем время
  switch (mainSettings.bright_mode) {
    case 1: indiSetBright(brightDefault[mainSettings.indiBright[changeBright()]]); break; //установка яркости индикаторов
    case 2: indiSetBright(brightDefault[lightSens()]); break; //установка яркости индикаторов
  }
  indiDisableSleep(); //включаем дисплей
  _batCheck(); //проверяем заряд акб
  if (bat < LOW_BAT_P) _msg_type = 3; //если батарея разряжена
  else if (bat < MSG_BAT_P) _msg_type = 2; //если осталось мало заряда
}
//-------------------------------Оповещения таймера----------------------------------------------------
void timerMessage(void) //оповещения таймера
{
  dot_state = 0; //выключаем точки
  _msg_type = 0; //сбрасываем тип оповещения
  _flask_block = 1; //запрещаем управление колбой
  _timer_start = 0; //сбрасываем режим таймера
  for (timer_millis = TIME_MSG_TMR_OVF; timer_millis && !check_keys();) {
    data_convert(); //преобразование данных
    if (!timer_dot) { //если таймер отработал
      indiClr(); //очистка индикаторов
      if (!flask_state) indiPrint("TOUT", 0); //если колба не горит
      flask_state = !flask_state; //инвертируем колбу
      timer_dot = 500; //устанавливаем таймер
    }
  }
  _timer_secs = timerDefault[timerSettings.timer_preset] * 60; //устанавливаем таймер в начало
  flask_state = mainSettings.flask_mode; //обновление стотояния колбы
  _mode = 0; //переходим в режим часов
  _disableSleep = 0; //разрешаем сон
  _timer_sleep = 0; //сбрасываем таймер сна
  _flask_block = 0; //разрешаем управление колбой
}
//-------------------------------Оповещения батареи----------------------------------------------------
void lowBatMessage(void) //оповещения батареи
{
  _msg_type = 0; //сбрасываем тип оповещения
  _flask_block = 1; //запрещаем управление колбой
  dot_state = 0; //выключаем точки
  uint8_t pos = 0; //позиция надписи
  for (timer_millis = TIME_MSG_BAT; timer_millis && !check_keys();) {
    data_convert(); //преобразование данных
    if (!timer_dot) { //если таймер отработал
      indiClr(); //очистка индикаторов
      indiPrint("LO", (pos > 2) ? 1 : pos); //отрисовка сообщения разряженной батареи
      if (pos < 3) pos++; else pos = 0; //меняем позицию
      flask_state = !flask_state; //инвертируем колбу
      timer_dot = 500; //устанавливаем таймер
    }
  }
  flask_state = mainSettings.flask_mode; //обновление стотояния колбы
  _timer_sleep = 0; //сбрасываем таймер сна
  _flask_block = 0; //разрешаем управление колбой
}
//-------------------------------Оповещения выключения----------------------------------------------------
void pwrDownMessage(void) //оповещения выключения
{
  if (_sleep) sleepOut(); //выход из сна
  _msg_type = 0; //сбрасываем тип оповещения
  dot_state = 0; //выключаем точки
  indiPrint(" OFF", 0); //отрисовка сообщения разряженной батареи
  uint16_t flask_tmr = 0; //таймер мигания колбой
  for (uint16_t tmr = TIME_MSG_BAT; tmr; tmr--) { //ждем
    if (!flask_tmr) { //если таймер отработал
      flask_state = !flask_state; //инвертируем колбу
      flask_tmr = 500; //устанавливаем таймер
    }
    else flask_tmr--; //отнимаем таймер мигания колбой
    _delay_ms(1); //ждем 1мс
  }
  eeprom_update_block((void*)&time, (void*)EEPROM_BLOCK_TIME, sizeof(time)); //записываем дату в память
  flask_state = mainSettings.flask_mode; //обновление стотояния колбы
  _timer_sleep = 0; //сбрасываем таймер сна
  _PowerDown(); //выключаем питание
}
//-------------------------------Анимция перелистывания----------------------------------------------------
void animFlip(void) //анимция перелистывания
{
  boolean stopIndi[4]; //буфер анимации
  uint8_t anim_buf[4]; //буфер анимации
  uint8_t indiPos = 0; //позиция индикатора
  uint8_t numState = 0; //фаза числа
  uint8_t stopTick = 0; //фаза числа

  _animStart = 0; //запрещаем анимацию

  switch (mainSettings.anim_mode) {
    case 1:
      anim_buf[0] = time[4] / 10; //часы
      anim_buf[1] = time[4] % 10; //часы
      anim_buf[2] = time[5] / 10; //минуты
      anim_buf[3] = time[5] % 10; //минуты

      for (uint8_t i = 0; i < 10;) {
        data_convert(); //преобразование данных
        dotFlash(); //мигаем точками
        if (check_keys()) return;
        if (!timer_millis) { //если таймер отработал
          for (uint8_t n = 0; n < 4; n++) {
            if (anim_buf[n] < 9) anim_buf[n]++; else anim_buf[n] = 0;
            indiPrintNum(anim_buf[n], n); //вывод часов
          }
          i++; //прибавляем цикл
          timer_millis = animTime[0]; //устанавливаем таймер
        }
      }
      break;

    case 2:
      for (uint8_t i = 0; i < 4; i++) anim_buf[i] = 9; //буфер анимации

      for (uint8_t i = 1; i;) {
        data_convert(); //преобразование данных
        dotFlash(); //мигаем точками
        if (check_keys()) return;
        if (!timer_millis) { //если таймер отработал
          i = 0; //сбрасываем счетчик циклов
          indiPrintNum(anim_buf[0], 0); //вывод часов
          if (anim_buf[0] > time[4] / 10) {
            anim_buf[0]--;
            i++;
          }
          indiPrintNum(anim_buf[1], 1); //вывод часов
          if (anim_buf[1] > time[4] % 10) {
            anim_buf[1]--;
            i++;
          }
          indiPrintNum(anim_buf[2], 2); //вывод минут
          if (anim_buf[2] > time[5] / 10) {
            anim_buf[2]--;
            i++;
          }
          indiPrintNum(anim_buf[3], 3); //вывод минут
          if (anim_buf[3] > time[5] % 10) {
            anim_buf[3]--;
            i++;
          }
          timer_millis = animTime[1]; //устанавливаем таймер
        }
      }
      break;

    case 3:
      anim_buf[3] = time[4] / 10; //часы
      anim_buf[2] = time[4] % 10; //часы
      anim_buf[1] = time[5] / 10; //минуты
      anim_buf[0] = time[5] % 10; //минуты

      for (uint8_t i = 0; i < 4;) {
        data_convert(); //преобразование данных
        dotFlash(); //мигаем точками
        if (check_keys()) return;
        if (!timer_millis) { //если таймер отработал
          for (uint8_t b = 0; b < 4; b++) {
            if (b <= i) indiPrintNum(anim_buf[i - b], b); //вывод часов
            else indiPrint(" ", b); //вывод пустого символа
          }
          i++; //прибавляем цикл
          timer_millis = animTime[2]; //устанавливаем таймер
        }
      }
      break;

    case 4:
      anim_buf[0] = time[4] / 10; //часы
      anim_buf[1] = time[4] % 10; //часы
      anim_buf[2] = time[5] / 10; //минуты
      anim_buf[3] = time[5] % 10; //минуты

      for (uint8_t i = 0; i < 4;) {
        data_convert(); //преобразование данных
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
          timer_millis = animTime[3]; //устанавливаем таймер
        }
      }
      break;

    case 5:
      for (uint8_t i = 0; i < 4; i++) anim_buf[i] = stopIndi[i] = 0; //буфер анимации

      for (uint8_t i = 1; i;) {
        data_convert(); //преобразование данных
        dotFlash(); //мигаем точками
        if (check_keys()) return;
        if (!timer_millis) { //если таймер отработал
          if (numState < 4) numState++;
          else {
            numState = i = stopTick = 0;
            if (anim_buf[0] < time[4] / 10) {
              anim_buf[0]++;
              i++;
            }
            else stopTick += 17;
            if (anim_buf[1] < time[4] % 10) {
              anim_buf[1]++;
              i++;
            }
            else stopTick += 17;
            if (anim_buf[2] < time[5] / 10) {
              anim_buf[2]++;
              i++;
            }
            else stopTick += 17;
            if (anim_buf[3] < time[5] % 10) {
              anim_buf[3]++;
              i++;
            }
            else stopTick += 17;
          }
          if (!stopIndi[0] && anim_buf[0] == time[4] / 10 && numState >= 2) stopIndi[0] = 1;
          if (!stopIndi[1] && anim_buf[1] == time[4] % 10 && numState >= 2) stopIndi[1] = 1;
          if (!stopIndi[2] && anim_buf[2] == time[5] / 10 && numState >= 2) stopIndi[2] = 1;
          if (!stopIndi[3] && anim_buf[3] == time[5] % 10 && numState >= 2) stopIndi[3] = 1;

          for (uint8_t c = 0; c < 4; c++) {
            if (!stopIndi[c]) indi_buf[c] = numbersForAnim[anim_buf[c]][numState]; //отрисовываем
            else indiPrintNum(anim_buf[c], c);
          }
          timer_millis = animTime[4] + stopTick; //устанавливаем таймер
        }
      }
      break;

    case 6:
      anim_buf[0] = time[4] / 10; //часы
      anim_buf[1] = time[4] % 10; //часы
      anim_buf[2] = time[5] / 10; //минуты
      anim_buf[3] = time[5] % 10; //минуты

      for (uint8_t i = 0; i < 4;) {
        data_convert(); //преобразование данных
        dotFlash(); //мигаем точками
        if (check_keys()) return;
        if (!timer_millis) { //если таймер отработал
          if (numState < 2) numState++;
          else {
            numState = 0; //переходим к следующему индикатору
            i++; //прибавляем цикл
          }
          indi_buf[i] = numbersForAnim[anim_buf[i]][numState]; //отрисовываем
          timer_millis = animTime[5]; //устанавливаем таймер
        }
      }
      break;
  }
}
//-------------------------------------Ожидание--------------------------------------------------------
void waint_pwr(void) //ожидание
{
  SMCR = (0x0 << 1) | (1 << SE);  //устанавливаем режим сна idle

  MCUCR = (0x03 << 5); //выкл bod
  MCUCR = (0x02 << 5);

  asm ("sleep");  //с этого момента спим.
}
//-------------------------------------Энергосбережение--------------------------------------------------------
void save_pwr(void) //энергосбережение
{
  SMCR = (0x7 << 1) | (1 << SE);  //устанавливаем режим сна extstandby

  MCUCR = (0x03 << 5); //выкл bod
  MCUCR = (0x02 << 5);

  asm ("sleep");  //с этого момента спим.
}
//-------------------------------------Глубокий сон-----------------------------------------------------------
void sleep_pwr(void) //глубокий сон
{
  SMCR = (0x2 << 1) | (1 << SE);  //устанавливаем режим сна powerdown

  MCUCR = (0x03 << 5); //выкл bod
  MCUCR = (0x02 << 5);

  asm ("sleep");  //с этого момента спим.
}
//-------------------------------Чтение датчика освещённости--------------------------------------------------
uint8_t readLightSens(void) //чтение датчика освещённости
{
  uint16_t result = 0; //результат опроса АЦП внутреннего опорного напряжения
  ADC_enable(); //включение ADC
  ADMUX = 0b01100110; //выбор внешнего опорного и А6
  ADCSRA = 0b10000101; //настройка АЦП
  ADCSRA |= (1 << ADSC); //запускаем преобразование

  for (uint8_t i = 0; i < 10; i++) { //делаем 10 замеров
    while (ADCSRA & (1 << ADSC)); //ждем окончания преобразования
    result += ADCH; //прибавляем замер в буфер
    ADCSRA |= (1 << ADSC); //перезапускаем преобразование
  }
  result /= 10; //находим среднее значение
  ADC_disable(); //выключение ADC

  return result; //возвращаем результат
}
//-------------------------------Максимальное количество дней--------------------------------------------------
uint8_t maxDays(void) //максимальное количество дней
{
  return ((time[1] == 2 && !(time[0] % 4)) ? 1 : 0) + daysInMonth[time[1] - 1]; //возвращаем количество дней в месяце
}
//-------------------------------Чтение датчика освещённости--------------------------------------------------
uint8_t lightSens(void) //чтение датчика освещённости
{
  return map(constrain(readLightSens(), mainSettings.adcMinAuto, mainSettings.adcMaxAuto), mainSettings.adcMinAuto + 1, mainSettings.adcMaxAuto - 1, 0, 4); //возвращаем результат
}
//----------------------------------Чтение напряжения батареи-------------------------------------------------
uint8_t Read_VCC(void)  //чтение напряжения батареи
{
  ADC_enable(); //включение ADC
  ADMUX = 0b01101110; //выбор внешнего опорного+BG
  ADCSRA = 0b10000111; //настройка АЦП
  _delay_ms(5); //ждем пока опорное успокоится
  ADCSRA |= (1 << ADSC); //запускаем преобразование
  while (ADCSRA & (1 << ADSC)); //ждем окончания преобразования
  uint8_t resu = ADCH; //результат опроса АЦП
  ADC_disable(); //выключение ADC
  return resu; //возвращаем результат опроса АЦП
}
//-------------------------------Включение WDT----------------------------------------------------
void WDT_enable(void) //включение WDT
{
  uint8_t sregCopy = SREG; //Сохраняем глобальные прерывания
  cli(); //Запрещаем глобальные прерывания
  WDTCSR = ((1 << WDCE) | (1 << WDE)); //Сбрасываем собаку
  WDTCSR = 0x40; //Устанавливаем пределитель 2(режим прерываний)
  SREG = sregCopy; //Восстанавливаем глобальные прерывания
}
//-------------------------------Выключение WDT---------------------------------------------------
void WDT_disable(void) //выключение WDT
{
  uint8_t sregCopy = SREG; //Сохраняем глобальные прерывания
  cli(); //Запрещаем глобальные прерывания
  WDTCSR = ((1 << WDCE) | (1 << WDE)); //Сбрасываем собаку
  WDTCSR = 0x00; //Выключаем собаку
  SREG = sregCopy; //Восстанавливаем глобальные прерывания
}
//-------------------------------Включение ADC----------------------------------------------------
void ADC_enable(void) //включение ADC
{
  PRR &= ~ (1 << 0); //включаем питание АЦП
  ADCSRA |= (1 << ADEN); //включаем ацп
}
//-------------------------------Выключение ADC---------------------------------------------------
void ADC_disable(void) //выключение ADC
{
  ADCSRA &= ~ (1 << ADEN); //выключаем ацп
  PRR |= (1 << 0); //выключаем питание ацп
}
//-------------------------------Включение TWI----------------------------------------------------
void TWI_enable(void) //включение TWI
{
  PRR &= ~ (1 << 7); //включаем питание i2c
  wireInit(); //инициализация шины i2c
}
//-------------------------------Выключение TWI---------------------------------------------------
void TWI_disable(void) //выключение TWI
{
  PRR |= (1 << 7); //выключаем питание i2c
}
//------------------------------------Включение питания----------------------------------------------
EMPTY_INTERRUPT(INT0_vect); //внешнее прерывание на пине INT0 - включение питания
//----------------------------------------------------------------------------------
void _PowerDown(void)
{
  indiEnableSleep(); //выключаем дисплей
  TWI_disable(); //выключение TWI
  WDT_disable(); //выключение WDT

  EICRA = 0b00000010; //настраиваем внешнее прерывание по спаду импульса на INT0
  EIMSK = 0b00000001; //разрешаем внешнее прерывание INT0

  while (1) {
    sleep_pwr(); //спим

    uint16_t startDellay = TIME_PWR_ON; //устанавливаем таймер
    while (!RIGHT_OUT) { //если кнопка не отжата
      if (startDellay) { //если время не истекло
        _delay_ms(1); //ждем 1мс
        startDellay--; //отнимаем от таймера 1 мс
      }
      else { //иначе включаем питание
        _batCheck(); //проверяем заряд акб
        if (bat > PWR_BAT_P) { //если батарея не разряжена
          TWI_enable(); //включение TWI
          TimeGetDate(time); //синхронизируем время

          switch (mainSettings.bright_mode) {
            case 0: indiSetBright(brightDefault[mainSettings.bright_levle]); break; //установка яркости индикаторов
            case 1: indiSetBright(brightDefault[mainSettings.indiBright[changeBright()]]); break; //установка яркости индикаторов
            case 2: indiSetBright(brightDefault[lightSens()]); break; //установка яркости индикаторов
          }

          indiDisableSleep(); //включаем дисплей
          indiPrint("####", 0); //отрисовка сообщения

          while (!RIGHT_OUT); //ждем пока отпустят кнопу

          WDT_enable(); //запускаем WatchDog с пределителем 2
          EIMSK = 0b00000000; //запрещаем внешнее прерывание INT0

          return; //выходим из выключения питания
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
}
//----------------------------------------------------------------------------------
void _batCheck(void)
{
  uint16_t vcc = (1.10 * 255.0) / Read_VCC() * 100; //рассчитываем напряжение
  bat = map(constrain(vcc, BAT_MIN_V, BAT_MAX_V), BAT_MIN_V, BAT_MAX_V, 0, 100); //состояние батареи
}
//-----------------------------Проверка кнопок----------------------------------------------------
uint8_t check_keys(void) //проверка кнопок
{
  static uint8_t btn_set; //флаг признака действия
  static uint8_t btn_switch; //флаг мультиопроса кнопок

  switch (btn_switch) { //переключаемся в зависимости от состояния мультиопроса
    case 0:
      if (!LEFT_OUT) { //если нажата левая кл.
        btn_switch = 1; //выбираем клавишу опроса
        btn_state = 0; //обновляем текущее состояние кнопки
      }
      else if (!RIGHT_OUT) { //если нажата правая кл.
        btn_switch = 2; //выбираем клавишу опроса
        btn_state = 0; //обновляем текущее состояние кнопки
      }
      else btn_state = 1; //обновляем текущее состояние кнопки
      break;
    case 1: btn_state = LEFT_OUT; break; //опрашиваем левую клавишу
    case 2: btn_state = RIGHT_OUT; break; //опрашиваем правую клавишу
  }

  switch (btn_state) { //переключаемся в зависимости от состояния клавиши
    case 0:
      if (btn_check) { //если разрешена провекрка кнопки
        if (btn_tmr > BTN_HOLD_TICK) { //если таймер больше длительности удержания кнопки
          btn_tmr = BTN_GIST_TICK; //сбрасываем таймер на антидребезг
          if (!_sleep) { //если не спим
            btn_set = 2; //поднимаем признак удержания
            _timer_sleep = 0; //сбрасываем таймер сна
            _scr = 0; //разрешаем обновить экран
          }
          btn_check = 0; //запрещем проврку кнопки
        }
      }
      break;

    case 1:
      if (btn_tmr > BTN_GIST_TICK) { //если таймер больше времени антидребезга
        btn_tmr = BTN_GIST_TICK; //сбрасываем таймер на антидребезг
        if (!_sleep) btn_set = 1; //если не спим, поднимаем признак нажатия
        else sleepOut(); //иначе выход из сна
        btn_check = 0; //запрещем проврку кнопки
        _timer_sleep = 0; //сбрасываем таймер сна
        _scr = 0; //разрешаем обновить экран
      }
      else if (!btn_tmr) {
        btn_check = 1; //разрешаем проврку кнопки
        btn_switch = 0; //сбрасываем мультиопрос кнопок
      }
      break;
  }

  switch (btn_set) { //переключаемся в зависимости от признака нажатия
    case 0: return 0; //клавиша не нажата, возвращаем 0
    case 1:
      btn_set = 0; //сбрасываем признак нажатия
      switch (btn_switch) { //переключаемся в зависимости от состояния мультиопроса
        case 1: return 1; //left press, возвращаем 1
        case 2: return 2; //right press, возвращаем 2
      }
      break;

    case 2:
      btn_set = 0; //сбрасываем признак нажатия
      switch (btn_switch) { //переключаемся в зависимости от состояния мультиопроса
        case 1: return 3; //left hold, возвращаем 3
        case 2: return 4; //right hold, возвращаем 4
      }
      break;
  }
  return 0;
}
//----------------------------------------------------------------------------------
void settings_time(void)
{
  uint8_t cur_mode = 0; //текущий режим
  boolean blink_data = 0; //мигание сигментами

  dot_state = 1; //включаем точку
  indiClr(); //очищаем индикаторы
  indiPrint("SET", 0);
  for (timer_millis = TIME_MSG; timer_millis && !check_keys();) data_convert(); // ждем, преобразование данных

  //настройки
  while (1) {
    data_convert(); //преобразование данных

    if (!_scr) {
      _scr = 1; //сбрасываем флаг
      indiClr(); //очистка индикаторов
      switch (cur_mode) {
        case 0:
        case 1:
          if (!blink_data || cur_mode == 1) indiPrintNum(time[4], 0, 2, '0'); //вывод часов
          if (!blink_data || cur_mode == 0) indiPrintNum(time[5], 2, 2, '0'); //вывод минут
          break;
        case 2:
        case 3:
          if (!blink_data || cur_mode == 3) indiPrintNum(time[2], 0, 2, '0'); //вывод даты
          if (!blink_data || cur_mode == 2) indiPrintNum(time[1], 2, 2, '0'); //вывод месяца
          break;
        case 4:
          indiPrint("20", 0); //вывод 2000
          if (!blink_data) indiPrintNum(time[0], 2, 2, '0'); //вывод года
          break;
      }
      blink_data = !blink_data; //мигание сигментами
    }

    //+++++++++++++++++++++  опрос кнопок  +++++++++++++++++++++++++++
    switch (check_keys()) {
      case 1: //left click
        switch (cur_mode) {
          //настройка времени
          case 0: if (time[4] > 0) time[4]--; else time[4] = 23; break; //часы
          case 1: if (time[5] > 0) time[5]--; else time[5] = 59; break; //минуты

          //настройка даты
          case 2: if (time[2] > 1) time[2]--; else time[2] = maxDays(); break; //день
          case 3: //месяц
            if (time[1] > 1) time[1]--;
            else time[1] = 12;
            if (time[2] > maxDays()) time[2] = maxDays();
            break;

          //настройка года
          case 4: if (time[0] > 20) time[0]--; else time[0] = 50; break; //год
        }
        blink_data = 0; //сбрасываем флаг мигания
        time[6] = 0; //сбрасываем секунды
        break;

      case 2: //right click
        switch (cur_mode) {
          //настройка времени
          case 0: if (time[4] < 23) time[4]++; else time[4] = 0; break; //часы
          case 1: if (time[5] < 59) time[5]++; else time[5] = 0; break; //минуты

          //настройка даты
          case 2: if (time[2] < maxDays()) time[2]++; else time[2] = 1; break; //день
          case 3: //месяц
            if (time[1] < 12) time[1]++;
            else time[1] = 1;
            if (time[2] > maxDays()) time[2] = maxDays();
            break;

          //настройка года
          case 4: if (time[0] < 50) time[0]++; else time[0] = 21; break; //год
        }
        blink_data = 0; //сбрасываем флаг мигания
        time[6] = 0; //сбрасываем секунды
        break;

      case 3: //left hold
        if (cur_mode < 4) cur_mode++; else cur_mode = 0;
        switch (cur_mode) {
          case 0:
            indiClr(); //очистка индикаторов
            indiPrint("T", 0);
            for (timer_millis = TIME_MSG_PNT; timer_millis && !check_keys();) data_convert(); // ждем, преобразование данных
            break;

          case 2:
            indiClr(); //очистка индикаторов
            indiPrint("D", 0);
            for (timer_millis = TIME_MSG_PNT; timer_millis && !check_keys();) data_convert(); // ждем, преобразование данных
            break;

          case 4:
            indiClr(); //очистка индикаторов
            indiPrint("Y", 0);
            for (timer_millis = TIME_MSG_PNT; timer_millis && !check_keys();) data_convert(); // ждем, преобразование данных
            break;
        }
        blink_data = 0; //сбрасываем флаг мигания
        time[6] = 0; //сбрасываем секунды
        break;

      case 4: //right hold
        eeprom_update_block((void*)&time, (void*)EEPROM_BLOCK_TIME, sizeof(time)); //записываем дату в память
        if (mainSettings.bright_mode == 2) indiSetBright(brightDefault[mainSettings.indiBright[changeBright()]]); //установка яркости индикаторов
        TimeSetDate(time); //обновляем время
        dot_state = 0; //выключаем точку
        indiClr(); //очистка индикаторов
        indiPrint("OUT", 0);
        for (timer_millis = TIME_MSG; timer_millis && !check_keys();) data_convert(); // ждем, преобразование данных
        if (timerSettings.timer_mode || !_timer_start || _timer_secs > timerSettings.timer_blink) _disableSleep = 0; //разрешаем сон
        if (_mode != 3) _mode = 0; //переходим в режим часов
        return;
    }
  }
}
//----------------------------------------------------------------------------------
void settings_bright(void)
{
  uint8_t cur_mode = 0; //текущий режим
  boolean blink_data = 0; //мигание сигментами
  uint16_t result = 0; //результат опроса сенсора освещенности

  dot_state = 0; //выключаем точку
  indiClr(); //очищаем индикаторы
  indiPrint("BRI", 0);
  for (timer_millis = TIME_MSG; timer_millis && !check_keys();) data_convert(); // ждем, преобразование данных
  if (mainSettings.bright_mode == 1) indiSetBright(brightDefault[mainSettings.indiBright[changeBright()]]); //установка яркости индикаторов

  //настройки
  while (1) {
    data_convert(); //преобразование данных

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
          indiPrint("AS", 0); //анимация
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
            case 2:
              indiPrint("D", 0); //день
              result = readLightSens(); //считываем сенсор
              if (result > mainSettings.adcMaxAuto) mainSettings.adcMaxAuto = result; //назодим максимум
              indiPrintNum(mainSettings.adcMaxAuto, 1, 3, ' '); //вывод порога день
              break;
            case 1:
              indiPrint("L", 0); //уровень ручной подсветки
              if (!blink_data) indiPrintNum(mainSettings.indiBright[0] + 1, 3); //вывод яркости ночь
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
      case 1: //left click
        switch (cur_mode) {
          case 0: //настройка колбы
            if (mainSettings.flask_mode > 0) mainSettings.flask_mode--; else mainSettings.flask_mode = 1 + USE_LIGHT_SENS;
            flask_state = mainSettings.flask_mode; //обновление стотояния колбы
            break;
          case 1: //настройка сна
            if (mainSettings.sleep_time > 3) mainSettings.sleep_time--; else mainSettings.sleep_time = 0;
            break;
          case 2: //настройка анимации
            if (mainSettings.anim_mode > 0) mainSettings.anim_mode--; else mainSettings.anim_mode = sizeof(animTime);
            animFlip(); //анимция перелистывания
            break;
          case 3: //настройка анимации сна
            mainSettings.sleep_anim = !mainSettings.sleep_anim;
            break;
          case 4: //настройка подсветки
            if (mainSettings.bright_mode > 0) mainSettings.bright_mode--; else mainSettings.bright_mode = 1 + USE_LIGHT_SENS;
            switch (mainSettings.bright_mode) {
              case 0: indiSetBright(brightDefault[mainSettings.bright_levle]); break; //установка яркости индикаторов
              case 1: indiSetBright(brightDefault[mainSettings.indiBright[changeBright()]]); break; //установка яркости индикаторов
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

      case 2: //right click
        switch (cur_mode) {
          case 0: //настройка колбы
            if (mainSettings.flask_mode < 1 + USE_LIGHT_SENS) mainSettings.flask_mode++; else mainSettings.flask_mode = 0;
            flask_state = mainSettings.flask_mode; //обновление стотояния колбы
            break;
          case 1: //настройка сна
            if (!mainSettings.sleep_time) mainSettings.sleep_time = 3; else if (mainSettings.sleep_time < 15) mainSettings.sleep_time++; else mainSettings.sleep_time = 3;
            break;
          case 2: //настройка анимации
            if (mainSettings.anim_mode < sizeof(animTime)) mainSettings.anim_mode++; else mainSettings.anim_mode = 0;
            animFlip(); //анимция перелистывания
            break;
          case 3: //настройка анимации сна
            mainSettings.sleep_anim = !mainSettings.sleep_anim;
            break;
          case 4: //настройка подсветки
            if (mainSettings.bright_mode < 1 + USE_LIGHT_SENS) mainSettings.bright_mode++; else mainSettings.bright_mode = 0;
            switch (mainSettings.bright_mode) {
              case 0: indiSetBright(brightDefault[mainSettings.bright_levle]); break; //установка яркости индикаторов
              case 1: indiSetBright(brightDefault[mainSettings.indiBright[changeBright()]]); break; //установка яркости индикаторов
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

      case 3: //left hold
        if (cur_mode < allModes[mainSettings.bright_mode]) cur_mode++; else cur_mode = 0;
        switch (cur_mode) {
          case 0:
            indiClr(); //очистка индикаторов
            indiPrint("FLS", 0); //колба
            for (timer_millis = TIME_MSG_PNT; timer_millis && !check_keys();) data_convert(); // ждем, преобразование данных
            if (mainSettings.bright_mode == 1) indiSetBright(brightDefault[mainSettings.indiBright[changeBright()]]); //установка яркости индикаторов
            _bright_block = 0; //разрешаем управление подсветкой
            break;

          case 1:
            indiClr(); //очистка индикаторов
            indiPrint("SLP", 0); //сон
            for (timer_millis = TIME_MSG_PNT; timer_millis && !check_keys();) data_convert(); // ждем, преобразование данных
            break;

          case 2:
            indiClr(); //очистка индикаторов
            indiPrint("ANI", 0); //анимация
            for (timer_millis = TIME_MSG_PNT; timer_millis && !check_keys();) data_convert(); // ждем, преобразование данных
            break;

          case 3:
            indiClr(); //очистка индикаторов
            indiPrint("ANS", 0); //анимация
            for (timer_millis = TIME_MSG_PNT; timer_millis && !check_keys();) data_convert(); // ждем, преобразование данных
            break;

          case 4:
            indiClr(); //очистка индикаторов
            indiPrint("BRI", 0); //подсветка
            for (timer_millis = TIME_MSG_PNT; timer_millis && !check_keys();) data_convert(); // ждем, преобразование данных
            break;

          case 5:
            indiClr(); //очистка индикаторов
            if (mainSettings.bright_mode) indiPrint("NHT", 0); //ночь
            else indiPrint("L-T", 0); //уровень подсветки
            for (timer_millis = TIME_MSG_PNT; timer_millis && !check_keys();) data_convert(); // ждем, преобразование данных
            break;

          case 6:
            if (mainSettings.bright_mode == 2) {
              indiClr(); //очистка индикаторов
              indiPrint("DAY", 0); //день
              for (timer_millis = TIME_MSG_PNT; timer_millis && !check_keys();) data_convert(); // ждем, преобразование данных
            }
            else {
              indiSetBright(brightDefault[mainSettings.indiBright[0]]);
              _bright_block = 1; //запрещаем управление подсветкой
            }
            break;

          case 7:
            indiClr(); //очистка индикаторов
            indiPrint("DAY", 0); //день
            for (timer_millis = TIME_MSG_PNT; timer_millis && !check_keys();) data_convert(); // ждем, преобразование данных
            indiSetBright(brightDefault[changeBright()]); //установка яркости индикаторов
            _bright_block = 0; //разрешаем управление подсветкой
            break;

          case 8:
            indiSetBright(brightDefault[mainSettings.indiBright[1]]);
            _bright_block = 1; //запрещаем управление подсветкой
            break;
        }
        blink_data = 0; //сбрасываем флаг мигания
        break;

      case 4: //right hold
        eeprom_update_block((void*)&mainSettings, (void*)EEPROM_BLOCK_SETTINGS_MAIN, sizeof(mainSettings)); //записываем основные настройки в память

        switch (mainSettings.bright_mode) {
          case 0: indiSetBright(brightDefault[mainSettings.bright_levle]); break; //установка яркости индикаторов
          case 1: indiSetBright(brightDefault[mainSettings.indiBright[changeBright()]]); break; //установка яркости индикаторов
          case 2: indiSetBright(brightDefault[lightSens()]); break; //установка яркости индикаторов
        }

        dot_state = 0; //выключаем точку
        indiClr(); //очистка индикаторов
        indiPrint("OUT", 0);
        for (timer_millis = TIME_MSG; timer_millis && !check_keys();) data_convert(); // ждем, преобразование данных
        if (timerSettings.timer_mode || !_timer_start || _timer_secs > timerSettings.timer_blink) _disableSleep = 0; //разрешаем сон
        if (_mode != 3) _mode = 0; //переходим в режим часов
        _bright_block = 0; //разрешаем управление подсветкой
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
  for (timer_millis = TIME_MSG; timer_millis && !check_keys();) data_convert(); // ждем, преобразование данных

  //настройки
  while (1) {
    data_convert(); //преобразование данных

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
      case 1: //left click
        switch (cur_mode) {
          case 0: timerSettings.timer_mode = 0; _timer_start = 0; _timer_secs = timerDefault[timerSettings.timer_preset] * 60; break;
          case 1: if (timerSettings.timer_preset > 0) timerSettings.timer_preset--; else timerSettings.timer_preset = 9; break;
          case 2: if (timerSettings.timer_blink > 5) timerSettings.timer_blink--; else timerSettings.timer_blink = 60; break;
        }
        blink_data = 0; //сбрасываем флаг мигания
        break;

      case 2: //right click
        switch (cur_mode) {
          case 0: timerSettings.timer_mode = 1; _timer_start = 0; _timer_secs = 0; break;
          case 1: if (timerSettings.timer_preset < 9) timerSettings.timer_preset++; else timerSettings.timer_preset = 0; break;
          case 2: if (timerSettings.timer_blink < 60) timerSettings.timer_blink++; else timerSettings.timer_blink = 5; break;
        }
        blink_data = 0; //сбрасываем флаг мигания
        break;

      case 3: //left hold
        if (cur_mode < 2) cur_mode++; else cur_mode = 0;
        switch (cur_mode) {
          case 0:
            indiClr(); //очистка индикаторов
            indiPrint("PAR", 0);
            for (timer_millis = TIME_MSG_PNT; timer_millis && !check_keys();) data_convert(); // ждем, преобразование данных
            break;

          case 1:
            indiClr(); //очистка индикаторов
            indiPrint("TPR", 0);
            for (timer_millis = TIME_MSG_PNT; timer_millis && !check_keys();) data_convert(); // ждем, преобразование данных
            break;

          case 2:
            indiClr(); //очистка индикаторов
            indiPrint("BLK", 0);
            for (timer_millis = TIME_MSG_PNT; timer_millis && !check_keys();) data_convert(); // ждем, преобразование данных
            break;
        }
        blink_data = 0; //сбрасываем флаг мигания
        break;

      case 4: //right hold
        eeprom_update_block((void*)&timerSettings, (void*)EEPROM_BLOCK_SETTINGS_TIMER, sizeof(timerSettings)); //записываем настройки таймера в память
        indiClr(); //очистка индикаторов
        indiPrint("OUT", 0);
        for (timer_millis = TIME_MSG; timer_millis && !check_keys();) data_convert(); // ждем, преобразование данных
        if (timerSettings.timer_mode || !_timer_start || _timer_secs > timerSettings.timer_blink) _disableSleep = 0; //разрешаем сон
        return;
    }
  }
}
//----------------------------------------------------------------------------------
boolean changeBright(void) { // установка яркости от времени суток
  if ((mainSettings.timeBright[0] > mainSettings.timeBright[1] && (time[4] >= mainSettings.timeBright[0] || time[4] < mainSettings.timeBright[1])) ||
      (mainSettings.timeBright[0] < mainSettings.timeBright[1] && time[4] >= mainSettings.timeBright[0] && time[4] < mainSettings.timeBright[1])) {
    return 0;
  } else {
    return 1;
  }
}
//----------------------------------------------------------------------------------
void dotFlash(void) {
  if (!timer_dot) {
    dot_state = !dot_state; //инвертируем точки
    timer_dot = DOT_TIME;
  }
}
//-----------------------------Главный экран------------------------------------------------
void main_screen(void) //главный экран
{
  switch (_msg_type) {
    case 1: timerMessage(); break; //оповещения таймера
    case 2: lowBatMessage(); break; //оповещения батареи
    case 3: pwrDownMessage(); break; //оповещения выключения питания
  }

  if (_animStart) {
    animFlip(); //анимция перелистывания
    _timer_sleep = 0; //сбрасываем таймер сна
  }

  if (!_scr) {
    _scr = 1; //сбрасываем флаг
    switch (_mode) {
      case 0:
        indiPrintNum(time[4], 0, 2, '0'); //вывод часов
        indiPrintNum(time[5], 2, 2, '0'); //вывод минут
        break;
      case 1:
        indiPrint("B", 0);
        indiPrintNum(bat, 1, 3, ' '); //вывод заряда акб
        dot_state = 0; //выключаем точки
        break;
      case 2:
        indiPrintNum(time[2], 0, 2, '0'); //вывод даты
        indiPrintNum(time[1], 2, 2, '0'); //вывод месяца
        dot_state = 1; //включаем точки
        break;
    }
  }

  if (!_sleep) { //если не спим
    switch (_mode) {
      case 0: //режим часов
        dotFlash(); //мигаем точками
        break;

      case 3: //режим таймера
        if (!timer_dot) {
          if (timerSettings.timer_mode || !_timer_start || _timer_secs >= timerSettings.timer_blink || !dot_state) { //если таймер не запущен или время больше утановленного или точки не горят
            indiPrintNum(_timer_secs / 60, 0, 2, '0'); //вывод минут
            indiPrintNum(_timer_secs % 60, 2, 2, '0'); //вывод секунд
          }
          if (_timer_start) dot_state = !dot_state; //инвертируем точки
          else dot_state = 1; //включаем точки
          timer_dot = DOT_TIME; //установка таймера
        }
        break;
    }
  }

  switch (check_keys()) {
    case 1: //left key press
      if (_mode != 3) _mode = 3; //если не в режиме таймера, переходим в режим таймера
      else { //иначе
        _timer_start = !_timer_start; //запуск - остановка таймера
        switch (timerSettings.timer_mode) {
          case 0: //режим таймера
            if (_timer_start && _timer_secs <= timerSettings.timer_blink) _disableSleep = 1; //запрещаем сон
            else if (_disableSleep) _disableSleep = 0; //разрешаем сон
            break;
        }
      }
      break;

    case 2: //right key press
      switch (timerSettings.timer_mode) {
        case 0: //режим таймера
          if (!_timer_start || _timer_secs > timerSettings.timer_blink) { //если таймер выключен или время таймера больше установлнного
            if (_mode < 2) _mode++; else _mode = 0; //переключаем режимы времени
          }
          else {
            _timer_secs = timerDefault[timerSettings.timer_preset] * 60; //иначе перезапускаем таймер
            if (_disableSleep) _disableSleep = 0; //разрешаем сон
          }
          break;
        case 1: //режим секундомера
          if (_mode < 2) _mode++; else _mode = 0; //переключаем режимы времени
          break;
      }
      break;

    case 3: //left key hold
      if (_mode != 3) settings_time(); //настройки времени
      else { //сброс таймера
        _timer_start = 0; //выключаем таймер
        switch (timerSettings.timer_mode) {
          case 0: //режим таймера
            _timer_secs = timerDefault[timerSettings.timer_preset] * 60;
            if (_disableSleep) _disableSleep = 0; //разрешаем сон
            break;
          case 1: //режим секундомера
            _timer_secs = 0; //сброс секундомера
            break;
        }
      }
      break;

    case 4: //right key hold
      if (_mode != 3) settings_bright(); //настройки яркости
      else set_timer(); //настройка таймера
      break;
  }
}
