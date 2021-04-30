#include "wire.h"

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define DS1307_ADDRESS 0x68

void TimeGetDate(uint8_t *values);
void TimeSetDate(const uint8_t *values);

uint8_t fromDecimalToBCD(uint8_t decimalValue)
{
  return ((decimalValue / 10) * 16) + (decimalValue % 10);
}

uint8_t fromBCDToDecimal(uint8_t BCDValue)
{
  return ((BCDValue / 16) * 10) + (BCDValue % 16);
}

void TimeSetDate(const uint8_t *values) // year, month, dayOfMonth, dayOfWeek, hour, minute, second
{
  RTC_ON;
  RTC_BAT_OFF;

  WireBeginTransmission(DS1307_ADDRESS);
  WireWrite(0x00);

  WireWrite(fromDecimalToBCD(values[6])); //отправляем время
  WireWrite(fromDecimalToBCD(values[5]));
  WireWrite(fromDecimalToBCD(values[4]));
  WireWrite(fromDecimalToBCD(values[3]));
  WireWrite(fromDecimalToBCD(values[2]));
  WireWrite(fromDecimalToBCD(values[1]));
  WireWrite(fromDecimalToBCD(values[0]));

  WireWrite(0x00);
  WireEnd(); //конец передачи

  RTC_BAT_ON;
  RTC_OFF;
}

void TimeGetDate(uint8_t *values) // year, month, dayOfMonth, dayOfWeek, hour, minute, second
{
  RTC_ON;
  RTC_BAT_OFF;

  if (WireRequestFrom(DS1307_ADDRESS, 0x00)) return; //запрашиваем чтение данных, если нет ответа выходим

  values[6] = WireRead();
  values[5] = WireRead();
  values[4] = WireRead();
  values[3] = WireRead();
  values[2] = WireRead();
  values[1] = WireRead();
  values[0] = WireReadEndByte();

  RTC_BAT_ON;
  RTC_OFF;
}
