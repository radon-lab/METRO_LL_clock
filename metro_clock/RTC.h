#define DS1307_ADDRESS 0x68

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
  RTC_ON; //включаем питание РТС
  RTC_BAT_OFF; //выключаем питание батареи

  _delay_us(100);

  wireBeginTransmission(DS1307_ADDRESS); //начало передачи
  wireWrite(0x00); //устанавливаем адрес записи

  wireWrite(fromDecimalToBCD(values[6])); //отправляем время
  wireWrite(fromDecimalToBCD(values[5]));
  wireWrite(fromDecimalToBCD(values[4]));
  wireWrite(fromDecimalToBCD(values[3]));
  wireWrite(fromDecimalToBCD(values[2]));
  wireWrite(fromDecimalToBCD(values[1]));
  wireWrite(fromDecimalToBCD(values[0]));

  wireWrite(0x00);
  wireEnd(); //конец передачи

  RTC_BAT_ON; //включаем питание батареи
  RTC_OFF; //выключаем питание РТС
}

void TimeGetDate(uint8_t *values) // year, month, dayOfMonth, dayOfWeek, hour, minute, second
{
  RTC_ON; //включаем питание РТС
  RTC_BAT_OFF; //выключаем питание батареи

  _delay_us(100);

  if (wireRequestFrom(DS1307_ADDRESS, 0x00)) return; //запрашиваем чтение данных, если нет ответа выходим

  values[6] = fromBCDToDecimal(wireRead()); //получаем время
  values[5] = fromBCDToDecimal(wireRead());
  values[4] = fromBCDToDecimal(wireRead());
  values[3] = fromBCDToDecimal(wireRead());
  values[2] = fromBCDToDecimal(wireRead());
  values[1] = fromBCDToDecimal(wireRead());
  values[0] = fromBCDToDecimal(wireReadEndByte());

  RTC_BAT_ON; //включаем питание батареи
  RTC_OFF; //выключаем питание РТС
}
