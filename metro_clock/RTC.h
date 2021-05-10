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

  WireBeginTransmission(DS1307_ADDRESS); //начало передачи
  WireWrite(0x00); //устанавливаем адрес записи

  WireWrite(fromDecimalToBCD(values[6])); //отправляем время
  WireWrite(fromDecimalToBCD(values[5]));
  WireWrite(fromDecimalToBCD(values[4]));
  WireWrite(fromDecimalToBCD(values[3]));
  WireWrite(fromDecimalToBCD(values[2]));
  WireWrite(fromDecimalToBCD(values[1]));
  WireWrite(fromDecimalToBCD(values[0]));

  WireWrite(0x00);
  WireEnd(); //конец передачи

  RTC_BAT_ON; //включаем питание батареи
  RTC_OFF; //выключаем питание РТС
}

void TimeGetDate(uint8_t *values) // year, month, dayOfMonth, dayOfWeek, hour, minute, second
{
  RTC_ON; //включаем питание РТС
  RTC_BAT_OFF; //выключаем питание батареи

  if (WireRequestFrom(DS1307_ADDRESS, 0x00)) return; //запрашиваем чтение данных, если нет ответа выходим

  values[6] = fromBCDToDecimal(WireRead()); //получаем время
  values[5] = fromBCDToDecimal(WireRead());
  values[4] = fromBCDToDecimal(WireRead());
  values[3] = fromBCDToDecimal(WireRead());
  values[2] = fromBCDToDecimal(WireRead());
  values[1] = fromBCDToDecimal(WireRead());
  values[0] = fromBCDToDecimal(WireReadEndByte());

  RTC_BAT_ON; //включаем питание батареи
  RTC_OFF; //выключаем питание РТС
}
