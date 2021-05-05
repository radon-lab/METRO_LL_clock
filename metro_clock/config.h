#define USE_LIGHT_SENS 1 //использовать сенсор освещённости(1 - да | 0 - нет)

#define DEFAULT_LIGHT_SENS_ADC 170 //значение ацп сенсора по умолчанию(0..255)
#define DEFAULT_BLINK_TIMER    30 //порог времени мигания таймером по умолчанию(5..60)(с)
#define DEFAULT_SLEEP_TIME     5  //время ухода в сон по умолчанию(3..15)(с)
#define DEFAULT_BRIGHT_MODE    0  //тип подсветки по умолчанию(0 - ручная | 1 - от датчика | 2 - от времени)
#define DEFAULT_BRIGHT         1  //яркость ручной подсветки по умолчанию(0..4)(1..5 в меню)
#define DEFAULT_FLASK_MODE     1  //тип свечения колбы по умолчанию(0 - выкл | 1 - вкл | 2 - от датчика)
#define DEFAULT_ANIM_MODE      1  //тип анимации времени по умолчанию(0 - выкл | 1 - перемотка анодов по кругу | 2 - перемотка анодов сверху | 3 - поезд | 4 - резинка | 5 - барабан | 6 - падение)

#define BAT_MIN_V        300  //минимальное напряжение акб(250..350)(v*100)
#define BAT_MAX_V        420  //максимальное напряжение акб(400..450)(v*100)
#define LOW_BAT_P        2    //минимальное количество процентов для отключения питания(0..5)(%)
#define PWR_BAT_P        5    //минимальное количество процентов для включения питания(0..15)(%)
#define MSG_BAT_P        10   //минимальное количество процентов для включения оповещения об разряженной батарее(0..15)(%)

#define TIME_MSG         1000 //время отображения сообщений входа/выхода из настроек(1000..10000)(ms) 
#define TIME_MSG_PNT     500  //время отображения сообщений подпунктов меню(1000..10000)(ms) 
#define TIME_MSG_BAT     3000 //время отображения сообщения оповещения об разряженной батарее(1000..10000)(ms) 
#define TIME_MSG_TMR_OVF 5000 //время отображения сообщения оповещения об окончании таймера(1000..10000)(ms) 
#define TIME_PWR_DOWN    2000 //время отображения сообщения оповещения об отключении питания(1000..10000)(ms)
#define TIME_PWR_ON      2000 //время ожидания до включения питания(1000..10000)(ms)

#define DOT_TIME         500  //период мигания точек

#define BAT_TIME         30   //время опроса заряда акб(5..120(с)

#define BTN_GIST_TICK    2    //количество циклов для защиты от дребезга(0..255)(1 цикл -> +-17.5мс)
#define BTN_HOLD_TICK    30   //количество циклов после которого считается что кнопка зажата(0..255)(1 цикл -> +-17.5мс)

uint8_t timeBright[] = { 23, 8 }; //время подсветки по умолчанию(ночь, день)(0..23)(ч)
uint8_t indiBright[] = { 0, 4 }; //яркость подсветки по умолчанию(ночь, день)(0..4)(1..5 в меню)

const uint8_t animTime [] = { 119, 119, 238, 119, 17, 119 }; //период цикла анимации 4(17..986)(1шаг - 17)(ms)
const uint8_t timerDefault [] = { 1, 2, 3, 4, 5, 7, 10, 15, 30, 60 }; //пресеты таймера(1..99)(м)(не более 10 пресетов)
const uint8_t brightDefault [] = { 20, 40, 60, 90, 125 }; //пресеты яркости(1..127)(не более 5 пресетов)

const uint8_t allModes[] = { 4, 7, 5 }; //всего режимов настроек подсветки
const uint8_t timeDefault [] = { 21, 1, 1, 5, 8, 0, 0 }; //время по умолчанию при потере питания(год, месяц, день, день_недели, часы, минуты, секунды)
const uint8_t daysInMonth [] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }; //дней в месяце

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
