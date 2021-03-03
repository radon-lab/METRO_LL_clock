#define BTN_GIST_TICK    2   //количество циклов для защиты от дребезга(0..255)(1 цикл -> +-17.5мс)
#define BTN_HOLD_TICK    30  //количество циклов после которого считается что кнопка зажата(0..255)(1 цикл -> +-17.5мс)

#define BAT_MIN_V 300 //минимальное напряжение акб(250..350)(v*100)
#define BAT_MAX_V 420 //максимальное напряжение акб(400..450)(v*100)
#define LOW_BAT_P 2   //минимальное количество процентов для отключения питания(0..5)(%)
#define PWR_BAT_P 5   //минимальное количество процентов для включения питания(0..15)(%)
#define MSG_BAT_P 10  //минимальное количество процентов для включения оповещения об разряженной батарее(0..15)(%)

#define TIME_MSG         1000 //время отображения сообщений входа/выхода из настроек(1000..10000)(ms) 
#define TIME_MSG_PNT     500  //время отображения сообщений подпунктов меню(1000..10000)(ms) 
#define TIME_MSG_BAT     3000 //время отображения сообщения оповещения об разряженной батарее(1000..10000)(ms) 
#define TIME_MSG_TMR_OVF 5000 //время отображения сообщения оповещения об окончании таймера(1000..10000)(ms) 
#define TIME_PWR_DOWN    2000 //время отображения сообщения оповещения об отключении питания(1000..10000)(ms)
#define TIME_PWR_ON      2000 //время ожидания до включения питания(1000..10000)(ms)

#define DOT_TIME 500 //период мигания точек

#define ANIM_1_TIME 119 //период цикла анимации 1(17..986)(1шаг - 17)(ms)
#define ANIM_2_TIME 119 //период цикла анимации 2(17..986)(1шаг - 17)(ms)
#define ANIM_3_TIME 238 //период цикла анимации 3(17..986)(1шаг - 17)(ms)
#define ANIM_4_TIME 119 //период цикла анимации 4(17..986)(1шаг - 17)(ms)

#define BAT_TIME 30 //время опроса заряда акб(5..120(с)

#define DEFAUL_LIGHT_SENS_ADC 170 //значение ацп сенсора по умолчанию(0..255)
#define DEFAUL_BLINK_TIMER    30 //порог времени мигания таймером по умолчанию(5..60)(с)
#define DEFAUL_SLEEP_TIME     5  //время ухода в сон по умолчанию(3..15)(с)
#define DEFAUL_BRIGHT_MODE    0  //тип подсветки по умолчанию(0 - ручная | 1 - от датчика | 2 - от времени)
#define DEFAUL_BRIGHT         1  //яркость ручной подсветки по умолчанию(0..4)(1..5 в меню)
#define DEFAUL_FLASK_MODE     1  //тип свечения колбы по умолчанию(0 - выкл | 1 - вкл | 2 - от датчика)
#define DEFAUL_ANIM_MODE      1  //тип анимации времени по умолчанию(0 - выкл | 1 - перемотка анодов по кругу | 2 - перемотка анодов сверху | 3 - поезд | 4 - резинка)

uint8_t timeBright[] = { 23, 8 }; //время подсветки по умолчанию(ночь, день)(0..23)(ч)
uint8_t indiBright[] = { 0, 4 }; //яркость подсветки по умолчанию(ночь, день)(0..4)(1..5 в меню)

const uint8_t timerDefault [] = { 1, 2, 3, 4, 5, 7, 10, 15, 30, 60 }; //пресеты таймера(1..99)(м)(не более 10 пресетов)
const uint8_t brightDefault [] = { 20, 40, 60, 90, 125 }; //пресеты яркости(1..127)(не более 5 пресетов)

const uint8_t allModes[] = { 4, 5, 7 }; //всего режимов настроек подсветки
const uint8_t timeDefault [] = { 21, 1, 1, 5, 8, 0, 0 }; //время по умолчанию при потере питания
const uint8_t daysInMonth [] PROGMEM = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }; //дней в месяце
