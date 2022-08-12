//Основные настройки
#define DEFAULT_MODE_TIMER 0   //режим работы таймера по умолчанию(0 - таймер | 1 - секундомер)
#define DEFAULT_PRESET_TIMER 0 //текущий номер выбранного пресета таймера по умолчанию(0..9)
#define DEFAULT_BLINK_TIMER 30 //порог времени мигания таймером по умолчанию(5..60)(с)

#define DEFAULT_SLEEP_TIME 5   //время ухода в сон по умолчанию(3..15)(с)
#define DEFAULT_BRIGHT_MODE 0  //тип подсветки по умолчанию(0 - ручная | 1 - от датчика | 2 - от времени)
#define DEFAULT_BRIGHT 1       //яркость ручной подсветки по умолчанию(0..4)(1..5 в меню)
#define DEFAULT_FLASK_MODE 1   //тип свечения колбы по умолчанию(0 - выкл | 1 - вкл | 2 - от датчика)
#define DEFAULT_ANIM_MODE 1    //тип анимации времени по умолчанию(0 - выкл | 1 - перемотка анодов по кругу | 2 - перемотка анодов сверху | 3 - поезд | 4 - резинка | 5 - барабан | 6 - падение)
#define DEFAULT_SLEEP_ANIM 1   //анимация ухода в сон по умолчанию(0 - выкл | 1 - вкл)

#define DEFAULT_BRIGHT_DAY 4   //яркость дневной подсветки по умолчанию(0..4)
#define DEFAULT_BRIGHT_NIGHT 0 //яркость ночной подсветки по умолчанию(0..4)

#define DEFAULT_TIME_DAY 8     //время дневной подсветки по умолчанию(0..23)(ч)
#define DEFAULT_TIME_NIGHT 23  //время ночной подсветки по умолчанию(0..23)(ч)

#define BAT_MIN_V 300          //минимальное напряжение акб(250..350)(в*100)
#define BAT_MAX_V 420          //максимальное напряжение акб(400..450)(в*100)
#define LOW_BAT_P 2            //минимальное количество процентов для отключения питания(0..5)(%)
#define PWR_BAT_P 5            //минимальное количество процентов для включения питания(0..15)(%)
#define MSG_BAT_P 10           //минимальное количество процентов для включения оповещения об разряженной батарее(0..15)(%)

#define TIME_MSG 1000          //время отображения сообщений входа/выхода из настроек(1000..10000)(мс) 
#define TIME_MSG_PNT 500       //время отображения сообщений подпунктов меню(1000..10000)(мс) 
#define TIME_MSG_BAT 3000      //время отображения сообщения оповещения об разряженной батарее(1000..10000)(мс) 
#define TIME_MSG_TMR_OVF 5000  //время отображения сообщения оповещения об окончании таймера(1000..10000)(мс) 
#define TIME_PWR_DOWN 2000     //время отображения сообщения оповещения об отключении питания(1000..10000)(мс)
#define TIME_PWR_ON 2000       //время ожидания до включения питания(1000..10000)(мс)

#define DOT_TIME 500           //период мигания точек(200..800)(мс)
#define TIMER_DOT_TIME 500     //период мигания точек в режиме таймера/секундомера(200..800)(мс)
#define SLEEP_ANIM_TIME 150    //период анимации выключения дисплея(50..300)(мс)

#define BAT_TIME 120           //время опроса заряда акб(60..240)(с)

const uint8_t animTime[] = {120, 120, 240, 120, 20, 120}; //период цикла анимации 4(10..1000)(мс)
const uint8_t timerDefault[] = {1, 2, 3, 4, 5, 7, 10, 15, 30, 60}; //пресеты таймера(1..99)(м)(не более 10 пресетов)
const uint8_t brightDefault[] = {20, 40, 60, 90, 125}; //пресеты яркости(1..127)(не более 5 пресетов)

//Технические настройки
#define USE_LIGHT_SENS 1           //использовать сенсор освещённости(1 - сенсор используется | 0 - сенсор не используется)
#define DEFAULT_LIGHT_SENS_ADC 170 //значение ацп сенсора по умолчанию(0..255)

#define BTN_GIST_TICK 50           //количество циклов для защиты от дребезга(0..200)(мс)
#define BTN_HOLD_TICK 500          //количество циклов после которого считается что кнопка зажата(0..1000)(мс)

#define EEPROM_BLOCK_NULL 0        //начальный блок памяти(0..512)

const uint8_t allModes[] = {5, 8, 6}; //всего режимов настроек подсветки
const uint8_t daysInMonth[] PROGMEM = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}; //дней в месяце

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