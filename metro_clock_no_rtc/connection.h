//Соединения периферии с пинами МК
//    (0 - D0 | 1 - D1 | 2 - D2 | 3 - D3 | 4 - D4 | 5 - D5 | 6 - D6 | 7 - D7)
//        (8 - D8 | 9 - D9 | 10 - D10 | 11 - D11 | 12 - D12 | 13 - D13)
//         (14 - A0 | 15 - A1 | 16 - A2 | 17 - A3 | 18 - A4 | 19 - A5)

#define INDI_1_PIN 16 //пин индикатора 1(0..19)(pin D)
#define INDI_2_PIN 13 //пин индикатора 2(0..19)(pin D)
#define INDI_3_PIN 12 //пин индикатора 3(0..19)(pin D)
#define INDI_4_PIN 11 //пин индикатора 4(0..19)(pin D)
 
#define SEG_A_PIN 14  //пин сегмента A(0..19)(pin D)
#define SEG_B_PIN 8   //пин сегмента B(0..19)(pin D)
#define SEG_C_PIN 7   //пин сегмента C(0..19)(pin D)
#define SEG_D_PIN 6   //пин сегмента D(0..19)(pin D)
#define SEG_E_PIN 4   //пин сегмента E(0..19)(pin D)
#define SEG_F_PIN 10  //пин сегмента F(0..19)(pin D)
#define SEG_G_PIN 3   //пин сегмента G(0..19)(pin D)

#define RIGHT_PIN 2   //пин правой кнопки(2)(pin D)
#define LEFT_PIN 0    //пин левой кнопки(0..7)(pin D)
#define DOT_PIN 5     //пин секундных точек(0..19)(pin D)
#define DOT_PM_PIN 17 //пин индикатора PM(0..19)(pin D)
#define FLASK_PIN 9   //пин колбы(0..19)(pin D)
#define SENS_PIN 15   //пин сенсора освещения(0..19)(pin D)

#define SENS_ANALOG_PIN 6 //аналоговый пин сенсора освещения(0..7)(pin A)

//Соединения периферии с портами МК

#define DDR_REG(portx)  (*(&portx - 1))
#define PIN_REG(portx)  (*(&portx - 2))
#define BIT_READ(value, bit) (((value) >> (bit)) & 0x01)
#define BIT_SET(value, bit) ((value) |= (0x01 << (bit)))
#define BIT_CLEAR(value, bit) ((value) &= ~(0x01 << (bit)))
#define BIT_WRITE(value, bit, bitvalue) (bitvalue ? BIT_SET(value, bit) : BIT_CLEAR(value, bit))

#define DECODE_PCMSK(pin) ((pin < 8) ? PCMSK2 : ((pin < 14) ? PCMSK0 : PCMSK1))
#define DECODE_PCIE(pin) ((pin < 8) ? PCIE2 : ((pin < 14) ? PCIE0 : PCIE1))
#define DECODE_PORT(pin) ((pin < 8) ? PORTD : ((pin < 14) ? PORTB : PORTC))
#define DECODE_BIT(pin) ((pin < 8) ? pin : ((pin < 14) ? (pin - 8) : (pin - 14)))

#define INDI_1_PORT DECODE_PORT(INDI_1_PIN)
#define INDI_1_BIT  DECODE_BIT(INDI_1_PIN)
#define INDI_2_PORT DECODE_PORT(INDI_2_PIN)
#define INDI_2_BIT  DECODE_BIT(INDI_2_PIN)
#define INDI_3_PORT DECODE_PORT(INDI_3_PIN)
#define INDI_3_BIT  DECODE_BIT(INDI_3_PIN)
#define INDI_4_PORT DECODE_PORT(INDI_4_PIN)
#define INDI_4_BIT  DECODE_BIT(INDI_4_PIN)

#define SEG_A_PORT DECODE_PORT(SEG_A_PIN)
#define SEG_A_BIT  DECODE_BIT(SEG_A_PIN)
#define SEG_B_PORT DECODE_PORT(SEG_B_PIN)
#define SEG_B_BIT  DECODE_BIT(SEG_B_PIN)
#define SEG_C_PORT DECODE_PORT(SEG_C_PIN)
#define SEG_C_BIT  DECODE_BIT(SEG_C_PIN)
#define SEG_D_PORT DECODE_PORT(SEG_D_PIN)
#define SEG_D_BIT  DECODE_BIT(SEG_D_PIN)
#define SEG_E_PORT DECODE_PORT(SEG_E_PIN)
#define SEG_E_BIT  DECODE_BIT(SEG_E_PIN)
#define SEG_F_PORT DECODE_PORT(SEG_F_PIN)
#define SEG_F_BIT  DECODE_BIT(SEG_F_PIN)
#define SEG_G_PORT DECODE_PORT(SEG_G_PIN)
#define SEG_G_BIT  DECODE_BIT(SEG_G_PIN)

//пин кнопки RIGHT
#define RIGHT_BIT   DECODE_BIT(RIGHT_PIN)
#define RIGHT_PORT  DECODE_PORT(RIGHT_PIN)

#define RIGHT_SET   (BIT_SET(RIGHT_PORT, RIGHT_BIT))
#define RIGHT_CHK   (BIT_READ(PIN_REG(RIGHT_PORT), RIGHT_BIT))
#define RIGHT_INP   (BIT_CLEAR((DDR_REG(RIGHT_PORT)), RIGHT_BIT))

#define RIGHT_INIT  RIGHT_SET; RIGHT_INP

//пин кнопки LEFT
#define LEFT_BIT   DECODE_BIT(LEFT_PIN)
#define LEFT_PORT  DECODE_PORT(LEFT_PIN)

#define LEFT_SET   (BIT_SET(LEFT_PORT, LEFT_BIT))
#define LEFT_CHK   (BIT_READ(PIN_REG(LEFT_PORT), LEFT_BIT))
#define LEFT_INP   (BIT_CLEAR((DDR_REG(LEFT_PORT)), LEFT_BIT))

#define LEFT_INIT  LEFT_SET; LEFT_INP

//пин точек
#define DOT_BIT   DECODE_BIT(DOT_PIN)
#define DOT_PORT  DECODE_PORT(DOT_PIN)

#define DOT_INV   (DOT_PORT ^= (1 << DOT_BIT))
#define DOT_ON    (BIT_SET(DOT_PORT, DOT_BIT))
#define DOT_OFF   (BIT_CLEAR(DOT_PORT, DOT_BIT))
#define DOT_OUT   (BIT_SET((DDR_REG(DOT_PORT)), DOT_BIT))

#define DOT_INIT  DOT_OFF; DOT_OUT

//пин точек
#define DOT_PM_BIT   DECODE_BIT(DOT_PM_PIN)
#define DOT_PM_PORT  DECODE_PORT(DOT_PM_PIN)

#define DOT_PM_ON    (BIT_SET(DOT_PM_PORT, DOT_PM_BIT))
#define DOT_PM_OFF   (BIT_CLEAR(DOT_PM_PORT, DOT_PM_BIT))
#define DOT_PM_OUT   (BIT_SET((DDR_REG(DOT_PM_PORT)), DOT_PM_BIT))

#define DOT_PM_INIT     DOT_PM_OFF; DOT_PM_OUT

//пин колбы
#define FLASK_BIT     DECODE_BIT(FLASK_PIN)
#define FLASK_PORT    DECODE_PORT(FLASK_PIN)

#define FLASK_ON      (BIT_SET(FLASK_PORT, FLASK_BIT))
#define FLASK_OFF     (BIT_CLEAR(FLASK_PORT, FLASK_BIT))
#define FLASK_OUT     (BIT_SET((DDR_REG(FLASK_PORT)), FLASK_BIT))

#define FLASK_INIT    FLASK_OFF; FLASK_OUT

//пин сенсора освещения
#define SENS_BIT   DECODE_BIT(SENS_PIN)
#define SENS_PORT  DECODE_PORT(SENS_PIN)

#define SENS_ON    (BIT_SET(SENS_PORT, SENS_BIT))
#define SENS_OFF   (BIT_CLEAR(SENS_PORT, SENS_BIT))
#define SENS_OUT   (BIT_SET((DDR_REG(SENS_PORT)), SENS_BIT))

#define SENS_INIT  SENS_OFF; SENS_OUT
