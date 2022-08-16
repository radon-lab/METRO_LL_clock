//Соединения периферии с пинами МК
//    (0 - D0 | 1 - D1 | 2 - D2 | 3 - D3 | 4 - D4 | 5 - D5 | 6 - D6 | 7 - D7)
//        (8 - D8 | 9 - D9 | 10 - D10 | 11 - D11 | 12 - D12 | 13 - D13)
//         (14 - A0 | 15 - A1 | 16 - A2 | 17 - A3 | 18 - A4 | 19 - A5)

#define CATHODE_1 16 //пин катода 1(0..19)(pin D)
#define CATHODE_2 13 //пин катода 2(0..19)(pin D)
#define CATHODE_3 12 //пин катода 3(0..19)(pin D)
#define CATHODE_4 11 //пин катода 4(0..19)(pin D)
 
#define ANODE_A 14  //пин анода A(0..19)(pin D)
#define ANODE_B 8   //пин анода B(0..19)(pin D)
#define ANODE_C 7   //пин анода C(0..19)(pin D)
#define ANODE_D 6   //пин анода D(0..19)(pin D)
#define ANODE_E 4   //пин анода E(0..19)(pin D)
#define ANODE_F 10  //пин анода F(0..19)(pin D)
#define ANODE_G 3   //пин анода G(0..19)(pin D)

#define RIGHT_PIN 2 //пин правой кнопки(2)(pin D)
#define LEFT_PIN 0  //пин левой кнопки(0..7)(pin D)
#define DOT_PIN 5   //пин секундных точек(0..19)(pin D)
#define FLASK_PIN 9 //пин колбы(0..19)(pin D)
#define SENS_PIN 15 //пин сенсора освещения(0..19)(pin D)

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

#define CATHODE_1_PORT DECODE_PORT(CATHODE_1)
#define CATHODE_1_BIT DECODE_BIT(CATHODE_1)
#define CATHODE_2_PORT DECODE_PORT(CATHODE_2)
#define CATHODE_2_BIT DECODE_BIT(CATHODE_2)
#define CATHODE_3_PORT DECODE_PORT(CATHODE_3)
#define CATHODE_3_BIT DECODE_BIT(CATHODE_3)
#define CATHODE_4_PORT DECODE_PORT(CATHODE_4)
#define CATHODE_4_BIT DECODE_BIT(CATHODE_4)

#define ANODE_A_PORT DECODE_PORT(ANODE_A)
#define ANODE_A_BIT DECODE_BIT(ANODE_A)
#define ANODE_B_PORT DECODE_PORT(ANODE_B)
#define ANODE_B_BIT DECODE_BIT(ANODE_B)
#define ANODE_C_PORT DECODE_PORT(ANODE_C)
#define ANODE_C_BIT DECODE_BIT(ANODE_C)
#define ANODE_D_PORT DECODE_PORT(ANODE_D)
#define ANODE_D_BIT DECODE_BIT(ANODE_D)
#define ANODE_E_PORT DECODE_PORT(ANODE_E)
#define ANODE_E_BIT DECODE_BIT(ANODE_E)
#define ANODE_F_PORT DECODE_PORT(ANODE_F)
#define ANODE_F_BIT DECODE_BIT(ANODE_F)
#define ANODE_G_PORT DECODE_PORT(ANODE_G)
#define ANODE_G_BIT DECODE_BIT(ANODE_G)

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
