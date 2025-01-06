#ifndef _M5EPD_H_
#define _M5EPD_H_

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "FS.h"
#include "SD.h"
#include "utility/Button.h"
#include "utility/GT911.h"
#include "utility/BM8563.h"
#include "utility/SHT3x.h"

#include "M5EPD_Canvas.h"
#include "M5EPD_Driver.h"

#include "esp_adc_cal.h"

#define M5EPD_MAIN_PWR_PIN   GPIO_NUM_2
#define M5EPD_CS_PIN         GPIO_NUM_15
#define M5EPD_SCK_PIN        GPIO_NUM_14
#define M5EPD_MOSI_PIN       GPIO_NUM_12
#define M5EPD_BUSY_PIN       GPIO_NUM_27
#define M5EPD_MISO_PIN       GPIO_NUM_13
#define M5EPD_EXT_PWR_EN_PIN GPIO_NUM_5
#define M5EPD_EPD_PWR_EN_PIN GPIO_NUM_23
#define M5EPD_KEY_RIGHT_PIN  GPIO_NUM_39
#define M5EPD_KEY_PUSH_PIN   GPIO_NUM_38
#define M5EPD_KEY_LEFT_PIN   GPIO_NUM_37
#define M5EPD_BAT_VOL_PIN    GPIO_NUM_35
#define M5EPD_PORTC_W_PIN    GPIO_NUM_19
#define M5EPD_PORTC_Y_PIN    GPIO_NUM_18
#define M5EPD_PORTB_W_PIN    GPIO_NUM_33
#define M5EPD_PORTB_Y_PIN    GPIO_NUM_26
#define M5EPD_PORTA_W_PIN    GPIO_NUM_32
#define M5EPD_PORTA_Y_PIN    GPIO_NUM_25
#define M5EPD_SD_CS_PIN      GPIO_NUM_4
#define M5EPD_CARD_CD_PIN    GPIO_NUM_34
#define M5EPD_TOUCH_ISR_PIN  GPIO_NUM_36
#define M5EPD_I2C_SDA_PIN    GPIO_NUM_21
#define M5EPD_I2C_SCL_PIN    GPIO_NUM_22

class M5EPD {
   public:
    M5EPD();
    void begin(bool touchEnable = true, bool SDEnable = true,
               bool SerialEnable = true, bool BatteryADCEnable = true,
               bool I2CEnable = false);
    void update();
    void enableEXTPower() {
        digitalWrite(M5EPD_EXT_PWR_EN_PIN, 1);
    }
    void disableEXTPower() {
        digitalWrite(M5EPD_EXT_PWR_EN_PIN, 0);
    }
    void enableEPDPower() {
        digitalWrite(M5EPD_EPD_PWR_EN_PIN, 1);
    }
    void disableEPDPower() {
        digitalWrite(M5EPD_EPD_PWR_EN_PIN, 0);
    }
    void enableMainPower() {
        digitalWrite(M5EPD_MAIN_PWR_PIN, 1);
    }
    void disableMainPower() {
        digitalWrite(M5EPD_MAIN_PWR_PIN, 0);
    }
    void BatteryADCBegin();
    uint32_t getBatteryRaw();
    uint32_t getBatteryVoltage();

    void shutdown();
    int shutdown(int seconds);
    int shutdown(const rtc_time_t &RTC_TimeStruct);
    int shutdown(const rtc_date_t &RTC_DateStruct,
                 const rtc_time_t &RTC_TimeStruct);

    Button BtnL = Button(M5EPD_KEY_LEFT_PIN, true, 10);
    Button BtnP = Button(M5EPD_KEY_PUSH_PIN, true, 10);
    Button BtnR = Button(M5EPD_KEY_RIGHT_PIN, true, 10);

    M5EPD_Driver EPD = M5EPD_Driver();
    GT911 TP         = GT911();
    BM8563 RTC       = BM8563();
    SHT3x SHT30      = SHT3x();

   private:
    bool _is_adc_start;
    bool _isInited;
    esp_adc_cal_characteristics_t *_adc_chars;
};

extern M5EPD M5;

#endif
