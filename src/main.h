#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include "stm32wlxx_hal.h"
#include "config.h"

#define DEBUG_PRINT(x)     \
    do                     \
    {                      \
        Serial.println(x); \
        Serial.flush();    \
    } while (0)
#define DEBUG_PRINT_VAR(a, b) \
    do                        \
    {                         \
        Serial.print(a);      \
        Serial.println(b);    \
        Serial.flush();       \
    } while (0)

#define MAX_TX_TIME 5000
#define DELAY_BEFORE_CHECKING_STATUS_MS 100

RTC_HandleTypeDef hrtc;

extern void setup();
extern void loop();

extern void radioInit();
extern void radioWakeup();
extern void loraSendSensor();
extern void radioSleep();
extern void radioReset();
extern void radioCheckStatus();

extern void GPIO_Init(void);
extern void GPIO_Sleep(void);
extern void SystemClock_Config(void);
extern void stop2Mode(time_t sleepMs);

extern void RTC_Init(void);
extern void rtcSetAlarm(time_t alarmMs);
extern void rtcAlarmAISR(void);

extern void Debug_Setup();
extern String stateDecode(const int16_t result);

#endif // MAIN_H
