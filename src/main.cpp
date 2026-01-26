#include "main.h"

STM32WLx radio = new STM32WLx_Module();
int16_t stateRadio = RADIOLIB_ERR_NONE;

LoRaWANNode node(&radio, &Region, subBand);
int16_t stateNode = RADIOLIB_ERR_NONE;

uint64_t joinEUI = RADIOLIB_LORAWAN_JOIN_EUI;
uint64_t devEUI = RADIOLIB_LORAWAN_DEV_EUI;
uint8_t appKey[] = {RADIOLIB_LORAWAN_APP_KEY};
uint8_t nwkKey[] = {RADIOLIB_LORAWAN_NWK_KEY};

LoRaWANEvent_t uplinkDetails;
const uint8_t payloadSize = 2;
uint8_t uplinkPayload[payloadSize];

LoRaWANEvent_t downlinkDetails;
uint8_t downlinkPayload[RADIOLIB_LORAWAN_MAX_DOWNLINK_SIZE];
size_t downlinkSize = 0;

int main(void)
{
  HAL_Init();

  SystemClock_Config();
  GPIO_Init();
  Debug_Setup();
  RTC_Init();

  setup();
  for (;;)
  {
    loop();
  }
  return 0;
}

void setup()
{
  radioInit();

  node.setADR(enableADR);
  node.setDutyCycle(enableDutyCycle, dutyCycleTime);
  node.setDwellTime(enableDwellTime, dwellTime);
  node.scanGuard = receiveWindowPadding;

  stateRadio = node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
  DEBUG_PRINT_VAR("node.beginOTAA : ", stateDecode(stateRadio));
  if (stateRadio != RADIOLIB_ERR_NONE)
  {
    DEBUG_PRINT("node.beginOTAA failed");
  }

  stateRadio = node.activateOTAA();
  DEBUG_PRINT_VAR("node.activateOTAA : ", stateDecode(stateRadio));
  if (stateRadio != RADIOLIB_LORAWAN_NEW_SESSION)
  {
    DEBUG_PRINT("node.activateOTAA failed ");
  }
} // setup()

void loop()
{
  int16_t test = 1024;
  uplinkPayload[0] = highByte(test);
  uplinkPayload[1] = lowByte(test);
  loraSendSensor();
  stop2Mode(uplinkIntervalSeconds * 1000);
} // loop()

void radioInit()
{
  static const uint32_t rfswitch_pins[] = {rfswitch_pin1, rfswitch_pin2, rfswitch_pin3,
                                           rfswitch_pin4, rfswitch_pin5};
  static const Module::RfSwitchMode_t rfswitch_table[] = {
      {STM32WLx::MODE_IDLE, {LOW, LOW}},
      {STM32WLx::MODE_RX, {HIGH, LOW}},
      //{STM32WLx::MODE_TX_LP, {LOW, HIGH}},
      {STM32WLx::MODE_TX_HP, {LOW, HIGH}},
      END_OF_MODE_TABLE,
  };

  radio.XTAL = radioXTAL;
  radio.standbyXOSC = tcxoWakeup;
  stateRadio = RADIOLIB_ERR_NONE;

  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);

  DEBUG_PRINT("radio.begin() starting");
  stateRadio =
      radio.begin(freq, bw, sf, cr, syncWord, power, preambleLength, tcxoVoltage, useRegulatorLDO);
  DEBUG_PRINT_VAR("radio.begin() : ", stateDecode(stateRadio));
  if (stateRadio != RADIOLIB_ERR_NONE)
  {
    DEBUG_PRINT("radio.begin failed");
  }

  stateRadio = radio.setCurrentLimit(currentLimit);
  if (stateRadio != RADIOLIB_ERR_NONE)
  {
    DEBUG_PRINT_VAR("radio.setCurrentLimit failed", stateDecode(stateRadio));
  }

  stateRadio = radio.setDio2AsRfSwitch(dio2AsRfSwitch);
  if (stateRadio != RADIOLIB_ERR_NONE)
  {
    DEBUG_PRINT_VAR("radio.setDio2AsRfSwitch failed", stateDecode(stateRadio));
  }
  stateRadio = radio.setCRC(crcLength);
  if (stateRadio != RADIOLIB_ERR_NONE)
  {
    DEBUG_PRINT_VAR("radio.setCRC failed", stateDecode(stateRadio));
  }

} // loraRadioInit()

void radioWakeup(void)
{
  DEBUG_PRINT("radioWakeup() begin");

  __HAL_RCC_SUBGHZSPI_CLK_ENABLE();

  stateRadio = radio.standby(RADIOLIB_SX126X_STANDBY_XOSC);
  if (stateRadio != RADIOLIB_ERR_NONE)
  {
    DEBUG_PRINT_VAR("radio.standby() failed", stateDecode(stateRadio));
  }
  delay(DELAY_BEFORE_CHECKING_STATUS_MS);
  radioCheckStatus();
} // radioWakeup()

void loraSendSensor()
{
  DEBUG_PRINT("loraSendSensor() begin");

  stateRadio = RADIOLIB_ERR_NONE;

  uint32_t fCntUp = node.getFCntUp();
  uint32_t minTimeUntilUplink = (uint32_t)node.timeUntilUplink();

  DEBUG_PRINT_VAR("fCntUp : ", fCntUp);
  DEBUG_PRINT_VAR("minTimeUntilUplink : ", minTimeUntilUplink);

  radioWakeup();
  DEBUG_PRINT("node.sendReceive()");
  uint32_t txStart = millis();
  stateRadio = node.sendReceive(uplinkPayload, payloadSize, payloadFPort, downlinkPayload,
                                &downlinkSize, false, &uplinkDetails, &downlinkDetails);
  uint32_t txDuration = millis() - txStart;
  radioSleep();

  if (stateRadio >= RADIOLIB_ERR_NONE)
  {
    DEBUG_PRINT("Uplink success!");
    // downlinkProcess();
  }
  else
  {
    DEBUG_PRINT_VAR("Uplink failed : ", stateDecode(stateRadio));
  }

  DEBUG_PRINT_VAR("sendReceive() duration (ms): ", txDuration);
  if (txDuration > MAX_TX_TIME)
  {
    radioTimeout();
  }
}

void radioSleep(void)
{
  DEBUG_PRINT("radioSleep()");

  stateRadio = radio.sleep();
  if (stateRadio != RADIOLIB_ERR_NONE)
  {
    DEBUG_PRINT_VAR("radio.sleep() failed", stateDecode(stateRadio));
  }
} // radioSleep()

void radioTimeout(void)
{
  DEBUG_PRINT("********************* TX TIMEOUT *******************************");

  DEBUG_PRINT("Status before radioReset:");
  radioCheckStatus();
  radioReset();

  DEBUG_PRINT("Status before radioInit:");
  radioCheckStatus();
  radioInit();

  DEBUG_PRINT("Status before loraSendSensor:");
  radioCheckStatus();
  loraSendSensor();

  DEBUG_PRINT("Status after loraSendSensor:");
  radioCheckStatus();

  // Temporary loop to stop logs
  while (true)
  {
  };
}

void radioReset(void)
{
  DEBUG_PRINT("************************** RESETTING RADIO **************************");

  stateRadio = radio.reset();
  if (stateRadio != RADIOLIB_ERR_NONE)
  {
    DEBUG_PRINT_VAR("radio.reset() failed", stateDecode(stateRadio));
  }

  __HAL_RCC_SUBGHZSPI_FORCE_RESET();
  delay(10);
  __HAL_RCC_SUBGHZSPI_RELEASE_RESET();
  delay(10);
  __HAL_RCC_SUBGHZ_CLK_ENABLE();

  DEBUG_PRINT("reset done");

  delay(DELAY_BEFORE_CHECKING_STATUS_MS);
  radioCheckStatus();

} // radioReset()

void radioCheckStatus(void)
{
  delay(DELAY_BEFORE_CHECKING_STATUS_MS);

  DEBUG_PRINT("=== Radio Status ===");

  DEBUG_PRINT_VAR("BUSY flag (RFBUSYS): ", LL_PWR_IsActiveFlag_RFBUSYS() ? "SET" : "CLEAR (ok)");
  DEBUG_PRINT_VAR("BUSY flag (RFBUSYMS): ", LL_PWR_IsActiveFlag_RFBUSYMS() ? "SET (ok)" : "CLEAR");
  DEBUG_PRINT_VAR("SubGHz SPI Clock: ",
                  __HAL_RCC_SUBGHZSPI_IS_CLK_ENABLED() ? "ENABLED (ok)" : "DISABLED");
  DEBUG_PRINT_VAR("SUBGHZ IRQ Pending: ",
                  HAL_NVIC_GetPendingIRQ(SUBGHZ_Radio_IRQn) ? "YES" : "NO (ok)");

  // Send GET_STATUS command (0xC0) using SPI
  SubGhz.SPI.beginTransaction(SubGhz.spi_settings);
  SubGhz.setNssActive(true);

  // Send command and receive status
  SubGhz.SPI.transfer(0xC0);                      // GET_STATUS command
  uint8_t statusByte = SubGhz.SPI.transfer(0x00); // Read status byte

  SubGhz.setNssActive(false);
  SubGhz.SPI.endTransaction();

  // Extract chip mode (bits 6-4)
  uint8_t chipMode = (statusByte >> 4) & 0x07;
  // Extract command status (bits 3-1)
  uint8_t cmdStatus = (statusByte >> 1) & 0x07;

  // Decode chip mode
  switch (chipMode)
  {
  case 0x00:
    DEBUG_PRINT("Chip Mode: UNUSED");
    break;
  case 0x01:
    DEBUG_PRINT("Chip Mode: RFU");
    break;
  case 0x02:
    DEBUG_PRINT("Chip Mode: STDBY_RC");
    break;
  case 0x03:
    DEBUG_PRINT("Chip Mode: STDBY_XOSC (ok)");
    break;
  case 0x04:
    DEBUG_PRINT("Chip Mode: FS");
    break;
  case 0x05:
    DEBUG_PRINT("Chip Mode: RX");
    break;
  case 0x06:
    DEBUG_PRINT("Chip Mode: TX");
    break;
  default:
    DEBUG_PRINT_VAR("Chip Mode: UNKNOWN ", chipMode);
    break;
  }

  // Decode command status
  switch (cmdStatus)
  {
  case 0x00:
    DEBUG_PRINT("Command Status: RESERVED");
    break;
  case 0x01:
    DEBUG_PRINT("Command Status: RFU (ok)");
    break;
  case 0x02:
    DEBUG_PRINT("Command Status: Data Available to host");
    break;
  case 0x03:
    DEBUG_PRINT("Command Status: Command Timeout");
    break;
  case 0x04:
    DEBUG_PRINT("Command Status: Command Processing error");
    break;
  case 0x05:
    DEBUG_PRINT("Command Status: Failure to execute command");
    break;
  case 0x06:
    DEBUG_PRINT("Command Status: Command TX Done");
    break;
  default:
    DEBUG_PRINT_VAR("Command Status: UNKNOWN ", cmdStatus);
    break;
  }
  DEBUG_PRINT_VAR("Raw Status: ", statusByte);

  // Get Device Errors (0x17)
  SubGhz.SPI.beginTransaction(SubGhz.spi_settings);
  SubGhz.setNssActive(true);

  SubGhz.SPI.transfer(0x17);                    // GET_DEVICE_ERRORS command
  uint8_t errByte1 = SubGhz.SPI.transfer(0x00); // Read error MSB
  uint8_t errByte2 = SubGhz.SPI.transfer(0x00); // Read error LSB

  SubGhz.setNssActive(false);
  SubGhz.SPI.endTransaction();

  uint16_t deviceErrors = ((uint16_t)errByte1 << 8) | errByte2;
  DEBUG_PRINT_VAR("Device Errors raw: ", deviceErrors);

  if (deviceErrors == 0)
  {
    DEBUG_PRINT("No device errors (ok)");
  }
  else
  {
    // Known error bits
    if (deviceErrors & 0x8000)
      DEBUG_PRINT("Device Errors: [Bit 15] RFU");
    if (deviceErrors & 0x4000)
      DEBUG_PRINT("Device Errors: [Bit 14] RFU");
    if (deviceErrors & 0x2000)
      DEBUG_PRINT("Device Errors: [Bit 13] RFU");
    if (deviceErrors & 0x1000)
      DEBUG_PRINT("Device Errors: [Bit 12] RFU");
    if (deviceErrors & 0x0800)
      DEBUG_PRINT("Device Errors: [Bit 11] RFU");
    if (deviceErrors & 0x0400)
      DEBUG_PRINT("Device Errors: [Bit 10] RFU");
    if (deviceErrors & 0x0200)
      DEBUG_PRINT("Device Errors: [Bit 9] RFU");
    if (deviceErrors & 0x0100)
      DEBUG_PRINT("Device Errors: [Bit 8] PA_RAMP (PA ramp failed)");
    if (deviceErrors & 0x0080)
      DEBUG_PRINT("Device Errors: [Bit 7] RFU");
    if (deviceErrors & 0x0040)
      DEBUG_PRINT("Device Errors: [Bit 6] PLL_LOCK (PLL lock failed)");
    if (deviceErrors & 0x0020)
      DEBUG_PRINT("Device Errors: [Bit 5] XOSC_START (XOSC start failed)");
    if (deviceErrors & 0x0010)
      DEBUG_PRINT("Device Errors: [Bit 4] IMG_CALIB (Image calibration failed)");
    if (deviceErrors & 0x0008)
      DEBUG_PRINT("Device Errors: [Bit 3] ADC_CALIB (ADC calibration failed)");
    if (deviceErrors & 0x0004)
      DEBUG_PRINT("Device Errors: [Bit 2] PLL_CALIB (PLL calibration failed)");
    if (deviceErrors & 0x0002)
      DEBUG_PRINT("Device Errors: [Bit 1] RC13M_CALIB (RC13M calibration failed)");
    if (deviceErrors & 0x0001)
      DEBUG_PRINT("Device Errors: [Bit 0] RC64K_CALIB (RC64K calibration failed)");

    // Check for completely unknown bits (outside documented range)
    uint16_t knownMask = 0xFFFF; // All bits are now documented
    uint16_t unknownBits = deviceErrors & ~knownMask;
    if (unknownBits != 0)
    {
      DEBUG_PRINT_VAR("Device Errors: UNEXPECTED ERROR BITS: 0x", String(unknownBits, HEX));
    }
  }

  // Get IRQ Status (0x12)
  SubGhz.SPI.beginTransaction(SubGhz.spi_settings);
  SubGhz.setNssActive(true);

  SubGhz.SPI.transfer(0x12);                    // GET_IRQ_STATUS command
  uint8_t irqByte1 = SubGhz.SPI.transfer(0x00); // Read IRQ MSB
  uint8_t irqByte2 = SubGhz.SPI.transfer(0x00); // Read IRQ LSB

  SubGhz.setNssActive(false);
  SubGhz.SPI.endTransaction();

  uint16_t irqStatus = ((uint16_t)irqByte1 << 8) | irqByte2;
  DEBUG_PRINT_VAR("IRQ Status raw: ", irqStatus);

  if (irqStatus == 0)
  {
    DEBUG_PRINT("No IRQ flags set");
  }
  else
  {
    if (irqStatus & 0x8000)
      DEBUG_PRINT("IRQ Status: [Bit 15] RFU");
    if (irqStatus & 0x4000)
      DEBUG_PRINT("IRQ Status: [Bit 14] RFU");
    if (irqStatus & 0x2000)
      DEBUG_PRINT("IRQ Status: [Bit 13] RFU");
    if (irqStatus & 0x1000)
      DEBUG_PRINT("IRQ Status: [Bit 12] RFU");
    if (irqStatus & 0x0800)
      DEBUG_PRINT("IRQ Status: [Bit 11] RFU");
    if (irqStatus & 0x0400)
      DEBUG_PRINT("IRQ Status: [Bit 10] RFU");
    if (irqStatus & 0x0200)
      DEBUG_PRINT("IRQ Status: [Bit 9] TIMEOUT (ok if no downlink)");
    if (irqStatus & 0x0100)
      DEBUG_PRINT("IRQ Status: [Bit 8] CAD_DETECTED");
    if (irqStatus & 0x0080)
      DEBUG_PRINT("IRQ Status: [Bit 7] CAD_DONE");
    if (irqStatus & 0x0040)
      DEBUG_PRINT("IRQ Status: [Bit 6] CRC_ERR");
    if (irqStatus & 0x0020)
      DEBUG_PRINT("IRQ Status: [Bit 5] HEADER_ERR");
    if (irqStatus & 0x0010)
      DEBUG_PRINT("IRQ Status: [Bit 4] HEADER_VALID");
    if (irqStatus & 0x0008)
      DEBUG_PRINT("IRQ Status: [Bit 3] SYNC_WORD_VALID");
    if (irqStatus & 0x0004)
      DEBUG_PRINT("IRQ Status: [Bit 2] PREAMBLE_DETECTED");
    if (irqStatus & 0x0002)
      DEBUG_PRINT("IRQ Status: [Bit 1] RX_DONE");
    if (irqStatus & 0x0001)
      DEBUG_PRINT("IRQ Status: [Bit 0] TX_DONE");

    // Check for completely unknown bits
    uint16_t knownMask = 0xFFFF; // All bits are now documented
    uint16_t unknownBits = irqStatus & ~knownMask;
    if (unknownBits != 0)
    {
      DEBUG_PRINT_VAR("IRQ Status: UNEXPECTED IRQ BITS: 0x", String(unknownBits, HEX));
    }
  }

  DEBUG_PRINT("==================");
} // radioCheckStatus()

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};

  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE |
                                     RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_PWR;
  RCC_OscInitStruct.HSEDiv = RCC_HSE_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV128;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_HCLK3 |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

void GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  // Do not configure PA13 and PA14 (SWDIO and SWCLK)
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                        GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
                        GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
                        GPIO_PIN_12 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Do not configure PB8 and PC13 (SubGHz)
  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |
                        GPIO_PIN_6 | GPIO_PIN_7 |
                        GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Do not configure PC13 (SubGHz), PC14 (OSC32_IN for LSE) and PC15 (OSC32_OUT for LSE)
}

void GPIO_Sleep(void)
{
  GPIO_Init();
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  // __HAL_RCC_GPIOC_CLK_DISABLE(); Keep GPIOC_CLK for LSE
  delay(10);
}

void stop2Mode(time_t sleepMs)
{
  rtcSetAlarm(sleepMs);

  DEBUG_PRINT("Stop 2 mode. Waiting for interrupt...\n\n");

  GPIO_Sleep();
  HAL_SuspendTick();
  HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

  // Wait for interrupt from RTC or GPIO

  SystemClock_Config();
  HAL_ResumeTick();
  GPIO_Init();
  Debug_Setup();

  DEBUG_PRINT("stop2Mode() end");
} // stop2Mode

void RTC_Init(void)
{
  DEBUG_PRINT("rtcSetup() starting");

  HAL_StatusTypeDef status;

  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127; // 128 async prescaler
  hrtc.Init.SynchPrediv = 255;  // 256 sync prescaler
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_NONE;

  status = HAL_RTC_Init(&hrtc);
  if (status != HAL_OK)
  {
    DEBUG_PRINT_VAR("HAL_RTC_Init failed : ", status);
    delay(10);
    Error_Handler();
  }

  RTC_TimeTypeDef sTime = {};
  RTC_DateTypeDef sDate = {};

  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    DEBUG_PRINT("HAL_RTC_SetTime failed");
    Error_Handler();
  }

  sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    DEBUG_PRINT("HAL_RTC_SetDate failed");
    Error_Handler();
  }

  HAL_RTC_MspInit(&hrtc);

  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 1, 0); // SubGHz uses priority 0
  HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);

  __HAL_RTC_WAKEUPTIMER_EXTI_ENABLE_IT();
  HAL_RTCEx_EnableBypassShadow(&hrtc);

  DEBUG_PRINT("RTC time and date initialized");
}

void rtcSetAlarm(time_t ms)
{
  DEBUG_PRINT("rtcSetAlarm()");

  uint32_t seconds = ms / 1000UL;
  if (seconds > 65535)
    seconds = 65535; // Max for 16-bit mode

  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, seconds - 1, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0) !=
      HAL_OK)
  {
    DEBUG_PRINT("HAL_RTCEx_SetWakeUpTimer_IT failed");
    Error_Handler();
  }
  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
}

void rtcAlarmAISR(void)
{
  DEBUG_PRINT("--- rtcISR ----");
} // rtcISR

extern "C"
{
  void HAL_RTC_MspInit(RTC_HandleTypeDef *rtcHandle)
  {
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

    if (rtcHandle->Instance == RTC)
    {
      PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
      PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;

      if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
      {
        DEBUG_PRINT("HAL_RCCEx_PeriphCLKConfig failed");
        Error_Handler();
      }

      HAL_PWR_EnableBkUpAccess();
      __HAL_RCC_RTCAPB_CLK_ENABLE();
      __HAL_RCC_RTC_ENABLE();
    }
  }

  void HAL_RTC_MspDeInit(RTC_HandleTypeDef *rtcHandle)
  {
    if (rtcHandle->Instance == RTC)
    {
      __HAL_RCC_RTC_DISABLE();
      __HAL_RCC_RTCAPB_CLK_DISABLE();

      HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);
    }
  }

  void RTC_WKUP_IRQHandler(void)
  {
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    if (hrtc.Instance != NULL)
    {
      HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
    }
  }

  void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *rtcHandle)
  {
    UNUSED(rtcHandle);
    rtcAlarmAISR();
  }

} // extern "C"

void Debug_Setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
  }
  delay(1000);
  DEBUG_PRINT("setup() starting");
}

String stateDecode(const int16_t result)
{
  String decoded;

  switch (result)
  {
  case RADIOLIB_ERR_NONE:
    decoded = "ERR_NONE";
    break;
  case RADIOLIB_ERR_CHIP_NOT_FOUND:
    decoded = "ERR_CHIP_NOT_FOUND";
    break;
  case RADIOLIB_ERR_PACKET_TOO_LONG:
    decoded = "ERR_PACKET_TOO_LONG";
    break;
  case RADIOLIB_ERR_RX_TIMEOUT:
    decoded = "ERR_RX_TIMEOUT";
    break;
  case RADIOLIB_ERR_MIC_MISMATCH:
    decoded = "ERR_MIC_MISMATCH";
    break;
  case RADIOLIB_ERR_INVALID_BANDWIDTH:
    decoded = "ERR_INVALID_BANDWIDTH";
    break;
  case RADIOLIB_ERR_INVALID_SPREADING_FACTOR:
    decoded = "ERR_INVALID_SPREADING_FACTOR";
    break;
  case RADIOLIB_ERR_INVALID_CODING_RATE:
    decoded = "ERR_INVALID_CODING_RATE";
    break;
  case RADIOLIB_ERR_INVALID_FREQUENCY:
    decoded = "ERR_INVALID_FREQUENCY";
    break;
  case RADIOLIB_ERR_INVALID_OUTPUT_POWER:
    decoded = "ERR_INVALID_OUTPUT_POWER";
    break;
  case RADIOLIB_ERR_NETWORK_NOT_JOINED:
    decoded = "RADIOLIB_ERR_NETWORK_NOT_JOINED";
    break;
  case RADIOLIB_ERR_DOWNLINK_MALFORMED:
    decoded = "RADIOLIB_ERR_DOWNLINK_MALFORMED";
    break;
  case RADIOLIB_ERR_INVALID_REVISION:
    decoded = "RADIOLIB_ERR_INVALID_REVISION";
    break;
  case RADIOLIB_ERR_INVALID_PORT:
    decoded = "RADIOLIB_ERR_INVALID_PORT";
    break;
  case RADIOLIB_ERR_NO_RX_WINDOW:
    decoded = "RADIOLIB_ERR_NO_RX_WINDOW";
    break;
  case RADIOLIB_ERR_INVALID_CID:
    decoded = "RADIOLIB_ERR_INVALID_CID";
    break;
  case RADIOLIB_ERR_UPLINK_UNAVAILABLE:
    decoded = "RADIOLIB_ERR_UPLINK_UNAVAILABLE";
    break;
  case RADIOLIB_ERR_COMMAND_QUEUE_FULL:
    decoded = "RADIOLIB_ERR_COMMAND_QUEUE_FULL";
    break;
  case RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND:
    decoded = "RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND";
    break;
  case RADIOLIB_ERR_JOIN_NONCE_INVALID:
    decoded = "RADIOLIB_ERR_JOIN_NONCE_INVALID";
    break;
  case RADIOLIB_ERR_DWELL_TIME_EXCEEDED:
    decoded = "RADIOLIB_ERR_DWELL_TIME_EXCEEDED";
    break;
  case RADIOLIB_ERR_CHECKSUM_MISMATCH:
    decoded = "RADIOLIB_ERR_CHECKSUM_MISMATCH";
    break;
  case RADIOLIB_ERR_NO_JOIN_ACCEPT:
    decoded = "RADIOLIB_ERR_NO_JOIN_ACCEPT";
    break;
  case RADIOLIB_LORAWAN_SESSION_RESTORED:
    decoded = "RADIOLIB_LORAWAN_SESSION_RESTORED";
    break;
  case RADIOLIB_LORAWAN_NEW_SESSION:
    decoded = "RADIOLIB_LORAWAN_NEW_SESSION";
    break;
  case RADIOLIB_ERR_NONCES_DISCARDED:
    decoded = "RADIOLIB_ERR_NONCES_DISCARDED";
    break;
  case RADIOLIB_ERR_SESSION_DISCARDED:
    decoded = "RADIOLIB_ERR_SESSION_DISCARDED";
    break;
  case 1:
    decoded = "DOWNLINK_RECEIVED_RX1";
    break;
  case 2:
    decoded = "DOWNLINK_RECEIVED_RX2";
    break;
  case 3:
    // Class C device or Multicast downlink
    decoded = "RADIOLIB_LORAWAN_RX_BC";
    break;
  default:
    decoded = "UNKNOWN";
    break;
  }

  // Append numeric code
  decoded += " (";
  decoded += String(result);
  decoded += ")";

  return decoded;
}