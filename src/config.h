#ifndef CONFIG_H
#define CONFIG_H

#include "RadioLib.h"
#include "lorawanKeys.h"

// =============================================================================
// LORAWAN CONFIGURATION
// =============================================================================

// LoRaWAN Keys and EUIs (put your own keys in /src/lorawanKeys.h)
//#define RADIOLIB_LORAWAN_DEV_EUI   0x
//#define RADIOLIB_LORAWAN_APP_KEY   0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x
//#define RADIOLIB_LORAWAN_NWK_KEY   0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x, 0x
//#define RADIOLIB_LORAWAN_JOIN_EUI  0x

// Regional Configuration
const LoRaWANBand_t Region = US915;
const uint8_t subBand = 2;

// Radio Configuration
const float freq = 915.0F;
const float bw = 125.0F;
const uint8_t sf = 10U;
const uint8_t cr = 5U;
const uint8_t joinDR = 3U;
const uint8_t startDR = 3U;

// Protocol Configuration
const bool enableADR = false; // ADR controls: Spreading factor, Bandwidth, Transmission power
const bool enableDutyCycle = false;
const RadioLibTime_t dutyCycleTime = 0UL;
const bool enableDwellTime = true;
const RadioLibTime_t dwellTime = 400UL;

// Frame Configuration
const uint8_t syncWord = 0x34;
const uint16_t preambleLength = 8U;
const uint8_t crcLength = 2U;

// Port Configuration
const uint8_t payloadFPort = 223; // Application-specific port for sensor data

// =============================================================================
// TIMING CONFIGURATION
// =============================================================================

// Uplink interval configuration
time_t uplinkIntervalSeconds = 10UL;

// LoRaWAN Timing Configuration
const RadioLibTime_t receiveWindowPadding = 100UL;

// =============================================================================
// BOARD CONFIGURATION
// =============================================================================

// RF Switch Configuration
const uint32_t rfswitch_pin1 = PB8;
const uint32_t rfswitch_pin2 = PC13;
const uint32_t rfswitch_pin3 = RADIOLIB_NC;
const uint32_t rfswitch_pin4 = RADIOLIB_NC;
const uint32_t rfswitch_pin5 = RADIOLIB_NC;
const bool dio2AsRfSwitch = false;

// Clock Configuration
const bool radioXTAL = false;   // Set to false if using an external TCXO
const bool tcxoWakeup = true;   // Set to false if using XTAL oscillator
const float tcxoVoltage = 3.0F; // Set to 0.0F if using XTAL oscillator

// Power Configuration
const bool useRegulatorLDO = true;
const int8_t power = 18;
const float currentLimit = 140.0F;

// Battery level
uint8_t batteryLevel = 255; // Not supported

#endif // CONFIG_H