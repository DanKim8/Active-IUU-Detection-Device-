/* catena4450m101_sensor.ino	Tue Mar 28 2017 19:52:20 tmm */

/*
//Edited by: Jonathan Lafleur on 2/10/2018 to add the ultrasonic sensor data for transmission

Module:  catena4450m101_sensor.ino

Function:
	Code for the electric sensor with Catena 4450-M101.

Version:
	V0.1.1	Tue Mar 28 2017 19:52:20 tmm	Edit level 1

Copyright notice:
	This file copyright (C) 2017 by

		MCCI Corporation
		3520 Krums Corners Road
		Ithaca, NY  14850

	An unpublished work.  All rights reserved.

	This file is proprietary information, and may not be disclosed or
	copied without the prior permission of MCCI Corporation.

Author:
	Terry Moore, MCCI Corporation	March 2017

Revision history:
   0.1.0  Fri Mar 10 2017 21:42:21  tmm
	Module created.

   0.1.1  Tue Mar 28 2017 19:52:20  tmm
	Fix bug: not reading current lux value because sensor was not in
	continuous mode. Add 150uA of current draw.

*/

#include "ThisCatena.h"


#include <Catena_Led.h>
#include <Catena_TxBuffer.h>
#include <Catena_CommandStream.h>
#include <Catena_Totalizer.h>

#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Arduino_LoRaWAN.h>
#include <BH1750.h>
#include <lmic.h>
#include <hal/hal.h>
#include <mcciadk_baselib.h>

#include <cmath>
#include <type_traits>

/****************************************************************************\
|
|		Manifest constants & typedefs.
|
|	This is strictly for private types and constants which will not
|	be exported.
|
\****************************************************************************/

using namespace McciCatena;
//using ThisCatena = Catena4450;

/* how long do we wait between measurements (in seconds) */
enum    {
        // set this to interval between measurements, in seconds
        // Actual time will be a little longer because have to
        // add measurement and broadcast time.
        CATCFG_T_CYCLE = 1 * 60,        // 60 messages/hour
        CATCFG_T_WARMUP = 1,
        CATCFG_T_SETTLE = 5,
        CATCFG_T_INTERVAL = CATCFG_T_CYCLE - (CATCFG_T_WARMUP +
                                                CATCFG_T_SETTLE),
        };

// forwards
static void settleDoneCb(osjob_t *pSendJob);
static void warmupDoneCb(osjob_t *pSendJob);
static void txFailedDoneCb(osjob_t *pSendJob);
static void sleepDoneCb(osjob_t *pSendJob);
static Arduino_LoRaWAN::SendBufferCbFn sendBufferDoneCb;

/****************************************************************************\
|
|	Read-only data.
|
|	If program is to be ROM-able, these must all be tagged read-only
|	using the ROM storage class; they may be global.
|
\****************************************************************************/

static const char sVersion[] = "0.1.7";

/****************************************************************************\
|
|	VARIABLES:
|
|	If program is to be ROM-able, these must be initialized
|	using the BSS keyword.  (This allows for compilers that require
|	every variable to have an initializer.)  Note that only those
|	variables owned by this module should be declared here, using the BSS
|	keyword; this allows for linkers that dislike multiple declarations
|	of objects.
|
\****************************************************************************/

// globals
ThisCatena gCatena;

//
// the LoRaWAN backhaul.  Note that we use the
// ThisCatena version so it can provide hardware-specific
// information to the base class.
//
ThisCatena::LoRaWAN gLoRaWAN;

//
// the LED
//
StatusLed gLed (ThisCatena::PIN_STATUS_LED);

// the RTC instance, used for sleeping
#ifdef ARDUINO_ARCH_SAMD
CatenaRTC gRtc;
#elif defined(ARDUINO_ARCH_STM32)
CatenaStm32L0Rtc gRtc;
#endif

//   The temperature/humidity sensor
Adafruit_BME280 bme; // The default initalizer creates an I2C connection
bool fBme;

//   The LUX sensor
BH1750 bh1750;
bool fLux;

//   The contact sensors
bool fHasPower1;
uint8_t kPinPower1P1;
uint8_t kPinPower1P2;

cTotalizer gPower1P1;
cTotalizer gPower1P2;

//  the job that's used to synchronize us with the LMIC code
static osjob_t sensorJob;
void sensorJob_cb(osjob_t *pJob);

// function for scaling power
static uint16_t
dNdT_getFrac(
        uint32_t deltaC,
        uint32_t delta_ms
        );

//Pins for HC-SR04 UltraSonic Sensor
#define TRIG_PIN A0
#define ECHO_PIN A1
bool hasUltraSonic=true;

// Anything over 400 cm (23200 us pulse) is "out of range"
const unsigned int MAX_DIST = 23200;

void setup(void)
        {
        gCatena.begin();

        gCatena.SafePrintf("Catena 4450 sensor1 V%s\n", sVersion);

        gLed.begin();
        gCatena.registerObject(&gLed);

        // set up the RTC object
        gRtc.begin();

        gCatena.SafePrintf("LoRaWAN init: ");
        if (!gLoRaWAN.begin(&gCatena))
                {
                gCatena.SafePrintf("failed\n");
                gCatena.registerObject(&gLoRaWAN);
                }
        else
                {
                gCatena.SafePrintf("OK\n");
                gCatena.registerObject(&gLoRaWAN);
                }

        ThisCatena::UniqueID_string_t CpuIDstring;

        gCatena.SafePrintf("CPU Unique ID: %s\n",
                gCatena.GetUniqueIDstring(&CpuIDstring)
                );

        /* find the platform */
        const ThisCatena::EUI64_buffer_t *pSysEUI = gCatena.GetSysEUI();

        uint32_t flags;
        const CATENA_PLATFORM * const pPlatform = gCatena.GetPlatform();

        if (pPlatform)
                {
                gCatena.SafePrintf("EUI64: ");
                for (unsigned i = 0; i < sizeof(pSysEUI->b); ++i)
                        {
                        gCatena.SafePrintf("%s%02x", i == 0 ? "" : "-", pSysEUI->b[i]);
                        }
                gCatena.SafePrintf("\n");
                flags = gCatena.GetPlatformFlags();
                gCatena.SafePrintf(
                        "Platform Flags:  %#010x\n",
                        flags
                        );
                gCatena.SafePrintf(
                        "Operating Flags:  %#010x\n",
                        gCatena.GetOperatingFlags()
                        );
                }
        else
                {
                gCatena.SafePrintf("**** no platform, check provisioning ****\n");
                flags = 0;
                }


        /* initialize the lux sensor */
        if (flags & ThisCatena::fHasLuxRohm)
                {
                bh1750.begin();
                fLux = true;
                bh1750.configure(BH1750_CONTINUOUS_HIGH_RES_MODE_2);
                }
        else
                {
                fLux = false;
                }

        /* initialize the BME280 */
        if (!bme.begin(BME280_ADDRESS, Adafruit_BME280::OPERATING_MODE::Sleep))
                {
                gCatena.SafePrintf("No BME280 found: check wiring\n");
                fBme = false;
                }
        else
                {
                fBme = true;
                }

        /* is it modded? */
        uint32_t modnumber = gCatena.PlatformFlags_GetModNumber(flags);

        fHasPower1 = false;

        if (modnumber != 0)
                {
                gCatena.SafePrintf("Catena 4450-M%u\n", modnumber);
                if (modnumber == 101)
                        {
                        fHasPower1 = true;
                        kPinPower1P1 = A0;
                        kPinPower1P2 = A1;
                        }
                else
                        {
                        gCatena.SafePrintf("unknown mod number %d\n", modnumber);
                        }
                }
        else
                {
                gCatena.SafePrintf("No mods detected\n");
                }

        if (fHasPower1)
                {
                if (! gPower1P1.begin(kPinPower1P1) ||
                    ! gPower1P2.begin(kPinPower1P2))
                        {
                        fHasPower1 = false;
                        }
                }

        /* now, we kick off things by sending our first message */
        gLed.Set(LedPattern::Joining);

        // unit testing for the scaling functions
        //gCatena.SafePrintf(
        //        "dNdT_getFrac tests: "
        //        "0/0: %04x 90/6m: %04x 89/6:00.1: %04x 1439/6m: %04x\n",
        //        dNdT_getFrac(0, 0),
        //        dNdT_getFrac(90, 6 * 60 * 1000),
        //        dNdT_getFrac(89, 6 * 60 * 1000 + 100),
        //        dNdT_getFrac(1439, 6 * 60 * 1000)
        //        );
        //gCatena.SafePrintf(
        //        "dNdT_getFrac tests: "
        //        "1/6m: %04x 20/6m: %04x 1/60:00.1: %04x 1440/5:59.99: %04x\n",
        //        dNdT_getFrac(1, 6 * 60 * 1000),
        //        dNdT_getFrac(20, 6 * 60 * 1000),
        //        dNdT_getFrac(1, 60 * 60 * 1000 + 100),
        //        dNdT_getFrac(1440, 6 * 60 * 1000 - 10)
        //        );

        /* warm up the BME280 by discarding a measurement */
        if (fBme)
                (void)bme.readTemperature();
                
        // The Trigger pin will tell the sensor to range find
        if(hasUltraSonic){
          pinMode(TRIG_PIN, OUTPUT);
          digitalWrite(TRIG_PIN, LOW);
        }
        /* trigger a join by sending the first packet */
        startSendingUplink();
        }

// The Arduino loop routine -- in our case, we just drive the other loops.
// If we try to do too much, we can break the LMIC radio. So the work is
// done by outcalls scheduled from the LMIC os loop.
void loop()
        {
        gCatena.poll();
        }

static uint16_t dNdT_getFrac(
        uint32_t deltaC,
        uint32_t delta_ms
        )
        {
        if (delta_ms == 0 || deltaC == 0)
                return 0;

        // this is a value in [0,1)
        float dNdTperHour = float(deltaC * 250) / float(delta_ms);

        if (dNdTperHour <= 0)
                return 0;
        else if (dNdTperHour >= 1)
                return 0xFFFF;
        else
                {
                int iExp;
                float normalValue;
                normalValue = frexpf(dNdTperHour, &iExp);

                // dNdTperHour is supposed to be in [0..1), so useful exp
                // is [0..-15]
                iExp += 15;
                if (iExp < 0)
                        iExp = 0;
                if (iExp > 15)
                        return 0xFFFF;


                return (uint16_t)((iExp << 12u) + (unsigned) scalbnf(normalValue, 12));
                }
        }

void startSendingUplink(void)
{
  TxBuffer_t b;
  LedPattern savedLed = gLed.Set(LedPattern::Measuring);

  b.begin();
  FlagsSensor2 flag;

  flag = FlagsSensor2(0);

  b.put(0x22); /* the flag for this record format */
  uint8_t * const pFlag = b.getp();
  b.put(0x00); /* will be set to the flags */

  // vBat is sent as 5000 * v
  float vBat = gCatena.ReadVbat();
  gCatena.SafePrintf("vBat:    %d mV\n", (int) (vBat * 1000.0f));
  b.putV(vBat);
  flag |= FlagsSensor2::FlagVbat;

  uint32_t bootCount;
  if (gCatena.getBootCount(bootCount))
        {
        b.putBootCountLsb(bootCount);
        flag |= FlagsSensor2::FlagBoot;
        }

  if (fBme)
       {
       Adafruit_BME280::Measurements m = bme.readTemperaturePressureHumidity();
       // temperature is 2 bytes from -0x80.00 to +0x7F.FF degrees C
       // pressure is 2 bytes, hPa * 10.
       // humidity is one byte, where 0 == 0/256 and 0xFF == 255/256.
       gCatena.SafePrintf(
                "BME280:  T: %d P: %d RH: %d\n",
                (int) m.Temperature,
                (int) m.Pressure,
                (int) m.Humidity
                );
       b.putT(m.Temperature);
       b.putP(m.Pressure);
       b.putRH(m.Humidity);

       flag |= FlagsSensor2::FlagTPH;
       }

  if (fLux)
        {
        /* Get a new sensor event */
        uint16_t light;

        light = bh1750.readLightLevel();
        gCatena.SafePrintf("BH1750:  %u lux\n", light);
        b.putLux(light);
        flag |= FlagsSensor2::FlagLux;
        }


  if (hasUltraSonic){
    float measurment = readUltraSonic();
    Serial.println(measurment);
    if (measurment > 0){
      b.put1u((int32_t) (measurment + 0.5));
      flag |= FlagsSensor2(1 << 6);
    }
  }
  *pFlag = uint8_t(flag);
  if (savedLed != LedPattern::Joining)
          gLed.Set(LedPattern::Sending);
  else
          gLed.Set(LedPattern::Joining);

  gLoRaWAN.SendBuffer(b.getbase(), b.getn(), sendBufferDoneCb, NULL);
}

static void
sendBufferDoneCb(
    void *pContext,
    bool fStatus
    )
    {
    osjobcb_t pFn;

    gLed.Set(LedPattern::Settling);
    if (! fStatus)
        {
        gCatena.SafePrintf("send buffer failed\n");
        pFn = txFailedDoneCb;
        }
    else
        {
        pFn = settleDoneCb;
        }
    os_setTimedCallback(
            &sensorJob,
            os_getTime()+sec2osticks(CATCFG_T_SETTLE),
            pFn
            );
    }

static void
txFailedDoneCb(
        osjob_t *pSendJob
        )
        {
        gCatena.SafePrintf("not provisioned, idling\n");
        gLoRaWAN.Shutdown();
        gLed.Set(LedPattern::NotProvisioned);
        }


//
// the following API is added to delay.c, .h in the BSP. It adjust millis()
// forward after a deep sleep.
//
// extern "C" { void adjust_millis_forward(unsigned); };
//
// If you don't have it, check the following commit at github:
// https://github.com/mcci-catena/ArduinoCore-samd/commit/78d8440dbcd29bf5ac659fd65514268c1334f683
//

static void settleDoneCb(
    osjob_t *pSendJob
    )
    {
    uint32_t startTime;
#if defined(ARDUINO_ARCH_SAMD)
    const bool fNoSleep = Serial.dtr() || fHasPower1;
#else
    const bool fNoSleep = true;
#endif

    // if connected to USB, don't sleep
    // ditto if we're monitoring pulses.
    if (fNoSleep)
        {
        gLed.Set(LedPattern::Sleeping);
        os_setTimedCallback(
                &sensorJob,
                os_getTime() + sec2osticks(CATCFG_T_INTERVAL),
                sleepDoneCb
                );
        return;
        }

#if defined(ARDUINO_ARCH_SAMD)
    /* ok... now it's time for a deep sleep */
    gLed.Set(LedPattern::Off);

    startTime = millis();
    gRtc.SetAlarm(CATCFG_T_INTERVAL);
    gRtc.SleepForAlarm(
        gRtc.MATCH_HHMMSS,
        gRtc.SleepMode::IdleCpuAhbApb
        );

    // add the number of ms that we were asleep to the millisecond timer.
    // we don't need extreme accuracy.
    adjust_millis_forward(CATCFG_T_INTERVAL  * 1000);

    /* and now... we're awake again. trigger another measurement */
    sleepDoneCb(pSendJob);
#endif // ARDUINO_ARCH_SAMD
    }

static void sleepDoneCb(
        osjob_t *pJob
        )
        {
        gLed.Set(LedPattern::WarmingUp);

        os_setTimedCallback(
                &sensorJob,
                os_getTime() + sec2osticks(CATCFG_T_WARMUP),
                warmupDoneCb
                );
        }

static void warmupDoneCb(
    osjob_t *pJob
    )
    {
    startSendingUplink();
    }

static float readUltraSonic(){
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  while ( digitalRead(ECHO_PIN) == 0 );

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1);
  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed 
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if ( pulse_width > MAX_DIST ) {
      return -1;
  }
  else {
    return inches;
  }
}

