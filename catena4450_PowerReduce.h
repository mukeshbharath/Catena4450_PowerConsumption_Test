/* catena4450_PowerReduce.h  Fri Dec 15 2017 12:45:16 mukeshbharath */

/*

Module:  catena4450_PowerReduce.h

Function:
  Header that defines the power reduction configuration of
  Catena 4450 using RTC.

Version:
  V0.1.0  Fri Dec 15 2017 12:45:16 mukeshbharath  Edit level 1

Copyright notice:
  This file copyright (C) 2017 by

    MCCI Corporation
    3520 Krums Corners Road
    Ithaca, NY  14850

  An unpublished work.  All rights reserved.

  This file is proprietary information, and may not be disclosed or
  copied without the prior permission of MCCI Corporation.

Author:
  Mukesh Bharath Ramalingam, MCCI Corporation December 2017

Revision history:
   0.1.0  Fri Dec 15 2017 12:45:16  mukeshbharath
  Module created.

*/

#ifndef CATENA4450_POWERREDUCE_H
#define CATENA4450_POWERREDUCE_H

#define CATENA4450_OSC32K      0
#define CATENA4450_XOSC32K     1
#define CATENA4450_XOSC        0

#include "Arduino.h"

typedef void(*voidFuncPtr)(void);

/* ========== Definition for GCLK Source Id ========== */
#define GCLK_CLKCTRL_DFLL48M_REF		0x00
#define GCLK_CLKCTRL_DPLL			0x01
#define GCLK_CLKCTRL_DPLL_32K			0x02
#define GCLK_CLKCTRL_WDT			0x03
#define GCLK_CLKCTRL_RTC			0x04
#define GCLK_CLKCTRL_EIC			0x05
#define GCLK_CLKCTRL_USB			0x06
#define GCLK_CLKCTRL_EVSYS_CHANNEL_0		0x07
#define GCLK_CLKCTRL_EVSYS_CHANNEL_1		0x08
#define GCLK_CLKCTRL_EVSYS_CHANNEL_2		0x09
#define GCLK_CLKCTRL_EVSYS_CHANNEL_3		0x0A
#define GCLK_CLKCTRL_EVSYS_CHANNEL_4		0x0B
#define GCLK_CLKCTRL_EVSYS_CHANNEL_5		0x0C
#define GCLK_CLKCTRL_EVSYS_CHANNEL_6		0x0D
#define GCLK_CLKCTRL_EVSYS_CHANNEL_7		0x0E
#define GCLK_CLKCTRL_EVSYS_CHANNEL_8		0x0F
#define GCLK_CLKCTRL_EVSYS_CHANNEL_9		0x10
#define GCLK_CLKCTRL_EVSYS_CHANNEL_10		0x11
#define GCLK_CLKCTRL_EVSYS_CHANNEL_11		0x12
#define GCLK_CLKCTRL_SERCOMx_SLOW		0x13
#define GCLK_CLKCTRL_SERCOM0_CORE		0x14
#define GCLK_CLKCTRL_SERCOM1_CORE		0x15
#define GCLK_CLKCTRL_SERCOM2_CORE		0x16
#define GCLK_CLKCTRL_SERCOM3_CORE		0x17
#define GCLK_CLKCTRL_SERCOM4_CORE		0x18
#define GCLK_CLKCTRL_SERCOM5_CORE		0x19
#define GCLK_CLKCTRL_TCC0_TCC1			0x1A
#define GCLK_CLKCTRL_TCC2_TC3			0x1B
#define GCLK_CLKCTRL_TC4_TC5			0x1C
#define GCLK_CLKCTRL_TC6_TC7			0x1D
#define GCLK_CLKCTRL_ADC			0x1E
#define GCLK_CLKCTRL_AC_DIG			0x1F
#define GCLK_CLKCTRL_AC_ANA			0x20
#define GCLK_CLKCTRL_DAC			0x21
#define GCLK_CLKCTRL_PTC			0x22
#define GCLK_CLKCTRL_I2S_0			0x23
#define GCLK_CLKCTRL_I2S_1			0x24

/* ========== Definition for GCLK Generator ========== */
#define GCLK_CLKCTRL_GEN0			0x0
#define GCLK_CLKCTRL_GEN1			0x1
#define GCLK_CLKCTRL_GEN2			0x2
#define GCLK_CLKCTRL_GEN3			0x3
#define GCLK_CLKCTRL_GEN4			0x4
#define GCLK_CLKCTRL_GEN5			0x5
#define GCLK_CLKCTRL_GEN6			0x6
#define GCLK_CLKCTRL_GEN7			0x7
#define GCLK_CLKCTRL_GEN8			0x8

/* ========== Definition for GCLK Generator ========== */
#define GCLK_GENCTRL_SRC_XOSC			0x0
#define GCLK_GENCTRL_SRC_GCLKIN			0x1
#define GCLK_GENCTRL_SRC_GCLKGEN1		0x2
#define GCLK_GENCTRL_SRC_OSCULP32K		0x3
#define GCLK_GENCTRL_SRC_OSC32K			0x4
#define GCLK_GENCTRL_SRC_XOSC32K		0x5
#define GCLK_GENCTRL_SRC_OSC8M			0x6
#define GCLK_GENCTRL_SRC_DFLL48M		0x7
#define GCLK_GENCTRL_SRC_FDPLL96M		0x8


class Catena4450_PR {
public:

	enum Alarm_Match: uint8_t // Should we have this enum or just use the identifiers from /component/rtc.h ?
		{
		MATCH_OFF          = RTC_MODE2_MASK_SEL_OFF_Val,          // Never
		MATCH_SS           = RTC_MODE2_MASK_SEL_SS_Val,           // Every Minute
		MATCH_MMSS         = RTC_MODE2_MASK_SEL_MMSS_Val,         // Every Hour
		MATCH_HHMMSS       = RTC_MODE2_MASK_SEL_HHMMSS_Val,       // Every Day
		MATCH_DHHMMSS      = RTC_MODE2_MASK_SEL_DDHHMMSS_Val,     // Every Month
		MATCH_MMDDHHMMSS   = RTC_MODE2_MASK_SEL_MMDDHHMMSS_Val,   // Every Year
		MATCH_YYMMDDHHMMSS = RTC_MODE2_MASK_SEL_YYMMDDHHMMSS_Val  // Once, on a specific date and a specific time
		};

	enum Sleep_Mode {
		/* IDLE 0 sleep mode */
		SLEEPMODE_IDLE_0,
		/* IDLE 1 sleep mode */
		SLEEPMODE_IDLE_1,
		/* IDLE 2 sleep mode */
		SLEEPMODE_IDLE_2,
		/* Standby sleep mode */
		SLEEPMODE_STANDBY,
		};

	enum selectOSC {
		SYSCTRL_XOSC,
		SYSCTRL_XOSC32K,
		SYSCTRL_OSC32K,
		SYSCTRL_OSCULP32K,
		SYSCTRL_OSC8M,
		SYSCTRL_DFLL48M,
		SYSCTRL_FDPLL96M,
		};

	enum BOOL {
		DISABLE,
		ENABLE,
		};

unsigned int
_GCLK_CLKCTRL_SELETION_ID[40] =
	{
	GCLK_CLKCTRL_DFLL48M_REF,
	GCLK_CLKCTRL_DPLL,
	GCLK_CLKCTRL_DPLL_32K,
	GCLK_CLKCTRL_WDT,
	GCLK_CLKCTRL_RTC,
	GCLK_CLKCTRL_EIC,
	GCLK_CLKCTRL_USB,
	GCLK_CLKCTRL_EVSYS_CHANNEL_0,
	GCLK_CLKCTRL_EVSYS_CHANNEL_1,
	GCLK_CLKCTRL_EVSYS_CHANNEL_2,
	GCLK_CLKCTRL_EVSYS_CHANNEL_3,
	GCLK_CLKCTRL_EVSYS_CHANNEL_4,
	GCLK_CLKCTRL_EVSYS_CHANNEL_5,
	GCLK_CLKCTRL_EVSYS_CHANNEL_6,
	GCLK_CLKCTRL_EVSYS_CHANNEL_7,
	GCLK_CLKCTRL_EVSYS_CHANNEL_8,
	GCLK_CLKCTRL_EVSYS_CHANNEL_9,
	GCLK_CLKCTRL_EVSYS_CHANNEL_10,
	GCLK_CLKCTRL_EVSYS_CHANNEL_11,
	GCLK_CLKCTRL_SERCOMx_SLOW,
	GCLK_CLKCTRL_SERCOM0_CORE,
	GCLK_CLKCTRL_SERCOM1_CORE,
	GCLK_CLKCTRL_SERCOM2_CORE,
	GCLK_CLKCTRL_SERCOM3_CORE,
	GCLK_CLKCTRL_SERCOM4_CORE,
	GCLK_CLKCTRL_SERCOM5_CORE,
	GCLK_CLKCTRL_TCC0_TCC1,
	GCLK_CLKCTRL_TCC2_TC3,
	GCLK_CLKCTRL_TC4_TC5,
	GCLK_CLKCTRL_TC6_TC7,
	GCLK_CLKCTRL_ADC,
	GCLK_CLKCTRL_AC_DIG,
	GCLK_CLKCTRL_AC_ANA,
	GCLK_CLKCTRL_DAC,
	GCLK_CLKCTRL_PTC,
	GCLK_CLKCTRL_I2S_0,
	GCLK_CLKCTRL_I2S_1
	};

unsigned int
_GCLK_CLKCTRL_GEN_ID[10] = {
	GCLK_CLKCTRL_GEN0,
	GCLK_CLKCTRL_GEN1,
	GCLK_CLKCTRL_GEN2,
	GCLK_CLKCTRL_GEN3,
	GCLK_CLKCTRL_GEN4,
	GCLK_CLKCTRL_GEN5,
	GCLK_CLKCTRL_GEN6,
	GCLK_CLKCTRL_GEN7,
	GCLK_CLKCTRL_GEN8
	};

unsigned int
_GCLK_GENCTRL_GEN_ID[10] = {
	GCLK_GENCTRL_SRC_XOSC,
	GCLK_GENCTRL_SRC_GCLKIN,
	GCLK_GENCTRL_SRC_GCLKGEN1,
	GCLK_GENCTRL_SRC_OSCULP32K,
	GCLK_GENCTRL_SRC_OSC32K,
	GCLK_GENCTRL_SRC_XOSC32K,
	GCLK_GENCTRL_SRC_OSC8M,
	GCLK_GENCTRL_SRC_DFLL48M,
	GCLK_GENCTRL_SRC_FDPLL96M
	};

	Catena4450_PR();
	void begin(bool resetTime = false);

	void enableAlarm(Alarm_Match match);
	void disableAlarm();

	void attachInterrupt(voidFuncPtr callback);
	void detachInterrupt();
	void blinkLed(uint8_t uCycles, uint32_t uDelay);
	void setSleepMode(Sleep_Mode sleep_mode);

	/* RTC Get Functions */

	uint8_t getSeconds();
	uint8_t getMinutes();
	uint8_t getHours();

	uint8_t getDay();
	uint8_t getMonth();
	uint8_t getYear();

	uint8_t getAlarmSeconds();
	uint8_t getAlarmMinutes();
	uint8_t getAlarmHours();

	uint8_t getAlarmDay();
	uint8_t getAlarmMonth();
	uint8_t getAlarmYear();

	/* RTC Set Functions */

	void setSeconds(uint8_t seconds);
	void setMinutes(uint8_t minutes);
	void setHours(uint8_t hours);
	void setTime(uint8_t hours, uint8_t minutes, uint8_t seconds);

	void setDay(uint8_t day);
	void setMonth(uint8_t month);
	void setYear(uint8_t year);
	void setDate(uint8_t day, uint8_t month, uint8_t year);

	void setAlarmSeconds(uint8_t seconds);
	void setAlarmMinutes(uint8_t minutes);
	void setAlarmHours(uint8_t hours);
	void setAlarmTime(uint8_t hours, uint8_t minutes, uint8_t seconds);

	void setAlarmDay(uint8_t day);
	void setAlarmMonth(uint8_t month);
	void setAlarmYear(uint8_t year);
	void setAlarmDate(uint8_t day, uint8_t month, uint8_t year);

	/* Epoch Functions */

	uint32_t getEpoch();
	uint32_t getY2kEpoch();
	void setEpoch(uint32_t ts);
	void setY2kEpoch(uint32_t ts);
	void setAlarmEpoch(uint32_t ts);

	bool isConfigured() {
		return _configured;
		}

	private:
	bool _configured;

	uint16_t _gclk_gen_id;
	uint16_t _gclk_genctrl_id;
	uint32_t _gclk_selection_id;
	
	void config32kOSC(selectOSC osc);
	void configureClock(void);
	void configIO(void);

	void configPeripherals(BOOL enable);
	void configSysCtrl(BOOL enable);
	void configSERCOM(Sercom *SERCOM, BOOL enable);
	void configTC(Tc *TC, BOOL enable);
	void configTCC(Tcc *TCC, BOOL enable);
	void configADC(Adc *ADCreg, BOOL enable);
	void configAC(Ac *ACreg, BOOL enable);
	void configDAC(Dac *DACreg, BOOL enable);
	void configI2S(I2s *I2Sreg, BOOL enable);
	void configUSB(Usb *USBreg, BOOL enable);
	void configGCLK(BOOL enable);
	void configPM(Pm *PMreg, BOOL enable);

	void enterSleepMode(void);

	void RTCreadRequest();
	bool RTCisSyncing(void);
	void RTCdisable();
	void RTCenable();
	void RTCreset();
	void RTCresetRemove();
	};

#endif
