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

#define CATENA4450_SLEEP_MODE  1
#define CATENA4450_OSC32K      0
#define CATENA4450_XOSC32K     1
#define CATENA4450_XOSC        0

#include "Arduino.h"

typedef void(*voidFuncPtr)(void);

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

	Catena4450_PR();
	void begin(bool resetTime = false);

	void enableAlarm(Alarm_Match match);
	void disableAlarm();

	void attachInterrupt(voidFuncPtr callback);
	void detachInterrupt();
	void blinkLed(uint8_t uCycles, uint32_t uDelay);

	void standbyMode();

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

	void config32kOSC(void);
	void configureClock(void);
	void RTCreadRequest();
	bool RTCisSyncing(void);
	void RTCdisable();
	void RTCenable();
	void RTCreset();
	void RTCresetRemove();
	};

#endif
