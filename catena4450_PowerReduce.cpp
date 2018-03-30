/* catena4450_PowerReduce.cpp  Mon Dec 18 2017 14:43:10 mukeshbharath */

/*

Module:  catena4450_PowerReduce.cpp

Function:
  Catena 4450 power reduce code to achieve minimum power
  during the device is idle.

Version:
  V0.1.0  Mon Dec 18 2017 14:43:10 mukeshbharath  Edit level 1

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
   0.1.0  Mon Dec 18 2017 14:43:10  mukeshbharath
  Module created.

*/
#include <time.h>

#include "catena4450_PowerReduce.h"

/****************************************************************************\
|
|		Manifest constants & typedefs.
|
|	This is strictly for private types and constants which will not
|	be exported.
|
\****************************************************************************/

#define EPOCH_TIME_OFF      946684800  // This is 1st January 2000,
					//00:00:00 in epoch time
#define EPOCH_TIME_YEAR_OFF 100        // years since 1900

// Default date & time after reset
#define DEFAULT_YEAR    2000    // 2000..2063
#define DEFAULT_MONTH   1       // 1..12
#define DEFAULT_DAY     1       // 1..31
#define DEFAULT_HOUR    0       // 1..23
#define DEFAULT_MINUTE  0       // 0..59
#define DEFAULT_SECOND  0       // 0..59

/****************************************************************************\
|
|	Read-only data.
|
|	If program is to be ROM-able, these must all be tagged read-only
|	using the ROM storage class; they may be global.
|
\****************************************************************************/

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

voidFuncPtr RTC_callBack1 = NULL;

Catena4450_PR::Catena4450_PR()
{
  _configured = false;
}

void Catena4450_PR::begin(bool resetTime) {
	
	uint16_t tmp_reg = 0;

	/* Turn on the digital interface clock */
	PM->APBAMASK.reg |= PM_APBAMASK_RTC; 

	/* Configuring OSC32K Clock */
	config32kOSC();

	bool validTime = false;
	RTC_MODE2_CLOCK_Type oldTime;

	if ((!resetTime) && (PM->RCAUSE.reg &
		(PM_RCAUSE_SYST | PM_RCAUSE_WDT | PM_RCAUSE_EXT))
		) {
		if (RTC->MODE2.CTRL.reg & RTC_MODE2_CTRL_MODE_CLOCK) {
			validTime = true;
			oldTime.reg = RTC->MODE2.CLOCK.reg;
			}
		}

	/* Setup clock GCLK2 with OSC32K divided by 32 */
	configureClock();

	RTCdisable();

	RTCreset();

	/* set the clock's operating mode */
	tmp_reg |= RTC_MODE2_CTRL_MODE_CLOCK;
	/* set the prescaler value to 1024 for MODE2 */
	tmp_reg |= RTC_MODE2_CTRL_PRESCALER_DIV1024;
	/* disable clear on match */
	tmp_reg &= ~RTC_MODE2_CTRL_MATCHCLR;
	
	/* According to the datasheet RTC_MODE2_CTRL_CLKREP = 0 for 24h */
	tmp_reg &= ~RTC_MODE2_CTRL_CLKREP; // 0x24h time representation

	/* disable continuously mode */
	RTC->MODE2.READREQ.reg &= ~RTC_READREQ_RCONT;

	RTC->MODE2.CTRL.reg = tmp_reg;
	while (RTCisSyncing())
	;

	/* enable RTC interrupt & setting priority*/
	NVIC_EnableIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 0x00);

	/* enabling RTC alarm interrupt */
	RTC->MODE2.INTENSET.reg |= RTC_MODE2_INTENSET_ALARM0;
	/* By default alarm match is off (disabled) */
	RTC->MODE2.Mode2Alarm[0].MASK.bit.SEL = MATCH_OFF;

	while (RTCisSyncing())
	;

	RTCenable();
	RTCresetRemove();

	/* 
	|| If desired and valid, restore the time value,
	|| else use first valid time value
	*/
	if ((!resetTime) && (validTime) && (oldTime.reg != 0L)) {
		RTC->MODE2.CLOCK.reg = oldTime.reg;
		}
	else {
		RTC->MODE2.CLOCK.reg = 				\
		RTC_MODE2_CLOCK_YEAR(DEFAULT_YEAR - 2000) |	\
		RTC_MODE2_CLOCK_MONTH(DEFAULT_MONTH) 		\
		| RTC_MODE2_CLOCK_DAY(DEFAULT_DAY) |		\
		RTC_MODE2_CLOCK_HOUR(DEFAULT_HOUR)		\ 
		| RTC_MODE2_CLOCK_MINUTE(DEFAULT_MINUTE) |	\
		RTC_MODE2_CLOCK_SECOND(DEFAULT_SECOND);
		}
	while (RTCisSyncing())
	;
	
	_configured = true;
	}

/* Blink code */
void Catena4450_PR::blinkLed(uint8_t uCycles, uint32_t uDelay) {
	for (uint8_t uIndex = 0; uIndex < uCycles; ++uIndex) {
		digitalWrite(13, HIGH);   // turn the LED on 
		delay(uDelay);              
		digitalWrite(13, LOW);    // turn the LED off
		delay(uDelay);
		}
	}

void RTC_Handler1(void)
	{
	if (RTC_callBack1 != NULL) {
		RTC_callBack1();
		}

	/* must clear flag at the end */
	RTC->MODE2.INTFLAG.reg = RTC_MODE2_INTFLAG_ALARM0; 
	}

void Catena4450_PR::enableAlarm(Alarm_Match match)
	{
	if (_configured) {
		RTC->MODE2.Mode2Alarm[0].MASK.bit.SEL = match;
		while (RTCisSyncing())
		;
		}
	}

void Catena4450_PR::disableAlarm()
	{
	if (_configured) {
		RTC->MODE2.Mode2Alarm[0].MASK.bit.SEL = 0x00;
		while (RTCisSyncing())
		;
		}
	}

void Catena4450_PR::attachInterrupt(voidFuncPtr callback)
	{
	RTC_callBack1 = callback;
	}

void Catena4450_PR::detachInterrupt()
	{
	RTC_callBack1 = NULL;
	}

void Catena4450_PR::standbyMode()
	{
  if (CATENA4450_SLEEP_MODE)   /* IDLE SLEEP MODE */
    {
    uint8_t IdleMode;
    /*
     * Making the CPU and AHB stop running to try consuming more power
     */
    IdleMode |= 0x2;    /* 0 - Stops CPU; 0x1 - stops CPU & AHB ; 0x2 - stops CPU, AHB & APB */
    //SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Pos;
    PM->SLEEP.reg = IdleMode;
    //PM->AHBMASK.reg &= ~(0x7F);   /* Masking the AHB Clock for various peripheral */
    //PM->APBBMASK.reg &= ~(0x7F);   /* Masking the APB Clock for various peripheral */
    }
  else              /* STANDBY SLEEP MODE */
    {
  	/*
  	|| Entering standby mode when connected
  	|| via the native USB port causes issues.
  	*/
  	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    }
	/*
	|| Puts the system to sleep waiting for interrupt
	|| Executes a device DSB (Data Synchronization Barrier)
	|| instruction to ensure all ongoing memory accesses have
	|| completed, then a WFI (Wait For Interrupt) instruction
	|| to place the device into the sleep mode specified by
	|| ref system_set_sleepmode until woken by an interrupt.
	 */
	__DSB();	/* Data Synchronization Barrier */
	__WFI();	/* Wait For Interrupt */
	}

/*
|| RTC Get Functions
*/

uint8_t Catena4450_PR::getSeconds()
	{
	RTCreadRequest();
	return RTC->MODE2.CLOCK.bit.SECOND;
	}

uint8_t Catena4450_PR::getMinutes()
	{
	RTCreadRequest();
	return RTC->MODE2.CLOCK.bit.MINUTE;
	}

uint8_t Catena4450_PR::getHours()
	{
	RTCreadRequest();
	return RTC->MODE2.CLOCK.bit.HOUR;
	}

uint8_t Catena4450_PR::getDay()
	{
	RTCreadRequest();
	return RTC->MODE2.CLOCK.bit.DAY;
	}

uint8_t Catena4450_PR::getMonth()
	{
	RTCreadRequest();
	return RTC->MODE2.CLOCK.bit.MONTH;
	}

uint8_t Catena4450_PR::getYear()
	{
	RTCreadRequest();
	return RTC->MODE2.CLOCK.bit.YEAR;
	}

uint8_t Catena4450_PR::getAlarmSeconds()
	{
	return RTC->MODE2.Mode2Alarm[0].ALARM.bit.SECOND;
	}

uint8_t Catena4450_PR::getAlarmMinutes()
	{
	return RTC->MODE2.Mode2Alarm[0].ALARM.bit.MINUTE;
	}

uint8_t Catena4450_PR::getAlarmHours()
	{
	return RTC->MODE2.Mode2Alarm[0].ALARM.bit.HOUR;
	}

uint8_t Catena4450_PR::getAlarmDay()
	{
	return RTC->MODE2.Mode2Alarm[0].ALARM.bit.DAY;
	}

uint8_t Catena4450_PR::getAlarmMonth()
	{
	return RTC->MODE2.Mode2Alarm[0].ALARM.bit.MONTH;
	}

uint8_t Catena4450_PR::getAlarmYear()
	{
	return RTC->MODE2.Mode2Alarm[0].ALARM.bit.YEAR;
	}

/*
|| RTC Set Functions
*/

void Catena4450_PR::setSeconds(uint8_t seconds)
	{
	if (_configured) {
		RTC->MODE2.CLOCK.bit.SECOND = seconds;
		while (RTCisSyncing())
		;
		}
	}

void Catena4450_PR::setMinutes(uint8_t minutes)
	{
	if (_configured) {
		RTC->MODE2.CLOCK.bit.MINUTE = minutes;
		while (RTCisSyncing())
		;
		}
	}

void Catena4450_PR::setHours(uint8_t hours)
	{
	if (_configured) {
		RTC->MODE2.CLOCK.bit.HOUR = hours;
		while (RTCisSyncing())
		;
		}
	}

void Catena4450_PR::setTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
	{
	if (_configured) {
		setSeconds(seconds);
		setMinutes(minutes);
		setHours(hours);
		}
	}

void Catena4450_PR::setDay(uint8_t day)
	{
	if (_configured) {
		RTC->MODE2.CLOCK.bit.DAY = day;
		while (RTCisSyncing())
		;
		}
	}

void Catena4450_PR::setMonth(uint8_t month)
	{
	if (_configured) {
		RTC->MODE2.CLOCK.bit.MONTH = month;
		while (RTCisSyncing())
		;
		}
	}

void Catena4450_PR::setYear(uint8_t year)
	{
	if (_configured) {
		RTC->MODE2.CLOCK.bit.YEAR = year;
		while (RTCisSyncing())
		;
		}
	}

void Catena4450_PR::setDate(uint8_t day, uint8_t month, uint8_t year)
	{
	if (_configured) {
		setDay(day);
		setMonth(month);
		setYear(year);
		}
	}

void Catena4450_PR::setAlarmSeconds(uint8_t seconds)
	{
	if (_configured) {
		RTC->MODE2.Mode2Alarm[0].ALARM.bit.SECOND = seconds;
		while (RTCisSyncing())
		;
		}
	}

void Catena4450_PR::setAlarmMinutes(uint8_t minutes)
	{
	if (_configured) {
		RTC->MODE2.Mode2Alarm[0].ALARM.bit.MINUTE = minutes;
		while (RTCisSyncing())
		;
		}
	}

void Catena4450_PR::setAlarmHours(uint8_t hours)
	{
	if (_configured) {
		RTC->MODE2.Mode2Alarm[0].ALARM.bit.HOUR = hours;
		while (RTCisSyncing())
		;
		}
	}

void Catena4450_PR::setAlarmTime(uint8_t hours, uint8_t minutes, uint8_t seconds)
	{
	if (_configured) {
		setAlarmSeconds(seconds);
		setAlarmMinutes(minutes);
		setAlarmHours(hours);
		}
	}

void Catena4450_PR::setAlarmDay(uint8_t day)
	{
	if (_configured) {
		RTC->MODE2.Mode2Alarm[0].ALARM.bit.DAY = day;
		while (RTCisSyncing())
		;
		}
	}

void Catena4450_PR::setAlarmMonth(uint8_t month)
{
if (_configured) {
	RTC->MODE2.Mode2Alarm[0].ALARM.bit.MONTH = month;
	while (RTCisSyncing())
	;
	}
}

void Catena4450_PR::setAlarmYear(uint8_t year)
	{
	if (_configured) {
		RTC->MODE2.Mode2Alarm[0].ALARM.bit.YEAR = year;
		while (RTCisSyncing())
		;
		}
	}

void Catena4450_PR::setAlarmDate(uint8_t day, uint8_t month, uint8_t year)
	{
	if (_configured) {
		setAlarmDay(day);
		setAlarmMonth(month);
		setAlarmYear(year);
		}
	}

uint32_t Catena4450_PR::getEpoch()
	{
	RTCreadRequest();
	RTC_MODE2_CLOCK_Type clockTime;
	clockTime.reg = RTC->MODE2.CLOCK.reg;

	struct tm tm;

	tm.tm_isdst = -1;
	tm.tm_yday = 0;
	tm.tm_wday = 0;
	tm.tm_year = clockTime.bit.YEAR + EPOCH_TIME_YEAR_OFF;
	tm.tm_mon = clockTime.bit.MONTH - 1;
	tm.tm_mday = clockTime.bit.DAY;
	tm.tm_hour = clockTime.bit.HOUR;
	tm.tm_min = clockTime.bit.MINUTE;
	tm.tm_sec = clockTime.bit.SECOND;

	return mktime(&tm);
	}

uint32_t Catena4450_PR::getY2kEpoch()
	{
	return (getEpoch() - EPOCH_TIME_OFF);
	}

void Catena4450_PR::setAlarmEpoch(uint32_t ts)
	{
	if (_configured) {
		if (ts < EPOCH_TIME_OFF) {
			ts = EPOCH_TIME_OFF;
			}

		time_t t = ts;
		struct tm* tmp = gmtime(&t);

		setAlarmDate(					\
			tmp->tm_mday,				\
			tmp->tm_mon + 1,			\
			tmp->tm_year - EPOCH_TIME_YEAR_OFF	\
			);
		setAlarmTime(tmp->tm_hour, tmp->tm_min, tmp->tm_sec);
		}
	}

void Catena4450_PR::setEpoch(uint32_t ts)
	{
	if (_configured) {
		if (ts < EPOCH_TIME_OFF) {
			ts = EPOCH_TIME_OFF;
			}

		time_t t = ts;
		struct tm* tmp = gmtime(&t);

		RTC->MODE2.CLOCK.bit.YEAR = tmp->tm_year - EPOCH_TIME_YEAR_OFF;
		RTC->MODE2.CLOCK.bit.MONTH = tmp->tm_mon + 1;
		RTC->MODE2.CLOCK.bit.DAY = tmp->tm_mday;
		RTC->MODE2.CLOCK.bit.HOUR = tmp->tm_hour;
		RTC->MODE2.CLOCK.bit.MINUTE = tmp->tm_min;
		RTC->MODE2.CLOCK.bit.SECOND = tmp->tm_sec;

		while (RTCisSyncing())
		;
		}
	}

void Catena4450_PR::setY2kEpoch(uint32_t ts)
	{
	if (_configured) {
		setEpoch(ts + EPOCH_TIME_OFF);
		}
	}

/* Attach peripheral clock to 32k oscillator */
void Catena4450_PR::configureClock() {

  /*
   * Enabling GCLK generator 2 and Division factor
   */
	GCLK->GENDIV.reg = GCLK_GENDIV_ID(2)|GCLK_GENDIV_DIV(4);

	while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  /*
   *Configuring Generic clock generator 2 to use OSC32K as input and generate 32KHz 
   */
	GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN			\
			| GCLK_GENCTRL_SRC_XOSC32K		\
			| GCLK_GENCTRL_ID(2)			\
			| GCLK_GENCTRL_DIVSEL			\
			);

	while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
	;

  /*
   * Enabling GCLK for RTC and linking it to Generic clock generator 2 (RTC CLK ID : 0x04)
   */
	GCLK->CLKCTRL.reg = (uint32_t)((GCLK_CLKCTRL_CLKEN		\
				| GCLK_CLKCTRL_GEN_GCLK2		\
				| (RTC_GCLK_ID << GCLK_CLKCTRL_ID_Pos))	\
				);

	while (GCLK->STATUS.bit.SYNCBUSY);
	}

/*
|| Private Utility Functions
*/

/* Configure the 32768Hz Oscillator */
void Catena4450_PR::config32kOSC() 
	{
#if CATENA4450_OSC32K
  //Enable Internal OSC32K
	SYSCTRL->OSC32K.reg = (SYSCTRL_OSC32K_ONDEMAND |	\
			 SYSCTRL_OSC32K_RUNSTDBY |		\
			 SYSCTRL_OSC32K_EN32K |		\
			 /*SYSCTRL_OSC32K_XTALEN |		\*/
			 SYSCTRL_OSC32K_STARTUP(6) |		\
			 SYSCTRL_OSC32K_ENABLE			\
			 );
#elif CATENA4450_XOSC32K
  //Enable External OSC32K
  SYSCTRL->XOSC32K.reg = (SYSCTRL_XOSC32K_ONDEMAND |  \
       SYSCTRL_XOSC32K_RUNSTDBY |    \
       SYSCTRL_XOSC32K_EN32K |   \
       SYSCTRL_XOSC32K_XTALEN |    \
       SYSCTRL_XOSC32K_STARTUP(6) |    \
       SYSCTRL_XOSC32K_ENABLE      \
       );
#elif CATENA4450_XOSC
  //Enable External XOSC
  SYSCTRL->XOSC.reg = (SYSCTRL_XOSC_ONDEMAND |  \
       SYSCTRL_XOSC_RUNSTDBY |    \
       /*SYSCTRL_XOSC_EN32K |   \*/
       SYSCTRL_XOSC_XTALEN |    \
       SYSCTRL_XOSC_STARTUP(0) |    \
       SYSCTRL_XOSC_ENABLE      \
       );
#endif
	}

/* Synchronise the CLOCK register for reading */
inline void Catena4450_PR::RTCreadRequest() {
	if (_configured) {
		RTC->MODE2.READREQ.reg = RTC_READREQ_RREQ;
		while (RTCisSyncing())
		;
		}
	}

/* Wait for the sync in write operations */
inline bool Catena4450_PR::RTCisSyncing()
	{
	return (RTC->MODE2.STATUS.bit.SYNCBUSY);
	}

void Catena4450_PR::RTCdisable()
	{
	RTC->MODE2.CTRL.reg &= ~RTC_MODE2_CTRL_ENABLE; // disable RTC
	while (RTCisSyncing())
	;
	}

void Catena4450_PR::RTCenable()
	{
	RTC->MODE2.CTRL.reg |= RTC_MODE2_CTRL_ENABLE; // enable RTC
	while (RTCisSyncing())
	;
	}

void Catena4450_PR::RTCreset()
	{
	RTC->MODE2.CTRL.reg |= RTC_MODE2_CTRL_SWRST; // software reset
	while (RTCisSyncing())
	;
	}

void Catena4450_PR::RTCresetRemove()
	{
	RTC->MODE2.CTRL.reg &= ~RTC_MODE2_CTRL_SWRST; // software reset remove
	while (RTCisSyncing())
	;
	}
