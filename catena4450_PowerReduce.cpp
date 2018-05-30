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


	/* Configuring XOSC32K Clock */
	config32kOSC(SYSCTRL_XOSC32K);
	/* Setup clock GCLK2 with XOSC32K divided by 32 */
	configureClock();
	/* Setup clock GCLK2 with XOSC32K divided by 32 */
	configIO();

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

/* Attach peripheral clock to External 32khz oscillator */
void Catena4450_PR::configureClock() {

	//Serial.println("configureClock");
  /*
   * Initializing GCLK selection Id and generator Id flag
   */
	_gclk_gen_id = 0;
	_gclk_selection_id = 0;
  /*
   * Enabling GCLK generator 2 and Division factor
   */
	GCLK->GENDIV.reg = GCLK_GENDIV_ID(2)|GCLK_GENDIV_DIV(4);

	while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
	

  /*
   *Configuring Generic clock generator 2 to use XOSC32K as input and generate 32KHz 
   */
	GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN			\
			| GCLK_GENCTRL_SRC_XOSC32K		\
			| GCLK_GENCTRL_ID(2)			\
			| GCLK_GENCTRL_DIVSEL			\
			);

	while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
	_gclk_gen_id |= 1 << GCLK_CLKCTRL_GEN2;
	;

  /*
   * Enabling GCLK for RTC and linking it to Generic clock generator 2 (RTC CLK ID : 0x04)
   */
	GCLK->CLKCTRL.reg = (uint32_t)((GCLK_CLKCTRL_CLKEN		\
				| GCLK_CLKCTRL_GEN_GCLK2		\
				| (RTC_GCLK_ID << GCLK_CLKCTRL_ID_Pos))	\
				);

	while (GCLK->STATUS.bit.SYNCBUSY);
	_gclk_selection_id |= 1 << GCLK_CLKCTRL_RTC;
	//Serial.println("configureClock done");
	}

/*
|| Private Utility Functions
*/

/* Configure the 32768Hz Oscillator */
void Catena4450_PR::config32kOSC(selectOSC osc) 
	{
	//Serial.println("config32kOSC");
	switch (osc)
		{
		case SYSCTRL_XOSC:
			//Serial.println("configured SYSCTRL_XOSC");
			//Enable External XOSC
			SYSCTRL->XOSC.reg = (SYSCTRL_XOSC_ONDEMAND |	\
				SYSCTRL_XOSC_RUNSTDBY |			\
				SYSCTRL_XOSC_XTALEN |			\
				SYSCTRL_XOSC_STARTUP(6) |		\
				SYSCTRL_XOSC_ENABLE			\
				);
		break;

		case SYSCTRL_XOSC32K:
			//Serial.println("configured SYSCTRL_XOSC32K");
			//Enable External XOSC32K
			SYSCTRL->XOSC32K.reg = (SYSCTRL_XOSC32K_ONDEMAND |	\
				SYSCTRL_XOSC32K_RUNSTDBY |			\
				SYSCTRL_XOSC32K_EN32K |				\
				SYSCTRL_XOSC32K_XTALEN |			\
				SYSCTRL_XOSC32K_STARTUP(6) |			\
				SYSCTRL_XOSC32K_ENABLE				\
				);
		break;

		case SYSCTRL_OSC32K:
			//Enable Internal OSC32K
			//Serial.println("configured SYSCTRL_OSC32K");
			SYSCTRL->OSC32K.reg = (SYSCTRL_OSC32K_ONDEMAND |	\
				 SYSCTRL_OSC32K_RUNSTDBY |			\
				 SYSCTRL_OSC32K_EN32K |				\
				 SYSCTRL_OSC32K_STARTUP(6) |			\
				 SYSCTRL_OSC32K_ENABLE				\
				 );
		break;

		default:
		/* Do Nothing */
		//Serial.println("configured Nothing!");
		break;
		}
	}

/* Make the IO Pins to low */
void Catena4450_PR::configIO(void)
	{
	Port *PORT;	/* IO pin  */
	unsigned int Index1, Index2;

	for (Index1 = 0; Index1 < 2; ++Index1)
		{
		for (Index2 = 0; Index2 < 2; ++Index2)
			{
			PORT->Group[Index1].DIR.reg &= ~(1 << Index2);
			PORT->Group[Index1].DIRSET.reg |= (1 << Index2);
			PORT->Group[Index1].OUT.reg &= ~(1 << Index2);
			
			PORT->Group[Index1].WRCONFIG.reg |=
						PORT_WRCONFIG_PINMASK_Msk;
			PORT->Group[Index1].WRCONFIG.reg |=
						(PORT_WRCONFIG_PINMASK_Msk
						| PORT_WRCONFIG_HWSEL);
			}
		}
	}

/* Select the mode of Sleep IDLE or STANDBY */
void Catena4450_PR::setSleepMode(Sleep_Mode sleep_mode)
	{
	
	//configSysCtrl(DISABLE);

	//configPeripherals(DISABLE);

	switch (sleep_mode)
		{
		case SLEEPMODE_IDLE_0:
		case SLEEPMODE_IDLE_1:
		case SLEEPMODE_IDLE_2:
			SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    			PM->SLEEP.reg = sleep_mode;
    		break;

    		case SLEEPMODE_STANDBY:
    			SCB->SCR |=  SCB_SCR_SLEEPDEEP_Msk;
    		break;

		default:
		/* Do Nothing */
		break;
		}

	enterSleepMode();
	}

void Catena4450_PR::enterSleepMode(void)
	{
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

void Catena4450_PR::configSysCtrl(BOOL enable)
	{
	//Serial.println("configSysCtrl");
	if (enable)
		{
    /* Disabling XOSC */
    SYSCTRL->XOSC.bit.ENABLE = ~SYSCTRL_XOSC_ENABLE;
    SYSCTRL->XOSC.bit.RUNSTDBY = ~SYSCTRL_XOSC_RUNSTDBY;

    /* Disabling OSC32K */
    SYSCTRL->OSC32K.bit.ENABLE = ~SYSCTRL_OSC32K_ENABLE;
    SYSCTRL->OSC32K.bit.EN32K = ~SYSCTRL_OSC32K_EN32K;
    SYSCTRL->OSC32K.bit.RUNSTDBY = ~SYSCTRL_OSC32K_RUNSTDBY;

    /* Disabling OSC8M */
    SYSCTRL->OSC8M.bit.ENABLE = ~SYSCTRL_OSC8M_ENABLE;
    SYSCTRL->OSC8M.bit.RUNSTDBY = ~SYSCTRL_OSC8M_RUNSTDBY;

    /* Disabling DFLLCTRL */
    SYSCTRL->DFLLCTRL.bit.ENABLE = ~SYSCTRL_DFLLCTRL_ENABLE;
    SYSCTRL->DFLLCTRL.bit.RUNSTDBY = ~SYSCTRL_DFLLCTRL_RUNSTDBY;

    /* Disabling BOD33 */
    SYSCTRL->BOD33.bit.ENABLE = ~SYSCTRL_BOD33_ENABLE;
    SYSCTRL->BOD33.bit.RUNSTDBY = ~SYSCTRL_BOD33_RUNSTDBY;

    /* Disabling DPLLCTRLA */
    SYSCTRL->DPLLCTRLA.bit.ENABLE = ~SYSCTRL_DPLLCTRLA_ENABLE;
    SYSCTRL->DPLLCTRLA.bit.RUNSTDBY = ~SYSCTRL_DPLLCTRLA_RUNSTDBY;

    /* Disabling DPLLCTRLA */
    SYSCTRL->DPLLCTRLA.bit.ENABLE = ~SYSCTRL_DPLLCTRLA_ENABLE;
    SYSCTRL->DPLLCTRLA.bit.RUNSTDBY = ~SYSCTRL_DPLLCTRLA_RUNSTDBY;
#if 0
		/* Disabling XOSC */
		SYSCTRL->XOSC.bit.ENABLE &= ~SYSCTRL_XOSC_ENABLE;
		SYSCTRL->XOSC.bit.RUNSTDBY &= ~SYSCTRL_XOSC_RUNSTDBY;

		/* Disabling OSC32K */
		SYSCTRL->OSC32K.bit.ENABLE &= ~SYSCTRL_OSC32K_ENABLE;
		SYSCTRL->OSC32K.bit.EN32K &= ~SYSCTRL_OSC32K_EN32K;
		SYSCTRL->OSC32K.bit.RUNSTDBY &= ~SYSCTRL_OSC32K_RUNSTDBY;

		/* Disabling OSC8M */
		SYSCTRL->OSC8M.bit.ENABLE &= ~SYSCTRL_OSC8M_ENABLE;
		SYSCTRL->OSC8M.bit.RUNSTDBY &= ~SYSCTRL_OSC8M_RUNSTDBY;

		/* Disabling DFLLCTRL */
		SYSCTRL->DFLLCTRL.bit.ENABLE &= ~SYSCTRL_DFLLCTRL_ENABLE;
		SYSCTRL->DFLLCTRL.bit.RUNSTDBY &= ~SYSCTRL_DFLLCTRL_RUNSTDBY;

		/* Disabling BOD33 */
		SYSCTRL->BOD33.bit.ENABLE &= ~SYSCTRL_BOD33_ENABLE;
		SYSCTRL->BOD33.bit.RUNSTDBY &= ~SYSCTRL_BOD33_RUNSTDBY;

		/* Disabling DPLLCTRLA */
		SYSCTRL->DPLLCTRLA.bit.ENABLE &= ~SYSCTRL_DPLLCTRLA_ENABLE;
		SYSCTRL->DPLLCTRLA.bit.RUNSTDBY &= ~SYSCTRL_DPLLCTRLA_RUNSTDBY;

		/* Disabling DPLLCTRLA */
		SYSCTRL->DPLLCTRLA.bit.ENABLE &= ~SYSCTRL_DPLLCTRLA_ENABLE;
		SYSCTRL->DPLLCTRLA.bit.RUNSTDBY &= ~SYSCTRL_DPLLCTRLA_RUNSTDBY;
#endif
		}
	else
		{
		/* Do Nothing */
		}
	//Serial.println("configSysCtrl");
	}

/* Disable the unused peripherals */
void Catena4450_PR::configPeripherals(BOOL enable)
	{
	//Serial.println("configPeripherals");
	/* disabling WDT */
	/*
	WDT->CTRL.bit.ENABLE = enable;
	while(WDT->STATUS.bit.SYNCBUSY);
	*/

	/* disabling EIC */
	/*
	EIC->CTRL.bit.ENABLE = enable;
	while(EIC->STATUS.bit.SYNCBUSY);
	*/

	/* disabling SERCOMx's */
	configSERCOM(SERCOM0, enable);
	configSERCOM(SERCOM1, enable);
	configSERCOM(SERCOM2, enable);
	configSERCOM(SERCOM3, enable);
	configSERCOM(SERCOM4, enable);
	configSERCOM(SERCOM5, enable);

	/* disabling TCx's */
	configTC(TC3, enable);
	configTC(TC4, enable);
	configTC(TC5, enable);
	//configTC(TC6, enable);
	//configTC(TC7, enable);

	/* disabling TCCx's */
	configTCC(TCC0, enable);
	configTCC(TCC1, enable);
	configTCC(TCC2, enable);

	/* disabling ADC */
	configADC(ADC, enable);

	/* disabling AC */
	configAC(AC, enable);

	configDAC(DAC, enable);

	configI2S(I2S, enable);

	configUSB(USB, enable);

	configGCLK(enable);
	
	configPM(PM, enable);
	
	//TCx->COUNT16.CTRLA.bit.ENABLE = 0;
	//PM->AHBMASK.reg &= ~(0x7F);   /* Masking the AHB Clock for various peripheral */
	//PM->APBAMASK.reg &= ~(0x51);   /* Masking the APBA Clock for peripherals EIC WDT PAC0 */
	//PM->APBBMASK.reg &= ~(0x1F);   /* Masking the APB Clock for peripherals DMAC PORT NVMCTRL DSU PAC1 */
	//PM->APBCMASK.reg &= ~(0x3FFFFD);   /* Masking the APB Clock for peripherals AC1 I2S PTC DAC AC ADC TC TCC SERCOM PAC2 */
	//Serial.println("configPeripherals done");
	}

/* Disable the unused SERCOMx's */
void Catena4450_PR::configSERCOM(Sercom *SERCOM, BOOL enable)
	{
	/* disabling SERCOM (USART) */
	if (!enable && SERCOM->USART.CTRLA.bit.ENABLE == SERCOM_USART_CTRLA_ENABLE)
		{
		SERCOM->USART.CTRLA.bit.ENABLE = SERCOM_USART_CTRLA_ENABLE;
		/*
		SERCOM->USART.CTRLA.bit.ENABLE =			\
				enable	? SERCOM_USART_CTRLA_ENABLE	\
					: ~SERCOM_USART_CTRLA_ENABLE;
		*/
		while(SERCOM->USART.SYNCBUSY.bit.ENABLE & 	\
				SERCOM_USART_SYNCBUSY_ENABLE	\
				);
		}

	/* disabling SERCOM (SPI) */
	if (!enable && SERCOM->SPI.CTRLA.bit.ENABLE == SERCOM_SPI_CTRLA_ENABLE)
		{
		SERCOM->SPI.CTRLA.bit.ENABLE = SERCOM_SPI_CTRLA_ENABLE;
		/*
		SERCOM->SPI.CTRLA.bit.ENABLE =				\
				enable	? SERCOM_SPI_CTRLA_ENABLE	\
					: ~SERCOM_SPI_CTRLA_ENABLE;
		*/
		while(SERCOM->SPI.SYNCBUSY.bit.ENABLE & 	\
				SERCOM_SPI_SYNCBUSY_ENABLE	\
				);
		}

	/* disabling SERCOM (I2CM) I2C Master mode */
	if (!enable && SERCOM->I2CM.CTRLA.bit.ENABLE == SERCOM_I2CM_CTRLA_ENABLE)
		{
		SERCOM->I2CM.CTRLA.bit.ENABLE =				\
				enable 	? SERCOM_I2CM_CTRLA_ENABLE	\
					: ~SERCOM_I2CM_CTRLA_ENABLE;
		while(SERCOM->I2CM.SYNCBUSY.bit.ENABLE & 	\
				SERCOM_I2CM_SYNCBUSY_ENABLE	\
				);
		}

	/* disabling SERCOM (I2CM) I2C Slave mode */
	if (!enable && SERCOM->I2CS.CTRLA.bit.ENABLE == SERCOM_I2CS_CTRLA_ENABLE)
		{
		SERCOM->I2CS.CTRLA.bit.ENABLE =				\
				enable	? SERCOM_I2CS_CTRLA_ENABLE	\
					: ~SERCOM_I2CS_CTRLA_ENABLE;
		while(SERCOM->I2CS.SYNCBUSY.bit.ENABLE & 	\
				SERCOM_I2CS_SYNCBUSY_ENABLE	\
				);
		}
	}

/* Disable the unused TCx's */
void Catena4450_PR::configTC(Tc *TC, BOOL enable)
	{
	/* disabling Timer/Counter (COUNT8, COUNT16, COUNT32) */
	TC->COUNT8.CTRLA.bit.ENABLE =
				enable ? TC_CTRLA_ENABLE : ~TC_CTRLA_ENABLE;
	while(TC->COUNT8.STATUS.bit.SYNCBUSY & TC_STATUS_SYNCBUSY);

	TC->COUNT16.CTRLA.bit.ENABLE =
				enable ? TC_CTRLA_ENABLE : ~TC_CTRLA_ENABLE;
	while(TC->COUNT16.STATUS.bit.SYNCBUSY & TC_STATUS_SYNCBUSY);

	TC->COUNT32.CTRLA.bit.ENABLE =
				enable ? TC_CTRLA_ENABLE : ~TC_CTRLA_ENABLE;
	while(TC->COUNT32.STATUS.bit.SYNCBUSY & TC_STATUS_SYNCBUSY);
	}

/* Disable the unused TCCx's */
void Catena4450_PR::configTCC(Tcc *TCC, BOOL enable)
	{
	/* disabling Timer/Counter for Control */
	TCC->CTRLA.bit.ENABLE = enable ? TCC_CTRLA_ENABLE : ~TCC_CTRLA_ENABLE;
	while(TCC->SYNCBUSY.bit.ENABLE & TCC_SYNCBUSY_ENABLE);
	}

/* Disable the ADC (Analog to Digital Comparator) */
void Catena4450_PR::configADC(Adc *ADCreg, BOOL enable)
	{
	/* disabling ADC */
	ADCreg->CTRLA.bit.ENABLE = enable ? ADC_CTRLA_ENABLE : ~ADC_CTRLA_ENABLE;
	}

/* Disable the AC (Analog Comparator) */
void Catena4450_PR::configAC(Ac *ACreg, BOOL enable)
	{
	/* disabling AC */
	ACreg->CTRLA.bit.ENABLE = enable ? AC_CTRLA_ENABLE : ~AC_CTRLA_ENABLE;
	}

/* Disable the DAC (Digital Analog Comparator) */
void Catena4450_PR::configDAC(Dac *DACreg, BOOL enable)
	{
	/* disabling DAC */
	DACreg->CTRLA.bit.ENABLE = enable ? DAC_CTRLA_ENABLE : ~DAC_CTRLA_ENABLE;
	}

/* Disable the I2S (I2C Sound Controller) */
void Catena4450_PR::configI2S(I2s *I2Sreg, BOOL enable)
	{
	/* disabling I2S */
	I2Sreg->CTRLA.bit.ENABLE = enable ? I2S_CTRLA_ENABLE : ~I2S_CTRLA_ENABLE;
	}

/* Disable the USB_HOST (USB Host Mode) */
void Catena4450_PR::configUSB(Usb *USBreg, BOOL enable)
	{
	/* disabling USB_HOST */
	USBreg->HOST.CTRLA.bit.ENABLE = enable ? USB_CTRLA_ENABLE : ~USB_CTRLA_ENABLE;
	}

/* Disable the GCLK for unused sources (USB Host Mode) */
void Catena4450_PR::configGCLK(BOOL enable)
	{
	/* disabling the unused GCLKs  */
	uint8_t Index, Index1;
	for (Index = 0; Index < sizeof (_GCLK_CLKCTRL_GEN_ID); ++Index)
		{
		if (! _gclk_gen_id & (1 << Index))
			{
			/* 
			|| disable the GCLKs which have not been enabled
			*/
			Index1 = 0;
			while (Index1 < sizeof (_GCLK_CLKCTRL_SELETION_ID))
				{
				if (! _gclk_selection_id & (1 << Index1))
					{
					GCLK->CLKCTRL.reg = 
						(uint32_t)((~GCLK_CLKCTRL_CLKEN)	\
						| _GCLK_CLKCTRL_GEN_ID[Index]			\
						| _GCLK_CLKCTRL_SELETION_ID[Index1]		\
						);

					while (GCLK->STATUS.bit.SYNCBUSY);
					}
				++Index1;
				}
			}
		else
			{
			/*
			|| if configured, do nothing and skip to the next GCLK
			*/
			continue;
			}
		}

	for (Index = 0; Index < sizeof (_GCLK_CLKCTRL_GEN_ID) && (! _gclk_genctrl_id & (1 << Index)); ++Index)
		{
		Index1 = 0;
		while (Index1 < sizeof (_GCLK_CLKCTRL_SELETION_ID))
			{
			if (! _gclk_selection_id & (1 << Index1))
				{
				/* 
				|| disable the GCLKs GENCTRL which have not been enabled
				*/
				GCLK->GENCTRL.reg = ((~GCLK_GENCTRL_GENEN)		\
						| _GCLK_CLKCTRL_SELETION_ID[Index1]	\
						| _GCLK_GENCTRL_GEN_ID[Index]		\
						| ~GCLK_GENCTRL_DIVSEL			\
						);

				while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
				}
			++Index1;
			}
		}
	}

/* Disable the Power Manager accessfor unused peripherals */
void Catena4450_PR::configPM(Pm *PMreg, BOOL enable)
	{
	if (!enable)
		{
		/* IDLE */
		PMreg->SLEEP.reg |= PM_SLEEP_IDLE_APB;
		/* AHBMASK */
		PMreg->AHBMASK.reg &= ~(PM_AHBMASK_HPB0 | PM_AHBMASK_HPB1 |		\
					PM_AHBMASK_HPB2 | PM_AHBMASK_DSU |		\
					PM_AHBMASK_NVMCTRL | PM_AHBMASK_DMAC |		\
					PM_AHBMASK_USB);
		/* APBAMASK */
		//PMreg->APBAMASK.reg 
		
		/* APBBMASK */
		PMreg->APBBMASK.reg &= ~(PM_APBBMASK_PAC1 | PM_APBBMASK_DSU |		\
					PM_APBBMASK_NVMCTRL | PM_APBBMASK_PORT |	\
					PM_APBBMASK_DMAC | PM_APBBMASK_USB |		\
					PM_APBBMASK_HMATRIX);
		/* APBCMASK */
		PMreg->APBCMASK.reg &= ~(PM_APBCMASK_PAC2 | PM_APBCMASK_EVSYS |		\
					PM_APBCMASK_SERCOM0 |				\
					PM_APBCMASK_SERCOM1 |				\
					PM_APBCMASK_SERCOM2 |				\
					PM_APBCMASK_SERCOM3 |				\
					PM_APBCMASK_SERCOM4 |				\
					PM_APBCMASK_SERCOM5 |				\
					PM_APBCMASK_TCC0 | PM_APBCMASK_TCC1 |		\
					PM_APBCMASK_TCC2 | PM_APBCMASK_TC3 |		\
					PM_APBCMASK_TC4 | PM_APBCMASK_TC5 |		\
					PM_APBCMASK_TC6 | PM_APBCMASK_TC7 |		\
					PM_APBCMASK_ADC | PM_APBCMASK_AC |		\
					PM_APBCMASK_DAC | PM_APBCMASK_PTC |		\
					PM_APBCMASK_I2S);
		}
	else
		{
		/* IDLE */
		PMreg->SLEEP.reg &= ~PM_SLEEP_IDLE_APB;
		/* AHBMASK */
		PMreg->AHBMASK.reg |= (PM_AHBMASK_HPB0 | PM_AHBMASK_HPB1 |		\
					PM_AHBMASK_HPB2 | PM_AHBMASK_DSU |		\
					PM_AHBMASK_NVMCTRL | PM_AHBMASK_DMAC |		\
					PM_AHBMASK_USB);
		/* APBAMASK */
		//PMreg->APBAMASK.reg 
		
		/* APBBMASK */
		PMreg->APBBMASK.reg |= (PM_APBBMASK_PAC1 | PM_APBBMASK_DSU |		\
					PM_APBBMASK_NVMCTRL | PM_APBBMASK_PORT |	\
					PM_APBBMASK_DMAC | PM_APBBMASK_USB |		\
					PM_APBBMASK_HMATRIX);

		/* APBCMASK */
		PMreg->APBCMASK.reg |= (PM_APBCMASK_PAC2 | PM_APBCMASK_EVSYS |		\
					PM_APBCMASK_SERCOM0 |				\
					PM_APBCMASK_SERCOM1 |				\
					PM_APBCMASK_SERCOM2 |				\
					PM_APBCMASK_SERCOM3 |				\
					PM_APBCMASK_SERCOM4 |				\
					PM_APBCMASK_SERCOM5 |				\
					PM_APBCMASK_TCC0 | PM_APBCMASK_TCC1 |		\
					PM_APBCMASK_TCC2 | PM_APBCMASK_TC3 |		\
					PM_APBCMASK_TC4 | PM_APBCMASK_TC5 |		\
					PM_APBCMASK_TC6 | PM_APBCMASK_TC7 |		\
					PM_APBCMASK_ADC | PM_APBCMASK_AC |		\
					PM_APBCMASK_DAC | PM_APBCMASK_PTC |		\
					PM_APBCMASK_I2S);
		}
	
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
