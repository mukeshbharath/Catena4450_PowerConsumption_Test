/* catena4450_PowerConsumption.ino  Mon Jun 25 2018 16:30:00 mukeshbharath */

/*

Module:  catena4450_PowerReduce.ino

Function:
  Catena 4450 power consumption code, to achieve minimum power
  during the device is in Standby. Used WDT to wake up during Standby.

Version:
  V0.1.0  Mon Jun 25 2018 16:30:00 mukeshbharath  Edit level 1

Copyright notice:
  This file copyright (C) 2018 by

    MCCI Corporation
    3520 Krums Corners Road
    Ithaca, NY  14850

  An unpublished work.  All rights reserved.

  This file is proprietary information, and may not be disclosed or
  copied without the prior permission of MCCI Corporation.

Author:
  Mukesh Bharath Ramalingam, MCCI Corporation June 2018

Revision history:
   0.1.0  Mon Jun 25 2018 16:30:00  mukeshbharath
  Module created.

*/
#include <Adafruit_ASFcore.h>
#include <power.h>

/****************************************************************************\
|
|		Manifest constants & typedefs.
|
|	This is strictly for private types and constants which will not
|	be exported.
|
\****************************************************************************/
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

  unsigned int
  _GCLK_CLKCTRL_SELETION_ID[] =
    {
    //GCLK_CLKCTRL_DFLL48M_REF,
    //GCLK_CLKCTRL_DPLL,
    //GCLK_CLKCTRL_DPLL_32K,
    //GCLK_CLKCTRL_WDT,
    GCLK_CLKCTRL_RTC,
    //GCLK_CLKCTRL_EIC,
    //GCLK_CLKCTRL_USB,
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

boolean _initialized = false; 

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED ON
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED OFF
  delay(1000);   

  sleep(3000); // low-power sleep for 3 seconds

  for (int i=0;i<2;i++) {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED ON
  delay(200);                       // wait for a 200ms
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED OFF
  delay(200);
  } 
}

int enable(int maxPeriodMS, bool isForSleep) {
    /*
    || Enable the watchdog with a period up to the specified max period in
    || milliseconds.
    */

    uint32_t cycles, actualMS;
    uint8_t bits;

    if(!_initialized) _initialize_wdt();

    WDT->CTRL.reg = 0; // Disable watchdog for config
    while(WDT->STATUS.bit.SYNCBUSY);

    if((maxPeriodMS >= 16000) || !maxPeriodMS) {
        cycles = 16384;
        bits   = 0xB;
    } else {
        cycles = (maxPeriodMS * 1024L + 500) / 1000; // ms -> WDT cycles
        if(cycles >= 8192) {
            cycles = 8192;
            bits   = 0xA;
        } else if(cycles >= 4096) {
            cycles = 4096;
            bits   = 0x9;
        } else if(cycles >= 2048) {
            cycles = 2048;
            bits   = 0x8;
        } else if(cycles >= 1024) {
            cycles = 1024;
            bits   = 0x7;
        } else if(cycles >= 512) {
            cycles = 512;
            bits   = 0x6;
        } else if(cycles >= 256) {
            cycles = 256;
            bits   = 0x5;
        } else if(cycles >= 128) {
            cycles = 128;
            bits   = 0x4;
        } else if(cycles >= 64) {
            cycles = 64;
            bits   = 0x3;
        } else if(cycles >= 32) {
            cycles = 32;
            bits   = 0x2;
        } else if(cycles >= 16) {
            cycles = 16;
            bits   = 0x1;
        } else {
            cycles = 8;
            bits   = 0x0;
        }

    }

    if(isForSleep) {
        WDT->INTENSET.bit.EW   = 1;      // Enable early warning interrupt
        WDT->CONFIG.bit.PER    = 0xB;    // Period = max
        WDT->CONFIG.bit.WINDOW = bits;   // Set time of interrupt
        WDT->CTRL.bit.WEN      = 1;      // Enable window mode
        while(WDT->STATUS.bit.SYNCBUSY); // Sync CTRL write
    } else {
        WDT->INTENCLR.bit.EW   = 1;      // Disable early warning interrupt
        WDT->CONFIG.bit.PER    = bits;   // Set period for chip reset
        WDT->CTRL.bit.WEN      = 0;      // Disable window mode
        while(WDT->STATUS.bit.SYNCBUSY); // Sync CTRL write
    }

    actualMS = (cycles * 1000L + 512) / 1024; // WDT cycles -> ms

    reset();                  // Clear watchdog interval
    WDT->CTRL.bit.ENABLE = 1; // Start watchdog now!
    while(WDT->STATUS.bit.SYNCBUSY);

    return actualMS;
}

void reset() {
	/*
	|| Write the watchdog clear key value (0xA5) to the watchdog
	|| clear register to clear the watchdog timer and reset it.
	*/
	WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
	while(WDT->STATUS.bit.SYNCBUSY);
}

void disable() {
    WDT->CTRL.bit.ENABLE = 0;
    while(WDT->STATUS.bit.SYNCBUSY);
}

void WDT_Handler(void) {
	/*
	|| ISR for watchdog early warning, DO NOT RENAME!
	*/
	WDT->CTRL.bit.ENABLE = 0;        // Disable watchdog
	while(WDT->STATUS.bit.SYNCBUSY); // Sync CTRL write
	WDT->INTFLAG.bit.EW  = 1;        // Clear interrupt flag
}

int sleep(int maxPeriodMS) {

	int actualPeriodMS = enable(maxPeriodMS, true); // true = for sleep


  /* Low Power configuration */
  _configure_for_low_power();
	/*
	|| USB_HOST & USB_DEVICE disable
	*/
	USB->HOST.CTRLA.reg &= ~USB_CTRLA_ENABLE;
	USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;

	system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY/*SYSTEM_SLEEPMODE_IDLE_2*/); // Deepest sleep
	system_sleep();
	/* Code resumes here on wake (WDT early warning interrupt) */

	/*
	|| USB_HOST & USB_DEVICE enable
	*/
	USB->HOST.CTRLA.reg |= USB_CTRLA_ENABLE;
	USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;
	
	__configure_for_low_power();

	return actualPeriodMS;
}

void _initialize_wdt() {

	/*
	|| One-time initialization of watchdog timer.
	|| Insights from rickrlh and rbrucemtl in Arduino forum!
	*/

	/* Generic clock generator 2, divisor = 32 (2^(DIV+1)) */
	GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);

	/*
	|| Enable clock generator 2 using low-power 32KHz oscillator.
	|| With /32 divisor above, this yields 1024Hz(ish) clock.
	*/
	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(4) |
			GCLK_GENCTRL_GENEN |
			GCLK_GENCTRL_SRC_OSCULP32K
			/*GCLK_GENCTRL_SRC_XOSC32K*/ |
			GCLK_GENCTRL_DIVSEL;
	while(GCLK->STATUS.bit.SYNCBUSY);
	
	/* WDT clock = clock gen 2 */
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT |
			GCLK_CLKCTRL_CLKEN |
			GCLK_CLKCTRL_GEN_GCLK2;

	/* Enable WDT early-warning interrupt */
	NVIC_DisableIRQ(WDT_IRQn);
	NVIC_ClearPendingIRQ(WDT_IRQn);
	NVIC_SetPriority(WDT_IRQn, 0); // Top priority
	NVIC_EnableIRQ(WDT_IRQn);

}

void _configure_for_low_power() {

	unsigned int Index, Index1;

#if 1
	/*
	|| GCLK
	*/
	for (Index = 0; Index < sizeof (_GCLK_CLKCTRL_SELETION_ID); ++Index)
		{
		GCLK->CLKCTRL.reg &= (~(GCLK_CLKCTRL_CLKEN)	\
				| GCLK_CLKCTRL_GEN_GCLK5		            \
				| _GCLK_CLKCTRL_SELETION_ID[Index]	    \
				);
		while (GCLK->STATUS.bit.SYNCBUSY);
		}

/*
|| XXX: Disabling the GCLK leads to some unknown state that
|| current consumption goes to 11.05mA approx
|| So, Disabling for now....
*/
	GCLK->GENCTRL.reg &= ~(GCLK_GENCTRL_GENEN			\
			| GCLK_GENCTRL_SRC_XOSC			\
			| GCLK_GENCTRL_ID(1)			\
			| GCLK_GENCTRL_DIVSEL			\
			);
	while (GCLK->STATUS.bit.SYNCBUSY);

	GCLK->CLKCTRL.reg &= ~((GCLK_CLKCTRL_CLKEN)	\
		| GCLK_CLKCTRL_GEN_GCLK2		\
		| (RTC_GCLK_ID << GCLK_CLKCTRL_ID_Pos)	\
		);
	while (GCLK->STATUS.bit.SYNCBUSY);
#endif
	/*
	|| I/O Ports
	*/
	configIO();

	/*
	|| SERCOM
	*/
	Sercom *SERCOM[] = {SERCOM0, SERCOM1, SERCOM2, SERCOM3, SERCOM4, SERCOM5};
	for (Index = 0; Index < 6; ++Index)
		{
		/* USART */
		SERCOM[Index]->USART.CTRLA.reg &= ~SERCOM_USART_CTRLA_ENABLE;
		while(SERCOM[Index]->USART.SYNCBUSY.bit.ENABLE & 	\
				SERCOM_USART_SYNCBUSY_ENABLE	\
			);

		/* SPI */
		SERCOM[Index]->SPI.CTRLA.reg &= ~SERCOM_SPI_CTRLA_ENABLE;
		while(SERCOM[Index]->SPI.SYNCBUSY.bit.ENABLE & 	\
				SERCOM_SPI_SYNCBUSY_ENABLE	\
				);

		/* I2CM */
		SERCOM[Index]->I2CM.CTRLA.reg &= ~SERCOM_I2CM_CTRLA_ENABLE;
		while(SERCOM[Index]->I2CM.SYNCBUSY.bit.ENABLE & 	\
				SERCOM_I2CM_SYNCBUSY_ENABLE	\
				);

		/* I2CS */
		SERCOM[Index]->I2CS.CTRLA.reg &= ~SERCOM_I2CS_CTRLA_ENABLE;
		while(SERCOM[Index]->I2CS.SYNCBUSY.bit.ENABLE & 	\
				SERCOM_I2CS_SYNCBUSY_ENABLE	\
				);
		}

	/*
	|| TCC
	*/
	Tcc *TCC[] = {TCC0, TCC1, TCC2};
	for (Index = 0; Index < 3; ++Index)
		{
		/* disabling Timer/Counter for Control */
		TCC[Index]->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
		while(TCC[Index]->SYNCBUSY.bit.ENABLE & TCC_SYNCBUSY_ENABLE);
		}

	/*
	|| TC
	*/
	Tc *TC[] = {TC3, TC4, TC5};
	for (Index = 0; Index < 3; ++Index)
		{
		/* disabling Timer/Counter (COUNT8, COUNT16, COUNT32) */
		TC[Index]->COUNT8.CTRLA.reg &= ~TC_CTRLA_ENABLE;
		while(TC[Index]->COUNT8.STATUS.bit.SYNCBUSY & TC_STATUS_SYNCBUSY);

		TC[Index]->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
		while(TC[Index]->COUNT16.STATUS.bit.SYNCBUSY & TC_STATUS_SYNCBUSY);

		TC[Index]->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;
		while(TC[Index]->COUNT32.STATUS.bit.SYNCBUSY & TC_STATUS_SYNCBUSY);
		}

	/*
	|| ADC
	*/
	ADC->CTRLA.reg &= ~ADC_CTRLA_ENABLE;

	/*
	|| AC
	*/
	AC->CTRLA.reg &= ~AC_CTRLA_ENABLE;

	/*
	|| DAC
	*/
	DAC->CTRLA.reg &= ~DAC_CTRLA_ENABLE;

	/*
	|| I2S
	*/
	I2S->CTRLA.reg &= ~I2S_CTRLA_ENABLE;
#if 1
	/*
	|| USB_HOST & USB_DEVICE
	*/
	//USB->HOST.CTRLA.reg &= ~USB_CTRLA_ENABLE;
	//USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;

	/*
	|| RTC
	*/
	RTC->MODE0.CTRL.reg &= ~RTC_MODE0_CTRL_ENABLE;
	while(RTC->MODE0.STATUS.bit.SYNCBUSY & RTC_STATUS_SYNCBUSY);

	RTC->MODE1.CTRL.reg &= ~RTC_MODE1_CTRL_ENABLE;
	while(RTC->MODE1.STATUS.bit.SYNCBUSY & RTC_STATUS_SYNCBUSY);

	RTC->MODE2.CTRL.reg &= ~RTC_MODE2_CTRL_ENABLE;
	while(RTC->MODE2.STATUS.bit.SYNCBUSY & RTC_STATUS_SYNCBUSY);
#endif
	/*
	|| SYSCTRL
	*/
	/* Disabling XOSC */
	SYSCTRL->XOSC.reg &= ~(SYSCTRL_XOSC_ENABLE |
	  SYSCTRL_XOSC_RUNSTDBY);

  /* Disabling OSC32K */
  SYSCTRL->XOSC32K.reg &= ~(SYSCTRL_XOSC32K_ENABLE |
    SYSCTRL_XOSC32K_EN32K |
    SYSCTRL_XOSC32K_RUNSTDBY);

	/* Disabling OSC32K */
	SYSCTRL->OSC32K.reg &= ~(SYSCTRL_OSC32K_ENABLE |
	  SYSCTRL_OSC32K_EN32K |
	  SYSCTRL_OSC32K_RUNSTDBY);

	/* Disabling OSC8M */
	SYSCTRL->OSC8M.reg &= ~(SYSCTRL_OSC8M_ENABLE |
	  SYSCTRL_OSC8M_RUNSTDBY);

	/* Disabling DFLLCTRL */
	SYSCTRL->DFLLCTRL.reg &= ~(SYSCTRL_DFLLCTRL_ENABLE |
	  SYSCTRL_DFLLCTRL_RUNSTDBY);

	/* Disabling BOD33 */
	SYSCTRL->BOD33.reg &= ~(SYSCTRL_BOD33_ENABLE |
	  SYSCTRL_BOD33_RUNSTDBY);

	/* Disabling DPLLCTRLA */
	SYSCTRL->DPLLCTRLA.reg &= ~(SYSCTRL_DPLLCTRLA_ENABLE |
	  SYSCTRL_DPLLCTRLA_RUNSTDBY);

	/* Disabling DPLLCTRLA */
	SYSCTRL->DPLLCTRLA.reg &= ~(SYSCTRL_DPLLCTRLA_ENABLE |
	    SYSCTRL_DPLLCTRLA_RUNSTDBY);

	/*
	|| EVSYS
	*/
	EVSYS->CTRL.reg &= ~EVSYS_CTRL_GCLKREQ;

	/*
	|| EIC (External Interrupt COntroller)
	*/
	/*
	EIC->CTRL.reg &= ~(EIC_CTRL_ENABLE);
	while(EIC->STATUS.reg & EIC_STATUS_SYNCBUSY);
	*/

	/*
	|| PM
	*/
	/* IDLE */
	PM->SLEEP.reg |= PM_SLEEP_IDLE_APB;
	/* AHBMASK */
	PM->AHBMASK.reg &= ~(PM_AHBMASK_HPB0 | PM_AHBMASK_HPB1 |		\
				PM_AHBMASK_HPB2 | PM_AHBMASK_DSU |		\
				PM_AHBMASK_NVMCTRL | PM_AHBMASK_DMAC |		\
				PM_AHBMASK_USB);
	/* APBAMASK */
	PM->APBAMASK.reg &= ~(PM_APBAMASK_PAC0 | PM_APBAMASK_RTC |		\
				PM_APBAMASK_SYSCTRL | PM_APBAMASK_GCLK |	\
				PM_APBAMASK_PM | PM_APBAMASK_EIC);

#if 1
  /* APBBMASK */
	PM->APBBMASK.reg &= ~(PM_APBBMASK_PAC1 | PM_APBBMASK_DSU |		\
				PM_APBBMASK_NVMCTRL | PM_APBBMASK_PORT |	\
				PM_APBBMASK_DMAC | PM_APBBMASK_USB |		\
				PM_APBBMASK_HMATRIX);
	/* APBCMASK */
	PM->APBCMASK.reg &= ~(PM_APBCMASK_PAC2 | PM_APBCMASK_EVSYS |		\
				PM_APBCMASK_SERCOM0 |				\
				PM_APBCMASK_SERCOM1 |				\
				PM_APBCMASK_SERCOM2 |				\
				PM_APBCMASK_SERCOM3 |				\
				PM_APBCMASK_SERCOM4 |				\
				PM_APBCMASK_SERCOM5 |				\
				PM_APBCMASK_TCC0 | PM_APBCMASK_TCC1 |		\
				PM_APBCMASK_TCC2 | PM_APBCMASK_TC3 |		\
				PM_APBCMASK_TC4 | PM_APBCMASK_TC5 |		\
        PM_APBCMASK_TC6 | PM_APBCMASK_TC7 |   \
				PM_APBCMASK_ADC | PM_APBCMASK_AC |		\
				PM_APBCMASK_DAC | PM_APBCMASK_PTC |		\
				PM_APBCMASK_I2S);
#endif
	/*
	|| NVMCTRL
	*/
	NVMCTRL->CTRLB.bit.SLEEPPRM &= ~NVMCTRL_CTRLB_SLEEPPRM_WAKEONACCESS;

	/*
	|| DMAC
	*/
	DMAC->CHCTRLA.bit.ENABLE &= ~DMAC_CHCTRLA_ENABLE;
	DMAC->CTRL.bit.DMAENABLE &= ~DMAC_CTRL_DMAENABLE;

	_initialized = true;
	}

void configIO(void)
	{
	unsigned int Index, Index1;

	for (Index = 0; Index < 2; ++Index)
		{

		PORT->Group[Index].DIR.reg &= ~(0xFFFFFFFF);
		PORT->Group[Index].DIRSET.reg |= (0xFFFFFFFF);
		PORT->Group[Index].OUT.reg &= ~(0xFFFFFFFF);

		for (Index1 = 0; Index1 < 32; ++Index1)
			{
			PORT->Group[Index].PINCFG[Index].bit.INEN &= ~PORT_PINCFG_INEN;
			PORT->Group[Index].PINCFG[Index].bit.PULLEN &= ~PORT_PINCFG_PULLEN;
			}

		PORT->Group[Index].WRCONFIG.reg |=
					PORT_WRCONFIG_PINMASK_Msk;
		PORT->Group[Index].WRCONFIG.reg |=
					(PORT_WRCONFIG_PINMASK_Msk
					| PORT_WRCONFIG_HWSEL);
		}
	}

void __configure_for_low_power() {
	unsigned int Index, Index1;

  /*
  || GCLK
  */
  for (Index = 0; Index < sizeof (_GCLK_CLKCTRL_SELETION_ID); ++Index)
    {
    GCLK->CLKCTRL.reg &= (~(GCLK_CLKCTRL_CLKEN) \
        | GCLK_CLKCTRL_GEN_GCLK5                \
        | _GCLK_CLKCTRL_SELETION_ID[Index]      \
        );
    while (GCLK->STATUS.bit.SYNCBUSY);
    }

#if 0
	/*
	|| XXX: Disabling the GCLK leads to some unknown state that
	|| current consumption goes to 11.05mA approx
	|| So, Disabling for now....
	*/
	/*
	|| GCLK
	*/
	GCLK->GENCTRL.reg |= (GCLK_GENCTRL_GENEN			\
			| GCLK_GENCTRL_SRC_XOSC			\
			| GCLK_GENCTRL_ID(1)			\
			| GCLK_GENCTRL_DIVSEL			\
			);
	while (GCLK->STATUS.bit.SYNCBUSY);

	GCLK->CLKCTRL.reg |= ((GCLK_CLKCTRL_CLKEN)	\
		| GCLK_CLKCTRL_GEN_GCLK2		\
		| (RTC_GCLK_ID << GCLK_CLKCTRL_ID_Pos)	\
		);
	while (GCLK->STATUS.bit.SYNCBUSY);
#endif
	/*
	|| I/O Ports
	*/
	_configIO();

	/*
	|| SERCOM
	*/
	Sercom *SERCOM[] = {SERCOM0, SERCOM1, SERCOM2, SERCOM3, SERCOM4, SERCOM5};
	for (Index = 0; Index < 6; ++Index)
		{
		/* USART */
		SERCOM[Index]->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
		while(SERCOM[Index]->USART.SYNCBUSY.bit.ENABLE & 	\
				SERCOM_USART_SYNCBUSY_ENABLE	\
			);

		/* SPI */
		SERCOM[Index]->SPI.CTRLA.reg |= SERCOM_SPI_CTRLA_ENABLE;
		while(SERCOM[Index]->SPI.SYNCBUSY.bit.ENABLE & 	\
				SERCOM_SPI_SYNCBUSY_ENABLE	\
				);

		/* I2CM */
		SERCOM[Index]->I2CM.CTRLA.reg |= SERCOM_I2CM_CTRLA_ENABLE;
		while(SERCOM[Index]->I2CM.SYNCBUSY.bit.ENABLE & 	\
				SERCOM_I2CM_SYNCBUSY_ENABLE	\
				);

		/* I2CS */
		SERCOM[Index]->I2CS.CTRLA.reg |= SERCOM_I2CS_CTRLA_ENABLE;
		while(SERCOM[Index]->I2CS.SYNCBUSY.bit.ENABLE & 	\
				SERCOM_I2CS_SYNCBUSY_ENABLE	\
				);
		}

	/*
	|| TCC
	*/
	Tcc *TCC[] = {TCC0, TCC1, TCC2};
	for (Index = 0; Index < 3; ++Index)
		{
		/* disabling Timer/Counter for Control */
		TCC[Index]->CTRLA.reg |= TCC_CTRLA_ENABLE;
		while(TCC[Index]->SYNCBUSY.bit.ENABLE & TCC_SYNCBUSY_ENABLE);
		}

	/*
	|| TC
	*/
	Tc *TC[] = {TC3, TC4, TC5};
	for (Index = 0; Index < 3; ++Index)
		{
		/* disabling Timer/Counter (COUNT8, COUNT16, COUNT32) */
		TC[Index]->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE;
		while(TC[Index]->COUNT8.STATUS.bit.SYNCBUSY & TC_STATUS_SYNCBUSY);

		TC[Index]->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
		while(TC[Index]->COUNT16.STATUS.bit.SYNCBUSY & TC_STATUS_SYNCBUSY);

		TC[Index]->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
		while(TC[Index]->COUNT32.STATUS.bit.SYNCBUSY & TC_STATUS_SYNCBUSY);
		}

	/*
	|| ADC
	*/
	ADC->CTRLA.reg |= ADC_CTRLA_ENABLE;

	/*
	|| AC
	*/
	AC->CTRLA.reg |= AC_CTRLA_ENABLE;

	/*
	|| DAC
	*/
	DAC->CTRLA.reg |= DAC_CTRLA_ENABLE;

	/*
	|| I2S
	*/
	I2S->CTRLA.reg |= I2S_CTRLA_ENABLE;
#if 1
	/*
	|| USB_HOST & USB_DEVICE
	*/
	//USB->HOST.CTRLA.reg |= USB_CTRLA_ENABLE;
	//USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;

	/*
	|| RTC
	*/
	RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_ENABLE;
	while(RTC->MODE0.STATUS.bit.SYNCBUSY & RTC_STATUS_SYNCBUSY);

	RTC->MODE1.CTRL.reg |= RTC_MODE1_CTRL_ENABLE;
	while(RTC->MODE1.STATUS.bit.SYNCBUSY & RTC_STATUS_SYNCBUSY);

	RTC->MODE2.CTRL.reg |= RTC_MODE2_CTRL_ENABLE;
	while(RTC->MODE2.STATUS.bit.SYNCBUSY & RTC_STATUS_SYNCBUSY);
#endif
	/*
	|| SYSCTRL
	*/
	/* Disabling XOSC */
	SYSCTRL->XOSC.reg |= (SYSCTRL_XOSC_ENABLE |
	  SYSCTRL_XOSC_RUNSTDBY);

  /* Disabling OSC32K */
  SYSCTRL->XOSC32K.reg |= (SYSCTRL_XOSC32K_ENABLE |
    SYSCTRL_XOSC32K_EN32K |
    SYSCTRL_XOSC32K_RUNSTDBY);

	/* Disabling OSC32K */
	SYSCTRL->OSC32K.reg |= (SYSCTRL_OSC32K_ENABLE |
	  SYSCTRL_OSC32K_EN32K |
	  SYSCTRL_OSC32K_RUNSTDBY);

	/* Disabling OSC8M */
	SYSCTRL->OSC8M.reg |= (SYSCTRL_OSC8M_ENABLE |
	  SYSCTRL_OSC8M_RUNSTDBY);

	/* Disabling DFLLCTRL */
	SYSCTRL->DFLLCTRL.reg |= (SYSCTRL_DFLLCTRL_ENABLE |
	  SYSCTRL_DFLLCTRL_RUNSTDBY);

	/* Disabling BOD33 */
	SYSCTRL->BOD33.reg |= (SYSCTRL_BOD33_ENABLE |
	  SYSCTRL_BOD33_RUNSTDBY);

	/* Disabling DPLLCTRLA */
	SYSCTRL->DPLLCTRLA.reg |= (SYSCTRL_DPLLCTRLA_ENABLE |
	  SYSCTRL_DPLLCTRLA_RUNSTDBY);

	/* Disabling DPLLCTRLA */
	SYSCTRL->DPLLCTRLA.reg |= (SYSCTRL_DPLLCTRLA_ENABLE |
	    SYSCTRL_DPLLCTRLA_RUNSTDBY);

	/*
	|| EVSYS
	*/
	EVSYS->CTRL.reg |= EVSYS_CTRL_GCLKREQ;

	/*
	|| EIC (External Interrupt COntroller)
	*/
	/*
	EIC->CTRL.reg |= (EIC_CTRL_ENABLE);
	while(EIC->STATUS.reg & EIC_STATUS_SYNCBUSY);
	*/

	/*
	|| PM
	*/
	/* IDLE */
	PM->SLEEP.reg |= PM_SLEEP_IDLE_APB;
	/* AHBMASK */
	PM->AHBMASK.reg |= (PM_AHBMASK_HPB0 | PM_AHBMASK_HPB1 |		\
				PM_AHBMASK_HPB2 | PM_AHBMASK_DSU |		\
				PM_AHBMASK_NVMCTRL | PM_AHBMASK_DMAC |		\
				PM_AHBMASK_USB);
	/* APBAMASK */
	PM->APBAMASK.reg |= (PM_APBAMASK_PAC0 | PM_APBAMASK_RTC |		\
				PM_APBAMASK_SYSCTRL | PM_APBAMASK_GCLK |	\
				PM_APBAMASK_PM | PM_APBAMASK_EIC);

	/* APBBMASK */
	PM->APBBMASK.reg |= (PM_APBBMASK_PAC1 | PM_APBBMASK_DSU |		\
				PM_APBBMASK_NVMCTRL | PM_APBBMASK_PORT |	\
				PM_APBBMASK_DMAC | PM_APBBMASK_USB |		\
				PM_APBBMASK_HMATRIX);
	/* APBCMASK */
	PM->APBCMASK.reg |= (PM_APBCMASK_PAC2 | PM_APBCMASK_EVSYS |		\
				PM_APBCMASK_SERCOM0 |				\
				PM_APBCMASK_SERCOM1 |				\
				PM_APBCMASK_SERCOM2 |				\
				PM_APBCMASK_SERCOM3 |				\
				PM_APBCMASK_SERCOM4 |				\
				PM_APBCMASK_SERCOM5 |				\
				PM_APBCMASK_TCC0 | PM_APBCMASK_TCC1 |		\
				PM_APBCMASK_TCC2 | PM_APBCMASK_TC3 |		\
				PM_APBCMASK_TC4 | PM_APBCMASK_TC5 |		\
        PM_APBCMASK_TC6 | PM_APBCMASK_TC7 |   \
				PM_APBCMASK_ADC | PM_APBCMASK_AC |		\
				PM_APBCMASK_DAC | PM_APBCMASK_PTC |		\
				PM_APBCMASK_I2S);

	/*
	|| NVMCTRL
	*/
	NVMCTRL->CTRLB.bit.SLEEPPRM |= NVMCTRL_CTRLB_SLEEPPRM_WAKEONACCESS;

	/*
	|| DMAC
	*/
	DMAC->CHCTRLA.bit.ENABLE |= DMAC_CHCTRLA_ENABLE;
	DMAC->CTRL.bit.DMAENABLE |= DMAC_CTRL_DMAENABLE;

  _initialized = false;
	}


void _configIO(void)
	{
	unsigned int Index, Index1;

	for (Index = 0; Index < 2; ++Index)
		{

		PORT->Group[Index].DIR.reg |= (0xFFFFFFFF);
		PORT->Group[Index].DIRSET.reg |= (0xFFFFFFFF);
		PORT->Group[Index].OUT.reg |= (0xFFFFFFFF);

		for (Index1 = 0; Index1 < 32; ++Index1)
			{
			PORT->Group[Index].PINCFG[Index].bit.INEN |= PORT_PINCFG_INEN;
			PORT->Group[Index].PINCFG[Index].bit.PULLEN |= PORT_PINCFG_PULLEN;
			}

		PORT->Group[Index].WRCONFIG.reg |=
					PORT_WRCONFIG_PINMASK_Msk;
		PORT->Group[Index].WRCONFIG.reg |=
					(PORT_WRCONFIG_PINMASK_Msk
					| PORT_WRCONFIG_HWSEL);
		}
	}
