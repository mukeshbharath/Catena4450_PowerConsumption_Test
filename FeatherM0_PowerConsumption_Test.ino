#include <Adafruit_ASFcore.h>
#include <power.h>
#include <ZeroRegs.h>

#define MCCI_LOW_POWER

#ifdef MCCI_LOW_POWER

ZeroRegOptions opts = { Serial, false };
#define MAX_GCLK_ID 0x25
#define WRITE8(x,y) *((uint8_t*)&(x)) = uint8_t(y)

void configIO(void);
void configurePM(void);
void disableClock(int gclkid);
void disableClockGenerator(int genid);
void lowPowerPins1(uint16_t pinStart);
/* extern bool system_gclk_gen_is_enabled(
   const uint8_t generator); */
#endif

boolean _initialized = false;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  /* printZeroRegs(opts); */
}

void loop() {
  // put your main code here, to run repeatedly:

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);

  sleep(3000); // low-power sleep for 3 seconds

  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(2000);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(2000);
  }

}

int enable(int maxPeriodMS, bool isForSleep) {
  // Enable the watchdog with a period up to the specified max period in
  // milliseconds.

  // Review the watchdog section from the SAMD21 datasheet section 17:
  //   http://www.atmel.com/images/atmel-42181-sam-d21_datasheet.pdf

  int     cycles, actualMS;
  uint8_t bits;

  if (!_initialized) _initialize_wdt();

  WDT->CTRL.reg = 0; // Disable watchdog for config
  while (WDT->STATUS.bit.SYNCBUSY);

  if ((maxPeriodMS >= 16000) || !maxPeriodMS) {
    cycles = 16384;
    bits   = 0xB;
  } else {
    cycles = (maxPeriodMS * 1024L + 500) / 1000; // ms -> WDT cycles
    if (cycles >= 8192) {
      cycles = 8192;
      bits   = 0xA;
    } else if (cycles >= 4096) {
      cycles = 4096;
      bits   = 0x9;
    } else if (cycles >= 2048) {
      cycles = 2048;
      bits   = 0x8;
    } else if (cycles >= 1024) {
      cycles = 1024;
      bits   = 0x7;
    } else if (cycles >= 512) {
      cycles = 512;
      bits   = 0x6;
    } else if (cycles >= 256) {
      cycles = 256;
      bits   = 0x5;
    } else if (cycles >= 128) {
      cycles = 128;
      bits   = 0x4;
    } else if (cycles >= 64) {
      cycles = 64;
      bits   = 0x3;
    } else if (cycles >= 32) {
      cycles = 32;
      bits   = 0x2;
    } else if (cycles >= 16) {
      cycles = 16;
      bits   = 0x1;
    } else {
      cycles = 8;
      bits   = 0x0;
    }
  }

  if (isForSleep) {
    WDT->INTENSET.bit.EW   = 1;      // Enable early warning interrupt
    WDT->CONFIG.bit.PER    = 0xB;    // Period = max
    WDT->CONFIG.bit.WINDOW = bits;   // Set time of interrupt
    WDT->CTRL.bit.WEN      = 1;      // Enable window mode
    while (WDT->STATUS.bit.SYNCBUSY); // Sync CTRL write
  } else {
    WDT->INTENCLR.bit.EW   = 1;      // Disable early warning interrupt
    WDT->CONFIG.bit.PER    = bits;   // Set period for chip reset
    WDT->CTRL.bit.WEN      = 0;      // Disable window mode
    while (WDT->STATUS.bit.SYNCBUSY); // Sync CTRL write
  }

  actualMS = (cycles * 1000L + 512) / 1024; // WDT cycles -> ms

  reset();                  // Clear watchdog interval
  WDT->CTRL.bit.ENABLE = 1; // Start watchdog now!
  while (WDT->STATUS.bit.SYNCBUSY);

  return actualMS;
}

void reset() {
  // Write the watchdog clear key value (0xA5) to the watchdog
  // clear register to clear the watchdog timer and reset it.
  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
  while (WDT->STATUS.bit.SYNCBUSY);
}

void disable() {
  WDT->CTRL.bit.ENABLE = 0;
  while (WDT->STATUS.bit.SYNCBUSY);
}

void WDT_Handler(void) {
  // ISR for watchdog early warning, DO NOT RENAME!
  WDT->CTRL.bit.ENABLE = 0;        // Disable watchdog
  while (WDT->STATUS.bit.SYNCBUSY); // Sync CTRL write
  WDT->INTFLAG.bit.EW  = 1;        // Clear interrupt flag
}

int sleep(int maxPeriodMS) {

  int actualPeriodMS = enable(maxPeriodMS, true); // true = for sleep

#ifdef MCCI_LOW_POWER

  for (int gclk_id = 0; gclk_id < MAX_GCLK_ID; ++gclk_id)
  {
    if (gclk_id == 0x3)
      continue;
    else
      GCLK->CLKCTRL.reg = ((gclk_id << GCLK_CLKCTRL_ID_Pos) | GCLK_CLKCTRL_GEN_GCLK7);

    disableClock(gclk_id);
  }

  configIO();
  configurePM();

  /* printZeroRegs(opts); */
  lowPowerPins1(0);

  SERCOM4->USART.CTRLA.bit.ENABLE = 0;
  while (SERCOM4->USART.SYNCBUSY.bit.ENABLE);

  SERCOM4->SPI.CTRLA.bit.ENABLE = 0;
  while (SERCOM4->SPI.SYNCBUSY.bit.ENABLE);

  SERCOM3->I2CM.CTRLA.bit.ENABLE = 0;
  while (SERCOM3->I2CM.SYNCBUSY.bit.ENABLE);

  SERCOM3->I2CS.CTRLA.bit.ENABLE = 0;
  while (SERCOM3->I2CS.SYNCBUSY.bit.ENABLE);

  /*
  SERCOM0->USART.CTRLA.bit.ENABLE=0;
  SERCOM1->USART.CTRLA.bit.ENABLE=0;
  SERCOM2->USART.CTRLA.bit.ENABLE=0;
  SERCOM3->USART.CTRLA.bit.ENABLE=0;
  SERCOM4->USART.CTRLA.bit.ENABLE=0;
  SERCOM5->USART.CTRLA.bit.ENABLE=0;
  */
  
  I2S->CTRLA.bit.ENABLE=0;
  
  ADC->CTRLA.bit.ENABLE=0;
  
  DAC->CTRLA.bit.ENABLE=0;
  
  AC->CTRLA.bit.ENABLE=0;
  
  TCC0->CTRLA.bit.ENABLE=0;
  TCC1->CTRLA.bit.ENABLE=0;
  TCC2->CTRLA.bit.ENABLE=0;
  
  RTC->MODE0.CTRL.bit.ENABLE=0;
  RTC->MODE1.CTRL.bit.ENABLE=0;
  RTC->MODE2.CTRL.bit.ENABLE=0;

  SYSCTRL->BOD33.reg = 0;
  NVMCTRL->CTRLB.bit.CACHEDIS = 1;
  //Readmode settings : 0-3
  NVMCTRL->CTRLB.bit.READMODE = 1;
  NVMCTRL->CTRLB.bit.SLEEPPRM = 0;

  /* disableClockGenerator(0); */
  disableClockGenerator(1);
  disableClockGenerator(3);
  disableClockGenerator(4);
  disableClockGenerator(5);
  disableClockGenerator(6);

  /* gclkGenEnableCheck(); */
#endif
  system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY); // Deepest sleep
  /* system_sleep(); */
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  USBDevice.detach();
  __DSB(); // Optimization barrier - wait for completion of prior instructions
  __WFI(); // Wait For Interrupt call
  // Code resumes here on wake (WDT early warning interrupt)

#ifdef MCCI_LOW_POWER
  USBDevice.attach();
  //__WFI();
#endif
  return actualPeriodMS;
}

void _initialize_wdt() {

  // One-time initialization of watchdog timer.
  // Insights from rickrlh and rbrucemtl in Arduino forum!

  // Generic clock generator 2, divisor = 32 (2^(DIV+1))
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);
  // Enable clock generator 2 using low-power 32KHz oscillator.
  // With /32 divisor above, this yields 1024Hz(ish) clock.
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) |
                      GCLK_GENCTRL_GENEN |
                      GCLK_GENCTRL_SRC_OSCULP32K |
                      GCLK_GENCTRL_DIVSEL;
  while (GCLK->STATUS.bit.SYNCBUSY);
  // WDT clock = clock gen 2
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT |
                      GCLK_CLKCTRL_CLKEN |
                      GCLK_CLKCTRL_GEN_GCLK2;

  // Enable WDT early-warning interrupt
  NVIC_DisableIRQ(WDT_IRQn);
  NVIC_ClearPendingIRQ(WDT_IRQn);
  NVIC_SetPriority(WDT_IRQn, 0); // Top priority
  NVIC_EnableIRQ(WDT_IRQn);

  _initialized = true;
}

#ifdef MCCI_LOW_POWER
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
      PORT->Group[Index].PINCFG[Index].bit.INEN = 0;
      PORT->Group[Index].PINCFG[Index].bit.PULLEN = 0;
    }

    PORT->Group[Index].WRCONFIG.reg |=
      PORT_WRCONFIG_PINMASK_Msk;
    PORT->Group[Index].WRCONFIG.reg |=
      (PORT_WRCONFIG_PINMASK_Msk
       | PORT_WRCONFIG_HWSEL);
  }
}

void lowPowerPins1(uint16_t pinStart) {
  for (uint16_t pin = pinStart; pin < NUM_DIGITAL_PINS; ++pin) {
    pinMode(pin, OUTPUT);
  }

  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(A6, OUTPUT);
  pinMode(A7, OUTPUT);
}

void disableClock(int gclkid)
{
  WRITE8(GCLK->CLKCTRL.reg, gclkid);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}
  GCLK->CLKCTRL.bit.CLKEN = 0;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}
}
void disableClockGenerator(int genid)
{
  WRITE8(GCLK->GENCTRL.reg, genid);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}
  GCLK->GENCTRL.bit.GENEN = 0;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}
}

void configurePM(void)
{
  // Disable CLK_USB_AHB
  PM->AHBMASK.reg &= ~(PM_AHBMASK_USB
                       | PM_AHBMASK_DSU
                       | PM_AHBMASK_HPB1
                       | PM_AHBMASK_HPB2
                       | PM_AHBMASK_DMAC
                       /* These clocks should remain enabled on this bus
                         | PM_AHBMASK_HPB1
                         | PM_AHBMASK_HPB2
                         | PM_AHBMASK_HPB0
                         | PM_AHBMASK_NVMCTRL
                       */
                      );
  PM->APBAMASK.reg &= ~(PM_APBAMASK_RTC
                        | PM_APBAMASK_PAC0
                        | PM_APBAMASK_EIC
                        | PM_APBAMASK_GCLK
                        /* These clocks should remain enabled on this bus
                          | PM_APBAMASK_SYSCTRL
                          | PM_APBAMASK_PM
                          | PM_APBAMASK_WDT
                        */
                       );
  PM->APBBMASK.reg &= ~(PM_APBBMASK_PAC1
                        | PM_APBBMASK_PORT
                        | PM_APBBMASK_DSU
                        | PM_APBBMASK_DMAC
                        | PM_APBBMASK_USB
                        | PM_APBBMASK_NVMCTRL
                        /* These clocks should remain enabled on this bus
                        */
                       );
  PM->APBCMASK.reg &= ~(PM_APBCMASK_ADC
                        | PM_APBCMASK_PAC2
                        | PM_APBCMASK_DAC
                        | PM_APBCMASK_AC
                        | PM_APBCMASK_TC7
                        | PM_APBCMASK_TC6
                        | PM_APBCMASK_TC5
                        | PM_APBCMASK_TC4
                        | PM_APBCMASK_TC3
                        | PM_APBCMASK_TCC2
                        | PM_APBCMASK_TCC1
                        | PM_APBCMASK_TCC0
                        | PM_APBCMASK_SERCOM5
                        | PM_APBCMASK_SERCOM4
                        | PM_APBCMASK_SERCOM3
                        | PM_APBCMASK_SERCOM2
                        | PM_APBCMASK_SERCOM1
                        | PM_APBCMASK_SERCOM0
                        | PM_APBCMASK_EVSYS
                        | PM_APBCMASK_I2S
                       );
}

/*
void gclkGenEnableCheck(void)
  {
  for (uint8_t gclkId = 0; gclkId < 8; ++gclkId)
    {
    Serial.print("GCLK");
    Serial.print(gclkId);
    if (system_gclk_gen_is_enabled(gclkId) == TRUE)
      Serial.println(" is Enabled");
    else
      Serial.println(" is Disabled");
    }
  }
*/
#endif
