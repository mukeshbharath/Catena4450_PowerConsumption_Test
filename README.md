# Catena4450_PowerConsumption_Test
Hi Mukesh,

One, you should not worry about normal mode. We *only* care about deep sleep. 

I’m pretty sure that you’re not getting into STANDBY mode. There is a chart from the datasheet page 792.

Your description looks as if you’re going between one of the various IDLE modes. Furthermore, IDLE mode should consume about 1 to 2ma *more* than STANDBY mode.  So if there’s a static current draw on the board other than the CPU, you should still be adjusting relative to the baseline by more than we’re seeing.

I’ve looked at the code you posted. We have not gone through all the peripheral blocks, disabling the clocks. This is very tedious, but it is required.

As stated in the attached app note section 7 and 8, you have to 
1.	turn off all the peripheral blocks by setting CTRLx.ENABLE to zero
2.	set the clock input for each peripheral block to a disabled clock generator
3.	set all I/O pins to the default I/O config: DIR[pin] = 0, PINCFGy.INEN[pin] = 0, PINCFGy.PULLEN[pin] = 0.  In otherwords, DIRSET = ~0; WRCONFIG = 0xFFFF; WRCONFIG = 0x8000FFFF;

The sample code in the attached zip file has a D21 branch. It doesn’t do much, but it shows correctly how to enter idle mode (you have at least one error at https://github.com/mukeshbharath/Catena4450_PowerConsumption_Test/blob/39abfcf46e1e0f8228216375f795de3c1a76d029/catena4450_PowerReduce.cpp#L229 which is oring a bit *position* into SCR (totally scrambling it). See how they do it in their code, in function system_set_sleepmode() and system_sleep(). Similar to what you do, but clearer.

Looking at section 4 of the block diagram, you need to power down all of the following:
•	SERCOMx6
•	TCx5
•	TCC#3
•	ADC
•	AC
•	DAC
•	PTC
•	I2S
•	USB 

I can tell that you’re testing with USB up (because you’re doing serial prints). That has to be disabled for this testing!

The order for power down is critical. You must first disable all the peripheral blocks using CTLRA.ENABLE=0, and waiting for the block to be synchronized. Then you must disable the clocks.  Power up in the reverse order.

But I list the rest of this in datasheet order.

Section 15: you must, for each relevant ID value in table 15-5 page 119, you must consider:
1.	Is the target block enabled? (Basically only true for RTC and possibly WDT)
2.	If not, set CLKCTRL to {ID = relevant ID value, GEN = a disabled clock generateor}

Section 16: In the Power manager, the AHB, APBA, APBB. APBC mask registers should be mostly cleared, so that clock are turned off to everything except the required peripherals (RTC and interrupt controller).

Section 17:
All oscillators other than the one you’re using, XOSC32K, should be disabled. The SYSCTRL.*.RUNSTDBY bit must be cleared for as many of the clock sources as possible.

Section 20: all the DMA controllers should be disabled

Section 22: the NVMCTRLneeds to be put in WAKEUPACCESS mode (CTRLB)

Section 25..28: for i == 0..5, set SERCOM[i].CTRLA.ENABLE = 0

Section 29: disable I2S (CTRLA.ENABLE = 0)

Section 30: for each TC, disable (CTRLA.ENABLE=0)

Section 31: for each TCC, disable (CTRLA.ENABLE=0)

Section 32: disable USB (CTRLA.ENABLE=0)

Section 33: disable ADC (CTRLA.ENABLE=0)

Section 34: disable AC (CTRLA.ENABLE = 0)

Section 35: disable DAC (CTRLA.ENABLE = 0)
