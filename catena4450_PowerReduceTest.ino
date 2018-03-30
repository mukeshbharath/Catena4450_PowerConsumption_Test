/* catena4450_LowPowerTest.ino  Thu Dec 14 2017 20:15:02 mukeshbharath */

/*

Module:  catena4450_LowPowerTest.ino

Function:
  Code for testing the low power state of Catena 4450.

Version:
  V0.1.0  Thu Dec 14 2017 20:15:02 mukeshbharath  Edit level 1

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
   0.1.0  Thu Dec 14 2017 20:15:02  mukeshbharath
  Module created.

*/
//#include "RegisterDump.h"
#include "catena4450_PowerReduce.h"
#include "ThisCatena.h"
//#include <delay.h>

#include <cmath>
#include <type_traits>

/****************************************************************************\
|
|   Manifest constants & typedefs.
|
| This is strictly for private types and constants which will not
| be exported.
|
\****************************************************************************/
using namespace McciCatena;

/****************************************************************************\
|
| Read-only data.
|
| If program is to be ROM-able, these must all be tagged read-only
| using the ROM storage class; they may be global.
|
\****************************************************************************/

/****************************************************************************\
|
| VARIABLES:
|
| If program is to be ROM-able, these must be initialized
| using the BSS keyword.  (This allows for compilers that require
| every variable to have an initializer.)  Note that only those
| variables owned by this module should be declared here, using the BSS
| keyword; this allows for linkers that dislike multiple declarations
| of objects.
|
\****************************************************************************/

// globals
Catena4450_PR	gCatena4450;
ThisCatena gCatena;
int AlarmTime;

//Callback function for USB Attach during wakeup
void attachCB(void);

/*

Name:	setup()

Function:
	Arduino setup function.

Definition:
	void setup(
		void
		);

Description:
  This function is called by the Arduino framework after
  basic framework has been initialized. We initialize the registers
  which are need to be configured for making catena4450 to go to
  the low power mode and (ultimately) return to the framework,
  which then calls loop().

Returns:
  No explicit result.

*/
void setup() {
	/* Setting up things */
  gCatena.begin();
	gCatena4450.begin();
	Serial.begin (9600);	
	}

/*

Name:	loop()

Function:
	Arduino setup function.

Definition:
	void loop(
		void
		);

Description:
  This function is called by the Arduino framework after
  things had been initialized on the setup () function. We initialize
  the sensors that are present on the platform, set up the LoRaWAN connection,
        and (ultimately) return to the framework, which then calls loop()
        forever.

Returns:
  No explicit result.

*/

void loop() {

	/* Adds 10 seconds to alarm time */
	AlarmTime = gCatena4450.getSeconds() + 10;
	/* checks for roll over 60 seconds and corrects */
	AlarmTime = AlarmTime % 60;
	Serial.print("Next Alarm Time:");
	Serial.println(AlarmTime);

	/* Wakes at next alarm time */
	gCatena4450.setAlarmSeconds(AlarmTime);
	/* Match seconds only */
	gCatena4450.enableAlarm(gCatena4450.MATCH_SS);

	gCatena4450.attachInterrupt(attachCB);

  //regDumpCtrl();
	//Serial.end();
 
 //gCatena4450.blinkLed(2, 500);
  //SYSCTRL->OSC8M.bit.ENABLE = 0;
  //SYSCTRL->DFLLCTRL.bit.ENABLE = 0;

  Serial.println("Going to Standby Mode!...");
  /* Blink LED with a delay */
  gCatena4450.blinkLed(5, 500);

	/* Safely detach the USB prior to sleeping */
	//USBDevice.detach();

	/* Sleep until next alarm match */
	gCatena4450.standbyMode();

	/* Re-attach the USB, audible sound on windows machines */
	//USBDevice.attach();

	/* Blink LED indicating awake */
	gCatena4450.blinkLed(3, 200);

	Serial.begin(9600);
	/* Wait till the Serial gets ready */
	while (! Serial);
	Serial.println("Awake...");
	//regDumpCtrl();
	}

void attachCB(void) // Do something when interrupt called
	{
    //SYSCTRL->OSC8M.bit.ENABLE = 1;
    //SYSCTRL->DFLLCTRL.bit.ENABLE = 1;

	//Serial.println("Attach...");
	}


