// Watchdog Example
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// Pushbutton:
//   SW1 pulls pin PF4 low (internal pull-up is used)
// Watchdog counter 0:
//   This counter is driven by system clock

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "clock.h"
#include "wait.h"
#include "gpio.h"
#include "wd0.h"
#include "tm4c123gh6pm.h"

// Pins
#define RED_LED PORTF,1
#define GREEN_LED PORTF,3
#define PUSH_BUTTON PORTF,4

// Watchdog timeout
#define FCYC_HZ   40000000
#define TIMEOUT_US 2000000

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    enablePort(PORTF);

    // Configure LED and pushbutton pins
    selectPinPushPullOutput(GREEN_LED);
    selectPinPushPullOutput(RED_LED);
    selectPinDigitalInput(PUSH_BUTTON);
    enablePinPullup(PUSH_BUTTON);
}

// Watchdog timer ISR
void watchdogIsr()
{
    setPinValue(RED_LED, 1);
    setPinValue(GREEN_LED, 0);
    // This is the last chance to avoid a reset
    // Write to ICR to avoid a reset
//    WATCHDOG0_ICR_R = 0;
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
	// Initialize hardware
	initHw();

	// Power-on flash of green LED
    setPinValue(RED_LED, 0);
    setPinValue(GREEN_LED, 1);
    waitMicrosecond(1000000);
    setPinValue(GREEN_LED, 0);
    waitMicrosecond(1000000);

    // Start watchdog
	initWatchdog0(TIMEOUT_US, FCYC_HZ);

    // Toggle green LED every 0.1 seconds
	while(true)
    {
	    // Simulate a stuck software loop
	    if (!getPinValue(PUSH_BUTTON))
	    {
	        while(true);
	    }

	    // Reset watchdog
	    resetWatchdog0();

	    // Simulate work in loop
	    setPinValue(GREEN_LED, !getPinValue(GREEN_LED));
	    waitMicrosecond(100000);
	}
}
