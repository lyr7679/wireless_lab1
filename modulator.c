// CSE 4377/5377 Modulator Example
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    80 MHz

// Hardware configuration:
// DAC on SPI0 Interface:
//   MOSI on PA5 (SSI0Tx)
//   ~CS on PA3  (SSI0Fss)
//   SCLK on PA2 (SSI0Clk)
//   ~LDAC on PA4

//---------------------------------------------------------x--------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdlib.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "gpio.h"
#include "nvic.h"
#include "spi0.h"
#include "uart0.h"
#include "wait.h"

//#define LDAC (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))

#define MAX_CHARS 80

#define FCYC 80e6
#define FDAC 20e6
#define FS 100000

#define SSI0TX PORTA,5
#define SSI0RX PORTA,4
#define SSI0FSS PORTA,3
#define SSI0CLK PORTA,2
#define LDAC PORTD,6

#define DC_WRITE_AB   0x8000
#define DC_WRITE_GA   0x2000
#define DC_WRITE_SHDN 0x1000
#define pi 3.14159265
//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
uint16_t LUTA[4096];
uint16_t LUTB[4096];
uint16_t index = 0;
uint16_t DCValue = 0; //raw dc value from 0-4096, if zero means ac is on
uint16_t indexA = 0; //start at 0 to make sin
uint16_t indexB = 1024; //start at 1024 to create cos

bool DACA = true;
//-----------------------------------------------------------------------------
// EEPROM
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// LUT
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Control
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Timer
//-----------------------------------------------------------------------------

void setSymbolRate(float sampleRate)
{
    TIMER1_TAILR_R = round(FCYC/sampleRate);
}

// Must leave this timer on to ensure UI commands like DC are updated
void initSymbolTimer(void)
{
    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    _delay_cycles(3);

    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = round(FCYC/FS);                 // set load value to match sample rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    enableNvicInterrupt(INT_TIMER1A);                // turn-on interrupt 37 (TIMER1A) in NVIC
}

// Symbol timer called by timer 1
void symbolTimerIsr()
{
    if(DCValue == 0) //means ac
    {
        setPinValue(LDAC, 1);
        setPinValue(SSI0FSS, 0);
        writeSpi0Data(DCValue);
        setPinValue(SSI0FSS, 1);
        setPinValue(LDAC, 0);
    }
    else
    {
        if(indexA >= 4096)
            indexA = 0;
        if(indexB >= 4096)
            indexB = 0;
        setPinValue(LDAC, 1);
        setPinValue(SSI0FSS, 0);
        writeSpi0Data(LUTA[indexA]);
        writeSpi0Data(LUTB[indexB]);
        setPinValue(SSI0FSS, 1);
        setPinValue(LDAC, 0);
        indexA++;
        indexB++;
    }

}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Setup UART0 baud rate
    initUart0();
    setUart0BaudRate(115200, FCYC);
    enablePort(PORTD); //enable for LDAC

    // Initialize SPI0
    initSpi0(USE_SSI0_FSS);
    setSpi0BaudRate(FDAC, FCYC);
    setSpi0Mode(0, 0);

    // Initialize symbol timer
    initSymbolTimer();

}

//-----------------------------------------------------------------------------
// UI
//-----------------------------------------------------------------------------

void processShell()
{
    bool knownCommand = false;
    bool end;
    char c;
    static char strInput[MAX_CHARS+1];
    char* token;
    static uint8_t count = 0;

    char* aOrB;
    char* strDC;
    float DC = 0.0;
    uint16_t total = 0;

    if (kbhitUart0())
    {
        c = tolower(getcUart0());
        end = (c == 13) || (count == MAX_CHARS);
        if (!end)
        {
            if ((c == 8 || c == 127) && count > 0)
                count--;
            if (c >= ' ' && c < 127)
                strInput[count++] = c;
        }
        else
        {
            strInput[count] = '\0';
            count = 0;
            token = strtok(strInput, " ");

            // dc a|b DC
            if (strcmp(token, "dc") == 0)
            {
                knownCommand = true;

                aOrB = strtok(NULL, " ");
                strDC = strtok(NULL, " ");
                DC = (float) atof(strDC) + .5; //get value between 0 and 1
                DCValue += DC * 4096; //value is now between 0 and 4095
                DCValue |= DC_WRITE_GA | DC_WRITE_SHDN; //turn on bit 12 and 13 for write register
                DCValue += (aOrB[0] - 97) * DC_WRITE_AB; //if b turn on bit 15 else leave as 0
            }

            // sine a|b FREQ [AMPL [PHASE [DC] ] ]
            if (strcmp(token, "sine") == 0)
            {
                knownCommand = true;
                // add code to process command
                DCValue = 0;
            }

            // tone FREQ [AMPL [PHASE [DC] ] ]
            if (strcmp(token, "tone") == 0)
            {
                knownCommand = true;
                // add code to process command
            }

            // mod bpsk|qpsk|8psk|16qam
            if (strcmp(token, "mod") == 0)
            {
                knownCommand = true;
                // add code to process command
            }

            // filter FILTER
            if (strcmp(token, "filter") == 0)
            {
                knownCommand = true;
                // add code to process command
            }

            // raw a|b RAW
            if (strcmp(token, "raw") == 0)
            {
                knownCommand = true;
                // add code to process command

                aOrB = strtok(NULL, " ");
                strDC = strtok(NULL, " ");
                total += atoi(strDC); //raw num between 0 and 4096
                total |= DC_WRITE_GA | DC_WRITE_SHDN; //turn on bit 12 and 13 for write register
                total += (aOrB[0] - 97) * DC_WRITE_AB; //if b turn on bit 15 else leave as 0

                setPinValue(LDAC, 1);
                setPinValue(SSI0FSS, 0);
                writeSpi0Data(total);
                setPinValue(SSI0FSS, 1);
                setPinValue(LDAC, 0);
            }

            // reboot
            if (strcmp(token, "reboot") == 0)
            {
                knownCommand = true;
                NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            }

            if (!knownCommand)
                putsUart0("Invalid command\n");

            // help
            if (strcmp(token, "help") == 0)
            {
                putsUart0("Commands:\n");
                putsUart0("  dc       i|q DC\n");
                putsUart0("  sine     i|q FREQ [AMPL [PHASE [DC] ] ]\n");
                putsUart0("  tone     FREQ [AMPL [PHASE [DC] ] ]\n");
                putsUart0("  mod      ook|bpsk|qpsk|8psk|16qam|64qam\n");
                putsUart0("  filter   rrc|off\n");
                putsUart0("  raw      i|q RAW\n");
                putsUart0("  reboot\n");
                putsUart0("\n");
                putsUart0("  where FREQ = [-Fs/2, Fs/2] Hz\n");
                putsUart0("        AMPL = [0, 0.5] V\n");
                putsUart0("        DC   = [-0.5, 0.5] V\n");
                putsUart0("        RAW  = [0, 4095] LSb\n");
            }
        }
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // Initialize hardware
    initHw();

    //create lookup tables
    for(int i = 0; i < 4096; i++)
    {
        LUTA[i] = 2047 + 2047 * sin((i / 4096) * (2 * pi));
        LUTA[i] |= DC_WRITE_GA | DC_WRITE_SHDN; //leave DC_WRITE_AB off to write to DACA
        LUTB[i] = 2047 + 2047 * sin((i / 4096) * (2 * pi));
        LUTB[i] |= DC_WRITE_GA | DC_WRITE_SHDN | DC_WRITE_AB;
    }

    // Randomize data set

    // Greeting
    putsUart0("CSE 4377/5377 Modulator\n");

    // Main Loop
    while (true)
    {
        // UI
        processShell();

        // Other foreground tasks as needed
    }
}
