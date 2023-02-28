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
#define FS 500000

#define SSI0TX PORTA,5
#define SSI0RX PORTA,4
#define SSI0FSS PORTA,3
#define SSI0CLK PORTA,2
#define LDAC PORTB,4

#define MAX_ARGS 5
#define WHITESPACE " "

#define DC_WRITE_AB   0x8000
#define DC_WRITE_GA   0x2000
#define DC_WRITE_SHDN 0x1000

#define pi 3.14159265
#define GAINI 1850
#define GAINQ 1900

uint32_t bpskI[2] = {GAINI,
                    -GAINI};
uint32_t bpskQ[2] = {0,
                     0};

uint32_t qpskI[2] = {GAINI,
                    -GAINI};
uint32_t qpskQ[2] = {GAINQ,
                    -GAINQ};

uint32_t psk8I[8] = {GAINI * 1,
                     GAINI * .71,
                    -GAINI * .71,
                     GAINI * 0,
                     GAINI * .71,
                    -GAINI * 0,
                    -GAINI * 1,
                    -GAINI * .71
                    };
uint32_t psk8Q[8] = {GAINQ * 0,
                     GAINQ * .71,
                    -GAINQ * .71,
                     GAINQ * 1,
                     GAINQ * .71,
                    -GAINQ * 1,
                    -GAINQ * 0,
                    -GAINQ * .71
                    };
//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
uint16_t LUTA[4096]; //lookup table for DAC A
uint16_t LUTB[4096]; //lookup table for DAC B
uint16_t DCValueA = 0; //raw dc value from 0-4096, if zero means ac is on
uint16_t DCValueB = 0; //raw dc value from 0-4096, if zero means ac is on
uint32_t indexA = 0; //start at 0 to make sin
uint32_t indexB = 0; //start at 1024 to create cos
uint8_t AnotB = 1; //if True output DAC A in isr, else DAC B
bool toneCommand = true;
float amplitude = .5;
uint32_t degreeShift = 0;

uint32_t phaseShift = (int) ((4294967296 / FS) * 10000);
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
void initLdacTimer(void)
{
    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;
    _delay_cycles(3);

    // Configure Timer 2
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER2_TAILR_R = round(FCYC/FS);                 // set load value to match sample rate
    TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    enableNvicInterrupt(INT_TIMER2A);                // turn-on interrupt (TIMER2A) in NVIC
}

void ldacTimerIsr()
{
    setPinValue(LDAC, 0);
    _delay_cycles(2);
    setPinValue(LDAC, 1);
}

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
    setPinValue(LDAC, 0);
    setPinValue(LDAC, 1);

    if(AnotB)
    {
        SSI0_DR_R = LUTA[indexA >> 20];
        indexA += (phaseShift);
        //indexA %= 4096;
    }
    else
    {
        SSI0_DR_R = LUTB[indexB >> 20];
        indexB += (phaseShift);
        //indexB %= 4096;
    }
    if(toneCommand)
        AnotB ^= 1;

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo80Mhz();

    // Setup UART0 baud rate
    initUart0();
    setUart0BaudRate(115200, FCYC);
    enablePort(PORTB); //enable for LDAC

    enablePort(PORTA);

    // Initialize SPI0
    initSpi0(USE_SSI0_FSS);
    setSpi0BaudRate(FDAC, FCYC);
    setSpi0Mode(0, 0);

    selectPinPushPullOutput(SSI0FSS);
    selectPinPushPullOutput(LDAC);

    //create LUT
    uint32_t i;
    for(i = 0; i < 4096; i++)
    {
//        LUTA[i] = 2150 + 1850 * sin((i / 4096.0) * (2 * pi));
//        LUTA[i] |= DC_WRITE_GA | DC_WRITE_SHDN; //leave DC_WRITE_AB off to write to DACA
        LUTA[i] = 2150 + 1900 * sin((i / 4096.0) * (2 * pi));
        LUTA[i] |= DC_WRITE_GA | DC_WRITE_SHDN;
        LUTB[i] = 2150 + 1900 * cos((i / 4096.0) * (2 * pi));
        LUTB[i] |= DC_WRITE_GA | DC_WRITE_SHDN | DC_WRITE_AB;
    }

    // Initialize symbol timer
    initSymbolTimer();
    //initLdacTimer();

    //setPinValue(SSI0FSS, 0);
    setPinValue(LDAC, 1);

}

//-----------------------------------------------------------------------------
// UI
//-----------------------------------------------------------------------------


uint8_t tokenizeInput(char *strInput, char *token_arr[])
{
    uint8_t token_count = 0;
    char *token;

   /* get the first token */
   token = strtok(strInput, WHITESPACE);
   token_arr[token_count] = token;
   token_count++;

   /* walk through other tokens */
   while( token != NULL ) {
       token = strtok(NULL, WHITESPACE);
       token_arr[token_count] = token;
       token_count++;
   }
   return token_count;
}

void processShell()
{
    bool knownCommand = false;
    toneCommand = false;
    bool end;
    char c;
    static char strInput[MAX_CHARS+1];
    char* token[MAX_ARGS];
    uint8_t token_count = 0;
    static uint8_t count = 0;

    float DC = 0.0;

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
            //token = strtok(strInput, " ");

            token_count = tokenizeInput(strInput, token);

            if (strcmp(token[0], "index") == 0)
            {
                knownCommand = true;
                if(token[1][0] == 'a')
                {
                    indexA = atoi(token[2]);
                }
                else if(token[1][0] == 'b')
                {
                    indexB = atoi(token[2]);
                }
            }

            // dc a|b DC
            if (strcmp(token[0], "dc") == 0)
            {
                knownCommand = true;
                DC = atof(token[2]) + .5; //get value between 0 and 1
                if(token[1][0] == 'a')
                {
                    AnotB = true;
                    DCValueA = DC * 4096; //value is now between 0 and 4095
                    DCValueA |= DC_WRITE_GA | DC_WRITE_SHDN; //turn on bit 12 and 13 for write register
                }
                else if(token[1][0] == 'b')
                {
                    AnotB = false;
                    DCValueB = DC * 4096; //value is now between 0 and 4095
                    DCValueB |= DC_WRITE_GA | DC_WRITE_SHDN; //turn on bit 12 and 13 for write register
                    DCValueB |= DC_WRITE_AB;
                }
            }

            // sine a|b FREQ [AMPL [PHASE [DC] ] ]
            if (strcmp(token[0], "sine") == 0)
            {
                knownCommand = true;
                token_count--;
                // add code to process command
                if(token_count)
                {
                    if(token[1][0] == 'a')
                        AnotB = 1;
                    else
                        AnotB = 0;
                    token_count--;
                }
                if(token_count)
                {
                    phaseShift = (int) ((4294967296 / FS) * atoi(token[2]));
                    token_count--;
                }
                if(token_count)
                {
                    amplitude = atof(token[3]);
                    token_count--;
                }
                if(token_count)
                {
                    phaseDegree = atoi(token[4]);
                    token_count--;
                }
            }

            // tone FREQ [AMPL [PHASE [DC] ] ]
            if (strcmp(token[0], "tone") == 0)
            {
                knownCommand = true;
                toneCommand = true;
                // add code to process command
            }

            // mod bpsk|qpsk|8psk|16qam
            if (strcmp(token[0], "mod") == 0)
            {
                knownCommand = true;
                // add code to process command
            }

            // filter FILTER
            if (strcmp(token[0], "filter") == 0)
            {
                knownCommand = true;
                // add code to process command
            }

            // raw a|b RAW
            if (strcmp(token[0], "raw") == 0)
            {
                knownCommand = true;
                // add code to process command
                if(token[1][0] == 'a')
                {
                    DCValueA = atoi(token[2]); //value between 0 and 4095
                    DCValueA |= DC_WRITE_GA | DC_WRITE_SHDN; //turn on bit 12 and 13 for write register
                }
                else if(token[1][0] == 'b')
                {
                    DCValueB = atoi(token[2]); //value between 0 and 4095
                    DCValueB |= DC_WRITE_GA | DC_WRITE_SHDN | DC_WRITE_AB; //turn on bit 12 and 13 for write register
                }
            }

            // reboot
            if (strcmp(token[0], "reboot") == 0)
            {
                knownCommand = true;
                NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            }

            if (!knownCommand)
                putsUart0("Invalid command\n");

            // help
            if (strcmp(token[0], "help") == 0)
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

    // Randomize data set
    //phaseShift = (int) ((pow(2, 32) / FS) * 10000);
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
