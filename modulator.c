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
#define FS 350000

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
#define GAINI 1900
#define GAINQ 1900

uint32_t *pskI[4];
uint32_t *pskQ[4];

uint32_t bpskI[2] = {GAINI,
                    -GAINI};
uint32_t bpskQ[2] = {0,
                     0};

uint32_t qpskI[4] = {GAINI,
                    -GAINI,
                    -GAINI,
                     GAINI};
uint32_t qpskQ[4] = {GAINQ,
                     GAINQ,
                    -GAINQ,
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
                    -GAINQ * .71,
                    -GAINQ * 1,
                    -GAINQ * 0,
                     GAINQ * .71
                    };

uint32_t qam16I[16] = {GAINI * .33,
                       GAINI * .33,
                       GAINI * 1,
                       GAINI * 1,
                       GAINI * .33,
                       GAINI * .33,
                       GAINI * 1,
                       GAINI * 1,
                      -GAINI * .33,
                      -GAINI * .33,
                      -GAINI * 1,
                      -GAINI * 1,
                      -GAINI * .33,
                      -GAINI * .33,
                      -GAINI * 1,
                      -GAINI * 1,
                    };
uint32_t qam16Q[16] = {GAINQ * .33,
                       GAINQ * 1,
                       GAINQ * .33,
                       GAINQ * 1,
                      -GAINQ * .33,
                      -GAINQ * 1,
                      -GAINQ * .33,
                      -GAINQ * 1,
                       GAINQ * .33,
                       GAINQ * 1,
                       GAINQ * .33,
                       GAINQ * 1,
                      -GAINQ * .33,
                      -GAINQ * 1,
                      -GAINQ * .33,
                      -GAINQ * 1,
                    };

uint32_t *bufferPtrI;
uint32_t *bufferPtrQ;
//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
uint16_t LUTA[4096]; //lookup table for DAC A
uint16_t LUTB[4096]; //lookup table for DAC B
uint16_t valueA = 0; //raw dc value from 0-4096, if zero means ac is on
uint16_t valueB = 0; //raw dc value from 0-4096, if zero means ac is on
uint32_t indexA = 0; //start at 0 to make sin
uint32_t indexB = 1024 << 20; //start at 1024 to create cos
uint8_t AnotB = 1; //if True output DAC A in isr, else DAC B
bool toneCommand = true;
float amplitude = .5;
uint32_t degreeShift = 0;

//mod variables
uint32_t bitsToParse = 1776411;
uint8_t shiftBy = 2;
int modIndex = 23;
bool isMod = false;
uint32_t index = 0;
uint32_t modMask = 3;
bool isDC = false;

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

    if(isMod)
    {
        if(modIndex < 0)
            modIndex = 23;
        index = bitsToParse & (modMask << ((modIndex + 1) - shiftBy));
        index = index >> ((modIndex + 1) - shiftBy);
        valueA = (bufferPtrI[index] + GAINI);
        valueA |= DC_WRITE_SHDN | DC_WRITE_GA;
        SSI0_DR_R = valueA;
        valueB = (bufferPtrQ[index] + GAINQ);
        valueB |= DC_WRITE_SHDN | DC_WRITE_GA | DC_WRITE_AB;
        SSI0_DR_R = valueB;
        modIndex -= shiftBy;
    }
    else
    {
        if(AnotB)
        {
            if(!isDC)
            {
                //valueA = (amplitude / .5) * LUTA[indexA >> 20];
                valueA = LUTA[indexA >> 20];
                //valueA |= DC_WRITE_GA | DC_WRITE_SHDN;
                if(toneCommand)
                    indexA += (phaseShift);
                indexA += (phaseShift);
            }
            SSI0_DR_R = valueA;
        }
        else
        {
            if(!isDC)
            {
//                valueB = (amplitude / .5) * LUTB[indexB >> 20]
                valueB = LUTB[indexB >> 20];
                //valueB |= DC_WRITE_GA | DC_WRITE_SHDN | DC_WRITE_AB;
                if(toneCommand)
                    indexB += (phaseShift);
                indexB += (phaseShift);
            }
            SSI0_DR_R = valueB;
        }
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
        LUTA[i] = 2150 + 1900 * sin((i / 4096.0) * (2 * pi));
        LUTA[i] |= DC_WRITE_GA | DC_WRITE_SHDN;
        LUTB[i] = 2150 + 1900 * sin((i / 4096.0) * (2 * pi));
        LUTB[i] |= DC_WRITE_GA | DC_WRITE_SHDN | DC_WRITE_AB;
    }

    pskI[0] = bpskI;
    pskI[1] = qpskI;
    pskI[2] = psk8I;
    pskI[3] = qam16I;

    pskQ[0] = bpskQ;
    pskQ[1] = qpskQ;
    pskQ[2] = psk8Q;
    pskQ[3] = qam16Q;

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

            token_count = tokenizeInput(strInput, token) - 1;

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
                isDC = true;
                DC = atof(token[2]) + .5; //get value between 0 and 1
                if(token[1][0] == 'a')
                {
                    AnotB = true;
                    valueA = DC * 4096; //value is now between 0 and 4095
                    valueA |= DC_WRITE_GA | DC_WRITE_SHDN; //turn on bit 12 and 13 for write register
                }
                else if(token[1][0] == 'b')
                {
                    AnotB = false;
                    valueB = DC * 4096; //value is now between 0 and 4095
                    valueB |= DC_WRITE_GA | DC_WRITE_SHDN | DC_WRITE_AB; //turn on bit 12 and 13 for write register
                }
            }

            // sine a|b FREQ [AMPL [PHASE [DC] ] ]
            if (strcmp(token[0], "sine") == 0)
            {
                knownCommand = true;
                isDC = false;
                toneCommand = false;
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
                    degreeShift = atoi(token[4]);
                    if(token[1][0] == 'a')
                        indexA = ((int)((degreeShift / 360.0) * 4096) << 20);
                    else
                        indexB = ((int)((degreeShift / 360.0) * 4096) << 20);
                    token_count--;
                }
            }

            // tone FREQ [AMPL [PHASE [DC] ] ]
            if (strcmp(token[0], "tone") == 0)
            {
                knownCommand = true;
                toneCommand = true;
                isDC = false;
                token_count--;
                // add code to process command
                if(token_count)
                {
                    phaseShift = (int) ((4294967296 / FS) * atoi(token[1]));
                    token_count--;
                }
                if(token_count)
                {
                    amplitude = atof(token[2]);
                    token_count--;
                }
                if(token_count)
                {
                    degreeShift = atoi(token[3]);
                    indexA = ((int)((degreeShift / 360.0) * 4096) << 20);
                    indexB = ((int)(((degreeShift / 360.0) * 4096) + 1024) << 20);
                    token_count--;
                }
            }

            // mod bpsk|qpsk|8psk|16qam
            if (strcmp(token[0], "mod") == 0)
            {
                knownCommand = true;
                isDC = false;
                isMod = true;
                // add code to process command
                if(token[1][0] == 'b')
                {
                    shiftBy = 1;
                    modMask = 1;
                    bitsToParse = 5592405;
                    bufferPtrI = pskI[0];
                    bufferPtrQ = pskQ[0];
                }
                else if(token[1][0] == 'q')
                {
                    shiftBy = 2;
                    modMask = 3;
                    bitsToParse = 454761243;
                    bufferPtrI = pskI[1];
                    bufferPtrQ = pskQ[1];
                }
                else if(token[1][0] == '8')
                {
                    shiftBy = 3;
                    modMask = 7;
                    bitsToParse = 342391;
                    bufferPtrI = pskI[2];
                    bufferPtrQ = pskQ[2];
                }
                else if(token[1][0] == '1')
                {
                    shiftBy = 4;
                    modMask = 15;
                    bitsToParse = 245591;
                    bufferPtrI = pskI[3];
                    bufferPtrQ = pskQ[3];
                }
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
                isDC = true;
                // add code to process command
                if(token[1][0] == 'a')
                {
                    valueA = atoi(token[2]); //value between 0 and 4095
                    valueA |= DC_WRITE_GA | DC_WRITE_SHDN; //turn on bit 12 and 13 for write register
                }
                else if(token[1][0] == 'b')
                {
                    valueB = atoi(token[2]); //value between 0 and 4095
                    valueB |= DC_WRITE_GA | DC_WRITE_SHDN | DC_WRITE_AB; //turn on bit 12 and 13 for write register
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
