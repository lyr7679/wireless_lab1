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
#include "modulator.h"

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

int *pskI[4];
int *pskQ[4];

int bpskI[2] = {GAINI,
                    -GAINI};
int bpskQ[2] = {0,
                     0};

int qpskI[4] = {GAINI,
                    -GAINI,
                    -GAINI,
                     GAINI};
int qpskQ[4] = {GAINQ,
                     GAINQ,
                    -GAINQ,
                    -GAINQ};

int psk8I[8] = {GAINI * 1,
                     GAINI * .71,
                     GAINI * 0,
                    -GAINI * .71,
                    -GAINI * 1,
                    -GAINI * .71,
                     GAINI * 0,
                     GAINI * .71
                    };
int psk8Q[8] = {GAINQ * 0,
                     GAINQ * .71,
                     GAINQ * 1,
                     GAINQ * .71,
                     GAINQ * 0,
                    -GAINQ * .71,
                    -GAINQ * 1,
                    -GAINQ * .71
                    };

int qam16I[16] = {GAINI * .33,
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
int qam16Q[16] = {GAINQ * .33,
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

int *bufferPtrI;
int *bufferPtrQ;
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

//filter variables
bool isFilter = false;
uint32_t conv_len = 0;
uint32_t rcc_pos_idx = 0;
uint32_t psk_pos_idx = 0;
uint32_t current = 0;
uint32_t check1 = 0;
uint32_t check2 = 0;
bool isQam = false;

uint32_t phaseShift = (int) ((4294967296 / FS) * 10000);

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
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_TAILR_R = round(FCYC/sampleRate);
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}

// Must leave this timer on to ensure UI commands like DC are updated
void initSymbolTimer()
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

    int i = 0;
    float tempA = 0;
    float tempB = 0;
    if(isMod)
    {
        if(isFilter)
        {
            for(i = 0; i < 32; i++)
            {
                tempA += h_rrc[i] * bufferPtrI[current];
                tempB += h_rrc[i] * bufferPtrQ[current];

                if(isQam)
                {
                    current = (current + 1) % 64;
                }
                else
                    current = (current + 1) % 32;
            }
            tempA = tempA / 65536 * 4;
            tempB = tempB / 65536 * 4;

            if(valueA > 1395)
                valueA = 1395;
            valueA = tempA + MIDDLEI;
            check1 = valueA;
            valueA |= DC_WRITE_SHDN | DC_WRITE_GA;
            SSI0_DR_R = valueA;

            if(valueB > 1388)
                valueB = 1388;
            valueB = tempB + MIDDLEQ;
            check2 = valueB;
            valueB |= DC_WRITE_SHDN | DC_WRITE_GA | DC_WRITE_AB;
            SSI0_DR_R = valueB;

            if(isQam)
                current = rand() % 65;
            else
                current = rand() % 33;
        }
        else
        {
            valueA = (bufferPtrI[index] + MIDDLEI);
            valueA |= DC_WRITE_SHDN | DC_WRITE_GA;
            SSI0_DR_R = valueA;

            valueB = (bufferPtrQ[index] + MIDDLEQ);
            valueB |= DC_WRITE_SHDN | DC_WRITE_GA | DC_WRITE_AB;
            SSI0_DR_R = valueB;

            index = rand() % (conv_len + 1);
        }
    }
    else
    {
        if(AnotB)
        {
            if(!isDC)
            {
                valueA = LUTA[indexA >> 20];
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
                valueB = LUTB[indexB >> 20];
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

void conv_Arr(int32_t *convArr, int16_t bufferPtr[], uint8_t buff_len)
{
    uint16_t i = 0, j = 0, h_start = 0, x_end = 0;
    int32_t x_start = 0;
    float temp = 0;

    uint8_t h_len = (sizeof(h_rrc) / sizeof(h_rrc[0]));
    conv_len = buff_len + h_len - 1;

    for (i = 0; i < conv_len; i++)
    {
      temp = 0;
      x_start = (((0) > (i - buff_len + 1))) ? (0) : (i - buff_len + 1);
      x_end = (((i + 1) < (h_len))) ? (i + 1) : (h_len);
      h_start = (((i) < (buff_len - 1))) ? (i) : (buff_len - 1);

      for(j = x_start; j < x_end; j++)
      {
          temp += (float)(bufferPtr[h_start]) * (h_rrc[j] * pow(2,16));
          if(h_start != 0)
              h_start = h_start - 1;
      }
          convArr[i] = (int)(temp / 16384);
    }
}

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
                setSymbolRate(350000);
                knownCommand = true;
                isDC = true;
                isMod = false;
                DC = atof(token[2]) + .5; //get value between 0 and 1
                if(token[1][0] == 'a')
                {
                    AnotB = 1;
                    valueA = DC * 4096; //value is now between 0 and 4095
                    valueA |= DC_WRITE_GA | DC_WRITE_SHDN; //turn on bit 12 and 13 for write register
                }
                else if(token[1][0] == 'b')
                {
                    AnotB = 0;
                    valueB = DC * 4096; //value is now between 0 and 4095
                    valueB |= DC_WRITE_GA | DC_WRITE_SHDN | DC_WRITE_AB; //turn on bit 12 and 13 for write register
                }
            }

            // sine a|b FREQ [AMPL [PHASE [DC] ] ]
            if (strcmp(token[0], "sine") == 0)
            {
                setSymbolRate(350000);
                knownCommand = true;
                isDC = false;
                isMod = false;
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
                setSymbolRate(350000);
                knownCommand = true;
                toneCommand = true;
                isMod = false;
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
                setSymbolRate(30000);
                knownCommand = true;
                isDC = false;
                isMod = true;
                isQam = false;

                if(token[1][0] == 'b')
                {
                    shiftBy = 1;
                    modMask = 1;
                    bitsToParse = 5592405;
                    conv_len = 1;
                    if(isFilter)
                    {
                        bufferPtrI = conv_pskI[0];
                        bufferPtrQ = conv_pskQ[0];
                        conv_len = 37;
                    }
                    else
                    {
                        bufferPtrI = pskI[0];
                        bufferPtrQ = pskQ[0];
                    }
                }
                else if(token[1][0] == 'q')
                {
                    shiftBy = 2;
                    modMask = 3;
                    bitsToParse = 454761243;
                    conv_len = 3;
                    isQam = true;
                    if(isFilter)
                    {
                        bufferPtrI = conv_pskI[1];
                        bufferPtrQ = conv_pskQ[1];
                        conv_len = 45;
                    }
                    else
                    {
                        bufferPtrI = pskI[1];
                        bufferPtrQ = pskQ[1];
                    }
                }
                else if(token[1][0] == '8')
                {
                    shiftBy = 3;
                    modMask = 7;
                    bitsToParse = 342391;
                    conv_len = 7;
                    if(isFilter)
                    {
                        bufferPtrI = conv_pskI[2];
                        bufferPtrQ = conv_pskQ[2];
                        conv_len = 61;
                    }
                    else
                    {
                        bufferPtrI = pskI[2];
                        bufferPtrQ = pskQ[2];
                    }
                }
                else if(token[1][0] == '1')
                {
                    shiftBy = 4;
                    modMask = 15;
                    bitsToParse = 245591;
                    conv_len = 15;
                    if(isFilter)
                    {
                        bufferPtrI = conv_pskI[3];
                        bufferPtrQ = conv_pskQ[3];
                        conv_len = 93;
                    }
                    else
                    {
                        bufferPtrI = pskI[3];
                        bufferPtrQ = pskQ[3];
                    }
                }
            }

            // filter FILTER
            if (strcmp(token[0], "filter") == 0)
            {
                knownCommand = true;
                if(token[1][0] == 'r')
                {
                    isFilter = true;
                }
                else if(token[1][0] == 'o')
                {
                    isFilter = false;
                }
            }

            // raw a|b RAW
            if (strcmp(token[0], "raw") == 0)
            {
                setSymbolRate(350000);
                knownCommand = true;
                isDC = true;
                isMod = false;
                toneCommand = false;
                // add code to process command
                if(token[1][0] == 'a')
                {
                    AnotB = 1;
                    valueA = atoi(token[2]); //value between 0 and 4095
                    valueA |= DC_WRITE_GA | DC_WRITE_SHDN; //turn on bit 12 and 13 for write register
                }
                else if(token[1][0] == 'b')
                {
                    AnotB = 0;
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

            if(strcmp(token[0], "rate") == 0)
            {
                knownCommand = true;
                setSymbolRate(atoi(token[1]));
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
