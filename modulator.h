
//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// SPI0 Interface:
//   MOSI on PA5 (SSI0Tx)
//   MISO on PA4 (SSI0Rx)
//   ~CS on PA3  (SSI0Fss)
//   SCLK on PA2 (SSI0Clk)

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef MODULATOR_H_
#define MODULATOR_H_

#include <stdint.h>
#include <stdbool.h>

#define GAINI 1900
#define GAINQ 1900


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------



//length of h_rrc = 31
float h_rrc[] = {-0.0141,   -0.0216,    -0.0165,    0.0000,     0.0198,     0.0310,     0.0245,     -0.0000,
                 -0.0317,   -0.0525,    -0.0447,    0.0000,     0.0748,     0.1590,     0.2250,     0.2500,
                  0.2250,    0.1590,     0.0748,    0.0000,     -0.0447,    -0.0525,    -0.0317,    -0.0000,
                  0.0245,    0.0310,     0.0198,    0.0000,     -0.0165,    -0.0216,    -0.0141};

uint32_t *conv_pskI[4];
uint32_t *conv_pskQ[4];

uint32_t conv_bpskI[(31 + 8 - 1)];
uint32_t conv_qpskI[(31 + 16 - 1)];
uint32_t conv_psk8I[(31 + 32 - 1)];
uint32_t conv_qam16I[(31 + 16 - 1)];

uint32_t conv_bpskQ[(31 + 8 - 1)];
uint32_t conv_qpskQ[(31 + 16 - 1)];
uint32_t conv_psk8Q[(31 + 32 - 1)];
uint32_t conv_qam16Q[(31 + 16 - 1)];

uint32_t rc_bpskI[8] = {GAINI,
                     0,
                     0,
                     0,
                    -GAINI,
                     0,
                     0,
                     0};
uint32_t rc_bpskQ[8] = {0,
                     0,
                     0,
                     0,
                     0,
                     0,
                     0,
                     0};

uint32_t rc_qpskI[16] = {GAINI,
                     0,
                     0,
                     0,
                    -GAINI,
                     0,
                     0,
                     0,
                    -GAINI,
                     0,
                     0,
                     0,
                     GAINI,
                     0,
                     0,
                     0};
uint32_t rc_qpskQ[4] = {GAINQ,
                     0,
                     0,
                     0,
                     GAINQ,
                     0,
                     0,
                     0,
                    -GAINQ,
                     0,
                     0,
                     0,
                    -GAINQ,
                     0,
                     0,
                     0};

uint32_t rc_psk8I[32] = {GAINI * 1,
                     0,
                     0,
                     0,
                     GAINI * .71,
                     0,
                     0,
                     0,
                     GAINI * 0,
                     0,
                     0,
                     0,
                    -GAINI * .71,
                     0,
                     0,
                     0,
                    -GAINI * 1,
                     0,
                     0,
                     0,
                    -GAINI * .71,
                     0,
                     0,
                     0,
                     GAINI * 0,
                     0,
                     0,
                     0,
                     GAINI * .71,
                     0,
                     0,
                     0
                    };
uint32_t rc_psk8Q[32] = {GAINQ * 0,
                      0,
                      0,
                      0,
                     GAINQ * .71,
                     0,
                     0,
                     0,
                     GAINQ * 1,
                     0,
                     0,
                     0,
                     GAINQ * .71,
                     0,
                     0,
                     0,
                     GAINQ * 0,
                     0,
                     0,
                     0,
                    -GAINQ * .71,
                     0,
                     0,
                     0,
                    -GAINQ * 1,
                     0,
                     0,
                     0,
                    -GAINQ * .71,
                     0,
                     0,
                     0,
                    };

uint32_t rc_qam16I[64] = {GAINI * .33,
                       0,
                       0,
                       0,
                       GAINI * .33,
                       0,
                       0,
                       0,
                       GAINI * 1,
                       0,
                       0,
                       0,
                       GAINI * 1,
                       0,
                       0,
                       0,
                       GAINI * .33,
                       0,
                       0,
                       0,
                       GAINI * .33,
                       0,
                       0,
                       0,
                       GAINI * 1,
                       0,
                       0,
                       0,
                       GAINI * 1,
                       0,
                       0,
                       0,
                      -GAINI * .33,
                       0,
                       0,
                       0,
                      -GAINI * .33,
                       0,
                       0,
                       0,
                      -GAINI * 1,
                       0,
                       0,
                       0,
                      -GAINI * 1,
                       0,
                       0,
                       0,
                      -GAINI * .33,
                       0,
                       0,
                       0,
                      -GAINI * .33,
                       0,
                       0,
                       0,
                      -GAINI * 1,
                       0,
                       0,
                       0,
                      -GAINI * 1,
                       0,
                       0,
                       0
                    };
uint32_t rc_qam16Q[64] = {GAINQ * .33,
                       0,
                       0,
                       0,
                       GAINQ * 1,
                       0,
                       0,
                       0,
                       GAINQ * .33,
                       0,
                       0,
                       0,
                       GAINQ * 1,
                       0,
                       0,
                       0,
                      -GAINQ * .33,
                       0,
                       0,
                       0,
                      -GAINQ * 1,
                       0,
                       0,
                       0,
                      -GAINQ * .33,
                       0,
                       0,
                       0,
                      -GAINQ * 1,
                       0,
                       0,
                       0,
                       GAINQ * .33,
                       0,
                       0,
                       0,
                       GAINQ * 1,
                       0,
                       0,
                       0,
                       GAINQ * .33,
                       0,
                       0,
                       0,
                       GAINQ * 1,
                       0,
                       0,
                       0,
                      -GAINQ * .33,
                       0,
                       0,
                       0,
                      -GAINQ * 1,
                       0,
                       0,
                       0,
                      -GAINQ * .33,
                       0,
                       0,
                       0,
                      -GAINQ * 1,
                       0,
                       0,
                       0
                    };

#endif
