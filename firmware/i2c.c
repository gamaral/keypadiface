/*
 * Copyright (c) 2017, Guillermo A. Amaral B. <g@maral.me>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stddef.h>
#include <stdint.h>

#include "lpc81x.h"
#include "i2c.h"
#include "state.h"

#define I2C_SLAVE_ADDR (0x10)

extern struct i2c_t *I2C;
extern struct pmu_t *PMU;
extern struct syscon_t *SYSCON;
extern volatile unsigned int k_keypad_state;

static struct nvic_t *NVIC = NVIC_BASE;
static struct switchmatrix_t *SWM = SWM_BASE;
static uint8_t k_keypad_byte = 0;

/*****************************************************************************/

void
i2c_init(void)
{
	/* I2C Peripheral Clock */
	SYSCON->SYSAHBCLKCTRL |= SYSCON_SYSAHBCLKCTRL_I2C;

	/* Reset I2C Peripheral */
	SYSCON->PRESETCTRL &= ~SYSCON_PRESETCTRL_I2C_RST_N;
	SYSCON->PRESETCTRL |= SYSCON_PRESETCTRL_I2C_RST_N;

	/* Select I2C Pins */
	SWM->PINASSIGN[7] = (SWM->PINASSIGN[7] & (0xFFFFFF << 0)) | (11 << 24); /* PIO0_11 */
	SWM->PINASSIGN[8] = (SWM->PINASSIGN[8] & (0xFFFFFF << 8)) | (10 << 0);  /* PIO0_10 */

	/* Slave Mode */
	I2C->CFG = I2C_CFG_SLVEN|I2C_CFG_TIMEOUTEN;
	I2C->INTENSET = I2C_INTENSET_SLVPENDINGEN;
	I2C->SLVADR[0] = I2C_SLAVE_ADDR << 1;
	I2C->TIMEOUT = 0xFF;

	/* Enable I2C IRQ */
	NVIC->ISER[0] |= NVIC_ISER0_ISE_I2C;

	/* I2C Wake-Up */
	SYSCON->STARTERP1 |= SYSCON_STARTERP1_I2C;
}

void
i2c_free(void)
{
	NVIC->ISER[0] &= ~NVIC_ISER0_ISE_I2C;
	SYSCON->STARTERP1 &= ~SYSCON_STARTERP1_I2C;
	SYSCON->SYSAHBCLKCTRL &= ~SYSCON_SYSAHBCLKCTRL_I2C;
}

void
i2c_isr(void)
{
	if ((I2C->INTSTAT & I2C_STAT_SLVPENDING) == 0)
		return;

	switch (I2C->STAT & I2C_STAT_SLVSTATE) {
	case I2C_STAT_SLVSTATE_ADDR:
		k_keypad_byte = 0;
		k_state = STATE_POLL;
		break;

	case I2C_STAT_SLVSTATE_TX:
		I2C->SLVDAT = (k_keypad_state >> ((k_keypad_byte % 4) * 8)) & 0xFF;
		++k_keypad_byte;
		break;

	/*
	 * Writing '0' will power down controller.
	 */
	case I2C_STAT_SLVSTATE_RX:
		switch (I2C->SLVDAT) {
		case 0x00: k_state = STATE_IDLE; break;
		case 0x01: k_state = STATE_POLL; break;
		default:
			I2C->SLVCTL = I2C_SLVCTL_SLVNACK|I2C_SLVCTL_SLVCONTINUE;
			return;
		}
		break;
	}

	I2C->SLVCTL = I2C_SLVCTL_SLVCONTINUE;
}

