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
#include "udrivers/keypad/generic/src/keypad.h"

#include "cmsis_gcc.h"

#define UNUSED(x) (void)x

volatile unsigned int k_keypad_state = 0;
volatile enum state_t k_state = STATE_POLL;

struct gpio_t *GPIO0 = GPIO0_BASE;
struct i2c_t *I2C = I2C_BASE;
struct iocon_t *IOCON = IOCON_BASE;
struct pmu_t *PMU = PMU_BASE;
struct scb_t *SCB = SCB_BASE;
struct syscon_t *SYSCON = SYSCON_BASE;

/* KEYPAD ********************************************************************/

static const unsigned int k_keypad_row_pins[] = { 1,  7,  6, 0 };
static const unsigned int k_keypad_col_pins[] = { 4, 12, 13 };
static const struct keypad_config_t k_keypad_config = {
	cols: 3, rows: 4,
	col_pins: k_keypad_col_pins,
	row_pins: k_keypad_row_pins
};
extern struct keypad_glue_t k_keypad_glue;

/*****************************************************************************/

int
main(void)
{
	struct keypad_handle_t *keypad;
	unsigned int state = 0;

	/*
	 * Initialize Peripherals
	 */
	keypad = keypad_init(&k_keypad_config, &k_keypad_glue, NULL);
	i2c_init();

	/*
	 * Sleep Configuration
	 */
	PMU->PCON |= 0x1;
	SCB->SCR |= SCB_SCR_SLEEPDEEP;

	do {
		switch (k_state) {
		case STATE_IDLE:
			k_keypad_state = 0;
			if (I2C->STAT & I2C_STAT_SLVNOTSTR)
				__WFI();
			break;

		case STATE_POLL:
			state = keypad_poll(keypad);

			__disable_irq();
			k_keypad_state = state;
			__enable_irq();
			break;
		}
		
	} while(1);

	/*
	 * This should never be reached.
	 */

	i2c_free();
	keypad_free(keypad);

	return(0);
}

