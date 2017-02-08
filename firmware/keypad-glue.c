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

#include "udrivers/keypad/generic/src/keypad.h"
#include "lpc81x.h"

#define UNUSED(x) (void)x

extern struct iocon_t *IOCON;
extern struct syscon_t *SYSCON;
extern struct gpio_t *GPIO0;

static void __keypad_init(void *);
static void __keypad_free(void *);
static void __gpio_direction(void *, unsigned int, uint8_t);
static void __gpio_set(void *, unsigned int, uint8_t);
static uint8_t __gpio_get(void *, unsigned int);

struct keypad_glue_t k_keypad_glue = {
	init: __keypad_init,
	free: __keypad_free,
	gpio_direction: __gpio_direction,
	gpio_get: __gpio_get,
	gpio_set: __gpio_set
};

/*****************************************************************************/

void
__keypad_init(void *arg)
{
	UNUSED(arg);

	/* GPIO Peripheral Clock */
	SYSCON->SYSAHBCLKCTRL |= SYSCON_SYSAHBCLKCTRL_GPIO;

	/* Reset GPIO Peripheral */
	SYSCON->PRESETCTRL &= ~SYSCON_PRESETCTRL_GPIO_RST_N;
	SYSCON->PRESETCTRL |= SYSCON_PRESETCTRL_GPIO_RST_N;

	/* Configure PU Resistors */
	SYSCON->SYSAHBCLKCTRL |= SYSCON_SYSAHBCLKCTRL_IOCON;
	IOCON->PIO0_0 = (1 << 3)|(1 << 10);
	IOCON->PIO0_1 = (1 << 3)|(1 << 10);
	IOCON->PIO0_12 = (1 << 3);
	IOCON->PIO0_13 = (1 << 3);
	IOCON->PIO0_4 = (1 << 3);
	IOCON->PIO0_6 = (1 << 3)|(1 << 10);
	IOCON->PIO0_7 = (1 << 3)|(1 << 10);
	SYSCON->SYSAHBCLKCTRL &= ~SYSCON_SYSAHBCLKCTRL_IOCON;
}

void
__keypad_free(void *arg)
{
	UNUSED(arg);
	SYSCON->SYSAHBCLKCTRL &= ~SYSCON_SYSAHBCLKCTRL_GPIO;
}

void
__gpio_direction(void *arg, unsigned int gpio, uint8_t io)
{
	UNUSED(arg);
	if (io) GPIO0->DIR0 |= (1 << gpio);
	else GPIO0->DIR0 &= ~(1 << gpio);
}

void
__gpio_set(void *arg, unsigned int gpio, uint8_t value)
{
	UNUSED(arg);
	if (value) GPIO0->SET0 = (1 << gpio);
	else GPIO0->CLR0 = (1 << gpio);
}

uint8_t
__gpio_get(void *arg, unsigned int gpio)
{
	UNUSED(arg);
	return((GPIO0->PIN0 & (1 << gpio)) ? 1 : 0);
}

