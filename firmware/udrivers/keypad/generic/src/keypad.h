/*
 * Copyright (c) 2016-2017, Guillermo A. Amaral B. <g@maral.me>
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

#ifndef UDRIVERS_KEYPAD_H
#define UDRIVERS_KEYPAD_H 1

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

struct keypad_config_t
{
	unsigned int cols;
	unsigned int rows;
	const unsigned int *col_pins;
	const unsigned int *row_pins;
};

struct keypad_glue_t
{
	void (*init)(void *arg);
	void (*free)(void *arg);
	void (*gpio_direction)(void *arg, unsigned int gpio, uint8_t io);
	void (*gpio_set)(void *arg, unsigned int gpio, uint8_t value);
	uint8_t (*gpio_get)(void *arg, unsigned int gpio);
};

struct keypad_handle_t;

/*****************************************************************************/

struct keypad_handle_t *
keypad_init(const struct keypad_config_t *config, struct keypad_glue_t *glue, void *arg);

void
keypad_free(struct keypad_handle_t *handle);

unsigned int
keypad_poll(struct keypad_handle_t *handle);


#ifdef __cplusplus
}
#endif

#endif /* UDRIVERS_KEYPAD_H */

