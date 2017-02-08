/*
 * Copyright (c) 2016, Guillermo A. Amaral B. <g@maral.me>
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

#include "keypad.h"

#include <stdlib.h>

struct keypad_handle_t
{
	const struct keypad_config_t *config;
	struct keypad_glue_t *glue;
	void *glue_arg;
};

/*****************************************************************************/

struct keypad_handle_t *
keypad_init(const struct keypad_config_t *config, struct keypad_glue_t *g, void *garg)
{
	struct keypad_handle_t *h = calloc(1, sizeof(struct keypad_handle_t));
	unsigned int i;

	h->config = config;
	h->glue = g;
	h->glue_arg = garg;

	if (h->glue->init)
		h->glue->init(h->glue_arg);

	/* GPIO Init */

	for (i = 0; i < h->config->rows; ++i) {
		h->glue->gpio_direction(h->glue_arg, h->config->row_pins[i], 0);
		h->glue->gpio_set(h->glue_arg, h->config->row_pins[i], 0);
	}

	for (i = 0; i < h->config->cols; ++i) {
		h->glue->gpio_direction(h->glue_arg, h->config->col_pins[i], 0);
		h->glue->gpio_set(h->glue_arg, h->config->col_pins[i], 0);
	}

	return(h);
}

void
keypad_free(struct keypad_handle_t *h)
{
	if (h->glue->free)
		h->glue->free(h->glue_arg);

	free(h);
}

unsigned int
keypad_poll(struct keypad_handle_t *h)
{
	unsigned int r, c;
	unsigned int state = 0;

	for (c = 0; c < h->config->cols; ++c) {
		h->glue->gpio_direction(h->glue_arg, h->config->col_pins[c], 1);
		h->glue->gpio_set(h->glue_arg, h->config->col_pins[c], 1);
		for (r = 0; r < h->config->rows; ++r) {
			if (h->glue->gpio_get(h->glue_arg, h->config->row_pins[r]))
				state |= (1 << ((r * h->config->cols) + c));
		}
		h->glue->gpio_set(h->glue_arg, h->config->col_pins[c], 0);
		h->glue->gpio_direction(h->glue_arg, h->config->col_pins[c], 0);
	}

	return(state);
}

/*****************************************************************************/

