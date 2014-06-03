/*
 * Copyright (C) 2014 Swift Navigation Inc.
 * Contact: Colin Beighley <colin@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/*
 * tutorial_implementation contains functions and definitions that are
 * implementation specific to this tutorial, to keep main.c as simple as possible.
 */

#include <stm32f4xx.h>

#define DO_EVERY(n, cmd) do { \
  static u32 do_every_count = 0; \
  if (do_every_count % (n) == 0) { \
    cmd; \
  } \
  do_every_count++; \
} while(0)

/* FIFO functions */
u8 fifo_empty(void);
u8 fifo_full(void);
u8 fifo_write(char c);
u8 fifo_read_char(char *c);
u32 fifo_read(u8 *buff, u32 n, void *context);

/* UART functions */
void usarts_setup(void);

/* LED functions */
void leds_set(void);
void leds_unset(void);
void leds_toggle(void);
void leds_setup(void);
