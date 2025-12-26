/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include "nimble/porting/nimble/include/os/os.h"
#include "nimble/porting/nimble/include/hal/hal_timer.h"
#include "nimble/porting/nimble/include/os/os_trace_api.h"

void TMR0_TimerInit(uint32_t t) {
	R32_TMR0_CNT_END = t;
	R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
	R8_TMR0_CTRL_MOD = RB_TMR_COUNT_EN;
}

#define	TMR0_ITCfg(s,f)				((s)?(R8_TMR0_INTER_EN|=f):(R8_TMR0_INTER_EN&=~f))		/* TMR0 corresponding interrupt bit on and off */


#define  TMR0_GetCurrentCount()		R32_TMR0_COUNT
#define CH592_HAL_TIMER_MAX     (4)

struct ch592_hal_timer {
	uint8_t tmr_enabled;
	uint8_t tmr_irq_num;
	uint8_t tmr_pad[2];
	uint32_t tmr_cntr; // Software counter extension (for >24-bit)
	uint32_t timer_isrs;
	uint32_t tmr_freq;
	void* tmr_reg; // Pointer to CH592 timer registers
	TAILQ_HEAD(hal_timer_qhead, hal_timer) hal_timer_q;
};

#if MYNEWT_VAL(TIMER_0)
struct ch592_hal_timer ch592_hal_timer0;
#endif
#if MYNEWT_VAL(TIMER_1)
struct ch592_hal_timer ch592_hal_timer1;
#endif
#if MYNEWT_VAL(TIMER_2)
struct ch592_hal_timer ch592_hal_timer2;
#endif
#if MYNEWT_VAL(TIMER_3)
struct ch592_hal_timer ch592_hal_timer3;
#endif

static const struct ch592_hal_timer* ch592_hal_timers[CH592_HAL_TIMER_MAX] = {
#if MYNEWT_VAL(TIMER_0)
	&ch592_hal_timer0,
#else
	NULL,
#endif
#if MYNEWT_VAL(TIMER_1)
	&ch592_hal_timer1,
#else
	NULL,
#endif
#if MYNEWT_VAL(TIMER_2)
	&ch592_hal_timer2,
#else
	NULL,
#endif
#if MYNEWT_VAL(TIMER_3)
	&ch592_hal_timer3
#else
	NULL
#endif
};

/* Resolve timer number into timer structure */
#define CH592_HAL_TIMER_RESOLVE(__n, __v)       \
    if ((__n) >= CH592_HAL_TIMER_MAX) {         \
        rc = EINVAL;                            \
        goto err;                               \
    }                                           \
    (__v) = (struct ch592_hal_timer *) ch592_hal_timers[(__n)];            \
    if ((__v) == NULL) {                        \
        rc = EINVAL;                            \
        goto err;                               \
    }

/**
 * Read hardware timer counter
 */
static uint32_t ch592_read_timer_cntr(struct ch592_hal_timer* bsptimer) {
	uint32_t tcntr = 0;
	tcntr          = TMR0_GetCurrentCount();

	return tcntr;
}

/**
 * Set hardware timer compare/capture value
 */
static void ch592_timer_set_ocmp(struct ch592_hal_timer* bsptimer, uint32_t expiry) {
	// TODO: Set CH592 timer compare value
	// Example:
	// - Disable compare interrupt
	// - Set compare register to expiry value
	// - Clear interrupt flags
	// - Enable compare interrupt
	// - Check if we already passed the expiry (force interrupt)
}

/**
 * Disable timer output compare interrupt
 */
static void ch592_timer_disable_ocmp(struct ch592_hal_timer* bsptimer) {
	// TODO: Disable CH592 timer compare interrupt
}

#if (MYNEWT_VAL(TIMER_0) || MYNEWT_VAL(TIMER_1) || MYNEWT_VAL(TIMER_2) || MYNEWT_VAL(TIMER_3))
/**
 * Check timer queue and process expired timers
 */
static void hal_timer_chk_queue(struct ch592_hal_timer* bsptimer) {
	uint32_t tcntr;
	os_sr_t sr;
	struct hal_timer* timer;

	OS_ENTER_CRITICAL(sr);
	while ((timer = TAILQ_FIRST(&bsptimer->hal_timer_q)) != NULL) {
		tcntr = ch592_read_timer_cntr(bsptimer);
		if ((int32_t)(tcntr - timer->expiry) >= 0) {
			TAILQ_REMOVE(&bsptimer->hal_timer_q, timer, link);
			timer->link.tqe_prev = NULL;
			timer->cb_func(timer->cb_arg);
		}
		else {
			break;
		}
	}

	/* Any timers left on queue? If so, we need to set compare */
	timer = TAILQ_FIRST(&bsptimer->hal_timer_q);
	if (timer) {
		ch592_timer_set_ocmp(bsptimer, timer->expiry);
	}
	else {
		ch592_timer_disable_ocmp(bsptimer);
	}
	OS_EXIT_CRITICAL(sr);
}
#endif

/**
 * Generic HAL timer interrupt handler
 */
static void hal_timer_irq_handler(struct ch592_hal_timer* bsptimer) {
	os_trace_isr_enter();

	// TODO: Check and clear interrupt flags

	++bsptimer->timer_isrs;

	hal_timer_chk_queue(bsptimer);

	os_trace_isr_exit();
}

/* Timer interrupt handlers - connect these to CH592 interrupt vectors */
#if MYNEWT_VAL(TIMER_0)
void ch592_timer0_irq_handler(void) {
	hal_timer_irq_handler(&ch592_hal_timer0);
}
#endif

#if MYNEWT_VAL(TIMER_1)
void ch592_timer1_irq_handler(void) {
	hal_timer_irq_handler(&ch592_hal_timer1);
}
#endif

#if MYNEWT_VAL(TIMER_2)
void ch592_timer2_irq_handler(void) {
	hal_timer_irq_handler(&ch592_hal_timer2);
}
#endif

#if MYNEWT_VAL(TIMER_3)
void ch592_timer3_irq_handler(void) {
	hal_timer_irq_handler(&ch592_hal_timer3);
}
#endif

/**
 * Initialize HAL timer
 */
int hal_timer_init(int timer_num, void* cfg) {
	int rc;
	struct ch592_hal_timer* bsptimer;

	CH592_HAL_TIMER_RESOLVE(timer_num, bsptimer);

	if (bsptimer->tmr_enabled) {
		rc = EINVAL;
		goto err;
	}

	// TODO: Initialize CH592 timer hardware
	// - Map timer_num to CH592 TMR peripheral (TMR0-TMR3)
	// - Store timer register base address in bsptimer->tmr_reg
	// - Store IRQ number in bsptimer->tmr_irq_num
	// - Disable and configure interrupt priority
	// - Set interrupt vector

	return 0;

err:
	return rc;
}

/**
 * Configure timer frequency and start it
 */
int hal_timer_config(int timer_num, uint32_t freq_hz) {
	int rc;
	os_sr_t sr;
	struct ch592_hal_timer* bsptimer;

	CH592_HAL_TIMER_RESOLVE(timer_num, bsptimer);

	if (bsptimer->tmr_enabled || (freq_hz == 0) || (bsptimer->tmr_reg == NULL)) {
		rc = EINVAL;
		goto err;
	}

	bsptimer->tmr_freq    = freq_hz;
	bsptimer->tmr_enabled = 1;

	OS_ENTER_CRITICAL(sr);

	// TODO: Configure CH592 timer
	// - Stop timer
	// - Set prescaler/clock source for desired frequency (32768 Hz for BLE)
	// - Clear counter
	// - Start timer
	// - Enable interrupts

	OS_EXIT_CRITICAL(sr);

	return 0;

err:
	return rc;
}

/**
 * De-initialize HAL timer
 */
int hal_timer_deinit(int timer_num) {
	int rc;
	os_sr_t sr;
	struct ch592_hal_timer* bsptimer;

	rc = 0;
	CH592_HAL_TIMER_RESOLVE(timer_num, bsptimer);

	OS_ENTER_CRITICAL(sr);

	// TODO: Stop and disable CH592 timer

	bsptimer->tmr_enabled = 0;
	bsptimer->tmr_reg     = NULL;

	OS_EXIT_CRITICAL(sr);

err:
	return rc;
}

/**
 * Get timer resolution in nanoseconds
 */
uint32_t hal_timer_get_resolution(int timer_num) {
	int rc;
	uint32_t resolution;
	struct ch592_hal_timer* bsptimer;

	CH592_HAL_TIMER_RESOLVE(timer_num, bsptimer);

	resolution = 1000000000 / bsptimer->tmr_freq;
	return resolution;

err:
	rc = 0;
	return rc;
}

/**
 * Read current timer counter value
 */
uint32_t hal_timer_read(int timer_num) {
	int rc;
	uint32_t tcntr;
	struct ch592_hal_timer* bsptimer;

	CH592_HAL_TIMER_RESOLVE(timer_num, bsptimer);

	tcntr = ch592_read_timer_cntr(bsptimer);

	return tcntr;

err:
	assert(0);
	rc = 0;
	return rc;
}

/**
 * Blocking delay for n ticks
 */
int hal_timer_delay(int timer_num, uint32_t ticks) {
	uint32_t until;

	until = hal_timer_read(timer_num) + ticks;
	while ((int32_t)(hal_timer_read(timer_num) - until) <= 0) {
		/* Loop here till finished */
	}

	return 0;
}

/**
 * Set timer callback
 */
int hal_timer_set_cb(
	int timer_num,
	struct hal_timer* timer,
	hal_timer_cb cb_func,
	void* arg
) {
	int rc;
	struct ch592_hal_timer* bsptimer;

	CH592_HAL_TIMER_RESOLVE(timer_num, bsptimer);

	timer->cb_func       = cb_func;
	timer->cb_arg        = arg;
	timer->link.tqe_prev = NULL;
	timer->bsp_timer     = bsptimer;

	rc = 0;

err:
	return rc;
}

/**
 * Start timer relative to current time
 */
int hal_timer_start(struct hal_timer* timer, uint32_t ticks) {
	int rc;
	uint32_t tick;
	struct ch592_hal_timer* bsptimer;

	bsptimer = (struct ch592_hal_timer*)timer->bsp_timer;
	tick     = ch592_read_timer_cntr(bsptimer) + ticks;
	rc       = hal_timer_start_at(timer, tick);
	return rc;
}

/**
 * Start timer at absolute tick value
 */
int hal_timer_start_at(struct hal_timer* timer, uint32_t tick) {
	os_sr_t sr;
	struct hal_timer* entry;
	struct ch592_hal_timer* bsptimer;

	if ((timer == NULL) || (timer->link.tqe_prev != NULL) ||
		(timer->cb_func == NULL)) {
		return EINVAL;
	}
	bsptimer      = (struct ch592_hal_timer*)timer->bsp_timer;
	timer->expiry = tick;

	OS_ENTER_CRITICAL(sr);

	if (TAILQ_EMPTY(&bsptimer->hal_timer_q)) {
		TAILQ_INSERT_HEAD(&bsptimer->hal_timer_q, timer, link);
	}
	else {
		TAILQ_FOREACH(entry, &bsptimer->hal_timer_q, link) {
			if ((int32_t)(timer->expiry - entry->expiry) < 0) {
				TAILQ_INSERT_BEFORE(entry, timer, link);
				break;
			}
		}
		if (!entry) {
			TAILQ_INSERT_TAIL(&bsptimer->hal_timer_q, timer, link);
		}
	}

	/* If this is the head, we need to set new compare value */
	if (timer == TAILQ_FIRST(&bsptimer->hal_timer_q)) {
		ch592_timer_set_ocmp(bsptimer, timer->expiry);
	}

	OS_EXIT_CRITICAL(sr);

	return 0;
}

/**
 * Stop a timer
 */
int hal_timer_stop(struct hal_timer* timer) {
	os_sr_t sr;
	int reset_ocmp;
	struct hal_timer* entry;
	struct ch592_hal_timer* bsptimer;

	if (timer == NULL) {
		return EINVAL;
	}

	bsptimer = (struct ch592_hal_timer*)timer->bsp_timer;

	OS_ENTER_CRITICAL(sr);

	if (timer->link.tqe_prev != NULL) {
		reset_ocmp = 0;
		if (timer == TAILQ_FIRST(&bsptimer->hal_timer_q)) {
			entry      = TAILQ_NEXT(timer, link);
			reset_ocmp = 1;
		}
		TAILQ_REMOVE(&bsptimer->hal_timer_q, timer, link);
		timer->link.tqe_prev = NULL;
		if (reset_ocmp) {
			if (entry) {
				ch592_timer_set_ocmp(
					(struct ch592_hal_timer*)entry->bsp_timer,
					entry->expiry
				);
			}
			else {
				ch592_timer_disable_ocmp(bsptimer);
			}
		}
	}

	OS_EXIT_CRITICAL(sr);

	return 0;
}
