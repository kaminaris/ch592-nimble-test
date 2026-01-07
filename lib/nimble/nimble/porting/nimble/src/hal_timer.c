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

static inline void NVIC_DisableIRQ(IRQn_Type IRQn) {
	NVIC->IRER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}

static inline void NVIC_EnableIRQ(IRQn_Type IRQn) {
	NVIC->IENR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}

static inline void NVIC_SetPriority(IRQn_Type IRQn, uint8_t priority) {
	NVIC->IPRIOR[(uint32_t)(IRQn)] = priority;
}

/* IRQ prototype */
typedef void (* hal_timer_irq_handler_t)(void);

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

#ifndef CH592_TIMER_MASK
#define CH592_TIMER_MASK  (0x3FFFFFFu)  // 26-bit mask, not 24-bit
#endif

/**
 * Read hardware timer counter
 */
static uint32_t ch592_read_timer_cntr(struct ch592_hal_timer* bsptimer) {
	uint32_t tcntr;
	uint32_t prev_low;

	// Read raw 26-bit hardware counter
	tcntr = TMR0_GetCurrentCount() & CH592_TIMER_MASK;
	bsptimer->tmr_cntr = tcntr; // TODO: we probably dont need 32bit anyways

	// Extract previous low 26 bits from software counter
	// prev_low = bsptimer->tmr_cntr & CH592_TIMER_MASK;
	//
	// // Detect wraparound by comparing current vs previous hardware value
	// if (tcntr < prev_low) {
	// 	// Wraparound: increment high bits, keep new low bits
	// 	bsptimer->tmr_cntr = tcntr + (bsptimer->tmr_cntr & ~CH592_TIMER_MASK) + (CH592_TIMER_MASK + 1);
	// } else {
	// 	// No wraparound: keep high bits, update low bits
	// 	bsptimer->tmr_cntr = tcntr + (bsptimer->tmr_cntr & ~CH592_TIMER_MASK);
	// }

	return bsptimer->tmr_cntr;
}

/**
 * nrf timer set ocmp
 *
 * Set the OCMP used by the timer to the desired expiration tick
 *
 * NOTE: Must be called with interrupts disabled.
 *
 * @param timer Pointer to timer.
 */
static void ch592_timer_set_ocmp(struct ch592_hal_timer* bsptimer, uint32_t expiry) {
	// uint32_t current = ch592_read_timer_cntr(bsptimer);
	//
	// // If already expired, fire ISR immediately
	// if ((int32_t)(current - expiry) >= 0) {
	// 	// Trigger interrupt manually
	// 	R8_TMR0_INT_FLAG = RB_TMR_IF_CYC_END;
	// 	TMR0_ITCfg(1, RB_TMR_IE_CYC_END);
	// 	return;
	// }

	TMR0_ITCfg(0, RB_TMR_IE_CYC_END);

	// Write only the lower 24 bits of the absolute expiry
	R32_TMR0_CNT_END = expiry & CH592_TIMER_MASK;

	R8_TMR0_INT_FLAG = RB_TMR_IF_CYC_END;
	TMR0_ITCfg(1, RB_TMR_IE_CYC_END);
}


/**
 * Disable timer output compare interrupt
 */
static void ch592_timer_disable_ocmp(struct ch592_hal_timer* bsptimer) {
	if (bsptimer == NULL) {
		return;
	}

	/* Currently only TMR0 support implemented in this HAL file.
	   Disable the cycle-end (compare) interrupt and clear any pending flag. */
	TMR0_ITCfg(0, RB_TMR_IE_CYC_END);    /* disable interrupt */
	R8_TMR0_INT_FLAG = RB_TMR_IF_CYC_END;/* clear pending flag */
}

#if (MYNEWT_VAL(TIMER_0) || MYNEWT_VAL(TIMER_1) || MYNEWT_VAL(TIMER_2) || MYNEWT_VAL(TIMER_3))
/**
 * Check timer queue and process expired timers
 */
volatile uint32_t hal_timer_chk_queue_hits = 0;
volatile uint32_t hal_timer_chk_queue_execs = 0;
volatile uint32_t halTimerLastExpiry = 0;
volatile uint32_t halTimerCheckCnt = 0;

static void hal_timer_chk_queue(struct ch592_hal_timer* bsptimer) {
	uint32_t tcntr;
	os_sr_t sr;
	struct hal_timer* timer;
	hal_timer_chk_queue_hits++;
	// TODO: investigate this, it was producing stack corruption
	OS_ENTER_CRITICAL(sr);
	while ((timer = TAILQ_FIRST(&bsptimer->hal_timer_q)) != NULL) {
		tcntr = ch592_read_timer_cntr(bsptimer);
		halTimerLastExpiry = timer->expiry;
		halTimerCheckCnt = tcntr;
		if ((int32_t)(tcntr - timer->expiry) >= 0) {
			TAILQ_REMOVE(&bsptimer->hal_timer_q, timer, link);
			timer->link.tqe_prev = NULL;
			timer->cb_func(timer->cb_arg);
			hal_timer_chk_queue_execs++;

			tcntr = ch592_read_timer_cntr(bsptimer);
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
 * hal timer irq handler
 *
 * Generic HAL timer irq handler.
 *
 * @param tmr
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

__HIGH_CODE
// __INTERRUPT
// void TMR0_IRQHandler(void) __attribute__((interrupt));
void TMR0_IRQHandler(void) {
	// Call the generic timer interrupt handler
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	ch592_timer0_irq_handler();
	R8_TMR0_INT_FLAG = RB_TMR_IF_CYC_END;
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * hal timer init
 *
 * Initialize platform specific timer items
 *
 * @param timer_num     Timer number to initialize
 * @param cfg           Pointer to platform specific configuration
 *
 * @return int          0: success; error code otherwise
 */
int hal_timer_init(int timer_num, void* cfg) {
	int rc;
	uint8_t irq_num;
	struct ch592_hal_timer* bsptimer;

	CH592_HAL_TIMER_RESOLVE(timer_num, bsptimer);

	/* If timer is enabled do not allow init */
	if (bsptimer->tmr_enabled) {
		rc = EINVAL;
		goto err;
	}

	switch (timer_num) {
#if MYNEWT_VAL(TIMER_0)
		case 0:
			irq_num = TMR0_IRQn;
			break;
#endif
#if MYNEWT_VAL(TIMER_1)
		case 1:
			irq_num = TIMER1_IRQn;
			hwtimer = NRF_TIMER1;
			irq_isr = nrf52_timer1_irq_handler;
			break;
#endif
#if MYNEWT_VAL(TIMER_2)
		case 2:
			irq_num = TIMER2_IRQn;
			hwtimer = NRF_TIMER2;
			irq_isr = nrf52_timer2_irq_handler;
			break;
#endif
#if MYNEWT_VAL(TIMER_3)
		case 3:
			irq_num = TIMER3_IRQn;
			hwtimer = NRF_TIMER3;
			irq_isr = nrf52_timer3_irq_handler;
			break;
#endif
		default:
			irq_num = 0;
			break;
	}

	if (irq_num == 0) {
		rc = EINVAL;
		goto err;
	}

	// bsptimer->tmr_reg = hwtimer;
	bsptimer->tmr_irq_num = irq_num;
	bsptimer->timer_isrs = 0;
	bsptimer->tmr_cntr = 0;  // Initialize software counter to 0

	TAILQ_INIT(&bsptimer->hal_timer_q);
	/* Disable IRQ, set priority and set vector in table */
	NVIC_DisableIRQ(irq_num);
#ifndef RIOT_VERSION
	NVIC_SetPriority(irq_num, 1);
#endif
#ifdef MYNEWT
	NVIC_SetVector(irq_num, (uint32_t)irq_isr);
#else
	// NVIC_EnableIRQ(TMR0_IRQn);
#endif

	return 0;

err:
	return rc;
}

uint32_t GetSysClock2(void) {
	uint16_t rev;

	rev = R32_CLK_SYS_CFG & 0xff;
	if ((rev & 0x40) == (0 << 6)) {
		// 32M���з�Ƶ
		return (32000000 / (rev & 0x1f));
	}
	else if ((rev & RB_CLK_SYS_MOD) == (1 << 6)) {
		// PLL���з�Ƶ
		return (480000000 / (rev & 0x1f));
	}
	else {
		// 32K����Ƶ
		return (32000);
	}
}

/**
 * hal timer config
 *
 * Configure a timer to run at the desired frequency. This starts the timer.
 *
 * @param timer_num
 * @param freq_hz
 *
 * @return int
 */
int hal_timer_config(int timer_num, uint32_t freq_hz) {
	int rc;
	os_sr_t sr;
	struct ch592_hal_timer* bsptimer;

	CH592_HAL_TIMER_RESOLVE(timer_num, bsptimer);

	if (bsptimer->tmr_enabled || (freq_hz == 0)) {
		rc = EINVAL;
		goto err;
	}

	bsptimer->tmr_freq    = freq_hz;
	bsptimer->tmr_enabled = 1;

	OS_ENTER_CRITICAL(sr);

	uint32_t sys_clk = GetSysClock2(); // Get system clock frequency
	uint32_t cnt_end = sys_clk / freq_hz; // Calculate counter end value
	TMR0_TimerInit(cnt_end);
	TMR0_ITCfg(1, RB_TMR_IE_CYC_END); // Enable cycle end interrupt
	PFIC_EnableIRQ(TMR0_IRQn); // Enable in NVIC
	// This may not be good idea? or ISR handling is bad?
	//NVIC_EnableIRQ(TMR0_IRQn);
	// bsptimer->tmr_cntr = TMR0_GetCurrentCount() & CH592_TIMER_MASK;
	OS_EXIT_CRITICAL(sr);

	return 0;

err:
	return rc;
}

/**
 * hal timer deinit
 *
 * De-initialize a HW timer.
 *
 * @param timer_num
 *
 * @return int
 */
int hal_timer_deinit(int timer_num) {
	int rc;
	os_sr_t sr;
	struct ch592_hal_timer* bsptimer;

	rc = 0;
	CH592_HAL_TIMER_RESOLVE(timer_num, bsptimer);

	OS_ENTER_CRITICAL(sr);

	/* Disable NVIC for this timer */
	NVIC_DisableIRQ(bsptimer->tmr_irq_num);
	if (bsptimer->tmr_irq_num == TMR0_IRQn) {
		TMR0_ITCfg(0, RB_TMR_IE_CYC_END);    /* disable compare/cycle-end interrupt */
		R8_TMR0_INT_FLAG = RB_TMR_IF_CYC_END;/* clear pending flag */
		R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR; /* stop and clear timer */
	}

	bsptimer->tmr_enabled = 0;
	bsptimer->tmr_reg     = NULL;

	OS_EXIT_CRITICAL(sr);

err:
	return rc;
}

/**
 * hal timer get resolution
 *
 * Get the resolution of the timer. This is the timer period, in nanoseconds
 *
 * @param timer_num
 *
 * @return uint32_t The
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
 * hal timer read
 *
 * Returns the timer counter. NOTE: if the timer is a 16-bit timer, only
 * the lower 16 bits are valid. If the timer is a 64-bit timer, only the
 * low 32-bits are returned.
 *
 * @return uint32_t The timer counter register.
 */
uint32_t hal_timer_read(int timer_num) {
	int rc;
	uint32_t tcntr;
	struct ch592_hal_timer* bsptimer;

	CH592_HAL_TIMER_RESOLVE(timer_num, bsptimer);

	tcntr = ch592_read_timer_cntr(bsptimer);

	return tcntr;

    /* Assert here since there is no invalid return code */
err:
	assert(0);
	rc = 0;
	return rc;
}

/**
 * hal timer delay
 *
 * Blocking delay for n ticks
 *
 * @param timer_num
 * @param ticks
 *
 * @return int 0 on success; error code otherwise.
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
 *
 * Initialize the HAL timer structure with the callback and the callback
 * argument. Also initializes the HW specific timer pointer.
 *
 * @param cb_func
 *
 * @return int
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

volatile uint32_t hal_timer_queue_inserts = 0;
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
		hal_timer_queue_inserts++;
	}
	else {
		TAILQ_FOREACH(entry, &bsptimer->hal_timer_q, link) {
			if ((int32_t)(timer->expiry - entry->expiry) < 0) {
				TAILQ_INSERT_BEFORE(entry, timer, link);
				hal_timer_queue_inserts++;
				break;
			}
		}
		if (!entry) {
			TAILQ_INSERT_TAIL(&bsptimer->hal_timer_q, timer, link);
			hal_timer_queue_inserts++;
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
 * hal timer stop
 *
 * Stop a timer.
 *
 * @param timer
 *
 * @return int
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
            /* If first on queue, we will need to reset OCMP */
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
