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

#include "syscfg/syscfg.h"

#include <assert.h>
#include <stddef.h>
#include <string.h>

#include "nimble/nimble/include/nimble/nimble_npl.h"

// CH592 specific includes
// #include "CH59x_common.h"

static void* radio_isr_addr;
static void* rng_isr_addr;
static void* rtc_isr_addr; // CH592 uses RTC for BLE timing

static inline bool in_isr(void) {
	// TODO: Implement CH592 ISR detection
	// Check if currently executing in interrupt context
	// Example: return __get_IPSR() != 0;  // ARM Cortex-M
	return (*PFIC->ISR & 0xFF) != 0;
}

// CH592 interrupt handlers
void RADIO_IRQHandler(void) {
	// TODO: Add CH592 radio interrupt handling
	((void (*)(void))radio_isr_addr)();
}

void RNG_IRQHandler(void) {
	// TODO: Add CH592 RNG interrupt handling
	((void (*)(void))rng_isr_addr)();
}

void RTC_IRQHandler(void) {
	// TODO: Add CH592 RTC interrupt handling
	((void (*)(void))rtc_isr_addr)();
}

/* This is called by NimBLE radio driver to set interrupt handlers */
void
npl_freertos_hw_set_isr(int irqn, void (* addr)(void)) {
	// TODO: Map CH592 IRQ numbers - check CH59x_common.h or startup file
	switch (irqn) {
		case 0: // TODO: Replace with CH592_RADIO_IRQn or BLE_IRQn
			radio_isr_addr = addr;
			// TODO: PFIC_EnableIRQ(RADIO_IRQn);
			// TODO: PFIC_SetPriority(RADIO_IRQn, priority);
			break;
		case 1: // TODO: Replace with CH592_RNG_IRQn
			rng_isr_addr = addr;
			// TODO: PFIC_EnableIRQ(RNG_IRQn);
			break;
		case 2: // TODO: Replace with CH592_RTC_IRQn
			rtc_isr_addr = addr;
			// TODO: PFIC_EnableIRQ(RTC_IRQn);
			break;
	}
}

struct ble_npl_event* npl_freertos_eventq_get(struct ble_npl_eventq* evq, ble_npl_time_t tmo) {
	struct ble_npl_event* ev = NULL;
	BaseType_t woken;
	BaseType_t ret;

	if (in_isr()) {
		assert(tmo == 0);
		ret = xQueueReceiveFromISR(evq->q, &ev, &woken);
		portYIELD_FROM_ISR(woken);
	}
	else {
		ret = xQueueReceive(evq->q, &ev, tmo);
	}
	assert(ret == pdPASS || ret == errQUEUE_EMPTY);

	if (ev) {
		ev->queued = false;
	}

	return ev;
}

void npl_freertos_eventq_put(struct ble_npl_eventq* evq, struct ble_npl_event* ev) {
	BaseType_t woken;
	BaseType_t ret;

	if (ev->queued) {
		return;
	}

	ev->queued = true;

	if (in_isr()) {
		ret = xQueueSendToBackFromISR(evq->q, &ev, &woken);
		portYIELD_FROM_ISR(woken);
	}
	else {
		ret = xQueueSendToBack(evq->q, &ev, portMAX_DELAY);
	}

	assert(ret == pdPASS);
}

void npl_freertos_eventq_remove(
	struct ble_npl_eventq* evq,
	struct ble_npl_event* ev
) {
	struct ble_npl_event* tmp_ev;
	BaseType_t ret;
	int i;
	int count;
	BaseType_t woken, woken2;

	if (!ev->queued) {
		return;
	}

	if (in_isr()) {
		woken = pdFALSE;

		count = uxQueueMessagesWaitingFromISR(evq->q);
		for (i = 0; i < count; i++) {
			ret = xQueueReceiveFromISR(evq->q, &tmp_ev, &woken2);
			assert(ret == pdPASS);
			woken |= woken2;

			if (tmp_ev == ev) {
				continue;
			}

			ret = xQueueSendToBackFromISR(evq->q, &tmp_ev, &woken2);
			assert(ret == pdPASS);
			woken |= woken2;
		}

		portYIELD_FROM_ISR(woken);
	}
	else {
		vPortEnterCritical();
		count = uxQueueMessagesWaiting(evq->q);
		for (i = 0; i < count; i++) {
			ret = xQueueReceive(evq->q, &tmp_ev, 0);
			assert(ret == pdPASS);

			if (tmp_ev == ev) {
				continue;
			}

			ret = xQueueSendToBack(evq->q, &tmp_ev, 0);
			assert(ret == pdPASS);
		}
		vPortExitCritical();
	}

	ev->queued = 0;
}

ble_npl_error_t npl_freertos_mutex_init(struct ble_npl_mutex* mu) {
	if (!mu) {
		return BLE_NPL_INVALID_PARAM;
	}

	mu->handle = xSemaphoreCreateRecursiveMutex();
	assert(mu->handle);

	return BLE_NPL_OK;
}

ble_npl_error_t npl_freertos_mutex_deinit(struct ble_npl_mutex* mu) {
	if (!mu) {
		return BLE_NPL_INVALID_PARAM;
	}

	if (mu->handle) {
		vSemaphoreDelete(mu->handle);
	}

	return BLE_NPL_OK;
}

ble_npl_error_t npl_freertos_mutex_pend(struct ble_npl_mutex* mu, ble_npl_time_t timeout) {
	BaseType_t ret;

	if (!mu) {
		return BLE_NPL_INVALID_PARAM;
	}

	assert(mu->handle);

	if (in_isr()) {
		ret = pdFAIL;
		assert(0);
	}
	else {
		ret = xSemaphoreTakeRecursive(mu->handle, timeout);
	}

	return ret == pdPASS ? BLE_NPL_OK : BLE_NPL_TIMEOUT;
}

ble_npl_error_t npl_freertos_mutex_release(struct ble_npl_mutex* mu) {
	if (!mu) {
		return BLE_NPL_INVALID_PARAM;
	}

	assert(mu->handle);

	if (in_isr()) {
		assert(0);
	}
	else {
		if (xSemaphoreGiveRecursive(mu->handle) != pdPASS) {
			return BLE_NPL_BAD_MUTEX;
		}
	}

	return BLE_NPL_OK;
}

ble_npl_error_t npl_freertos_sem_init(struct ble_npl_sem* sem, uint16_t tokens) {
	if (!sem) {
		return BLE_NPL_INVALID_PARAM;
	}

	sem->handle = xSemaphoreCreateCounting(128, tokens);
	assert(sem->handle);

	return BLE_NPL_OK;
}

ble_npl_error_t npl_freertos_sem_deinit(struct ble_npl_sem* sem) {
	if (!sem) {
		return BLE_NPL_INVALID_PARAM;
	}

	if (sem->handle) {
		vSemaphoreDelete(sem->handle);
	}

	return BLE_NPL_OK;
}

ble_npl_error_t npl_freertos_sem_pend(struct ble_npl_sem* sem, ble_npl_time_t timeout) {
	BaseType_t woken;
	BaseType_t ret;

	if (!sem) {
		return BLE_NPL_INVALID_PARAM;
	}

	assert(sem->handle);

	if (in_isr()) {
		assert(timeout == 0);
		ret = xSemaphoreTakeFromISR(sem->handle, &woken);
		portYIELD_FROM_ISR(woken);
	}
	else {
		ret = xSemaphoreTake(sem->handle, timeout);
	}

	return ret == pdPASS ? BLE_NPL_OK : BLE_NPL_TIMEOUT;
}

ble_npl_error_t npl_freertos_sem_release(struct ble_npl_sem* sem) {
	BaseType_t ret;
	BaseType_t woken;

	if (!sem) {
		return BLE_NPL_INVALID_PARAM;
	}

	assert(sem->handle);

	if (in_isr()) {
		ret = xSemaphoreGiveFromISR(sem->handle, &woken);
		portYIELD_FROM_ISR(woken);
	}
	else {
		ret = xSemaphoreGive(sem->handle);
	}

	assert(ret == pdPASS);
	return BLE_NPL_OK;
}

static void os_callout_timer_cb(TimerHandle_t timer) {
	struct ble_npl_callout* co;

	co = (struct ble_npl_callout*)pvTimerGetTimerID(timer);
	assert(co);

	if (co->evq) {
		ble_npl_eventq_put(co->evq, &co->ev);
	}
	else {
		co->ev.fn(&co->ev);
	}
}

int npl_freertos_callout_init(
	struct ble_npl_callout* co,
	struct ble_npl_eventq* evq,
	ble_npl_event_fn* ev_cb,
	void* ev_arg
) {
	if (co->handle == NULL) {
		co->handle = xTimerCreate("co", 1, pdFALSE, co, os_callout_timer_cb);
		if (co->handle == NULL) {
			return -1;
		}
	}

	co->evq = evq;
	ble_npl_event_init(&co->ev, ev_cb, ev_arg);

	return 0;
}

void npl_freertos_callout_deinit(struct ble_npl_callout* co) {
	if (!co->handle) {
		return;
	}

	xTimerDelete(co->handle, portMAX_DELAY);
	ble_npl_event_deinit(&co->ev);
	memset(co, 0, sizeof(struct ble_npl_callout));
}

ble_npl_error_t npl_freertos_callout_reset(struct ble_npl_callout* co, ble_npl_time_t ticks) {
	BaseType_t woken1, woken2, woken3;

	if (ticks == 0) {
		ticks = 1;
	}

	if (in_isr()) {
		xTimerStopFromISR(co->handle, &woken1);
		xTimerChangePeriodFromISR(co->handle, ticks, &woken2);
		xTimerResetFromISR(co->handle, &woken3);
		portYIELD_FROM_ISR(woken1 || woken2 || woken3);
	}
	else {
		xTimerStop(co->handle, portMAX_DELAY);
		xTimerChangePeriod(co->handle, ticks, portMAX_DELAY);
		xTimerReset(co->handle, portMAX_DELAY);
	}

	return BLE_NPL_OK;
}

void npl_freertos_callout_stop(struct ble_npl_callout* co) {
	if (!co->handle) {
		return;
	}
	xTimerStop(co->handle, portMAX_DELAY);
}

bool npl_freertos_callout_is_active(struct ble_npl_callout* co) {
	return xTimerIsTimerActive(co->handle) == pdTRUE &&
		xTimerGetExpiryTime(co->handle) > xTaskGetTickCountFromISR();
}

ble_npl_time_t npl_freertos_callout_get_ticks(struct ble_npl_callout* co) {
	return xTimerGetExpiryTime(co->handle);
}

ble_npl_time_t npl_freertos_callout_remaining_ticks(
	struct ble_npl_callout* co,
	ble_npl_time_t now
) {
	ble_npl_time_t rt;
	uint32_t exp;

	exp = xTimerGetExpiryTime(co->handle);

	if (exp > now) {
		rt = exp - now;
	}
	else {
		rt = 0;
	}

	return rt;
}

ble_npl_error_t npl_freertos_time_ms_to_ticks(uint32_t ms, ble_npl_time_t* out_ticks) {
	uint64_t ticks;

	ticks = ((uint64_t)ms * configTICK_RATE_HZ) / 1000;
	if (ticks > UINT32_MAX) {
		return BLE_NPL_EINVAL;
	}

	*out_ticks = ticks;

	return BLE_NPL_OK;
}

ble_npl_error_t npl_freertos_time_ticks_to_ms(ble_npl_time_t ticks, uint32_t* out_ms) {
	uint64_t ms;

	ms = ((uint64_t)ticks * 1000) / configTICK_RATE_HZ;
	if (ms > UINT32_MAX) {
		return BLE_NPL_EINVAL;
	}

	*out_ms = ms;

	return BLE_NPL_OK;
}
