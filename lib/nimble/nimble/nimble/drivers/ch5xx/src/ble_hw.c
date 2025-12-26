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

#include <stdint.h>
#include <assert.h>
#include <string.h>
#include "syscfg/syscfg.h"
#include "nimble/porting/nimble/include/os/os.h"
#include "nimble/nimble/include/nimble/ble.h"
#include "nimble/nimble/include/nimble/nimble_opt.h"
#include "nimble/nimble/controller/include/controller/ble_hw.h"
// #include "../include/ble/iSLER.h"

/* Total number of resolving list elements - adjust based on CH592 capabilities */
#define BLE_HW_RESOLV_LIST_SIZE     (16)

/* Bitmask tracking which whitelist entries are active */
static uint8_t g_ble_hw_whitelist_mask;

/* Random number generator callback function pointer */
ble_rng_isr_cb_t g_ble_rng_isr_cb;

#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LL_PRIVACY)
/* Define how many IRK entries CH592 can support */
#if (MYNEWT_VAL(BLE_LL_RESOLV_LIST_SIZE) < 16)
#define CH592_IRK_LIST_ENTRIES    (MYNEWT_VAL(BLE_LL_RESOLV_LIST_SIZE))
#else
#define CH592_IRK_LIST_ENTRIES    (16)
#endif

/* IRK list storage - each entry is 16 bytes (128-bit key) */
uint32_t g_ch592_irk_list[CH592_IRK_LIST_ENTRIES * 4];

/* Current number of IRKs in the list */
uint8_t g_ch592_num_irks;
#endif

/**
 * Get the public device address from CH592 hardware.
 * Check if CH592 has factory-programmed public address in OTP/EFUSE/similar.
 *
 * @param addr Pointer to store the retrieved address
 * @return 0 on success, -1 if no public address available
 */
#define ROM_CFG_MAC_ADDR	0x7F018
int ble_hw_get_public_addr(ble_addr_t* addr) {
	uint32_t addressHigh = *(vu32*)(ROM_CFG_MAC_ADDR + 4);
	uint32_t addressLow = *(vu32*)ROM_CFG_MAC_ADDR;

	memcpy(addr->val, &addressLow, 4);
	memcpy(&addr->val[4], &addressHigh, 2);

	addr->val[5] |= 0xc0;
	addr->type   = BLE_ADDR_PUBLIC;

	return 0;
}

/**
 * Get a random static address from CH592 hardware.
 * Generate or retrieve a random static address (must have top 2 bits set to 11).
 *
 * @param addr Pointer to store the generated/retrieved address
 * @return 0 on success, -1 if not available
 */

int ble_hw_get_static_addr(ble_addr_t* addr) {
	return -1;
}

/**
 * Clear the hardware whitelist.
 * Reset all whitelist entries and mask.
 */
void ble_hw_whitelist_clear(void) {
	// TODO: Clear CH592 hardware whitelist registers (if available)
	// TODO: Otherwise maintain software whitelist for filtering
	g_ble_hw_whitelist_mask = 0;
}

/**
 * Add a device to the hardware whitelist.
 * Store address for filtering incoming packets.
 *
 * @param addr 6-byte Bluetooth address
 * @param addr_type BLE_ADDR_PUBLIC or BLE_ADDR_RANDOM
 * @return BLE_ERR_SUCCESS or BLE_ERR_MEM_CAPACITY if full
 */
int ble_hw_whitelist_add(const uint8_t* addr, uint8_t addr_type) {
	// TODO: Find free slot in whitelist (use g_ble_hw_whitelist_mask)
	// TODO: Program CH592 hardware filter registers with address
	// TODO: Mark address type (public/random) in hardware
	// TODO: Update g_ble_hw_whitelist_mask |= (1 << slot)
	return BLE_ERR_MEM_CAPACITY;
}

/**
 * Remove a device from the hardware whitelist.
 *
 * @param addr 6-byte Bluetooth address to remove
 * @param addr_type BLE_ADDR_PUBLIC or BLE_ADDR_RANDOM
 */
void ble_hw_whitelist_rmv(const uint8_t* addr, uint8_t addr_type) {
	// TODO: Search whitelist for matching address and type
	// TODO: Clear the hardware filter entry
	// TODO: Update g_ble_hw_whitelist_mask &= ~(1 << slot)
}

/**
 * Get the maximum whitelist size supported by CH592 hardware.
 *
 * @return Number of whitelist entries (typically 8-16)
 */
uint8_t ble_hw_whitelist_size(void) {
	// TODO: Return actual CH592 whitelist capacity
	// return BLE_HW_WHITE_LIST_SIZE;
	return 8;
}

/**
 * Enable hardware whitelist filtering.
 * Activate the configured whitelist entries.
 */
void ble_hw_whitelist_enable(void) {
	// TODO: Set CH592 hardware register bit to enable whitelist filtering
	// TODO: Use g_ble_hw_whitelist_mask to enable only active entries
}

/**
 * Disable hardware whitelist filtering.
 * Allow all devices regardless of whitelist.
 */
void ble_hw_whitelist_disable(void) {
	// TODO: Clear CH592 hardware register bit to disable whitelist filtering
}

/**
 * Check if received packet matched a whitelisted device.
 *
 * @return 1 if match occurred, 0 otherwise
 */
int ble_hw_whitelist_match(void) {
	// TODO: Read CH592 hardware status register for whitelist match flag
	// TODO: Return 1 if EVENTS_DEVMATCH or equivalent is set
	return 0;
}

/**
 * Encrypt a 16-byte block using AES-128.
 * Used for BLE security (pairing, encryption).
 *
 * @param ecb Pointer to encryption block structure containing:
 *            - key[16]: AES-128 key
 *            - cleartext[16]: Input plaintext
 *            - ciphertext[16]: Output ciphertext
 * @return 0 on success, -1 on error
 */
int ble_hw_encrypt_block(struct ble_encryption_block* ecb) {
	// TODO: CH592 has AES hardware accelerator but i couldnt find docs
	// TODO: Program key, start encryption, wait for completion
	// TODO: Copy result to ecb->ciphertext
	return -1;
}

/**
 * Random number generator interrupt handler.
 * Called when RNG has a new random value ready.
 */
static void ble_rng_isr(void) {
	uint8_t rnum;

	// os_trace_isr_enter();

	// TODO: as far as i can tell CH5xx doesnt have an RNG peripheral, would probably need to implement a software RNG
	if (g_ble_rng_isr_cb == NULL) {
		// os_trace_isr_exit();
		return;
	}

	// os_trace_isr_exit();
}

/**
 * Initialize the random number generator.
 *
 * @param cb Callback function to call when random data ready (can be NULL)
 * @param bias Enable bias correction (1) or not (0)
 * @return 0 on success
 */
int ble_hw_rng_init(ble_rng_isr_cb_t cb, int bias) {
	// TODO: as far as i can tell CH5xx doesnt have an RNG peripheral, would probably need to implement a software RNG
	g_ble_rng_isr_cb = cb;
	return 0;
}

/**
 * Start the random number generator.
 * Begin generating random numbers.
 *
 * @return 0 on success
 */
int ble_hw_rng_start(void) {
	// TODO: as far as i can tell CH5xx doesnt have an RNG peripheral, would probably need to implement a software RNG
	return 0;
}

/**
 * Stop the random number generator.
 *
 * @return 0 on success
 */
int ble_hw_rng_stop(void) {
	// TODO: as far as i can tell CH5xx doesnt have an RNG peripheral, would probably need to implement a software RNG
	return 0;
}

/**
 * Read a random byte from the RNG (blocking).
 * Waits until a random value is available.
 *
 * @return Random byte value
 */
uint8_t ble_hw_rng_read(void) {
	// TODO: as far as i can tell CH5xx doesnt have an RNG peripheral, would probably need to implement a software RNG
	return 0;
}

#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LL_PRIVACY)
/**
 * Clear the resolving list (for privacy feature).
 * Remove all IRK entries used for address resolution.
 */
void ble_hw_resolv_list_clear(void) {
	// TODO: Reset IRK count
	g_ch592_num_irks = 0;
	// TODO: no idea if ch5xx has hardware address resolution
}

/**
 * Add an IRK (Identity Resolving Key) to the resolving list.
 * Used to resolve random resolvable private addresses.
 *
 * @param irk Pointer to 16-byte IRK
 * @return BLE_ERR_SUCCESS or BLE_ERR_MEM_CAPACITY if list full
 */
int ble_hw_resolv_list_add(uint8_t* irk) {
	// TODO: Check if list is full
	if (g_ch592_num_irks >= CH592_IRK_LIST_ENTRIES) {
		return BLE_ERR_MEM_CAPACITY;
	}

	// TODO: no idea if ch5xx has hardware address resolution
	return BLE_ERR_SUCCESS;
}

/**
 * Remove an IRK from the resolving list by index.
 *
 * @param index Index of IRK to remove (0 to g_ch592_num_irks-1)
 */
void ble_hw_resolv_list_rmv(int index) {
	// TODO: no idea if ch5xx has hardware address resolution
}

/**
 * Get the maximum resolving list size.
 *
 * @return Maximum number of IRK entries
 */
uint8_t ble_hw_resolv_list_size(void) {
	return BLE_HW_RESOLV_LIST_SIZE;
	// TODO: no idea if ch5xx has hardware address resolution
}

/**
 * Check if a received address was resolved using the resolving list.
 * Called after RX to determine if incoming random address matched an IRK.
 *
 * @return Negative if unresolved, 0-N for index of matched IRK
 */
int ble_hw_resolv_list_match(void) {
	// TODO: no idea if ch5xx has hardware address resolution
	return -1;
}
#endif
