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
#include <string.h>
#include <assert.h>
#include "syscfg/syscfg.h"
// #include "os/os.h"
#include "nimble/nimble/controller/include/controller/ble_phy.h"

#include "../include/ble/xcvr.h"
#include "../include/ble/iSLER.h"
// #include "nimble/ble.h"
// #include "nimble/nimble_opt.h"
// #include "controller/ble_phy.h"
// #include "controller/ble_ll.h"

/* CH5xx has no BLE hardware - minimal stub implementation */

/* Stub structures */
static uint8_t g_ble_phy_tx_buf[BLE_PHY_MAX_PDU_LEN];
static uint8_t g_ble_phy_rx_buf[BLE_PHY_MAX_PDU_LEN];

/* Forward declarations */
static void ble_phy_isr(void);

/* Initialize the PHY */
int ble_phy_init(void) {
	// TODO: Not sure if this is needed
	RFCoreInit(LL_TX_POWER_0_DBM);
	return 0;
}

/* Reset the PHY */
int ble_phy_reset(void) {
	/* CH5xx: No hardware to reset */
	return 0;
}

// LL power array [-18, -10, -5, -3, 0, 1, 2, 3, 4, 5, 6, 7] dBm
int ble_phy_tx_power_round(int dbm) {
	/* "Rail" power level if outside supported range */
	int dbm_levels[] = { -18, -10, -5, -3, 0, 1, 2, 3, 4, 5, 6, 7 };
	int i;
	int dbm_rounded = dbm_levels[0];
	for (i = 0; i < sizeof(dbm_levels) / sizeof(dbm_levels[0]); i++) {
		if (dbm >= dbm_levels[i]) {
			dbm_rounded = dbm_levels[i];
		}
		else {
			break;
		}
	}

	return dbm;
}

int ble_phy_tx_power_set(int dbm) {
	/* Get actual TX power supported by radio */
	dbm = ble_phy_tx_power_round(dbm);

	DevInit(dbm);
	// g_ble_phy_data.phy_txpwr_dbm = dbm;

	return 0;
}

/* Get transmit power level */
int ble_phy_txpwr_get(void) {
	// TOOD: probably save it as variable
	return 0;
}

/* Set PHY mode */
void ble_phy_mode_set(uint8_t tx_phy_mode, uint8_t rx_phy_mode) {
	/* CH5xx: No PHY modes supported */
	// return BLE_PHY_ERR_INV_PARAM;
}

/* Enable reception */
int ble_phy_rx(void) {
	return 0;
}

/* Disable PHY */
void ble_phy_disable(void) {
	/* CH5xx: No hardware to disable */
}

/* Set access address */
void ble_phy_set_access_address(uint32_t access_addr) {
	/* CH5xx: No hardware registers to configure */
	(void)access_addr;
}

/* Get access address */
uint32_t ble_phy_access_addr_get(void) {
	return 0;
}

/* Set transmit start time */
int ble_phy_tx_set_start_time(uint32_t cputime, uint8_t rem_usecs) {
	/* CH5xx: No timing hardware */
	return BLE_PHY_ERR_INV_PARAM;
}

/* Set receive start time */
int ble_phy_rx_set_start_time(uint32_t cputime, uint8_t rem_usecs) {
	/* CH5xx: No timing hardware */
	return BLE_PHY_ERR_INV_PARAM;
}

/* Transmit packet */
int ble_phy_tx(ble_phy_tx_pducb_t pducb, void* pducb_arg, uint8_t end_trans) {
	/* CH5xx: No radio hardware */
	return BLE_ERR_SUCCESS;
}

/* Set channel */
int ble_phy_setchan(uint8_t chan, uint32_t access_addr, uint32_t crcinit) {
	DevSetChannel(chan);
	return 0;
}

void ble_phy_encrypt_enable(const uint8_t* key) {
	// TODO: not implemented yet
}

void ble_phy_encrypt_disable(void) {
	// TODO: not implemented yet
}

void ble_phy_encrypt_iv_set(const uint8_t* iv) {
	// TODO: not implemented yet
	//memcpy(g_nrf_ccm_data.iv, iv, 8);
}

void ble_phy_encrypt_counter_set(uint64_t counter, uint8_t dir_bit) {
	// TODO: not implemented yet
	// g_nrf_ccm_data.pkt_counter = counter;
	// g_nrf_ccm_data.dir_bit     = dir_bit;
}

void ble_phy_wfr_enable(int txrx, uint8_t tx_phy_mode, uint32_t wfr_usecs) {
	// TODO: not implemented yet
}

#define NRF_MAX_ENCRYPTED_PYLD_LEN  (27)

uint8_t ble_phy_max_data_pdu_pyld(void) {
	// TODO: not implemented yet
#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_ENCRYPTION)
	return NRF_MAX_ENCRYPTED_PYLD_LEN;
#else
	return BLE_LL_DATA_PDU_MAX_PYLD;
#endif
}

void ble_phy_rfclk_disable(void) {
	// TODO: not implemented yet
}

void ble_phy_rfclk_enable(void) {
	// TODO: not implemented yet
}

void ble_phy_resolv_list_enable(void) {
	// TODO: not implemented yet
}

void ble_phy_resolv_list_disable(void) {
	// TODO: not implemented yet
}

void ble_phy_set_txend_cb(ble_phy_tx_end_func txend_cb, void* arg) {
	// TODO: not implemented yet
}

/* Resolve antenna */
uint8_t ble_phy_ant_resolve(uint8_t ant) {
	return 0;
}

/* Get RSSI */
int8_t ble_phy_get_rssi(void) {
	/* CH5xx: No RSSI measurement */
	return 0;
}

/* PHY ISR - should never be called on CH5xx */
static void ble_phy_isr(void) {
	assert(0);
}
