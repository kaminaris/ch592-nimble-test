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
#include "nimble/porting/nimble/include/os/os.h"
/* Keep os_cputime explicitly to enable build on non-Mynewt platforms */
#include "nimble/porting/nimble/include/os/os_cputime.h"
#include "nimble/nimble/include/nimble/ble.h"
#include "nimble/nimble/include/nimble/nimble_opt.h"
#include "nimble/nimble/include/nimble/nimble_npl.h"
#include "nimble/nimble/controller/include/controller/ble_phy.h"
#include "nimble/nimble/controller/include/controller/ble_ll.h"
#include "nimble/nimble/controller/include/controller/ble_phy_trace.h"

#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_CODED_PHY)
#error LE Coded PHY cannot be enabled for CH59x
#endif

#ifndef min
#define min(a, b) ((a)<(b)?(a):(b))
#endif

// CH592 radio IRQ is mapped to LLE_IRQn probably
#ifndef RADIO_IRQn
#define RADIO_IRQn         LLE_IRQn
#endif

#if BABBLESIM
extern void tm_tick(void);
#endif

#include <nimble/nimble/controller/include/controller/ble_ll_pdu.h>

static void ble_phy_isr(void);

#include "../include/ble/xcvr.h"
#include "../include/ble/iSLER.h"
// #include "nimble/ble.h"
// #include "nimble/nimble_opt.h"
// #include "controller/ble_phy.h"
// #include "controller/ble_ll.h"

/*
 * XXX: Maximum possible transmit time is 1 msec for a 60ppm crystal
 * and 16ms for a 30ppm crystal! We need to limit PDU size based on
 * crystal accuracy. Look at this in the spec.
 */

/* Forward declarations */
// uint32_t accessAddress = 0x8E89BED6;
// uint32_t crcInit = 0x555555;
// //TODO: no idea if this is correct
// uint8_t channelSet = 37;
// int txPowerLevel = 0;
struct ble_phy_obj
{
	uint8_t phy_stats_initialized;
	int8_t  phy_txpwr_dbm;
	uint8_t phy_chan;
	uint8_t phy_state;
	uint8_t phy_transition;
	uint8_t phy_transition_late;
	uint8_t phy_rx_started;
	uint8_t phy_encrypted;
#if PHY_USE_HEADERMASK_WORKAROUND
	uint8_t phy_headermask;
	uint8_t phy_headerbyte;
#endif
	uint8_t phy_privacy;
	uint8_t phy_tx_pyld_len;
	uint8_t phy_cur_phy_mode;
	uint8_t phy_tx_phy_mode;
	uint8_t phy_rx_phy_mode;
	uint8_t phy_bcc_offset;
	uint32_t phy_aar_scratch;
	uint32_t phy_access_address;
	uint32_t g_tx_end_time;
	uint32_t g_aa_timestamp;
    uint32_t g_rxen_timestamp;
	struct ble_mbuf_hdr rxhdr;
	void *txend_arg;
	ble_phy_tx_end_func txend_cb;
	uint32_t phy_start_cputime;
#if MYNEWT_VAL(BLE_PHY_VARIABLE_TIFS)
	uint16_t tifs;
#endif

	uint16_t txtx_time_us;
	uint8_t txtx_time_anchor;
};
static struct ble_phy_obj g_ble_phy_data;

/* XXX: if 27 byte packets desired we can make this smaller */
/* Global transmit/receive buffer */
// TODO: not sure if aligning this is necessary on CH59x
__attribute__((aligned(4))) static uint32_t g_ble_phy_tx_buf[(BLE_PHY_MAX_PDU_LEN + 3) / 4];
__attribute__((aligned(4))) static uint32_t g_ble_phy_rx_buf[(BLE_PHY_MAX_PDU_LEN + 3) / 4];

#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_ENCRYPTION)
/* Make sure word-aligned for faster copies */
static uint32_t g_ble_phy_enc_buf[(BLE_PHY_MAX_PDU_LEN + 3) / 4];
#endif

/* RF center frequency for each channel index (offset from 2400 MHz) */
static const uint8_t g_ble_phy_chan_freq[BLE_PHY_NUM_CHANS] = {
	4,  6,  8, 10, 12, 14, 16, 18, 20, 22, /* 0-9 */
   24, 28, 30, 32, 34, 36, 38, 40, 42, 44, /* 10-19 */
   46, 48, 50, 52, 54, 56, 58, 60, 62, 64, /* 20-29 */
   66, 68, 70, 72, 74, 76, 78,  2, 26, 80, /* 30-39 */
};

#if (BLE_LL_BT5_PHY_SUPPORTED == 1)
/* packet start offsets (in usecs) */
static const uint16_t g_ble_phy_mode_pkt_start_off[BLE_PHY_NUM_MODE] = {
	[BLE_PHY_MODE_1M] = 40,
	[BLE_PHY_MODE_2M] = 24,
	[BLE_PHY_MODE_CODED_125KBPS] = 376,
	[BLE_PHY_MODE_CODED_500KBPS] = 376
};
#endif

/* Various radio timings */
/* Radio ramp-up times in usecs (fast mode) */
#define BLE_PHY_T_TXENFAST      (XCVR_TX_RADIO_RAMPUP_USECS)
#define BLE_PHY_T_RXENFAST      (XCVR_RX_RADIO_RAMPUP_USECS)

#if BABBLESIM
/* delay between EVENTS_READY and start of tx */
static const uint8_t g_ble_phy_t_txdelay[BLE_PHY_NUM_MODE] = {
	[BLE_PHY_MODE_1M] = 1,
	[BLE_PHY_MODE_2M] = 1,
};
/* delay between EVENTS_END and end of txd packet */
static const uint8_t g_ble_phy_t_txenddelay[BLE_PHY_NUM_MODE] = {
	[BLE_PHY_MODE_1M] = 1,
	[BLE_PHY_MODE_2M] = 1,
};
/* delay between rxd access address (w/ TERM1 for coded) and EVENTS_ADDRESS */
static const uint8_t g_ble_phy_t_rxaddrdelay[BLE_PHY_NUM_MODE] = {
	[BLE_PHY_MODE_1M] = 9,
	[BLE_PHY_MODE_2M] = 5,
};
/* delay between end of rxd packet and EVENTS_END */
static const uint8_t g_ble_phy_t_rxenddelay[BLE_PHY_NUM_MODE] = {
	[BLE_PHY_MODE_1M] = 9,
	[BLE_PHY_MODE_2M] = 5,
};
#else
/* delay between EVENTS_READY and start of tx */
static const uint8_t g_ble_phy_t_txdelay[BLE_PHY_NUM_MODE] = {
	[BLE_PHY_MODE_1M] = 4,
	[BLE_PHY_MODE_2M] = 3,
	[BLE_PHY_MODE_CODED_125KBPS] = 5,
	[BLE_PHY_MODE_CODED_500KBPS] = 5
};
/* delay between EVENTS_END and end of txd packet */
static const uint8_t g_ble_phy_t_txenddelay[BLE_PHY_NUM_MODE] = {
	[BLE_PHY_MODE_1M] = 4,
	[BLE_PHY_MODE_2M] = 3,
	[BLE_PHY_MODE_CODED_125KBPS] = 9,
	[BLE_PHY_MODE_CODED_500KBPS] = 3
};
/* delay between rxd access address (w/ TERM1 for coded) and EVENTS_ADDRESS */
static const uint8_t g_ble_phy_t_rxaddrdelay[BLE_PHY_NUM_MODE] = {
	[BLE_PHY_MODE_1M] = 6,
	[BLE_PHY_MODE_2M] = 2,
	[BLE_PHY_MODE_CODED_125KBPS] = 17,
	[BLE_PHY_MODE_CODED_500KBPS] = 17
};
/* delay between end of rxd packet and EVENTS_END */
static const uint8_t g_ble_phy_t_rxenddelay[BLE_PHY_NUM_MODE] = {
	[BLE_PHY_MODE_1M] = 6,
	[BLE_PHY_MODE_2M] = 2,
	[BLE_PHY_MODE_CODED_125KBPS] = 27,
	[BLE_PHY_MODE_CODED_500KBPS] = 22
};
#endif

#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_ENCRYPTION)

/*
 * Per nordic, the number of bytes needed for scratch is 16 + MAX_PKT_SIZE.
 * However, when I used a smaller size it still overwrote the scratchpad. Until
 * I figure this out I am just going to allocate 67 words so we have enough
 * space for 267 bytes of scratch. I used 268 bytes since not sure if this
 * needs to be aligned and burning a byte is no big deal.
 */
//#define NRF_ENC_SCRATCH_WORDS (((MYNEWT_VAL(BLE_LL_MAX_PKT_SIZE) + 16) + 3) / 4)
#define NRF_ENC_SCRATCH_WORDS   (67)

static uint32_t g_nrf_encrypt_scratchpad[NRF_ENC_SCRATCH_WORDS];

struct nrf_ccm_data
{
	uint8_t key[16];
	uint64_t pkt_counter;
	uint8_t dir_bit;
	uint8_t iv[8];
} __attribute__((packed));

struct nrf_ccm_data g_nrf_ccm_data;
#endif

#if (BLE_LL_BT5_PHY_SUPPORTED == 1)

/* Packet start offset (in usecs). This is the preamble plus access address.
 * For LE Coded PHY this also includes CI and TERM1. */
static uint32_t
ble_phy_mode_pdu_start_off(int phy_mode)
{
    return g_ble_phy_mode_pkt_start_off[phy_mode];
}

#if NRF52_ERRATA_191_ENABLE_WORKAROUND
static bool
ble_phy_mode_is_coded(uint8_t phy_mode)
{
    return (phy_mode == BLE_PHY_MODE_CODED_125KBPS) ||
           (phy_mode == BLE_PHY_MODE_CODED_500KBPS);
}

static void
phy_nrf52_errata_191(uint8_t new_phy_mode)
{
    bool from_coded = ble_phy_mode_is_coded(g_ble_phy_data.phy_cur_phy_mode);
    bool to_coded = ble_phy_mode_is_coded(new_phy_mode);

    /* [191] RADIO: High packet error rate in BLE Long Range mode
     * Should be applied only if switching to/from LE Coded, no need to apply
     * on each mode change.
     */
    if (from_coded == to_coded) {
        return;
    }

    if (to_coded) {
        *(volatile uint32_t *)0x40001740 =
            ((*((volatile uint32_t *)0x40001740)) & 0x7fff00ff) |
            0x80000000 | (((uint32_t)(196)) << 8);
    } else {
        *(volatile uint32_t *) 0x40001740 =
            ((*((volatile uint32_t *) 0x40001740)) & 0x7fffffff);
    }
}
#endif

static void
ble_phy_mode_apply(uint8_t phy_mode)
{
	if (phy_mode == g_ble_phy_data.phy_cur_phy_mode) {
		return;
	}

    // TODO: those should be NOP probably or ISLER needs a change
	switch (phy_mode) {
		case BLE_PHY_MODE_1M:
	        BB->CTRL_CFG = CTRL_CFG_PHY_1M;
			break;
#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_2M_PHY)
		case BLE_PHY_MODE_2M:
	        BB->CTRL_CFG = CTRL_CFG_PHY_2M;
			break;
#endif
#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_CODED_PHY)
		case BLE_PHY_MODE_CODED_125KBPS:
			// TODO: coded phy not supported yet
			break;
		case BLE_PHY_MODE_CODED_500KBPS:
	        // TODO: coded phy not supported yet
			break;
#endif
		default:
			assert(0);
	}

	g_ble_phy_data.phy_cur_phy_mode = phy_mode;
}

void
ble_phy_mode_set(uint8_t tx_phy_mode, uint8_t rx_phy_mode)
{
	g_ble_phy_data.phy_tx_phy_mode = tx_phy_mode;
	g_ble_phy_data.phy_rx_phy_mode = rx_phy_mode;
}
#else
static uint32_t
ble_phy_mode_pdu_start_off(int phy_mode)
{
	return 40;
}
#endif

static int
ble_phy_get_cur_phy(void)
{
#if (BLE_LL_BT5_PHY_SUPPORTED == 1)
	switch (g_ble_phy_data.phy_cur_phy_mode) {
		case BLE_PHY_MODE_1M:
			return BLE_PHY_1M;
		case BLE_PHY_MODE_2M:
			return BLE_PHY_2M;
		case BLE_PHY_MODE_CODED_125KBPS:
		case BLE_PHY_MODE_CODED_500KBPS:
			return BLE_PHY_CODED;
		default:
			assert(0);
			return -1;
	}
#else
	return BLE_PHY_1M;
#endif
}

/**
 * Copies the data from the phy receive buffer into a mbuf chain.
 *
 * @param dptr Pointer to receive buffer
 * @param rxpdu Pointer to already allocated mbuf chain
 *
 * NOTE: the packet header already has the total mbuf length in it. The
 * lengths of the individual mbufs are not set prior to calling.
 *
 */
void
ble_phy_rxpdu_copy(uint8_t *dptr, struct os_mbuf *rxpdu)
{
    uint32_t rem_len;
    uint32_t copy_len;
    uint32_t block_len;
    uint32_t block_rem_len;
    void *dst;
    void *src;
    struct os_mbuf * om;

    /* Better be aligned */
    assert(((uint32_t)dptr & 3) == 0);

    block_len = rxpdu->om_omp->omp_databuf_len;
    rem_len = OS_MBUF_PKTHDR(rxpdu)->omp_len;
    src = dptr;

    /*
     * Setup for copying from first mbuf which is shorter due to packet header
     * and extra leading space
     */
    copy_len = block_len - rxpdu->om_pkthdr_len - 4;
    om = rxpdu;
    dst = om->om_data;

    while (true) {
        /*
         * Always copy blocks of length aligned to word size, only last mbuf
         * will have remaining non-word size bytes appended.
         */
        block_rem_len = copy_len;
        copy_len = min(copy_len, rem_len);
        copy_len &= ~3;

        dst = om->om_data;
        om->om_len = copy_len;
        rem_len -= copy_len;
        block_rem_len -= copy_len;

#if BABBLESIM
        memcpy(dst, src, copy_len);
        dst += copy_len;
        src += copy_len;
#else
        __asm__ volatile (
            "   mv   t1, %[len]           \n"  // Save original length in t1 (like r4)
            "   j    2f                   \n"  // Jump to loop condition
            "1: add  t0, %[src], %[len]   \n"  // Calculate src address
            "   lw   t2, 0(t0)            \n"  // Load word from [src + len]
            "   add  t0, %[dst], %[len]   \n"  // Calculate dst address
            "   sw   t2, 0(t0)            \n"  // Store word to [dst + len]
            "2: addi %[len], %[len], -4   \n"  // len -= 4
            "   bgez %[len], 1b           \n"  // Branch if len >= 0
            "   add  %[src], %[src], t1   \n"  // Restore src pointer
            "   add  %[dst], %[dst], t1   \n"  // Restore dst pointer
            : [dst] "+r" (dst), [src] "+r" (src),
              [len] "+r" (copy_len)
            :
            : "t0", "t1", "t2", "memory"
        );
#endif

        if ((rem_len < 4) && (block_rem_len >= rem_len)) {
            break;
        }

        /* Move to next mbuf */
        om = SLIST_NEXT(om, om_next);
        copy_len = block_len;
    }

    /* Copy remaining bytes, if any, to last mbuf */
    om->om_len += rem_len;

#if BABBLESIM
    memcpy(dst, src, rem_len);
#else
    __asm__ volatile (
        "   j    2f                   \n"  // Jump to loop condition
        "1: add  t0, %[src], %[len]   \n"  // Calculate src address
        "   lbu  t1, 0(t0)            \n"  // Load byte from [src + len]
        "   add  t0, %[dst], %[len]   \n"  // Calculate dst address
        "   sb   t1, 0(t0)            \n"  // Store byte to [dst + len]
        "2: addi %[len], %[len], -1   \n"  // len -= 1
        "   bgez %[len], 1b           \n"  // Branch if len >= 0
        : [len] "+r" (rem_len)
        : [dst] "r" (dst), [src] "r" (src)
        : "t0", "t1", "memory"
    );
#endif

    /* Copy header */
    memcpy(BLE_MBUF_HDR_PTR(rxpdu), &g_ble_phy_data.rxhdr,
           sizeof(struct ble_mbuf_hdr));
}

/**
 * Called when we want to wait if the radio is in either the rx or tx
 * disable states. We want to wait until that state is over before doing
 * anything to the radio
 */
static void
nrf_wait_disabled(void)
{
	uint32_t state;

	//state = LL->STATUS;
    // TODO: no idea how to handle this, just a NOP for now

#if BABBLESIM
				tm_tick();
#endif
}

#if MYNEWT_VAL(BLE_PHY_VARIABLE_TIFS)
void
ble_phy_tifs_set(uint16_t tifs)
{
	g_ble_phy_data.tifs = tifs;
}
#endif

/**
 *
 *
 */
static int
ble_phy_set_start_time(uint32_t cputime, uint8_t rem_us, bool tx)
{
    uint32_t next_cc;
    uint32_t cur_cc;
    uint32_t cntr;
    uint32_t delta;
    int radio_rem_us;
    int rem_us_corr;
    int min_rem_us;

    /* Calculate rem_us for radio and FEM enable. The result may be a negative
     * value, but we'll adjust later.
     */
    if (tx) {
        radio_rem_us = rem_us - BLE_PHY_T_TXENFAST -
                       g_ble_phy_t_txdelay[g_ble_phy_data.phy_cur_phy_mode];
    } else {
        radio_rem_us = rem_us - BLE_PHY_T_TXENFAST;
    }

    min_rem_us = radio_rem_us;

    /* We need to adjust rem_us values, so they are >=1 for TIMER0 compare
     * event to be triggered.
     */

    if (min_rem_us <= -30) {
        /* rem_us is -60..-30 */
        cputime -= 2;
        rem_us_corr = 61;
    } else {
        /* rem_us is -29..0 */
        cputime -= 1;
        rem_us_corr = 30;
    }

    /*
     * Can we set the RTC compare to start TIMER0? We can do it if:
     *      a) Current compare value is not N+1 or N+2 ticks from current
     *      counter.
     *      b) The value we want to set is not at least N+2 from current
     *      counter.
     *
     * NOTE: since the counter can tick 1 while we do these calculations we
     * need to account for it.
     */
    next_cc = cputime & 0xffffff;
    // TODO: is it correct register?
    cur_cc = R32_RTC_TRIG;//NRF_RTC0->CC[0];
    cntr = R32_RTC_CNT_32K; //NRF_RTC0->COUNTER;

    delta = (cur_cc - cntr) & 0xffffff;
    if ((delta <= 3) && (delta != 0)) {
        return -1;
    }
    delta = (next_cc - cntr) & 0xffffff;
    if ((delta & 0x800000) || (delta < 3)) {
        return -1;
    }

    /* Clear and set TIMER0 to fire off at proper time */
    // CH592: Stop and clear TMR0
    R8_TMR0_CTRL_MOD &= ~RB_TMR_COUNT_EN;  // Stop timer
    R32_TMR0_COUNT = 0;                     // Clear counter (not R32_TMR0_CNT)

    // CH592: Set compare value for TMR0
    R32_TMR0_CNT_END = radio_rem_us + rem_us_corr;

    // CH592: Clear timer interrupt flag
    R8_TMR0_INT_FLAG = RB_TMR_IF_CYC_END;

    /* Set RTC compare to start TIMER0 */
    // CH592: Clear RTC trigger flag
    R8_RTC_FLAG_CTRL |= RB_RTC_TRIG_CLR;

    // CH592: Set RTC trigger value
    R32_RTC_TRIG = next_cc;

    // CH592: Enable RTC trigger mode
    R8_RTC_MODE_CTRL |= RB_RTC_TRIG_EN;

    /* PPI equivalent - Manual trigger needed */
    // CH592: No hardware PPI, you'll need to handle this in RTC interrupt:
    // TODO: When RTC trigger fires, start TMR0 manually in the RTC ISR

    /* Store the cputime at which we set the RTC */
    g_ble_phy_data.phy_start_cputime = cputime;

    return 0;
}


static int
ble_phy_set_start_now(void)
{
    os_sr_t sr;
    uint32_t now;
    uint32_t radio_rem_us;

    OS_ENTER_CRITICAL(sr);

    /* We need to set TIMER0 compare registers to at least 1 as otherwise
     * compare event won't be triggered. Event (FEM/radio) that have to be
     * triggered first is set to 1, other event is set to 1+diff.
     *
     * Note that this is only used for rx, so only need to handle LNA.
     */

    radio_rem_us = 1;

    /* Clear and set TIMER0 to fire off at proper time */
    // CH592: Stop timer
    R8_TMR0_CTRL_MOD &= ~RB_TMR_COUNT_EN;

    // CH592: Clear counter
    R32_TMR0_COUNT = 0;

    // CH592: Set compare value
    R32_TMR0_CNT_END = radio_rem_us;

    // CH592: Clear compare interrupt flag
    R8_TMR0_INT_FLAG = RB_TMR_IF_CYC_END;


    /*
     * Set RTC compare to start TIMER0. We need to set it to at least N+2 ticks
     * from current value to guarantee triggering compare event, but let's set
     * it to N+3 to account for possible extra tick on RTC0 during these
     * operations.
     */
    now = os_cputime_get32();

    // CH592: Clear RTC trigger flag
    R8_RTC_FLAG_CTRL |= RB_RTC_TRIG_CLR;

    // CH592: Set RTC trigger value (N+3 ticks from now)
    R32_RTC_TRIG = (now + 3) & 0xffffff;

    // CH592: Enable RTC trigger mode
    R8_RTC_MODE_CTRL |= RB_RTC_TRIG_EN;

    /* Enable PPI equivalent - needs RTC ISR handler */
    // CH592: No hardware PPI - must handle in RTC interrupt
    // TODO: Make sure RTC interrupt is enabled to catch the trigger


    /*
     * Store the cputime at which we set the RTC
     *
     * XXX Compare event may be triggered on previous CC value (if it was set to
     * less than N+2) so in rare cases actual start time may be 2 ticks earlier
     * than what we expect. Since this is only used on RX, it may cause AUX scan
     * to be scheduled 1 or 2 ticks too late so we'll miss it - it's acceptable
     * for now.
     */
    g_ble_phy_data.phy_start_cputime = now + 3;

    OS_EXIT_CRITICAL(sr);

    return 0;
}

/**
 * Function is used to set PPI so that we can time out waiting for a reception
 * to occur. This happens for two reasons: we have sent a packet and we are
 * waiting for a response (txrx should be set to ENABLE_TXRX) or we are
 * starting a connection event and we are a slave and we are waiting for the
 * master to send us a packet (txrx should be set to ENABLE_RX).
 *
 * NOTE: when waiting for a txrx turn-around, wfr_usecs is not used as there
 * is no additional time to wait; we know when we should receive the address of
 * the received frame.
 *
 * @param txrx Flag denoting if this wfr is a txrx turn-around or not.
 * @param tx_phy_mode phy mode for last TX (only valid for TX->RX)
 * @param wfr_usecs Amount of usecs to wait.
 */
void
ble_phy_wfr_enable(int txrx, uint8_t tx_phy_mode, uint32_t wfr_usecs)
{
    uint32_t end_time;
    uint8_t phy;
    uint16_t tifs;

    phy = g_ble_phy_data.phy_cur_phy_mode;

#if MYNEWT_VAL(BLE_PHY_VARIABLE_TIFS)
    tifs = g_ble_phy_data.tifs;
#else
    tifs = BLE_LL_IFS;
#endif

    if (txrx == BLE_PHY_WFR_ENABLE_TXRX) {
        /* RX shall start exactly T_IFS after TX end captured in CC[2] */
        end_time = g_ble_phy_data.g_tx_end_time + tifs;
        /* Adjust for delay between EVENT_END and actual TX end time */
        end_time += g_ble_phy_t_txenddelay[tx_phy_mode];
        /* Wait a bit longer due to allowed active clock accuracy */
        end_time += 2;
        /*
         * It's possible that we'll capture PDU start time at the end of timer
         * cycle and since wfr expires at the beginning of calculated timer
         * cycle it can be almost 1 usec too early. Let's compensate for this
         * by waiting 1 usec more.
         */
        end_time += 1;
    } else {
        /*
         * RX shall start no later than wfr_usecs after RX enabled.
         * CC[0] is the time of RXEN so adjust for radio ram-up.
         * Do not add jitter since this is already covered by LL.
         */
        end_time = g_ble_phy_data.g_rxen_timestamp + BLE_PHY_T_RXENFAST + wfr_usecs;
    }

    /*
     * Note: on LE Coded EVENT_ADDRESS is fired after TERM1 is received, so
     *       we are actually calculating relative to start of packet payload
     *       which is fine.
     */

    /* Adjust for receiving access address since this triggers EVENT_ADDRESS */
    end_time += ble_phy_mode_pdu_start_off(phy);
    /* Adjust for delay between actual access address RX and EVENT_ADDRESS */
    end_time += g_ble_phy_t_rxaddrdelay[phy];

    /* CH592: Set TMR0 compare value for wait-for-response timeout */
    R32_TMR0_CNT_END = end_time;

    /* CH592: Clear timer compare interrupt flag */
    R8_TMR0_INT_FLAG = RB_TMR_IF_CYC_END;

    /* Enable wait for response PPI */
    // CH592: No hardware PPI - handle timeout in TMR0 interrupt
    // TODO: Enable TMR0 interrupt to trigger radio disable on timeout
    R8_TMR0_INT_FLAG = RB_TMR_IF_CYC_END;  // Clear flag again before enabling
    R8_TMR0_INTER_EN |= RB_TMR_IE_CYC_END; // Enable compare interrupt


    /*
     * It may happen that if CPU is halted for a brief moment (e.g. during flash
     * erase or write), TIMER0 already counted past CC[3] and thus wfr will not
     * fire as expected. In case this happened, let's just disable PPIs for wfr
     * and trigger wfr manually (i.e. disable radio).
     *
     * Note that the same applies to RX start time set in CC[0] but since it
     * should fire earlier than wfr, fixing wfr is enough.
     *
     * CC[1] is only used as a reference on RX start, we do not need it here so
     * it can be used to read TIMER0 counter.
     */
    /* CH592: Manually capture current timer value */
    uint32_t current_time = R32_TMR0_COUNT;

    /* Check if timer already passed the compare value (timeout already occurred) */
    if (current_time > R32_TMR0_CNT_END) {
        /* Timeout already happened - disable radio manually */

        /* Disable TMR0 compare interrupt */
        R8_TMR0_INTER_EN &= ~RB_TMR_IE_CYC_END;

        /* Clear interrupt flag */
        R8_TMR0_INT_FLAG = RB_TMR_IF_CYC_END;

        /* Trigger radio disable */
        // TODO: Add CH592 radio disable command here
        // This depends on your radio implementation
        DevSetMode(0);
    }
}

#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_ENCRYPTION)
static uint32_t
ble_phy_get_ccm_datarate(void)
{
#if BLE_LL_BT5_PHY_SUPPORTED
	switch (g_ble_phy_data.phy_cur_phy_mode) {
		case BLE_PHY_MODE_1M:
			return CCM_MODE_DATARATE_1Mbit << CCM_MODE_DATARATE_Pos;
		case BLE_PHY_MODE_2M:
			return CCM_MODE_DATARATE_2Mbit << CCM_MODE_DATARATE_Pos;
#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_CODED_PHY)
		case BLE_PHY_MODE_CODED_125KBPS:
			return CCM_MODE_DATARATE_125Kbps << CCM_MODE_DATARATE_Pos;
		case BLE_PHY_MODE_CODED_500KBPS:
			return CCM_MODE_DATARATE_500Kbps << CCM_MODE_DATARATE_Pos;
#endif
	}

	assert(0);
	return 0;
#else
	return CCM_MODE_DATARATE_1Mbit << CCM_MODE_DATARATE_Pos;
#endif
}
#endif

/**
 * Setup transceiver for receive.
 */
static void
ble_phy_rx_xcvr_setup(void)
{
    uint8_t *dptr;

    dptr = (uint8_t *)&g_ble_phy_rx_buf[0];
    dptr += 3;

#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_ENCRYPTION)
    if (g_ble_phy_data.phy_encrypted) {
        NRF_RADIO->PACKETPTR = (uint32_t)&g_ble_phy_enc_buf[0];
        NRF_CCM->INPTR = (uint32_t)&g_ble_phy_enc_buf[0];
        NRF_CCM->OUTPTR = (uint32_t)dptr;
        NRF_CCM->SCRATCHPTR = (uint32_t)&g_nrf_encrypt_scratchpad[0];
        NRF_CCM->MODE = CCM_MODE_LENGTH_Msk | CCM_MODE_MODE_Decryption |
                                                    ble_phy_get_ccm_datarate();
        NRF_CCM->CNFPTR = (uint32_t)&g_nrf_ccm_data;
        NRF_CCM->SHORTS = 0;
        NRF_CCM->EVENTS_ERROR = 0;
        NRF_CCM->EVENTS_ENDCRYPT = 0;
        nrf_ccm_task_trigger(NRF_CCM, NRF_CCM_TASK_KSGEN);
        phy_ppi_radio_address_to_ccm_crypt_enable();
    } else {
        NRF_RADIO->PACKETPTR = (uint32_t)dptr;
    }
#else
    LL->RXBUF = (uint32_t)dptr;
#endif

#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LL_PRIVACY)
    if (g_ble_phy_data.phy_privacy) {
        NRF_AAR->ENABLE = AAR_ENABLE_ENABLE_Enabled;
        NRF_AAR->IRKPTR = (uint32_t)&g_nrf_irk_list[0];
        NRF_AAR->SCRATCHPTR = (uint32_t)&g_ble_phy_data.phy_aar_scratch;
        NRF_AAR->EVENTS_END = 0;
        NRF_AAR->EVENTS_RESOLVED = 0;
        NRF_AAR->EVENTS_NOTRESOLVED = 0;
    } else {
        if (g_ble_phy_data.phy_encrypted == 0) {
            NRF_AAR->ENABLE = AAR_ENABLE_ENABLE_Disabled;
        }
    }
#endif

    /* Turn off trigger TXEN on output compare match and AAR on bcmatch */
    /* CH592: Disable timer-to-radio and radio-to-AAR triggers */
    /* No hardware PPI on CH592 - these are NOPs or handled differently */

    /* Disable TMR0 compare interrupt that would trigger radio TXEN */
    R8_TMR0_INTER_EN &= ~RB_TMR_IE_CYC_END;

    /* Clear any pending timer interrupt */
    R8_TMR0_INT_FLAG = RB_TMR_IF_CYC_END;

    /* CH592: No AAR (Address Resolution) hardware equivalent */
    /* If you implemented software AAR, disable its trigger here */

    /* Reset the rx started flag. Used for the wait for response */
    g_ble_phy_data.phy_rx_started = 0;
    g_ble_phy_data.phy_state = BLE_PHY_STATE_RX;

#if BLE_LL_BT5_PHY_SUPPORTED
    /*
     * On Coded PHY there are CI and TERM1 fields before PDU starts so we need
     * to take this into account when setting up BCC.
     */
    if (g_ble_phy_data.phy_cur_phy_mode == BLE_PHY_MODE_CODED_125KBPS ||
            g_ble_phy_data.phy_cur_phy_mode == BLE_PHY_MODE_CODED_500KBPS) {
        g_ble_phy_data.phy_bcc_offset = 5;
    } else {
        g_ble_phy_data.phy_bcc_offset = 0;
    }
#else
    g_ble_phy_data.phy_bcc_offset = 0;
#endif

    /* I want to know when 1st byte received (after address) */
    /* CH592: No bit counter compare (BCC) hardware */
    /* The ISLER library doesn't expose bit-level events during reception */

    /* CH592: No event registers - reception is handled by ISLER internally */
    /* These events don't exist on CH592:
     * - EVENTS_ADDRESS: Access address match (hidden inside ISLER)
     * - EVENTS_DEVMATCH: Device address match (no hardware support)
     * - EVENTS_BCMATCH: Bit counter match (no hardware support)
     * - EVENTS_RSSIEND: RSSI measurement complete (no separate event)
     * - EVENTS_CRCOK: CRC validation (checked in ble_phy_isr after reception)
     */

    /* CH592: No SHORTS hardware */
    /* These automatic state transitions don't exist:
     * - END → DISABLE: Must call DevSetMode(0) manually after Frame_RX completes
     * - READY → START: Automatically handled by Frame_RX()
     * - ADDRESS → BCSTART: N/A (no bit counter)
     * - ADDRESS → RSSISTART: Call ReadRSSI() in your ISR callback
     * - DISABLED → RSSISTOP: N/A (RSSI is read on-demand)
     */
    NVIC_EnableIRQ(RADIO_IRQn);

    /* CH592: Enable radio interrupt via ISLER callback */
    /* Your ble_phy_isr() is called when LLE_IRQHandler() fires */
    /* This happens ONLY when a complete frame is received */
    /* No separate ADDRESS or DISABLED events available */

    /* Note: In ble_phy_isr(), you can now access:
     * - Received frame data in LLE_BUF
     * - RSSI via ReadRSSI()
     * - LL->STATUS for error flags (but they're cleared on entry)
     */
}

/**
 * Called from interrupt context when the transmit ends
 *
 */
static void
ble_phy_tx_end_isr(void)
{
    uint8_t tx_phy_mode;
    uint8_t was_encrypted;
    uint8_t transition;
    uint32_t rx_time;
    uint32_t tx_time;
    uint32_t radio_time;
    uint16_t tifs;

    /* CH592: Capture TX end time from TMR0 */
    g_ble_phy_data.g_tx_end_time = R32_TMR0_COUNT;

    /* Store PHY on which we've just transmitted smth */
    tx_phy_mode = g_ble_phy_data.phy_cur_phy_mode;

    /* If this transmission was encrypted we need to remember it */
    was_encrypted = g_ble_phy_data.phy_encrypted;
    (void)was_encrypted;

    /* Better be in TX state! */
    assert(g_ble_phy_data.phy_state == BLE_PHY_STATE_TX);

#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_ENCRYPTION)
    /*
     * XXX: not sure what to do. We had a HW error during transmission.
     * For now I just count a stat but continue on like all is good.
     */
    if (was_encrypted) {
        if (NRF_CCM->EVENTS_ERROR) {
            STATS_INC(ble_phy_stats, tx_hw_err);
            NRF_CCM->EVENTS_ERROR = 0;
        }
    }
#endif

#if MYNEWT_VAL(BLE_PHY_VARIABLE_TIFS)
    tifs = g_ble_phy_data.tifs;
    g_ble_phy_data.tifs = BLE_LL_IFS;
#else
    tifs = BLE_LL_IFS;
#endif
    transition = g_ble_phy_data.phy_transition;

    if (g_ble_phy_data.txend_cb) {
        g_ble_phy_data.txend_cb(g_ble_phy_data.txend_arg);
    }

    if (transition == BLE_PHY_TRANSITION_TX_RX) {
#if (BLE_LL_BT5_PHY_SUPPORTED == 1)
        ble_phy_mode_apply(g_ble_phy_data.phy_rx_phy_mode);
#endif

        /* Packet pointer needs to be reset. */
        ble_phy_rx_xcvr_setup();

        /* CH592: Set up timer for RX start after TIFS */
        // Stop timer
        R8_TMR0_CTRL_MOD &= ~RB_TMR_COUNT_EN;

        // Clear counter
        R32_TMR0_COUNT = 0;

        // Calculate when to start RX (TX end + TIFS)
        rx_time = g_ble_phy_data.g_tx_end_time + tifs;
        rx_time += g_ble_phy_t_txenddelay[tx_phy_mode];
        rx_time += 2; // Clock accuracy compensation
        rx_time += 1; // Extra compensation for timer cycle

        // Set compare for RX enable
        R32_TMR0_CNT_END = rx_time;

        // Clear interrupt flag
        R8_TMR0_INT_FLAG = RB_TMR_IF_CYC_END;

        // Start timer
        R8_TMR0_CTRL_MOD |= RB_TMR_COUNT_EN;

        // TODO: You'll need to handle RX enable in TMR0 ISR when it fires

        /* In case TIMER0 did already count past CC[0] and/or CC[2], radio
         * and/or LNA may not be enabled. In any case we won't be stuck since
         * wfr will cancel rx if needed.
         *
         * FIXME failing to enable LNA may result in unexpected RSSI drop in
         *       case we still rxd something, so perhaps we could check it here
         */
    } else if (transition == BLE_PHY_TRANSITION_TX_TX) {
        if (g_ble_phy_data.txtx_time_anchor) {
            /* Schedule next TX relative to current TX end. TX end timestamp is
             * captured in CC[2].
             */
            tx_time = g_ble_phy_data.g_tx_end_time + g_ble_phy_data.txtx_time_us;
        } else {
            /* Schedule next TX relative to current TX start. AA timestamp is
             * captured in CC[1], we need to adjust for sync word to get TX
             * start.
             */
            // CH592: Must manually capture AA timestamp
            tx_time = g_ble_phy_data.g_aa_timestamp - ble_ll_pdu_syncword_us(tx_phy_mode) +
                      g_ble_phy_data.txtx_time_us;
            /* Adjust for delay between EVENT_ADDRESS and actual address TX time */
            /* FIXME assume this is the same as EVENT_END to end, but we should
             *       measure this to be sure */
            tx_time += g_ble_phy_t_txenddelay[tx_phy_mode];
        }

        /* Adjust for delay between EVENT_END and actual TX end time */
        tx_time += g_ble_phy_t_txenddelay[tx_phy_mode];

        /* Adjust for delay between EVENT_READY and actual TX start time */
        tx_time -= g_ble_phy_t_txdelay[g_ble_phy_data.phy_cur_phy_mode];

        radio_time = tx_time - BLE_PHY_T_TXENFAST;
        /* CH592: No hardware timer compare/capture */
        /* The LL->TMR register counts DOWN, not up like NRF_TIMER0 */

        /* Store the desired TX start time (we'll need to calculate delay manually) */
        uint32_t current_time = os_cputime_get32();
        int32_t delay_ticks = radio_time - current_time;

        if (delay_ticks <= 0) {
            /* We're already late - cannot schedule TX at this time */
            g_ble_phy_data.phy_transition_late = 1;
        } else {
            /* CH592: Cannot hardware-trigger radio TX at precise time */
            /* Best effort: busy-wait until scheduled time */
            while (os_cputime_get32() < radio_time) {
                /* Spin wait */
            }

            /* Now start TX immediately */
            /* The Frame_TX() call in your TX routine will handle this */
            g_ble_phy_data.phy_transition_late = 0;
        }

    } else {
        /*
         * XXX: not sure we need to stop the timer here all the time. Or that
         * it should be stopped here.
         */
        /* CH592: Stop timer and disable radio operations */
        R8_TMR0_CTRL_MOD &= ~RB_TMR_COUNT_EN;  // Stop TMR0
        R32_TMR0_COUNT = 0;                     // Clear counter

        /* Disable radio interrupts */
        LL->INT_EN = 0;

        /* Stop radio if active */
        DevSetMode(0);

        /* Clear any pending operations */
        if (LL->LL0 & 3) {
            LL->CTRL_MOD &= CTRL_MOD_RFSTOP;
            LL->LL0 |= 0x08;
        }

        assert(transition == BLE_PHY_TRANSITION_NONE);
    }
}

static inline uint8_t
ble_phy_get_cur_rx_phy_mode(void)
{
    uint8_t phy;

    phy = g_ble_phy_data.phy_cur_phy_mode;

#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_CODED_PHY)
    /*
     * For Coded PHY mode can be set to either codings since actual coding is
     * set in packet header. However, here we need actual coding of received
     * packet as this determines pipeline delays so need to figure this out
     * using CI field.
     */
    if ((phy == BLE_PHY_MODE_CODED_125KBPS) ||
                                    (phy == BLE_PHY_MODE_CODED_500KBPS)) {
        phy = NRF_RADIO->PDUSTAT & RADIO_PDUSTAT_CISTAT_Msk ?
                                   BLE_PHY_MODE_CODED_500KBPS :
                                   BLE_PHY_MODE_CODED_125KBPS;
    }
#endif

    return phy;
}

static void
ble_phy_rx_end_isr(void)
{
    int rc;
    uint8_t *dptr;
    uint8_t crcok;
    uint32_t tx_time;
    uint32_t radio_time;
    uint16_t tifs;
    struct ble_mbuf_hdr *ble_hdr;
    bool is_late;

    /* Disable automatic RXEN */
    /* CH592: Disable automatic RX operations */
    /* Stop TMR0 if it was scheduling RX */
    R8_TMR0_CTRL_MOD &= ~RB_TMR_COUNT_EN;

    /* Disable radio interrupts for RX */
    LL->INT_EN &= ~0x0F; // Clear RX-related interrupt enables

    /* Stop radio if in RX mode */
    if ((LL->CTRL_MOD & 0xFF) == DEVSETMODE_RX) {
        DevSetMode(0);
    }

    /* Set RSSI and CRC status flag in header */
    ble_hdr = &g_ble_phy_data.rxhdr;\

    /* CH592: Read RSSI from baseband register */
    ble_hdr->rxinfo.rssi = ReadRSSI();

    dptr = (uint8_t *)&g_ble_phy_rx_buf[0];
    dptr += 3;

    /* Count PHY crc errors and valid packets */
    /* CH592: Read CRC status from link layer */
    crcok = (LL->STATUS & 0x01) ? 1 : 0;
    if (!crcok) {
        STATS_INC(ble_phy_stats, rx_crc_err);
    } else {
        STATS_INC(ble_phy_stats, rx_valid);
        ble_hdr->rxinfo.flags |= BLE_MBUF_HDR_F_CRC_OK;
#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_ENCRYPTION)
        if (g_ble_phy_data.phy_encrypted) {
            while (NRF_CCM->EVENTS_ENDCRYPT == 0) {
                /* Make sure CCM finished */
            };

            /* Only set MIC failure flag if frame is not zero length */
            if ((dptr[1] != 0) && (NRF_CCM->MICSTATUS == 0)) {
                ble_hdr->rxinfo.flags |= BLE_MBUF_HDR_F_MIC_FAILURE;
            }

            /*
             * XXX: not sure how to deal with this. This should not
             * be a MIC failure but we should not hand it up. I guess
             * this is just some form of rx error and that is how we
             * handle it? For now, just set CRC error flags
             */
            if (NRF_CCM->EVENTS_ERROR) {
                STATS_INC(ble_phy_stats, rx_hw_err);
                ble_hdr->rxinfo.flags &= ~BLE_MBUF_HDR_F_CRC_OK;
            }
        }
#endif
    }

#if (BLE_LL_BT5_PHY_SUPPORTED == 1)
    ble_phy_mode_apply(g_ble_phy_data.phy_tx_phy_mode);
#endif

    /*
     * Let's schedule TX now and we will just cancel it after processing RXed
     * packet if we don't need TX.
     *
     * We need this to initiate connection in case AUX_CONNECT_REQ was sent on
     * LE Coded S8. In this case the time we process RXed packet is roughly the
     * same as the limit when we need to have TX scheduled (i.e. TIMER0 and PPI
     * armed) so we may simply miss the slot and set the timer in the past.
     *
     * When TX is scheduled in advance, we may event process packet a bit longer
     * during radio ramp-up - this gives us extra 40 usecs which is more than
     * enough.
     */

#if MYNEWT_VAL(BLE_PHY_VARIABLE_TIFS)
    tifs = g_ble_phy_data.tifs;
    g_ble_phy_data.tifs = BLE_LL_IFS;
#else
    tifs = BLE_LL_IFS;
#endif

    /* Schedule TX exactly T_IFS after RX end captured in CC[2] */
    tx_time = g_ble_phy_data.g_tx_end_time + tifs;
    /* Adjust for delay between actual RX end time and EVENT_END */
    tx_time -= g_ble_phy_t_rxenddelay[ble_hdr->rxinfo.phy_mode];


    /* Adjust for delay between EVENT_READY and actual TX start time */
    tx_time -= g_ble_phy_t_txdelay[g_ble_phy_data.phy_cur_phy_mode];

    radio_time = tx_time - BLE_PHY_T_TXENFAST;
    /* CH592: Set up timer to trigger TX at precise time */
    R8_TMR0_CTRL_MOD &= ~RB_TMR_COUNT_EN;  // Stop timer
    R32_TMR0_COUNT = 0;                     // Clear counter
    R32_TMR0_CNT_END = radio_time;          // Set compare value for TX timing
    R8_TMR0_INT_FLAG = RB_TMR_IF_CYC_END;  // Clear interrupt flag
    R8_TMR0_CTRL_MOD |= RB_TMR_COUNT_EN;    // Start timer


    /* Need to check if TIMER0 did not already count past CC[0] and/or CC[2], so
     * we're not stuck waiting for events in case radio and/or PA was not
     * started. If event was triggered we're fine regardless of timer value.
     *
     * Note: CC[3] is used only for wfr which we do not need here.
     */
    is_late = (R32_TMR0_COUNT > radio_time);

    if (is_late) {
        R8_TMR0_CTRL_MOD &= ~RB_TMR_COUNT_EN;  // Stop timer
        g_ble_phy_data.phy_transition_late = 1;
    }

    /*
     * XXX: This is a horrible ugly hack to deal with the RAM S1 byte
     * that is not sent over the air but is present here. Simply move the
     * data pointer to deal with it. Fix this later.
     */
    dptr[2] = dptr[1];
    dptr[1] = dptr[0];
    rc = ble_ll_rx_end(dptr + 1, ble_hdr);
    if (rc < 0) {
        ble_phy_disable();
    }
}

static bool
ble_phy_rx_start_isr(void)
{
    int rc;
    uint32_t state;
    uint32_t usecs;
    uint32_t pdu_usecs;
    uint32_t ticks;
    struct ble_mbuf_hdr *ble_hdr;
    uint8_t *dptr;
#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LL_PRIVACY)
    int adva_offset;
#endif
    g_ble_phy_data.g_aa_timestamp = R32_TMR0_COUNT;
    dptr = (uint8_t *)&g_ble_phy_rx_buf[0];

    /* CH592: Clear RX start event and interrupt */
    LL->STATUS &= ~0x02;  // Clear ADDRESS event (bit 1)
    LL->INT_EN &= ~0x02;  // Disable ADDRESS interrupt

    /* No WFR timer cleanup needed - CH592 has no PPI */

    /* Initialize the ble mbuf header */
    ble_hdr = &g_ble_phy_data.rxhdr;
    ble_hdr->rxinfo.flags = ble_ll_state_get();
    ble_hdr->rxinfo.channel = g_ble_phy_data.phy_chan;
    ble_hdr->rxinfo.handle = 0;
    ble_hdr->rxinfo.phy = ble_phy_get_cur_phy();
    ble_hdr->rxinfo.phy_mode = ble_phy_get_cur_rx_phy_mode();
#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LL_EXT_ADV)
    ble_hdr->rxinfo.user_data = NULL;
#endif

    /*
     * Calculate accurate packets start time (with remainder)
     *
     * We may start receiving packet somewhere during preamble in which case
     * it is possible that actual transmission started before TIMER0 was
     * running - need to take this into account.
     */
    ble_hdr->beg_cputime = g_ble_phy_data.phy_start_cputime;

    usecs = g_ble_phy_data.g_aa_timestamp;
    pdu_usecs = ble_phy_mode_pdu_start_off(ble_hdr->rxinfo.phy_mode) +
                g_ble_phy_t_rxaddrdelay[ble_hdr->rxinfo.phy_mode];
    if (usecs < pdu_usecs) {
        g_ble_phy_data.phy_start_cputime--;
        usecs += 30;
    }
    usecs -= pdu_usecs;

    ticks = os_cputime_usecs_to_ticks(usecs);
    usecs -= os_cputime_ticks_to_usecs(ticks);
    if (usecs == 31) {
        usecs = 0;
        ++ticks;
    }

    ble_hdr->beg_cputime += ticks;
    ble_hdr->rem_usecs = usecs;

    /* CH592: Wait to get 1st byte of frame */
    while (1) {
        state = LL->STATUS;

        /* Check if we have received first byte (bit pattern match) */
        if (state & 0x04) {  // Assuming bit 2 is BCMATCH equivalent
            break;
        }

        /* If RX is disabled/stopped, something went wrong */
        if ((LL->CTRL_MOD & 0x07) == 0) {  // RX mode bits cleared
            LL->INT_EN = 0;  // Disable all interrupts
            LL->STATUS = 0xFFFFFFFF;  // Clear all status bits
            return false;
        }

#if BABBLESIM
        tm_tick();
#endif
    }


#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LL_PRIVACY)
    /*
     * If privacy is enabled and received PDU has TxAdd bit set (i.e. random
     * address) we try to resolve address using AAR.
     */
    if (g_ble_phy_data.phy_privacy && (dptr[3] & 0x40)) {
        /*
         * AdvA is located at 4th octet in RX buffer (after S0, length an S1
         * fields). In case of extended advertising PDU we need to add 2 more
         * octets for extended header.
         */
        adva_offset = (dptr[3] & 0x0f) == 0x07 ? 2 : 0;
        NRF_AAR->ADDRPTR = (uint32_t)(dptr + 3 + adva_offset);

        /* Trigger AAR after last bit of AdvA is received */
        NRF_RADIO->EVENTS_BCMATCH = 0;
        phy_ppi_radio_bcmatch_to_aar_start_enable();
        nrf_radio_bcc_set(NRF_RADIO, (BLE_LL_PDU_HDR_LEN + adva_offset +
            BLE_DEV_ADDR_LEN) * 8 + g_ble_phy_data.phy_bcc_offset);
    }
#endif

    /* Call Link Layer receive start function */
    rc = ble_ll_rx_start(dptr + 3,
                         g_ble_phy_data.phy_chan,
                         &g_ble_phy_data.rxhdr);
    if (rc >= 0) {
        /* Set rx started flag and enable rx end ISR */
        g_ble_phy_data.phy_rx_started = 1;
    } else {
        /* Disable PHY */
        ble_phy_disable();
        STATS_INC(ble_phy_stats, rx_aborts);
    }

    /* Count rx starts */
    STATS_INC(ble_phy_stats, rx_starts);

    return true;
}

static void
ble_phy_isr(void)
{
    uint32_t status;
    uint32_t irq_en;

    os_trace_isr_enter();
    // Check if this is a TX ADDRESS event (access address transmitted)
    if (g_ble_phy_data.phy_state == BLE_PHY_STATE_TX) {
        // Capture AA timestamp when address is transmitted
        g_ble_phy_data.g_aa_timestamp = R32_TMR0_COUNT;
    }
    /* Read irq register to determine which interrupts are enabled */
    irq_en = LL->INT_EN;
    status = LL->STATUS;
    {
        LL->STATUS &= LL->INT_EN;
        BB->CTRL_TX = (BB->CTRL_TX & 0xfffffffc) | 1;
    }
    DevSetMode(0);
    LL->CTRL_MOD &= CTRL_MOD_RFSTOP;
    LL->LL0 |= 0x08;

    /* Clear NVIC pending */
    // NVIC->IPRR[0] = (1 << (LLE_IRQn & 0x1F));

    /*
     * NOTE: order of checking is important! Possible, if things get delayed,
     * we have both an ADDRESS and DISABLED interrupt in rx state. If we get
     * an address, we disable the DISABLED interrupt.
     */

    /* We get this if we have started to receive a frame */
    /* Check if ADDRESS event occurred (access address received) */
    // TODO: 0x02 is taken out of ass
    if ((irq_en & 0x02) && (LL->STATUS & 0x02)) {  // Bit 1 for ADDRESS event
        /*
         * wfr timer is calculated to expire at the exact time we should start
         * receiving a packet (with 1 usec precision) so it is possible it will
         * fire at the same time as ADDRESS event. If this happens, radio will
         * be disabled while we are waiting for first byte match after 1st byte
         * of payload is received and ble_phy_rx_start_isr() will fail. In this
         * case we should not clear DISABLED irq mask so it will be handled as
         * regular radio disabled event below. In other case radio was disabled
         * on purpose and there's nothing more to handle so we can clear mask.
         */
        if (ble_phy_rx_start_isr()) {
            irq_en &= ~0x01;  // Clear DISABLED interrupt mask (bit 0)
        }
    }


    /* Handle disabled event. This is enabled for both TX and RX. On RX, we
     * need to check phy_rx_started flag to make sure we actually were receiving
     * a PDU, otherwise this is due to wfr.
     */
    /* Check if DISABLED event occurred and is enabled */
    if ((irq_en & 0x01) && (LL->STATUS & 0x01)) {  // Bit 0 for DISABLED event
        // On CH592, we may not have separate END/DISABLED events
        // Need to verify the actual status bits available

        /* Clear the status flags */
        LL->STATUS = 0x01;  // Clear DISABLED status bit

        /* Disable the DISABLED interrupt */
        LL->INT_EN &= ~0x01;  // Clear bit 0

        switch (g_ble_phy_data.phy_state) {
            case BLE_PHY_STATE_RX:
                if (g_ble_phy_data.phy_rx_started) {
                    ble_phy_rx_end_isr();
                } else {
                    ble_ll_wfr_timer_exp(NULL);
                }
                break;
            case BLE_PHY_STATE_TX:
                ble_phy_tx_end_isr();
                break;
            default:
                BLE_LL_ASSERT(0);
        }
    }

    g_ble_phy_data.phy_transition_late = 0;

    /* Count # of interrupts */
    STATS_INC(ble_phy_stats, phy_isrs);

    os_trace_isr_exit();

}

#if PHY_USE_HEADERMASK_WORKAROUND
static void
ble_phy_ccm_isr(void)
{
    volatile uint8_t *tx_buf = (uint8_t *)g_ble_phy_tx_buf;

    if (NRF_CCM->EVENTS_ENDKSGEN) {
        while (tx_buf[0] == 0xff);
        tx_buf[0] = g_ble_phy_data.phy_headerbyte;
        NRF_CCM->INTENCLR = CCM_INTENCLR_ENDKSGEN_Msk;
    }
}
#endif

/**
 * ble phy init
 *
 * Initialize the PHY.
 *
 * @return int 0: success; PHY error code otherwise
 */
int
ble_phy_init(void)
{
    int rc;

    /* Default phy to use is 1M */
    g_ble_phy_data.phy_cur_phy_mode = BLE_PHY_MODE_1M;
    g_ble_phy_data.phy_tx_phy_mode = BLE_PHY_MODE_1M;
    g_ble_phy_data.phy_rx_phy_mode = BLE_PHY_MODE_1M;

    /* Set phy channel to an invalid channel so first set channel works */
    g_ble_phy_data.phy_chan = BLE_PHY_NUM_CHANS;

#if MYNEWT_VAL(BLE_PHY_VARIABLE_TIFS)
    g_ble_phy_data.tifs = BLE_LL_IFS;
#endif

    /* Toggle peripheral power to reset (just in case) */
    // nrf_radio_power_set(NRF_RADIO, false);
    // nrf_radio_power_set(NRF_RADIO, true);
    RFCoreInit(LL_TX_POWER_0_DBM);

#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_ENCRYPTION)
    nrf_ccm_int_disable(NRF_CCM, 0xffffffff);
    NRF_CCM->SHORTS = CCM_SHORTS_ENDKSGEN_CRYPT_Msk;
    NRF_CCM->EVENTS_ERROR = 0;
    memset(g_nrf_encrypt_scratchpad, 0, sizeof(g_nrf_encrypt_scratchpad));

#if PHY_USE_HEADERMASK_WORKAROUND
    NVIC_SetVector(CCM_AAR_IRQn, (uint32_t)ble_phy_ccm_isr);
    NVIC_EnableIRQ(CCM_AAR_IRQn);
    NRF_CCM->INTENCLR = CCM_INTENCLR_ENDKSGEN_Msk;;
#endif
#endif

#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LL_PRIVACY)
    g_ble_phy_data.phy_aar_scratch = 0;
    NRF_AAR->IRKPTR = (uint32_t)&g_nrf_irk_list[0];
    nrf_aar_int_disable(NRF_AAR, 0xffffffff);
    NRF_AAR->EVENTS_END = 0;
    NRF_AAR->EVENTS_RESOLVED = 0;
    NRF_AAR->EVENTS_NOTRESOLVED = 0;
    NRF_AAR->NIRK = 0;
#endif

    /* TIMER0 setup for PHY when using RTC */
    // nrf_timer_task_trigger(NRF_TIMER0, NRF_TIMER_TASK_STOP);
    // NRF_TIMER0->TASKS_SHUTDOWN = 1;
    // NRF_TIMER0->BITMODE = 3;    /* 32-bit timer */
    // NRF_TIMER0->MODE = 0;       /* Timer mode */
    // NRF_TIMER0->PRESCALER = 4;  /* gives us 1 MHz */
    //
    // phy_ppi_init();

#if PHY_USE_DEBUG
    phy_debug_init();
#endif

    /* Set isr in vector table and enable interrupt */
#ifndef RIOT_VERSION
#ifdef FREERTOS
    NVIC_SetPriority(RADIO_IRQn, 5);
#else
    // NVIC_SetPriority(RADIO_IRQn, 0); TODO: not sure if this exists
#endif
#endif
#if MYNEWT
    NVIC_SetVector(RADIO_IRQn, (uint32_t)ble_phy_isr);
#else
    ble_npl_hw_set_isr(RADIO_IRQn, ble_phy_isr);
#endif
    NVIC_EnableIRQ(RADIO_IRQn);

    /* Register phy statistics */
    if (!g_ble_phy_data.phy_stats_initialized) {
        rc = stats_init_and_reg(STATS_HDR(ble_phy_stats),
                                STATS_SIZE_INIT_PARMS(ble_phy_stats,
                                                      STATS_SIZE_32),
                                STATS_NAME_INIT_PARMS(ble_phy_stats),
                                "ble_phy");
        assert(rc == 0);

        g_ble_phy_data.phy_stats_initialized  = 1;
    }

    return 0;
}

/**
 * Puts the phy into receive mode.
 *
 * @return int 0: success; BLE Phy error code otherwise
 */
static int
ble_phy_rx(void)
{
    /*
     * Check radio state.
     *
     * In case radio is now disabling we'll wait for it to finish, but if for
     * any reason it's just in idle state we proceed with RX as usual since
     * nRF52 radio can ramp-up from idle state as well.
     *
     * Note that TX and RX states values are the same except for 3rd bit so we
     * can make a shortcut here when checking for idle state.
     */
    nrf_wait_disabled();

    // Capture timestamp when enabling RX
    g_ble_phy_data.g_rxen_timestamp = R32_TMR0_COUNT;

    /* Check radio state - CH592 approach */

    /* CH592 doesn't have granular state checking like nRF52.
     * Instead, ensure radio is not in the middle of an operation.
     * Check if radio is busy by examining status flags.
     */

    // TODO: we essentially need to replicate without setting address
    // Frame_RX(accessAddress, channelSet, PHY_1M);

    /* Wait for any ongoing operation to complete */
    uint32_t timeout = 1000;
    while ((LL->STATUS & 0x20000) && timeout--) {  // 0x20000 = LL_STATUS_TX
        /* Radio is transmitting, wait for completion */
    }

    if (timeout == 0) {
        ble_phy_disable();
        STATS_INC(ble_phy_stats, radio_state_errs);
        return BLE_PHY_ERR_RADIO_STATE;
    }

    /* Check if CTRL_MOD indicates radio is active */
    if ((LL->CTRL_MOD & 0x00FF) != 0) {
        /* Radio is in active mode (not idle), attempt to stop it */
        LL->CTRL_MOD &= CTRL_MOD_RFSTOP;
        DevSetMode(0);

        /* Wait for mode change */
        timeout = 1000;
        while ((LL->CTRL_MOD & 0x00FF) && timeout--);

        if (timeout == 0) {
            STATS_INC(ble_phy_stats, radio_state_errs);
            return BLE_PHY_ERR_RADIO_STATE;
        }
    }
    /* Make sure all interrupts are disabled */
    LL->INT_EN = 0;
    /* Clear all pending status flags (write-1-to-clear) */
    LL->STATUS = 0xFFFFFFFF;

    /* Setup for rx */
    ble_phy_rx_xcvr_setup();

    return 0;
}

#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_ENCRYPTION)
void
ble_phy_encrypt_enable(const uint8_t *key)
{
    memcpy(g_nrf_ccm_data.key, key, 16);
    g_ble_phy_data.phy_encrypted = 1;
    NRF_AAR->ENABLE = AAR_ENABLE_ENABLE_Disabled;
    NRF_CCM->ENABLE = CCM_ENABLE_ENABLE_Enabled;
#ifdef NRF5340_XXAA
    NRF_CCM->HEADERMASK = BLE_LL_PDU_HEADERMASK_DATA;
#endif
#if PHY_USE_HEADERMASK_WORKAROUND
    g_ble_phy_data.phy_headermask = BLE_LL_PDU_HEADERMASK_DATA;
#endif
}

void
ble_phy_encrypt_header_mask_set(uint8_t mask)
{
#ifdef NRF5340_XXAA
    NRF_CCM->HEADERMASK = mask;
#endif
#if PHY_USE_HEADERMASK_WORKAROUND
    g_ble_phy_data.phy_headermask = mask;
#endif
}

void
ble_phy_encrypt_iv_set(const uint8_t *iv)
{
    memcpy(g_nrf_ccm_data.iv, iv, 8);
}

void
ble_phy_encrypt_counter_set(uint64_t counter, uint8_t dir_bit)
{
    g_nrf_ccm_data.pkt_counter = counter;
    g_nrf_ccm_data.dir_bit = dir_bit;
}

void
ble_phy_encrypt_disable(void)
{
    phy_ppi_radio_address_to_ccm_crypt_disable();
    nrf_ccm_task_trigger(NRF_CCM, NRF_CCM_TASK_STOP);
    NRF_CCM->EVENTS_ERROR = 0;
    NRF_CCM->ENABLE = CCM_ENABLE_ENABLE_Disabled;

    g_ble_phy_data.phy_encrypted = 0;
}
#endif

void
ble_phy_set_txend_cb(ble_phy_tx_end_func txend_cb, void *arg)
{
    /* Set transmit end callback and arg */
    g_ble_phy_data.txend_cb = txend_cb;
    g_ble_phy_data.txend_arg = arg;
}

/**
 * Called to set the start time of a transmission.
 *
 * This function is called to set the start time when we are not going from
 * rx to tx automatically.
 *
 * NOTE: care must be taken when calling this function. The channel should
 * already be set.
 *
 * @param cputime   This is the tick at which the 1st bit of the preamble
 *                  should be transmitted
 * @param rem_usecs This is used only when the underlying timing uses a 32.768
 *                  kHz crystal. It is the # of usecs from the cputime tick
 *                  at which the first bit of the preamble should be
 *                  transmitted.
 * @return int
 */
int
ble_phy_tx_set_start_time(uint32_t cputime, uint8_t rem_usecs)
{
    int rc;

    ble_phy_trace_u32x2(BLE_PHY_TRACE_ID_START_TX, cputime, rem_usecs);

#if (BLE_LL_BT5_PHY_SUPPORTED == 1)
    ble_phy_mode_apply(g_ble_phy_data.phy_tx_phy_mode);
#endif

    if (ble_phy_set_start_time(cputime, rem_usecs, true) != 0) {
        STATS_INC(ble_phy_stats, tx_late);
        ble_phy_disable();
        rc = BLE_PHY_ERR_TX_LATE;
    } else {
        /* Enable PPI to automatically start TXEN */
        // phy_ppi_timer0_compare0_to_radio_txen_enable();
        rc = 0;
    }

    return rc;
}

/**
 * Called to set the start time of a reception
 *
 * This function acts a bit differently than transmit. If we are late getting
 * here we will still attempt to receive.
 *
 * NOTE: care must be taken when calling this function. The channel should
 * already be set.
 *
 * @param cputime
 *
 * @return int
 */
int
ble_phy_rx_set_start_time(uint32_t cputime, uint8_t rem_usecs)
{
    bool late = false;
    int rc = 0;

    ble_phy_trace_u32x2(BLE_PHY_TRACE_ID_START_RX, cputime, rem_usecs);

#if (BLE_LL_BT5_PHY_SUPPORTED == 1)
    ble_phy_mode_apply(g_ble_phy_data.phy_rx_phy_mode);
#endif

    if (ble_phy_set_start_time(cputime, rem_usecs, false) != 0) {
        STATS_INC(ble_phy_stats, rx_late);

        /* We're late so let's just try to start RX as soon as possible */
        ble_phy_set_start_now();

        late = true;
    }

    /* Enable PPI to automatically start RXEN */
    // phy_ppi_timer0_compare0_to_radio_rxen_enable();

    /* Start rx */
    rc = ble_phy_rx();

    /*
     * If we enabled receiver but were late, let's return proper error code so
     * caller can handle this.
     */
    if (!rc && late) {
        rc = BLE_PHY_ERR_RX_LATE;
    }

    return rc;
}

int
ble_phy_tx(ble_phy_tx_pducb_t pducb, void *pducb_arg, uint8_t end_trans)
{
    int rc;
    uint8_t *dptr;
    uint8_t *pktptr;
    uint8_t payload_len;
    uint8_t hdr_byte;
    uint32_t state;
    uint32_t shortcuts;

    if (g_ble_phy_data.phy_transition_late) {
        ble_phy_disable();
        STATS_INC(ble_phy_stats, tx_late);
        return BLE_PHY_ERR_TX_LATE;
    }

    /*
     * This check is to make sure that the radio is not in a state where
     * it is moving to disabled state. If so, let it get there.
     */
    nrf_wait_disabled();


#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_ENCRYPTION)
    if (g_ble_phy_data.phy_encrypted) {
        dptr = (uint8_t *)&g_ble_phy_enc_buf[0];
        pktptr = (uint8_t *)&g_ble_phy_tx_buf[0];
        NRF_CCM->SHORTS = CCM_SHORTS_ENDKSGEN_CRYPT_Msk;
        NRF_CCM->INPTR = (uint32_t)dptr;
        NRF_CCM->OUTPTR = (uint32_t)pktptr;
        NRF_CCM->SCRATCHPTR = (uint32_t)&g_nrf_encrypt_scratchpad[0];
        NRF_CCM->EVENTS_ERROR = 0;
        NRF_CCM->MODE = CCM_MODE_LENGTH_Msk | ble_phy_get_ccm_datarate();
        NRF_CCM->CNFPTR = (uint32_t)&g_nrf_ccm_data;
    } else {
#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LL_PRIVACY)
        NRF_AAR->IRKPTR = (uint32_t)&g_nrf_irk_list[0];
#endif
        dptr = (uint8_t *)&g_ble_phy_tx_buf[0];
        pktptr = dptr;
    }
#else
    dptr = (uint8_t *)&g_ble_phy_tx_buf[0];
    pktptr = dptr;
#endif

    /* Set PDU payload */
    payload_len = pducb(&dptr[3], pducb_arg, &hdr_byte);

    /* RAM representation has S0, LENGTH and S1 fields. (3 bytes) */
    dptr[0] = hdr_byte;
    dptr[1] = payload_len;
    dptr[2] = 0;

#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_ENCRYPTION)
    /* Start key-stream generation and encryption (via short) */
    if (g_ble_phy_data.phy_encrypted) {
#if PHY_USE_HEADERMASK_WORKAROUND
        if (g_ble_phy_data.phy_headermask != BLE_LL_PDU_HEADERMASK_DATA) {
            g_ble_phy_data.phy_headerbyte = dptr[0];
            dptr[0] &= g_ble_phy_data.phy_headermask;
            g_ble_phy_tx_buf[0] = 0xffffffff;
            NRF_CCM->EVENTS_ENDKSGEN = 0;
            NRF_CCM->INTENSET = CCM_INTENSET_ENDKSGEN_Msk;
        }
#endif
        nrf_ccm_task_trigger(NRF_CCM, NRF_CCM_TASK_KSGEN);
    }
#endif
    // TODO: need to replicate
    // Frame_TX(accessAddress, pktptr, payload_len, channelSet, PHY_1M, crcInit);
    BB->CTRL_TX = (BB->CTRL_TX & 0xfffffffc) | 1;

    // Uncomment to disable whitening to debug RF.
    //BB->CTRL_CFG |= (1<<6);
    DevSetMode(DEVSETMODE_TX);
    LL->TXBUF = (uint32_t)pktptr;

    // default 1M for now TODO: may not be needed?
    BB->CTRL_CFG = (g_ble_phy_data.phy_tx_phy_mode == BLE_PHY_MODE_1M) ? CTRL_CFG_PHY_2M: CTRL_CFG_PHY_1M;


    /* Clear all pending status flags (write-1-to-clear) */
    LL->STATUS = LL_STATUS_TX;

    /* CH592 doesn't have hardware shortcuts like nRF52 */
    /* The radio state transitions are handled automatically by the hardware */
    /* READY->START and END->DISABLE happen implicitly */

    /* Enable the interrupt for radio operations complete */
    LL->INT_EN = (1 << 0);  // Enable bit 0 - basic radio completion interrupt

    /* Set the PHY transition */
    g_ble_phy_data.phy_transition = end_trans;

    /* Set transmitted payload length */
    g_ble_phy_data.phy_tx_pyld_len = payload_len;

    /* If we already started transmitting, abort it! */
    state = LL->STATUS;
    if (!(state & LL_STATUS_TX)) {
        /* Set phy state to transmitting and count packet statistics */
        g_ble_phy_data.phy_state = BLE_PHY_STATE_TX;
        STATS_INC(ble_phy_stats, tx_good);
        STATS_INCN(ble_phy_stats, tx_bytes, payload_len + BLE_LL_PDU_HDR_LEN);
        rc = BLE_ERR_SUCCESS;
    } else {
        ble_phy_disable();
        STATS_INC(ble_phy_stats, tx_late);
        rc = BLE_PHY_ERR_RADIO_STATE;
    }

    return rc;
}

// LL power array [-18, -10, -5, -3, 0, 1, 2, 3, 4, 5, 6, 7] dBm
int phy_txpower_round(int dbm) {
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

    return dbm_rounded;
}

/**
 * ble phy txpwr set
 *
 * Set the transmit output power (in dBm).
 *
 * NOTE: If the output power specified is within the BLE limits but outside
 * the chip limits, we "rail" the power level so we dont exceed the min/max
 * chip values.
 *
 * @param dbm Power output in dBm.
 *
 * @return int 0: success; anything else is an error
 */
int
ble_phy_tx_power_set(int dbm)
{
    /* Get actual TX power supported by radio */
    dbm = phy_txpower_round(dbm);

    DevInit(dbm);
    g_ble_phy_data.phy_txpwr_dbm = dbm;

    return 0;
}

/**
 * ble phy txpwr round
 *
 * Get the rounded transmit output power (in dBm).
 *
 * @param dbm Power output in dBm.
 *
 * @return int Rounded power in dBm
 */
int
ble_phy_tx_power_round(int dbm)
{
    return phy_txpower_round(dbm);
}

/**
 * ble phy set access addr
 *
 * Set access address.
 *
 * @param access_addr Access address
 *
 * @return int 0: success; PHY error code otherwise
 */
static int
ble_phy_set_access_addr(uint32_t access_addr)
{
    BB->ACCESSADDRESS1 = access_addr;

    g_ble_phy_data.phy_access_address = access_addr;

    return 0;
}

/**
 * ble phy txpwr get
 *
 * Get the transmit power.
 *
 * @return int  The current PHY transmit power, in dBm
 */
int
ble_phy_tx_power_get(void)
{
    return g_ble_phy_data.phy_txpwr_dbm;
}

/**
 * ble phy setchan
 *
 * Sets the logical frequency of the transceiver. The input parameter is the
 * BLE channel index (0 to 39, inclusive). The NRF frequency register works like
 * this: logical frequency = 2400 + FREQ (MHz).
 *
 * Thus, to get a logical frequency of 2402 MHz, you would program the
 * FREQUENCY register to 2.
 *
 * @param chan This is the Data Channel Index or Advertising Channel index
 *
 * @return int 0: success; PHY error code otherwise
 */
int
ble_phy_setchan(uint8_t chan, uint32_t access_addr, uint32_t crcinit)
{
    assert(chan < BLE_PHY_NUM_CHANS);

    /* Check for valid channel range */
    if (chan >= BLE_PHY_NUM_CHANS) {
        return BLE_PHY_ERR_INV_PARAM;
    }

    /* Set current access address */
    ble_phy_set_access_addr(access_addr);
    // TODO: this needs massive overhaul
    /* Configure crcinit */
    BB->CRCINIT1 = crcinit;
    /* Set the frequency and the data whitening initial value */
    g_ble_phy_data.phy_chan = chan;
    DevSetChannel(chan); // OR g_ble_phy_chan_freq[chan];
    // NRF_RADIO->DATAWHITEIV = chan;

    return 0;
}

uint8_t
ble_phy_chan_get(void)
{
    return g_ble_phy_data.phy_chan;
}

/**
 * Stop the timer used to count microseconds when using RTC for cputime
 */
static void
ble_phy_stop_usec_timer(void)
{
    // TODO: will we be using TMR0 or built in LL->TMR ?
    LL->TMR = 0;
}

/**
 * ble phy disable irq and ppi
 *
 * This routine is to be called when reception was stopped due to either a
 * wait for response timeout or a packet being received and the phy is to be
 * restarted in receive mode. Generally, the disable routine is called to stop
 * the phy.
 */
static void
ble_phy_disable_irq_and_ppi(void)
{
    /* Disable all LL interrupts */
    LL->INT_EN = 0;

    /* Stop the radio by clearing mode */
    DevSetMode(0);

    /* Stop RF module */
    LL->CTRL_MOD &= CTRL_MOD_RFSTOP;

    /* Clear pending interrupt */
    NVIC->IPRR[0] = (1 << (LLE_IRQn & 0x1F));

    /* Clear LL status flags */
    LL->STATUS = 0xFFFFFFFF;

    g_ble_phy_data.phy_state = BLE_PHY_STATE_IDLE;
}

void
ble_phy_restart_rx(void)
{
    ble_phy_stop_usec_timer();
    ble_phy_disable_irq_and_ppi();

    ble_phy_set_start_now();
    /* CH592 doesn't have PPI - RX start is handled directly by Frame_RX() */

    ble_phy_rx();
}

/**
 * ble phy disable
 *
 * Disables the PHY. This should be called when an event is over. It stops
 * the usec timer (if used), disables interrupts, disables the RADIO, disables
 * PPI and sets state to idle.
 */
void
ble_phy_disable(void)
{
    ble_phy_trace_void(BLE_PHY_TRACE_ID_DISABLE);

#if PHY_USE_HEADERMASK_WORKAROUND
    NRF_CCM->INTENCLR = CCM_INTENCLR_ENDKSGEN_Msk;
#endif

    ble_phy_stop_usec_timer();
    ble_phy_disable_irq_and_ppi();

    g_ble_phy_data.phy_transition_late = 0;

}

/* Gets the current access address */
uint32_t ble_phy_access_addr_get(void)
{
    return g_ble_phy_data.phy_access_address;
}

/**
 * Return the phy state
 *
 * @return int The current PHY state.
 */
int
ble_phy_state_get(void)
{
    return g_ble_phy_data.phy_state;
}

/**
 * Called to see if a reception has started
 *
 * @return int
 */
int
ble_phy_rx_started(void)
{
    return g_ble_phy_data.phy_rx_started;
}

/**
 * Return the transceiver state
 *
 * @return int transceiver state.
 */
uint8_t
ble_phy_xcvr_state_get(void)
{
    uint32_t mode = LL->CTRL_MOD;

    /* Check if radio is in TX mode */
    if ((mode & 0xFF) == DEVSETMODE_TX) {
        return 1; /* TX state */
    }
    /* Check if radio is in RX mode */
    else if ((mode & 0xFF) == DEVSETMODE_RX) {
        return 2; /* RX state */
    }
    /* Otherwise idle/disabled */
    return 0;
}

/**
 * Called to return the maximum data pdu payload length supported by the
 * phy. For this chip, if encryption is enabled, the maximum payload is 27
 * bytes.
 *
 * @return uint8_t Maximum data channel PDU payload size supported
 */
uint8_t
ble_phy_max_data_pdu_pyld(void)
{
    return BLE_LL_DATA_PDU_MAX_PYLD;
}

#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LL_PRIVACY)
void
ble_phy_resolv_list_enable(void)
{
    NRF_AAR->NIRK = (uint32_t)g_nrf_num_irks;
    g_ble_phy_data.phy_privacy = 1;
}

void
ble_phy_resolv_list_disable(void)
{
    g_ble_phy_data.phy_privacy = 0;
}
#endif

#if MYNEWT_VAL(BLE_LL_DTM)
void ble_phy_enable_dtm(void)
{
    /* When DTM is enabled we need to disable whitening as per
     * Bluetooth v5.0 Vol 6. Part F. 4.1.1
     */
    NRF_RADIO->PCNF1 &= ~RADIO_PCNF1_WHITEEN_Msk;
}

void ble_phy_disable_dtm(void)
{
    /* Enable whitening */
    NRF_RADIO->PCNF1 |= RADIO_PCNF1_WHITEEN_Msk;
}
#endif

void
ble_phy_rfclk_enable(void)
{
    // TODO: Probably not needed on CH5xx
}

void
ble_phy_rfclk_disable(void)
{
    // TODO: Probably not needed on CH5xx
}

void
ble_phy_tifs_txtx_set(uint16_t usecs, uint8_t anchor)
{
    g_ble_phy_data.txtx_time_us = usecs;
    g_ble_phy_data.txtx_time_anchor = anchor;
}