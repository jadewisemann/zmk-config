/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <errno.h>
#include <nrf.h>
#include <esb_tdma.h>
#include <stddef.h>
#include <string.h>
#include <nrf_erratas.h>

#include <hal/nrf_radio.h>
#include <hal/nrf_timer.h>
#include <nrfx_timer.h>
#include <nrfx_rtc.h>
#include <hal/nrf_clock.h>

#ifdef NRF53_SERIES
#include "hal/nrf_vreqctrl.h"
#endif /* NRF53_SERIES */

#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/random/random.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

#include <mpsl_fem_protocol_api.h>

#include "esb_peripherals.h"
#include "esb_ppi_api.h"
#include "tx_api.h"

LOG_MODULE_REGISTER(esb, CONFIG_ZMK_ESB_LOG_LEVEL);

// #define LOG_ALL
// #define MONITORING_WORK
// #define PERIPHERAL_LOG_ALL
#define CENTRAL_LOG_TS_END
#define PERIPHERAL_LOG_TS_END

#ifdef LOG_ALL
#define CENTRAL_LOG_ALL
#define PERIPHERAL_LOG_ALL
#endif

#ifdef CENTRAL_LOG_ALL
#define CENTRAL_LOG_TS_START
#define CENTRAL_LOG_TS_END
#endif

#ifdef PERIPHERAL_LOG_ALL
#define PERIPHERAL_LOG_DS_START
#define PERIPHERAL_LOG_DS_END
#define PERIPHERAL_LOG_SYNC
#define PERIPHERAL_LOG_TS_START
#define PERIPHERAL_LOG_TS_END
#endif

/* Constants */

/* 2 Mb RX wait for acknowledgment time-out value.
 * Smallest reliable value: 160.
 */
#define RX_ACK_TIMEOUT_US_2MBPS	    160
/* 1 Mb RX wait for acknowledgment time-out value. */
#define RX_ACK_TIMEOUT_US_1MBPS	    300
/* 250 Kb RX wait for acknowledgment time-out value. */
#define RX_ACK_TIMEOUT_US_250KBPS   300
/* 1 Mb RX wait for acknowledgment time-out (combined with BLE). */
#define RX_ACK_TIMEOUT_US_1MBPS_BLE 300
/* 4 Mb RX wait for acknowledgment time-out value. */
#define RX_ACK_TIMEOUT_US_4MBPS	    160

/* Minimum retransmit time */
#define RETRANSMIT_DELAY_MIN 435

/* Radio Tx ramp-up time in microseconds. */
#define TX_RAMP_UP_TIME_US 129

/* Radio Rx fast ramp-up time in microseconds. */
#define TX_FAST_RAMP_UP_TIME_US 40

/* Radio Rx ramp-up time in microseconds. */
#define RX_RAMP_UP_TIME_US 124

/* Interrupt flags */
/* Interrupt mask value for TX success. */
#define INT_TX_SUCCESS_MSK	 BIT(0)
/* Interrupt mask value for TX failure. */
#define INT_TX_FAILED_MSK	 BIT(1)
/* Interrupt mask value for RX_DR. */
#define INT_RX_DATA_RECEIVED_MSK BIT(2)
#define INT_PERIPHERAL_SYNC_MSK	 BIT(3)

/* Mask value to signal updating BASE0 radio address. */
#define ADDR_UPDATE_MASK_BASE0	BIT(0)
/* Mask value to signal updating BASE1 radio address. */
#define ADDR_UPDATE_MASK_BASE1	BIT(1)
/* Mask value to signal updating radio prefixes. */
#define ADDR_UPDATE_MASK_PREFIX BIT(2)

/* Radio address event latency in microseconds. */
#define ADDR_EVENT_LATENCY_US (13)

/* The maximum value for PID. */
#define PID_MAX 3

#define BIT_MASK_UINT_8(x) (0xFF >> (8 - (x)))

/* Radio base frequency. */
#define RADIO_BASE_FREQUENCY 2400UL

/* NRF5340 Radio high voltage gain. */
#define NRF5340_HIGH_VOLTAGE_GAIN 3

/* Fast switching is available for the nRF54H20 SoC.
 * The nRF54H20 is a non-RSSISTOP device.
 */
#if defined(RADIO_SHORTS_DISABLED_RSSISTOP_Msk)
#define RADIO_SHORTS_COMMON                                                                        \
	(NRF_RADIO_SHORT_READY_START_MASK | ESB_SHORT_DISABLE_MASK |                               \
	 NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK | NRF_RADIO_SHORT_DISABLED_RSSISTOP_MASK)
#else
/* Devices without RSSISTOP task will stop RSSI measurement after specific period. */
#define RADIO_SHORTS_FAST_SWITCHING_NO_RSSISTOP                                                    \
	(NRF_RADIO_SHORT_READY_START_MASK | NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK)
#define RADIO_SHORTS_NO_FAST_SWITCHING_NO_RSSISTOP                                                 \
	(NRF_RADIO_SHORT_READY_START_MASK | ESB_SHORT_DISABLE_MASK |                               \
	 NRF_RADIO_SHORT_ADDRESS_RSSISTART_MASK)

#define RADIO_SHORTS_COMMON                                                                        \
	(IS_ENABLED(CONFIG_ZMK_ESB_FAST_SWITCHING) ? RADIO_SHORTS_FAST_SWITCHING_NO_RSSISTOP           \
					       : RADIO_SHORTS_NO_FAST_SWITCHING_NO_RSSISTOP)
#endif /* !defined(RADIO_SHORTS_DISABLED_RSSISTOP_Msk) */

#define RADIO_SHORTS_BASIC (NRF_RADIO_SHORT_READY_START_MASK | ESB_SHORT_DISABLE_MASK)

/* Flag for changing radio channel. */
#define RF_CHANNEL_UPDATE_FLAG 0

/* Empty trim value */
#define TRIM_VALUE_EMPTY 0xFFFFFFFF

#define ERRATA_216_PRESENT DT_NODE_HAS_STATUS(DT_NODELABEL(cpurad_cpusys_errata216_mboxes), okay)
#define ERRATA_216_RADIO_ENABLE_DELAY_US  40
#define ERRATA_216_MIN_TIME_TO_DISABLE_US 100

/* Internal Enhanced ShockBurst module state. */
enum esb_state {
	ESB_STATE_IDLE, /* Idle. */
	ESB_STATE_CENTRAL_TX,

	ESB_STATE_PERIPHERAL_DESYNC,
	ESB_STATE_PERIPHERAL_RX,
	ESB_STATE_PERIPHERAL_RX_READY,
	ESB_STATE_PERIPHERAL_TX_ACK,
};

/* Pipe info PID and CRC and acknowledgment payload. */
struct pipe_info {
	uint16_t crc; /* CRC of the last received packet.
		       * Used to detect retransmits.
		       */
	bool sn;
	bool nesn;
	uint16_t tx_try;
	bool synced;
	// bool ack_payload; /* State of the transmission of ACK payloads. */
};

/* First-in, first-out queue of received payloads. */
struct payload_rx_fifo {
	/* Payload queue */
	struct esb_payload *payload[CONFIG_ZMK_ESB_RX_FIFO_SIZE];

	uint32_t back;	/* Back of the queue (last in). */
	uint32_t front; /* Front of queue (first out). */
	uint32_t count; /* Number of elements in the queue. */
};

/* Fixed radio PDU header definition. */
struct esb_radio_fixed_pdu {
	/* Packet ID of the last received packet. Used to detect retransmits. */
	uint8_t pid: 2;
	uint8_t rfu: 6;
	uint8_t rfu1;
} __packed;

/* Dynamic length radio PDU header definition. */
struct esb_radio_dynamic_pdu {
	/* Payload length. */
#if ESB_MAX_PAYLOAD_LENGTH > 63
	uint8_t length;
#else
	uint8_t length: 6;
	uint8_t rfu0: 2;
#endif /* ESB_MAX_PAYLOAD_LENGTH > 63 */

	/* Disable acknowledge. */
	uint8_t no_ack: 1;

	/* Packet ID of the last received packet. Used to detect retransmits. */
	uint8_t pid: 2;
	uint8_t rfu1: 5;
} __packed;

struct my_esb_radio_pdu {
	uint8_t length: 6;
	uint8_t rfu0: 2;
	uint8_t ctrl: 1;
	uint8_t more: 1;
	uint8_t sn: 1;
	uint8_t nesn: 1;
	uint8_t rfu1: 4;
} __packed;

/* Radio PDU header definition. */
union esb_radio_pdu_type {
	/* Fixed PDU header. */
	struct esb_radio_fixed_pdu fixed_pdu;

	/* Dynamic PDU header. */
	struct esb_radio_dynamic_pdu dpl_pdu;
} __packed;

/* Radio PDU definition. */
struct esb_radio_pdu {
	/* PDU header. */
	struct my_esb_radio_pdu pdu;

	/* PDU data. */
	uint8_t data[];
} __packed;

/* Enhanced ShockBurst address.
 *
 * Enhanced ShockBurst addresses consist of a base address and a prefix
 * that is unique for each pipe. See @ref esb_addressing in the ESB user
 * guide for more information.
 */
struct esb_address {
	uint8_t base_addr_p0[4];   /* Base address for pipe 0, in big endian. */
	uint8_t base_addr_p1[4];   /* Base address for pipe 1-7, in big endian. */
	uint8_t pipe_prefixes[8];  /* Address prefix for pipe 0 to 7. */
	uint8_t num_pipes;	   /* Number of pipes available. */
	uint8_t addr_length;	   /* Length of the address plus the prefix. */
	uint8_t rx_pipes_enabled;  /* Bitfield for enabled pipes. */
	uint8_t rf_channel;	   /* Channel to use (between 0 and 100). */
	atomic_t rf_channel_flags; /* Flags for setting the channel. */
};

static nrfx_timer_t esb_timer = ESB_NRFX_TIMER_INSTANCE;
static nrfx_rtc_t esb_rtc = ESB_NRFX_RTC_INSTANCE;

static bool esb_initialized;
static struct esb_config esb_cfg;
static volatile enum esb_state esb_state = ESB_STATE_IDLE;

static struct esb_tdma_context {
	uint32_t last_hb; // last heartbeat timer value
	int32_t drift;	  // drift between central and peripheral, us per heartbeat * SCALE
	int32_t drift_now; // drift correction (ticks) applied to the current rx start
	int32_t sync_rem_us; // what rounding last_hb to a tick left over (true - rounded)
	uint8_t scan_margin; // extra window margin (ticks) until the first re-sync after a scan
	uint8_t margin_now;  // window margin (ticks) of the current rx start
	uint32_t refslot;	 // refslot from central
	uint32_t hb_loops;	 // calculated hb loops
	uint32_t slotsize_ticks; // slot size
	uint32_t slotsize_us;	 // slot size in us
	uint32_t window_size;	 // slot window size (for rx, tx)
	uint32_t window_size_us;
	uint32_t start;	       // current radio start timer value
	uint32_t timeout;      // current timeout timer value
	uint32_t last_tx;      // last fired tx
	uint32_t desync_count; // number of times that peripheral has desynced
	uint32_t slotdiff;
	uint8_t pipe;		// current pipe(fixed for periheral)
	uint16_t timeout_count; // desync count
	uint16_t timeout_count_max;
	uint8_t addr_delay;  // calcualted addr delay
	uint8_t channel_idx; // current channel index(for rssi, tx_power)
	uint8_t pipes;	     // number of pipes that are enabled

	uint8_t scan_count;  // scans since sync was lost (saturates at DESYNC_FAST_SCANS)
	uint32_t resync_loops; // loops until the next re-sync; grows up to hb_loops
	uint32_t drift_span;   // ticks the drift estimate was measured over (at most a heartbeat)

	bool is_hb; // true if current slot is heartbeat
	bool hello_pending; // the first packet after a scan has to be the self-report
	bool hb_failed;
	bool more_data;
} ctx;

struct esb_ctrl_packet {
	uint32_t refslot;
	uint32_t slotsize_ticks;
	uint8_t addr_delay;
	uint8_t pipes;
	uint8_t version; /* ESB_PROTOCOL_VERSION of the central */
} __packed;

struct esb_header_dn {
	uint8_t rssi;
	// uint8_t dummy[3];
} __packed;

struct esb_header_up {
	uint8_t dummy[4];
} __packed;

struct esb_packet_dn {
	struct esb_header_dn header;
	uint8_t data[];
} __packed;

struct esb_packet_up {
	struct esb_header_up header;
	uint8_t data[];
} __packed;

struct esb_packet {
	struct esb_header header;
	uint8_t data[];
} __packed;

static sys_slist_t esb_conn_cb_list;
K_MSGQ_DEFINE(sync_event_msgq, sizeof(struct esb_evt), ESB_PIPE_COUNT * 2, 4);

#define ESB_CENTRAL    CONFIG_ZMK_ESB_CENTRAL
#define ESB_PERIPHERAL !CONFIG_ZMK_ESB_CENTRAL

#define TICK			   (30)
/* A drift measurement beyond this is taken for a bad capture, not a bad clock.
 * 4 % is twice what an uncalibrated RC low-frequency oscillator may be off by.
 */
#define DRIFT_LIMIT_PPM		   (40000)
#define DESYNC_LIMIT_MS_PERIPHERAL (500)
#define DESYNC_LIMIT_MS_CENTRAL	   (3000)

#define RADIO_MARGIN_TICKS   (3)
#define TIMEOUT_MARGIN_TICKS (4)
#define HFCLK_WARMUP_TICKS   (12)
/* How long before the expected slot start a peripheral opens its receiver.
 *
 * This is the whole budget for the central's clock running ahead of ours since
 * the last heartbeat, minus up to half a tick of rounding in last_hb. A packet
 * that arrives late is caught by the rest of the RX window (hundreds of us);
 * one that arrives early is simply missed. It was 2 ticks (61 us) with a 1 s
 * heartbeat, i.e. roughly 45 ppm between two 32 kHz crystals, which two
 * in-tolerance crystals can already exceed - and that half then lost every
 * heartbeat and cycled through desync. 5 ticks (152 us) over a 250 ms
 * heartbeat is about 550 ppm before the drift estimate has learned anything.
 */
#define WINDOW_MARGIN_TICKS  (5)
#define DRIFT_MARGIN_TICKS   (1)
#define ISR_MARGIN_TICKS     (5)

#define RADIO_MARGIN_US	  (k_ticks_to_us_near32(RADIO_MARGIN_TICKS))
#define TIMEOUT_MARGIN_US (k_ticks_to_us_near32(TIMEOUT_MARGIN_TICKS))
#define HFCLK_WARMUP_US	  (k_ticks_to_us_near32(HFCLK_WARMUP_TICKS))
#define WINDOW_MARGIN_US  (k_ticks_to_us_near32(WINDOW_MARGIN_TICKS))
#define DRIFT_MARGIN_US	  (k_ticks_to_us_near32(DRIFT_MARGIN_TICKS))
#define ISR_MARGIN_US	  (k_ticks_to_us_near32(ISR_MARGIN_TICKS))

/* How often an idle peripheral wakes up to re-sync with the central, in RTC
 * ticks (250 ms). It bounds how much clock drift has to be predicted, and how
 * long a command from the central waits for an idle peripheral. One heartbeat
 * is an RX window of a few hundred us plus a 4-byte reply.
 */
#define HEARTBEAT_TICKS (32768 / 4)

/* After a scan nothing is known about the clocks, and a whole heartbeat is far
 * too long to wait: the window margin over 250 ms only covers ~600 ppm, and a
 * peripheral whose clock was further behind the central's missed its first
 * heartbeat, timed out, scanned again and never got to measure anything. So
 * the first re-syncs follow each other quickly - after RESYNC_FIRST_LOOPS
 * loops, then RESYNC_GROWTH times longer each time, up to the heartbeat - and
 * each one measures the drift for the next, longer one. The first interval
 * (4 ms with two halves at 1 kHz) tolerates a few percent of clock error.
 */
#define RESYNC_FIRST_LOOPS (2)
#define RESYNC_GROWTH	   (4)

#ifdef CONFIG_ZMK_ESB_FAST_SWITCHING_PERIPHERAL
#define ESB_FAST_SWITCHING_PERIPHERAL 1
#else
#define ESB_FAST_SWITCHING_PERIPHERAL 0
#endif

// static uint64_t channel_map;
#define CHANNELS	39
#define CHANNEL_MASK(x) ((x) - 1)
#define CENTRAL_PIPE	(7)

#define DESYNC_AIRTIME_DEFAULT (250000)
/* A peripheral without sync scans: it listens on one channel until a packet for
 * its pipe arrives there. The first scans after start-up or after losing sync
 * follow each other directly, so a link that only hiccuped is back within a
 * fraction of a second. After that one scan starts per second (as every scan
 * used to), to spare the battery while the dongle is not there at all.
 */
#define DESYNC_FAST_SCANS     (10)
/* A scan measures where the slot is with the 1 MHz timer, over up to 400 ms,
 * while slots are scheduled on the 32 kHz clock. The two disagree by however
 * far the crystals are apart, so until the first re-sync the window opens
 * earlier by this share of the measured time, at most DESYNC_SCAN_MARGIN_MAX.
 */
#define DESYNC_SCAN_MARGIN_PPM (500)
#define DESYNC_SCAN_MARGIN_MAX (5)
#define DESYNC_SCAN_LEAD_TICKS (HFCLK_WARMUP_TICKS + ISR_MARGIN_TICKS + RADIO_MARGIN_TICKS)
#define SCALE		       (1024)
#define ALPHA		       (120)
#define RSSI_BASELINE	       (63)
#define RSSI_DIFF_THRESHOLD    (4)
#define RSSI_DIFF_THRESHOLD_UP (4)
#define RSSI_DIFF_THRESHOLD_DN (-2)
#define TX_POWER_MAX	       (4)
#define TX_POWER_MIN	       (-20)
#define TX_POWER_BASE	       (-4)
#define TX_POWER_STEP	       (8)
#define TX_POWER_STEP_DN       (-2)
#define TX_POWER_SAMPLE_CNT    (8)
#define TPMAX_SCALED	       (TX_POWER_MAX * SCALE)
#define TPMIN_SCALED	       (TX_POWER_MIN * SCALE)
#define TPSTEP_SCALED	       (TX_POWER_STEP * SCALE)

static struct pipe_info rx_pipe_info[ESB_PIPE_COUNT];
static struct pipe_info *rx_pipe_info_get(uint8_t pipe)
{
	return &rx_pipe_info[resolve_pipe(pipe)];
}

static struct payload_rx_fifo rx_fifo;

static void reset_rx_fifo(void)
{
	rx_fifo.back = 0;
	rx_fifo.front = 0;
	rx_fifo.count = 0;
}

static void initialize_rx_fifo(void)
{
	static struct esb_payload rx_payload[CONFIG_ZMK_ESB_RX_FIFO_SIZE];

	for (size_t i = 0; i < CONFIG_ZMK_ESB_RX_FIFO_SIZE; i++) {
		rx_fifo.payload[i] = &rx_payload[i];
	}

	reset_rx_fifo();
}

static inline uint32_t hash(uint32_t x)
{
	x *= 0x9E3779B1;
	x = (x ^ (x >> 16)) * 0x85ebca6b;
	x = (x ^ (x >> 13)) * 0xc2b2ae35;
	x = x ^ (x >> 16);

	return x;
}

static uint8_t get_channel(uint32_t seq)
{
	uint32_t x = hash(seq);
	ctx.channel_idx = x % CHANNELS;
	return 1 + (ctx.channel_idx * 2);
}

static void _set_slotsize(void)
{
	ctx.slotsize_us = k_ticks_to_us_near32(ctx.slotsize_ticks);

	ctx.window_size = ctx.slotsize_ticks / 2;
	ctx.window_size_us = ctx.slotsize_us / 2;

	LOG_WRN("slot %u window %u slots %u", ctx.slotsize_ticks, ctx.window_size, ctx.pipes);
	LOG_WRN("slot %u window %u", ctx.slotsize_us, ctx.window_size_us);
}

static void set_slotsize_us(uint32_t slotsize_us)
{
	ctx.slotsize_ticks = k_us_to_ticks_ceil32(slotsize_us);

	_set_slotsize();
}

static void set_slotsize_ticks(uint32_t slotsize_ticks)
{
	ctx.slotsize_ticks = slotsize_ticks;

	_set_slotsize();
}

static void set_hb_loops(void)
{
	uint32_t loop_size = ctx.slotsize_ticks * ctx.pipes;
	ctx.hb_loops = MAX(HEARTBEAT_TICKS / loop_size, 1);

	// LOG_WRN("HEARTBEAT LOOPS(%u) SIZE(%u)", ctx.hb_loops, ctx.hb_loops * loop_size);
}

static void set_control_packet(void *data)
{
	if (data == NULL) {
		return;
	}

	struct esb_ctrl_packet *control = (struct esb_ctrl_packet *)data;

	control->slotsize_ticks = ctx.slotsize_ticks;
	control->addr_delay = ctx.addr_delay;
	control->pipes = ctx.pipes;
	control->refslot = ctx.refslot;
	control->version = ESB_PROTOCOL_VERSION;
}

static void apply_control_packet(void *data)
{
	if (data == NULL) {
		return;
	}

	struct esb_ctrl_packet *control = (struct esb_ctrl_packet *)data;

	ctx.slotsize_ticks = control->slotsize_ticks;
	set_slotsize_ticks(ctx.slotsize_ticks);
	ctx.pipes = control->pipes;
	ctx.addr_delay = control->addr_delay;
	ctx.refslot = control->refslot;

	ctx.timeout_count_max =
		k_ms_to_ticks_near32(DESYNC_LIMIT_MS_PERIPHERAL) / (ctx.slotsize_ticks * ctx.pipes);

	// LOG_WRN("slot %u window %u slots %u to max %u", ctx.slotsize_ticks, ctx.window_size,
	// ctx.pipes, 	ctx.timeout_count_max);
	set_hb_loops();
}

int moving_average(int new_val, int before)
{
	return (ALPHA * new_val + (SCALE - ALPHA) * before) / SCALE;
}

static void sync_op_work_cb(struct k_work *work)
{
	struct esb_evt evt;
	while (k_msgq_get(&sync_event_msgq, &evt, K_NO_WAIT) == 0) {
		struct esb_conn_cb *cb;
#if ESB_CENTRAL
		// LOG_WRN("pipe %d %s", evt.sync.pipe, evt.sync.up ? "connected" : "disconnected");
#else
		// LOG_WRN("central %s", evt.sync.up ? "connected" : "disconnected");
#endif
		SYS_SLIST_FOR_EACH_CONTAINER(&esb_conn_cb_list, cb, node) {
			if (evt.sync.up) {
				if (cb->connected) {
					cb->connected(evt.sync.pipe);
				}
			} else {
				if (cb->disconnected) {
					cb->disconnected(evt.sync.pipe);
				}
			}
		}
	}
}
K_WORK_DEFINE(sync_op_work, sync_op_work_cb);

static bool hfclk_is_on = false;

static void hfclk_on(void)
{
	z_nrf_clock_bt_ctlr_hf_request();
	hfclk_is_on = true;
	// LOG_WRN("HF clock on.");
}

static void hfclk_off(void)
{
	z_nrf_clock_bt_ctlr_hf_release();
	hfclk_is_on = false;

	// LOG_WRN("HF clock released.");
}

#if ESB_CENTRAL

static struct peripheral_slot {
	uint32_t last_sync[ESB_PIPE_COUNT];
	uint8_t pipe_state;
} slots;

static bool is_slot_synced(uint8_t pipe)
{
	return (slots.pipe_state & BIT(resolve_pipe(pipe)));
}

static bool slot_sync_check(uint8_t pipe, uint32_t ref)
{
	const uint32_t timeout = k_ms_to_ticks_near32(DESYNC_LIMIT_MS_CENTRAL);

	bool in_sync = (NRF_RTC_WRAP(ref - slots.last_sync[resolve_pipe(pipe)]) <= timeout);
	WRITE_BIT(slots.pipe_state, resolve_pipe(pipe), in_sync);

	return in_sync;
}

static void slot_sync_update(uint8_t pipe, uint32_t ref)
{
	slots.last_sync[resolve_pipe(pipe)] = ref;
	WRITE_BIT(slots.pipe_state, resolve_pipe(pipe), true);
}

uint8_t esb_get_slot_state(void)
{
	return slots.pipe_state;
}
#else
uint8_t esb_get_slot_state(void)
{
	return BIT(ctx.pipe);
}
#endif

static uint32_t RSSI[ESB_PIPE_COUNT];
static bool rssi_initialized[ESB_PIPE_COUNT];

static void rssi_reset(uint8_t pipe)
{
	rssi_initialized[resolve_pipe(pipe)] = false;
}

static void rssi_reset_all(void)
{
	for (int pipe = 0; pipe < ARRAY_SIZE(RSSI); ++pipe) {
		// RSSI[pipe] = (RSSI_BASELINE * SCALE);
		rssi_reset(pipe);
	}
}

static void rssi_update(uint8_t pipe, uint8_t sample)
{
	uint32_t rssi_scaled = sample * SCALE;

	if (rssi_initialized[resolve_pipe(pipe)] == false) {
		rssi_initialized[resolve_pipe(pipe)] = true;
		RSSI[resolve_pipe(pipe)] = rssi_scaled;

		return;
	}

	RSSI[resolve_pipe(pipe)] = moving_average(rssi_scaled, RSSI[resolve_pipe(pipe)]);
}

static uint8_t rssi_get(uint8_t pipe)
{
	return (uint8_t)(RSSI[resolve_pipe(pipe)] / SCALE);
}

static int32_t TX_POWER[ESB_PIPE_COUNT];
static uint8_t settle_count[ESB_PIPE_COUNT];

static void tx_power_reset(uint8_t pipe)
{
	TX_POWER[resolve_pipe(pipe)] = TX_POWER_BASE;
}

static void tx_power_raise(uint8_t pipe)
{
	int new_power = TX_POWER[resolve_pipe(pipe)] + TX_POWER_STEP;
	TX_POWER[resolve_pipe(pipe)] = MIN(new_power, TX_POWER_MAX);
	settle_count[resolve_pipe(pipe)] = 0;
}

static int16_t tx_power_get(uint8_t pipe)
{
	return TX_POWER[resolve_pipe(pipe)];
}

static bool tx_power_update(uint8_t pipe, uint8_t rssi)
{
	int rssi_diff = rssi - RSSI_BASELINE;
	// if (abs(rssi_diff) < RSSI_DIFF_THRESHOLD) {
	// 	settle_count[resolve_pipe(pipe)] = 0;

	// 	return false;
	// }

	if (rssi_diff <= RSSI_DIFF_THRESHOLD_UP && rssi_diff >= RSSI_DIFF_THRESHOLD_DN) {
		settle_count[resolve_pipe(pipe)] = 0;

		return false;
	}

	if (++settle_count[resolve_pipe(pipe)] < TX_POWER_SAMPLE_CNT) {
		return false;
	}

	settle_count[resolve_pipe(pipe)] = 0;

	int new_power = TX_POWER[resolve_pipe(pipe)] + (rssi_diff / 2);

	TX_POWER[resolve_pipe(pipe)] = CLAMP(new_power, TX_POWER_MIN, TX_POWER_MAX);

	return true;
}

#if ESB_PERIPHERAL
static bool drift_initialized = true;
static inline void drift_reset(void)
{
	drift_initialized = true;
	ctx.drift = 0;
	ctx.drift_now = 0;
	ctx.sync_rem_us = 0;
	ctx.scan_margin = 0;
	ctx.resync_loops = RESYNC_FIRST_LOOPS;
	ctx.drift_span = 0;
}

/* Loops from one re-sync to the next: short right after a scan, then the heartbeat. */
static inline uint32_t resync_loops(void)
{
	return CLAMP(ctx.resync_loops, 1, MAX(ctx.hb_loops, 1));
}

static inline int32_t us_to_ticks_signed(int32_t us)
{
	return us >= 0 ? (int32_t)k_us_to_ticks_near32(us) : -(int32_t)k_us_to_ticks_near32(-us);
}

static inline int32_t ticks_to_us_signed(int32_t ticks)
{
	return ticks >= 0 ? (int32_t)k_ticks_to_us_near32(ticks)
			  : -(int32_t)k_ticks_to_us_near32(-ticks);
}

/* drift: us per heartbeat, measured over `span` ticks (at most a heartbeat).
 *
 * A measurement is only as good as it is long: the same microsecond of error
 * is 30 times more drift over 8 ms than over a heartbeat. So a longer
 * measurement replaces the estimate, an equally long one is averaged in, and a
 * shorter one is ignored.
 */
static inline void drift_update(int64_t drift, uint32_t span)
{
	int64_t drift_scaled = drift * SCALE;

	if (span < ctx.drift_span) {
		return;
	}

	if (drift_initialized || span > ctx.drift_span) {
		drift_initialized = false;
		ctx.drift = drift_scaled;
		ctx.drift_span = span;

		return;
	}

	ctx.drift = (drift_scaled + ctx.drift * 3) / 4;
}

static inline int32_t drift_get_us(uint32_t clock_diff)
{
	uint32_t hb_size = ctx.hb_loops * ctx.slotsize_ticks * ctx.pipes;
	if (hb_size == 0) {
		return 0;
	}

	uint32_t denominator = hb_size * SCALE;
	int64_t numerator = (int64_t)ctx.drift * clock_diff;
	int32_t drift = 0;

	if (numerator >= 0) {
		drift = (int32_t)((numerator + (denominator >> 1)) / denominator);
	} else {
		drift = (int32_t)((numerator - (denominator >> 1)) / denominator);
	}

	return drift;
}

/* How many ticks before the expected slot start the receiver opens. The RX
 * window is this plus half a slot, and it has to be over before the watchdog
 * compare that sits TIMEOUT_MARGIN_TICKS before the end of the slot - which
 * only gets tight at a 2 kHz polling rate.
 */
static inline int32_t window_margin_ticks(void)
{
	int32_t room = (int32_t)ctx.slotsize_ticks - TIMEOUT_MARGIN_TICKS - 1 -
		       (int32_t)ctx.window_size;

	return CLAMP(WINDOW_MARGIN_TICKS + ctx.scan_margin, 1, MAX(room, 1));
}

/* Rounded to the nearest tick, so the correction is off by at most half a tick
 * either way. (It used to round towards "earlier", which made any negative
 * estimate, however small, open the window a full tick early.)
 */
static inline int32_t drift_get_ticks(uint32_t clock_diff)
{
	return us_to_ticks_signed(drift_get_us(clock_diff));
}
#endif

/* Link statistics, for diagnostics only: nothing in the protocol reads them.
 * The central counts per pipe; a peripheral counts for itself and reports the
 * result in every packet that has no payload (see struct esb_peer_stats).
 */
#if ESB_CENTRAL
static struct esb_pipe_stats link_stats[ESB_PIPE_COUNT];
static uint32_t slots_skipped;
static uint32_t slots_stalled;

int esb_get_pipe_stats(uint8_t pipe, struct esb_pipe_stats *stats)
{
	if (pipe >= ESB_PIPE_COUNT || stats == NULL) {
		return -EINVAL;
	}

	unsigned int key = irq_lock();

	*stats = link_stats[pipe];
	stats->slots_skipped = slots_skipped;
	stats->slots_stalled = slots_stalled;
	stats->rssi = rssi_get(pipe);
	stats->synced = is_slot_synced(pipe);

	irq_unlock(key);

	return 0;
}
#else
/* The queue wait is kept as the maximum of the current and the previous
 * second of the RTC, so a report shows a slow delivery for 1-2 s after it
 * happened without anything having to be reset when a report is sent.
 */
#define QUEUE_WINDOW_SHIFT (15)
#define QUEUE_WINDOW_MASK  (NRF_RTC_WRAP(UINT32_MAX) >> QUEUE_WINDOW_SHIFT)

static struct {
	uint8_t rx_timeouts;
	uint8_t tx_retries;
	uint8_t scans;
	uint8_t queue_cur;
	uint8_t queue_prev;
	bool hfxo_was_off;    /* the 32 MHz crystal was not running at a wake-up */
	bool queue_pending;   /* a payload is waiting for its acknowledgement */
	uint32_t queue_since; /* RTC value when the oldest such payload was queued */
	uint32_t queue_window;
} self_stats;

static void queue_wait_roll(uint32_t now)
{
	uint32_t window = now >> QUEUE_WINDOW_SHIFT;

	if (window == self_stats.queue_window) {
		return;
	}

	bool next = window == ((self_stats.queue_window + 1) & QUEUE_WINDOW_MASK);

	self_stats.queue_prev = next ? self_stats.queue_cur : 0;
	self_stats.queue_cur = 0;
	self_stats.queue_window = window;
}

/* Called with interrupts locked, when a payload is put into the TX ring. */
static void queue_wait_start(void)
{
	if (!self_stats.queue_pending) {
		self_stats.queue_pending = true;
		self_stats.queue_since = nrfx_rtc_counter_get(&esb_rtc);
	}
}

/* Called from the radio interrupt, after acknowledged data left the TX ring. */
static void queue_wait_end(void)
{
	uint32_t now = nrfx_rtc_counter_get(&esb_rtc);

	if (self_stats.queue_pending) {
		uint32_t ms = NRF_RTC_WRAP(now - self_stats.queue_since) * 1000 / 32768;

		queue_wait_roll(now);
		self_stats.queue_cur = MAX(self_stats.queue_cur, MIN(ms, UINT8_MAX));
	}

	/* What is still in the ring was queued in the meantime. */
	self_stats.queue_pending = count_tx(0) > 0;
	self_stats.queue_since = now;
}

static uint8_t peer_stats_fill(uint8_t *data)
{
	struct esb_peer_stats *stats = (struct esb_peer_stats *)data;

	queue_wait_roll(nrfx_rtc_counter_get(&esb_rtc));

	stats->version = ESB_PROTOCOL_VERSION;
	stats->rx_timeouts = self_stats.rx_timeouts;
	stats->tx_retries = self_stats.tx_retries;
	stats->scans = self_stats.scans;
	stats->queue_max_ms = MAX(self_stats.queue_cur, self_stats.queue_prev);
	stats->tx_power = (int8_t)tx_power_get(ctx.pipe);

	int64_t hb_us = k_ticks_to_us_near32(MAX(ctx.hb_loops, 1) * ctx.slotsize_ticks * ctx.pipes);

	stats->drift_ppm = hb_us == 0 ? 0
				      : (int16_t)CLAMP((int64_t)ctx.drift * 1000000 / (hb_us * SCALE),
						       INT16_MIN, INT16_MAX);

	nrf_clock_lfclk_t lf_src;

	stats->clocks = 0;
	if (nrf_clock_is_running(NRF_CLOCK, NRF_CLOCK_DOMAIN_LFCLK, &lf_src) &&
	    lf_src == NRF_CLOCK_LFCLK_XTAL) {
		stats->clocks |= ESB_PEER_CLOCK_LF_XTAL;
	}
	if (self_stats.hfxo_was_off) {
		stats->clocks |= ESB_PEER_CLOCK_HF_WAS_OFF;
	}

	return sizeof(*stats);
}
#endif

/* Default address configuration for ESB.
 * Roughly equal to the nRF24Lxx defaults, except for the number of pipes,
 * because more pipes are supported.
 */
__ALIGN(4)
static struct esb_address esb_addr = {
	.base_addr_p0 = {0xE7, 0xE7, 0xE7, 0xE7},
	.base_addr_p1 = {0xC2, 0xC2, 0xC2, 0xC2},
	.pipe_prefixes = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8},
	.addr_length = 5,
	.num_pipes = CONFIG_ZMK_ESB_PIPE_COUNT,
	.rf_channel = 2,
	.rx_pipes_enabled = 0xFF};

enum {
	ERRATA_216_DISABLED,
	ERRATA_216_ENABLED,
};
static atomic_t errata_216_status = ATOMIC_INIT(ERRATA_216_DISABLED);
static uint32_t errata_216_timer_shorts;

#if ERRATA_216_PRESENT
static const struct mbox_dt_spec on_channel =
	MBOX_DT_SPEC_GET(DT_NODELABEL(cpurad_cpusys_errata216_mboxes), on_req);
static const struct mbox_dt_spec off_channel =
	MBOX_DT_SPEC_GET(DT_NODELABEL(cpurad_cpusys_errata216_mboxes), off_req);
#endif /* ERRATA_216_PRESENT */

static esb_event_handler event_handler;

static void set_addr_delay(void)
{
	uint8_t ramp_up_delay =
		esb_cfg.use_fast_ramp_up ? TX_FAST_RAMP_UP_TIME_US : TX_RAMP_UP_TIME_US;

	/* preamble(8 bits) + address(addr_length bytes × 8 bits) */
	uint8_t preamble_addr_pcf_bits = 8 + (esb_addr.addr_length * 8) + 9;

	/* airtime in µs: bits, half if 2Mbps */
	uint8_t addr_airtime = esb_cfg.bitrate == ESB_BITRATE_2MBPS ? preamble_addr_pcf_bits / 2
								    : preamble_addr_pcf_bits;

	ctx.addr_delay = ramp_up_delay + addr_airtime;
	// LOG_WRN("ADDR_DELAY(%u)", ctx.addr_delay);
}

#if ESB_CENTRAL
#ifdef MONITORING_WORK
void monitoring_work_cb(struct k_work *work)
{
	printk("pipe  rssi\n");
	printk("----  ----\n");
	for (int pipe = 0; pipe < ESB_PIPE_COUNT; ++pipe) {
		int rssi = -rssi_get(pipe);
		printk("%4u  %+4d dBm\n", pipe, rssi);
	}

	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	k_work_reschedule(dwork, K_SECONDS(1));
}
K_WORK_DELAYABLE_DEFINE(monitoring_work, monitoring_work_cb);
#endif
static void central_setup(void)
{
	nrf_timer_int_enable(esb_timer.p_reg, NRF_TIMER_INT_COMPARE1_MASK);
	nrf_radio_int_enable(NRF_RADIO, NRF_RADIO_INT_CRCOK_MASK);
	ctx.pipes = ESB_PIPE_COUNT;
	set_slotsize_us(USEC_PER_SEC / CONFIG_ZMK_ESB_POLLING_RATE);
	// set_slotsize_us(62500);

	set_hb_loops();
	set_addr_delay();
	rssi_reset_all();

	// rx address is fixed, but tx address is changed
	nrf_radio_rxaddresses_set(NRF_RADIO, BIT(CENTRAL_PIPE));

	ctx.refslot = sys_rand32_get();
	// LOG_WRN("SEED: %u", ctx.refslot);
}

#else
#ifdef MONITORING_WORK
void monitoring_work_cb(struct k_work *work)
{
	int rssi = -rssi_get(ctx.pipe);
	int tx_power_cfg = tx_power_get(ctx.pipe);
	int tx_power = (int8_t)nrf_radio_txpower_get(NRF_RADIO);

	printk("rssi cfg  reg\n");
	printk("---- ---- ----\n");
	printk("%+4d %+4d %+4d dBm\n", rssi, tx_power_cfg, tx_power);

	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	k_work_reschedule(dwork, K_SECONDS(1));
}
K_WORK_DELAYABLE_DEFINE(monitoring_work, monitoring_work_cb);
#endif

/* Picks the channel of each scan. The scans start in interrupt context, where
 * the entropy driver behind sys_rand32_get() may not be called (it can block),
 * and all a scan needs is to spread over the channels.
 */
static uint32_t scan_seed;

static void peripheral_setup(void)
{
	nrf_radio_int_enable(NRF_RADIO, NRF_RADIO_INT_DISABLED_MASK);
	scan_seed = sys_rand32_get();

	ctx.pipe = esb_cfg.pipe;

	tx_power_reset(ctx.pipe);

	// rx address is fixed, but tx address is changed
	nrf_radio_rxaddresses_set(NRF_RADIO, BIT(ctx.pipe));
#if !ESB_FAST_SWITCHING_PERIPHERAL
	nrf_radio_txaddress_set(NRF_RADIO, CENTRAL_PIPE);
#endif
}
#endif

void pto_context_reset(void)
{
	ctx.start = 0;
	ctx.timeout = 0;
	ctx.refslot = 0;
	ctx.desync_count = 0;
	ctx.channel_idx = 0;
	ctx.scan_count = 0;
}

/* Run time variables */
static volatile uint32_t interrupt_flags;
static volatile struct tx_evt last_tx_evt;
static volatile uint32_t wait_for_ack_timeout_us;

static uint32_t radio_shorts_common = RADIO_SHORTS_COMMON;
static const bool fast_switching = IS_ENABLED(CONFIG_ZMK_ESB_FAST_SWITCHING);

static const mpsl_fem_event_t rx_event = {
	.type = MPSL_FEM_EVENT_TYPE_TIMER,
	.event.timer =
		{
			.p_timer_instance = ESB_NRF_TIMER_INSTANCE,
			.compare_channel_mask =
				(BIT(NRF_TIMER_CC_CHANNEL2) | BIT(NRF_TIMER_CC_CHANNEL3)),
			.counter_period =
				{
					.end = RX_RAMP_UP_TIME_US,
				},
		},
};

static const mpsl_fem_event_t tx_event = {
	.type = MPSL_FEM_EVENT_TYPE_TIMER,
	.event.timer =
		{
			.p_timer_instance = ESB_NRF_TIMER_INSTANCE,
			.compare_channel_mask =
				(BIT(NRF_TIMER_CC_CHANNEL2) | BIT(NRF_TIMER_CC_CHANNEL3)),
			.counter_period =
				{
					.end = TX_RAMP_UP_TIME_US,
				},
		},
};

static mpsl_fem_event_t tx_time_shifted = {
	.type = MPSL_FEM_EVENT_TYPE_TIMER,
	.event.timer =
		{
			.p_timer_instance = ESB_NRF_TIMER_INSTANCE,
			.compare_channel_mask =
				(BIT(NRF_TIMER_CC_CHANNEL2) | BIT(NRF_TIMER_CC_CHANNEL3)),
		},
};

static mpsl_fem_event_t disable_event = {
	.type = MPSL_FEM_EVENT_TYPE_GENERIC,
};

/* These function pointers are changed dynamically, depending on protocol
 * configuration and state. Note that they will be 0 initialized.
 */
static void (*on_radio_disabled)(void);
static void (*on_timer_compare1)(void);
static void (*update_rf_payload_format)(uint32_t payload_length);

static void central_prepare_tx(void);
static void central_timeslot_end(void);
static void central_watchdog(void);

static void peripheral_start_desync(void);
static void peripheral_disabled_desync(void);
static void peripheral_prepare_rx(void);
static void peripheral_disabled_rx(void);
static void peripheral_disabled_tx_ack(void);

// static inline uint32_t

/*  Function to do bytewise bit-swap on an unsigned 32-bit value */
static uint32_t bytewise_bit_swap(const uint8_t *input)
{
#if __CORTEX_M == (0x04U)
	uint32_t inp = (*(uint32_t *)input);

	return sys_cpu_to_be32((uint32_t)__RBIT(inp));
#else
	uint32_t inp = sys_cpu_to_le32(*(uint32_t *)input);

	inp = (inp & 0xF0F0F0F0) >> 4 | (inp & 0x0F0F0F0F) << 4;
	inp = (inp & 0xCCCCCCCC) >> 2 | (inp & 0x33333333) << 2;
	inp = (inp & 0xAAAAAAAA) >> 1 | (inp & 0x55555555) << 1;
	return inp;
#endif
}

/* Convert a base address from nRF24L format to nRF5 format */
static uint32_t addr_conv(const uint8_t *addr)
{
	return BSWAP_32(bytewise_bit_swap(addr));
}

static inline void apply_errata143_workaround(void)
{
	/* Workaround for Errata 143
	 * Check if the most significant bytes of address 0 (including
	 * prefix) match those of another address. It's recommended to
	 * use a unique address 0 since this will avoid the 3dBm penalty
	 * incurred from the workaround.
	 */
	uint32_t base_address_mask = esb_addr.addr_length == 5 ? 0xFFFF0000 : 0xFF000000;

	/* Load the two addresses before comparing them to ensure
	 * defined ordering of volatile accesses.
	 */
	uint32_t addr0 = nrf_radio_base0_get(NRF_RADIO) & base_address_mask;
	uint32_t addr1 = nrf_radio_base1_get(NRF_RADIO) & base_address_mask;

	if (addr0 == addr1) {
		uint32_t radio_prefix0 = nrf_radio_prefix0_get(NRF_RADIO);
		uint32_t radio_prefix1 = nrf_radio_prefix1_get(NRF_RADIO);

		uint8_t prefix0 = radio_prefix0 & RADIO_PREFIX0_AP0_Msk;
		uint8_t prefix1 = (radio_prefix0 & RADIO_PREFIX0_AP1_Msk) >> RADIO_PREFIX0_AP1_Pos;
		uint8_t prefix2 = (radio_prefix0 & RADIO_PREFIX0_AP2_Msk) >> RADIO_PREFIX0_AP2_Pos;
		uint8_t prefix3 = (radio_prefix0 & RADIO_PREFIX0_AP3_Msk) >> RADIO_PREFIX0_AP3_Pos;
		uint8_t prefix4 = radio_prefix1 & RADIO_PREFIX1_AP4_Msk;
		uint8_t prefix5 = (radio_prefix1 & RADIO_PREFIX1_AP5_Msk) >> RADIO_PREFIX1_AP5_Pos;
		uint8_t prefix6 = (radio_prefix1 & RADIO_PREFIX1_AP6_Msk) >> RADIO_PREFIX1_AP6_Pos;
		uint8_t prefix7 = (radio_prefix1 & RADIO_PREFIX1_AP7_Msk) >> RADIO_PREFIX1_AP7_Pos;

		if ((prefix0 == prefix1) || (prefix0 == prefix2) || (prefix0 == prefix3) ||
		    (prefix0 == prefix4) || (prefix0 == prefix5) || (prefix0 == prefix6) ||
		    (prefix0 == prefix7)) {
			/* This will cause a 3dBm sensitivity loss,
			 * avoid using such address combinations if possible.
			 */
			*(volatile uint32_t *)0x40001774 =
				((*(volatile uint32_t *)0x40001774) & 0xfffffffe) | 0x01000000;
		}
	}
}

static void errata216_on(void)
{
#if ERRATA_216_PRESENT
	if (mbox_send_dt(&on_channel, NULL) != 0) {
		LOG_ERR("Failed to enable Errata 216");
		/* Should not happen. */
		__ASSERT_NO_MSG(false);
	} else {
		atomic_set(&errata_216_status, ERRATA_216_ENABLED);
	}
#endif /* ERRATA_216_PRESENT */
}

static void errata216_off(void)
{
#if ERRATA_216_PRESENT
	if (mbox_send_dt(&off_channel, NULL) != 0) {
		LOG_ERR("Failed to disable Errata 216");
		/* Should not happen. */
		__ASSERT_NO_MSG(false);
	} else {
		atomic_set(&errata_216_status, ERRATA_216_DISABLED);
	}
#endif /* ERRATA_216_PRESENT */
}

static void esb_fem_for_tx_set(bool ack)
{
	// uint32_t timer_shorts;

	// timer_shorts = NRF_TIMER_SHORT_COMPARE2_STOP_MASK;

	// if (ack) {
	// 	/* In case of expecting ACK reception, clear the timer to start counting from 0
	// 	 * on the RADIO_DISABLED event. This is needed to start external front-end module
	// 	 * with valid timeout because radio uses DISABLED_RXEN short.
	// 	 */
	// 	timer_shorts |= NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK;
	// }

	// if (mpsl_fem_pa_configuration_set(&tx_event, &disable_event) == 0) {
	// 	mpsl_fem_enable();
	// 	esb_ppi_for_fem_set();
	// } else {
	// 	/* We want to start counting ACK timeout and potential packet retransmission from
	// 	 * RADIO_DISABLED event so timer starts through EGU together with radio ramp-up,
	// 	 * we want to stop and clear it before RADIO_DISABLED event to start it again
	// 	 * when this event occurs. The timer value must be big enough to give us possibility
	// 	 * to reconfigure timer shorts before timer will be cleared by them.
	// 	 */
	// 	uint16_t ramp_up =
	// 		esb_cfg.use_fast_ramp_up ? TX_FAST_RAMP_UP_TIME_US : TX_RAMP_UP_TIME_US;
	// 	nrf_timer_cc_set(esb_timer.p_reg, NRF_TIMER_CC_CHANNEL2, ramp_up);
	// }

	// if (IS_ENABLED(CONFIG_ZMK_ESB_NEVER_DISABLE_TX)) {
	// 	uint32_t cc1 = nrfx_timer_capture_get(&esb_timer, NRF_TIMER_CC_CHANNEL1);
	// 	uint32_t cc2 = nrfx_timer_capture_get(&esb_timer, NRF_TIMER_CC_CHANNEL2);

	// 	if (cc1 > cc2) {
	// 		timer_shorts = NRF_TIMER_SHORT_COMPARE1_STOP_MASK;
	// 		timer_shorts |= NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK;
	// 	}
	// }

	// nrf_timer_shorts_set(esb_timer.p_reg, timer_shorts);
}

static void esb_fem_for_rx_set(void)
{
	// if (mpsl_fem_lna_configuration_set(&rx_event, &disable_event) == 0) {
	// 	mpsl_fem_enable();
	// 	esb_ppi_for_fem_set();
	// 	nrf_timer_shorts_set(esb_timer.p_reg, (NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK |
	// 					       NRF_TIMER_SHORT_COMPARE2_STOP_MASK));
	// }
}

static void esb_fem_for_ack_rx(void)
{
	/* Timer is running and timer's shorts and PPI connections have been configured. */
	mpsl_fem_pa_configuration_clear();
	mpsl_fem_lna_configuration_set(&rx_event, &disable_event);
}

static void esb_fem_for_tx_ack(void)
{
	/* Timer is running and timer's shorts and PPI connections have been configured. */
	mpsl_fem_lna_configuration_clear();
	mpsl_fem_pa_configuration_set(&tx_event, &disable_event);
}

static void esb_fem_reset(void)
{
	// #if NRF_TIMER_HAS_SHUTDOWN
	// 	nrf_timer_task_trigger(esb_timer.p_reg, NRF_TIMER_TASK_SHUTDOWN);
	// #else
	// 	nrf_timer_task_trigger(esb_timer.p_reg, NRF_TIMER_TASK_STOP);
	// 	nrf_timer_task_trigger(esb_timer.p_reg, NRF_TIMER_TASK_CLEAR);
	// #endif

	mpsl_fem_lna_configuration_clear();
	mpsl_fem_pa_configuration_clear();

	esb_ppi_for_fem_clear();

	mpsl_fem_deactivate_now(MPSL_FEM_ALL);
	mpsl_fem_disable();
}

static void esb_fem_lna_reset(void)
{
	// #if NRF_TIMER_HAS_SHUTDOWN
	// 	nrf_timer_task_trigger(esb_timer.p_reg, NRF_TIMER_TASK_SHUTDOWN);
	// #else
	// 	nrf_timer_task_trigger(esb_timer.p_reg, NRF_TIMER_TASK_STOP);
	// 	nrf_timer_task_trigger(esb_timer.p_reg, NRF_TIMER_TASK_CLEAR);
	// #endif

	esb_ppi_for_fem_clear();

	mpsl_fem_lna_configuration_clear();
	mpsl_fem_disable();
}

static void esb_fem_pa_reset(void)
{
	mpsl_fem_pa_configuration_clear();

	// #if NRF_TIMER_HAS_SHUTDOWN
	// 	nrf_timer_task_trigger(esb_timer.p_reg, NRF_TIMER_TASK_SHUTDOWN);
	// #else
	// 	nrf_timer_task_trigger(esb_timer.p_reg, NRF_TIMER_TASK_STOP);
	// 	nrf_timer_task_trigger(esb_timer.p_reg, NRF_TIMER_TASK_CLEAR);
	// #endif

	esb_ppi_for_fem_clear();

	mpsl_fem_disable();
}

void esb_fem_for_rx_ack(void)
{
	mpsl_fem_pa_configuration_clear();
	mpsl_fem_lna_configuration_set(&rx_event, &disable_event);

	/* The timer is running, shorts must be reconfigured because we do not want to
	 * stop timer after front-end module was triggered. Timer needs to count ACK reception
	 * timeout and potential retransmission delay. Timer cannot be stopped here because
	 * these timeout are counted since RADIO_DISABLED event.
	 */
	// nrf_timer_shorts_disable(esb_timer.p_reg, (NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK |
	// 					   NRF_TIMER_SHORT_COMPARE2_STOP_MASK));
}

void esb_fem_for_tx_retry(void)
{
	// /* The radio is ramped-up with delay set in the timer compare channel 1.
	//  * Calculate the ramp-up time for external front-end module.
	//  */
	// tx_time_shifted.event.timer.counter_period.end =
	// 	nrf_timer_cc_get(esb_timer.p_reg, NRF_TIMER_CC_CHANNEL1) + TX_RAMP_UP_TIME_US;

	// /* This starts the ESB TIMER on the radio disabled event. This connection is needed even
	//  * when front-end module is not used.
	//  */
	// esb_ppi_for_fem_set();

	// if (mpsl_fem_pa_configuration_set(&tx_time_shifted, &disable_event) == 0) {
	// 	/* In case of retransmission timer is configured to trigger RADIO_TXEN task after
	// 	 * retransmission timeout, but with external front-end module, it must also
	// 	 * schedule front-end module ramp-up which occurs later. Shorts need to be
	// 	 * reconfigured here to stop and clear the timer after front-end module will be
	// 	 * ramped-up.
	// 	 */
	// 	nrf_timer_shorts_set(esb_timer.p_reg, (NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK |
	// 					       NRF_TIMER_SHORT_COMPARE2_STOP_MASK));
	// }
}

void esb_fem_for_tx_retry_clear(void)
{
	// esb_ppi_for_fem_clear();

	// mpsl_fem_pa_configuration_clear();
	// mpsl_fem_deactivate_now(MPSL_FEM_ALL);

	// nrf_timer_shorts_disable(esb_timer.p_reg, (NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK |
	// 					   NRF_TIMER_SHORT_COMPARE2_STOP_MASK));
}

static void radio_start(void)
{
	// if (IS_ENABLED(ERRATA_216_PRESENT) &&
	//     atomic_get(&errata_216_status) == ERRATA_216_DISABLED) {
	// 	errata216_on();

	// 	nrfx_timer_compare(
	// 		&esb_timer, NRF_TIMER_CC_CHANNEL3,
	// 		nrfx_timer_us_to_ticks(&esb_timer, ERRATA_216_RADIO_ENABLE_DELAY_US), true);

	// 	errata_216_timer_shorts = esb_timer.p_reg->SHORTS;
	// 	nrf_timer_shorts_set(esb_timer.p_reg, (NRF_TIMER_SHORT_COMPARE3_STOP_MASK |
	// 					       NRF_TIMER_SHORT_COMPARE3_CLEAR_MASK));
	// 	nrfx_timer_clear(&esb_timer);
	// 	nrf_timer_task_trigger(ESB_NRF_TIMER_INSTANCE, NRF_TIMER_TASK_START);
	// } else {
	// 	/* Event generator unit is used to start radio and protocol timer if needed. */
	// 	nrf_egu_task_trigger(ESB_EGU, ESB_EGU_TASK);
	// }
}

static void update_rf_payload_format_esb_dpl(uint32_t payload_length)
{
	nrf_radio_packet_conf_t packet_config = {0};

	packet_config.s0len = 0;
	packet_config.s1len = 4;

	/* Using 6 bits or 8 bits for length */
	packet_config.lflen = (CONFIG_ZMK_ESB_MAX_PAYLOAD_LENGTH <= 32) ? 6 : 8;
#if CONFIG_ZMK_ESB_WHITEEN
	packet_config.whiteen = true;
#else
	packet_config.whiteen = false;
#endif
	packet_config.big_endian = true;
	packet_config.balen = (esb_addr.addr_length - 1);
	packet_config.statlen = 0;
	packet_config.maxlen = CONFIG_ZMK_ESB_MAX_PAYLOAD_LENGTH;
#if defined(RADIO_PCNF0_PLEN_Msk)
	if (esb_cfg.bitrate == ESB_BITRATE_2MBPS) {
		packet_config.plen = NRF_RADIO_PREAMBLE_LENGTH_16BIT;
	}

#if defined(RADIO_MODE_MODE_Ble_2Mbit)
	if (esb_cfg.bitrate == ESB_BITRATE_2MBPS_BLE) {
		packet_config.plen = NRF_RADIO_PREAMBLE_LENGTH_16BIT;
	}
#endif /* defined(RADIO_MODE_MODE_Ble_2Mbit) */

#if defined(RADIO_MODE_MODE_Nrf_4Mbit_0BT6)
	if (esb_cfg.bitrate == ESB_BITRATE_4MBPS) {
		packet_config.plen = NRF_RADIO_PREAMBLE_LENGTH_16BIT;
	}
#endif /* defined(RADIO_MODE_MODE_Nrf_4Mbit_0BT6) */

#endif /* defined(RADIO_PCNF0_PLEN_Msk) */

	nrf_radio_packet_configure(NRF_RADIO, &packet_config);
}

static void update_rf_payload_format_esb(uint32_t payload_length)
{
	const nrf_radio_packet_conf_t packet_config = {.s0len = 1,
						       .lflen = 0,
						       .s1len = 1,
						       .whiteen = false,
						       .big_endian = true,
						       .balen = (esb_addr.addr_length - 1),
						       .statlen = payload_length,
						       .maxlen = payload_length};

	nrf_radio_packet_configure(NRF_RADIO, &packet_config);
}

static void update_radio_addresses(uint8_t update_mask)
{
	if ((update_mask & ADDR_UPDATE_MASK_BASE0) != 0) {
		nrf_radio_base0_set(NRF_RADIO, addr_conv(esb_addr.base_addr_p0));
	}

	if ((update_mask & ADDR_UPDATE_MASK_BASE1) != 0) {
		nrf_radio_base1_set(NRF_RADIO, addr_conv(esb_addr.base_addr_p1));
	}

	if ((update_mask & ADDR_UPDATE_MASK_PREFIX) != 0) {
		nrf_radio_prefix0_set(NRF_RADIO, bytewise_bit_swap(&esb_addr.pipe_prefixes[0]));
		nrf_radio_prefix1_set(NRF_RADIO, bytewise_bit_swap(&esb_addr.pipe_prefixes[4]));
	}

	/* Workaround for Errata 143 */
#if NRF52_ERRATA_143_ENABLE_WORKAROUND
	if (nrf52_errata_143()) {
		apply_errata143_workaround();
	}
#endif
}

#if defined(CONFIG_SOC_SERIES_NRF54HX) || defined(CONFIG_SOC_SERIES_NRF54LX)
static nrf_radio_txpower_t dbm_to_nrf_radio_txpower(int8_t tx_power)
{
	switch (tx_power) {
#if defined(RADIO_TXPOWER_TXPOWER_Neg100dBm)
	case -100:
		return RADIO_TXPOWER_TXPOWER_Neg100dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg100dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Neg70dBm)
	case -70:
		return RADIO_TXPOWER_TXPOWER_Neg70dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg70dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Neg46dBm)
	case -46:
		return RADIO_TXPOWER_TXPOWER_Neg46dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg46dBm) */

	case -40:
		return RADIO_TXPOWER_TXPOWER_Neg40dBm;

#if defined(RADIO_TXPOWER_TXPOWER_Neg30dBm)
	case -30:
		return RADIO_TXPOWER_TXPOWER_Neg30dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg30dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Neg28dBm)
	case -28:
		return RADIO_TXPOWER_TXPOWER_Neg28dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg28dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Neg22dBm)
	case -22:
		return RADIO_TXPOWER_TXPOWER_Neg22dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg22dBm) */

	case -20:
		return RADIO_TXPOWER_TXPOWER_Neg20dBm;

#if defined(RADIO_TXPOWER_TXPOWER_Neg18dBm)
	case -18:
		return RADIO_TXPOWER_TXPOWER_Neg18dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg18dBm) */

	case -16:
		return RADIO_TXPOWER_TXPOWER_Neg16dBm;

#if defined(RADIO_TXPOWER_TXPOWER_Neg14dBm)
	case -14:
		return RADIO_TXPOWER_TXPOWER_Neg14dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg14dBm) */

	case -12:
		return RADIO_TXPOWER_TXPOWER_Neg12dBm;

#if defined(RADIO_TXPOWER_TXPOWER_Neg10dBm)
	case -10:
		return RADIO_TXPOWER_TXPOWER_Neg10dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg10dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Neg9dBm)
	case -9:
		return RADIO_TXPOWER_TXPOWER_Neg9dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg9dBm) */

	case -8:
		return RADIO_TXPOWER_TXPOWER_Neg8dBm;

#if defined(RADIO_TXPOWER_TXPOWER_Neg7dBm)
	case -7:
		return RADIO_TXPOWER_TXPOWER_Neg7dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg7dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Neg6dBm)
	case -6:
		return RADIO_TXPOWER_TXPOWER_Neg6dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg6dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Neg5dBm)
	case -5:
		return RADIO_TXPOWER_TXPOWER_Neg5dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg5dBm) */

	case -4:
		return RADIO_TXPOWER_TXPOWER_Neg4dBm;

#if defined(RADIO_TXPOWER_TXPOWER_Neg3dBm)
	case -3:
		return RADIO_TXPOWER_TXPOWER_Neg3dBm;
#endif /* defined (RADIO_TXPOWER_TXPOWER_Neg3dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Neg2dBm)
	case -2:
		return RADIO_TXPOWER_TXPOWER_Neg2dBm;
#endif /* defined (RADIO_TXPOWER_TXPOWER_Neg2dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Neg1dBm)
	case -1:
		return RADIO_TXPOWER_TXPOWER_Neg1dBm;
#endif /* defined (RADIO_TXPOWER_TXPOWER_Neg1dBm) */

	case 0:
		return RADIO_TXPOWER_TXPOWER_0dBm;

#if defined(RADIO_TXPOWER_TXPOWER_Pos1dBm)
	case 1:
		return RADIO_TXPOWER_TXPOWER_Pos1dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos1dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Pos2dBm)
	case 2:
		return RADIO_TXPOWER_TXPOWER_Pos2dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos2dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Pos3dBm)
	case 3:
		return RADIO_TXPOWER_TXPOWER_Pos3dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos3dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Pos4dBm)
	case 4:
		return RADIO_TXPOWER_TXPOWER_Pos4dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos4dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Pos5dBm)
	case 5:
		return RADIO_TXPOWER_TXPOWER_Pos5dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos5dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Pos6dBm)
	case 6:
		return RADIO_TXPOWER_TXPOWER_Pos6dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos6dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Pos7dBm)
	case 7:
		return RADIO_TXPOWER_TXPOWER_Pos7dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos7dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Pos8dBm)
	case 8:
		return RADIO_TXPOWER_TXPOWER_Pos8dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos8dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Pos9dBm)
	case 9:
		return RADIO_TXPOWER_TXPOWER_Pos9dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos9dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Pos10dBm)
	case 10:
		return RADIO_TXPOWER_TXPOWER_Pos10dBm;
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos10dBm) */

	default:
		printk("TX power to enumerator conversion failed, defaulting to 0 dBm\n");
		return RADIO_TXPOWER_TXPOWER_0dBm;
	}
}
#endif /* defined(CONFIG_SOC_SERIES_NRF54HX) || defined(CONFIG_SOC_SERIES_NRF54LX) */

#if !(defined(CONFIG_SOC_SERIES_NRF54HX) || defined(CONFIG_SOC_SERIES_NRF54LX))
static mpsl_phy_t convert_bitrate_to_mpsl_phy(enum esb_bitrate bitrate)
{
	switch (bitrate) {
	case ESB_BITRATE_1MBPS:
		return MPSL_PHY_NRF_1Mbit;
	case ESB_BITRATE_2MBPS:
		return MPSL_PHY_NRF_2Mbit;
#if defined(RADIO_MODE_MODE_Nrf_250Kbit)
	case ESB_BITRATE_250KBPS:
		return MPSL_PHY_NRF_250Kbit;
#endif
	case ESB_BITRATE_1MBPS_BLE:
		return MPSL_PHY_BLE_1M;
#if defined(RADIO_MODE_MODE_Ble_2Mbit)
	case ESB_BITRATE_2MBPS_BLE:
		return MPSL_PHY_BLE_2M;
#endif
#if defined(RADIO_MODE_MODE_Nrf_4Mbit_0BT6)
	case ESB_BITRATE_4MBPS:
		return MPSL_PHY_NRF_4Mbit_0BT6;
#endif
	default:
		return MPSL_PHY_NRF_1Mbit;
	}
}
#endif /* !(defined(CONFIG_SOC_SERIES_NRF54HX) || defined(CONFIG_SOC_SERIES_NRF54LX)) */

static void update_radio_tx_power(void)
{
#if !(defined(CONFIG_SOC_SERIES_NRF54HX) || defined(CONFIG_SOC_SERIES_NRF54LX))
	int32_t err;
	mpsl_tx_power_split_t tx_power;

	(void)mpsl_fem_tx_power_split(esb_cfg.tx_output_power, &tx_power,
				      convert_bitrate_to_mpsl_phy(esb_cfg.bitrate),
				      (RADIO_BASE_FREQUENCY + esb_addr.rf_channel), false);

	err = mpsl_fem_pa_power_control_set(tx_power.fem_pa_power_control);
	if (err) {
		/* Should not happen. */
		__ASSERT_NO_MSG(false);
	}

#ifdef NRF53_SERIES
	bool high_voltage = false;

	if (tx_power.radio_tx_power > 0) {
		high_voltage = true;
		tx_power.radio_tx_power -= NRF5340_HIGH_VOLTAGE_GAIN;
	}

	nrf_vreqctrl_radio_high_voltage_set(NRF_VREQCTRL_NS, high_voltage);
#endif /* NRF53_SERIES */

	nrf_radio_txpower_set(NRF_RADIO, tx_power.radio_tx_power);
#else
	nrf_radio_txpower_set(NRF_RADIO, dbm_to_nrf_radio_txpower(esb_cfg.tx_output_power));
#endif /* !(defined(CONFIG_SOC_SERIES_NRF54HX) || defined(CONFIG_SOC_SERIES_NRF54LX)) */
}

static bool update_radio_bitrate(void)
{
	nrf_radio_mode_set(NRF_RADIO, esb_cfg.bitrate);

	switch (esb_cfg.bitrate) {

#if defined(RADIO_MODE_MODE_Nrf_4Mbit_0BT6)
	case ESB_BITRATE_4MBPS:
		wait_for_ack_timeout_us = RX_ACK_TIMEOUT_US_4MBPS;
		break;
#endif /* defined(RADIO_MODE_MODE_Nrf_4Mbit_0BT6) */

	case ESB_BITRATE_2MBPS:

#if defined(RADIO_MODE_MODE_Ble_2Mbit)
	case ESB_BITRATE_2MBPS_BLE:
#endif /* defined(RADIO_MODE_MODE_Ble_2Mbit) */

		wait_for_ack_timeout_us = RX_ACK_TIMEOUT_US_2MBPS;
		break;

	case ESB_BITRATE_1MBPS:
		wait_for_ack_timeout_us = RX_ACK_TIMEOUT_US_1MBPS;
		break;

#if defined(RADIO_MODE_MODE_Nrf_250Kbit)
	case ESB_BITRATE_250KBPS:
		wait_for_ack_timeout_us = RX_ACK_TIMEOUT_US_250KBPS;
		break;
#endif /* defined(RADIO_MODE_MODE_Nrf_250Kbit) */

	case ESB_BITRATE_1MBPS_BLE:
		wait_for_ack_timeout_us = RX_ACK_TIMEOUT_US_1MBPS_BLE;
		break;

	default:
		/* Should not be reached */
		return false;
	}

	return true;
}

static bool update_radio_protocol(void)
{
	switch (esb_cfg.protocol) {
	case ESB_PROTOCOL_ESB_DPL:
		update_rf_payload_format = update_rf_payload_format_esb_dpl;
		break;

	case ESB_PROTOCOL_ESB:
		update_rf_payload_format = update_rf_payload_format_esb;
		break;

	default:
		/* Should not be reached */
		return false;
	}
	return true;
}

static bool update_radio_crc(void)
{
	switch (esb_cfg.crc) {
	case ESB_CRC_16BIT:
		nrf_radio_crcinit_set(NRF_RADIO, 0xFFFFUL); /* Initial value */
#if CONFIG_ZMK_ESB_WHITEEN
		nrf_radio_crc_configure(NRF_RADIO, ESB_CRC_16BIT, NRF_RADIO_CRC_ADDR_SKIP,
					0x11021UL); /* CRC poly: x^16+x^12^x^5+1 */
#else
		nrf_radio_crc_configure(NRF_RADIO, ESB_CRC_16BIT, NRF_RADIO_CRC_ADDR_INCLUDE,
					0x11021UL); /* CRC poly: x^16+x^12^x^5+1 */
#endif
		break;

	case ESB_CRC_8BIT:
		nrf_radio_crcinit_set(NRF_RADIO, 0xFFUL); /* Initial value */
#if CONFIG_ZMK_ESB_WHITEEN
		nrf_radio_crc_configure(NRF_RADIO, ESB_CRC_8BIT, NRF_RADIO_CRC_ADDR_SKIP,
					0x107UL); /* CRC poly: x^8+x^2^x^1+1 */
#else
		nrf_radio_crc_configure(NRF_RADIO, ESB_CRC_8BIT, NRF_RADIO_CRC_ADDR_INCLUDE,
					0x107UL); /* CRC poly: x^8+x^2^x^1+1 */
#endif
		break;

	case ESB_CRC_OFF:
		nrf_radio_crcinit_set(NRF_RADIO, 0x00UL);
#if CONFIG_ZMK_ESB_WHITEEN
		nrf_radio_crc_configure(NRF_RADIO, ESB_CRC_OFF, NRF_RADIO_CRC_ADDR_SKIP, 0x00UL);
#else
		nrf_radio_crc_configure(NRF_RADIO, ESB_CRC_OFF, NRF_RADIO_CRC_ADDR_INCLUDE, 0x00UL);
#endif

		break;

	default:
		return false;
	}

	return true;
}

static bool update_radio_parameters(void)
{
	bool params_valid = true;

	params_valid &= update_radio_bitrate();
	params_valid &= update_radio_protocol();
	params_valid &= update_radio_crc();
	update_rf_payload_format(esb_cfg.payload_length);
	params_valid &= (esb_cfg.retransmit_delay >= RETRANSMIT_DELAY_MIN);

	return params_valid;
}

/*  Function to push the content of the rx_buffer to the RX FIFO.
 *
 *  The module will point the register NRF_RADIO->PACKETPTR to a buffer for
 *  receiving packets. After receiving a packet the module will call this
 *  function to copy the received data to the RX FIFO.
 *
 *  @param  pipe Pipe number to set for the packet.
 *  @param  pid  Packet ID.
 *
 *  @retval true   Operation successful.
 *  @retval false  Operation failed.
 */
static bool push_rx_fifo(uint8_t pipe, uint8_t pid, uint8_t rx_len, uint8_t *rx_data)
{
	if (rx_fifo.count >= CONFIG_ZMK_ESB_RX_FIFO_SIZE) {
		return false;
	}

	if (rx_len > ESB_MAX_PAYLOAD_LENGTH) {
		return false;
	}

	rx_fifo.payload[rx_fifo.back]->length = rx_len;

	memcpy(rx_fifo.payload[rx_fifo.back]->data, rx_data, rx_len);

	rx_fifo.payload[rx_fifo.back]->pipe = pipe;
	rx_fifo.payload[rx_fifo.back]->rssi = rssi_get(pipe);
	rx_fifo.payload[rx_fifo.back]->pid = pid;
	// rx_fifo.payload[rx_fifo.back]->noack = !rx_pdu->type.dpl_pdu.no_ack;

	if (++rx_fifo.back >= CONFIG_ZMK_ESB_RX_FIFO_SIZE) {
		rx_fifo.back = 0;
	}
	rx_fifo.count++;

	return true;
}

/* See CONFIG_ZMK_ESB_SIM_ISR_LATENCY_US. Nothing on real hardware. */
static inline void sim_isr_latency(void)
{
#if defined(CONFIG_ZMK_ESB_SIM_ISR_LATENCY_US) && (CONFIG_ZMK_ESB_SIM_ISR_LATENCY_US > 0)
	k_busy_wait(CONFIG_ZMK_ESB_SIM_ISR_LATENCY_US);
#endif
}

static void esb_timer_handler(nrf_timer_event_t event_type, void *context)
{
	sim_isr_latency();

	if (nrf_timer_int_enable_check(esb_timer.p_reg, NRF_TIMER_INT_COMPARE1_MASK)) {
		nrf_timer_event_clear(esb_timer.p_reg, NRF_TIMER_EVENT_COMPARE1);
		if (on_timer_compare1 != NULL) {
			on_timer_compare1();
		}
	}

	if (IS_ENABLED(ERRATA_216_PRESENT) && event_type == NRF_TIMER_EVENT_COMPARE3) {
		nrf_timer_int_disable(esb_timer.p_reg, NRF_TIMER_INT_COMPARE3_MASK);

		if (atomic_get(&errata_216_status) == ERRATA_216_ENABLED) {
			/* This case is triggered after calling the radio_start() function */

			/* Restore timer shorts */
			nrf_timer_shorts_set(esb_timer.p_reg, errata_216_timer_shorts);

			nrf_egu_task_trigger(ESB_EGU, ESB_EGU_TASK);
		} else {
			/* This case is triggered during retransmission. */
			errata216_on();
		}
	}
}

static void esb_rtc_handler(nrf_rtc_event_t event_type, void *context)
{
	sim_isr_latency();

	if (nrfy_rtc_int_enable_check(esb_rtc.p_reg, NRF_RTC_INT_COMPARE1_MASK)) {
		nrfy_rtc_event_clear(esb_rtc.p_reg, NRF_RTC_EVENT_COMPARE_1);
#if ESB_CENTRAL
		central_watchdog();
#else
		if (on_timer_compare1 != NULL) {
			on_timer_compare1();
		}
#endif
	}
}

static int sys_timer_init(void)
{
	nrfx_err_t nrfx_err;
	const nrfx_timer_config_t config = {
		.frequency = NRFX_MHZ_TO_HZ(1),
		.mode = NRF_TIMER_MODE_TIMER,
		.bit_width = NRF_TIMER_BIT_WIDTH_32,
	};

	nrfx_err = nrfx_timer_init(&esb_timer, &config, esb_timer_handler);
	if (nrfx_err != NRFX_SUCCESS) {
		LOG_ERR("Failed to initialize nrfx timer (err %d)", nrfx_err);
		return -EFAULT;
	}

	return 0;
}

static void sys_timer_deinit(void)
{
	nrfx_timer_uninit(&esb_timer);
}

static int sys_rtc_init(void)
{
	nrfx_err_t nrfx_err;
	const nrfx_rtc_config_t config = {
		.prescaler = 0,
		.reliable = true,
		.tick_latency = RADIO_MARGIN_TICKS,
	};

	nrfx_err = nrfx_rtc_init(&esb_rtc, &config, esb_rtc_handler);
	if (nrfx_err != NRFX_SUCCESS) {
		LOG_ERR("Failed to initialize nrfx rtc (err %d)", nrfx_err);
		return -EFAULT;
	}

	return 0;
}

static void sys_rtc_deinit(void)
{
	nrfx_rtc_uninit(&esb_rtc);
}

static void rtc_irq_enable(void)
{
	nrfy_rtc_event_clear(esb_rtc.p_reg, NRF_RTC_EVENT_COMPARE_1);
	nrfy_rtc_int_enable(esb_rtc.p_reg, NRF_RTC_INT_COMPARE1_MASK);
	irq_enable(ESB_RTC_IRQ);
}

static void rtc_irq_disable(void)
{
	irq_disable(ESB_RTC_IRQ);
	NVIC_ClearPendingIRQ(ESB_RTC_IRQ);
	nrfy_rtc_int_disable(esb_rtc.p_reg, NRF_RTC_INT_COMPARE1_MASK);
	nrfy_rtc_event_clear(esb_rtc.p_reg, NRF_RTC_EVENT_COMPARE_1);
}

static void timer_irq_enable(void)
{
	nrfy_timer_event_clear(esb_timer.p_reg, NRF_TIMER_EVENT_COMPARE1);
	nrfy_timer_int_enable(esb_timer.p_reg, NRF_TIMER_INT_COMPARE1_MASK);
	NVIC_ClearPendingIRQ(ESB_TIMER_IRQ);
	irq_enable(ESB_TIMER_IRQ);
}

static void timer_irq_disable(void)
{
	irq_disable(ESB_TIMER_IRQ);
	nrfy_timer_int_disable(esb_timer.p_reg, NRF_TIMER_INT_COMPARE1_MASK);
}

static void radio_irq_enable(void)
{
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
	nrf_radio_int_enable(NRF_RADIO, NRF_RADIO_INT_DISABLED_MASK);
	irq_enable(ESB_RADIO_IRQ_NUMBER);
}

static void radio_irq_disable(void)
{
	nrf_radio_int_disable(NRF_RADIO, NRF_RADIO_INT_DISABLED_MASK);
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
	NVIC_ClearPendingIRQ(ESB_RADIO_IRQ_NUMBER);
	irq_disable(ESB_RADIO_IRQ_NUMBER);
}

static void crcok_irq_enable(void)
{
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_CRCOK);
	nrf_radio_int_enable(NRF_RADIO, NRF_RADIO_INT_CRCOK_MASK);
	NVIC_ClearPendingIRQ(ESB_RADIO_IRQ_NUMBER);
	irq_enable(ESB_RADIO_IRQ_NUMBER);
}

static void crcok_irq_disable(void)
{
	nrf_radio_int_disable(NRF_RADIO, NRF_RADIO_INT_CRCOK_MASK);
	irq_disable(ESB_RADIO_IRQ_NUMBER);
}

static void set_evt_interrupt(void)
{
	if (IS_ENABLED(ESB_EVT_USING_EGU)) {
		nrf_egu_task_trigger(ESB_EGU, ESB_EGU_EVT_TASK);
	} else {
		NVIC_SetPendingIRQ(ESB_EVT_IRQ_NUMBER);
	}
}

static void set_tx_evt_interrupt(uint8_t pipe, bool success)
{
	struct pipe_info *info = rx_pipe_info_get(pipe);
	last_tx_evt.tx_attempts = info->tx_try;

#if !CONFIG_ZMK_ESB_DISABLE_IRQ_TX
	interrupt_flags |= (success ? INT_TX_SUCCESS_MSK : INT_TX_FAILED_MSK);

	set_evt_interrupt();
#endif
}

static void set_sync_evt_interrupt(uint8_t pipe, bool up)
{
	ctx.timeout_count = 0;
	if (!up) {
		ctx.desync_count++;
	}

	struct esb_evt event = {.sync.pipe = pipe, .sync.up = up};

	if (k_msgq_put(&sync_event_msgq, &event, K_NO_WAIT) == 0) {
		k_work_submit(&sync_op_work);
	}
}

static void set_rx_evt_interrupt(void)
{
	interrupt_flags |= INT_RX_DATA_RECEIVED_MSK;

	set_evt_interrupt();
}

static void fast_switchinng_set_channel(uint8_t channel)
{
	*(volatile uint32_t *)((uint8_t *)(NRF_RADIO) + 0x70C) &= ~(1 << 31);
	nrf_radio_frequency_set(NRF_RADIO, (RADIO_BASE_FREQUENCY + channel));
	*(volatile uint32_t *)((uint8_t *)(NRF_RADIO) + 0x07C) = 1;
}

/* Retrieve interrupt flags and reset them.
 *
 * @param[out] interrupts	Interrupt flags.
 */
static void get_and_clear_irqs(uint32_t *interrupts)
{
	__ASSERT_NO_MSG(interrupts != NULL);

	unsigned int key = irq_lock();

	*interrupts = interrupt_flags;
	interrupt_flags = 0;

	irq_unlock(key);
}

static void radio_irq_handler(void)
{
	sim_isr_latency();

	if (nrf_radio_int_enable_check(NRF_RADIO, NRF_RADIO_INT_CRCOK_MASK) &&
	    nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_CRCOK)) {
		// nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_CRCOK);
		/* Call the correct on_radio_disable function, depending on the
		 * current protocol state.
		 */
		// uint32_t cc = nrfy_timer_cc_get(esb_timer.p_reg, NRF_TIMER_CC_CHANNEL3);
		// LOG_WRN("CRCOK - cc: %u", cc);
		if (on_radio_disabled) {
			on_radio_disabled();
		}

		// early return
		return;
	}

	if (nrf_radio_int_enable_check(NRF_RADIO, NRF_RADIO_INT_DISABLED_MASK) &&
	    nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_DISABLED)) {
		nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
		/* Call the correct on_radio_disable function, depending on the
		 * current protocol state.
		 */
		if (on_radio_disabled) {
			on_radio_disabled();
		}
	}

	if (nrf_radio_int_enable_check(NRF_RADIO, ESB_RADIO_INT_END_MASK) &&
	    nrf_radio_event_check(NRF_RADIO, ESB_RADIO_EVENT_END)) {
		/* The PHYEND event is called when fast switching is enabled
		 * instead of the DISABLE event.
		 * This event is handled in the analogous way to the disable event.
		 */
		if (on_radio_disabled) {
			on_radio_disabled();
		}
		nrf_radio_event_clear(NRF_RADIO, ESB_RADIO_EVENT_END);
	}

#if defined(CONFIG_ZMK_ESB_FAST_CHANNEL_SWITCHING)
	if (nrf_radio_int_enable_check(NRF_RADIO, NRF_RADIO_INT_RXREADY_MASK) &&
	    nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_RXREADY)) {
		nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_RXREADY);
		if (atomic_test_and_clear_bit(&esb_addr.rf_channel_flags, RF_CHANNEL_UPDATE_FLAG)) {
			fast_switchinng_set_channel(esb_addr.rf_channel);
		}
	}
#endif /* defined(CONFIG_ZMK_ESB_FAST_CHANNEL_SWITCHING) */
}

static void esb_evt_irq_handler(void)
{
	uint32_t interrupts;
	struct esb_evt event;

	get_and_clear_irqs(&interrupts);

	if (event_handler != NULL) {
		if (interrupts & INT_TX_SUCCESS_MSK) {
			event.evt_id = ESB_EVENT_TX_SUCCESS;
			event.tx = last_tx_evt;
			event_handler(&event);
		}
		if (interrupts & INT_TX_FAILED_MSK) {
			event.evt_id = ESB_EVENT_TX_FAILED;
			event_handler(&event);
		}
		if (interrupts & INT_RX_DATA_RECEIVED_MSK) {
			event.evt_id = ESB_EVENT_RX_RECEIVED;
			event_handler(&event);
		}
	}
}

#if IS_ENABLED(CONFIG_ZMK_ESB_DYNAMIC_INTERRUPTS)

static void radio_dynamic_irq_handler(const void *args)
{
	ARG_UNUSED(args);
	radio_irq_handler();
	ISR_DIRECT_PM();
}

static void evt_dynamic_irq_handler(const void *args)
{
	ARG_UNUSED(args);
	if (IS_ENABLED(ESB_EVT_USING_EGU)) {
		nrf_egu_event_clear(ESB_EGU, ESB_EGU_EVT_EVENT);
	}
	esb_evt_irq_handler();
	ISR_DIRECT_PM();
}

static void timer_dynamic_irq_handler(const void *args)
{
	ARG_UNUSED(args);
	ESB_TIMER_IRQ_HANDLER();
	ISR_DIRECT_PM();
}

#else /* !IS_ENABLED(CONFIG_ZMK_ESB_DYNAMIC_INTERRUPTS) */

ISR_DIRECT_DECLARE(esb_radio_direct_irq_handler)
{
	radio_irq_handler();

	ISR_DIRECT_PM();

	return 1;
}

ISR_DIRECT_DECLARE(esb_evt_direct_irq_handler)
{
	if (IS_ENABLED(ESB_EVT_USING_EGU)) {
		nrf_egu_event_clear(ESB_EGU, ESB_EGU_EVT_EVENT);
	}

	esb_evt_irq_handler();

	ISR_DIRECT_PM();

	return 1;
}

ISR_DIRECT_DECLARE(ESB_SYS_TIMER_IRQHandler)
{
	ISR_DIRECT_PM();

	return 1;
}

#endif /* IS_ENABLED(CONFIG_ZMK_ESB_DYNAMIC_INTERRUPTS) */

static void initialize_fifos(void)
{
	init_tx();
	initialize_rx_fifo();
}

static void esb_irq_disable(void)
{
	irq_disable(ESB_RADIO_IRQ_NUMBER);
	irq_disable(ESB_EVT_IRQ_NUMBER);
	irq_disable(ESB_TIMER_IRQ);
	irq_disable(ESB_RTC_IRQ);
}

int esb_init(const struct esb_config *config)
{
	int err;

	if (!config) {
		return -EINVAL;
	}

	if (esb_initialized) {
		esb_disable();
	}

	event_handler = config->event_handler;

	memcpy(&esb_cfg, config, sizeof(esb_cfg));

	if (fast_switching) {
		if (!esb_cfg.use_fast_ramp_up) {
			return -EINVAL;
		}
	}

	interrupt_flags = 0;

	memset(rx_pipe_info, 0, sizeof(rx_pipe_info));

	update_radio_parameters();

	/* Configure radio address registers according to ESB default values */
	nrf_radio_base0_set(NRF_RADIO, 0xE7E7E7E7);
	nrf_radio_base1_set(NRF_RADIO, 0x43434343);
	nrf_radio_prefix0_set(NRF_RADIO, 0x23C343E7);
	nrf_radio_prefix1_set(NRF_RADIO, 0x13E363A3);

	initialize_fifos();

	err = sys_timer_init();
	if (err) {
		LOG_ERR("Failed to initialize ESB system timer");
		return err;
	}

	err = sys_rtc_init();
	if (err) {
		LOG_ERR("Failed to initialize ESB system RTC");
		return err;
	}

	err = esb_ppi_init();
	if (err) {
		LOG_ERR("Failed to initialize PPI");
		return err;
	}

	// disable_event.event.generic.event = esb_ppi_radio_disabled_get();

	nrf_radio_fast_ramp_up_enable_set(NRF_RADIO, esb_cfg.use_fast_ramp_up);

#if defined(CONFIG_ZMK_ESB_FAST_CHANNEL_SWITCHING)
	nrf_radio_int_enable(NRF_RADIO, NRF_RADIO_INT_RXREADY_MASK);
#endif /* defined(CONFIG_ZMK_ESB_FAST_CHANNEL_SWITCHING) */

#if IS_ENABLED(CONFIG_ZMK_ESB_DYNAMIC_INTERRUPTS)

	/* Ensure IRQs are disabled before attaching. */
	esb_irq_disable();

	ARM_IRQ_DIRECT_DYNAMIC_CONNECT(ESB_RADIO_IRQ_NUMBER, CONFIG_ZMK_ESB_RADIO_IRQ_PRIORITY, 0,
				       reschedule);
	ARM_IRQ_DIRECT_DYNAMIC_CONNECT(ESB_EVT_IRQ_NUMBER, CONFIG_ZMK_ESB_EVENT_IRQ_PRIORITY, 0,
				       reschedule);
	ARM_IRQ_DIRECT_DYNAMIC_CONNECT(ESB_TIMER_IRQ, CONFIG_ZMK_ESB_EVENT_IRQ_PRIORITY, 0, reschedule);

	irq_connect_dynamic(ESB_RADIO_IRQ_NUMBER, CONFIG_ZMK_ESB_RADIO_IRQ_PRIORITY,
			    radio_dynamic_irq_handler, NULL, 0);
	irq_connect_dynamic(ESB_EVT_IRQ_NUMBER, CONFIG_ZMK_ESB_EVENT_IRQ_PRIORITY,
			    evt_dynamic_irq_handler, NULL, 0);
	irq_connect_dynamic(ESB_TIMER_IRQ, CONFIG_ZMK_ESB_EVENT_IRQ_PRIORITY, timer_dynamic_irq_handler,
			    NULL, 0);

#else /* !IS_ENABLED(CONFIG_ZMK_ESB_DYNAMIC_INTERRUPTS) */

	IRQ_DIRECT_CONNECT(ESB_RADIO_IRQ_NUMBER, CONFIG_ZMK_ESB_RADIO_IRQ_PRIORITY,
			   esb_radio_direct_irq_handler, 0);
	IRQ_DIRECT_CONNECT(ESB_EVT_IRQ_NUMBER, CONFIG_ZMK_ESB_EVENT_IRQ_PRIORITY,
			   esb_evt_direct_irq_handler, 0);
	IRQ_DIRECT_CONNECT(ESB_TIMER_IRQ, CONFIG_ZMK_ESB_RADIO_IRQ_PRIORITY, ESB_TIMER_IRQ_HANDLER, 0);
	IRQ_DIRECT_CONNECT(ESB_RTC_IRQ, CONFIG_ZMK_ESB_RADIO_IRQ_PRIORITY, ESB_RTC_IRQ_HANDLER, 0);

#endif /* IS_ENABLED(CONFIG_ZMK_ESB_DYNAMIC_INTERRUPTS) */

	irq_enable(ESB_RADIO_IRQ_NUMBER);
	irq_enable(ESB_EVT_IRQ_NUMBER);
	if (IS_ENABLED(ESB_EVT_USING_EGU)) {
		nrf_egu_int_enable(ESB_EGU, ESB_EGU_EVT_INT);
	}
	irq_enable(ESB_TIMER_IRQ);

	esb_state = ESB_STATE_IDLE;
	esb_initialized = true;

	if (nrf52_errata_182()) {
		/* Check if the device is an nRF52832 Rev. 2. */
		/* Workaround for nRF52832 rev 2 errata 182 */
		*(volatile uint32_t *)0x4000173C |= (1 << 10);
	}

#if defined(CONFIG_SOC_SERIES_NRF54HX)
	/* Apply HMPAN-102 workaround for nRF54H series */
	*(volatile uint32_t *)0x5302C7E4 =
		(((*((volatile uint32_t *)0x5302C7E4)) & 0xFF000FFF) | 0x0012C000);

	/* Apply HMPAN-18 workaround for nRF54H series - load trim values*/
	if (*(volatile uint32_t *)0x0FFFE458 != TRIM_VALUE_EMPTY) {
		*(volatile uint32_t *)0x5302C734 = *(volatile uint32_t *)0x0FFFE458;
	}

	if (*(volatile uint32_t *)0x0FFFE45C != TRIM_VALUE_EMPTY) {
		*(volatile uint32_t *)0x5302C738 = *(volatile uint32_t *)0x0FFFE45C;
	}

	if (*(volatile uint32_t *)0x0FFFE460 != TRIM_VALUE_EMPTY) {
		*(volatile uint32_t *)0x5302C73C = *(volatile uint32_t *)0x0FFFE460;
	}

	if (*(volatile uint32_t *)0x0FFFE464 != TRIM_VALUE_EMPTY) {
		*(volatile uint32_t *)0x5302C740 = *(volatile uint32_t *)0x0FFFE464;
	}

	if (*(volatile uint32_t *)0x0FFFE468 != TRIM_VALUE_EMPTY) {
		*(volatile uint32_t *)0x5302C74C = *(volatile uint32_t *)0x0FFFE468;
	}

	if (*(volatile uint32_t *)0x0FFFE46C != TRIM_VALUE_EMPTY) {
		*(volatile uint32_t *)0x5302C7D8 = *(volatile uint32_t *)0x0FFFE46C;
	}

	if (*(volatile uint32_t *)0x0FFFE470 != TRIM_VALUE_EMPTY) {
		*(volatile uint32_t *)0x5302C840 = *(volatile uint32_t *)0x0FFFE470;
	}

	if (*(volatile uint32_t *)0x0FFFE474 != TRIM_VALUE_EMPTY) {
		*(volatile uint32_t *)0x5302C844 = *(volatile uint32_t *)0x0FFFE474;
	}

	if (*(volatile uint32_t *)0x0FFFE478 != TRIM_VALUE_EMPTY) {
		*(volatile uint32_t *)0x5302C848 = *(volatile uint32_t *)0x0FFFE478;
	}

	if (*(volatile uint32_t *)0x0FFFE47C != TRIM_VALUE_EMPTY) {
		*(volatile uint32_t *)0x5302C84C = *(volatile uint32_t *)0x0FFFE47C;
	}

	/* Apply HMPAN-103 workaround for nRF54H series*/
	if ((*(volatile uint32_t *)0x5302C8A0 == 0x80000000) ||
	    (*(volatile uint32_t *)0x5302C8A0 == 0x0058120E)) {
		*(volatile uint32_t *)0x5302C8A0 = 0x0058090E;
	}

	*(volatile uint32_t *)0x5302C8A4 = 0x00F8AA5F;
	*(volatile uint32_t *)0x5302C7AC = 0x8672827A;
	*(volatile uint32_t *)0x5302C7B0 = 0x7E768672;
	*(volatile uint32_t *)0x5302C7B4 = 0x0406007E;
#endif /* (CONFIG_SOC_SERIES_NRF54HX) */

#if ESB_CENTRAL
	central_setup();
#else
	peripheral_setup();
#endif
#ifdef MONITORING_WORK
	k_work_reschedule(&monitoring_work, K_SECONDS(1));
#endif
	return 0;
}

int esb_suspend(void)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}

	/*  Clear PPI */
	esb_ppi_disable_all();

	esb_state = ESB_STATE_IDLE;
	errata216_off();

	return 0;
}

void esb_disable(void)
{
	on_radio_disabled = NULL;

	esb_irq_disable();

	nrf_radio_shorts_disable(NRF_RADIO, 0xFFFFFFFF);
	nrf_radio_int_disable(NRF_RADIO, 0xFFFFFFFF);

	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_DISABLE);

	while (!nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_DISABLED)) {
		/* wait for register to settle */
	}

	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);

	esb_ppi_disable_all();
	esb_fem_reset();

	sys_timer_deinit();
	sys_rtc_deinit();
	esb_ppi_deinit();

	/* Radio ramp-up time to default mode */
	nrf_radio_fast_ramp_up_enable_set(NRF_RADIO, false);

	esb_state = ESB_STATE_IDLE;
	errata216_off();
	esb_initialized = false;

	memset(rx_pipe_info, 0, sizeof(rx_pipe_info));
}

bool esb_is_idle(void)
{
	return (esb_state == ESB_STATE_IDLE);
}

bool esb_is_initialized(void)
{
	return esb_initialized;
}

int esb_write_payload(const struct esb_payload *payload)
{
	if (!esb_initialized) {
		return -EACCES;
	}

	if (payload == NULL) {
		return -EINVAL;
	}

	if ((payload->length == 0) || (payload->length > ESB_MAX_PAYLOAD_LENGTH) ||
	    ((esb_cfg.protocol == ESB_PROTOCOL_ESB) &&
	     (payload->length > esb_cfg.payload_length))) {
		return -EMSGSIZE;
	}

	if (!tx_ok(payload->pipe, payload->length)) {
		return -ENOMEM;
	}

	if (resolve_pipe(payload->pipe) >= ESB_PIPE_COUNT) {
		return -EINVAL;
	}

	unsigned int key = irq_lock();

	put_tx(payload);

	irq_unlock(key);

#if ESB_PERIPHERAL
	if (esb_state == ESB_STATE_PERIPHERAL_RX_READY) {
		peripheral_prepare_rx();
	}
#endif

	return 0;
}

int esb_tx_start(uint8_t pipe, uint32_t size)
{
	if (!esb_initialized) {
		return -EINVAL;
	}

#if ESB_CENTRAL
	if (!is_slot_synced(pipe)) {
		return -EINVAL;
	}
#endif

	if (!tx_ok(pipe, size)) {
		return -ENOMEM;
	}

	tx_start(pipe);

#if ESB_PERIPHERAL
	queue_wait_start();
#endif

	return 0;
}

int esb_tx_put(uint8_t *data, uint32_t size)
{
	tx_put(data, size);

	return 0;
}

int esb_tx_finish(void)
{
	tx_finish();

#if ESB_PERIPHERAL
	if (esb_state == ESB_STATE_PERIPHERAL_RX_READY) {
		peripheral_prepare_rx();
	}
#endif

	return 0;
}

int esb_read_rx_payload(struct esb_payload *payload)
{
	if (!esb_initialized) {
		return -EACCES;
	}
	if (payload == NULL) {
		return -EINVAL;
	}

	if (rx_fifo.count == 0) {
		return -ENODATA;
	}

	unsigned int key = irq_lock();

	payload->length = rx_fifo.payload[rx_fifo.front]->length;
	payload->pipe = rx_fifo.payload[rx_fifo.front]->pipe;
	payload->rssi = rx_fifo.payload[rx_fifo.front]->rssi;
	payload->pid = rx_fifo.payload[rx_fifo.front]->pid;
	// payload->noack = rx_fifo.payload[rx_fifo.front]->noack;
	memcpy(payload->data, rx_fifo.payload[rx_fifo.front]->data, payload->length);

	if (++rx_fifo.front >= CONFIG_ZMK_ESB_RX_FIFO_SIZE) {
		rx_fifo.front = 0;
	}

	rx_fifo.count--;

	irq_unlock(key);

	return 0;
}

int esb_tdma_start(void)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}
	LOG_WRN("tdma started");

	nrf_radio_shorts_set(NRF_RADIO, RADIO_SHORTS_COMMON);
	nrfy_rtc_event_enable(esb_rtc.p_reg, NRF_RTC_INT_COMPARE0_MASK);
#if ESB_CENTRAL
//
#else // ESB_PERIPHERAL
	radio_irq_enable();
#endif

#if ESB_CENTRAL
	hfclk_on();
	central_prepare_tx();
#else // ESB_PERIPHERAL
	peripheral_start_desync();
#endif

	nrfx_rtc_enable(&esb_rtc);

	return 0;
}

int esb_tdma_stop(bool force)
{
#if !CONFIG_ZMK_ESB_CENTRAL
	if (esb_state != ESB_STATE_PERIPHERAL_DESYNC && !force) {
		return -EINVAL;
	}
#endif

	on_radio_disabled = NULL;

	LOG_WRN("tdma stopped");

	nrfx_rtc_disable(&esb_rtc);
	nrfx_rtc_counter_clear(&esb_rtc);
	nrfx_timer_disable(&esb_timer);

#if ESB_CENTRAL
	irq_disable(ESB_TIMER_IRQ);
#else
	irq_disable(ESB_RADIO_IRQ_NUMBER);
#endif
	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_DISABLE);
	if (hfclk_is_on) {
		hfclk_off();
	}

	esb_state = ESB_STATE_IDLE;
	pto_context_reset();
	reset_tx_all();
	reset_rx_fifo();
#if ESB_PERIPHERAL
	self_stats.queue_pending = false;
#endif

	return 0;
}
static uint8_t rx_payload_buffer[CONFIG_ZMK_ESB_MAX_PAYLOAD_LENGTH + sizeof(struct esb_radio_pdu)];
static uint8_t tx_payload_buffer[CONFIG_ZMK_ESB_MAX_PAYLOAD_LENGTH + sizeof(struct esb_radio_pdu)];

#if ESB_CENTRAL
static uint8_t *buffer[2] = {
	rx_payload_buffer,
	tx_payload_buffer,
};
static uint8_t buf_idx = 0;
static void central_prepare_tx(void)
{
	// LOG_WRN("start_tx_transaction");
	uint32_t now = nrfx_rtc_counter_get(&esb_rtc);
	uint32_t slot_diff = NRF_RTC_WRAP(now - ctx.start) / ctx.slotsize_ticks + 1;
	uint32_t tx_start = NRF_RTC_WRAP(ctx.start + (slot_diff * ctx.slotsize_ticks));

	if ((int32_t)(tx_start - now) < RADIO_MARGIN_TICKS) {
		slot_diff++;
		tx_start = NRF_RTC_WRAP(tx_start + ctx.slotsize_ticks);
	}

	/* Not the very first call, which starts from an arbitrary counter value. */
	if (esb_state == ESB_STATE_CENTRAL_TX) {
		slots_skipped += slot_diff - 1;
	}

	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_CRCOK);

	uint32_t next_slot = ctx.refslot + slot_diff;
	uint32_t timeout = ctx.slotsize_us - TIMEOUT_MARGIN_US;

#if defined(CONFIG_ZMK_ESB_SIM_STALL_EVERY) && (CONFIG_ZMK_ESB_SIM_STALL_EVERY > 0)
	static uint32_t prepared;

	if (++prepared % CONFIG_ZMK_ESB_SIM_STALL_EVERY == 0) {
		k_busy_wait(2 * ctx.slotsize_us);
	}
#endif

	nrfy_rtc_cc_set(esb_rtc.p_reg, 0, tx_start);
	nrfy_timer_cc_set(esb_timer.p_reg, NRF_TIMER_CC_CHANNEL1, timeout);

	ctx.start = tx_start;
	ctx.refslot = next_slot;
	ctx.pipe = (ctx.pipe + slot_diff) % ctx.pipes;

	uint8_t pipe = ctx.pipe;
	struct pipe_info *pipe_info = rx_pipe_info_get(pipe);
	struct esb_radio_pdu *pdu = (struct esb_radio_pdu *)buffer[buf_idx];
	struct esb_packet_dn *tx_packet = (struct esb_packet_dn *)pdu->data;

	bool slot_synced = is_slot_synced(pipe);
	uint8_t packet_length = sizeof(struct esb_header_dn);
	uint8_t data_length = slot_synced ? copy_tx(pipe, tx_packet->data) : 0;

	/* Every packet that carries no payload carries the slot parameters, not
	 * only the ones for a pipe this side already considers lost.
	 *
	 * A peripheral gives up after 0.5 s without a packet, the central only
	 * after 3 s without an answer. In between the peripheral used to scan
	 * and find nothing but empty polls it could not sync to, so every lost
	 * sync cost at least 3 s - and the central then released that half's
	 * keys. Now any poll is enough to get back in.
	 *
	 * A synced peripheral answers a control packet, where it stayed silent
	 * on an empty poll. That also narrows a hole in the ARQ: it used to take
	 * the empty poll's sequence number without saying so, and then dropped
	 * the next real payload sent under that number as a duplicate. Now that
	 * only happens when its answer is lost on the way.
	 *
	 * Not at 2 kHz: the 10 extra bytes there push a full-length answer past
	 * the end of the central's RX window.
	 */
	bool is_ctrl = !slot_synced ||
		       (data_length == 0 && CONFIG_ZMK_ESB_POLLING_RATE <= 1000);

	if (is_ctrl) {
		packet_length += sizeof(struct esb_ctrl_packet);
		set_control_packet(tx_packet->data);
	} else {
		packet_length += data_length;
	}

	/* Only a payload may be popped from the ring when the answer comes in. */
	pipe_info->tx_try = data_length > 0 ? 1 : 0;

	pdu->pdu.ctrl = is_ctrl;
	pdu->pdu.length = packet_length;
	pdu->pdu.sn = pipe_info->sn;
	pdu->pdu.nesn = pipe_info->nesn;
	pdu->pdu.more = count_tx_net(pipe) > 0;

	esb_addr.rf_channel = get_channel(next_slot);
#if CONFIG_ZMK_ESB_WHITEEN
	nrf_radio_datawhiteiv_set(NRF_RADIO, esb_addr.rf_channel);
#endif
	tx_packet->header.rssi = rssi_get(pipe);

	on_timer_compare1 = central_timeslot_end;
	on_radio_disabled = central_timeslot_end;

	// update_rf_payload_format(packet_length);
	nrf_radio_txaddress_set(NRF_RADIO, pipe);
	nrf_radio_frequency_set(NRF_RADIO, (RADIO_BASE_FREQUENCY + esb_addr.rf_channel));

	update_radio_tx_power();

	nrf_radio_packetptr_set(NRF_RADIO, pdu);

	crcok_irq_enable();
	timer_irq_enable();

	ppi_for_txrx_set(true, true);

	/* Every slot is started by the compare armed above and ended by an
	 * interrupt of the timer that slot starts; preparing the next one from
	 * there is all that keeps the sequence going. If this function is ever
	 * held up until the compare value has passed, nothing starts the slot and
	 * nothing ends it: the radio stays silent until the 24-bit counter comes
	 * round again, 512 s later, and the whole keyboard is dead meanwhile. So a
	 * second compare, moved on with every slot, restarts the sequence if no
	 * slot was prepared for three slot lengths. It is set from the counter as
	 * it is now, not from the slot time, so it cannot be in the past itself.
	 */
	nrfy_rtc_cc_set(esb_rtc.p_reg, 1,
			NRF_RTC_WRAP(nrfx_rtc_counter_get(&esb_rtc) + 3 * ctx.slotsize_ticks));
	if (esb_state != ESB_STATE_CENTRAL_TX) {
		rtc_irq_enable();
	}

	esb_state = ESB_STATE_CENTRAL_TX;

#ifdef CENTRAL_LOG_TS_START
	LOG_WRN("PIPE %u SLOT %u CH %u RSSI %d LEN %u SN %d %d SYNC %d CTRL %d WT %u", pipe,
		ctx.refslot, esb_addr.rf_channel, (int)(tx_packet->header.rssi), packet_length,
		pipe_info->sn, pipe_info->nesn, slot_synced, pdu->pdu.ctrl,
		nrf_radio_datawhiteiv_get(NRF_RADIO));
#endif
}

static void central_watchdog(void)
{
	slots_stalled++;

	crcok_irq_disable();
	timer_irq_disable();
	ppi_for_txrx_clear();
	nrf_timer_task_trigger(esb_timer.p_reg, NRF_TIMER_TASK_SHUTDOWN);
	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_DISABLE);

	central_prepare_tx();
}

static void central_timeslot_end(void)
{
	uint8_t pipe = ctx.pipe;
	uint32_t start = ctx.start;
	struct esb_radio_pdu *rx_pdu = (struct esb_radio_pdu *)buffer[buf_idx];
	struct esb_packet *rx_packet = (struct esb_packet *)rx_pdu->data;
	struct pipe_info *pipe_info = rx_pipe_info_get(pipe);

	crcok_irq_disable();
	timer_irq_disable();

	buf_idx = !buf_idx;

	ppi_for_txrx_clear();
	/* Just clear LNA configuration and disable front-end module. */
	mpsl_fem_lna_configuration_clear();
	mpsl_fem_disable();

	// get current timeslot regiter first
	bool tx_failed = !nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_CRCOK);
	uint32_t crc = nrf_radio_rxcrc_get(NRF_RADIO);
	uint8_t rssi = nrf_radio_rssi_sample_get(NRF_RADIO);

	bool synced_before = is_slot_synced(pipe);
	if (tx_failed) {
		if (synced_before) {
			if (pipe_info->tx_try > 0) {
				pipe_info->tx_try++;
			}

			if (!slot_sync_check(pipe, start)) {
				rssi_reset(pipe);
				link_stats[resolve_pipe(pipe)].sync_lost++;
				link_stats[resolve_pipe(pipe)].peer_valid = false;
				set_sync_evt_interrupt(pipe, false);
			}
		}

		central_prepare_tx();
		return;
	}

	uint8_t rx_sn = rx_pdu->pdu.sn;
	uint8_t tx_nesn = pipe_info->nesn;
	uint8_t rx_nesn = rx_pdu->pdu.nesn;
	uint8_t tx_sn = pipe_info->sn;
	uint16_t tx_try = pipe_info->tx_try;
	bool rx_is_ctrl = rx_pdu->pdu.ctrl;

	if (rx_nesn != tx_sn) {
		pipe_info->sn = (!pipe_info->sn);
		if (tx_try > 0) {
			pop_tx(pipe);
			set_tx_evt_interrupt(pipe, true);
		}
	}

	slot_sync_update(pipe, start);

	if (!synced_before) {
		set_sync_evt_interrupt(pipe, true);
	}

	uint8_t rx_len = rx_pdu->pdu.length;
	bool retransmit_payload = (pipe_info->crc == crc) && (tx_nesn != rx_sn);
	bool send_rx_event = !retransmit_payload;

	if (rx_len >= 4) {
		rssi_update(pipe, rssi);
	}

	struct esb_pipe_stats *stats = &link_stats[resolve_pipe(pipe)];

	stats->answers++;
	if (retransmit_payload) {
		stats->duplicates++;
	}

	if (send_rx_event) {
		if (rx_is_ctrl) {
			pipe_info->crc = crc;
			pipe_info->nesn = (!pipe_info->nesn);

			/* The exact length keeps out the leftovers of a payload
			 * that a peripheral resends as a control packet.
			 */
			if (rx_len == sizeof(struct esb_peer_stats) &&
			    rx_pdu->data[0] == ESB_PROTOCOL_VERSION) {
				memcpy(&stats->peer, rx_pdu->data, sizeof(stats->peer));
				stats->peer_valid = true;
			}
		} else if (!stats->peer_valid) {
			/* Not from a peripheral known to speak this protocol
			 * version: neither taken nor acknowledged. One that does
			 * sends its self-report first after every scan, so this
			 * only ever meets firmware from another release.
			 */
			stats->refused++;
		} else if (push_rx_fifo(pipe, rx_sn, rx_len, rx_pdu->data)) {
			pipe_info->crc = crc;
			pipe_info->nesn = (!pipe_info->nesn);
			stats->payloads++;
			set_rx_evt_interrupt();
		}
	}

#ifdef CENTRAL_LOG_TS_END
	LOG_WRN("pipe %u rx_len %02u rssi %02d crc %04lx tx[%d %d] rx[%d %d] r %d s %d ctrl %u",
		pipe, rx_len, (int)(-rssi), crc, tx_sn, rx_nesn, tx_nesn, rx_sn, retransmit_payload,
		send_rx_event, rx_is_ctrl);
#endif

	// prepare for next slot
	central_prepare_tx();
}
#else // ESB_PERIPHERAL

static void hfclk_on_isr(void)
{
	// LOG_WRN("hfclk on isr");
	hfclk_on();
}

#if ESB_FAST_SWITCHING_PERIPHERAL
static void disable_radio(void)
{
	nrf_radio_int_disable(NRF_RADIO, NRF_RADIO_INT_DISABLED_MASK);
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_DISABLE);
	while (!nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_DISABLED)) {
	}
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
	nrf_radio_int_enable(NRF_RADIO, NRF_RADIO_INT_DISABLED_MASK);
}
#endif

static void peripheral_start_desync(void)
{
	uint32_t airtime = MIN(ctx.slotsize_us * ctx.pipes * 200, USEC_PER_SEC);
	airtime = (airtime != 0 ? airtime : DESYNC_AIRTIME_DEFAULT);

	/* The lead is what the 32 MHz crystal needs to start. */
	uint32_t lead = DESYNC_SCAN_LEAD_TICKS;
	bool fast_scan = ctx.scan_count < DESYNC_FAST_SCANS;
	if (fast_scan) {
		ctx.scan_count++;
	} else {
		lead = MAX(k_us_to_ticks_near32(USEC_PER_SEC - airtime), lead);
	}

	uint32_t start = NRF_RTC_WRAP(nrfx_rtc_counter_get(&esb_rtc) + lead);
	uint32_t timeout = airtime;
	nrfy_rtc_cc_set(esb_rtc.p_reg, 0, start);
	nrfy_timer_cc_set(esb_timer.p_reg, NRF_TIMER_CC_CHANNEL1, timeout);
	ctx.start = start;

	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_CRCOK);

	if ((esb_state != ESB_STATE_PERIPHERAL_DESYNC) && (esb_state != ESB_STATE_IDLE)) {
		set_sync_evt_interrupt(0, false);
	}

	if (fast_scan) {
		/* The scan starts too soon to arm an interrupt for the clock. */
		rtc_irq_disable();
		if (!hfclk_is_on) {
			hfclk_on();
		}
	} else {
		on_timer_compare1 = hfclk_on_isr;
		nrfy_rtc_cc_set(esb_rtc.p_reg, 1,
				NRF_RTC_WRAP(start - HFCLK_WARMUP_TICKS - ISR_MARGIN_TICKS));
		rtc_irq_enable();
	}

	esb_state = ESB_STATE_PERIPHERAL_DESYNC;
	on_radio_disabled = peripheral_disabled_desync;

	self_stats.scans++;
	scan_seed = scan_seed * 1664525U + 1013904223U;
	esb_addr.rf_channel = get_channel(scan_seed);
#if CONFIG_ZMK_ESB_WHITEEN
	nrf_radio_datawhiteiv_set(NRF_RADIO, esb_addr.rf_channel);
#endif
	nrf_radio_frequency_set(NRF_RADIO, (RADIO_BASE_FREQUENCY + esb_addr.rf_channel));

	nrf_radio_packetptr_set(NRF_RADIO, rx_payload_buffer);

	ppi_for_txrx_set(false, false);
#ifdef PERIPHERAL_LOG_DS_START
	LOG_WRN("start rx desync CHAN(%u), DESYNC(%u) PIPE(%u) NOW(%u) START(%u)",
		esb_addr.rf_channel, ctx.desync_count, ctx.pipe,
		nrfy_rtc_counter_get(esb_rtc.p_reg), nrfy_rtc_cc_get(esb_rtc.p_reg, 0));
#endif
}

static void peripheral_disabled_desync(void)
{
	struct esb_radio_pdu *pdu = (struct esb_radio_pdu *)rx_payload_buffer;
	struct esb_packet_dn *packet = (struct esb_packet_dn *)pdu->data;
	struct pipe_info *pipe_info = rx_pipe_info_get(0);
	ppi_for_txrx_clear();

	hfclk_off();
	bool ctrl = pdu->pdu.ctrl;
	bool crc_ok = nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_CRCOK);
	if (!crc_ok || !ctrl) {
		/* A good packet without slot parameters means the central is
		 * there and has a payload queued for us. Such a scan ends within
		 * milliseconds, so it does not use up one of the fast ones.
		 */
		if (crc_ok && ctx.scan_count > 0) {
			ctx.scan_count--;
		}

		peripheral_start_desync();
		return;
	}

	/* Slot parameters of a central with another protocol version are not for
	 * us: its packets or its timing differ in ways this side does not know.
	 * A central from before the version existed sends a shorter packet.
	 */
	const struct esb_ctrl_packet *control = (const struct esb_ctrl_packet *)packet->data;

	if (!IS_ENABLED(CONFIG_ZMK_ESB_SIM_SKIP_VERSION_CHECK) &&
	    (pdu->pdu.length < sizeof(struct esb_header_dn) + sizeof(struct esb_ctrl_packet) ||
	     control->version != ESB_PROTOCOL_VERSION)) {
		peripheral_start_desync();
		return;
	}

	drift_reset();

	/* sn: nothing of ours is waiting for an acknowledgement.
	 * nesn: the packet we synced on had no payload, so the central's current
	 * sequence number is still to come. It used to be marked as already
	 * received, and when the central sent its next payload under that same
	 * number the payload was dropped as a duplicate - and acknowledged.
	 */
	bool rx_sn = pdu->pdu.sn;
	bool rx_nesn = pdu->pdu.nesn;

	/* The payload that was in flight when the link dropped: the central's
	 * nesn says whether it arrived. If it did, pop it now. If not, it stays
	 * in the ring and is sent again. Either way the TX buffer holds no
	 * payload any more, so the first exchange pops nothing on its own.
	 * (It used to pop unconditionally, losing the payload whenever it was
	 * the uplink that had failed.)
	 */
	struct esb_radio_pdu *tx_pdu = (struct esb_radio_pdu *)tx_payload_buffer;
	if (!tx_pdu->pdu.ctrl && rx_nesn != pipe_info->sn) {
		pop_tx(0);
		queue_wait_end();
	}
	tx_pdu->pdu.ctrl = 1;

	pipe_info->sn = !rx_nesn;
	pipe_info->nesn = rx_sn;
	pipe_info->tx_try = 0;
	ctx.desync_count = 0;

	apply_control_packet(packet->data);

	int32_t sync = (int32_t)nrf_timer_cc_get(esb_timer.p_reg, NRF_TIMER_CC_CHANNEL2) -
		       (int32_t)ctx.addr_delay;
	int32_t sync_ticks = us_to_ticks_signed(sync);
	ctx.last_hb = NRF_RTC_WRAP(ctx.start + (uint32_t)sync_ticks);
	ctx.sync_rem_us = sync - ticks_to_us_signed(sync_ticks);

	/* Do not wait a whole heartbeat for the first exchange: answer the next
	 * poll (hb_failed makes that exchange a re-sync). The central then sees
	 * this peripheral at once, and last_hb is measured again over a few
	 * hundred us instead of the length of the scan.
	 */
	ctx.hb_failed = true;
	ctx.hello_pending = true;
	ctx.scan_margin = MIN(k_us_to_ticks_ceil32((uint32_t)MAX(sync, 0) /
						   (USEC_PER_SEC / DESYNC_SCAN_MARGIN_PPM)),
			      DESYNC_SCAN_MARGIN_MAX);

	set_sync_evt_interrupt(ctx.pipe, true);
#ifdef PERIPHERAL_LOG_DS_END
	LOG_WRN("disabled rx desync: REF(%u) SYNC(%u) HB(%u) SN[%d %d] CTRL %d", ctx.refslot, sync,
		ctx.last_hb, rx_sn, rx_nesn, ctrl);
#endif

	peripheral_prepare_rx();
}

static void peripheral_prepare_next_rx(void)
{
	uint32_t now = nrfx_rtc_counter_get(&esb_rtc);
	uint32_t loop_size = ctx.slotsize_ticks * ctx.pipes;
	uint32_t passed = NRF_RTC_WRAP(now - ctx.last_hb);
	uint32_t hfclk_margin = hfclk_is_on ? 0 : HFCLK_WARMUP_TICKS;
	uint32_t loops_diff = passed / loop_size + 1;

	esb_state = ESB_STATE_PERIPHERAL_RX;

	/* Pick the first of our slots that can still be reached. Everything is
	 * kept relative to last_hb: comparing absolute counter values goes wrong
	 * when the 24-bit RTC wraps between last_hb and now.
	 */
	int32_t margin = window_margin_ticks();
	int32_t drift_ticks = 0;
	int32_t rx_offset = 0; /* RX start, in ticks after last_hb */
	for (int i = 0;; ++i) {
		uint32_t diff = loops_diff * loop_size;

		drift_ticks = drift_get_ticks(diff);
		rx_offset = (int32_t)diff + drift_ticks - margin;

		/* The second condition only bounds the loop; one or two extra
		 * loops are always enough to clear the margins.
		 */
		if (rx_offset - (int32_t)passed >= (int32_t)(RADIO_MARGIN_TICKS + hfclk_margin) ||
		    i >= 8) {
			break;
		}

		loops_diff++;
	}

	ctx.start = NRF_RTC_WRAP(ctx.last_hb + (uint32_t)rx_offset);
	ctx.drift_now = drift_ticks;
	ctx.margin_now = margin;
	ctx.timeout = ctx.window_size_us + k_ticks_to_us_near32(margin);

	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_CRCOK);
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_ADDRESS);
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_TXREADY);

	// we can safely assume that start is at least 2 tick larger than now.
	nrfy_rtc_cc_set(esb_rtc.p_reg, 0, ctx.start);
	nrfy_timer_cc_set(esb_timer.p_reg, NRF_TIMER_CC_CHANNEL1, ctx.timeout);

	/* A heartbeat is any exchange a re-sync period or more after the last
	 * sync. (It used to be only the loop that is an exact multiple of the
	 * period, so waking up late for that one loop skipped the re-sync for a
	 * whole further period.)
	 */
	ctx.is_hb = loops_diff >= resync_loops();

	if (!hfclk_is_on) {
		nrf_clock_hfclk_t hf_src;

		/* For the link statistics: with the crystal kept running (see
		 * extra-modules/hfxo-always-on) it must be found running here.
		 */
		if (!nrf_clock_is_running(NRF_CLOCK, NRF_CLOCK_DOMAIN_HFCLK, &hf_src) ||
		    hf_src != NRF_CLOCK_HFCLK_HIGH_ACCURACY) {
			self_stats.hfxo_was_off = true;
		}

		hfclk_on();
	}

	ctx.slotdiff = loops_diff * ctx.pipes;
	uint32_t slot = ctx.refslot + ctx.slotdiff;
	esb_addr.rf_channel = get_channel(slot);
#if CONFIG_ZMK_ESB_WHITEEN
	nrf_radio_datawhiteiv_set(NRF_RADIO, esb_addr.rf_channel);
#endif
	nrf_radio_frequency_set(NRF_RADIO, (RADIO_BASE_FREQUENCY + esb_addr.rf_channel));

#if ESB_FAST_SWITCHING_PERIPHERAL
	// we have to reset txaddress here in case rx disabled irq hasn't been reached
	// in time.
	nrf_radio_txaddress_set(NRF_RADIO, ctx.pipe);
#endif
	nrf_radio_packetptr_set(NRF_RADIO, rx_payload_buffer);

	on_radio_disabled = peripheral_disabled_rx;
#if ESB_FAST_SWITCHING_PERIPHERAL
	bool tx_on = ((count_tx_net(0) > 0) || ctx.is_hb || ctx.hb_failed);
#else
	bool tx_on = false;
#endif
	ppi_for_txrx_set(false, tx_on);

	// set wdt in case something happens.
	nrfy_rtc_cc_set(esb_rtc.p_reg, 1,
			NRF_RTC_WRAP(ctx.start + ctx.slotsize_ticks - TIMEOUT_MARGIN_TICKS));
	on_timer_compare1 = peripheral_disabled_rx;
	rtc_irq_enable();

#ifdef PERIPHERAL_LOG_TS_START
	static int32_t diff_min = 0xffffff;
	int32_t diff_final = (int32_t)(ctx.start - now);
	if (diff_min > diff_final) {
		diff_min = diff_final;
	}

	LOG_WRN("[%s]\ttx_on %d now %u start %u slot %u ch %u drift %ld %ld hb %d tx %d to %u diff "
		"%u %d",
		__func__, tx_on, now, ctx.start, slot, esb_addr.rf_channel,
		drift_get_ticks(loops_diff * loop_size), drift_get_us(loops_diff * loop_size),
		ctx.is_hb, tx_power_get(ctx.channel_idx), ctx.timeout_count, diff_final, diff_min);
#endif
}

static void peripheral_prepare_sync(void)
{
	uint32_t now = nrfx_rtc_counter_get(&esb_rtc);
	uint32_t loop_size = ctx.slotsize_ticks * ctx.pipes;
	uint32_t hb_size = resync_loops() * loop_size; /* until the next re-sync */
	uint32_t passed = NRF_RTC_WRAP(now - ctx.last_hb);
	uint32_t hfclk_margin = (hfclk_is_on ? 0 : HFCLK_WARMUP_TICKS);

	/* Wake up early enough that peripheral_prepare_next_rx() can still pick
	 * the heartbeat loop. It needs RADIO_MARGIN_TICKS + hfclk_margin before
	 * the RX start, and the RX start itself moves with the drift estimate,
	 * so that has to be part of this too. In ticks after last_hb.
	 */
	int32_t isr_offset = (int32_t)hb_size + drift_get_ticks(hb_size) - window_margin_ticks() -
			     (int32_t)(hfclk_margin + RADIO_MARGIN_TICKS + ISR_MARGIN_TICKS);
	int32_t ahead = isr_offset - (int32_t)passed;

	/* Signed on purpose: once the wake-up point is behind us "ahead" is
	 * negative, and an unsigned compare would read that as far in the future,
	 * arm a compare the counter has already passed and leave this peripheral
	 * silent until its next key press.
	 */
	if (ahead < (int32_t)loop_size) {
		peripheral_prepare_next_rx();

		return;
	}

	on_timer_compare1 = peripheral_prepare_next_rx;
	nrfy_rtc_cc_set(esb_rtc.p_reg, 1, NRF_RTC_WRAP(ctx.last_hb + (uint32_t)isr_offset));
	rtc_irq_enable();

	esb_state = ESB_STATE_PERIPHERAL_RX_READY;
#ifdef PERIPHERAL_LOG_SYNC
	LOG_WRN("[%s]\tnow %u isr +%d ahead %d", __func__, now, isr_offset, ahead);
#endif
}

static void peripheral_prepare_rx(void)
{
	uint32_t key = irq_lock();
	rtc_irq_disable();

	// nrfy_rtc_event_clear(esb_rtc.p_reg, NRF_RTC_EVENT_COMPARE_0);

	struct pipe_info *pipe_info = rx_pipe_info_get(0);
	bool send_data = (count_tx(0) > 0) || (pipe_info->tx_try > 0) || ctx.timeout_count > 0 ||
			 ctx.more_data || ctx.hb_failed;

	if (send_data) {
		peripheral_prepare_next_rx();
	} else {
		peripheral_prepare_sync();
	}

	irq_unlock(key);
}

/* Re-anchor on the packet just received.
 *
 * sync is when the central's slot started, in us after our RX start. The RX
 * start was last_hb + elapsed + drift correction - window margin, so what the
 * clocks actually drifted apart over `elapsed` is the part of sync the window
 * margin does not explain, plus the correction that was already applied, minus
 * the fraction of a tick last_hb was rounded by.
 *
 * (The estimate used to be fed sync - margin alone, i.e. only the error left
 * over after its own correction, so it kept unlearning what it had learned.
 * It also ignored anything beyond 50 us, which is exactly the range where a
 * correction is needed.)
 */
static void set_sync(int64_t sync)
{
	int32_t sync_ticks = us_to_ticks_signed((int32_t)sync);
	uint32_t loop_size = ctx.slotsize_ticks * ctx.pipes;
	uint32_t elapsed = ctx.slotdiff * ctx.slotsize_ticks;
	uint32_t hb_size = ctx.hb_loops * loop_size;

	/* Shorter than the first re-sync interval is the exchange right after a
	 * scan: too short to tell drift from measurement error, so it only moves
	 * the anchor.
	 */
	if (elapsed >= RESYNC_FIRST_LOOPS * loop_size) {
		int64_t drift = sync - k_ticks_to_us_near32(ctx.margin_now) +
				ticks_to_us_signed(ctx.drift_now) - ctx.sync_rem_us;
		int64_t drift_per_hb = drift * hb_size / elapsed;
		int64_t limit = (int64_t)k_ticks_to_us_near32(hb_size) * DRIFT_LIMIT_PPM / 1000000;

		if (drift_per_hb >= -limit && drift_per_hb <= limit) {
			drift_update(drift_per_hb, MIN(elapsed, hb_size));

			/* The estimate now holds for a longer interval. */
			if (elapsed >= resync_loops() * loop_size) {
				ctx.resync_loops = MIN(resync_loops() * RESYNC_GROWTH, ctx.hb_loops);
			}
		}
	}

	ctx.last_hb = NRF_RTC_WRAP(ctx.start + (uint32_t)sync_ticks);
	ctx.sync_rem_us = (int32_t)sync - ticks_to_us_signed(sync_ticks);
	ctx.refslot += ctx.slotdiff;
	ctx.scan_margin = 0;
	/* Not when the scan succeeds: a link that syncs but then cannot complete
	 * an exchange would never fall back to the slow scans.
	 */
	ctx.scan_count = 0;
}

static void peripheral_disabled_rx(void)
{
	struct esb_radio_pdu *rx_pdu = (struct esb_radio_pdu *)rx_payload_buffer;
	struct esb_packet_dn *rx_packet = (struct esb_packet_dn *)rx_pdu->data;
	struct esb_radio_pdu *tx_pdu = (struct esb_radio_pdu *)tx_payload_buffer;
	struct esb_packet_up *tx_packet = (struct esb_packet_up *)tx_pdu->data;
	struct pipe_info *pipe_info = rx_pipe_info_get(0);

	esb_state = ESB_STATE_PERIPHERAL_TX_ACK;
	ppi_for_txrx_clear();
	rtc_irq_disable();

	// if crcok hasn't been generated, then timeout
	// or txready has been already generated, then radio has been already latched
	// so we skip to the next slot
	bool is_timeout = !nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_CRCOK);

#if ESB_FAST_SWITCHING_PERIPHERAL
	bool tx_fired = nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_TXREADY);
	bool rx_failed = is_timeout || tx_fired;
#else
	bool rx_failed = is_timeout;
#endif

	if (rx_failed) {
#if ESB_FAST_SWITCHING_PERIPHERAL
		if (tx_fired) {
			// LOG_WRN("tx fired");
			disable_radio();
		}
#endif

		if (ctx.is_hb) {
			ctx.hb_failed = true;
		}

		LOG_WRN("timeout");
		self_stats.rx_timeouts++;
		if (++ctx.timeout_count > ctx.timeout_count_max) {
			// something's wrong, disable hfclk and goto desync state
			hfclk_off();
			peripheral_start_desync();
		} else {
			peripheral_prepare_rx();
		}

		return;
	}

	ctx.timeout_count = 0;

	// update TX
	uint8_t rx_sn = rx_pdu->pdu.sn;
	uint8_t tx_nesn = pipe_info->nesn;
	uint8_t rx_nesn = rx_pdu->pdu.nesn;
	uint8_t tx_sn = pipe_info->sn;
	uint8_t tx_try = pipe_info->tx_try;
	bool rx_is_ctrl = rx_pdu->pdu.ctrl;

	bool tx_failed = (tx_sn == rx_nesn);
	bool retransmit_payload = (tx_nesn != rx_sn);

	int rx_len = rx_pdu->pdu.length - sizeof(struct esb_header_dn);
	// bool send_rx_event = !retransmit_payload && rx_len > sizeof(struct esb_header_dn);
	bool send_rx_event = false;

	// we toggle nesn here before pushing rxbuf for ack to central
	if (!retransmit_payload) {
		if (rx_len > 0) {
			if (rx_fifo.count < CONFIG_ZMK_ESB_RX_FIFO_SIZE) {
				send_rx_event = true;
				pipe_info->nesn = (!pipe_info->nesn);
			}
		} else {
			pipe_info->nesn = (!pipe_info->nesn);
		}
	}

	uint8_t tx_len = tx_pdu->pdu.length;
	if (tx_failed) {
		self_stats.tx_retries++;
		if (++pipe_info->tx_try > ctx.timeout_count_max) {
			set_tx_evt_interrupt(0, false);
			peripheral_start_desync();
			return;
		}

		// tx power raise if tx failed 3 times in a row.
		if ((pipe_info->tx_try & 3) == 3) {
			tx_power_raise(0);
		}

		if (ctx.is_hb) {
			ctx.hb_failed = true;
		}
	} else {
		// if packet before was payload, then tx was successful.
		if (++pipe_info->tx_try && !tx_pdu->pdu.ctrl) {
			set_tx_evt_interrupt(0, true);
			pop_tx(0);
			queue_wait_end();
		}

		/* The first packet after a scan is the self-report, whatever is
		 * queued: the central takes payloads only once it has seen the
		 * protocol version in one.
		 */
		tx_len = ctx.hello_pending ? 0 : copy_tx(0, tx_pdu->data);
		ctx.hello_pending = false;
		pipe_info->tx_try = 0;

		/* A packet without a payload carries this side's link statistics.
		 * It needs a body of a few bytes anyway, for the central's RSSI
		 * measurement to settle.
		 */
		tx_pdu->pdu.length = tx_len > 0 ? tx_len : peer_stats_fill(tx_pdu->data);
		tx_pdu->pdu.ctrl = tx_len == 0;
#if !CONFIG_ZMK_ESB_HFCLK_OFF_EVERY_TX
		if (tx_len > 0) {
			ctx.last_tx = ctx.start;
		}
#endif
	}

	uint8_t rssi = rx_packet->header.rssi;
	tx_power_update(ctx.pipe, rssi);
	esb_cfg.tx_output_power = tx_power_get(ctx.pipe);

	bool tx_triggered = tx_len > 0 || ctx.is_hb || tx_failed || ctx.hb_failed || rx_len > 0;
	if (tx_triggered) {
		update_radio_tx_power();

		if (!tx_failed) {
			pipe_info->sn = !pipe_info->sn;
		}

		tx_pdu->pdu.sn = pipe_info->sn;
		tx_pdu->pdu.nesn = pipe_info->nesn;
		// update_rf_payload_format(tx_pdu->pdu.length);
		nrf_radio_packetptr_set(NRF_RADIO, tx_pdu);

		esb_fem_for_tx_ack();
		on_radio_disabled = peripheral_disabled_tx_ack;

// we set txaddress manually here.
#if ESB_FAST_SWITCHING_PERIPHERAL
		nrf_radio_txaddress_set(NRF_RADIO, CENTRAL_PIPE);
#endif
		nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_TXEN);
	}
#if ESB_FAST_SWITCHING_PERIPHERAL
	else {
		disable_radio();
	}
#endif
	// set sync
	int64_t sync = (int64_t)(nrf_timer_cc_get(esb_timer.p_reg, NRF_TIMER_CC_CHANNEL2)) -
		       (int64_t)(ctx.addr_delay);

	if (ctx.is_hb || (ctx.hb_failed && !tx_failed)) {
		set_sync(sync);
		ctx.hb_failed = false;
	}

	ctx.more_data = rx_pdu->pdu.more;
	// push RX
	if (send_rx_event) {
		if (rx_is_ctrl == true) {
			//
		} else if (push_rx_fifo(ctx.pipe, rx_sn, rx_len, rx_packet->data)) {
			set_rx_evt_interrupt();
		}
	}

#ifdef PERIPHERAL_LOG_TS_END
	int64_t drift = sync - WINDOW_MARGIN_US;
	LOG_WRN("len [%u %d] tx[%d %d] rx[%d %d] t %d r %d s %d sy %lld dr %lld rssi %d tp %d TR "
		"%d ctrl [%u %u]",
		tx_len, rx_len, tx_sn, rx_nesn, tx_nesn, rx_sn, pipe_info->tx_try,
		retransmit_payload, send_rx_event, sync, drift, (int)(-rssi),
		esb_cfg.tx_output_power, tx_triggered, tx_pdu->pdu.ctrl, rx_is_ctrl);
#endif

	if (!tx_triggered) {
		peripheral_disabled_tx_ack();
	}
}

static void peripheral_disabled_tx_ack(void)
{
	// LOG_WRN("disabled rx ack");

#if CONFIG_ZMK_ESB_HFCLK_OFF_EVERY_TX
	struct pipe_info *info = rx_pipe_info_get(0);

	bool hfclk_turn_off = info->tx_try == 0;
#else
	bool hfclk_turn_off = NRF_RTC_WRAP(ctx.start - ctx.last_tx) > k_ms_to_ticks_near32(500);
#endif

	if (hfclk_turn_off) {
		hfclk_off();
	}

	peripheral_prepare_rx();
}
#endif // !CONFIG_ZMK_ESB_CENTRAL

int esb_set_address_length(uint8_t length)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}
	if (!((length > 2) && (length < 6))) {
		return -EINVAL;
	}

	esb_addr.addr_length = length;

	update_rf_payload_format(esb_cfg.payload_length);

	return 0;
}

int esb_set_base_address_0(const uint8_t *addr)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}
	if (addr == NULL) {
		return -EINVAL;
	}

	memcpy(esb_addr.base_addr_p0, addr, sizeof(esb_addr.base_addr_p0));

	update_radio_addresses(ADDR_UPDATE_MASK_BASE0);

	return 0;
}

int esb_set_base_address_1(const uint8_t *addr)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}
	if (addr == NULL) {
		return -EINVAL;
	}

	memcpy(esb_addr.base_addr_p1, addr, sizeof(esb_addr.base_addr_p1));

	update_radio_addresses(ADDR_UPDATE_MASK_BASE1);

	return 0;
}

int esb_set_prefixes(const uint8_t *prefixes, uint8_t num_pipes)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}
	if (prefixes == NULL) {
		return -EINVAL;
	}

	if (!(num_pipes <= CONFIG_ZMK_ESB_PIPE_COUNT)) {
		return -EINVAL;
	}

	memcpy(esb_addr.pipe_prefixes, prefixes, num_pipes);

	esb_addr.num_pipes = num_pipes;
	esb_addr.rx_pipes_enabled = BIT_MASK_UINT_8(num_pipes);

	update_radio_addresses(ADDR_UPDATE_MASK_PREFIX);

	return 0;
}

int esb_update_prefix(uint8_t pipe, uint8_t prefix)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}

	if (pipe >= CONFIG_ZMK_ESB_PIPE_COUNT) {
		return -EINVAL;
	}

	esb_addr.pipe_prefixes[pipe] = prefix;

	update_radio_addresses(ADDR_UPDATE_MASK_PREFIX);

	return 0;
}

int esb_enable_pipes(uint8_t enable_mask)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}

	if ((enable_mask | BIT_MASK_UINT_8(CONFIG_ZMK_ESB_PIPE_COUNT)) !=
	    BIT_MASK_UINT_8(CONFIG_ZMK_ESB_PIPE_COUNT)) {
		return -EINVAL;
	}

	esb_addr.rx_pipes_enabled = enable_mask;

	return 0;
}

int esb_set_rf_channel(uint32_t channel)
{
	if (channel > 100) {
		return -EINVAL;
	}

	if (esb_state != ESB_STATE_IDLE) {
		// if (IS_ENABLED(CONFIG_ZMK_ESB_FAST_CHANNEL_SWITCHING)) {
		// 	if (esb_state == ESB_STATE_PRX) {
		// 		fast_switchinng_set_channel(channel);
		// 	} else {
		// 		atomic_set_bit(&esb_addr.rf_channel_flags,
		// RF_CHANNEL_UPDATE_FLAG);
		// 	}
		// } else {
		// 	return -EBUSY;
		// }
		return -EBUSY;
	}

	esb_addr.rf_channel = channel;

	return 0;
}

int esb_get_rf_channel(uint32_t *channel)
{
	if (channel == NULL) {
		return -EINVAL;
	}

	*channel = esb_addr.rf_channel;

	return 0;
}

int esb_set_tx_power(int8_t tx_output_power)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}

	esb_cfg.tx_output_power = tx_output_power;

	return 0;
}

int esb_set_retransmit_delay(uint16_t delay)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}
	if (delay < RETRANSMIT_DELAY_MIN) {
		return -EINVAL;
	}

	esb_cfg.retransmit_delay = delay;

	return 0;
}

int esb_set_retransmit_count(uint16_t count)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}

	esb_cfg.retransmit_count = count;

	return 0;
}

int esb_set_bitrate(enum esb_bitrate bitrate)
{
	if (esb_state != ESB_STATE_IDLE) {
		return -EBUSY;
	}

	esb_cfg.bitrate = bitrate;

	return update_radio_bitrate() ? 0 : -EINVAL;
}

int esb_conn_cb_register(struct esb_conn_cb *cb)
{
	sys_slist_append(&esb_conn_cb_list, &cb->node);
	return 0;
}
