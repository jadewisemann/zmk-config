"""Teach the nRF52 RADIO/PPI models what the ESB TDMA link needs:
the Nordic proprietary 1/2 Mbit modes (modelled like the BLE ones, with the
configured packet format), any single logical RX address, and TIMER SHUTDOWN
as a PPI task. Run from the nrf_hw_models directory; idempotent."""
import re, sys
from pathlib import Path
H = Path("src/HW_models")
MARK = "/* esb-tdma test patch */"

def patch(name, fn):
    p = H / name; s = p.read_text()
    if MARK in s:
        return
    s = fn(s)
    p.write_text(MARK + "\n" + s)

MACROS = """
#define NHW_ESB_IS_NRF(m) ((m) == RADIO_MODE_MODE_Nrf_1Mbit || (m) == RADIO_MODE_MODE_Nrf_2Mbit)
#define NHW_ESB_IS_1M(m) ((m) == RADIO_MODE_MODE_Ble_1Mbit || (m) == RADIO_MODE_MODE_Nrf_1Mbit)
#define NHW_ESB_IS_2M(m) ((m) == RADIO_MODE_MODE_Ble_2Mbit || (m) == RADIO_MODE_MODE_Nrf_2Mbit)
/* preamble and address length in bytes, for the proprietary modes */
#define NHW_ESB_PREAMBLE() (((NRF_RADIO_regs.PCNF0 & RADIO_PCNF0_PLEN_Msk) >> RADIO_PCNF0_PLEN_Pos) == RADIO_PCNF0_PLEN_16bit ? 2 : 1)
#define NHW_ESB_ADDRLEN()  (((NRF_RADIO_regs.PCNF1 & RADIO_PCNF1_BALEN_Msk) >> RADIO_PCNF1_BALEN_Pos) + 1)
"""

def modes(s):
    s = re.sub(r"(\(?)(NRF_RADIO_regs\.MODE|MODE) == RADIO_MODE_MODE_Ble_1Mbit\b", r"\1NHW_ESB_IS_1M(\2)", s)
    s = re.sub(r"(\(?)(NRF_RADIO_regs\.MODE|MODE) == RADIO_MODE_MODE_Ble_2Mbit\b", r"\1NHW_ESB_IS_2M(\2)", s)
    return s

def add_macros(s):
    i = s.index("extern NRF_RADIO_Type NRF_RADIO_regs;") if "extern NRF_RADIO_Type NRF_RADIO_regs;" in s else None
    if i is None:
        # after the last #include
        i = [m.end() for m in re.finditer(r"^#include .*$", s, re.M)][-1]
        return s[:i] + "\n" + MACROS + s[i:]
    j = s.index("\n", i) + 1
    return s[:j] + MACROS + s[j:]

def utils(s):
    s = add_macros(modes(s))
    # no BLE packet-format checks in the proprietary modes
    s = s.replace("void nhwra_check_packet_conf(void){\n",
                  "void nhwra_check_packet_conf(void){\n  if (NHW_ESB_IS_NRF(NRF_RADIO_regs.MODE)) { return; }\n", 1)
    # the one enabled logical RX address instead of always address 0
    n = s.count("rx_addresses[0] = nhwra_get_address(0);")
    assert n == 2, n
    s = s.replace("rx_addresses[0] = nhwra_get_address(0);",
                  "rx_addresses[0] = nhwra_get_address(nhwra_esb_rx_logical_addr());")
    s = s.replace("static p2G4_modulation_t nhra_modulation_from_mode(",
                  """uint nhwra_esb_rx_logical_addr(void) {
  uint32_t en = NRF_RADIO_regs.RXADDRESSES & 0xFF;
  if (en == 0) { return 0; }
  if (en & (en - 1)) {
    bs_trace_error_time_line("esb-tdma patch: only one enabled RX address is modelled (RXADDRESSES=0x%x)\\n", en);
  }
  return __builtin_ctz(en);
}

static p2G4_modulation_t nhra_modulation_from_mode(""", 1)
    # preamble/address length of the proprietary modes in the Rx request
    old = "  rx_req->coding_rate = 0;\n\n  p2G4_freq_t center_freq;"
    assert s.count(old) == 1
    s = s.replace(old, """  if (NHW_ESB_IS_NRF(NRF_RADIO_regs.MODE)) {
    preamble_length = NHW_ESB_PREAMBLE();
    address_length  = NHW_ESB_ADDRLEN();
  }

  rx_req->coding_rate = 0;

  p2G4_freq_t center_freq;""")
    return s

def radio(s):
    s = add_macros(modes(s))
    old = "  payload_len = nhwra_tx_copy_payload(tx_buf);"
    assert s.count(old) == 1
    s = s.replace(old, """  if (NHW_ESB_IS_NRF(NRF_RADIO_regs.MODE)) {
    preamble_len = NHW_ESB_PREAMBLE();
    address_len  = NHW_ESB_ADDRLEN();
  }

""" + old)
    old = """    radio_state = RAD_RXDISABLE;
    NRF_RADIO_regs.STATE = RAD_RXDISABLE;
    TIFS_state = TIFS_DISABLE;
    nhwra_set_Timer_RADIO(nsi_hws_get_time() + nhwra_timings_get_RX_rampdown_time());"""
    assert s.count(old) == 1
    s = s.replace(old, """    radio_state = RAD_RXDISABLE;
    NRF_RADIO_regs.STATE = RAD_RXDISABLE;
    TIFS_state = TIFS_DISABLE;
    if (nhwra_timings_get_RX_rampdown_time() == 0) {
      /* A ramp-down of 0 us is over before any interrupt handler can run.
       * Scheduling it as a timer for "now" lets the handler of an interrupt
       * raised at this same instant run first, which a real radio never does. */
      radio_state = RAD_DISABLED;
      NRF_RADIO_regs.STATE = RAD_DISABLED;
      nhwra_set_Timer_RADIO(TIME_NEVER);
      nhw_radio_stop_bit_counter();
      nhw_RADIO_signal_EVENTS_DISABLED(0);
    } else {
      nhwra_set_Timer_RADIO(nsi_hws_get_time() + nhwra_timings_get_RX_rampdown_time());
    }""")
    # The radio layer shorts ADDRESS to RSSISTART for both directions. In a
    # transmission that measures nothing, and warning about it once per slot
    # makes the logs hundreds of megabytes.
    old = """    bs_trace_warning_time_line("TASK_RSSISTART received while the radio was not in Rx. "
                               "The spec does not allow this. We just use the last Rx value in the model\\n");"""
    assert s.count(old) == 1
    s = s.replace(old, "    /* no warning */")
    old = "    NRF_RADIO_regs.RXMATCH = 0; //The only we support so far"
    assert s.count(old) == 1
    s = s.replace(old, "    { extern uint nhwra_esb_rx_logical_addr(void); NRF_RADIO_regs.RXMATCH = nhwra_esb_rx_logical_addr(); }")
    return s

def timings(s):
    return add_macros(modes(s))

def ppi(s):
    old = "    { (void*)&NRF_TIMER_regs[2].TASKS_STOP,        nhw_timer2_TASK_STOP},"
    assert s.count(old) == 1
    s = s.replace(old, old + "\n    { (void*)&NRF_TIMER_regs[2].TASKS_SHUTDOWN,    nhw_esb_timer2_TASK_SHUTDOWN},")
    i = [m.end() for m in re.finditer(r"^#include .*$", s, re.M)][-1]
    return s[:i] + "\nextern void nhw_timer_TASK_SHUTDOWN(int t);\nstatic void nhw_esb_timer2_TASK_SHUTDOWN(void) { nhw_timer_TASK_SHUTDOWN(2); }\n" + s[i:]

patch("NHW_RADIO_utils.c", utils)
patch("NHW_RADIO.c", radio)
patch("NHW_RADIO_timings.c", timings)
patch("NRF_PPI.c", ppi)
print("patched")
