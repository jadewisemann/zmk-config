# ESB link test

Runs the ESB firmware of the dongle and both halves in a simulator, with the
real KLOTZ keymap, and checks what the dongle types. No hardware involved.

The simulator is [BabbleSim](https://babblesim.github.io) with Zephyr's
`nrf52_bsim` board: models of the nRF52's RADIO, RTC, TIMER, PPI, EGU and
clocks run the unmodified firmware, a simulated 2.4 GHz channel connects the
three devices, and each device can be given its own clock error. ZMK's own
Bluetooth split tests work the same way.

## What a run does

Most scenarios use one gesture: the left half holds its thumb key (Nav on
hold, Space on tap) and the right half presses J while it is held, which is
Left on the Nav layer. A lost or late thumb event types `J` instead, and that
is exactly what a keyboard with a struggling link does. Every scenario states
what the dongle has to type, and the halves that are meant to work may not
lose sync after their start-up scan.

| scenario | condition |
|---|---|
| `ideal` | perfect clocks, no packet loss |
| `slow80` | left half's clock 80 ppm slow |
| `slow1000` | left half's clock 1000 ppm slow |
| `fast1000` | right half's clock 1000 ppm fast |
| `dongle1000` | dongle's clock 1000 ppm fast, so both halves are slow |
| `loss30` | 30 % of all packets lost, clocks 300 and 200 ppm off |
| `long` | 12 minutes across the wrap of the 24-bit RTC, 5 % loss |
| `rolls` | both hands alternate, 35 ms from key to key with the keys overlapping, 5 % loss: the letters must come out in order |
| `paced` | the same rolling with `CONFIG_ZMK_SPLIT_ESB_PACED_SOURCE=1`: the order must survive the right half's keys being held back |
| `late` | the left half is switched on 10 s after the dongle |
| `stall` | every 3 s the dongle is held up while preparing a slot, so nothing starts that slot: its watchdog has to restart the slot sequence |
| `other-half` | the left half has another protocol version: it must not sync, and only `J` is typed |
| `old-half` | the left half syncs to any dongle, like firmware from before the version check: the dongle must not take its keys |

`slow80` is what the firmware up to `1365f2b` failed, and `slow1000` what it
failed up to `27a86b8`: both type `J` there (the older one also a late
`Space`), with the left half scanning for the dongle over and over. Without
the watchdog, `stall` types nothing at all after the first three seconds.

## Running it

Needs `west`, `cmake`, `ninja`, `dtc` and a 32-bit capable `gcc`
(`gcc-multilib`). The simulator lives in a west workspace of its own, about
3.5 GB, anywhere outside this repository:

```sh
python3 tests/esb/run.py setup ~/esb-sim          # once, and after a west.yml change
python3 tests/esb/run.py test  ~/esb-sim          # ~5 minutes
python3 tests/esb/run.py test  ~/esb-sim --only slow1000 --only stall
python3 tests/esb/run.py list
```

CI runs all of it on every push that touches the ESB firmware, the keymap or
this test (`.github/workflows/esb-test.yml`).

The logs of every device end up in `<workspace>/esb-test/<scenario>/`. The
dongle's log has the link statistics once per second and every key event as it
arrives (see the module README for the fields).

## What it does not show

A pass means the protocol logic, its arithmetic and its use of the peripherals
are right. It says nothing about:

- the radio itself: range, antennas, interference with a pattern (the packet
  loss here is random), the real sensitivity at 2 Mbit;
- the 32 MHz crystal. The model's radio works whether that clock runs or not;
- separate clocks. A device's 32 kHz and 32 MHz clocks drift together here.
  On a board whose 32 kHz clock is the RC oscillator they do not, which
  matters for how a scan converts its time into slot times;
- code execution time, beyond the fixed interrupt latency of
  `CONFIG_ZMK_ESB_SIM_ISR_LATENCY_US`;
- a half that is switched off, or goes to deep sleep, in the middle of a run,
  and clocks whose error changes while it runs: a simulated device runs from
  start to end with one fixed clock error.

For those the link reports on itself on real hardware: the `dongle-log`
firmware.

## Changes to the models

`setup` patches the hardware models with `patch_model.py`, because they only
know Bluetooth and 802.15.4:

- the Nordic proprietary 1 and 2 Mbit modes, treated like the Bluetooth ones
  but with the configured preamble, address length and packet format;
- one logical RX address other than address 0;
- TIMER's SHUTDOWN task as a PPI task;
- the end of a reception takes effect at once. The model scheduled it for
  "now", which let the handler of an interrupt raised in the same microsecond
  run first and re-arm the radio; a real radio is done long before any handler
  runs;
- no warning for an RSSI measurement started during a transmission, which the
  radio layer's shortcut does once per slot.

The firmware itself is not changed for the test. It has two guards for the
simulated board (no reset cause, no battery) and three options that exist only
there: `CONFIG_ZMK_ESB_SIM_ISR_LATENCY_US`, `CONFIG_ZMK_ESB_SIM_STALL_EVERY`
and `CONFIG_ZMK_ESB_SIM_SKIP_VERSION_CHECK`.
