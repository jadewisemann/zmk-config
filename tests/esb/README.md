# ESB link test

Runs the ESB firmware of the dongle and both halves in a simulator, with the
real KLOTZ keymap, and checks what the dongle types. No hardware involved.

The simulator is [BabbleSim](https://babblesim.github.io) with Zephyr's
`nrf52_bsim` board: models of the nRF52's RADIO, RTC, TIMER, PPI, EGU and
clocks run the unmodified firmware, a simulated 2.4 GHz channel connects the
three devices, and each device can be given its own clock error. ZMK's own
Bluetooth split tests work the same way.

## What a run does

The left half holds its thumb key (Nav on hold, Space on tap) and the right
half presses J while it is held, which is Left on the Nav layer. A lost or
late thumb event types `J` instead, and that is exactly what a keyboard with a
struggling link does. Every cycle must type `Left`, and no half may lose sync
after its start-up scan.

| scenario | condition |
|---|---|
| `ideal` | perfect clocks, no packet loss |
| `slow80` | left half's clock 80 ppm slow |
| `slow1000` | left half's clock 1000 ppm slow |
| `fast1000` | right half's clock 1000 ppm fast |
| `dongle1000` | dongle's clock 1000 ppm fast, so both halves are slow |
| `loss30` | 30 % of all packets lost, clocks 300 and 200 ppm off |
| `long` (`--long`) | 12 minutes across the wrap of the 24-bit RTC, 5 % loss |

`slow80` is what the firmware up to `1365f2b` failed, and `slow1000` what it
failed up to `27a86b8`: both type `J` there (the older one also a late
`Space`), with the left half scanning for the dongle over and over.

## Running it

Needs `west`, `cmake`, `ninja`, `dtc` and a 32-bit capable `gcc`
(`gcc-multilib`). The simulator lives in a west workspace of its own, about
3.5 GB, anywhere outside this repository:

```sh
python3 tests/esb/run.py setup ~/esb-sim          # once, and after a west.yml change
python3 tests/esb/run.py test  ~/esb-sim          # ~3 minutes
python3 tests/esb/run.py test  ~/esb-sim --long   # ~5 minutes
python3 tests/esb/run.py test  ~/esb-sim --only slow1000
```

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
  `CONFIG_ZMK_ESB_SIM_ISR_LATENCY_US`.

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
  runs.

The firmware itself is not changed for the test. It has two guards for the
simulated board (no reset cause, no battery) and one option,
`CONFIG_ZMK_ESB_SIM_ISR_LATENCY_US`.
