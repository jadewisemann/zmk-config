# zmk-split-esb

ZMK split transport over a TDMA / frequency-hopping protocol on the Nordic ESB
physical layer: a USB dongle is the central and polls every half in its own
time slot. Used by the `esb` build mode (see `config/modes/README.md`).

The code is kept in this repository on purpose. It used to come from two
external repositories, and fixing the link meant patching forks of both.

| directory | what | taken from | licence |
|---|---|---|---|
| `src/transport` | ZMK split transport (central + peripheral) | [kumbung01/zmk-feature-split-esb](https://github.com/kumbung01/zmk-feature-split-esb) `f7b1bd1`, plus the two fixes from our fork (`57a31ec`) | MIT, `app_esb.*` Nordic 5-Clause |
| `src/radio` | radio layer: slots, hopping, ARQ, sync | `subsys/esb` and `include/esb.h` of [kumbung01/sdk-nrf](https://github.com/kumbung01/sdk-nrf) `07b7e70` (branch `v3.1-branch-tdma-fhss`) | Nordic 5-Clause (`LICENSE-Nordic-5-Clause`) |

## How it is built

`scripts/build.py` passes this directory as a Zephyr module
(`ZMK_EXTRA_MODULES`) to every build that uses the `config-esb` manifest.
nRF Connect SDK is still in that manifest because the radio layer links
against its MPSL glue, but the SDK's own `subsys/esb` is not built:
`CONFIG_ESB` stays off and `CONFIG_ZMK_SPLIT_ESB` selects the same
dependencies itself.

Only SoCs with PPI (nRF52 series) are supported.

## Changes made when the code was brought in

No behaviour change; the firmware built from this directory matched the one
built from the external repositories function for function.

- Radio options are `CONFIG_ZMK_ESB_*` instead of `CONFIG_ESB_*`, so they
  cannot be confused with the SDK's symbols of the same name.
- `esb.h` is `esb_tdma.h`: the SDK ships an `esb.h` on the include path.
- Dropped what no build here could use: the nRF53/nRF54 variants (DPPI, the
  nRF5340 network-core split), the non-ring-buffer TX FIFO and the unused
  `zmk_split_central_status_changed` event.

## How the link works, briefly

The dongle transmits in every slot (1 ms at 1 kHz), alternating between the
halves and hopping over 39 channels by slot number. A half does not listen to
all of that. It opens its receiver only

- when it has something to send - it then answers the poll with its data, and
- once per heartbeat (250 ms), to measure where the dongle's slots are now.

In between it predicts the slot times on its own 32 kHz clock. A half that
gets no packet for 0.5 s gives up and scans: it listens on one channel until a
packet for it arrives there.

## Changes to the link since it was brought in

The halves kept losing sync and took seconds to get back, which showed up as
keys arriving a second late in a burst and one half dropping out at a time.

Keeping sync (peripheral):

- The receiver opens 5 ticks (152 us) before the expected slot instead of 2
  (61 us), and the heartbeat is 250 ms instead of 1 s. The old pair tolerated
  the dongle's clock running ahead by 30-40 ppm; two in-tolerance 32 kHz
  crystals can be 40 ppm apart. A packet that comes early is missed outright,
  one that comes late is not, so only a half *slower* than the dongle failed -
  one half could work while the other did not.
- The drift estimate is fed the drift, not just the error left over after its
  own correction, is rounded to the nearest tick, and no longer ignores
  anything beyond 50 us.
- After a scan the half re-syncs after 4 ms, then 16 ms, 64 ms, and only then
  every 250 ms, and each of these measures the drift for the next, longer one.
  Waiting a whole heartbeat for the first measurement meant a half more than
  about 600 ppm behind the dongle never got one: it missed that heartbeat,
  timed out, scanned again and started over, delivering its keys only right
  after each scan, up to half a second late. In simulation it now holds at
  1000 ppm either way, and learns the drift to a few ppm.
- A heartbeat is any exchange a period or more after the last sync, not only
  the one loop that is an exact multiple of the period.
- The idle wake-up time includes the drift correction, and is compared as a
  signed value: a wake-up point already in the past used to arm a compare the
  counter had passed, leaving the half silent until its next key press.
- Slot times are computed relative to the last sync, so the 24-bit RTC
  wrapping (every 512 s) cannot misplace a slot.

Getting sync back:

- Every packet from the dongle that carries no payload carries the slot
  parameters. They used to be sent only to a half the dongle had given up on,
  which takes it 3 s, while the half starts scanning after 0.5 s - so a lost
  sync cost at least 3 s. Now any poll will do.
- A half scans at once after losing sync and after start-up, ten scans back to
  back, before falling back to the old one scan per second.
- After a scan the half answers the next poll right away and re-measures the
  slot time there, with a wider window for that one exchange.
- After a scan the half no longer treats the dongle's current sequence number
  as received (the next payload under it was dropped as a duplicate). The data
  that was in flight when the link dropped is discarded only if the dongle's
  packet shows it arrived; otherwise it is sent again.
- A synced half answers these control packets where it stayed silent on an
  empty poll. A command from the dongle (backlight, reset, bootloader) sent
  right after a key press used to be acknowledged and thrown away as a
  duplicate nearly every time; now that needs the half's answer to be lost.
- The fast scans are given back by the first completed exchange, and a scan
  that ends on a packet without slot parameters does not use one up.

## Timing between the halves

Over ESB both halves reach the dongle within a millisecond or three. With a
Bluetooth split the half that is the central handles its own keys at once and
the other half's keys wait for the next connection event, up to 7.5 ms. That
difference is small, but a hold-tap on one half and a key on the other are
decided by which was released first, and a keymap tuned over Bluetooth has
that offset built in: in simulation, a left thumb (Space / Nav) released up
to about 6 ms after a right-hand key is a tap over Bluetooth and a hold over
ESB.

`CONFIG_ZMK_SPLIT_ESB_PACED_SOURCE=<peripheral id>` makes the dongle pass that
half's key events on only at multiples of 7.5 ms, which puts the decision
point back where it is over Bluetooth, at the price of that latency for that
half. It is a per-keyboard choice: `config/devices/<id>/esb_dongle.conf`,
which only the ESB dongle firmware gets. KLOTZ sets it for the right half.

## Guard rails

- **A half does not turn its transmitter down.** It used to lower its power
  to as little as -20 dBm while the dongle heard it well, which left no margin
  against interference; every packet lost to that is a 2 ms retry. The floor is
  now the -4 dBm it starts with (it still goes up to +4 dBm when packets fail).
  This cannot be shown in the simulation; `power`, `retries` and `rssi` in the
  statistics are the way to see it.

- **The dongle restarts its slot sequence.** Every slot is started by a
  compare of the 32 kHz counter and ended by an interrupt of the timer that
  slot starts; preparing the next slot from there is all that keeps the
  sequence going. Held up once for longer than the ~100 us it has, the dongle
  armed a compare value that had already passed: nothing started the slot,
  nothing ended it, and the radio stayed silent until the counter came round
  again 512 s later - the whole keyboard dead until the dongle was plugged in
  again. A second compare, moved on with every slot, now restarts the
  sequence within three slots (`stalled` in the statistics counts these).
- **Firmware of different releases does not half-work.** The dongle sends a
  protocol version with the slot parameters and a half sends it with its
  self-report (`CONFIG_ZMK_ESB_PROTOCOL_VERSION`; raise its default whenever
  the packets or the timing rules change). A half does not sync to a dongle
  with another version, and the dongle takes no key events from a half it
  has not seen its own version from, which a half therefore sends first
  after every scan. Before, a half with older firmware connected and was
  merely slow, which looked like a bad link.

## Link statistics

The link is tested in simulation (`tests/esb`), which covers the logic but
not the radio and the crystals of real boards. For those it reports on itself.

Every half counts what happens on its side and sends the counts to the dongle
in the packets that carry no payload (`struct esb_peer_stats`); that is always
on and costs no air time. The dongle counts per half as well. Printing is a
separate firmware: `<keyboard>-esb-dongle-log.uf2` is the ESB dongle built with
the `zmk-usb-logging` snippet and `CONFIG_ZMK_SPLIT_ESB_LINK_STATS=y`
(`config/modes/esb_log.conf`). It works as a dongle like the normal one and is
a USB serial port besides. Open that port (`screen /dev/tty.usbmodem* 115200`
on macOS, PuTTY on the COM port on Windows) and it prints, once per second and
half (half 0 is the left one):

```
ESB half 0 up rssi -58 answers 4 keys 0 dup 0 lost(total) 0 skipped 0 stalled(total) 0 refused 0
ESB half 0 reports: timeouts 0 retries 0 scans(total) 1 wait 3 ms power -4 dBm drift -12 ppm 32k crystal 32M ok
ESB key: half 0 position 30 down
```

| field | meaning | healthy |
|---|---|---|
| `up` / `DOWN` | whether the dongle counts the half as connected | `up` |
| `rssi` | signal strength of the half at the dongle, dBm | -45 to -70 |
| `answers` | polls the half answered in that second | 4 when idle (the heartbeats), more while typing |
| `keys` | answers that carried key events | |
| `dup` | key packets received twice (the dongle's acknowledgement was lost) | 0, now and then 1 |
| `lost(total)` | times the dongle gave up on the half (3 s of silence) | stays the same |
| `skipped` | slots the dongle could not prepare in time (both halves) | 0 |
| `stalled(total)` | times the dongle's slot sequence stopped and its watchdog restarted it | 0 |
| `refused` | key packets not taken because the half has not shown this dongle's protocol version | 0 |
| `timeouts` | RX windows of the half that stayed empty | 0, a few while typing |
| `retries` | packets of the half the dongle did not acknowledge | 0, a few while typing |
| `scans(total)` | times the half lost sync and searched for the dongle | stays the same after start-up |
| `wait` | longest time a key event waited in the half, from the key scan until the dongle acknowledged it, over the last 1-2 s | 2-6 ms |
| `power` | TX power the half has settled on | |
| `drift` | how far the half's 32 kHz clock is off the dongle's, as the half has learned it | within about 50 ppm, steady |
| `32k` | what the half's 32 kHz clock runs from | `crystal`; `RC` means the board's crystal is missing or dead |
| `32M` | `WAS-OFF` if the half ever found its 32 MHz crystal stopped when it woke up | `ok` |

`wait` is the number that says whether the radio is what makes a key late.
The `ESB key` lines show the order in which the dongle saw the key events.
A half that prints `reports: nothing usable` forever is switched off, or runs
firmware from another release (then `refused` counts up, or `answers` stays 0).

## Known limits

- 1 kHz is the rate this is meant for. At 2 kHz the slot is too short for the
  slot parameters in every poll, so the dongle sends them only to a half it
  has given up on, as before, and recovery takes the old 3 s.
- Dongle-to-half commands can still be lost or repeated when a packet is lost
  at the wrong moment: the dongle reuses a sequence number when it replaces a
  payload-less packet with a command, and re-reads its ring on every
  retransmission. Key events (half to dongle) are not affected, and
  whole-key state is re-sent after every reconnect.
- Nothing here has been measured on hardware. `tests/esb` runs the three
  firmwares in simulation; its README lists what that cannot show.
- A half whose 32 kHz clock is the RC oscillator converts the time of a long
  scan into slot times with that clock's error, which the simulation does not
  have. A board without a working 32 kHz crystal should be built with
  `CONFIG_CLOCK_CONTROL_NRF_K32SRC_RC=y` (calibrated, 500 ppm). With the
  crystal setting the clock starts on the uncalibrated RC oscillator and stays
  there if the crystal never comes up.
