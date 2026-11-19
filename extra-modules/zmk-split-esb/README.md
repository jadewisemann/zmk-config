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
  anything beyond 50 us. With it the link holds to several hundred ppm.
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
  as received (the next payload under it was dropped as a duplicate), and no
  longer discards the data that was in flight when the link dropped.
- A synced half answers these control packets where it stayed silent on an
  empty poll. That removes the same duplicate drop from normal operation: a
  command from the dongle (backlight, reset, bootloader) sent right after a
  key press was acknowledged and thrown away.
