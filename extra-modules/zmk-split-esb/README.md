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
