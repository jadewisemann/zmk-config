# Build modes

Each firmware is `shield/board defaults` + the mode files below (applied in the
order listed in `scripts/matrix.py`) + the device's own files in
`config/devices/<device>/` (`half.conf` for its halves, `dongle.conf` for its dongle,
`ble.conf` for every one of its Bluetooth builds). Later files win.

| file | used by |
|---|---|
| `half.conf` | every battery-powered half, all modes |
| `ble.conf` | everything that talks Bluetooth (BLE halves, BLE dongle) |
| `ble_central.conf` | whatever is the BLE split central (left half in `ble`, right half in `bleright`, dongle in `bledongle`) |
| `central.conf` | right half when it is the central (`bleright`) |
| `peripheral.conf` | left half when something else is the central (`bleright`, `bledongle`, `esb`) |
| `ble_dongle.conf` | BLE dongle |
| `esb.conf` | every ESB build (halves and dongle) - radio settings must match |
| `esb_half.conf` | both ESB halves: no idle before deep sleep, and the 32 MHz crystal kept running (`extra-modules/hfxo-always-on`) |
| `esb_left.conf` / `esb_right.conf` | ESB peripheral IDs |
| `esb_dongle.conf` | ESB dongle |
