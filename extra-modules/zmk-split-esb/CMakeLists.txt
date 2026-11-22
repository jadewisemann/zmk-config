# Copyright (c) 2025 The ZMK Contributors
# SPDX-License-Identifier: MIT

if(CONFIG_ZMK_SPLIT_ESB)
  if(NOT CONFIG_HAS_HW_NRF_PPI)
    message(FATAL_ERROR "zmk-split-esb: only SoCs with PPI (nRF52 series) are supported")
  endif()

  # Radio layer: TDMA / frequency hopping on the ESB physical layer.
  zephyr_library_named(zmk_split_esb_radio)
  zephyr_library_include_directories(src/radio)
  zephyr_library_sources(
    src/radio/esb.c
    src/radio/esb_ppi.c
    src/radio/tx_ringbuf.c
  )

  # ZMK split transport on top of it.
  zephyr_library_named(zmk_split_esb_transport)
  zephyr_library_include_directories(
    src/radio
    src/transport
    ${APPLICATION_SOURCE_DIR}/include
  )
  zephyr_library_sources(
    src/transport/app_esb.c
    src/transport/common.c
  )
  zephyr_library_sources_ifdef(CONFIG_ZMK_SPLIT_ROLE_CENTRAL src/transport/central.c)
  zephyr_library_sources_ifndef(CONFIG_ZMK_SPLIT_ROLE_CENTRAL src/transport/peripheral.c)

  # ZMK core sizes its per-peripheral arrays from this definition; app/CMakeLists.txt
  # only sets it for the BLE transport.
  zephyr_compile_definitions_ifdef(CONFIG_ZMK_SPLIT_ROLE_CENTRAL
    ZMK_SPLIT_BLE_PERIPHERAL_COUNT=${CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS})
endif()
