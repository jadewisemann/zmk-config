if(CONFIG_KB_HFXO_ALWAYS_ON)
  zephyr_library()
  zephyr_library_sources(src/hfxo_always_on.c)
endif()
