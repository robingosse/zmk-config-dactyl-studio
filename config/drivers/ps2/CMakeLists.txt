zephyr_library()

zephyr_library_sources_ifdef(CONFIG_PS2_GPIO ps2_gpio.c)
