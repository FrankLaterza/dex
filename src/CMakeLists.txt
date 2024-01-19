add_executable(${PROJECT_NAME}
    pid.c
    stepper.c
    controls.c
    pinout.c
    kalman.c
    mpu6050.c
    utils.c
    bt_hid.c
    main.c
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(${PROJECT_NAME} 
    pico_stdlib
    pico_cyw43_arch_none
    pico_multicore
    FreeRTOS-Kernel-Heap4
    pico_btstack_classic
    pico_btstack_ble
    pico_btstack_cyw43
    hardware_i2c
    hardware_timer
    hardware_flash
    hardware_watchdog
)

# slow down spi flash for overclock over 250Mhz
# pico_define_boot_stage2(pico-coremark-div4 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
# target_compile_definitions(pico-coremark-div4 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
# pico_set_boot_stage2(${PROJECT_NAME} pico-coremark-div4)

# enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_enable_stdio_semihosting(${PROJECT_NAME} 0)


pico_add_extra_outputs(${PROJECT_NAME})