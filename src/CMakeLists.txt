add_executable(${PROJECT_NAME}
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
)

# enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

pico_add_extra_outputs(${PROJECT_NAME})