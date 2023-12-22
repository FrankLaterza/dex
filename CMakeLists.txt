cmake_minimum_required(VERSION 3.13)

# extract the name of the parent directory
get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

# set the freertos path
SET(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel)

# set the board
set(PICO_BOARD pico_w)

# include the sdk
include(pico-sdk/external/pico_sdk_import.cmake)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# define project
project(${PROJECT_NAME})

pico_sdk_init()

add_subdirectory(src)