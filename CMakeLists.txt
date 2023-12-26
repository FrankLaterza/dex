cmake_minimum_required(VERSION 3.13)

# extract the name of the parent directory
get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(PICO_BTSTACK_PATH ${CMAKE_CURRENT_LIST_DIR}/btstack)

# set the freertos path
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel)

# set the board
set(PICO_BOARD pico_w)

# include the sdk
include(pico-sdk/external/pico_sdk_import.cmake)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# define project
project(${PROJECT_NAME} C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(src)