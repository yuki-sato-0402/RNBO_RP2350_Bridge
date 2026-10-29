# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.17)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(baremetal C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(baremetal
    main.cpp
    tlsf.c
)

pico_generate_pio_header(baremetal ${CMAKE_CURRENT_LIST_DIR}/audio_i2s_full_duplex.pio)

target_include_directories(baremetal PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/export
    ${CMAKE_CURRENT_LIST_DIR}/export/common
)

target_compile_options(baremetal PRIVATE
    -O3
    -fsingle-precision-constant
)

target_link_libraries(baremetal PRIVATE
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_adc
    hardware_clocks
)

pico_add_extra_outputs(baremetal)
