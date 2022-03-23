cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(gb-cartridge C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(gb-cartridge)

pico_generate_pio_header(gb-cartridge
    ${CMAKE_CURRENT_LIST_DIR}/src/cartridge_interface.pio
)

target_sources(gb-cartridge PRIVATE src/main.c)

target_include_directories(gb-cartridge PRIVATE src)

target_link_libraries(gb-cartridge
    hardware_clocks
    hardware_dma
    hardware_pio
    hardware_pll
    pico_stdlib
)

pico_add_extra_outputs(gb-cartridge)
