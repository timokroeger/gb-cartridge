cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(gb-cartridge C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(gb-cartridge
    src/main.c
    src/roms/dummy.c
)

pico_generate_pio_header(gb-cartridge
    ${CMAKE_CURRENT_LIST_DIR}/src/read_addr.pio
)

target_link_libraries(gb-cartridge
    pico_stdlib
    hardware_dma
    hardware_pio
)

pico_set_binary_type(gb-cartridge no_flash)
pico_add_extra_outputs(gb-cartridge)
