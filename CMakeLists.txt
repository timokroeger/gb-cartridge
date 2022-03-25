cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(gb-cartridge C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(cartridge)
pico_generate_pio_header(cartridge ${CMAKE_CURRENT_LIST_DIR}/src/cartridge_interface.pio)
target_sources(cartridge PRIVATE src/main.c)
target_link_libraries(cartridge
    hardware_clocks
    hardware_dma
    hardware_pio
    hardware_pll
    pico_stdlib
)
pico_add_extra_outputs(cartridge)

add_executable(simulator)
pico_generate_pio_header(simulator ${CMAKE_CURRENT_LIST_DIR}/src/simulation.pio)
target_sources(simulator PRIVATE src/simulator.c)
target_link_libraries(simulator
    hardware_pio
    pico_stdlib
)
pico_set_binary_type(simulator no_flash)
pico_add_extra_outputs(simulator)
