cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(gb-cartridge C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_compile_options(-Wall -Wextra)

add_executable(cartridge)
pico_generate_pio_header(cartridge ${CMAKE_CURRENT_LIST_DIR}/src/cartridge_interface.pio)
target_sources(cartridge PRIVATE
    src/main.c
    src/rom.c
)
target_link_libraries(cartridge
    hardware_clocks
    hardware_dma
    hardware_pio
    pico_multicore
    pico_stdlib
)
target_compile_definitions(cartridge PRIVATE
    PICO_TIME_DEFAULT_ALARM_POOL_DISABLED
)
pico_add_extra_outputs(cartridge)

add_executable(simulator)
pico_generate_pio_header(simulator ${CMAKE_CURRENT_LIST_DIR}/src/simulation.pio)
target_sources(simulator PRIVATE
    src/simulator.c
    lib/cobs-c/cobs.c
)
target_include_directories(simulator PRIVATE lib/cobs-c)
target_link_libraries(simulator
    hardware_pio
    pico_stdlib
)
pico_enable_stdio_usb(simulator 1)
pico_enable_stdio_uart(simulator 0)
pico_set_binary_type(simulator no_flash)
pico_add_extra_outputs(simulator)
