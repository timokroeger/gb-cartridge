#include <stdint.h>

#include "config.h"

uint8_t g_rom[ROM_SIZE]  __attribute__((aligned(ROM_SIZE))) = {
    0 // Include your own backup ROMS here
};
