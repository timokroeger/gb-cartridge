// TODO: License Header

#if !PICO_NO_FLASH && !PICO_COPY_TO_RAM
#error "code must execute from RAM for predictable flash access times"
#endif

#include <stdint.h>

// TODO: Move to QSPI flash
extern uint8_t g_rom[32 * 1024];

int main() {
    return 0;
}
