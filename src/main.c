// TODO: License Header

#if !PICO_NO_FLASH && !PICO_COPY_TO_RAM
#error "code must execute from RAM for predictable flash access times"
#endif

#include <assert.h>
#include <stdint.h>

#include "hardware/dma.h"
#include "pico/stdlib.h"
#include "read_addr.pio.h"

// Just enough for simple games like Tetris or Dr. Mario.
#define ROM_SIZE (32 * 1024)

// Multiplexed cartridge pins
#define PIN_CARTRIDGE_BASE 0

// OE and DIR pins of the transceivers
#define PIN_CONTROL_BASE

// TODO: Move to QSPI flash
extern uint8_t g_rom[ROM_SIZE] __attribute__((aligned(ROM_SIZE)));

static void pio_init_gpio(PIO pio) {
  for (int i = 0; i < 8; i++) {
    pio_gpio_init(pio, PIN_CARTRIDGE_BASE + i);
  }

  for (int i = 0; i < 5; i++) {
    pio_gpio_init(pio, PIN_CONTROL_BASE + i);
  }
}

static void init_dma(uint addr_dreq, const volatile void *addr_fifo,
                     volatile void *data_fifo, uintptr_t rom_addr) {
  printf("ROM base: %p\n", rom_addr);
  assert((rom_addr % ROM_SIZE) == 0 && "GB ROM memory is not aligned properly");

  // Make it live forever so that DMA can use it.
  static uintptr_t s_rom_addr;
  s_rom_addr = rom_addr;

  // Read ROM data DMA chain
  // TODO: is transfer count modified???
  int dma_fetch_addr = dma_claim_unused_channel(true);
  int dma_addr_offset = dma_claim_unused_channel(true);
  int dma_data_out = dma_claim_unused_channel(true);
  dma_channel_config c;

  // Step 1: Fetch ROM address from the addr decoder PIO FIFO
  c = dma_channel_get_default_config(dma_fetch_addr);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, false);
  channel_config_set_chain_to(&c, dma_addr_offset);
  channel_config_set_dreq(&c, addr_dreq);
  dma_channel_configure(dma_fetch_addr, &c,
                        &dma_hw->ch[dma_data_out].al1_read_addr, addr_fifo, 1,
                        true);

  // Step 2: Add the ROM address offset to the address by writing to the atomic
  //         set alias
  c = dma_channel_get_default_config(dma_addr_offset);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, false);
  channel_config_set_chain_to(&c, dma_data_out);
  dma_channel_configure(
      dma_addr_offset, &c,
      hw_set_alias_untyped(&dma_hw->ch[dma_data_out].al1_read_addr),
      &s_rom_addr, 1, true);

  // Step 3: Transfer ROM byte to the ROM data PIO FIFO
  c = dma_channel_get_default_config(dma_data_out);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, false);
  dma_channel_configure(dma_data_out, &c, data_fifo,
                        NULL,  // The source address is updated in step 1 and 2
                        1, true);
}

static void init_cartridge_interface(PIO pio, uint in_pin_base,
                                     uint out_pin_base, uintptr_t rom_addr) {
  uint sm, offset;

  // Configure read_addr state machine
  sm = pio_claim_unused_sm(pio, true);
  offset = pio_add_program(pio, &read_addr_program);
  pio_sm_config c = read_addr_program_get_default_config(offset);
  sm_config_set_in_pins(&c, in_pin_base);
  sm_config_set_out_pins(&c, out_pin_base, 5);
  pio_sm_set_consecutive_pindirs(pio, sm, in_pin_base, 8, false);
  pio_sm_set_consecutive_pindirs(pio, sm, in_pin_base, 5, true);
  pio_sm_init(pio, sm, offset, &c);
  pio_sm_set_enabled(pio, sm, true);

  // TODO: Configure data state machine

  // DMA chain to connect the two state machines
  init_dma(pio_get_dreq(pio, sm, false), &pio->rxf[sm], NULL, rom_addr);
}

int main() {
  stdio_init_all();

  init_cartridge_interface(pio0, 0, 8, (uintptr_t)&g_rom);

  // TODO: have a look at bus priories

  return 0;
}
