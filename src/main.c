// TODO: License Header

// #if !PICO_NO_FLASH && !PICO_COPY_TO_RAM
// #error "code must execute from RAM for predictable flash access times"
// #endif

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "cartridge_interface.pio.h"
#include "config.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/pll.h"
#include "pico/stdlib.h"

const uint8_t g_rom[ROM_SIZE] __attribute__((aligned(ROM_SIZE))) = {
  0 // TODO: Include actual GB code
};

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
  assert((rom_addr % ROM_SIZE) == 0 && "GB ROM memory is not aligned properly");

  // Make it live forever so that DMA can use it.
  static uintptr_t s_rom_addr;
  s_rom_addr = rom_addr;

  // Read ROM data DMA chain
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
      &s_rom_addr, 1, false);

  // Step 3: Transfer ROM byte to the ROM data PIO FIFO
  c = dma_channel_get_default_config(dma_data_out);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, false);
  channel_config_set_chain_to(&c, dma_fetch_addr);
  dma_channel_configure(dma_data_out, &c, data_fifo,
                        NULL,  // The source address is updated in step 1 and 2
                        1, false);
}

static void init_cartridge_interface(PIO pio, uint muxed_cartridge_signals,
                                     uint cartridge_signals, uint mux_control,
                                     uintptr_t rom_addr) {
  uint sm, offset;
  pio_sm_config c;

  // Configure GPIOs
  uint temp_sm = pio_claim_unused_sm(pio, true);
  pio_sm_set_pins(pio, sm, ADDR_HI << mux_control);
  pio_sm_set_consecutive_pindirs(pio, sm, muxed_cartridge_signals,
                                 MUXED_SIGNAL_BITS, false);
  pio_sm_set_consecutive_pindirs(pio, sm, cartridge_signals, SIGNAL_BITS,
                                 false);
  pio_sm_set_consecutive_pindirs(pio, sm, mux_control, CONTROL_BITS, true);

  // Configure read_addr state machine
  uint read_addr_sm = temp_sm;
  offset = pio_add_program(pio, &read_addr_program);
  c = read_addr_program_get_default_config(offset);
  sm_config_set_in_pins(&c, muxed_cartridge_signals);
  sm_config_set_set_pins(&c, mux_control, 4);
  sm_config_set_in_shift(&c, false,  // shift_direction=left
                         true, 15);  // autopush enabled
  pio_sm_init(pio, sm, offset, &c);
  pio_sm_set_enabled(pio, sm, true);

  // Configure data state machine
  uint data_out_sm = pio_claim_unused_sm(pio, true);
  offset = pio_add_program(pio, &data_out_program);
  c = data_out_program_get_default_config(offset);
  sm_config_set_out_pins(&c, muxed_cartridge_signals, 8);
  sm_config_set_in_pins(&c, muxed_cartridge_signals + 8);
  sm_config_set_sideset_pins(&c, mux_control);
  sm_config_set_in_shift(&c, false,   // shift_direction=left
                         false, 32);  // autopush disabled

  // TODO: configure GPIO

  // DMA chain to connect the two state machines
  init_dma(pio_get_dreq(pio, read_addr_sm, false), &pio->rxf[read_addr_sm],
           &pio->txf[data_out_sm], rom_addr);

  // TODO: enable SMs
}

void init_memory_benchmark(PIO pio, uint pin, uintptr_t rom_addr) {
  uint sm = pio_claim_unused_sm(pio, true);

  pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
  pio_gpio_init(pio, pin);

  init_dma(pio_get_dreq(pio, sm, false), &pio->rxf[sm], &pio->txf[sm],
           rom_addr);

  uint offset = pio_add_program(pio, &memory_benchmark_program);
  pio_sm_config c = memory_benchmark_program_get_default_config(offset);
  pio_sm_init(pio, sm, offset, &c);
  pio_sm_set_sideset_pins(pio, sm, pin);
  pio_sm_set_enabled(pio, sm, true);
}

int main() {
  // Run system and peripheral clock at 12MHz for easy cycle counting with a
  // logic analyzer.
  clock_configure(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF, 0, 12 * MHZ,
                  1 * MHZ);
  clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_XOSC_CLKSRC,
                  12 * MHZ, 12 * MHZ);
  pll_deinit(pll_sys);

  stdio_init_all();

  // init_cartridge_interface(pio0, 0, 8, 11, (uintptr_t)&g_rom);

  printf("memory benchmark\n");
  clock_gpio_init(21, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 1);
  // const char *romaddr = &g_rom;
  const char *romaddr = xip_nocache_noalloc_alias_untyped(&g_rom);
  init_memory_benchmark(pio1, 20, (uintptr_t)romaddr);

  while (true) {
  }

  return 0;
}
