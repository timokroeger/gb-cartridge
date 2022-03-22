// TODO: License Header

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "cartridge_interface.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/pll.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

// Just enough for simple games like Tetris or Dr. Mario.
#define ROM_SIZE (32 * 1024)

// Pin Base:
// 0 .. 7: D0-D7, A0-A7, A8-A15 - 8 bit multiplexed:
// 8 ..11: CLK, A15, #CS, #RD
// 12..15: OE0-2, DIR0
#define PIN_BASE 2

// TODO: Include actual GB code
static const uint8_t __in_flash() __aligned(ROM_SIZE) g_rom[ROM_SIZE] = {0xFF};

// Pointer to the start of the current ROM memory bank.
static volatile uintptr_t s_rom_addr;

// GB reads ROM, we provide the data.
static volatile uint16_t s_rom_out_command[4] = {
    // DMA writes actual ROM data to the first byte before we send this
    // command sequence to the SM.
    DATA_OUT << 12 | 0xAA,  // pins
    0xF0FF,                 // pindir: Sets D0-D7 to output.
    0xF000,                 // pindir: Resets D0-D7 to input.
    // Release the data lines by switching transceiver.
    ADDR_HI << 12 | 0xAA,  // pins
};

static void init_dma(uint addr_dreq, const volatile void *addr_fifo,
                     volatile void *data_fifo) {
  // Read ROM data DMA chain
  int reset_rom_addr_ch = dma_claim_unused_channel(true);
  int addr_ch = dma_claim_unused_channel(true);
  int copy_rom_ch = dma_claim_unused_channel(true);
  int data_inout_ch = dma_claim_unused_channel(true);
  dma_channel_config c;

  // Step 1: Reset `copy_rom_ch` read register to start of ROM memory bank.
  s_rom_addr = (uintptr_t)xip_nocache_noalloc_alias_untyped(&g_rom);
  c = dma_channel_get_default_config(reset_rom_addr_ch);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, false);
  channel_config_set_chain_to(&c, addr_ch);
  dma_channel_configure(reset_rom_addr_ch, &c,
                        &dma_hw->ch[copy_rom_ch].read_addr, &s_rom_addr, 1,
                        false);

  // Step 2: Get the ROM address from the `read_addr` SM and use it as offset
  //         into the current ROM bank. Directly write the offset to the read
  //         register of the `copy_rom_ch` dma channel. Use the atomic set alias
  //         memory address to add the offset without overwriting.
  c = dma_channel_get_default_config(addr_ch);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, false);
  channel_config_set_chain_to(&c, copy_rom_ch);
  channel_config_set_dreq(&c, addr_dreq);
  dma_channel_configure(addr_ch, &c,
                        hw_set_alias(&dma_hw->ch[copy_rom_ch].read_addr),
                        addr_fifo, 1, false);

  // Step 3: Copy from ROM to temporary memory location.
  //         This takes ~50 cycles because it accesses flash and makes it the
  //         slowest operation of a bus access cycle.
  c = dma_channel_get_default_config(copy_rom_ch);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, false);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_chain_to(&c, data_inout_ch);
  dma_channel_configure(copy_rom_ch, &c, &s_rom_out_command[0],
                        NULL,  // Calculated in step 1 and 2.
                        1, false);

  // Step 4: Transfer the selected command to the `data_out_in` SM.
  c = dma_channel_get_default_config(data_inout_ch);
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, false);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
  channel_config_set_chain_to(&c, reset_rom_addr_ch);
  dma_channel_configure(data_inout_ch, &c, data_fifo,
                        s_rom_out_command,  // Updated by address decoder.
                        4, false);

  // TODO: Independent DMA channel to pull data from `data_out_in`.

  dma_channel_start(reset_rom_addr_ch);
}

static void init_cartridge_interface(PIO pio, uint pins) {
  uint offset;
  pio_sm_config c;

  uint control_pins = pins + CARTRIDGE_BITS;

  // Configure GPIOs
  uint temp_sm = pio_claim_unused_sm(pio, true);
  pio_sm_set_pins(pio, temp_sm, ADDR_HI << control_pins);
  pio_sm_set_consecutive_pindirs(pio, temp_sm, pins, CARTRIDGE_BITS, false);
  pio_sm_set_consecutive_pindirs(pio, temp_sm, control_pins, CONTROL_BITS,
                                 true);
  for (int i = 0; i < CARTRIDGE_BITS; i++) {
    pio_gpio_init(pio, pins + i);
  }
  for (int i = 0; i < CONTROL_BITS; i++) {
    pio_gpio_init(pio, control_pins + i);
  }
  pio->input_sync_bypass |= (1 << (pins + CLK));

  // Configure the `read_addr` state machine
  uint read_addr_sm = temp_sm;
  offset = pio_add_program(pio, &read_addr_program);
  c = read_addr_program_get_default_config(offset);
  sm_config_set_in_pins(&c, pins);
  sm_config_set_set_pins(&c, control_pins, CONTROL_BITS);
  sm_config_set_in_shift(&c, false,  // shift_direction=left
                         true, 15);  // autopush enabled
  pio_sm_init(pio, read_addr_sm, offset, &c);
  pio_sm_set_enabled(pio, read_addr_sm, true);

  // Configure the `data_out_in` state machine
  uint data_out_in_sm = pio_claim_unused_sm(pio, true);
  offset = pio_add_program(pio, &data_out_in_program);
  c = data_out_in_program_get_default_config(offset);
  sm_config_set_in_pins(&c, pins);
  sm_config_set_out_pins(&c, pins, CARTRIDGE_BITS + CONTROL_BITS);
  sm_config_set_out_shift(&c, true,   // shift_direction=right
                          true, 16);  // autopull enabled
  sm_config_set_in_shift(&c, false,   // shift_direction=left
                         true, 8);    // autopush enabled
  pio_sm_init(pio, data_out_in_sm, offset, &c);
  pio_sm_set_enabled(pio, data_out_in_sm, true);

  // DMA chain to connect the two state machines
  init_dma(pio_get_dreq(pio, read_addr_sm, false), &pio->rxf[read_addr_sm],
           &pio->txf[data_out_in_sm]);
}

static uint init_clk_emu(PIO pio, uint pin) {
  uint sm = pio_claim_unused_sm(pio, true);

  pio_sm_set_consecutive_pindirs(pio, sm, pin, 2, true);
  pio_gpio_init(pio, pin);
  pio_gpio_init(pio, pin + 1);

  uint offset = pio_add_program(pio, &clk_emu_program);
  pio_sm_config c = clk_emu_program_get_default_config(offset);
  sm_config_set_set_pins(&c, pin, 2);
  sm_config_set_clkdiv_int_frac(&c, 15, 0);
  pio_sm_init(pio, sm, offset, &c);
  return sm;
}

static uint init_memory_benchmark(PIO pio, uint pin) {
  uint sm = pio_claim_unused_sm(pio, true);

  pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
  pio_gpio_init(pio, pin);

  init_dma(pio_get_dreq(pio, sm, false), &pio->rxf[sm], &pio->txf[sm]);

  uint offset = pio_add_program(pio, &memory_benchmark_program);
  pio_sm_config c = memory_benchmark_program_get_default_config(offset);
  sm_config_set_sideset_pins(&c, pin);
  pio_sm_init(pio, sm, offset, &c);
  return sm;
}

static void __attribute__((optimize("O3")))
__no_inline_not_in_flash_func(address_decoder)(void) {
  // TODO: Address decoder
}

static void __no_inline_not_in_flash_func(ram_sleep_loop)(PIO pio, uint sm) {
  pio_sm_set_enabled(pio, sm, true);
  while (true) {
    __wfi();
  }
}

int main() {
  // Run system clock at 1MHz for easy cycle counting with a logic analyzer.
  clock_configure(clk_ref, CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC, 0,
                  XOSC_MHZ * MHZ, XOSC_MHZ * MHZ);
  clock_configure(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF, 0,
                  XOSC_MHZ * MHZ, 1 * MHZ);
  clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                  48 * MHZ, 48 * MHZ);
  pll_deinit(pll_sys);

  // Have a clock ouput to verify our configuration.
  clock_gpio_init(21, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 1);

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  gpio_put(PICO_DEFAULT_LED_PIN, 1);

  stdio_init_all();
  printf("hello\n");

  init_cartridge_interface(pio0, PIN_BASE);
  uint sm = init_clk_emu(pio0, 18);
  // int sm = init_memory_benchmark(pio0, 20);

  // Make sure that no cade runs from flash memory anymore to guarantee for
  // constant flash access times.
  ram_sleep_loop(pio0, sm);

  return 0;
}
