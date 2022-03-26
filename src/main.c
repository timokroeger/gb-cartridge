// Copyright 2022 Timo Kröger <timokroeger93@gmail.com>

#include <assert.h>
#include <stdint.h>

#include "cartridge_interface.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

#define ROM_SIZE (32 * 1024)
#define RAM_SIZE (8 * 1024)

// Pin Base:
// 0 .. 7: D0-D7, A0-A7, A8-A15 - 8 bit multiplexed:
// 8 ..11: A15, #CS, #RD, CLK
// 12..15: OE0-2, DIR0
#define PIN_BASE 0

#define CARTRIDGE_PIO pio0
#define CARTRIDGE_SM_ADDR 0
#define CARTRIDGE_SM_DATA 1

#define DMA_CH_RST_ROM_ADDR 0
#define DMA_CH_ADDR_OFFSET 1
#define DMA_CH_FLASH_TO_RAM 2
#define DMA_CH_DATA_CMD 3

// pindir in upper word
// pin data in lower word
#define DATA_CMD_OUT                                            \
  ((((uint32_t)CONTROL_MASK << CARTRIDGE_BITS) | 0xFF) << 16) | \
      (DATA_OUT << CARTRIDGE_BITS)
#define DATA_CMD_IN                                    \
  (((uint32_t)CONTROL_MASK << CARTRIDGE_BITS) << 16) | \
      (DATA_IN << CARTRIDGE_BITS)

// pin in upper word
// pindir data in lower word
#define DATA_CMD_RELEASE \
  ((ADDR_HI << CARTRIDGE_BITS) << 16) | (CONTROL_MASK << CARTRIDGE_BITS)

// https://github.com/svendahlstrand/10-print-game-boy
static const uint8_t __in_flash() __aligned(ROM_SIZE) g_rom[ROM_SIZE] = {
    0x03, 0x03, 0x07, 0x07, 0x0E, 0x0E, 0x1C, 0x1C, 0x38, 0x38, 0x70, 0x70,
    0xE0, 0xE0, 0xC0, 0xC0, 0xC0, 0xC0, 0xE0, 0xE0, 0x70, 0x70, 0x38, 0x38,
    0x1C, 0x1C, 0x0E, 0x0E, 0x07, 0x07, 0x03, 0x03, 0xCD, 0xD7, 0x01, 0xE6,
    0x01, 0x3C, 0xCD, 0x88, 0x01, 0xC3, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xC3, 0x50, 0x01, 0xCE, 0xED, 0x66, 0x66,
    0xCC, 0x0D, 0x00, 0x0B, 0x03, 0x73, 0x00, 0x83, 0x00, 0x0C, 0x00, 0x0D,
    0x00, 0x08, 0x11, 0x1F, 0x88, 0x89, 0x00, 0x0E, 0xDC, 0xCC, 0x6E, 0xE6,
    0xDD, 0xDD, 0xD9, 0x99, 0xBB, 0xBB, 0x67, 0x63, 0x6E, 0x0E, 0xEC, 0xCC,
    0xDD, 0xDC, 0x99, 0x9F, 0xBB, 0xB9, 0x33, 0x3E, 0x31, 0x30, 0x20, 0x50,
    0x52, 0x49, 0x4E, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x02, 0xD6, 0x88, 0xB6,
    0x3E, 0xE4, 0xE0, 0x47, 0x21, 0x26, 0xFF, 0xCB, 0xBE, 0x21, 0x00, 0x00,
    0x01, 0x20, 0x00, 0x11, 0x10, 0x80, 0xCD, 0xF0, 0x01, 0x11, 0x00, 0x98,
    0xCD, 0xFF, 0x01, 0x3E, 0x01, 0xEA, 0x03, 0xC0, 0x3E, 0x69, 0xEA, 0x02,
    0xC0, 0x11, 0x00, 0x98, 0x01, 0x00, 0x04, 0x3E, 0x00, 0xCD, 0xE5, 0x01,
    0x3E, 0x2A, 0xEA, 0x04, 0xC0, 0xC3, 0x20, 0x00, 0xD5, 0xE5, 0xC5, 0xF5,
    0xFA, 0x00, 0xC0, 0x6F, 0xFA, 0x01, 0xC0, 0x67, 0xFA, 0x03, 0xC0, 0x57,
    0xFA, 0x02, 0xC0, 0x5F, 0x1B, 0x1C, 0x1D, 0xC2, 0xBE, 0x01, 0x14, 0x15,
    0xC2, 0xBE, 0x01, 0x11, 0x28, 0x00, 0xD5, 0xE5, 0x54, 0x5D, 0x01, 0x40,
    0x00, 0x3E, 0x00, 0xCD, 0xE5, 0x01, 0xE1, 0xD1, 0xF0, 0x42, 0xC6, 0x10,
    0xE0, 0x42, 0x7A, 0xEA, 0x03, 0xC0, 0x7B, 0xEA, 0x02, 0xC0, 0xF0, 0x44,
    0xFE, 0x90, 0x20, 0xFA, 0xF1, 0x22, 0x54, 0x5D, 0xCD, 0xFF, 0x01, 0xC1,
    0xE1, 0xD1, 0xC9, 0xFA, 0x04, 0xC0, 0xCB, 0x27, 0xD2, 0xE1, 0x01, 0xEE,
    0x1D, 0xEA, 0x04, 0xC0, 0xC9, 0xF0, 0x41, 0xE6, 0x02, 0x20, 0xFA, 0x12,
    0x62, 0x6B, 0x13, 0x0B, 0xF0, 0x41, 0xE6, 0x02, 0x20, 0xFA, 0x2A, 0x12,
    0x13, 0x0B, 0x79, 0xB0, 0x20, 0xF2, 0xC9, 0xF5, 0xE5, 0x3E, 0x14, 0xBB,
    0x28, 0x09, 0xFE, 0xF4, 0x28, 0x18, 0xC6, 0x20, 0xC3, 0x03, 0x02, 0xC6,
    0x0B, 0x5F, 0x13, 0x3E, 0x9C, 0xBA, 0xC2, 0x22, 0x02, 0x3E, 0x00, 0xBB,
    0xC2, 0x22, 0x02, 0x11, 0x00, 0x98, 0x21, 0x00, 0xC0, 0x73, 0x23, 0x72,
    0xE1, 0xF1, 0xC9};

static uint8_t g_ram[RAM_SIZE];

// Pointer to the start of the current ROM memory bank.
static volatile uintptr_t s_rom_addr;

// DMA writes actual data to the first byte before we send a command to the SM.
static volatile uint32_t s_cmd_rom_out = DATA_CMD_OUT;
static volatile uint32_t s_cmd_ram_out = DATA_CMD_OUT;
static volatile uint32_t s_cmd_in = DATA_CMD_IN;

static void InitDma(uint addr_dreq, const volatile void *addr_fifo,
                    volatile void *data_fifo) {
  dma_claim_mask((1 << DMA_CH_RST_ROM_ADDR) | (1 << DMA_CH_ADDR_OFFSET) |
                 (1 << DMA_CH_FLASH_TO_RAM) | (1 << DMA_CH_DATA_CMD));
  dma_channel_config c;

  // Step 1: Reset `DMA_CH_FLASH_TO_RAM` read register to start of ROM memory
  // bank.
  s_rom_addr = (uintptr_t)xip_nocache_noalloc_alias_untyped(&g_rom);
  c = dma_channel_get_default_config(DMA_CH_RST_ROM_ADDR);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, false);
  channel_config_set_chain_to(&c, DMA_CH_ADDR_OFFSET);
  dma_channel_configure(DMA_CH_RST_ROM_ADDR, &c,
                        &dma_hw->ch[DMA_CH_FLASH_TO_RAM].read_addr, &s_rom_addr,
                        1, false);

  // Step 2: Get the ROM address from the `read_addr` SM and use it as offset
  //         into the current ROM bank. Directly write the offset to the read
  //         register of the `DMA_CH_FLASH_TO_RAM` dma channel. Use the atomic
  //         set alias memory address to add the offset without overwriting.
  c = dma_channel_get_default_config(DMA_CH_ADDR_OFFSET);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, false);
  channel_config_set_dreq(&c, addr_dreq);
  // Chaining cannot be used here because it starts the next DMA before its
  // the read register received the update. Use the trigger feature instead
  // to start step 3 with the write to the read register.
  dma_channel_configure(
      DMA_CH_ADDR_OFFSET, &c,
      hw_set_alias(&dma_hw->ch[DMA_CH_FLASH_TO_RAM].al3_read_addr_trig),
      addr_fifo, 1, false);

  // Step 3: Copy from ROM to temporary memory location.
  //         This takes ~50 cycles because it accesses flash and makes it the
  //         slowest operation of a bus access cycle.
  c = dma_channel_get_default_config(DMA_CH_FLASH_TO_RAM);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, false);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_chain_to(&c, DMA_CH_DATA_CMD);
  dma_channel_configure(DMA_CH_FLASH_TO_RAM, &c, &s_cmd_rom_out,
                        NULL,  // Calculated in step 1 and 2.
                        1, false);

  // Step 4: Transfer the selected command to the `data_out_in` SM.
  c = dma_channel_get_default_config(DMA_CH_DATA_CMD);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, false);
  channel_config_set_chain_to(&c, DMA_CH_RST_ROM_ADDR);
  dma_channel_configure(DMA_CH_DATA_CMD, &c, data_fifo,
                        NULL,  // Updated by address decoder.
                        1, false);

  dma_channel_start(DMA_CH_RST_ROM_ADDR);
}

static void InitCartridgeInterface(PIO pio, uint pins) {
  uint offset;
  pio_sm_config c;

  uint control_pins = pins + CARTRIDGE_BITS;

  // Configure GPIOs
  uint temp_sm = pio_claim_unused_sm(pio, true);
  pio_sm_set_pins(pio, temp_sm, ADDR_HI << control_pins);
  pio_sm_set_consecutive_pindirs(pio, temp_sm, pins, CARTRIDGE_BITS, false);
  pio_sm_set_consecutive_pindirs(pio, temp_sm, control_pins, CONTROL_BITS,
                                 true);
  pio_sm_set_pins(pio, temp_sm, ADDR_HI << control_pins);
  for (int i = 0; i < CARTRIDGE_BITS; i++) {
    pio_gpio_init(pio, pins + i);
  }
  for (int i = 0; i < CONTROL_BITS; i++) {
    pio_gpio_init(pio, control_pins + i);
  }
  pio_sm_unclaim(pio, temp_sm);

  // Disable the synchronizer circuit for all cartridge signals.
  // Makes waiting for CLK edges 2 cycles faster.
  pio->input_sync_bypass |= (0x0FFF << pins);

  // Configure the `read_addr` state machine
  pio_sm_claim(pio, CARTRIDGE_SM_ADDR);
  offset = pio_add_program(pio, &read_addr_program);
  c = read_addr_program_get_default_config(offset);
  sm_config_set_in_pins(&c, pins);
  sm_config_set_set_pins(&c, control_pins, CONTROL_BITS);
  sm_config_set_in_shift(&c, false,  // shift_direction=left
                         true, 15);  // autopush enabled
  pio_sm_init(pio, CARTRIDGE_SM_ADDR, offset, &c);

  // Configure the `data_out_in` state machine
  pio_sm_claim(pio, CARTRIDGE_SM_DATA);
  offset = pio_add_program(pio, &data_out_in_program);
  c = data_out_in_program_get_default_config(offset);
  sm_config_set_in_pins(&c, pins);
  sm_config_set_out_pins(&c, pins, CARTRIDGE_BITS + CONTROL_BITS);
  sm_config_set_out_shift(&c, true,    // shift_direction=right
                          false, 32);  // autopull disabled
  sm_config_set_in_shift(&c, false,    // shift_direction=left
                         true, 8);     // autopush enabled
  pio_sm_init(pio, CARTRIDGE_SM_DATA, offset, &c);
  pio_sm_put(pio, CARTRIDGE_SM_DATA, DATA_CMD_RELEASE);

  // DMA chain to connect the two state machines
  InitDma(pio_get_dreq(pio, CARTRIDGE_SM_ADDR, false),
          &pio->rxf[CARTRIDGE_SM_ADDR], &pio->txf[CARTRIDGE_SM_DATA]);

  pio_set_sm_mask_enabled(
      pio, (1 << CARTRIDGE_SM_ADDR) | (1 << CARTRIDGE_SM_DATA), true);
}

static void __attribute__((optimize("O3")))
__no_inline_not_in_flash_func(RunAddressDecoder)() {
  while (true) {
    // Wait until`read_addr` SM is ready to give us the address.
    while (!pio_interrupt_get(CARTRIDGE_PIO, 0))
      ;
    pio_interrupt_clear(CARTRIDGE_PIO, 0);

    uint32_t signals = pio_sm_get_blocking(CARTRIDGE_PIO, CARTRIDGE_SM_ADDR);

    uint32_t action = signals >> 16;
    uint32_t addr = signals & 0xFFFF;
    if (action == RAM_READ && addr & 0xA000) {
      // A15 and A13 as secondary high CS.
      // Only update the data portion (lowest byte) of the command.
      *(volatile uint8_t *)&s_cmd_ram_out = g_ram[addr & 0x0FFF];
      dma_channel_set_read_addr(DMA_CH_DATA_CMD, &s_cmd_ram_out, false);
    } else if (action == ROM_READ && addr < 0x8000) {  // A15 as chip select
      dma_channel_set_read_addr(DMA_CH_DATA_CMD, &s_cmd_rom_out, false);
    } else {
      // idle or write
      dma_channel_set_read_addr(DMA_CH_DATA_CMD, &s_cmd_in, false);
    }

    uint8_t data = pio_sm_get_blocking(CARTRIDGE_PIO, CARTRIDGE_SM_DATA);
    if (action == RAM_WRITE && addr & 0xA000) {
      // A15 and A13 as secondary high CS.
      g_ram[addr & 0x0FFF] = data;
    } else if (action == MBC_WRITE && addr < 0x8000 && addr & 0x4000) {
      // A15 as primary and A14 as secondary high CS.
      // TODO
    }
  }
}

int main() {
  // TODO: remove
  // Run system clock at 1MHz for easy cycle counting with a logic analyzer.
  clock_configure(clk_ref, CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC, 0,
                  XOSC_MHZ * MHZ, XOSC_MHZ * MHZ);
  clock_configure(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF, 0,
                  XOSC_MHZ * MHZ, 1 * MHZ);

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  gpio_put(PICO_DEFAULT_LED_PIN, 1);

  InitCartridgeInterface(CARTRIDGE_PIO, PIN_BASE);

  // Make sure that no cade runs from flash memory anymore to guarantee for
  // constant flash access times.
  RunAddressDecoder();

  return 0;
}
