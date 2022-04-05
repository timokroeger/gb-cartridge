// Copyright 2022 Timo Kröger <timokroeger93@gmail.com>

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "cartridge_interface.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/structs/ssi.h"
#include "hardware/structs/xip_ctrl.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "rom.h"

#define RAM_SIZE (8 * 1024)

// Pin Base:
// 0 .. 7: D0-D7, A0-A7, A8-A15 (multiplexed)
// 8 ..11: A15, #CS, #RD, CLK
// 12..15: OE0-2, DIR0
#define PIN_BASE 0

#define PIO_CARTRIDGE pio0
#define SM_ADDR 0
#define SM_DATA_OUT 1
#define SM_DATA_IN 2

#define DMA_CH_FLASH_ADDR 0
#define DMA_CH_FLASH 1

#define ROM_BANK0 ((uint8_t *)XIP_SRAM_BASE)

static uint8_t g_ram[RAM_SIZE];

static void InitDma(uint addr_dreq, const volatile void *addr_fifo,
                    volatile void *data_fifo) {
  dma_channel_config c;

  // Get the flash address from the `read_addr` SM and wite it to the SSI FIFO
  // to request a flash memory read.
  dma_channel_claim(DMA_CH_FLASH_ADDR);
  c = dma_channel_get_default_config(DMA_CH_FLASH_ADDR);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, false);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_dreq(&c, addr_dreq);
  channel_config_set_chain_to(&c, DMA_CH_FLASH);
  dma_channel_configure(DMA_CH_FLASH_ADDR, &c, &ssi_hw->dr0, addr_fifo, 1,
                        false);

  // Get the result from the SSI read after ~40 cycles.
  dma_channel_claim(DMA_CH_FLASH);
  c = dma_channel_get_default_config(DMA_CH_FLASH);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, false);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_dreq(&c, DREQ_XIP_SSIRX);
  dma_channel_configure(DMA_CH_FLASH, &c, data_fifo, &ssi_hw->dr0, 1, false);

  bus_ctrl_hw->priority =
      BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;
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
  pio->input_sync_bypass |= (((1 << CARTRIDGE_BITS) - 1) << pins);

  // Configure the `read_addr` state machine
  pio_sm_claim(pio, SM_ADDR);
  offset = pio_add_program(pio, &read_addr_program);
  c = read_addr_program_get_default_config(offset);
  sm_config_set_set_pins(&c, control_pins, CONTROL_BITS);
  sm_config_set_in_pins(&c, pins);
  sm_config_set_in_shift(&c, false,    // shift_direction=left
                         true, 32);    // autopush enabled
  sm_config_set_out_shift(&c, true,    // shift_direction=right
                          false, 32);  // autopull enabled
  pio_sm_init(pio, SM_ADDR, offset, &c);

  // Init flash offset to bank 1 in ROM.
  uint32_t rom_bank1_offset = (uint32_t)&g_rom[0] - XIP_BASE;
  pio_sm_put(pio, SM_ADDR,
             (rom_bank1_offset >> 14) |  // Flash offset
                 (0xA0 << 10));          // QSPI mode continuation bits

  // Configure the `data_out` state machine
  pio_sm_claim(pio, SM_DATA_OUT);
  offset = pio_add_program(pio, &data_out_program);
  c = data_out_program_get_default_config(offset);
  sm_config_set_sideset_pins(&c, control_pins);
  sm_config_set_out_pins(&c, pins, 8);
  sm_config_set_out_shift(&c, true,    // shift_direction=right
                          false, 32);  // autopull disabled
  pio_sm_init(pio, SM_DATA_OUT, offset, &c);

  // Configure the `data_in` state machine
  pio_sm_claim(pio, SM_DATA_IN);
  offset = pio_add_program(pio, &data_in_program);
  c = data_in_program_get_default_config(offset);
  sm_config_set_set_pins(&c, control_pins, CONTROL_BITS);
  sm_config_set_in_pins(&c, pins);
  sm_config_set_in_shift(&c, false,  // shift_direction=left
                         true, 8);   // autopush enabled
  pio_sm_init(pio, SM_DATA_IN, offset, &c);

  // DMA chain to connect the two state machines
  InitDma(pio_get_dreq(pio, SM_ADDR, false), &pio->rxf[SM_ADDR],
          &pio->txf[SM_DATA_OUT]);
}

static inline void StartCartridgeInterface(PIO pio) {
  pio_set_sm_mask_enabled(
      pio, (1 << SM_ADDR) | (1 << SM_DATA_OUT) | (1 << SM_DATA_IN), true);
}

__noinline __scratch_x("main") static void MainCore1(void) {
  // To guarantue constant flash access times for the cartridge interface we
  // must not run any code from flash anymore. Disable the unused XIP cache and
  // repurpose it as additional SRAM to mirror ROM bank 0.
  xip_ctrl_hw->ctrl = 0;
  memcpy(ROM_BANK0, g_rom, ROM_BANK_SIZE);

  // Read only 8bits (instead of 32bit) with each serial transfer.
  // Make sure that no code runs from flash anymore or it will crash now.
  // This reduces flash access times by 12 cycles when only the least
  // significant byte is required.
  // Leave all other settings as configured by boot2.
  ssi_hw->ssienr = 0;
  hw_write_masked(&ssi_hw->ctrlr0, (7 << SSI_CTRLR0_DFS_32_LSB),
                  SSI_CTRLR0_DFS_32_BITS);
  ssi_hw->dmacr = SSI_DMACR_RDMAE_BITS;
  ssi_hw->ssienr = 1;

  StartCartridgeInterface(PIO_CARTRIDGE);

  while (true) {
    // TODO: check for possible delay in read after write

    // Synchronize to the bus clock.
    // TODO: wfi to reduce jitter
    pio_interrupt_clear(PIO_CARTRIDGE, IRQ_START);
    while (!pio_interrupt_get(PIO_CARTRIDGE, IRQ_START))
      ;

    // The flash DMA writes data even for RAM acces, write and idle cycles.
    // Clear the FIFO to discard remaining data from previous cycles.
    pio_sm_clear_fifos(PIO_CARTRIDGE, SM_DATA_OUT);

    // Trigger the DMA chain to read from flash in background as soon as a new
    // address is available on the bus.
    dma_channel_set_trans_count(DMA_CH_FLASH_ADDR, 1, true);

    // Wait until the `read_addr` SM is ready to give us the address.
    // We use the IRQ as additional barrier so that we do not accidentally
    // "steal" the FIFO entry from the DMA.
    // TODO: wfi to reduce jitter
    pio_interrupt_clear(PIO_CARTRIDGE, IRQ_ADDR);
    while (!pio_interrupt_get(PIO_CARTRIDGE, IRQ_ADDR))
      ;
    uint32_t signals = pio_sm_get(PIO_CARTRIDGE, SM_ADDR);

    uint32_t action = signals >> 16;
    uint32_t addr = signals & 0xFFFF;

    if (action == ROM_READ && addr < 0x8000) {  // A15 as low CS.
      if (addr < 0x4000) {                      // Bank 0
        // We race the flash DMA and put data into the FIFO first.
        // DMA will still write to the FIFO but the SM ignores that and we
        // clear it from the FIFO at the beginning of the next cycle.
        pio_sm_put(PIO_CARTRIDGE, SM_DATA_OUT, ROM_BANK0[addr]);
      } else {  // Bank 1..n
        // No action required, all handled by the flash DMA.
      }
      PIO_CARTRIDGE->irq_force = (1 << IRQ_DATA_OUT);
    } else if (action == RAM_READ &&
               addr & 0x2000) {  // A13 as secondary high CS.
      // Same principle as with ROM bank 0 acces, we are faster than the flash
      // DMA.
      pio_sm_put(PIO_CARTRIDGE, SM_DATA_OUT, g_ram[addr & 0x0FFF]);
      PIO_CARTRIDGE->irq_force = (1 << IRQ_DATA_OUT);
    } else if (action == RAM_WRITE &&
               addr & 0x2000) {  // A13 as secondary high CS.
      PIO_CARTRIDGE->irq_force = (1 << IRQ_DATA_IN);
      uint8_t data = pio_sm_get_blocking(PIO_CARTRIDGE, SM_DATA_IN);
      g_ram[addr & 0x0FFF] = data;
    } else if (action == MBC_WRITE &&
               // A15 as primary and A14 as secondary high CS.
               addr < 0x8000 && addr & 0x4000) {
      PIO_CARTRIDGE->irq_force = (1 << IRQ_DATA_IN);
      uint8_t data = pio_sm_get_blocking(PIO_CARTRIDGE, SM_DATA_IN);
      (void)data;
      // TODO
    }

    // TODO: wfi
  }
}

__noinline __scratch_y("main") static void MainCore0(void) {
  // multicore_launch_core1(MainCore1);
  MainCore1();

  while (true) {
    __wfi();
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

  InitCartridgeInterface(PIO_CARTRIDGE, PIN_BASE);
  MainCore0();

  return 0;
}
