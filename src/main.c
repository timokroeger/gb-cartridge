// Copyright 2022 Timo Kröger <timokroeger93@gmail.com>

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "cartridge_interface.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
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

#define DMA_CH_ADDR_OFFSET 0
#define DMA_CH_FLASH 1

static uint8_t g_rom_bank0[ROM_BANK_SIZE];
static uint8_t g_ram[RAM_SIZE];

// Mirrors ROM bank 0 from flash to RAM.
static void MirrorBank0(void) {
  uint dma_ch_memcpy = dma_claim_unused_channel(true);
  dma_channel_config c = dma_channel_get_default_config(dma_ch_memcpy);
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, true);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  dma_channel_configure(dma_ch_memcpy, &c, g_rom_bank0, g_rom,
                        ROM_BANK_SIZE / 4, true);
  dma_channel_wait_for_finish_blocking(dma_ch_memcpy);
  dma_channel_unclaim(dma_ch_memcpy);
}

static void InitDma(uint addr_dreq, const volatile void *addr_fifo,
                    volatile void *data_fifo) {
  dma_channel_config c;

  // Step 1: Get the ROM address from the `read_addr` SM and use it as offset
  //         into the current ROM bank. Directly write the offset to the read
  //         register of the `DMA_CH_FLASH` dma channel. Use the atomic
  //         set alias memory address to add the offset without overwriting.
  dma_channel_claim(DMA_CH_ADDR_OFFSET);
  c = dma_channel_get_default_config(DMA_CH_ADDR_OFFSET);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, false);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_dreq(&c, addr_dreq);
  // Chaining cannot be used here because it starts the next DMA before its
  // read register received the update. Use the trigger feature instead to
  // start step 2 with the write to the read register.
  dma_channel_configure(
      DMA_CH_ADDR_OFFSET, &c,
      hw_set_alias(&dma_hw->ch[DMA_CH_FLASH].al3_read_addr_trig), addr_fifo, 1,
      false);

  // Step 2: Copy from flash to `data_out` SM. This takes ~50 cycles and is the
  //         slowest operation of a bus access cycle.
  dma_channel_claim(DMA_CH_FLASH);
  c = dma_channel_get_default_config(DMA_CH_FLASH);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, false);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  dma_channel_configure(DMA_CH_FLASH, &c, data_fifo,
                        NULL,  // Reset to beginning of current ROM bank by the
                               // CPU and updated by DMA Step 1.
                        1, false);
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
  sm_config_set_in_shift(&c, false,  // shift_direction=left
                         true, 14);  // autopush enabled
  pio_sm_init(pio, SM_ADDR, offset, &c);

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

  MirrorBank0();
}

static inline void StartCartridgeInterface(PIO pio) {
  pio_set_sm_mask_enabled(
      pio, (1 << SM_ADDR) | (1 << SM_DATA_OUT) | (1 << SM_DATA_IN), true);
}

__noinline __scratch_x("main") static void MainCore1() {
  StartCartridgeInterface(PIO_CARTRIDGE);

  while (true) {
    // Synchronize to the bus clock.
    while (!pio_interrupt_get(PIO_CARTRIDGE, IRQ_START))
      ;
    pio_interrupt_clear(PIO_CARTRIDGE, IRQ_START);

    // The flash DMA writes data even for idle cycles.
    // Clear the FIFO to discard this data.
    pio_sm_clear_fifos(PIO_CARTRIDGE, SM_DATA_OUT);

    // Reset the flash to ram DMA chanel read register to the start of the
    // current ROM memory bank and trigger the DMA chain to read from flash in
    // background as soon as a new address is available on the bus.
    dma_channel_set_read_addr(
        DMA_CH_FLASH, xip_nocache_noalloc_alias(&g_rom[ROM_BANK_SIZE]), false);
    dma_channel_set_trans_count(DMA_CH_ADDR_OFFSET, 1, true);

    // Wait until the `read_addr` SM is ready to give us the address.
    // We use the IRQ as additional barrier so that we do not accidentally
    // "steal" the FIFO entry from the DMA.
    while (!pio_interrupt_get(PIO_CARTRIDGE, IRQ_ADDR))
      ;
    pio_interrupt_clear(PIO_CARTRIDGE, IRQ_ADDR);

    uint32_t signals = pio_sm_get_blocking(PIO_CARTRIDGE, SM_ADDR);

    uint32_t action = signals >> 16;
    uint32_t addr = signals & 0xFFFF;

    if (action == ROM_READ && addr < 0x8000) {  // A15 as low CS.
      if (addr < 0x4000) {                      // Bank 0
        // We race the flash DMA and put data into the FIFO first.
        // DMA will still write to the FIFO but the SM ignores that and we
        // clear it from the FIFO at the beginning of the next cycle.
        pio_sm_put(PIO_CARTRIDGE, SM_DATA_OUT, g_rom_bank0[addr]);
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

__noinline __scratch_y("main") static void MainCore0() {
  // We are now running from RAM so that the cartridge interface has exclusive
  // access to flash. Because we rely on constant flash access time there is no
  // need to keep the XIP cache so lets disable it.
  xip_ctrl_hw->ctrl = 0;

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
