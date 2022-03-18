// TODO: License Header

#if !PICO_NO_FLASH && !PICO_COPY_TO_RAM
#error "code must execute from RAM for predictable flash access times"
#endif

#include <assert.h>
#include <stdint.h>

#include "hardware/dma.h"
#include "pico/stdlib.h"

#define ROM_SIZE (32 * 1024)

// TODO: Move to QSPI flash
extern uint8_t g_rom[ROM_SIZE] __attribute__((aligned(ROM_SIZE)));

static uintptr_t g_rom_addr = 0;

int main() {
  stdio_init_all();

  g_rom_addr = (uintptr_t)&g_rom;
  assert((g_rom_addr % ROM_SIZE) == 0 &&
         "GB ROM memory is not aligned properly");

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
  channel_config_set_dreq(&c, -1);  // TODO: select PIO signal
  dma_channel_configure(dma_fetch_addr, &c,
                        &dma_hw->ch[dma_data_out].al1_read_addr,
                        NULL,  // TODO: PIO fifo as source
                        1, true);

  // Step 2: Add the `g_rom` symbol offset to the address by writing to the
  //         atomic set alias
  c = dma_channel_get_default_config(dma_addr_offset);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, false);
  channel_config_set_chain_to(&c, dma_data_out);
  dma_channel_configure(
      dma_addr_offset, &c,
      hw_set_alias_untyped(&dma_hw->ch[dma_data_out].al1_read_addr),
      &g_rom_addr, 1, true);

  // Step 3: Transfer ROM byte to the ROM data PIO FIFO
  c = dma_channel_get_default_config(dma_data_out);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, false);
  dma_channel_configure(dma_data_out, &c,
                        NULL,  // TODO: PIO fifo as target
                        NULL,  // The source address is updated in step 1 and 2.
                        1, true);

  return 0;
}
