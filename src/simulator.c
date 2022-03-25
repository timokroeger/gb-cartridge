// Copyright 2022 Timo Kröger <timokroeger93@gmail.com>

#define PICO_STDIO_ENABLE_CRLF_SUPPORT 0

#include <stdio.h>

#include "cobs.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "simulation.pio.h"

#define TARGET_CLK_HZ (1048576u / 125)
#define CLK_SM_DIV 125000000u / (TARGET_CLK_HZ * 12 * 2)

// output of 8 patters 4bit each
#define SIM_IDLE 0b11011101110111011100110011001100u
#define SIM_ROM_RD 0b11011101010101010100010001000100u
#define SIM_MBC_WR 0b11011111011101110110011001100110u
#define SIM_RAM_RD 0b11011101100110011000100010001000u
#define SIM_RAM_WR 0b11011111101110111010101010101010u

typedef struct {
  PIO pio_clk;
  uint sm_clk;

  PIO pio_mux;
  uint sm_addr_hi;
  uint sm_addr_lo;
  uint sm_data;
} Simulation;

static void SimulationInit(Simulation *sim, PIO pio_clk, uint pins_clk,
                           PIO pio_mux, uint pins_mux, uint dir_data,
                           uint oe_addr_lo, uint oe_addr_hi, uint oe_data) {
  assert(sim);

  pio_gpio_init(pio_clk, pins_clk);
  pio_gpio_init(pio_clk, pins_clk + 1);
  pio_gpio_init(pio_clk, pins_clk + 2);
  pio_gpio_init(pio_clk, pins_clk + 3);
  for (uint i = 0; i < 8; i++) {
    pio_gpio_init(pio_mux, pins_mux + i);
    gpio_set_pulls(pins_mux + i, true, false);
  }
  pio_gpio_init(pio_mux, dir_data);
  gpio_set_pulls(dir_data, true, false);
  pio_gpio_init(pio_mux, oe_addr_hi);
  gpio_set_pulls(oe_addr_hi, true, false);
  pio_gpio_init(pio_mux, oe_addr_lo);
  gpio_set_pulls(oe_addr_lo, true, false);
  pio_gpio_init(pio_mux, oe_data);
  gpio_set_pulls(oe_data, true, false);

  uint sm_clk = pio_claim_unused_sm(pio_clk, true);
  pio_sm_set_consecutive_pindirs(pio_clk, sm_clk, pins_clk, 4, true);
  uint offset = pio_add_program(pio_clk, &clk_program);
  pio_sm_config c = clk_program_get_default_config(offset);
  sm_config_set_out_pins(&c, pins_clk, 4);
  sm_config_set_out_shift(&c, false, false, 32);
  sm_config_set_in_pins(&c, pins_mux);
  sm_config_set_in_shift(&c, false, false, 32);
  sm_config_set_clkdiv_int_frac(&c, CLK_SM_DIV, 0);
  pio_sm_init(pio_clk, sm_clk, offset, &c);
  pio_sm_put(pio_clk, sm_clk, SIM_IDLE);
  pio_sm_set_enabled(pio_clk, sm_clk, true);

  sim->pio_clk = pio_clk;
  sim->sm_clk = sm_clk;

  uint sm_muxed_pindirs = pio_claim_unused_sm(pio_mux, true);
  offset = pio_add_program(pio_mux, &muxed_pindirs_program);
  c = muxed_pindirs_program_get_default_config(offset);
  sm_config_set_out_pins(&c, pins_mux, 8);
  sm_config_set_jmp_pin(&c, dir_data);
  pio_sm_init(pio_mux, sm_muxed_pindirs, offset, &c);
  pio_sm_set_enabled(pio_mux, sm_muxed_pindirs, true);

  offset = pio_add_program(pio_mux, &muxed_pins_program);

  uint sm_addr_hi = pio_claim_unused_sm(pio_mux, true);
  c = muxed_pins_program_get_default_config(offset);
  sm_config_set_out_pins(&c, pins_mux, 8);
  sm_config_set_jmp_pin(&c, oe_addr_hi);
  pio_sm_init(pio_mux, sm_addr_hi, offset, &c);
  pio_sm_set_enabled(pio_mux, sm_addr_hi, true);

  uint sm_addr_lo = pio_claim_unused_sm(pio_mux, true);
  c = muxed_pins_program_get_default_config(offset);
  sm_config_set_out_pins(&c, pins_mux, 8);
  sm_config_set_jmp_pin(&c, oe_addr_lo);
  pio_sm_init(pio_mux, sm_addr_lo, offset, &c);
  pio_sm_set_enabled(pio_mux, sm_addr_lo, true);

  uint sm_data = pio_claim_unused_sm(pio_mux, true);
  c = muxed_pins_program_get_default_config(offset);
  sm_config_set_out_pins(&c, pins_mux, 8);
  sm_config_set_jmp_pin(&c, oe_data);
  pio_sm_init(pio_mux, sm_data, offset, &c);
  pio_sm_set_enabled(pio_mux, sm_data, true);

  sim->pio_mux = pio_mux;
  sim->sm_addr_hi = sm_addr_hi;
  sim->sm_addr_lo = sm_addr_lo;
  sim->sm_data = sm_data;
}

static uint8_t SimulationRead(const Simulation *sim, uint16_t addr) {
  pio_sm_put_blocking(sim->pio_clk, sim->sm_clk,
                      (addr & 0x8000) ? SIM_RAM_RD : SIM_ROM_RD);
  pio_sm_put(sim->pio_mux, sim->sm_addr_hi, addr >> 8);
  pio_sm_put(sim->pio_mux, sim->sm_addr_lo, addr);
  pio_sm_put(sim->pio_mux, sim->sm_data, 0xFFFFFFFF);

  // Wait until the cycle has started.
  while (pio_sm_get_tx_fifo_level(sim->pio_clk, sim->sm_clk) != 0)
    ;

  // Throw away input from previous writes.
  pio_sm_clear_fifos(sim->pio_clk, sim->sm_clk);

  return pio_sm_get_blocking(sim->pio_clk, sim->sm_clk);
}

static void SimulationWrite(const Simulation *sim, uint16_t addr,
                            uint8_t data) {
  pio_sm_put_blocking(sim->pio_clk, sim->sm_clk,
                      (addr & 0x8000) ? SIM_RAM_WR : SIM_MBC_WR);
  pio_sm_put(sim->pio_mux, sim->sm_addr_hi, addr >> 8);
  pio_sm_put(sim->pio_mux, sim->sm_addr_lo, addr);
  pio_sm_put(sim->pio_mux, sim->sm_data, data);
}

int main() {
  stdio_init_all();

  Simulation sim;
  SimulationInit(&sim, pio0, 8, pio1, 0, 15, 14, 13, 12);

  uint8_t rx_buf[8];
  size_t rx_buf_idx = 0;

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  gpio_put(PICO_DEFAULT_LED_PIN, 1);

  while (true) {
    gpio_put(PICO_DEFAULT_LED_PIN, stdio_usb_connected());

    int c = getchar_timeout_us(10000);
    if (c < 0) {
      continue;
    }

    if (c != 0) {
      if (rx_buf_idx < sizeof(rx_buf)) {
        rx_buf[rx_buf_idx++] = c;
      }
      continue;
    }

    uint8_t cmd[4];
    cobs_decode_result dr = cobs_decode(cmd, sizeof(cmd), rx_buf, rx_buf_idx);
    rx_buf_idx = 0;
    if (dr.status != COBS_DECODE_OK || dr.out_len != 4) {
      continue;
    }

    uint16_t addr = cmd[0] | cmd[1] << 8;
    if (cmd[3] == 0) {
      uint8_t data = SimulationRead(&sim, addr);
      uint8_t tx_buf[2];
      cobs_encode_result er = cobs_encode(tx_buf, sizeof(tx_buf), &data, 1);
      for (size_t i = 0; i < er.out_len; i++) {
        putchar(tx_buf[i]);
      }
      putchar_raw(0);  // frame marker
    } else {
      uint8_t data = cmd[2];
      SimulationWrite(&sim, addr, data);
    }
  }

  return 0;
}
