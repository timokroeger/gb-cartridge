// Copyright 2022 Timo Kröger <timokroeger93@gmail.com>

#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include "simulation.pio.h"

#define TARGET_CLK_HZ (1048576u / 125)
#define CLK_SM_DIV 125000000u / (TARGET_CLK_HZ * 8 * 2)

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
                           uint oe_addr_hi, uint oe_addr_lo, uint oe_data) {
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
  sm_config_set_in_shift(&c, false, true, 8);
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
  pio_sm_put(sim->pio_clk, sim->sm_clk,
             (addr & 0x8000) ? SIM_RAM_RD : SIM_ROM_RD);
  pio_sm_put(sim->pio_mux, sim->sm_addr_hi, addr >> 8);
  pio_sm_put(sim->pio_mux, sim->sm_addr_lo, addr);
  pio_sm_put(sim->pio_mux, sim->sm_data, 0xFFFFFFFF);
  return pio_sm_get_blocking(sim->pio_clk, sim->sm_clk);
}

static void SimulationWrite(const Simulation *sim, uint16_t addr,
                            uint8_t data) {
  pio_sm_put(sim->pio_clk, sim->sm_clk,
             (addr & 0x8000) ? SIM_RAM_WR : SIM_MBC_WR);
  pio_sm_put(sim->pio_mux, sim->sm_addr_hi, addr >> 8);
  pio_sm_put(sim->pio_mux, sim->sm_addr_lo, addr);
  pio_sm_put(sim->pio_mux, sim->sm_data, data);
  pio_sm_get_blocking(sim->pio_clk, sim->sm_clk);
}

int main() {
  Simulation sim;
  SimulationInit(&sim, pio0, 8, pio1, 0, 15, 14, 13, 12);

  while (true) {
    SimulationRead(&sim, 0x5555);
    SimulationWrite(&sim, 0xAAAA, 0x33);
  }

  return 0;
}
