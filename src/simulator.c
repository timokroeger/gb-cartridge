// TODO: License Header

#include "hardware/sync.h"
#include "pico/stdlib.h"
#include "simulation.pio.h"

int main() {
  Simulation sim;
  SimulationInit(&sim, pio1, 19, 15);
  SimulationStart(&sim);

  while (true) {
    __wfi();
  }

  return 0;
}
