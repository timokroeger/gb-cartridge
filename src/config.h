#ifndef CONFIG_H
#define CONFIG_H

// Just enough for simple games like Tetris or Dr. Mario.
#define ROM_SIZE (32 * 1024)

// Multiplexed cartridge signals: D0-D7, A0-A7, A8-A15.
#define MUXED_CARTRIDGE_SIGNALS_BASE 2

// OE and DIR of transceivers.
#define MUX_CONTROL_BASE 10

// Other cartridge signals: A15, #CS, CLK, #RD.
#define CARTRIDGE_SIGNALS_BASE 16

#endif // CONFIG_H
