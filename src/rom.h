// Copyright 2022 Timo Kröger <timokroeger93@gmail.com>

#ifndef ROM_H_
#define ROM_H_

#include "pico/platform.h"

#define ROM_BANK_SIZE 16 * 1024
#define ROM_ATTRIBUTES __in_flash() __aligned(ROM_BANK_SIZE)

extern const uint8_t g_rom[] ROM_ATTRIBUTES;

#endif
