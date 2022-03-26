#include "rom.h"

// https://github.com/svendahlstrand/10-print-game-boy
const uint8_t g_rom[] ROM_ATTRIBUTES = {
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
