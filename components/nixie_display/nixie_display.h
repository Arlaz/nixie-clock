#pragma once

#include <esp_attr.h>
#include <stdbool.h>
#include <stdint.h>

#define NIXIE_ON false
#define NIXIE_OFF true

#define NUM_SR 3
#define NUM_CHANNELS 32
#define NUM_PINS (NUM_SR * NUM_CHANNELS)

#define NUM_TUBES 8
#define NUM_DIGITS 10
#define NUM_DOTS_ROW 2
#define NUM_DOTS_COL 7
#define NUM_DOTS (NUM_DOTS_COL * NUM_DOTS_ROW)

typedef struct DIGIT {
    uint32_t uptime;  // uptime of the digit
    uint_fast8_t pin;
} digit_t;

typedef struct TUBE {
    digit_t* shown;
    uint32_t uptime;  // uptime when at least one digit of the tube was on
    digit_t digits[NUM_DIGITS];
} tube_t;

typedef struct DOT {
    bool status;
    uint_fast8_t pin;
} dot_t;

typedef struct NIXIE_CLOCK {
    bool clock_status;
    uint_fast8_t brightness;
    tube_t tubes[NUM_TUBES];
    dot_t dots[NUM_DOTS_COL][NUM_DOTS_ROW];
    enum MODE {
        idle,
        hour,
        countdown,
    } mode;
} nixie_clock_t;

esp_err_t init_display();
