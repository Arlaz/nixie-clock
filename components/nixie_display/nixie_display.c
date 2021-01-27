#include <esp_err.h>
#include <esp_event.h>
#include <esp_log.h>
#include <shift_registers.h>
#include <wiring.h>

#include "nixie_display.h"

static const char* TAG = "Nixie Display";

static DMA_ATTR sr_handle_t hv5530_registers;
static DMA_ATTR uint32_t sr_data[NUM_SR];
static DMA_ATTR uint32_t tubes_mask[NUM_TUBES][NUM_SR];
static DMA_ATTR uint32_t dots_mask[NUM_SR];

static DMA_ATTR nixie_clock_t nixie;

/**
 * Mapping table to find the HV5530 pin number for a digit on a tube
 */
// TODO: modify (update values + start from 0)
const static DMA_ATTR uint_fast8_t pinTubeIndex[NUM_TUBES][NUM_DIGITS] = {
    {88, 90, 92, 95, 87, 89, 91, 93, 94, 85},  // Tube 1 (0 - 9) (most left tube)
    {81, 79, 77, 82, 74, 76, 78, 80, 75, 83},  // Tube 2 (0 - 9)
    {68, 66, 64, 61, 71, 69, 67, 65, 63, 70},  // Tube 3 (0 - 9)
    {58, 56, 53, 52, 55, 57, 51, 50, 54, 60},  // Tube 4 (0 - 9)
    {45, 43, 40, 41, 42, 44, 46, 39, 38, 47},  // Tube 5 (0 - 9)
    {32, 31, 30, 28, 35, 34, 25, 27, 29, 33},  // Tube 6 (0 - 9)
    {22, 19, 17, 14, 24, 20, 18, 16, 15, 21},  // Tube 7 (0 - 9)
    {8, 5, 6, 4, 9, 11, 13, 1, 3, 7}           // Tube 8 (0 - 9) (most right tube)
};

/**
 * Mapping table to find the HV5530 pin number for INS-1 tubes
 * pinDotIndex[x][0] for bottom dot
 * pinDotIndex[x][1] for top dot
 */
// TODO: modify (update values + start from 0)
const static DMA_ATTR uint_fast8_t pinDotIndex[NUM_DOTS_COL][NUM_DOTS_ROW] = {
    {86, 84},  // IND 1 (most left tube holder)
    {72, 73},  // IND 2
    {62, 59},  // IND 3
    {49, 48},  // IND 4
    {36, 37},  // IND 5
    {26, 23},  // IND 6
    {10, 12}   // IND 7 (most right tube holder)
};

void IRAM_ATTR reset_tube_display_buffer(uint_fast8_t tube_to_reset) {
    for (int i = 0; i < NUM_SR; i++) {
        sr_data[i] |= tubes_mask[tube_to_reset][i];
    }
}

void IRAM_ATTR reset_dots_display_buffer() {
    for (int i = 0; i < NUM_SR; i++) {
        sr_data[i] |= dots_mask[i];
    }
}

void IRAM_ATTR reset_clock_display_buffer() {
    for (int t = 0; t < NUM_TUBES; t++) {
        reset_tube_display_buffer(t);
    }
    reset_dots_display_buffer();
}

void IRAM_ATTR set_index_buffer(uint32_t* arr, uint_fast8_t i, bool val) {
    assert(i < NUM_PINS);
    uint32_t* loc = &arr[NUM_SR - 1 - (i / (8 * sizeof(uint32_t)))];
    uint32_t pos = __builtin_bswap32((uint32_t)(1 << (i % (8 * sizeof(uint32_t)))));
    if (val)
        *(loc) |= pos;
    else
        *(loc) &= ~pos;
}

void IRAM_ATTR set_tube(nixie_clock_t* clock, uint_fast8_t t) {
    reset_tube_display_buffer(t);
    set_index_buffer(sr_data, clock->tubes[t].shown->pin, NIXIE_ON);
}

void calc_masks(nixie_clock_t* clock) {
    for (int i = 0; i < NUM_TUBES; i++) {
        for (int j = 0; j < NUM_DIGITS; j++) {
            set_index_buffer(tubes_mask[i], clock->tubes[i].digits[j].pin, NIXIE_OFF);
        }
    }
    for (int c = 0; c < NUM_DOTS_COL; c++) {
        for (int r = 0; r < NUM_DOTS_ROW; r++) {
            set_index_buffer(dots_mask, clock->dots[c][r].pin, NIXIE_OFF);
        }
    }
}

void IRAM_ATTR refresh(nixie_clock_t* clock, bool* ttr) {
    for (int t = 0; t < NUM_TUBES; t++) {
        if (ttr[t]) {
            assert(clock->tubes[t].shown != NULL);
            set_tube(clock, t);
        }
    }
    reset_dots_display_buffer();
    for (int c = 0; c < NUM_DOTS_COL; c++) {
        for (int r = 0; r < NUM_DOTS_ROW; r++) {
            if (nixie.dots[c][r].status == NIXIE_ON) {
                set_index_buffer(sr_data, clock->dots[c][r].pin, NIXIE_ON);
            }
        }
    }
}

void IRAM_ATTR all_refresh_send(nixie_clock_t* clock) {
    bool ttr[] = {[0 ... NUM_TUBES] = true};
    refresh(clock, ttr);
    send_data(&hv5530_registers, sr_data);
}

void IRAM_ATTR refresh_send(nixie_clock_t* clock, bool* ttr) {
    refresh(clock, ttr);
    send_data(&hv5530_registers, sr_data);
}

/**
 * Test function : display 1..9 on all tubes
 * Keeps dots turned off
 *
 * @param clock
 */
void IRAM_ATTR display_loop(nixie_clock_t* clock) {
    uint_fast8_t digit = 0;
    while (true) {
        for (int t = 0; t < NUM_TUBES; t++) {
            clock->tubes[t].shown = &clock->tubes[t].digits[digit];
        }
        if (++digit >= NUM_DIGITS) {
            digit = 0;
        }
        all_refresh_send(clock);
        ESP_LOGI(TAG, "digit : %u send", digit);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void init_clock() {
    ESP_LOGI(TAG, "Initialize Clock characteristics");
    for (int t = 0; t < NUM_TUBES; t++) {
        for (int d = 0; d < NUM_DIGITS; d++) {
            nixie.tubes[t].digits[d].pin = pinTubeIndex[t][d];
        }
        nixie.tubes[t].shown = NULL;
    }
    for (int c = 0; c < NUM_DOTS_COL; c++) {
        for (int r = 0; r < NUM_DOTS_ROW; r++) {
            nixie.dots[c][r].status = NIXIE_OFF;
            nixie.dots[c][r].pin = pinDotIndex[c][r];
        }
    }
    calc_masks(&nixie);
}

esp_err_t init_display() {
    esp_err_t ret;

    hv5530_registers.num_channels = NUM_CHANNELS;
    hv5530_registers.cascade_size = NUM_SR;

    ret = init_shift_registers(&hv5530_registers, GPIO_HVDRIVER_DATA, GPIO_HVDRIVER_CLK, GPIO_HVDRIVER_LE);
    if (ret != ESP_OK) {
        return ret;
    }

    return xTaskCreatePinnedToCore((TaskFunction_t)display_loop, "NixieLoop", 2048, &nixie, 3, NULL, APP_CPU_NUM);
}
