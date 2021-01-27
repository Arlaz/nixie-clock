#pragma once

/**
 * @brief turn on/off the lowlevel leds
 */
esp_err_t leds_set_on(bool value);

/**
 * @brief set the brightness of the lowlevel leds
 */
esp_err_t leds_set_brightness(int value);

/**
 * @brief set the hue of the lowlevel leds
 */
esp_err_t leds_set_hue(float value);

/**
 * @brief set the saturation of the lowlevel leds
 */
esp_err_t leds_set_saturation(float value);

/**
 * Must be called before any other function
 */
void init_leds();
