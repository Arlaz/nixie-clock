#include <esp_err.h>
#include <esp_log.h>
#include <led_strip.h>
#include <wiring.h>

#include "leds_light.h"

typedef struct HSB {
    uint16_t h;  // 0-360
    uint16_t s;  // 0-100
    uint16_t b;  // 0-100
} hsb_t;

static led_strip_t strip = {
    .type = LED_STRIP_WS2812,
    .is_rgbw = false,
    .length = 8,
    .gpio = GPIO_LED,
    .channel = RMT_CHANNEL_2,
    .buf = NULL
};

static hsb_t hsb_val;
static uint16_t brightness;
static bool is_on = true;

static const char *TAG = "Leds lights";

/**
 * @brief Simple helper function, converting HSB color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
static esp_err_t leds_hsb2rgb(uint32_t h, uint32_t s, uint32_t b, rgb_t* color) {
    if (!color)
        return ESP_ERR_INVALID_ARG;
    if (s > 100)
        return ESP_ERR_INVALID_ARG;
    if (b > 100)
        return ESP_ERR_INVALID_ARG;

    h %= 360;  // h -> [0,360]
    uint32_t rgb_max = b * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        color->r = rgb_max;
        color->g = rgb_min + rgb_adj;
        color->b = rgb_min;
        break;
    case 1:
        color->r = rgb_max - rgb_adj;
        color->g = rgb_max;
        color->b = rgb_min;
        break;
    case 2:
        color->r = rgb_min;
        color->g = rgb_max;
        color->b = rgb_min + rgb_adj;
        break;
    case 3:
        color->r = rgb_min;
        color->g = rgb_max - rgb_adj;
        color->b = rgb_max;
        break;
    case 4:
        color->g = rgb_min;
        color->b = rgb_max;
        break;
    case 5:
        color->r = rgb_max;
        color->g = rgb_min;
        color->b = rgb_max - rgb_adj;
        break;
    default:
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * @brief set the leds' "HSV"
 */
static esp_err_t leds_set_aim_hsb(uint16_t h, uint16_t s, uint16_t b) {
    rgb_t color_rgb;
    esp_err_t ret = leds_hsb2rgb(h, s, b, &color_rgb);
    if (ret != ESP_OK)
        return ret;
    return led_strip_set_pixels(&strip, 0, strip.length, &color_rgb);
}

/**
 * @brief update the leds' states
 */
static esp_err_t leds_update() {
    esp_err_t ret = leds_set_aim_hsb(hsb_val.h, hsb_val.s, hsb_val.b);
    if (ret != ESP_OK)
        return ret;
    return led_strip_flush(&strip);
}

esp_err_t leds_set_on(bool value) {
    ESP_LOGI(TAG, "leds_set_on : %s", value == true ? "true" : "false");

    if (value == true) {
        hsb_val.b = brightness;
        is_on = true;
    } else {
        brightness = hsb_val.b;
        hsb_val.b = 0;
        is_on = false;
    }
    leds_update();

    return ESP_OK;
}

esp_err_t leds_set_brightness(int value) {
    ESP_LOGI(TAG, "leds_set_brightness : %d", value);

    hsb_val.b = value;
    brightness = hsb_val.b;
    if (is_on == true)
        leds_update();

    return ESP_OK;
}

esp_err_t leds_set_hue(float value) {
    ESP_LOGI(TAG, "leds_set_hue : %f", value);
    hsb_val.h = value;
    if (is_on == true)
        leds_update();
    return ESP_OK;
}

esp_err_t leds_set_saturation(float value) {
    ESP_LOGI(TAG, "leds_set_saturation : %f", value);
    hsb_val.s = value;
    if (is_on == true)
        leds_update();

    return ESP_OK;
}

void init_leds() {
    led_strip_install();
    ESP_ERROR_CHECK(led_strip_init(&strip));
}
