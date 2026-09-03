/**
 * @file neopixel.h
 * @brief NeoPixel 3x3 LED Grid Module for Athera
 *
 * Controls a 3x3 grid of NeoPixel LEDs (9 LEDs in series). Drives both
 * module revisions transparently: the original RGB (WS2812B) module wired
 * on Line A, and the newer RGBW (SK6812) module wired on Line B. Color
 * data is written to both lines every frame; whichever module is plugged
 * in latches the signal on its own line and ignores the other.
 *
 * Brightness is clamped to MAX_BRIGHTNESS (20% of the raw 0-255 range) to
 * stop the newer RGBW module from being painfully bright.
 *
 * Grid layout (index):
 *   0  1  2
 *   3  4  5
 *   6  7  8
 *
 * Athera Connector:
 * - Pin 1: 5V (LED power)
 * - Pin 4: GND
 * - Pin 5: Data (Line A) — RGB module
 * - Pin 6: Data (Line B) — RGBW module
 */

#ifndef NEOPIXEL_MODULE_H
#define NEOPIXEL_MODULE_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

class NeoPixelGrid {
public:
    static const uint8_t GRID_SIZE = 3;
    static const uint8_t NUM_LEDS = 9;
    static const uint8_t MAX_BRIGHTNESS = 51;  // 20% of 255

    /**
     * @brief Construct NeoPixel grid controller
     * @param dataPinA GPIO for Line A (RGB module)
     * @param dataPinB GPIO for Line B (RGBW module)
     * @param numLeds Number of LEDs (default 9 for 3x3)
     */
    NeoPixelGrid(int dataPinA, int dataPinB, uint8_t numLeds = NUM_LEDS);

    /**
     * @brief Initialize the LED strip
     */
    void begin();

    /**
     * @brief Set a single pixel by index (0-8)
     * @param index Pixel index
     * @param r Red (0-255)
     * @param g Green (0-255)
     * @param b Blue (0-255)
     */
    void setPixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Set a single pixel by grid position
     * @param row Row (0-2)
     * @param col Column (0-2)
     * @param r Red (0-255)
     * @param g Green (0-255)
     * @param b Blue (0-255)
     */
    void setPixelXY(uint8_t row, uint8_t col, uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Set all pixels to the same color
     * @param r Red (0-255)
     * @param g Green (0-255)
     * @param b Blue (0-255)
     */
    void setAll(uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Set an entire row to a color
     * @param row Row index (0-2)
     * @param r Red (0-255)
     * @param g Green (0-255)
     * @param b Blue (0-255)
     */
    void setRow(uint8_t row, uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Set an entire column to a color
     * @param col Column index (0-2)
     * @param r Red (0-255)
     * @param g Green (0-255)
     * @param b Blue (0-255)
     */
    void setColumn(uint8_t col, uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Turn off all pixels
     */
    void clear();

    /**
     * @brief Update the physical LEDs with current buffer
     */
    void show();

    /**
     * @brief Set global brightness. Values above MAX_BRIGHTNESS (51 = 20%
     *        of 255) are clamped to protect against the RGBW module's high
     *        output.
     * @param brightness 0-255 (clamped to MAX_BRIGHTNESS)
     */
    void setBrightness(uint8_t brightness);

    /**
     * @brief Get current brightness
     */
    uint8_t getBrightness() const;

    /**
     * @brief Get a pixel's color as packed 32-bit (0x00RRGGBB)
     * @param index Pixel index
     * @return Packed color, or 0 if index out of range
     */
    uint32_t getPixelColor(uint8_t index) const;

    /**
     * @brief Convert row/col to linear index
     */
    uint8_t xyToIndex(uint8_t row, uint8_t col) const {
        return row * GRID_SIZE + col;
    }

private:
    Adafruit_NeoPixel _stripA;  // RGB, Line A (old module)
    Adafruit_NeoPixel _stripB;  // RGBW, Line B (new module)
    int _dataPinA;
    int _dataPinB;
    uint8_t _numLeds;
};

#endif // NEOPIXEL_MODULE_H
