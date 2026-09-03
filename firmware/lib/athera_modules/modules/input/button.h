/**
 * @file button.h
 * @brief Single Button Input Module for Athera
 *
 * Simple digital button input with debouncing.
 * Active-low (pressed = LOW) with internal pull-up.
 *
 * Athera Connector:
 * - Pin 5: Button (Data Line A)
 * - Pin 4: GND
 */

#ifndef BUTTON_MODULE_H
#define BUTTON_MODULE_H

#include <Arduino.h>

class ButtonModule {
public:
    /**
     * @brief Construct button module
     * @param pin GPIO for button
     * @param activeLow true if pressed = LOW (default)
     */
    ButtonModule(int pin, bool activeLow = true);

    /**
     * @brief Initialize button pin with internal pull-up
     */
    void begin();

    /**
     * @brief Check if button is currently pressed
     */
    bool isPressed();

    /**
     * @brief Check if button was just pressed (edge detection)
     * Call this in loop() - returns true once per press
     */
    bool wasPressed();

    /**
     * @brief Check if button was just released (edge detection)
     */
    bool wasReleased();

    /**
     * @brief Update button state (call in loop for edge detection)
     */
    void update();

    /**
     * @brief Get raw button state (0 or 1)
     */
    int getState();

private:
    int _pin;
    bool _activeLow;
    bool _lastState;
    bool _currentState;
    bool _pressedFlag;
    bool _releasedFlag;
};

#endif // BUTTON_MODULE_H
