/**
 * @file button.cpp
 * @brief Single Button Input Module Implementation
 */

#include "button.h"

ButtonModule::ButtonModule(int pin, bool activeLow)
    : _pin(pin)
    , _activeLow(activeLow)
    , _lastState(false)
    , _currentState(false)
    , _pressedFlag(false)
    , _releasedFlag(false)
{
}

void ButtonModule::begin() {
    pinMode(_pin, INPUT_PULLUP);
    _currentState = isPressed();
    _lastState = _currentState;
    Serial.print("[Button] Initialized on pin ");
    Serial.print(_pin);
    Serial.println(_activeLow ? " (active low)" : " (active high)");
}

bool ButtonModule::isPressed() {
    int raw = digitalRead(_pin);
    return _activeLow ? (raw == LOW) : (raw == HIGH);
}

bool ButtonModule::wasPressed() {
    update();
    if (_pressedFlag) {
        _pressedFlag = false;
        Serial.println("[Button] Pressed");
        return true;
    }
    return false;
}

bool ButtonModule::wasReleased() {
    update();
    if (_releasedFlag) {
        _releasedFlag = false;
        Serial.println("[Button] Released");
        return true;
    }
    return false;
}

void ButtonModule::update() {
    bool reading = isPressed();

    // Edge detection (no software debounce - hardware capacitor handles it)
    if (reading != _currentState) {
        _currentState = reading;
        if (_currentState) {
            _pressedFlag = true;  // Just pressed
        } else {
            _releasedFlag = true;  // Just released
        }
    }
}

int ButtonModule::getState() {
    return isPressed() ? 1 : 0;
}
