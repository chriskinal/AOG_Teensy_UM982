#include "zALib0.h"

AButton::AButton(byte buttonPin) {
    pin = buttonPin;
    pinMode(pin, INPUT_PULLUP);
    state = Stable;
    pressed = digitalRead(pin);
  }
ButtonState AButton::check() {
    bool curState = digitalRead(pin);
    if (curState != pressed) {
      pressed = curState;
      state = Changing;
      lastChange = millis();
    }
    else if (state == Changing)
      if (millis()-lastChange >= DEBOUNCE) state = Changed;
    return state;
  }
bool AButton::is(bool hilo) {
    switch (check()) {
      case Changing: break; // return false;
      case Changed: state = Stable; //detected!
                    [[fallthrough]];
      case Stable: return pressed == hilo;
    }
		return false;
  }
bool AButton::changed() {
    switch (check()) {
      case Stable:
      case Changing: break; // return false;
      case Changed:
        state = Stable; //detected!
        return true;
    }
    return false;
  }
bool AButton::changedTo(bool hilo) {
    return changed() && pressed == hilo;
  }