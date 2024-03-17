/* Simple debounced button object and task macros
 * by DoDi under MIT License
 */
#ifndef ALIB0_H
#define ALIB0_H

typedef unsigned long millis_t; //unless defined elsewhere

 #include <arduino.h>

//task macros, from Combie
 #define taskBegin() static int _mark = 0; static millis_t __attribute__((unused)) time_Stamp = 0; switch(_mark){ case 0:
 #define taskSwitch() { _mark = __LINE__; return; case __LINE__: ; }
 #define taskDelay(interval) time_Stamp = millis(); taskSwitch(); if ((millis() - time_Stamp) < (interval)) return;
 #define taskWaitFor(condition) taskSwitch(); if (!(condition)) return;
 #define taskRestart() { _mark = 0; return; }
 #define taskEnd() _mark=0; }

//do something at regular intervals. This is designed for use in loop(), not within a task
#define every(ms) for (static millis_t _t=0; millis()-_t>=ms; _t+=ms)

//button class
 #define DEBOUNCE 20 //debounce interval in ms

 typedef enum {Stable, Changing, Changed} ButtonState;

 class AButton {
 public:
   AButton(byte buttonPin);
   bool is(bool hilo);
   bool changed();
   bool changedTo(bool hilo);
 private:
  byte pin;
  millis_t lastChange;
 public: //for experts
  ButtonState state;
  bool pressed;
  ButtonState check();
};
#endif