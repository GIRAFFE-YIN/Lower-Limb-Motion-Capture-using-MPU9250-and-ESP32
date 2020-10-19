#include "timer.h"
#include "config.h"
#include "data.h"

hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer(){
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}

void init_timer() {
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();
  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);
  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);
  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, 150000, true);
  // Start an alarm
  timerAlarmEnable(timer);
}

bool timer_trigger() {
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
    portENTER_CRITICAL(&timerMux);
    portEXIT_CRITICAL(&timerMux);
    return true;
  }
  return false;
}
