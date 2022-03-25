#ifndef ARDUINO_TYPE_INTERFACE_H
#define ARDUINO_TYPE_INTERFACE_H

#include "r2b_simcontroller.h"

extern R2bSimController r2b;

/**
 * @brief Code runs only once after basic initializations are completed.
 * 
 */
void setup(void);

/**
 * @brief Code keeps on running continuously until CTRL+C is  pressed.
 * 
 */
void loop(void);

/**
 * @brief Add delay for the set milli-seconds
 * 
 * @param millis Milli-second delay
 */
void delay(long millis);


#endif