/**
 * \file CONFIG_2004_01_V1.h
 * \brief CONFIG FILE FOR DEVICE ADDRESS DEFINITION ON 2004 BOARD
 * \author Raphael Thurnherr
 * \version 0.1
 * \date 09.06.2020
 */

#ifndef CONFIG_2004_01_V1_H
#define CONFIG_2004_01_V1_H

// ARDUINO PIN DEFINITION
#define ARDUINO_LED_PIN 13

#define  LED_RED_PIN     9
#define  LED_GREEN_PIN   10

#define RSWITCH_SW_PIN  2
#define RSWITCH_CLK_PIN 3
#define RSWITCH_DAT_PIN 4
#define PULSE_COUNT_PIN 6

// Define the default motor speed and steps for run
#define DEFAULT_MOTOR_SPEED 25          // Could be negative (-25) for rotation in CCW
#define DEFAULT_MOTOR_STEPS 200         // Could be negative (<0) for continuous rotation and (=0) for motor stop

#define PCA9629A_ADR    0x20



#endif