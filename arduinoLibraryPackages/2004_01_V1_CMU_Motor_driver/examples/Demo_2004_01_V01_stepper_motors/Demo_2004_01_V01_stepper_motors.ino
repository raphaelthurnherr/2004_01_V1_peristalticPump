#include <Arduino.h>

#include "cmu_ws_2004_01_V1_board.h"
#include "cmu_ws_2004_01_V1_board.h"


// Define the default motor speed and steps for run from BNC trigger
#define DEFAULT_MOTOR_SPEED 25
#define DEFAULT_MOTOR_STEPS 200


// Main board 1921 - IC7 digital output bit definition for driver enable
#define GATE_A_ENABLE_BIT 6

//  Frame slots for commands
#define SYNC_SOF  0
#define MCMD  1
#define SPEED 2
#define STEPSCOUNT 3
#define SYNC_EOF  5


// Boards declaration
board_2004_01_V01 motor_2004_board;
// Arduino setup
void setup() {
  // Main board_2004 initialization
  motor_2004_board.begin();


  
  // Enable the motors power H-BRIDGES
 //motor_2004_board.setDigitalOutput(GATE_A_ENABLE_BIT,1);     // Gate A enable


  delay(10);

}


// Main ARDUINO LOOP

void loop() {


  // Get input the BNC input state and make action if triggered, 
  // reset trigget when BNC input go to lower state
  
  //  USE BNC INPUT TRIGGER 0 for motor B(=motor 1)

      motor_2004_board.stepperRotation(MOTOR_A, DEFAULT_MOTOR_SPEED, DEFAULT_MOTOR_STEPS);  
         
  delay(1500);
}

