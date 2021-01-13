#include <Arduino.h>
#include <U8x8lib.h>

#include "CONFIG_2004_01_V1.h"
#include "rotarySwitch.h"
#include "displayManager.h"
#include "device_drivers/src/pca9629.h"


// u8x8 OLED display object declaration
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);   // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED
//U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);   // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED

// PUMP structure definition
PPUMP pump;

// Rotary button variable (Constant defined in "rotarySwitch.h")
signed char rotarySwitchEvent = ROTARY_NO_EVENT;

// Menu manager variable
unsigned char menu = 1, old_menu = 0;

// Timer variable
int timer1Sec = 0, timer3Sec=0, timerRotaryPress1mS = 0;

// Error flow detection
int errorPumping;

// Motor driver settings variable
device_pca9629 motor;

void runPumping(unsigned int speed, int volume);
void stopPumping(void);
unsigned int getPulseCount(unsigned char pin, unsigned char reset);

// Rotary button event functions 
void on_release_action(unsigned char menuNb);
void on_rotary_cw_action(unsigned char menuNb);
void on_rotary_ccw_action(unsigned char menuNb);
void on_longPress_action(unsigned char menuNb);

int PID_PowerControl(int currentFlow, int setPoint, int reset);

// Timer function
void onTimer1Sec(void);


void setup() {

  Serial.begin(9600); // open the serial port at 9600 bps:

  // Arduino pin configuration
  pinMode(PULSE_COUNT_PIN, INPUT);
  pinMode(RSWITCH_SW_PIN, INPUT);
  pinMode(RSWITCH_CLK_PIN, INPUT);
  pinMode(RSWITCH_DAT_PIN, INPUT);
  
  pinMode(ARDUINO_LED_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);

  // Test LED
  digitalWrite(ARDUINO_LED_PIN, 1);

  // Motor drivers initialization
  motor.bipolar_mode=MOTOR_MODE_BIPOLAR;
  //motor.bipolar_mode=MOTOR_MODE_BIPOLAR_HALF_STEP;
  
  motor.deviceAddress=PCA9629A_ADR;
  
  pca9629_init(&motor);

  // OLED Display init
  u8x8.begin();
  u8x8.setFlipMode(1);
  u8x8.setFont(u8x8_font_7x14B_1x2_f);
  u8x8.drawString(0, 0, "----------------");
  u8x8.drawString(0, 2, "PERISTALTIC PUMP");
  u8x8.drawString(0, 4, "----------------");
  u8x8.drawString(0, 6, " RTH 13.01.2021 ");
  
  delay(1000);

  // Config P2 & P3 for L298P Enable 
  PCA9629_GPIOConfig(&motor, 0xC3);
  
  actuator_setStepperDriveMode(&motor, MOTOR_MODE_BIPOLAR);

  delay(1000);

  digitalWrite(ARDUINO_LED_PIN, 0);
  digitalWrite(LED_RED_PIN, 1);         // Error led OFF
  digitalWrite(LED_GREEN_PIN, 1);       // RUNNING led OFF
}

unsigned int FlowCounterPulses=0;
unsigned int oldFlowCounterPulses=0;
void loop() {

  FlowCounterPulses = getPulseCount(PULSE_COUNT_PIN, 0);
  pump.VolumeCounter_ml =  FlowCounterPulses / FLOW_SENSOR_PPML;

// ------------  ROTARY BUTTON EVENT MANAGEMENT -------------
  rotarySwitchEvent = getRotarySwitchEvent(RSWITCH_CLK_PIN, RSWITCH_DAT_PIN, RSWITCH_SW_PIN, &timerRotaryPress1mS);
  // Event detection on rotary switch
  if(rotarySwitchEvent != ROTARY_NO_EVENT){
    switch(rotarySwitchEvent){
      case ROTARY_INC : on_rotary_cw_action(menu); break;
      case ROTARY_DEC : on_rotary_ccw_action(menu); break;
      case ROTARY_SW_PUSH :  break;
      case ROTARY_SW_RELEASE : on_release_action(menu); break;
      case ROTARY_SW_LONG_PUSH : on_longPress_action(menu); break;
      default : break;
    }
  }

// ------------ DISPLAY MANAGEMENT -------------
    // Refresh display only when menu change
    if((old_menu != menu)){
      refreshMenuDisplay(&u8x8, menu, &pump, 1);    
      old_menu = menu;
    }

    if((pump.VolumeCounter_ml >= pump.AutoVolumeSetpoint_ml) && menu == 20){
        
        stopPumping();
        u8x8.drawString(0, 6, "  END PUMPING   ");
    }

    if(timer1Sec>=1000){
      if(menu == 10 || menu == 20 ){
        // Refresh display data when running
        refreshMenuDisplay(&u8x8, menu, &pump, 0);

        // Reset watchdog flow when flowrate detected
        if(pump.MeasuredFlowRate_ml_min[0] > 0){
          timer3Sec=0;              // Reset watchdog for empty flow
        }
      }else{
              timer3Sec=0;              // Reset watchdog for empty flow detection 
      }

      // Liquid flow autotunnig 
      if(menu == 20){
        int temp = PID_PowerControl(pump.MeasuredFlowRate_ml_min[0], pump.AutoFlowRate_ml_min, 0);
        runPumping(temp, -1);
      }

      //pump.MeasuredFlowRate_ml_min[1] = pump.MeasuredFlowRate_ml_min[0];
      //pump.MeasuredFlowRate_ml_min[0] = round((pump.MeasuredFlowRate_ml_min[1] + round(((FlowCounterPulses - oldFlowCounterPulses) *60 ) / FLOW_SENSOR_PPML))/2);
      pump.MeasuredFlowRate_ml_min[0] = round(((FlowCounterPulses - oldFlowCounterPulses) *60 ) / FLOW_SENSOR_PPML);
      
      oldFlowCounterPulses = FlowCounterPulses;
      onTimer1Sec();
      timer1Sec=0;
    }

    if(timer3Sec >= 3000){       
        // Liquide flow error ! stop pumping
        // Stop pumping when auto mode is selected
        if(menu==20){
          stopPumping();
        }
        // Display ERROR MENU
        menu = 3;
        digitalWrite(LED_RED_PIN, 0);     // Turn on ERROR RED LED
        digitalWrite(LED_GREEN_PIN, 1);     // Turn OFF green LED
        Serial.print("ERROR ");
        timer3Sec=0;
    }

  timer1Sec++;
  timer3Sec++;
  timerRotaryPress1mS++;
  delay(1);
}


//*******************************************
// ---------  ROTARY AND MENU FUNCTIONS
//*******************************************

void on_rotary_cw_action(unsigned char menuNb){
  switch(menuNb){
    // Switch menu1<->menu2
    case 1: menu = 2; break;
    case 2: menu = 1; break;
  
  // Menu 1.0 change setting
    case 10: if(pump.ManualFlowRate_percent <= (100-FLOWRATE_STEPS)) pump.ManualFlowRate_percent+=FLOWRATE_STEPS;
             runPumping(pump.ManualFlowRate_percent, -1);
             display_menu_1_0(&u8x8, pump.ManualFlowRate_percent, pump.VolumeCounter_ml, pump.MeasuredFlowRate_ml_min[0]); break;
  // Menu 2.1.0 change setting for AUTO MENU VOLUME
    case 210: if(pump.AutoVolumeSetpoint_ml<=(MAX_VOLUME_ML - VOLUME_ML_STEPS)) pump.AutoVolumeSetpoint_ml+=VOLUME_ML_STEPS;
              display_menu_2_1_0(&u8x8, pump.AutoVolumeSetpoint_ml, pump.AutoFlowRate_ml_min); break;
  // Menu 2.2.0 change setting for AUTO MENU FLOWRATE
    case 220: if(pump.AutoFlowRate_ml_min <=(MAX_FLOWRATE - FLOWRATE_STEPS)) pump.AutoFlowRate_ml_min+=FLOWRATE_STEPS;
              display_menu_2_2_0(&u8x8, pump.AutoVolumeSetpoint_ml, pump.AutoFlowRate_ml_min); break;
    default :  break;
  }
}
//*******************************************
void on_rotary_ccw_action(unsigned char menuNb){
  switch(menuNb){
    // Switch menu1<->menu2
    case 1: menu = 2; break;
    case 2: menu = 1; break;
  
  // Menu 1.0 change setting
    case 10: if(pump.ManualFlowRate_percent >= FLOWRATE_STEPS) pump.ManualFlowRate_percent-=FLOWRATE_STEPS;
            runPumping(pump.ManualFlowRate_percent, -1);
            display_menu_1_0(&u8x8, pump.ManualFlowRate_percent, pump.VolumeCounter_ml, pump.MeasuredFlowRate_ml_min[0]); break;
  // Menu 2.1.0 change setting for MANUAL MENU
    case 210: if(pump.AutoVolumeSetpoint_ml >= VOLUME_ML_STEPS) pump.AutoVolumeSetpoint_ml-=VOLUME_ML_STEPS;
              display_menu_2_1_0(&u8x8, pump.AutoVolumeSetpoint_ml, pump.AutoFlowRate_ml_min); break;
  // Menu 2.2.0 change setting for AUTO MENU
    case 220: if(pump.AutoFlowRate_ml_min >= (MIN_FLOWRATE + FLOWRATE_STEPS)) pump.AutoFlowRate_ml_min-=FLOWRATE_STEPS;
              display_menu_2_2_0(&u8x8, pump.AutoVolumeSetpoint_ml, pump.AutoFlowRate_ml_min); break;
    default :  break;
  } 
}
//*******************************************
void on_release_action(unsigned char menuNb){
  switch(menuNb){
    case 1: menu = 10;  FlowCounterPulses = oldFlowCounterPulses=0;
                        getPulseCount(PULSE_COUNT_PIN, 1);                  // Reset volume counter
                        runPumping(pump.ManualFlowRate_percent, -1); break;  // Start pumping continuous
    case 2: menu = 20;  FlowCounterPulses = oldFlowCounterPulses=0;
                        PID_PowerControl(0, 0, 1);
                        getPulseCount(PULSE_COUNT_PIN, 1);                  // Reset volume counter
                        break;

    case 10: menu = 1; stopPumping();break;
    case 20: menu = 2; stopPumping();break;

    case 210: menu = 220; break;
    case 220: menu = 210; break;
    default :  break;
  } 
}

//*******************************************
void on_longPress_action(unsigned char menuNb){
    switch(menuNb){
      case 10:  getPulseCount(PULSE_COUNT_PIN, 1); break;         // Reset volume counter
      case 2:   menu = 210; break;
      case 3:   menu = 1; digitalWrite(LED_RED_PIN, 1); stopPumping(); break;    // Turn OFF ERROR RED LED
      case 20:  getPulseCount(PULSE_COUNT_PIN, 1); break;         // Reset volume counter
      case 210: menu = 2; break;
      case 220: menu = 2; break;
    default :  break;
  } 
}

//*******************************************
// --------- TIMER FUNCTION
//*******************************************
void onTimer1Sec(void){

}


//*******************************************
// --------- MOTOR PUMPING
//*******************************************
void runPumping(unsigned int speed, int volume){
  actuator_setStepperSpeed(&motor, speed);
  actuator_setStepperStepAction(&motor, MOTOR_CCW, volume*40);
  digitalWrite(LED_GREEN_PIN, 0);       // Turn ON GREEN LED
  digitalWrite(LED_RED_PIN, 1);         // Turn OFF RED LED (CLEAR ERROR)
}

void stopPumping(void){
  actuator_setStepperStepAction(&motor, MOTOR_STOP, 0);
  digitalWrite(LED_GREEN_PIN, 1);     // Turn OFF GREEN LED
}

//*************************  PULSE COUNTER
unsigned int getPulseCount(unsigned char pin, unsigned char reset){
  static unsigned char pulse_CN2_oneShot=0;
  static unsigned int pulse_CN2_counter=0;
  unsigned char pulse_CN2_State =0;

  if(reset){
    pulse_CN2_counter = 0;
    pulse_CN2_oneShot = 0;
  }else{
        pulse_CN2_State=digitalRead(pin);
        if(pulse_CN2_State){
          if(!pulse_CN2_oneShot){
          pulse_CN2_counter ++;
          pulse_CN2_oneShot = 1; 
          }
        }else pulse_CN2_oneShot=0;
  }
  return pulse_CN2_counter;
}


int PID_PowerControl(int currentFlow, int setPoint, int reset){

    float Kp = 0.8;
    float Ki = 0.5;
    float Kd = 0.02;
    float loopTimeDT = 1.0; 
    
    static int lastSpeed;
    static int sumError;
        
    float output;
    float outputMin=0;
    float outputMax=100;
    float error;
    float newSum;
    float dErrorLoopTime; 

    if (reset){
      sumError = 0;
      lastSpeed = 0;
    }

    error = setPoint - currentFlow;

    newSum = (sumError + error) * loopTimeDT;

    dErrorLoopTime = (lastSpeed - currentFlow) / loopTimeDT;
    lastSpeed = currentFlow;
    
    output = Kp * error + Ki * sumError + Kd * dErrorLoopTime;
    
    if(output >= outputMax)
        output = outputMax;
    else
        if(output <= outputMin)
            output = outputMin;
        else 
            sumError =  newSum;
    
    return (int) output;
}