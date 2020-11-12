#ifndef MAIN_H
#define MAIN_H

#define MIN_FLOWRATE 10
#define MAX_FLOWRATE 80

#define MAX_VOLUME_ML 250
#define VOLUME_ML_STEPS 5
#define FLOWRATE_STEPS  5

#define FLOW_SENSOR_PPML 10.5
/*
typedef struct t_NTCsensor{
  struct s_ntc_setting{
      int RThbeta=3000;  
      int RTh0=10000;
      int RRef=10000;
  }settings;
  struct s_ntc_data{
      float RThValue=-1;  
      float Temp=-1;
  }measure;
}NTCsensor;
*/

typedef struct ppump{
    int MotorState=0;
    unsigned int AutoVolumeSetpoint_ml = 120;
    unsigned int AutoFlowRate_ml_min = 40;
    unsigned int ManualFlowRate_percent = 50;
    unsigned int MeasuredFlowRate_ml_min[2];
    unsigned int VolumeCounter_ml=0;

//    NTCsensor myNTC, myNTC1, myNTC2;
} PPUMP;

#endif // U8X8_USE_PINS