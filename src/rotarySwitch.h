// Rotary switch pin definition in getRotarySwitchEvent()
/*
#define rswitch_clk_pin 2
#define rswitch_dat_pin 0
#define rswitch_sw_pin 16
*/


#ifndef ROTARYSWITCH_H
#define ROTARYSWITCH_H

#define ROTARY_NO_EVENT -1
#define ROTARY_INC 0
#define ROTARY_DEC 1
#define ROTARY_SW_PUSH 2
#define ROTARY_SW_RELEASE 3
#define ROTARY_SW_LONG_PUSH 4

extern signed char getRotarySwitchEvent(unsigned char rswitch_clk_pin, unsigned char rswitch_dat_pin, unsigned char rswitch_sw_pin, int * timerPress);

#endif // U8X8_USE_PINS