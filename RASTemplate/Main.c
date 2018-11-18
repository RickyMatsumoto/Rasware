#include <RASLib/inc/common.h>
#include <RASLib/inc/gpio.h>
#include <RASLib/inc/time.h>
#include <RASLib/inc/motor.h>
#include <RASLib/inc/adc.h>
#include<stdio.h>

// Blink the LED to show we're on
tBoolean blink_on = true;
static tMotor *left;
static tMotor *right;
static tMotor *mid;
static tADC *adc[4];
static tBoolean initializedLine = false;

#define AVOID_W1 0x0
#define AVOID_W2 0x1
#define AVOID_W3 0x2
#define AVOID_W4 0x3
#define AVOID_W5 0x4
#define AVOID_W6 0x5
#define AVOID_W7 0x6
#define SPIN 0x7
#define AVOID_W8 0x8
#define AVOID_W9 0x9
#define AVOID_W10 0xA
#define AVOID_W11 0xB
#define AVOID_W12 0xC
#define AVOID_W13 0xD
#define AVOID_W14 0xE
#define MOVE 0xF

typedef const struct struct_t{
    int out;
    int next[16];
}struct_t;

struct_t FSM[16] = {
        {AVOID_W1,{AVOID_W1,AVOID_W2,AVOID_W3,AVOID_W4,AVOID_W5,AVOID_W6,AVOID_W7,SPIN,AVOID_W8,AVOID_W9,AVOID_W10,AVOID_W11,AVOID_W12,AVOID_W13,AVOID_W4,MOVE}},
        {AVOID_W2,{AVOID_W1,AVOID_W2,AVOID_W3,AVOID_W4,AVOID_W5,AVOID_W6,AVOID_W7,SPIN,AVOID_W8,AVOID_W9,AVOID_W10,AVOID_W11,AVOID_W12,AVOID_W13,AVOID_W4,MOVE}},
        {AVOID_W3,{AVOID_W1,AVOID_W2,AVOID_W3,AVOID_W4,AVOID_W5,AVOID_W6,AVOID_W7,SPIN,AVOID_W8,AVOID_W9,AVOID_W10,AVOID_W11,AVOID_W12,AVOID_W13,AVOID_W4,MOVE}},
        {AVOID_W4,{AVOID_W1,AVOID_W2,AVOID_W3,AVOID_W4,AVOID_W5,AVOID_W6,AVOID_W7,SPIN,AVOID_W8,AVOID_W9,AVOID_W10,AVOID_W11,AVOID_W12,AVOID_W13,AVOID_W4,MOVE}},
        {AVOID_W5,{AVOID_W1,AVOID_W2,AVOID_W3,AVOID_W4,AVOID_W5,AVOID_W6,AVOID_W7,SPIN,AVOID_W8,AVOID_W9,AVOID_W10,AVOID_W11,AVOID_W12,AVOID_W13,AVOID_W4,MOVE}},
        {AVOID_W6,{AVOID_W1,AVOID_W2,AVOID_W3,AVOID_W4,AVOID_W5,AVOID_W6,AVOID_W7,SPIN,AVOID_W8,AVOID_W9,AVOID_W10,AVOID_W11,AVOID_W12,AVOID_W13,AVOID_W4,MOVE}},
        {AVOID_W7,{AVOID_W1,AVOID_W2,AVOID_W3,AVOID_W4,AVOID_W5,AVOID_W6,AVOID_W7,SPIN,AVOID_W8,AVOID_W9,AVOID_W10,AVOID_W11,AVOID_W12,AVOID_W13,AVOID_W4,MOVE}},
        {SPIN ,{AVOID_W1,AVOID_W2,AVOID_W3,AVOID_W4,AVOID_W5,AVOID_W6,AVOID_W7,SPIN,AVOID_W8,AVOID_W9,AVOID_W10,AVOID_W11,AVOID_W12,AVOID_W13,AVOID_W4,MOVE}},
        {AVOID_W8,{AVOID_W1,AVOID_W2,AVOID_W3,AVOID_W4,AVOID_W5,AVOID_W6,AVOID_W7,SPIN,AVOID_W8,AVOID_W9,AVOID_W10,AVOID_W11,AVOID_W12,AVOID_W13,AVOID_W4,MOVE}},
        {AVOID_W9,{AVOID_W1,AVOID_W2,AVOID_W3,AVOID_W4,AVOID_W5,AVOID_W6,AVOID_W7,SPIN,AVOID_W8,AVOID_W9,AVOID_W10,AVOID_W11,AVOID_W12,AVOID_W13,AVOID_W4,MOVE}},
        {AVOID_W10,{AVOID_W1,AVOID_W2,AVOID_W3,AVOID_W4,AVOID_W5,AVOID_W6,AVOID_W7,SPIN,AVOID_W8,AVOID_W9,AVOID_W10,AVOID_W11,AVOID_W12,AVOID_W13,AVOID_W4,MOVE}},
        {AVOID_W11,{AVOID_W1,AVOID_W2,AVOID_W3,AVOID_W4,AVOID_W5,AVOID_W6,AVOID_W7,SPIN,AVOID_W8,AVOID_W9,AVOID_W10,AVOID_W11,AVOID_W12,AVOID_W13,AVOID_W4,MOVE}},
        {AVOID_W12,{AVOID_W1,AVOID_W2,AVOID_W3,AVOID_W4,AVOID_W5,AVOID_W6,AVOID_W7,SPIN,AVOID_W8,AVOID_W9,AVOID_W10,AVOID_W11,AVOID_W12,AVOID_W13,AVOID_W4,MOVE}},
        {AVOID_W13,{AVOID_W1,AVOID_W2,AVOID_W3,AVOID_W4,AVOID_W5,AVOID_W6,AVOID_W7,SPIN,AVOID_W8,AVOID_W9,AVOID_W10,AVOID_W11,AVOID_W12,AVOID_W13,AVOID_W4,MOVE}},
        {AVOID_W14,{AVOID_W1,AVOID_W2,AVOID_W3,AVOID_W4,AVOID_W5,AVOID_W6,AVOID_W7,SPIN,AVOID_W8,AVOID_W9,AVOID_W10,AVOID_W11,AVOID_W12,AVOID_W13,AVOID_W4,MOVE}},
        {MOVE,{AVOID_W1,AVOID_W2,AVOID_W3,AVOID_W4,AVOID_W5,AVOID_W6,AVOID_W7,SPIN,AVOID_W8,AVOID_W9,AVOID_W10,AVOID_W11,AVOID_W12,AVOID_W13,AVOID_W4,MOVE}}
};


void blink(void) {
    SetPin(PIN_F3, blink_on);
    blink_on = !blink_on;
}

void initSensor(void) {
    // don't initialize this if we've already done so
    if (initializedLine) {
        return;
    }
    
    initializedLine = true;

    adc[0] = InitializeADC(PIN_E1);
    adc[1] = InitializeADC(PIN_E2);
    adc[2] = InitializeADC(PIN_E3);
    adc[3] = InitializeADC(PIN_E4);
}

void findRobot(void){
    int state = 7;
    int input = 0x00;
    int pin1;
    int pin2;
    int pin3;
    int pin4;

    //SetMotor(left,-1.0);
    //SetMotor(right, 1.0);
    //Wait(1);

    SetMotor(mid, 1.0);
    Wait(1.5);


    while(1){
        SetMotor(mid, 0);
        /*
        Printf(
            "IR values: %1.3f %1.3f %1.3f %1.3f\r",
            ADCRead(adc[0]),
            ADCRead(adc[1]),
            ADCRead(adc[2]),
            ADCRead(adc[3])
        );
        */
        
        //out
        if(FSM[state].out == SPIN){
            SetMotor(left, -0.1);
            SetMotor(right, 0.1);
        }else if(FSM[state].out == MOVE){
            if(ADCRead(adc[3]) < .5){
                SetMotor(left, .7);
                SetMotor(right, .7);
            }else if(ADCRead(adc[3]) < .7){
                SetMotor(left, .8);
                SetMotor(right, .8);
            }else if(ADCRead(adc[3]) < .9){
                SetMotor(left, .9);
                SetMotor(right, .9);
            }else{
                SetMotor(left, 1.0);
                SetMotor(right, 1.0);
            }
        }else {
            SetMotor(left, -0.3);
            SetMotor(right, -0.3);
            Wait(.5);

            SetMotor(left, -0.175);
            SetMotor(right, 0.175);
            Wait(.75);
        }


        //input
        if(ADCRead(adc[3]) > 0.26){
            pin4 = 0x8;
        }else{
            pin4 = 0x0;
        }

        if(ADCRead(adc[0]) < 0.2){
            pin1 = 0x0;
        }else{
            pin1 = 0x4;
        }

        if(ADCRead(adc[1]) < 0.2){
            pin2 = 0x0;
        }else{
            pin2 = 0x2;
        }

        if(ADCRead(adc[2]) < 0.2){
            pin3 = 0x0;
        }else{
            pin3 = 0x1;
        }
        


        input = ((pin1 | pin2) | pin3) | pin4; 

        state = FSM[state].next[input];
        //state = input;

        //Reset Input
        input = 0x00;
        
    }
}


// The 'main' function is the entry point of the program
int main(void) {

    // Initialization code can go here
    CallEvery(blink, 0, 0.5);
    right = InitializeServoMotor(PIN_B6, false);
    left = InitializeServoMotor(PIN_B7, true);
    mid = InitializeServoMotor(PIN_C7, true);

    initSensor();
    findRobot();
}
