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
static tADC *adc[4];
static tBoolean initializedIR = false;
static tBoolean initializedLine = false;



#define STRAIGHT 0x2
#define CCW_SMALL 0x6
#define CCW_LARGE 0x4
#define SCREWED 0x0
#define CW_LARGE 0x1
#define CW_SMALL 0x3
#define SPIN 0x7



#define P_LEFT 0x0
#define PR_LEFT 0x1
#define STAY 0x2
#define PR_RIGHT 0x3
#define SPIN2 0x4
#define STRAIGHT2 0x5



typedef const struct struct_t{
    int out;
    int next[8];
}struct_t;

struct_t FSM[8] = {
        {SCREWED,{SCREWED,CW_LARGE,STRAIGHT,CCW_SMALL,CCW_LARGE,STRAIGHT,CW_SMALL,SPIN}},
        {CW_LARGE,{SCREWED,CW_LARGE,STRAIGHT,CCW_SMALL,CCW_LARGE,STRAIGHT,CW_SMALL,SPIN}},
        {STRAIGHT,{SCREWED,CW_LARGE,STRAIGHT,CCW_SMALL,CCW_LARGE,STRAIGHT,CW_SMALL,SPIN}},
        {CCW_SMALL,{SCREWED,CW_LARGE,STRAIGHT,CCW_SMALL,CCW_LARGE,STRAIGHT,CW_SMALL,SPIN}},
        {CCW_LARGE,{SCREWED,CW_LARGE,STRAIGHT,CCW_SMALL,CCW_LARGE,STRAIGHT,CW_SMALL,SPIN}},
        {STRAIGHT,{SCREWED,CW_LARGE,STRAIGHT,CCW_SMALL,CCW_LARGE,STRAIGHT,CW_SMALL,SPIN}},
        {CW_SMALL,{SCREWED,CW_LARGE,STRAIGHT,CCW_SMALL,CCW_LARGE,STRAIGHT,CW_SMALL,SPIN}},
        {SPIN ,{SCREWED,CW_LARGE,STRAIGHT,CCW_SMALL,CCW_LARGE,STRAIGHT,CW_SMALL,SPIN}}
};


typedef const struct struct_t2{
    int out2;
    int next2[16];
}struct_t2;

struct_t2 FSM2[16] = {
        {P_LEFT,{P_LEFT,PR_LEFT,STAY,PR_LEFT,PR_RIGHT,STAY,PR_RIGHT,SPIN2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,SPIN2}},
        {PR_LEFT,{P_LEFT,PR_LEFT,STAY,PR_LEFT,PR_RIGHT,STAY,PR_RIGHT,SPIN2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,SPIN2}},
        {STAY,{P_LEFT,PR_LEFT,STAY,PR_LEFT,PR_RIGHT,STAY,PR_RIGHT,SPIN2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,SPIN2}},
        {PR_LEFT,{P_LEFT,PR_LEFT,STAY,PR_LEFT,PR_RIGHT,STAY,PR_RIGHT,SPIN2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,SPIN2}},
        {PR_RIGHT,{P_LEFT,PR_LEFT,STAY,PR_LEFT,PR_RIGHT,STAY,PR_RIGHT,SPIN2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,SPIN2}},
        {STAY,{P_LEFT,PR_LEFT,STAY,PR_LEFT,PR_RIGHT,STAY,PR_RIGHT,SPIN2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,SPIN2}},
        {PR_RIGHT,{P_LEFT,PR_LEFT,STAY,PR_LEFT,PR_RIGHT,STAY,PR_RIGHT,SPIN2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,SPIN2}},
        {SPIN2 ,{P_LEFT,PR_LEFT,STAY,PR_LEFT,PR_RIGHT,STAY,PR_RIGHT,SPIN2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,SPIN2}},
        {STRAIGHT2,{P_LEFT,PR_LEFT,STAY,PR_LEFT,PR_RIGHT,STAY,PR_RIGHT,SPIN2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,SPIN2}},
        {STRAIGHT2,{P_LEFT,PR_LEFT,STAY,PR_LEFT,PR_RIGHT,STAY,PR_RIGHT,SPIN2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,SPIN2}},
        {STRAIGHT2,{P_LEFT,PR_LEFT,STAY,PR_LEFT,PR_RIGHT,STAY,PR_RIGHT,SPIN2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,SPIN2}},
        {STRAIGHT2,{P_LEFT,PR_LEFT,STAY,PR_LEFT,PR_RIGHT,STAY,PR_RIGHT,SPIN2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,SPIN2}},
        {STRAIGHT2,{P_LEFT,PR_LEFT,STAY,PR_LEFT,PR_RIGHT,STAY,PR_RIGHT,SPIN2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,SPIN2}},
        {STRAIGHT2,{P_LEFT,PR_LEFT,STAY,PR_LEFT,PR_RIGHT,STAY,PR_RIGHT,SPIN2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,SPIN2}},
        {STRAIGHT2,{P_LEFT,PR_LEFT,STAY,PR_LEFT,PR_RIGHT,STAY,PR_RIGHT,SPIN2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,SPIN2}},
        {SPIN2 ,{P_LEFT,PR_LEFT,STAY,PR_LEFT,PR_RIGHT,STAY,PR_RIGHT,SPIN2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,STRAIGHT2,SPIN2}}
};


void blink(void) {
    SetPin(PIN_F3, blink_on);
    blink_on = !blink_on;
}


void initIRSensor(void) {
    // don't initialize this if we've already done so
    if (initializedIR) {
        return;
    }
    
    initializedIR = true;

    // initialize 3 pins to be used for ADC input
    adc[0] = InitializeADC(PIN_E1);
    adc[1] = InitializeADC(PIN_E2);
    adc[2] = InitializeADC(PIN_E3);
}

void followBlackLine(void){
	int state = 0;
    int input = 0x0;
	int pin1;
    int pin2;
    int pin3;

    while(1){
            Printf(
            "IR values:  %1.3f  %1.3f  %1.3f\r",
            ADCRead(adc[0]),
            ADCRead(adc[1]),
            ADCRead(adc[2])
            );

        
        //out
        if(FSM[state].out == STRAIGHT){
            SetMotor(left, -0.1);
            SetMotor(right, -0.1);
        }else if(FSM[state].out == CCW_SMALL){
            SetMotor(left, -0.1);
            SetMotor(right, -0.04);
        }else if(FSM[state].out == CCW_LARGE){
            SetMotor(left, -0.15);
            SetMotor(right, -.04);
        }else if(FSM[state].out == SCREWED){
            SetMotor(left, -0.1);
            SetMotor(right, 0.1);
        }else if(FSM[state].out == CW_LARGE){
            SetMotor(left, -0.04);
            SetMotor(right, -0.15);
        }else if(FSM[state].out == CW_SMALL){
            SetMotor(left, -0.04);
            SetMotor(right, -0.1);
        }else if(FSM[state].out ==  SPIN){
            SetMotor(left, -0.1);
            SetMotor(right, -0.1);
        }

        //input
        if(ADCRead(adc[0]) > 0.975){
            pin1 = 0x4;
        }else{
            pin1 = 0x0;
        }

        if(ADCRead(adc[1]) > 0.975){
             pin2 = 0x2;
        }else{
            pin2 = 0x0;
        }

        if(ADCRead(adc[2]) > 0.975){
            pin3 = 0x1;
        }else{
            pin3 = 0x0;
        }

        input = (pin1 | pin2) | pin3; 

        state = FSM[state].next[input];

        //Reset Input
        input = 0x0;
    }
}

void initLineSensor(void) {
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

void findBlock(void){
	int state = 0;
    int input = 0x00;
	int pin1;
    int pin2;
    int pin3;
    int pin4;
    int leftTurn = 1;

    while(1){

/*
            Printf(
            "IR values: %1.3f %1.3f %1.3f %1.3f\r",
            ADCRead(adc[0]),
            ADCRead(adc[1]),
            ADCRead(adc[2]),
            ADCRead(adc[3])
            );
            */

    	//Printf("Input: %d", input);
        
        //out
        if(FSM2[state].out2 == P_LEFT){
        	if(leftTurn == 1){
            	SetMotor(left, -0.1);
            	SetMotor(right, 0);
        	}else{
        		SetMotor(left, 0);
            	SetMotor(right, -0.1);
        	}
        }else if(FSM2[state].out2 == PR_LEFT){
            SetMotor(left, 0.1);
            SetMotor(right, 0);
            //Wait(3);
            leftTurn = 0;
        }else if(FSM2[state].out2 == STAY){
            SetMotor(left, 0);
            SetMotor(right, 0);
        }else if(FSM2[state].out2 == PR_RIGHT){
            SetMotor(left, 0);
            SetMotor(right, 0.1);
            //Wait(3);
            leftTurn = 1;
        }else if(FSM2[state].out2 == SPIN2){
            SetMotor(left, -0.1);
            SetMotor(right, 0.1);
            //Wait(3);
        }else if(FSM2[state].out2 == STRAIGHT2){
            SetMotor(left, -0.1);
            SetMotor(right, -0.1);
       }


        //input
       	if(ADCRead(adc[3]) > 0.26){
        	pin4 = 0x8;
        }else{
        	pin4 = 0x0;
        }

        if(ADCRead(adc[0]) > 0.975){
            pin1 = 0x4;
        }else{
            pin1 = 0x0;
        }

        if(ADCRead(adc[1]) > 0.975){
             pin2 = 0x2;
        }else{
            pin2 = 0x0;
        }

        if(ADCRead(adc[2]) > 0.975){
            pin3 = 0x1;
        }else{
            pin3 = 0x0;
        }
        


        input = ((pin1 | pin2) | pin3) | pin4; 

        state = FSM2[state].next2[input];

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

    //SetMotor(left, 0.1);
    //SetMotor(right, 0.1);

    //initIRSensor();
   	//followBlackLine();

   	initLineSensor();
   	findBlock();
}
