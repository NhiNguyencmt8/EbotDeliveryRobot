#include <RBE1001Lib.h>

#include <Arduino.h>

#include <ESP32AnalogRead.h>

#include <button.h>

#include <linefollowing.h>

 

//unsigned long = positive long integer

unsigned long lastTime = 0;

const unsigned long LINE_FOLLOWING_INTERVAL = 2;

 

ESP32AnalogRead leftLineSensor;

ESP32AnalogRead rightLineSensor;

 

bool prevLeft = false;

bool prevRight = false;

 

int setPointOn = 1000;

int setPointOff = 70;

//int setPoint = 55;

 

float Kp = 0.1;

float Ki = 0.1;

float leftMotorSpeed;

float rightMotorSpeed;

 

static uint32_t lastErrorLeft = 0;

static uint32_t lastErrorRight = 0;

 

void lineFollow(int baseSpeed)

{

    unsigned long currTime = millis();

 

    if(currTime - lastTime > LINE_FOLLOWING_INTERVAL)

    {

        //read sensors

        float leftLineSense = analogRead(LEFT_LINE_SENSE);

        float rightLineSense = analogRead(RIGHT_LINE_SENSE);

 


 

        int currLeft = leftLineSense;

        int currRight = rightLineSense;



        //comand the motors

        if (currLeft > setPointOn) //need to move right

        {

            float errorLeft = abs(setPointOn - currLeft);

            float deltaErrorLeft = abs(errorLeft - lastErrorLeft);

            float effortLeft = Kp * errorLeft + Ki * deltaErrorLeft;

 

            //move right motor faster

            leftMotorSpeed = baseSpeed + effortLeft;

            rightMotorSpeed = baseSpeed - effortLeft;

            left_motor.setSpeed(leftMotorSpeed);

            right_motor.setSpeed(rightMotorSpeed);

            lastErrorLeft = errorLeft;

        }

        if (currRight > setPointOn)  //need to move right

        {

            float errorRight = abs(setPointOn - currRight);

            float deltaErrorRight = abs(errorRight - lastErrorRight);

            float effortRight = Kp * errorRight + Ki * deltaErrorRight;

 

            //move left motor faster

            leftMotorSpeed = baseSpeed - effortRight;
            rightMotorSpeed = baseSpeed + effortRight;
            left_motor.setSpeed(leftMotorSpeed);
            right_motor.setSpeed(rightMotorSpeed);
            lastErrorRight = errorRight;
        }

        if(currLeft < setPointOff && currRight < setPointOff)

        {

            left_motor.setSpeed(baseSpeed);
            right_motor.setSpeed(baseSpeed);

        }


 

    lastTime = currTime;

    }
 

}

bool meetIntersection(void){
    bool value = false;
    int currLeft = analogRead(LEFT_LINE_SENSE);
    int currRight = analogRead(RIGHT_LINE_SENSE);
    if(currLeft > setPointOn && currRight > setPointOn){

        value = true;   

        } return value;

}