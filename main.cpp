#include <Arduino.h>
#include <RBE1001Lib.h>
#include <ESP32Servo.h>
#include <ESP32AnalogRead.h>
#include <IRdecoder.h>
#include <button.h>
#include <navigator.h>
#include <linefollowing.h>
#include <list>


const int photoresistorPin = 34;
ESP32AnalogRead intersectSensor;
static int baseSpeed = 100;
int speed = 10;
static float AngleTurn = 0 ;

Navigator navigator;

// https://wpiroboticsengineering.github.io/RBE1001Lib/classServo.html
Servo lifter;
// https://wpiroboticsengineering.github.io/RBE1001Lib/classESP32AnalogRead.html
ESP32AnalogRead servoPositionFeedback;
Rangefinder ultrasonic;
// define the pin for the IR receiver
const uint8_t IR_DETECTOR_PIN = 15;

//define the key on IR Remote
int VOL_PLUS = 2;
int VOL_MINUS = 0;
int STOP_BUTTON = 6;
int KEY1 = 16;
int KEY2 = 17;
int KEY3 = 18;
int KEY4 = 20;
int ENTER = 9;
int CH_PLUS = 10;
int CH_MINUS = 8;
int PLAY = 1;
int RETURN = 14;
static int key = 0;

float preReading = 0.0;
float diameter = 6.00; // wheel's diameter
float targetWheelAngle;
int count = 0;

// Create a button object for the built-in button on the ESP32
Button bootButton(BOOT_FLAG_PIN);
Button enButton(ESP_RST_POWERON);

enum ROBOT_STATE {ROBOT_IDLE,ROBOT_ACTIVE,ROBOT_DRIVING,ROBOT_APPROACHING,ROBOT_LINING,ROBOT_PLACING, ROBOT_FREERANGE};

// create an IRDecoder object
IRDecoder decoder(IR_DETECTOR_PIN);

// Declare a variable, robotState, of our new type, ROBOT_STATE. Initialize it to ROBOT_IDLE.
ROBOT_STATE robotState = ROBOT_IDLE;
Motor left_motor;
Motor right_motor;


void setup() 
{
  // This will initialize the Serial as 115200 for prints
  Serial.begin(115200);

  //Pins typically default to INPUT, but the code reaads easier if you are explicit:
  pinMode(photoresistorPin, INPUT);
    // Initialize the decoder
  decoder.init();
  //Some motor admin; be sure to include this with any motor code
  Motor::allocateTimer(0);
  //Pin definitions can be found in RBE1001Lib.h
  left_motor.attach(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR,
  MOTOR_LEFT_ENCA, MOTOR_LEFT_ENCB);
  right_motor.attach(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR,
  MOTOR_RIGHT_ENCA, MOTOR_RIGHT_ENCB);
  intersectSensor.attach(15);

  ESP32PWM::allocateTimer(1); // Used by servos

	// pin definitions https://wpiroboticsengineering.github.io/RBE1001Lib/RBE1001Lib_8h.html#define-members
	lifter.attach(SERVO_PIN);
	servoPositionFeedback.attach(SERVO_FEEDBACK_SENSOR);
	//lifter.write(0);
  ultrasonic.attach(SIDE_ULTRASONIC_TRIG, SIDE_ULTRASONIC_ECHO);
}

void stop()
{
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
}

void driveForDistance(int givenDistance){ 
  Serial.println("Driving for given distance");
  Serial.print(givenDistance);
  float angle_drive = givenDistance*360/(diameter*3.14);
  left_motor.startMoveFor(angle_drive, 120);
  right_motor.startMoveFor(angle_drive,120);
}

void HandleIntersection(void)
{
  TURN turning = navigator.CalcTurn();
  ultrasonic.getDistanceCM();

  switch(turning)// might want to turn over the intersection (course is tight)
  {
    case TURN_LEFT:
    right_motor.moveFor(360, 100);
    //Serial.println("Turned Left");
    break;

    case TURN_RIGHT:
    left_motor.moveFor(300, 100);
    //Serial.println("turned Right");
    break;

    case TURN_STRAIGHT:
    left_motor.setSpeed(60);
    right_motor.setSpeed(60);
    delay(500);
    stop();
    delay(1000);
    //Serial.println("Passed Intersection");
    break;

    case TURN_UTURN:
    break;
  }

}


void startDriving()
{
Serial.println("Driving !");
left_motor.setSpeed(150);
right_motor.setSpeed(150);
}

void driveWithSpeed(float speed)
{
  left_motor.setSpeed(speed);
  right_motor.setSpeed(speed);
}

void turnAround()
{
  left_motor.setSpeed(180);
  right_motor.setSpeed(-180);
  delay(2100);
  stop();
  Serial.println("Pass Intersection");
  driveWithSpeed(60);
  delay (1000);
  stop();
}

void backUp()
{
  left_motor.setSpeed(-100);
  right_motor.setSpeed(-100);
  delay(250);
  stop();
}

void scanBag()
{
  left_motor.setSpeed(-40);
  right_motor.setSpeed(40);
}


void loop() 
{

  
  // Check for a key press on the remote
  int16_t keyPress = decoder.getKeyCode();

  // If a valid key is pressed, print out its value
  if(keyPress >= 0) {
    Serial.println("Key: " + String(keyPress));
  }

  //IDLE STATE
    // Go through the state machine
  if(robotState == ROBOT_IDLE)
  {
    if(bootButton.CheckButtonPress()) //if the button was pressed, switch to ACTIVE
    {
      // Notify us that we're switching to ACTIVE
      Serial.println("Button press -> ACTIVE");
      //robotState = ROBOT_LINING;

      // Finally, update the state
      robotState = ROBOT_ACTIVE;
    }
  }

    if(keyPress == VOL_MINUS){
      Serial.println(ultrasonic.getDistanceCM());
    }

  //ACTIVE STATE
  //note that we use else..if for each additional state, so it doesn't get confused
  if(robotState == ROBOT_ACTIVE)
  {
 
    // Go to pickup zone
    if(keyPress == KEY4){
      navigator.updateDestination(keyPress);
      Serial.println("START PICK UP BAG");
      robotState = ROBOT_LINING;
    }


    if(keyPress == RETURN) //if the button was pressed, switch to IDLE
    {
      Serial.println("Button press -> IDLE");
      // Finally, update the state
      robotState = ROBOT_IDLE;
    }

 
    // Go to Houses
    if (keyPress == KEY1 || keyPress == KEY2 || keyPress == KEY3 )
      {
        key = keyPress;
        navigator.updateDestination(keyPress);
        Serial.println("GOING TO HOUSES");
        robotState = ROBOT_LINING;
      }

  }

  //DRIVING STATE
  else if(robotState == ROBOT_DRIVING)
  {
   //turn back to line
    right_motor.setSpeed(100);
    left_motor.setSpeed(0);
    delay(5000);
    stop();
    delay(1000);
    while (meetIntersection() == false)
    {
      Serial.println("Finding PICKUP road");
      right_motor.setSpeed(100);
      left_motor.setSpeed(100);
    }

    if (meetIntersection() == true)
    {
      Serial.println("Found PICKUP road");
      stop();
      left_motor.moveFor(320, 100);
      count ++;
      robotState = ROBOT_ACTIVE;
    }

  
  
  }

  //ROBOT APPROACHING MODE
  else if(robotState == ROBOT_APPROACHING)
  {
    float distance = ultrasonic.getDistanceCM();
    // Position the arm
    lifter.write(20);


    //Begin "creeping" towards the bag
    //While creeping, continually check the distance
    
    if (distance > 5){
      float error = distance - 5;
      driveWithSpeed(error*10);

    }

    //When the proper distance is reached, stop and pick up the bag
    else  {
      driveWithSpeed(60);
      delay(1000);
      Serial.println("Found the bag!");
      stop();
      delay(1000);
      //pickup the bag
      lifter.write(180);
      Serial.println("Sucessfully picked up the bag!");
      delay(1000);
      
      turnAround();
      Serial.println("turned around");
      delay(1000);
      driveWithSpeed(100);
      delay(500);
      stop();
      robotState = ROBOT_ACTIVE;
  
      
    }
  }

  else if(robotState == ROBOT_FREERANGE){
    float distance = ultrasonic.getDistanceCM();
    // Position the arm
    lifter.write(20);


    //Begin "creeping" towards the bag
    //While creeping, continually check the distance
    
    if (distance > 4){
      float error = distance - 4;
      driveWithSpeed(error*3);

    }

    //When the proper distance is reached, stop and pick up the bag
    else  {
      Serial.println("Found the bag!");
      stop();
      delay(1000);
      //pickup the bag
      lifter.write(180);
      Serial.println("Sucessfully picked up the bag!");
      robotState = ROBOT_DRIVING;
  }}

  if (robotState == ROBOT_LINING)
  {
    //Main line following FCN

    lineFollow(baseSpeed);

    if (meetIntersection())
    {
      ultrasonic.getDistanceCM();
      Serial.println(navigator.GetRoad());
      Serial.println(navigator.GetDestination());
      Serial.println(ultrasonic.getDistanceCM());
      stop();

      delay(1000);
      if(navigator.ReachDestination() == true && navigator.PICKUPDestination() == true)
      { if(key == KEY4)
        {
          Serial.println("Free Range Bag");
          float initalAngle = left_motor.getCurrentDegrees();
          delay(500);
          while(ultrasonic.getDistanceCM() > 30)
          {
            scanBag();
          }
          stop();
          AngleTurn = initalAngle - left_motor.getCurrentDegrees();      
          robotState = ROBOT_FREERANGE;    
        } 

        else{
        //Pick up the bag
        Serial.println("Robot is in approaching state");
        count ++;
        //lifter.write(20);
        //delay(1000);
        Serial.println(ultrasonic.getDistanceCM());
        robotState = ROBOT_APPROACHING;
        }
      }
      else if(navigator.ReachDestination() == true){
        Serial.println("Destination Reached");
        navigator.updateDestination(20);
        //count ++;
        robotState = ROBOT_PLACING;
      }
      else 
      {
        Serial.println("handling intersection");
        HandleIntersection();
        //count++;
        Serial.println(navigator.GetRoad());
        Serial.println(navigator.GetDestination());
        //Serial.println(ultrasonic.getDistanceCM());
        robotState = ROBOT_LINING;
      }
      
    }
  }

  else if(robotState == ROBOT_PLACING)
  {
    lineFollow(baseSpeed);
    if(meetIntersection()){
    stop();
    lifter.write(180);
    delay(1000);    
    if(navigator.DestHouseC() == true){
      lifter.write(90);
      delay(500);
      left_motor.setSpeed(-60);
      right_motor.setSpeed(-60);
      delay(2000);
    } else {
      lifter.write(0);
      delay(2000);
    }
    backUp();
    //lifter.write(170);
    turnAround();
    count = 0;
    //navigator.updateDestination(20);
    robotState = ROBOT_ACTIVE;
    }

  }

  //Emergency stop situation
  if(keyPress == STOP_BUTTON)
  {
    Serial.println("Emergency STOP");
    stop();
    robotState = ROBOT_IDLE;
  }
  
}