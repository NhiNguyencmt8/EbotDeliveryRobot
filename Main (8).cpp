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
static int baseSpeed = 110;
int speed = 10;
static float AngleTurn;

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

float preReading = 0.0;
float diameter = 6.00; // wheel's diameter
float targetWheelAngle;

bool FreeRange = false;

// Create a button object for the built-in button on the ESP32
Button bootButton(BOOT_FLAG_PIN);
Button enButton(ESP_RST_POWERON);

enum ROBOT_STATE {ROBOT_IDLE,ROBOT_ACTIVE,ROBOT_APPROACHING,ROBOT_LINING,ROBOT_PLACING,ROBOT_FREERANGE,ROBOT_FINDLINE};

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
  ultrasonic.attach(SIDE_ULTRASONIC_TRIG, SIDE_ULTRASONIC_ECHO);
}

void stop()
{
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
}

void rotateforBag()
{
  left_motor.setSpeed(-40);
  right_motor.setSpeed(40);
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
    left_motor.moveFor(360, 100);
    //Serial.println("turned Right");
    break;

    case TURN_STRAIGHT:
    left_motor.setSpeed(70);
    right_motor.setSpeed(72);
    delay(500);
    stop();
    delay(800);
    //Serial.println("Passed Intersection");
    break;

    case TURN_UTURN:
    break;
  }

 
}


void startDriving()
{
Serial.println("Driving");
left_motor.setSpeed(75);
right_motor.setSpeed(75);
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
}



void loop() 
{

  //float distance = ultrasonic.getDistanceCM();

  
  // Check for a key press on the remote
  int16_t keyPress = decoder.getKeyCode();

  // If a valid key is pressed, print out its value
  if(keyPress >= 0) 
  {
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

  //ACTIVE STATE
  //note that we use else..if for each additional state, so it doesn't get confused
  if(robotState == ROBOT_ACTIVE)
  {
     
    // Go to pickup zone
    if(keyPress == KEY4)
    {
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
        navigator.updateDestination(keyPress);
        Serial.println("GOING TO HOUSES");
        robotState = ROBOT_LINING;
      }
  }

  //ROBOT APPROACHING MODE
  else if(robotState == ROBOT_APPROACHING)
  {
    float distance = ultrasonic.getDistanceCM();

    // Position the arm down
    lifter.write(180);

    //Begin "creeping" towards the bag
    //While creeping, continually check the distance
    if (distance > 7)
    {
      float error = distance - 3;
      driveWithSpeed(error*25);
    }

    //slow down
    else if (distance <= 7 && distance > 3)
    {
      float error = distance-3;
      driveWithSpeed(error*25);
    }

    //When the proper distance is reached, stop and pick up the bag
    else if (distance <= 3)
    {
      Serial.println("Found the bag");
      stop();

      //pickup the bag
      driveWithSpeed(100);
      delay(500);
      stop();

      //arm goes up
      lifter.write(20);
      Serial.println("Sucessfully picked up the bag");

      if(FreeRange == true)
      {
        //changes state for the robot to find the PICKUP road after bag pick up
        robotState = ROBOT_FINDLINE;
      }
      else if (FreeRange == false)
      {
        //leaves PICKUP zone
        turnAround();
        Serial.println("turned around");
        delay(1000);
        driveWithSpeed(100);
        delay(500);
        stop();
        robotState = ROBOT_ACTIVE;
      }      
    }
  }

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
      {
        if(ultrasonic.getDistanceCM() > 20)
        {
          //switch to Free Range mode
          robotState = ROBOT_FREERANGE;
        }
        else
        {
          Serial.println("Robot is in approaching state");
          Serial.println(ultrasonic.getDistanceCM());
          robotState = ROBOT_APPROACHING;
        }
      }

      else if(navigator.ReachDestination() == true)
      {
        Serial.println("Destination Reached");
        //changes desination to bag pickup
        navigator.updateDestination(20);
        robotState = ROBOT_PLACING;
      }
      else 
      {
        Serial.println("handling intersection");
        HandleIntersection();
        Serial.println(navigator.GetRoad());
        Serial.println(navigator.GetDestination());
        robotState = ROBOT_LINING;
      }
      
    }
  }

  else if(robotState == ROBOT_PLACING)
  {
    //approach house
    driveWithSpeed(65);
    delay(325);
    stop();
    //pause to decrease bag swinging
    delay(1200);

    //lower bag
    if (navigator.GetDestination() == 1)
    {
      //for HOUSE_A
      lifter.write(50);
      delay(1000);
      lifter.write(90);
      delay(1000);
      lifter.write(165);
    }
    else if(navigator.GetDestination() == 2)
    {
      //for HOUSE_B
      lifter.write(90);
      delay(1000);
      lifter.write(115);      
    }
    else if(navigator.GetDestination() == 3)
    {
      //for HOUSE_C
      lifter.write(50);
      delay(1000);
      lifter.write(90);      
    }    
    backUp();
    delay(1200);
    stop();
    turnAround();
    stop();
    robotState = ROBOT_ACTIVE;
  }

  else if(robotState == ROBOT_FREERANGE)
  {
    //Scan for the bag in the Free Range zone
    Serial.println("Free Range Bag");
    float initalAngle = left_motor.getCurrentDegrees();
    Serial.println(initalAngle);
    delay(500);
    //rotates away from PICKUP zone
    rotateforBag();
    delay(3500);
    while(ultrasonic.getDistanceCM() > 50)
    {
      //rotates while no bag is detected
      rotateforBag();
    }
    //completes rotation angle needed to drive straight towards bag
    rotateforBag();
    delay(1300);

    Serial.println(ultrasonic.getDistanceCM());
    stop();
    delay(1000);

    //Calculates the angle the robot has turned
    AngleTurn = right_motor.getCurrentDegrees() - initalAngle;
    Serial.println(AngleTurn);

    FreeRange = true;

    Serial.println("Approaching Free Range Bag");
    robotState = ROBOT_APPROACHING;
  }

  else if(robotState == ROBOT_FINDLINE)
  {
    //Calculate angle needed to turn to get back to PICKUP road
    float Theta = AngleTurn - 180;
    Serial.println(AngleTurn);
    float AngleTotal = 360 - Theta;
    float FinalTurn = 2.13 * AngleTotal;
    Serial.println(FinalTurn);
    right_motor.moveFor(FinalTurn, 80);
    FreeRange = false;

    while(meetIntersection() == false)
    {
      //robot drives to the PICKUP road
      Serial.println("Finding PICKUP road");
      right_motor.setSpeed(85);
      left_motor.setSpeed(85);
    }


    if (meetIntersection() == true)
    {
      //once PICKUP road is reached, the robot turns right to the houses
      Serial.println("Found PICKUP road");
      stop();
      left_motor.moveFor(420, 100);
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