#include <Max3421e.h>
#include <Usb.h>
#include <AndroidAccessory.h>
#include <TankDrive.h>
#include <MotorController.h>
#include <string.h>
#include <PID_v1.h>

#define LAMBDA 0.85
#define DIST_THRESHOLD_NORMAL 15
#define DIST_THRESHOLD_APPROACH 9
#define DIST_RELIABLE_LIMIT 40

char manufacturer[] = "Rose-Hulman";
char model[] = "WirelessRobotController";
char versionStr[] = "1.0";
char onMessage[] = "I have an idea!";                                                                                                                                                                                                                                                                                                                                                                                                                 
char offMessage[] = "Nope. Lost it.";

char sensorStopMessage[] = "SENSOR_STOP";
char sensorRejectMessage[] = "SENSOR_REJECT";

AndroidAccessory acc(manufacturer,
                      model,
                      "This program drives -a robot",
                      versionStr,
                      "https://sites.google.com/site/me435spring2013/",
                      "12345");

char rxBuf[255];

TankDrive drive;
MotorController intakeMotor;

int motorIntakePin = 5;
int motorIntakeDirectionPin = 4;

int motorPin1 = 3;
int motorDirectionPin1 = 12;
int encoderPin1 = 19;

int motorPin2 = 11;
int motorDirectionPin2 = 13;
int encoderPin2 = 18;

// Change these pins as necessary
int frontLeft_distanceSensorPin = A4;
int frontRight_distanceSensorPin = A3;
int rear_distanceSensorPin = A6;
int left_distanceSensorPin = A5;
int right_distanceSensorPin = A2;

double frontLeft_distance_raw;
double frontRight_distance_raw; 
double rear_distance_raw;
double left_distance_raw;
double right_distance_raw;

double frontLeft_distance = 0;
double frontRight_distance = 0; 
double rear_distance = 0;
double left_distance = 0;
double right_distance = 0;

enum DRIVE_STATE {
  STOPPED,
  FORWARD,
  BACKWARD,
  ROTATE_LEFT,
  ROTATE_RIGHT,
  DRIVING_STRAIGHT,
  APPROACH_WALL
};

DRIVE_STATE driveState = STOPPED;

// 1 = override mode, 0 = otherwise
int isOverride = 0;

//----------PID CONTROLLER STUFF----------
//Define Variables we'll be connecting to
//double PID_setpoint, PID_input, PID_output;

//Specify the links and initial tuning parameters
//PID myPID(&PID_input, &PID_output, &PID_setpoint, 2, 0.5, 0.1, DIRECT);
//----------------------------------------

void interrupt2() { 
  //drive.countTicks1(); 
}
void interrupt3() { 
  //drive.countTicks2(); 
}

void setup() {
  Serial.begin(115200);

  drive.initializeMotors(motorPin1, motorDirectionPin1, encoderPin1, motorPin2, motorDirectionPin2, encoderPin2, 47, 59);

  intakeMotor.initialize(motorIntakePin, motorIntakeDirectionPin, -1, 47, 59);

  attachInterrupt(2, interrupt2, RISING);
  attachInterrupt(3, interrupt3, RISING);

  // Distance Sensor
  pinMode(frontLeft_distanceSensorPin, INPUT);
  pinMode(frontRight_distanceSensorPin, INPUT);
  //pinMode(rear_distanceSensorPin, INPUT);
  pinMode(left_distanceSensorPin, INPUT);
  pinMode(right_distanceSensorPin, INPUT);

  delay(1500);
  acc.powerOn();
  
  //PID_input = 0;
  //PID_setpoint = 0;
}

void loop() {
  if (acc.isConnected()) {
    
    // Acquire Distance Measurements
    // 3.91e-4 = ( 0.08 (slope) )*( 5.0/1023 (ticks to volts) )
    frontLeft_distance_raw = 1/(analogRead(frontLeft_distanceSensorPin)*(3.91e-4)) - 0.42;
    frontRight_distance_raw = 1/(analogRead(frontRight_distanceSensorPin)*(3.91e-4)) - 0.42;
    //rear_distance_raw = 1/(analogRead(rear_distanceSensorPin)*(3.91e-4)) - 0.42;
    left_distance_raw = 1/(analogRead(left_distanceSensorPin)*(3.91e-4)) - 0.42;
    right_distance_raw = 1/(analogRead(right_distanceSensorPin)*(3.91e-4)) - 0.42;
    
    // Dummy variable for rear distance
    rear_distance_raw = 50;
    
    // Condition distance measurements
    if (frontLeft_distance_raw > 50 || frontLeft_distance_raw < 0)
      frontLeft_distance_raw = 50;
    if (frontRight_distance_raw > 50 || frontRight_distance_raw < 0)
      frontRight_distance_raw = 50;
    if (rear_distance_raw > 50 || rear_distance_raw < 0)
      rear_distance_raw = 50;
    if (left_distance_raw > 50 || left_distance_raw < 0)
      left_distance_raw = 50;
    if (right_distance_raw > 50 || right_distance_raw < 0)
      right_distance_raw = 50;
      
    // Use a momentum factor LAMBDA to smooth distance measurements (LAMBDA is #define-ed)
    frontLeft_distance = ( (1-LAMBDA) * frontLeft_distance_raw ) + ( LAMBDA * frontLeft_distance );
    frontRight_distance = ( (1-LAMBDA) * frontRight_distance_raw ) + ( LAMBDA * frontRight_distance );
    rear_distance = ( (1-LAMBDA) * rear_distance_raw ) + ( LAMBDA * rear_distance );
    left_distance = ( (1-LAMBDA) * left_distance_raw ) + ( LAMBDA * left_distance );
    right_distance = ( (1-LAMBDA) * right_distance_raw ) + ( LAMBDA * right_distance );
    
    // Print raw and smoothed distances for debugging purposes
    Serial.println("---------------------------------------------");
    Serial.println("Raw Distances:");
    Serial.print("Front Left = "); Serial.println(frontLeft_distance_raw); 
    Serial.print("Front Right = ");  Serial.println(frontRight_distance_raw);
    Serial.print("Rear = ");  Serial.println(rear_distance_raw);
    Serial.print("Left = "); Serial.println(left_distance_raw);
    Serial.print("Right = ");  Serial.println(right_distance_raw);
    Serial.println("\nSmoothed Distances:");
    Serial.print("Front Left = "); Serial.println(frontLeft_distance); 
    Serial.print("Front Right = ");  Serial.println(frontRight_distance);
    Serial.print("Rear = ");  Serial.println(rear_distance);
    Serial.print("Left = "); Serial.println(left_distance);
    Serial.print("Right = ");  Serial.println(right_distance);
    Serial.println("---------------------------------------------\n");
    
    
    // Stop if see a wall (when driving forward or straight w/o override)
    if ( ( frontLeft_distance < DIST_THRESHOLD_NORMAL
        || frontRight_distance < DIST_THRESHOLD_NORMAL )
      && (driveState == FORWARD || driveState == DRIVING_STRAIGHT)
      && isOverride == 0 ) {
      drive.stop();
      driveState = STOPPED;
      acc.write(sensorStopMessage, sizeof(sensorStopMessage));
    }
    if ( ( frontLeft_distance < DIST_THRESHOLD_APPROACH
        || frontRight_distance < DIST_THRESHOLD_APPROACH )
      && driveState == APPROACH_WALL && isOverride == 0 ) {
      drive.stop();
      driveState = STOPPED;
      acc.write(sensorStopMessage, sizeof(sensorStopMessage));
    }
    //  Keep this commented until we install rear sensor
    /*
    if (rear_distance < DIST_THRESHOLD_NORMAL && driveState == BACKWARD && isOverride == 0) {
      drive.stop();
      driveState = STOPPED;
      acc.write(sensorStopMessage, sizeof(sensorStopMessage));
    }
    */
    
    // Drive straight PWM control step
    if (driveState == DRIVING_STRAIGHT) {
      // If the walls are close enough to be reliable
      if ( left_distance < DIST_RELIABLE_LIMIT
        && right_distance < DIST_RELIABLE_LIMIT ) {
        double difference = left_distance - right_distance;
        int pwmL = min(220 - round(difference),255);
        int pwmR = min(220 + round(difference),255);
        
        drive.tankDrive(pwmL, pwmR); // Update drive to reflet those values
        
        // Experimental PID code
        /*
        PID_input = left_distance - right_distance;
        myPID.Compute();
        
        int pwmL = round(constrain(255 + PID_output, 200, 255));
        int pwmR = round(constrain(255 - PID_output, 200, 255));
        
        drive.tankDrive(pwmL, pwmR);
        */
      } else {
        // Stop if walls are far away
        drive.stop();
        driveState = STOPPED;
        acc.write(sensorStopMessage, sizeof(sensorStopMessage));
        // myPID.SetMode(MANUAL);
      }
    }
    
    // Read message from Android
    int len = acc.read(rxBuf, sizeof(rxBuf), 1);
    if (len > 0) {
      // Convert terminator to null character
      rxBuf[len -1] = '\0'; 
      String inputString = String(rxBuf);

      // Serial.println(inputString);

      // Break up string into command and numbers
      // String command = strtok_r(rxBuf, " ", NULL);
      // int speedMotor = atoi(strtok_r(NULL, " ", NULL));
      // int directionTheta = atoi(strtok_r(NULL, " ", NULL));
      // int distance = atoi(strtok_r(NULL, " ", NULL));  

      int firstSpace = inputString.indexOf(' ');
      int secondSpace = inputString.indexOf(' ', firstSpace + 1);

      String command = inputString.substring(0, firstSpace);

      int speedMotor = inputString.substring(firstSpace, secondSpace).toInt();
      int distance = inputString.substring(secondSpace, inputString.length()).toInt();

      int speedMotorL = speedMotor, speedMotorR = distance;

      /*
      Serial.println(command);
      Serial.print("Speed = "); 
      Serial.println(speedMotor);
      Serial.print("Distance = "); 
      Serial.println(distance);
      Serial.println(" ");
      */

      // Do something with the recieved messages
      if (command.equalsIgnoreCase("Forward")){
        if ( frontLeft_distance > DIST_THRESHOLD_NORMAL
          && frontRight_distance > DIST_THRESHOLD_NORMAL ) { 
          drive.driveForward(speedMotor);
          driveState = FORWARD;
          isOverride = 0;
        } 
        else {
          drive.stop();
          driveState = STOPPED;
          isOverride = 0;
          acc.write(sensorRejectMessage, sizeof(sensorRejectMessage));
        }
      }
      else if (command.equalsIgnoreCase("Reverse")){
        if ( rear_distance > DIST_THRESHOLD_NORMAL ) { 
          drive.driveBackward(speedMotor);
          driveState = BACKWARD;
          isOverride = 0;
        } 
        else {
          drive.stop();
          driveState = STOPPED;
          isOverride = 0;
          acc.write(sensorRejectMessage, sizeof(sensorRejectMessage));
        }
      }
      else if (command.equalsIgnoreCase("Forward_Override")){
        drive.driveForward(speedMotor);
        driveState = FORWARD;
        isOverride = 1;
      }
      else if (command.equalsIgnoreCase("Reverse_Override")){
        drive.driveBackward(speedMotor);
        driveState = BACKWARD;
        isOverride = 1;
      }
      else if (command.equalsIgnoreCase("Rotate_CW")){
        
        // Uncomment to make robot stop rotating on side proximity
        /*
        if (right_distance > 10) { 
          drive.rotateCCW(speedMotor);
          driveState = ROTATE_RIGHT;
          isOverride = 0;
        }
        else {
          drive.stop();
          driveState = STOPPED;
          isOverride = 0;
          acc.write(sensorRejectMessage, sizeof(sensorRejectMessage));
        }
        */
        
        drive.rotateCCW(speedMotor);
        driveState = ROTATE_RIGHT;
        isOverride = 0;
      }
      else if (command.equalsIgnoreCase("Rotate_CCW")){
        
        // Uncomment to make robot stop rotating on side proximity
        /*
        if (left_distance > 10) { 
          drive.rotateCW(speedMotor);
          driveState = ROTATE_LEFT;
          isOverride = 0;
        }
        else {
          drive.stop();
          driveState = STOPPED;
          isOverride = 0;
          acc.write(sensorRejectMessage, sizeof(sensorRejectMessage));
        }
        */
        
        drive.rotateCW(speedMotor);
        driveState = ROTATE_LEFT;
        isOverride = 0;
      }
      else if (command.equalsIgnoreCase("Stop")) {
        drive.stop();
        driveState = STOPPED;
        isOverride = 0;
      }
      else if (command.equalsIgnoreCase("all_stop")) {
        drive.stop();
        intakeMotor.stop();
        driveState = STOPPED;
        isOverride = 0;
      }
      else if (command.equalsIgnoreCase("Drive_distance")) {
        // If command is to move forward
        if (distance > 0) {
          if ( frontLeft_distance > DIST_THRESHOLD_NORMAL
            && frontRight_distance > DIST_THRESHOLD_NORMAL ) { 
            drive.drive(speedMotor, distance);
            driveState = FORWARD;
            isOverride = 0;
          } 
          else {
            drive.stop();
            driveState = STOPPED;
            isOverride = 0;
            acc.write(sensorRejectMessage, sizeof(sensorRejectMessage));
          }
          // If command is to move backward
        } 
        else if (distance < 0) {
          if ( rear_distance > DIST_THRESHOLD_NORMAL ) { 
            drive.drive(speedMotor, distance);
            driveState = BACKWARD;
            isOverride = 0;
          } 
          else {
            drive.stop();
            driveState = STOPPED;
            isOverride = 0;
            acc.write(sensorRejectMessage, sizeof(sensorRejectMessage));
          }
        }
      }
      else if (command.equalsIgnoreCase("Drive_distance_Override")) {
        // If command is to move forward
        if (distance > 0) {
          drive.drive(speedMotor, distance);
          driveState = FORWARD;
          isOverride = 1;
        // If command is to move backward
        } 
        else if (distance < 0) {
          drive.drive(speedMotor, distance);
          driveState = BACKWARD;
          isOverride = 1;
        }
      }
      else if (command.equalsIgnoreCase("Intake")) {
        intakeMotor.moveForward(speedMotor);
      }
      else if (command.equalsIgnoreCase("Outtake")) {
        intakeMotor.moveBackward(speedMotor);
      }
      else if (command.equalsIgnoreCase("Stop_Intake")) {
        intakeMotor.stop();
      }
      else if (command.equalsIgnoreCase("Tank_Drive")) {
        drive.tankDrive(speedMotorL, speedMotorR);
        isOverride = 0;
      }
      else if (command.equalsIgnoreCase("Drive_Straight")) {
        driveState = DRIVING_STRAIGHT;
        isOverride = 0;
       // myPID.SetMode(AUTOMATIC);
      }
      else if (command.equalsIgnoreCase("Approach_Wall")) {
        if ( frontLeft_distance > DIST_THRESHOLD_APPROACH 
          && frontRight_distance > DIST_THRESHOLD_APPROACH ) { 
          drive.driveForward(speedMotor);
          driveState = APPROACH_WALL;
          isOverride = 0;
        } 
        else {
          drive.stop();
          driveState = APPROACH_WALL;
          isOverride = 0;
          acc.write(sensorRejectMessage, sizeof(sensorRejectMessage));
        }
      }
    }  
  } 
  else {
    // All stop if acc is not connected
    drive.stop();
    intakeMotor.stop();
    driveState = STOPPED;
    isOverride = 0;
  }
}
        

