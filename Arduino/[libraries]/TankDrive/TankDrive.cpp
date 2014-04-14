// TankDrive.cpp
#include "TankDrive.h"

 TankDrive::TankDrive() {
	// Do nothing
}

 void TankDrive::initializeMotors(int motorPin1, int motorDirectionPin1, int encoderPin1, int motorPin2, int motorDirectionPin2, int encoderPin2, int gearRatio, double wheelDiameter) {
	_m1.initialize(motorPin1, motorDirectionPin1, encoderPin1, gearRatio, wheelDiameter);
	_m2.initialize(motorPin2, motorDirectionPin2, encoderPin2, gearRatio, wheelDiameter);
}

/*
 TankDrive::initializeInterrupts() {
	attachInterrupt(2, interrupt2, RISING);
	attachInterrupt(3, interrupt3, RISING);
	attachInterrupt(4, interrupt4, RISING);
	attachInterrupt(5, interrupt5, RISING);
}
*/

 void TankDrive::driveForward(int pwmMotor) {
	_m1.moveForward(pwmMotor);
	_m2.moveBackward(pwmMotor);
}

 void TankDrive::driveBackward(int pwmMotor) {
	_m1.moveBackward(pwmMotor);
	_m2.moveForward(pwmMotor);
}

 void TankDrive::stop() {
	_m1.stop();
	_m2.stop();
}

void TankDrive::rotateCW(int pwmMotor) {
	_m1.moveBackward(pwmMotor);
	_m2.moveBackward(pwmMotor);
}

void TankDrive::rotateCCW(int pwmMotor) {
	_m1.moveForward(pwmMotor);
	_m2.moveForward(pwmMotor);
}

void TankDrive::tankDrive(int pwmMotorL, int pwmMotorR) {
	
	
	if (pwmMotorL < 0) _m1.moveBackward(pwmMotorL);
	else 			   _m1.moveForward(pwmMotorL);
	
	// m2 reversed due to direction of motors on tank drive robot (forward = backwards)
	if (pwmMotorR < 0) _m2.moveForward(pwmMotorR);
	else 			   _m2.moveBackward(pwmMotorR);
}

void TankDrive::drive(int pwmMotor, double distance) {
	// Drive motors 1 and 4
	if (distance > 0) {
		_m1.move(pwmMotor, distance, true);
		_m2.move(pwmMotor, distance, false);
	} else {
		_m1.move(pwmMotor, distance, false);
		_m2.move(pwmMotor, distance, true);
	}
}

void TankDrive::countTicks1() {
	_m1._countTicks();
}

void TankDrive::countTicks2() {
	_m2._countTicks();
}
