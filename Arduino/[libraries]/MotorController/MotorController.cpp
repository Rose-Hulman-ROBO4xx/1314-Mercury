// motorController.cpp
#include "MotorController.h"

// Constructors
MotorController::MotorController() {
	// Empty
}

// Set up
void MotorController::initialize(int motorPin, int motorDirectionPin, int encoderPin, int gearRatio, double wheelDiameter) {
	_motorPin = motorPin;
	_motorDirectionPin = motorDirectionPin;
	_encoderPin = encoderPin;
	_runForDistance = false;
	
	pinMode(_motorDirectionPin, OUTPUT);
	pinMode(_encoderPin, INPUT);
	
	_distanceForOneRotation = M_PI * wheelDiameter;
	_ticksForOneRotation = floor(gearRatio * ENCODER_ONE_ROTATION);
	_ticks = 0;
}
/*
void setupInterrupt(int interruptNumber) {
	attachInterrupt(interruptNumber, _countTicks, CHANGE);
}
 */
 
// Public Functions
void MotorController::move(int pwmDutyCycle, double distanceInMeters, boolean moveForward) {
	double numRotations = distanceInMeters/_distanceForOneRotation;
	_motorSpeed = pwmDutyCycle;
	
	if (moveForward) {
		digitalWrite(_motorDirectionPin, HIGH); // Set direction to forward
	} else {
		digitalWrite(_motorDirectionPin, LOW); // Set direction to backward
	}
	
	analogWrite(_motorPin, pwmDutyCycle);
	_ticks = 0;
	_runForDistance = true;
	_runDistanceTicks = floor(numRotations * _ticksForOneRotation);
}

void MotorController::moveForward(int pwmDutyCycle) {
	_motorSpeed = pwmDutyCycle;
	digitalWrite(_motorDirectionPin, HIGH); // Set direction to forward
	analogWrite(_motorPin, pwmDutyCycle);  // PWM Speed Control of motor 1
}

void MotorController::moveBackward(int pwmDutyCycle) {
	_motorSpeed = pwmDutyCycle;
	digitalWrite(_motorDirectionPin, LOW); // Set direction to backward
	analogWrite(_motorPin, pwmDutyCycle);  // PWM Speed Control of motor 1
}

void MotorController::stop(){
	_motorSpeed = 0;
	analogWrite(_motorPin, 0);  // Stop the motor
}

// Private Functions
void MotorController::_countTicks() {
	_ticks++;
	if (_runForDistance == true && _ticks > _runDistanceTicks) {
		stop();
		_runForDistance = false;
	}
}