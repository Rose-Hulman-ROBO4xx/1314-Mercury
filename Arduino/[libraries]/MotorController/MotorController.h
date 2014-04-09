// motorController.h
#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#define _USE_MATH_DEFINES
#include <math.h>
#include "Arduino.h"

#define ENCODER_ONE_ROTATION 12

class MotorController {
public:
	MotorController();
	void initialize(int motorPin, int motorDirectionPin, int encoderPin, int gearRatio, double wheelDiameter);
	// void setupInterrupt(int interruptNumber);
	void move(int pwmDutyCycle, double distance, boolean moveForward);
	void moveForward(int pwmDutyCycle);
	void moveBackward(int pwmDutyCycle);
	void stop();

	void _countTicks();
	
private:
	int _ticksForOneRotation;
	double _distanceForOneRotation;
	int _motorPin;
	int _motorDirectionPin;
	int _encoderPin;
	volatile long _ticks;
	volatile int _motorSpeed;
	volatile boolean _runForDistance;
	volatile long _runDistanceTicks;
};

#endif