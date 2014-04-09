// TankDrive.h
#ifndef TANKDRIVE_H
#define TANKDRIVE_H

#include <math.h>

#include "MotorController.h"
#include "Arduino.h"

/* TODO: make functions for rotate(boolean rotateCW), drive(int speed, enum Direction), 
drive(int speed, double distance, enum Direction) or int x, int y or double x, double y

*/

// Motor Numbering system
/*
		  Front
Left	1		2	Right
		  Back
*/

// Angle 0 to the front, Positive to the right (clockwise)

class TankDrive {
public: 
	TankDrive();
	void initializeMotors(int, int, int, int, int, int, int, double);
	// void initializeInterrupts();
	void driveForward(int);
	void driveBackward(int);
	void driveLeft(int);
	void driveRight(int);
	void tankDrive(int, int);
	void stop();
	void rotateCW(int);
	void rotateCCW(int);
	void drive(int, double);
	
	void countTicks1();
	void countTicks2();

private:
	MotorController _m1;
	MotorController _m2;
};

#endif