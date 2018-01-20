/** @file opcontrol.c
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * Any copyright is dedicated to the Public Domain.
 * http://creativecommons.org/publicdomain/zero/1.0/
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

/*
 * Runs the user operator control code. This function will be started in its own task with the
 * default priority and stack size whenever the robot is enabled via the Field Management System
 * or the VEX Competition Switch in the operator control mode. If the robot is disabled or
 * communications is lost, the operator control task will be stopped by the kernel. Re-enabling
 * the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will
 * run the operator control task. Be warned that this will also occur if the VEX Cortex is
 * tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and
 * the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */
 int intakeTarget = intakeTop;
 int liftTarget = floorHeight;
 bool liftPidEnabled = false;
 int conesStacked = 0;
 int liftSpeeds[2] = {0,0};
 bool lastBtn8r = false;
 int stackStatus = 0;
 bool autoStackEnabled = false;

void manualLift();
void driveControl();
void mogoControl();
void intakeControl();
void liftControl();
void liftPid();
void autoStack();
void accelTo(int port, int speed);
void intakePid();

void operatorControl() {
	while (1) {
		// Drive
		driveControl();

		// Mogo
		mogoControl();

		bool btn8r = joystickGetDigital(1, 8, JOY_RIGHT);
		if(btn8r && !lastBtn8r){
			autoStackEnabled = !autoStackEnabled;
		}

		if(autoStackEnabled){
			autoStack();
      lcdSetText(uart1, 1, "Auto Stacking");
		}
		else {
      lcdSetText(uart1, 1, "Manual Mode");
      lcdSetText(uart1, 2, "Btn8r to toggle");
			intakeControl();
			liftControl();
		}

		// lcdPrint(uart1, 1, "L: %d", analogRead(liftL));
		// lcdPrint(uart1, 2, "R: %d", analogRead(liftR));
		lastBtn8r = btn8r;
		delay(20);
	}
}

void autoStack(){
	bool btn5u = joystickGetDigital(1, 5, JOY_UP);
	bool btn5d = joystickGetDigital(1, 5, JOY_DOWN);
  bool btn8u = joystickGetDigital(1, 8, JOY_UP);
  bool btn8d = joystickGetDigital(1, 8, JOY_DOWN);
	int topPot = 0;
	static bool lastBtn5d = false;
	static bool lastBtn5u = false;
	int liftPot = analogRead(liftL);
  if(btn8u) stackStatus = 5;
  if(btn8d) stackStatus = 0;
	switch(stackStatus){
		case 0: // Hovering
			liftTarget = hoverHeight;
			liftPid();
			intakeTarget = intakeBottom;
			intakePid();
			if(btn5u) stackStatus = 2;
			if(btn5d && !lastBtn5d) stackStatus = 1;
      lcdSetText(uart1, 2, "Hovering");
			break;
		case 1: // Plunging
			liftSet(-127);
			intakeTarget = intakeBottom;
			intakePid();
			if(liftPot < floorHeight || btn5u) stackStatus = 0;
      lcdSetText(uart1, 2, "Plunging");
			break;
		case 2: // Lifting
			manualLift();
			intakeTarget = intakeBottom;
			intakePid();
			if(!btn5u) stackStatus = 3;
      lcdSetText(uart1, 2, "Lifting");
			break;
		case 3: // Pre-Stacking
      manualLift();
			topPot = liftPot;
			intakeTarget = intakeTop;
			intakePid();
			if(btn5u) stackStatus = 2;
			else if(btn5d) stackStatus = 4;
      lcdSetText(uart2, 2, "Pre-stacking");
			break;
		case 4: // Stacking
			liftTarget = hoverHeight;
			liftPid();
			intakeTarget = intakeTop;
			intakePid();
			if(liftPot < (topPot - 200)) stackStatus = 0;
      lcdSetText(uart1, 2, "Stacking");
      break;
    case 5: // Scoring Mogo
      liftTarget = fullHeight;
      liftPid();
      if(liftPot > (fullHeight - 200)) intakeTarget = intakeTop;
      intakePid();
      if(btn5d) stackStatus = 0;
      lcdSetText(uart1, 2, "Scoring Mogo");
      break;
	}
	lastBtn5d = btn5d;
	lastBtn5u = btn5u;
}

void accelTo(int port, int speed){
	int curSpeed = motorGet(port);
	int pwr = curSpeed - speed;
	const int slew = 10;
	if(pwr > slew) pwr = slew;
	else if(pwr < -slew) pwr = -slew;
	motorSet(port, pwr);
}

void manualLift(){
	lcdSetText(uart1, 1, "Manual Lift");
	int btn5u = joystickGetDigital(1, 5, JOY_UP);
	int btn5d = joystickGetDigital(1, 5, JOY_DOWN);
	liftSet((btn5u - btn5d) * 127);
}

void liftControl(){
	bool btn8d = joystickGetDigital(1, 8, JOY_DOWN);
	bool btn8u = joystickGetDigital(1, 8, JOY_UP);
	bool btn8l = joystickGetDigital(1, 8, JOY_LEFT);
	bool btn5u = joystickGetDigital(1, 5, JOY_UP);
	bool btn5d = joystickGetDigital(1, 5, JOY_DOWN);
	if(btn8d){
		liftTarget = hoverHeight;
		liftPidEnabled = true;
	}
	else if(btn8l){
		liftTarget = halfHeight;
		liftPidEnabled = true;
	}
	else if(btn8u){
		liftTarget = fullHeight;
		liftPidEnabled = true;
	}
	else if(btn5u || btn5d){
		liftPidEnabled = false;
	}
	if(liftPidEnabled) liftPid();
	else manualLift();
}

void liftPid(){
	lcdSetText(uart1, 1, "PID enabled");
	const double kp = 0.3;
	const double ki = 0.0;
	const double kd = 2.0;

	static double integL;
	static double integR;
	int potL = analogRead(liftL)-15;
	int potR = analogRead(liftR);
	int errorL = liftTarget - potL;
	int errorR = liftTarget - potR;

	static int prevErrorL;
	static int prevErrorR;
	int deltaErrorL = (errorL - prevErrorL);
	int deltaErrorR = (errorR - prevErrorR);

	lcdPrint(uart1, 2, "T %d E %d", liftTarget, errorL);

	if(abs(errorL) < 400){
		integL += errorL;
	}
	else{
		deltaErrorL *= 0.4;
		integL = 0;
	}

	if(abs(errorR) < 400){
		integR += errorR;
	}
	else{
		deltaErrorR *= 0.4;
		integR = 0;
	}

	int pwrL = (errorL * kp) + (integL * ki) + (deltaErrorL * kd);
	int pwrR = (errorR * kp) + (integR * ki) + (deltaErrorR * kd);
	liftSetInd(pwrL, pwrR);
	prevErrorL = errorL;
	prevErrorR = errorR;
}

void driveControl(){
	int leftStick = joystickGetAnalog(1, 3);
	int rightStick = joystickGetAnalog(1, 2);
	driveSet(leftStick, -rightStick);
}

void mogoControl(){
	int btn7l = joystickGetDigital(1, 7, JOY_LEFT);
	int btn7d = joystickGetDigital(1, 7, JOY_DOWN);
	bool allInBtn = digitalRead(mogoInBtn);
	if(!allInBtn) btn7l = 0;
	mogoSet((btn7d - btn7l) * 127);
}

void intakeControl(){
	bool btn6u = joystickGetDigital(1, 6, JOY_UP);
	bool btn6d = joystickGetDigital(1, 6, JOY_DOWN);
	if(btn6u) intakeTarget = intakeTop;
	if(btn6d) intakeTarget = intakeBottom;
	intakePid();
	// lcdPrint(uart1, 2, "%d, %d, %d", int(error * kp), int(integ * ki), int(deltaError * kd));
}

void intakePid(){
	static double integ;
	int pot = analogRead(inPot);
	int error = intakeTarget - pot;
	static int prevError;
	const double kp = 0.2;
	const double ki = 0.0;
	const double kd = 0.0;
	int deltaError = error - prevError;
	if(abs(error) < 100){
		integ += error;
	}
	else{
		integ = 0;
	}

	int pwr = (error * kp) + (integ * ki) + (deltaError * kd);
	intakeSet(pwr);
	prevError = error;
}

int linSpeed(int speed){
	const unsigned int motors[128] =
	{
		0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
		0, 21, 21, 21, 22, 22, 22, 23, 24, 24,
	 25, 25, 25, 25, 26, 27, 27, 28, 28, 28,
	 28, 29, 30, 30, 30, 31, 31, 32, 32, 32,
	 33, 33, 34, 34, 35, 35, 35, 36, 36, 37,
	 37, 37, 37, 38, 38, 39, 39, 39, 40, 40,
	 41, 41, 42, 42, 43, 44, 44, 45, 45, 46,
	 46, 47, 47, 48, 48, 49, 50, 50, 51, 52,
	 52, 53, 54, 55, 56, 57, 57, 58, 59, 60,
	 61, 62, 63, 64, 65, 66, 67, 67, 68, 70,
	 71, 72, 72, 73, 74, 76, 77, 78, 79, 79,
	 80, 81, 83, 84, 84, 86, 86, 87, 87, 88,
	 88, 89, 89, 90, 90,127,127,127
 };
 int val = 0;
 if(speed < 0) val = -motors[abs(speed)];
 else val = motors[speed];
 return val;
}

void driveSet(int left, int right){
 left = linSpeed(left);
 right = linSpeed(right);
 motorSet(driveOutL, -left);
 motorSet(driveMidL, left);
 motorSet(driveOutR, -right);
 motorSet(driveMidR, right);
}

void liftSet(int power){
	power = rectify(power, 127, -127);
	power = linSpeed(power);
 motorSet(liftTL, power);
 motorSet(liftBL, power);
 motorSet(liftTR, -power);
 motorSet(liftBR, -power);
}

void liftSetInd(int left, int right){
	left = rectify(left, 127, -127);
	left = linSpeed(left);
	right = rectify(right, 127, -127);
	right = linSpeed(right);
 motorSet(liftTL, left);
 motorSet(liftBL, left);
 motorSet(liftTR, -right);
 motorSet(liftBR, -right);
}

void intakeSet(int power){
	power = rectify(power, 127, -127);
	power = linSpeed(power);
	motorSet(intake, power);
}

void mogoSet(int power){
	power = rectify(power, 127, -127);
	power = linSpeed(power);
	motorSet(mogo, -power);
}

void mset(int port, int power){
	power = rectify(power, 127, -127);
 //  int val = motorGet(port);
 //  int diff = power-val;
 //  diff = rectify(diff, 10, -10);
 //  power += diff;
	motorSet(port, power);
}

int rectify(int val, int highbound, int lowbound){
	if (val > highbound) val = highbound;
	else if(val < lowbound) val = lowbound;
	return val;
}
