/** @file init.c
 * @brief File for initialization code
 *
 * This file should contain the user initialize() function and any functions related to it.
 *
 * Any copyright is dedicated to the Public Domain.
 * http://creativecommons.org/publicdomain/zero/1.0/
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

/*
 * Runs pre-initialization code. This function will be started in kernel mode one time while the
 * VEX Cortex is starting up. As the scheduler is still paused, most API functions will fail.
 *
 * The purpose of this function is solely to set the default pin modes (pinMode()) and port
 * states (digitalWrite()) of limit switches, push buttons, and solenoids. It can also safely
 * configure a UART port (usartOpen()) but cannot set up an LCD (lcdInit()).
 */
 void initializeIO() {
   for(int i=1; i<11; i++){
     pinMode(i, INPUT);
   }
 }

 /*
  * Runs user initialization code. This function will be started in its own task with the default
  * priority and stack size once when the robot is starting up. It is possible that the VEXnet
  * communication link may not be fully established at this time, so reading from the VEX
  * Joystick may fail.
  *
  * This function should initialize most sensors (gyro, encoders, ultrasonics), LCDs, global
  * variables, and IMEs.
  *
  * This function must exit relatively promptly, or the operatorControl() and autonomous() tasks
  * will not start. An autonomous mode selection menu like the pre_auton() in other environments
  * can be implemented in this task if desired.
  */
 void initialize() {
   driveR = encoderInit(encR1, encR2, false);
   driveL = encoderInit(encL1, encL2, false);
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
   return motors[speed];
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
 	motorSet(liftL, power);
 	motorSet(liftR, -power);
 }

 void intakeSet(int power){
 	motorSet(intake, power);
 }

 void mogoSet(int power){
 	motorSet(mogo, -power);
 }
