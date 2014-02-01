/*******************************************************************************
* autonomous.h
* autonomous defines 
*******************************************************************************/
#ifndef __autonomous_h_
#define __autonomous_h_

//state machine
#define AUTO_CASE_MACHINE 	switch(autonomous_case){
#define END_AUTONOMOUS		}


#define AUTO_STATE_EXEC(x) 	case x:
#define END_AUTO_STATE		break		

#define AUTO_MODE_GOTO(x) 	autonomous_case = x

//autobuttons
#define ab_startMid				rc_dig_in14
#define ab_skipFirst			rc_dig_in15
#define ab_delay				rc_dig_in16
#define ab_doNothing			rc_dig_in17

//states
#define AUTO_INITIALIZATION 	0
#define DRIVE_STRAIGHT 			1
#define AUTO_SMART_CORRECT		2
#define TURN_LEFT_INIT 			3
#define TURN_LEFT 				4
#define WAIT_FOR_WALL 			5
#define AUTO_NULL				6

/******************
AUTO INITIALIZATION	(Drive out to the beginning of the wall)
******************/
#define DIST_GOTO_DRIVE_STRAIGHT	-1800

//for position 2 (middle) (10ths of a degree)
#define THETA_NEAR		-100
#define THETA_MID		0
#define THETA_FAR		100

//for position 1 (left) (the 2 is sorta contradictory)
#define THETA2_NEAR		0
#define THETA2_MID		100
#define THETA2_FAR		200

/******************
DRIVE STRAIGHT		(Correct for ball position)
******************/
//250 ticks = 22.3 inches, so about 10 ticks per inch
#define DIST_ITER_ONE 		-6000
#define DIST_ITER_TWO 		-3700

//wait constant when deciding off of the wall
#define DRIVE_STRAIGHT_WAIT 100

/******************
AUTO SMART CORRECT	(Ball position setup, raise arm)
******************/
//sonar sensors (1 for near, 2 for far); 4 ticks per inch
#define BALL_NEAR		134 // 96
#define BALL_MID		286 // 278
#define BALL_FAR		436 // 474
#define BALL2_NEAR		134
#define BALL2_MID		286
#define BALL2_FAR		436

//turn offsets in wheel encoder ticks
#define OFF_33 			0
#define OFF_31 			-350
#define OFF_13 			0

//arm height in encoder ticks (256ths of a circle)
//~10 counts per inch for 256 to 200 (remember sin wave)												
#define ARM_HEIGHT 		240

/******************
TURN LEFT INIT		(Setup of turning parameters)
******************/
#define TURN_SPEED		-32
#define DESIRED_OMEGA	40
#define TURN_COMPENSATION_K 28  //UP IS TIGHTER, DOWN IS LOOSER

//Theta offsets for tricky balls (negative = looser curve)
//beware of desired_theta.  THESE VALUES ARE SENSITIVE
#define FIX_33	3
#define FIX_31	3
#define FIX_13	0

/******************
TURN LEFT			(Do the arc)
******************/
//acceleration constant through the turn (angle/this number)
//lower number = more acceleration
#define TURN_ACCEL_K 55

/******************
WAIT FOR WALL		(Wait for the wall in the case that the turn completes before it)
******************/
//no constants i guess

/******************
GENERAL
******************/
#define WALL_MAX 		700
#define STRAIGHT_SPEED -65
#define GYRO_BIAS 		994

#endif


