/*******************************************************************************
* FILE NAME: user_routines_fast.c <FRC VERSION>
*
* DESCRIPTION:
*  This file is where the user can add their custom code within the framework
*  of the routines below. 
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
* OPTIONS:  Interrupts are disabled and not used by default.
*
*******************************************************************************/

#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "user_Serialdrv.h"
#include "encoder.h"
#include "motor_speed.h"
#include "pid.h"
#include "autonomous.h"
#include "adc.h"
#include "gyro.h"



/*** DEFINE USER VARIABLES AND INITIALIZE THEM HERE ***/

/*******************************************************************************
* FUNCTION NAME: InterruptVectorLow
* PURPOSE:       Low priority interrupt vector
* CALLED FROM:   nowhere by default
* ARGUMENTS:     none
* RETURNS:       void
* DO NOT MODIFY OR DELETE THIS FUNCTION 
*******************************************************************************/
#pragma code InterruptVectorLow = LOW_INT_VECTOR
void InterruptVectorLow (void)
{
  _asm
    goto InterruptHandlerLow  /*jump to interrupt routine*/
  _endasm
}


/*******************************************************************************
* FUNCTION NAME: InterruptHandlerLow
* PURPOSE:       Low priority interrupt handler
* If you want to use these external low priority interrupts or any of the
* peripheral interrupts then you must enable them in your initialization
* routine.  Innovation First, Inc. will not provide support for using these
* interrupts, so be careful.  There is great potential for glitchy code if good
* interrupt programming practices are not followed.  Especially read p. 28 of
* the "MPLAB(R) C18 C Compiler User's Guide" for information on context saving.
* CALLED FROM:   this file, InterruptVectorLow routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
#pragma code
#pragma interruptlow InterruptHandlerLow save=PROD /* You may want to save additional symbols. */

void InterruptHandlerLow () {      
	unsigned char Port_B;
	unsigned char Port_B_Delta;
                         
	if (INTCON3bits.INT2IF && INTCON3bits.INT2IE) // encoder 1 interrupt?
	{ 
		INTCON3bits.INT2IF = 0; // clear the interrupt flag
		#ifdef ENABLE_ENCODER_1
		Encoder_1_Int_Handler(); // call the left encoder interrupt handler (in encoder.c)
		#endif
	}
	else if (INTCON3bits.INT3IF && INTCON3bits.INT3IE) // encoder 2 interrupt?
	{
		INTCON3bits.INT3IF = 0; // clear the interrupt flag
		#ifdef ENABLE_ENCODER_2
		Encoder_2_Int_Handler(); // call right encoder interrupt handler (in encoder.c)
		#endif
	}else if(PIR1bits.TMR1IF && PIE1bits.TMR1IE) {
		PIR1bits.TMR1IF = 0;
		//printf("i do things");
		//timer_clock++;
		Timer_1_Int_Handler();
	}else if(PIR1bits.TMR2IF && PIE1bits.TMR2IE) // timer 2 interrupt?
	{
		PIR1bits.TMR2IF = 0; // clear the timer 2 interrupt flag [92]
		Timer_2_Int_Handler(); // call the timer 2 interrupt handler (in adc.c)
	}                     
	else if(PIR1bits.ADIF && PIE1bits.ADIE) // ADC interrupt
	{
		PIR1bits.ADIF = 0; // clear the ADC interrupt flag
		ADC_Int_Handler(); // call the ADC interrupt handler (in adc.c)
	}else if (INTCONbits.RBIF && INTCONbits.RBIE) // encoder 3-6 interrupt?
	{
		Port_B = PORTB; // remove the "mismatch condition" by reading port b            
		INTCONbits.RBIF = 0; // clear the interrupt flag
		Port_B_Delta = Port_B ^ Old_Port_B; // determine which bits have changed
		Old_Port_B = Port_B; // save a copy of port b for next time around
	 
		if(Port_B_Delta & 0x10) // did external interrupt 3 change state?
		{
			#ifdef ENABLE_ENCODER_3
			Encoder_3_Int_Handler(Port_B & 0x10 ? 1 : 0); // call the encoder 3 interrupt handler (in encoder.c)
			#endif
		}
	} 
}


/*******************************************************************************
* FUNCTION NAME: User_Autonomous_Code
* PURPOSE:       Execute user's code during autonomous robot operation.
* You should modify this routine by adding code which you wish to run in
* autonomous mode.  It will be executed every program loop, and not
* wait for or use any data from the Operator Interface.
* CALLED FROM:   main.c file, main() routine when in Autonomous mode
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Autonomous_Code(void)
{
	//core autonomous variables
	int int_left = 0, int_right = 0; 		//speed control RL
	int distance = 0, angle_rate = 0, ahead = 0;		//Sensor Feedback Storage
	//int prevDistance = 0;
	int curDesDist = 0;
	long int angle = 0;		//More Sensor Storage
	long int wavg2 = 0;
	int omega = 0;
	char closed_loop_drive = 0;				//Closed loop drive toggle
	int counter = 0, counter_verify = 0, counter_verify2 = 0, counter_IR = 0, counter_stall = 0;	//loop counters
	char autonomous_case = 0;				//state machine
	char runIteration = 0;
	char changeGear = 0;
	int distFromWallAvg[4] = {0,0,0,0};
	int turnOffset = 0;
	//int runAvgNoWall = 0;
	
	//variables for distance correcting 
	int desDistClose = BALL_MID, desDistFar = BALL_MID;
	int desArmPos = 0;
	char firstIR = 0, secondIR = 0;
	char KILLAUTO = 0;

	//turning stuff
	//long wheelInner = 0, wheelOuter = 0;
	//DT_PID turn_correction;
	
	//init_pid(&turn_correction, 100, 0, 0, 250, 0);
	
	//Resets
	pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
	pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;
	relay1_fwd = relay1_rev = relay2_fwd = relay2_rev = 0;
	relay3_fwd = relay3_rev = relay4_fwd = relay4_rev = 0;
	relay5_fwd = relay5_rev = relay6_fwd = relay6_rev = 0;
	relay7_fwd = relay7_rev = relay8_fwd = relay8_rev = 0;

	Set_Gyro_Bias(GYRO_BIAS);
	Reset_Gyro_Angle();

	while (autonomous_mode){			//DO NOT CHANGE!!!
		distance = Get_ADC_Result(2);
		//ahead = Get_ADC_Result(3);
		angle_rate = Get_Gyro_Rate();
		
		if (statusflag.NEW_SPI_DATA){	//slow loop
			Getdata(&rxdata);   		//DO NOT DELETE, or you will be stuck here forever!
			angle = Get_Gyro_Angle() + runIteration*1800;

			//program the autonomous states
			if (irButton1) {
				if (!firstIR) {
					firstIR = 1;
					counter_IR = counter;
					desDistClose = BALL_NEAR;
				}else if (firstIR && counter > counter_IR + 17) {
					desDistFar = BALL2_NEAR;
					secondIR = 1;
				}
			}
			if (irButton2) {
				if (!firstIR) {
					firstIR = 1;
					counter_IR = counter;
					desDistClose = BALL_MID;
				}else if (firstIR && counter > counter_IR + 17) {
					desDistFar = BALL2_MID;
					secondIR = 1;
				}
			}
			if (irButton3) {
				if (!firstIR) {
					firstIR = 1;
					counter_IR = counter;
					desDistClose = BALL_FAR;
				}else if (firstIR && counter > counter_IR + 17) {
					desDistFar = BALL2_FAR;
					secondIR = 1;
				}
			}
			if (irButton4) {
				KILLAUTO = 1;
			}

			AUTODEBUG = wavg2;
			
			if (!KILLAUTO && !ab_doNothing) {	
			AUTO_CASE_MACHINE
				//initialize state: delay (if neccesary), drive till wall
				AUTO_STATE_EXEC(AUTO_INITIALIZATION);
					closed_loop_drive = 1;
					//printf("INITIALIZE\r\n");
					changeGear = 1;

					//delay...
					if (!ab_delay || counter > 40) {
						if ((left_wheel_pos + right_wheel_pos)/2 > DIST_GOTO_DRIVE_STRAIGHT) {
							int desTheta = 0;
							if (!ab_startMid) {
								switch (desDistClose) {
									case BALL_NEAR: desTheta = THETA_NEAR;		break;
									case BALL_MID: desTheta = THETA_MID;		break;
									case BALL_FAR: desTheta = THETA_FAR;		break;
									default: desTheta = THETA_MID;				break;
								}
							}else{
								switch (desDistClose) {
									case BALL_NEAR: desTheta = THETA2_NEAR;		break;
									case BALL_MID: desTheta = THETA2_MID;		break;
									case BALL_FAR: desTheta = THETA2_FAR;		break;
									default: desTheta = THETA_MID;				break;
								}
							}

							desSpeedL = STRAIGHT_SPEED + (angle - desTheta)/5;
							desSpeedR = STRAIGHT_SPEED - (angle - desTheta)/5;
						}else{
							AUTO_MODE_GOTO(AUTO_SMART_CORRECT);
						}
					}
				END_AUTO_STATE;

				//smart correct: set correction distances & arm positions
				AUTO_STATE_EXEC(AUTO_SMART_CORRECT);
					if (runIteration <= 1) {
						if (runIteration == 0 && firstIR) {
							curDesDist = desDistClose;
							if (!ab_skipFirst) {
								desArmPos = ARM_HEIGHT;
							}
						}else if (runIteration == 1 && secondIR) {
							curDesDist = desDistFar;
							desArmPos = ARM_HEIGHT;
						}else if (runIteration == 2) {
							curDesDist = BALL_NEAR;
							desArmPos = 0;
						}else{
							curDesDist = BALL_MID;
							desArmPos = 0;//ARM_HEIGHT;
						}
					}else{
						curDesDist = BALL_MID;
						desArmPos = 0;//ARM_HEIGHT;
					}

					if (desDistFar == BALL2_FAR && desDistClose == BALL_FAR) { //3 3
						turnOffset = OFF_33;
					}else if (desDistFar == BALL2_NEAR && desDistClose == BALL_FAR) { //3 1
						turnOffset = OFF_31;
					}else if (desDistFar == BALL2_FAR && desDistClose == BALL_NEAR) { //1 3
						turnOffset = OFF_13;
					}

					if (runIteration != 0) turnOffset = 0;

					counter_verify2 = counter;
					wavg2 = left_wheel_pos;
					AUTO_MODE_GOTO(DRIVE_STRAIGHT);
				END_AUTO_STATE;

				//drive straight, follow the wall, stay straight, proceed at no wall
				AUTO_STATE_EXEC(DRIVE_STRAIGHT);
					changeGear = 1;
					closed_loop_drive = 1;
					//AUTODEBUG = 4*(right_wheel_pos / left_wheel_pos + left_wheel_pos)*left_wheel_pos;
				
						
					if ( (runIteration == 0 && left_wheel_pos > (DIST_ITER_ONE + turnOffset) ) || (runIteration == 1 && left_wheel_pos > (DIST_ITER_TWO + wavg2) ) || (runIteration > 1 && distance < WALL_MAX) ) {//distance < WALL_MAX) {//if (1) {
						int desAngle = 0, distCor, angCor;
						//relay6_fwd = 1;
						
						//User_Byte5 = curDesDist/10;

						distCor = (curDesDist - distance)/9;
						angCor = (angle)/((abs(angle) < 670) ? 7 +  abs(curDesDist - distance)/15 : 4);

						pwm14 = 127 + distCor;
						pwm15 = 127 + angCor;

						//relay6_fwd = 1;
						//distVecL = STRAIGHT_SPEED - distCor;
						//distVecR = STRAIGHT_SPEED + distCor;

						//add the vectors, divide the angle correction as distance increases
						desSpeedL = STRAIGHT_SPEED - distCor + angCor;
						desSpeedR = STRAIGHT_SPEED + distCor - angCor;

						pwm13 = 127 + desSpeedL;
						pwm16 = 127 + desSpeedR;
						
						counter_verify = counter;

						//wall distance average datas
						distFromWallAvg[3] = distFromWallAvg[2];
						distFromWallAvg[2] = distFromWallAvg[1];
						distFromWallAvg[1] = distFromWallAvg[0];
						distFromWallAvg[0] = distance;
					}else {		
						if (runIteration < 2 || (counter > counter_verify + 1 && counter > counter_verify2 + DRIVE_STRAIGHT_WAIT)) {
							AUTO_MODE_GOTO(TURN_LEFT_INIT);
							//relay6_fwd = 0;
						}
					}
					
				END_AUTO_STATE;

				//initialize turning
				AUTO_STATE_EXEC(TURN_LEFT_INIT);
					//printf("TLEFT\r\n");
					closed_loop_drive = 1;

					{//implicit block definition
						int omgComp2, superFix;
						
						//omega = 30 is optimal for 2,2; +3 moves about .75 of a ball \=
						if (desDistFar == BALL2_FAR && desDistClose == BALL_FAR) { //3 3
							superFix = FIX_33;
						}else if (desDistFar == BALL2_NEAR && desDistClose == BALL_FAR) { //3 1
							superFix = FIX_31;
						}else if (desDistFar == BALL2_FAR && desDistClose == BALL_NEAR) { //1 3
							superFix = FIX_13;
						}
	
						if (runIteration != 0) superFix = 0;
	
						if (runIteration == 0) {
							omgComp2 = desDistFar;
						}else{
							omgComp2 = BALL_NEAR;
						}
	
						omega = DESIRED_OMEGA - ( (distFromWallAvg[0] + distFromWallAvg[1] + distFromWallAvg[2] + distFromWallAvg[3])/4)/TURN_COMPENSATION_K - omgComp2/TURN_COMPENSATION_K + superFix;
					}

					
					AUTO_MODE_GOTO(TURN_LEFT);
				END_AUTO_STATE;

				//turn left for 165* at a speed determined by the dist from wall
				AUTO_STATE_EXEC(TURN_LEFT);
					//printf("TLEFT FOR REAL:: ");
					closed_loop_drive = 1;
					changeGear = 1;

					if (angle > -1650L) {
						desSpeedL = (TURN_SPEED - abs(angle/TURN_ACCEL_K)) + omega;
						desSpeedR = (TURN_SPEED - abs(angle/TURN_ACCEL_K)) - omega;
					}else{
						//Set_Gyro_Angle(angle + 2100);
						left_wheel.totalError = right_wheel.totalError = 0;
						runIteration++;
						AUTO_MODE_GOTO(WAIT_FOR_WALL);
					}
				END_AUTO_STATE;

				//drive forward until wall comes back (high angular correction)
				AUTO_STATE_EXEC(WAIT_FOR_WALL);
					changeGear = 1;
					if (distance < WALL_MAX) {
						if (counter > counter_verify + 0) {
							//Set_Gyro_Angle(100);
							AUTO_MODE_GOTO(AUTO_SMART_CORRECT);
						}
					}else{
						desSpeedL = STRAIGHT_SPEED + (angle)/7;
						desSpeedR = STRAIGHT_SPEED - (angle)/7;
						counter_verify = counter;
					}
				END_AUTO_STATE;

				AUTO_STATE_EXEC(AUTO_NULL);
					desSpeedL = STRAIGHT_SPEED + (angle)/5;
					desSpeedR = STRAIGHT_SPEED - (angle)/5;
					
				END_AUTO_STATE;
			END_AUTONOMOUS

			//User_Byte2 = runIteration;
			//if (!ab_noBalls) {
				pwm03 = pwm05 = pid_control(&shoulder_enc, desArmPos - Get_Encoder_3_Count());
			//}

			//light controls
			if (secondIR) {
				relay6_fwd = 0;
				User_Byte4 = desDistFar/10;
			}else if (firstIR) {
				relay6_fwd = 1;
				User_Byte3 = desDistClose/10;
			}else{
				//relay6_fwd = 0;
			}

			//User_Byte3 = left_wheel_speed + 127;
			//User_Byte4 = right_wheel_speed + 127;
			User_Byte5 = User_Byte6;

			//if (0 && changeGear) {
//				if (max(abs(desSpeedL), abs(desSpeedR)) > LOW_SPEED_MAX && max(abs(left_wheel_speed), abs(right_wheel_speed)) > LOW_SWITCH_MAX) {
//					shifter = 0;
//				}else{
//					shifter = 1;
//				}
			//}else{
				shifter = 0;
				printf(" CANT SHIFT ");
			//}

			if (abs(left_wheel_speed) < 3 && abs(right_wheel_speed) < 3) {
				if (counter > counter_stall + 68) {
					KILLAUTO = 1;
				}
			}else{
				counter_stall = counter;
			}

			//if (closed_loop_drive) {
				pwm06 = pwm07 = pid_control(&left_wheel, left_wheel_speed - desSpeedL);
				pwm08 = pwm09 = pid_control(&right_wheel, right_wheel_speed - desSpeedR);
			//}else{
			//	pwm06=pwm07= desSpeedL;
			//	pwm08=pwm09= desSpeedR;
			//}
			
			//User_Byte6 = abs(angle)/10;
			pwm12 = runIteration;
			pwm01 = autonomous_case;


			
			//printf("ULTRASIG: %d | ANGLE: %li %d \r\n", distance, angle, User_Byte5);
			counter++;
			}else{
				pwm06=pwm07=pwm08=pwm09 = 127;
			}
			Putdata(&txdata);   		// DO NOT DELETE, or you will get no PWM outputs!
		}
		Process_Data_From_Local_IO();
	}
}

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Local_IO
* PURPOSE:       Execute user's realtime code.
* You should modify this routine by adding code which you wish to run fast.
* It will be executed every program loop, and not wait for fresh data 
* from the Operator Interface.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Local_IO(void)
{
	long int twc;
	//static prevWallSensor = 0;
	left_wheel_pos = Get_Encoder_1_Count();
	right_wheel_pos = Get_Encoder_2_Count();
	read_timer_clock();
	User_Byte1 = (unsigned char)(Get_ADC_Result(2) / 10);
	User_Byte2 = Get_Encoder_3_Count();
	//User_Byte5 = 5;
	//User_Byte3 = 3;
	//User_Byte4 = 4;
	User_Byte6 = 127 + ((127*Get_Gyro_Angle())/3600);
	//global_clock;

	//twc = (left_wheel_pos + right_wheel_pos)/2;
	//pwm10 = twc & 0xFF;
	//pwm11 = (twc << 8) & 0xFF;

	pwm10 = 127 + left_wheel_speed;
	pwm11 = 127 + right_wheel_speed;

	//printf("gc: %li\n", global_clock);
	
	//printf("gc: %i; pgc: %i\r\n", (int)left_wheel_speed, (int)right_wheel_speed);
	if (global_clock != previous_global_clock) {
		left_wheel_speed = (left_wheel_pos - previous_left_wheel_pos) / (global_clock - previous_global_clock);
		right_wheel_speed = (right_wheel_pos - previous_right_wheel_pos) / (global_clock - previous_global_clock);
	
		previous_left_wheel_pos = left_wheel_pos;
		previous_right_wheel_pos = right_wheel_pos;
	}
	

	//desired speeds set in other prgm
	//Left motor
	if (global_clock > prevGearChange + DRIVE_DELAY_TIME && !autonomous_mode) {
		if (CLOSED_DRIVE_OVERRIDE) {
			pwm06 = pwm07 = pid_control(&left_wheel, left_wheel_speed - desSpeedL);
			//Right motor
			pwm08 = pwm09 = pid_control(&right_wheel, right_wheel_speed - desSpeedR);
		}
	}else{
		pwm06 = pwm07 = pwm08 = pwm09 = 127;
	}

	//ADC processing
	if(Get_ADC_Result_Count()) {
		Process_Gyro_Data();
		Reset_ADC_Result_Count();
	}

	relay8_fwd 	= !rc_dig_in18;
	previous_global_clock = global_clock;
}

/*******************************************************************************
* FUNCTION NAME: Serial_Char_Callback
* PURPOSE:       Interrupt handler for the TTL_PORT.
* CALLED FROM:   user_SerialDrv.c
* ARGUMENTS:     
*     Argument             Type    IO   Description
*     --------             ----    --   -----------
*     data        unsigned char    I    Data received from the TTL_PORT
* RETURNS:       void
*******************************************************************************/

void Serial_Char_Callback(unsigned char data)
{
  /* Add code to handle incomming data (remember, interrupts are still active) */
}


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
