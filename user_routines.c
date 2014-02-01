/*******************************************************************************
* FILE NAME: user_routines.c <FRC VERSION>
*
* DESCRIPTION:
*  This file contains the default mappings of inputs  
*  (like switches, joysticks, and buttons) to outputs on the RC.  
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
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
#include "adc.h"
#include "gyro.h"
#include "autonomous.h"

extern unsigned char aBreakerWasTripped;

unsigned char pickUpMode = 0;

long enc_wheel_L = 0;

DT_PID left_wheel, right_wheel, shoulder_enc;
int right_wheel_speed = 0, left_wheel_speed = 0;
int desSpeedR = 0, desSpeedL = 0;
long right_wheel_pos = 0, left_wheel_pos = 0, previous_right_wheel_pos = 0, previous_left_wheel_pos = 0;
int AUTODEBUG = 0;
rom const char cosLookUp[181] = {100, 100, 100, 100, 100, 100, 99, 99, 99, 99, 98, 98, 98, 97, 97, 97, 96, 96, 95, 95, 94, 93, 93, 92, 91, 91, 90, 89, 88, 87, 87, 86, 85, 84, 83, 82, 81, 80, 79, 78, 77, 75, 74, 73, 72, 71, 69, 68, 67, 66, 64, 63, 62, 60, 59, 57, 56, 54, 53, 52, 50, 48, 47, 45, 44, 42, 41, 39, 37, 36, 34, 33, 31, 29, 28, 26, 24, 22, 21, 19, 17, 16, 14, 12, 10, 9, 7, 5, 3, 2, 0, -2, -3, -5, -7, -9, -10, -12, -14, -16, -17, -19, -21, -22, -24, -26, -28, -29, -31, -33, -34, -36, -37, -39, -41, -42, -44, -45, -47, -48, -50, -52, -53, -54, -56, -57, -59, -60, -62, -63, -64, -66, -67, -68, -69, -71, -72, -73, -74, -75, -77, -78, -79, -80, -81, -82, -83, -84, -85, -86, -87, -87, -88, -89, -90, -91, -91, -92, -93, -93, -94, -95, -95, -96, -96, -97, -97, -97, -98, -98, -98, -99, -99, -99, -99, -100, -100, -100, -100, -100, -100};

int abs(int x) {
	return (x > 0) ? x : -x; 
}

int max(int a, int b) {
	return (a > b) ? a : b;
}

int min(int a, int b) {
	return (b > a) ? a : b;
}

char cosDeg(int deg) {
	deg = abs(deg);
	return cosLookUp[deg];
}


/*** DEFINE USER VARIABLES AND INITIALIZE THEM HERE ***/
/* EXAMPLES: (see MPLAB C18 User's Guide, p.9 for all types)
unsigned char wheel_revolutions = 0; (can vary from 0 to 255)
unsigned int  delay_count = 7;       (can vary from 0 to 65,535)
int           angle_deviation = 142; (can vary from -32,768 to 32,767)
unsigned long very_big_counter = 0;  (can vary from 0 to 4,294,967,295)
*/

/*******************************************************************************
* FUNCTION NAME: Limit_Switch_Max
* PURPOSE:       Sets a PWM value to neutral (127) if it exceeds 127 and the
*                limit switch is on.
* CALLED FROM:   this file
* ARGUMENTS:     
*     Argument       Type             IO   Description
*     --------       -------------    --   -----------
*     switch_state   unsigned char    I    limit switch state
*     *input_value   pointer           O   points to PWM byte value to be limited
* RETURNS:       void
*******************************************************************************/
void Limit_Switch_Max(unsigned char switch_state, unsigned char *input_value)
{
  if (switch_state == CLOSED)
  { 
    if(*input_value > 127)
      *input_value = 127;
  }
}


/*******************************************************************************
* FUNCTION NAME: Limit_Switch_Min
* PURPOSE:       Sets a PWM value to neutral (127) if it's less than 127 and the
*                limit switch is on.
* CALLED FROM:   this file
* ARGUMENTS:     
*     Argument       Type             IO   Description
*     --------       -------------    --   -----------
*     switch_state   unsigned char    I    limit switch state
*     *input_value   pointer           O   points to PWM byte value to be limited
* RETURNS:       void
*******************************************************************************/
void Limit_Switch_Min(unsigned char switch_state, unsigned char *input_value)
{
  if (switch_state == CLOSED)
  { 
    if(*input_value < 127)
      *input_value = 127;
  }
}


/*******************************************************************************
* FUNCTION NAME: Limit_Mix
* PURPOSE:       Limits the mixed value for one joystick drive.
* CALLED FROM:   Default_Routine, this file
* ARGUMENTS:     
*     Argument             Type    IO   Description
*     --------             ----    --   -----------
*     intermediate_value    int    I    
* RETURNS:       unsigned char
*******************************************************************************/
unsigned char Limit_Mix (int intermediate_value)
{
  static int limited_value;
  
  if (intermediate_value < 2000)
  {
    limited_value = 2000;
  }
  else if (intermediate_value > 2254)
  {
    limited_value = 2254;
  }
  else
  {
    limited_value = intermediate_value;
  }
  return (unsigned char) (limited_value - 2000);
}


/*******************************************************************************
* FUNCTION NAME: User_Initialization
* PURPOSE:       This routine is called first (and only once) in the Main function.  
*                You may modify and add to this function.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Initialization (void)
{
	
  Set_Number_of_Analog_Channels(SIXTEEN_ANALOG);    /* DO NOT CHANGE! */

/* FIRST: Set up the I/O pins you want to use as digital INPUTS. */
  digital_io_01 = digital_io_02 = digital_io_03 = digital_io_04 = INPUT;
  digital_io_05 = digital_io_06 = digital_io_07 = digital_io_08 = INPUT;
  digital_io_09 = digital_io_10 = digital_io_11 = digital_io_12 = INPUT;
  digital_io_13 = digital_io_14 = digital_io_15 = digital_io_16 = INPUT;
  digital_io_18 = INPUT;  /* Used for pneumatic pressure switch. */
    /* 
     Note: digital_io_01 = digital_io_02 = ... digital_io_04 = INPUT; 
           is the same as the following:

           digital_io_01 = INPUT;
           digital_io_02 = INPUT;
           ...
           digital_io_04 = INPUT;
    */

/* SECOND: Set up the I/O pins you want to use as digital OUTPUTS. */
  digital_io_17 = INPUT;    /* Example - Not used in Default Code. */

/* THIRD: Initialize the values on the digital outputs. */
  //rc_dig_out17 = 0;

/* FOURTH: Set your initial PWM values.  Neutral is 127. */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;

/* FIFTH: Set your PWM output types for PWM OUTPUTS 13-16.
  /*   Choose from these parameters for PWM 13-16 respectively:               */
  /*     IFI_PWM  - Standard IFI PWM output generated with Generate_Pwms(...) */
  /*     USER_CCP - User can use PWM pin as digital I/O or CCP pin.           */
  Setup_PWM_Output_Type(IFI_PWM,IFI_PWM,IFI_PWM,IFI_PWM);

  /* 
     Example: The following would generate a 40KHz PWM with a 50% duty cycle on the CCP2 pin:

         CCP2CON = 0x3C;
         PR2 = 0xF9;
         CCPR2L = 0x7F;
         T2CON = 0;
         T2CONbits.TMR2ON = 1;

         Setup_PWM_Output_Type(USER_CCP,IFI_PWM,IFI_PWM,IFI_PWM);
  */

  /* Add any other initialization code here. */
 
  Putdata(&txdata);             /* DO NOT CHANGE! */

	Initialize_Encoders();
	Timer_1_Initialize();

	init_pid(&left_wheel, 220, 115, 0, 65, 1); //220	115`
	init_pid(&right_wheel, 220, 115, 0, 65, 1); //220	115
	init_pid(&shoulder_enc, 150, 150, 0, 100, 8);

  //Serial_Driver_Initialize();
	Initialize_Serial_Comms();	

	Initialize_Gyro();
	Initialize_ADC();

  printf("IFI 2006 User Processor Initialized ...\r");  /* Optional - Print initialization message. */

  User_Proc_Is_Ready();         /* DO NOT CHANGE! - last line of User_Initialization */
}

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Master_uP
* PURPOSE:       Executes every 26.2ms when it gets new data from the master 
*                microprocessor.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Master_uP(void)
{
	static int i = 0;
	
	Getdata(&rxdata);   /* Get fresh data from the master microprocessor. */
	
	Default_Routine();  /* Optional.  See below. */

	//if ((left_wheel_pos + right_wheel_pos)/2 < - 5590) {
	//	relay6_fwd = 1;
	//}else{
	//	relay6_fwd = 0;
	//}
	
	
	//Gyro Autocalibration
	if (ab_doNothing) {
		if (i == 6) {
			Start_Gyro_Bias_Calc();
			relay6_fwd = 1;
		}else if (i == 200) {
			Stop_Gyro_Bias_Calc();
			Reset_Gyro_Angle();
			//printf("Gyro Done.\r\n");
		}else if (i > 200) { //usually 200
			relay6_fwd = 0;
			if (i%20 == 0) printf("%d\r\n", Get_Gyro_Bias());
		}
	}else{
		Set_Gyro_Bias(GYRO_BIAS);
		if (i % 10 == 0) {
			//printf("gyro angle: %li\r\n", Get_Gyro_Angle());
			printf("ADBG: %d ::", AUTODEBUG);
			printf("lwp: %li, rwp %li ::  ", left_wheel_pos, right_wheel_pos);
			printf("lws: %d, rws %d ::  ", left_wheel_speed, right_wheel_speed);
			printf("ac: %li", Get_Encoder_3_Count());
			printf("ultra: %d %li %d\r\n", Get_ADC_Result(2), Get_Gyro_Angle(), Get_Gyro_Bias());
			//printf(" %d %d %d %d \r\n", ab_noBalls, ab_skipFirst, ab_delay, ab_doNothing);
			
		}
	}

		//printf("wheel speeds: %d \t %d\r\n", left_wheel_speed, right_wheel_speed);

	i++;
	
	Putdata(&txdata);             /* DO NOT CHANGE! */
}

/*******************************************************************************
* FUNCTION NAME: Default_Routine
* PURPOSE:       Performs the default mappings of inputs to outputs for the
*                Robot Controller.
* CALLED FROM:   this file, Process_Data_From_Master_uP routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Default_Routine(void)
{
	long shoulderValue = 100;
	static char prevGearSwitch;
	static int icnt = 0;
	static int scnt = 0;
	read_timer_clock();

	if (GEAR_SWITCH_BUTTON != prevGearSwitch && GEAR_SWITCH_BUTTON == 1) {
		scnt = icnt;
	}

	//printf("%i %i\r\n", p3_x, p3_y);
	if (GEAR_SWITCH_BUTTON) {	//High Gear
		desSpeedR = -(MAX_SPEED_HI*((int)Limit_Mix(2000 + (p3_x) + p3_y - 127)-127))/127;
		desSpeedL = (MAX_SPEED_HI*((int)Limit_Mix(2000 + (p3_x) - p3_y + 127)-127))/127;
	}else{				//Low Gear
		desSpeedR = -(MAX_SPEED_LO*((int)Limit_Mix(2000 + (p3_x) + p3_y - 127)-127))/127;
		desSpeedL = (MAX_SPEED_LO*((int)Limit_Mix(2000 + (p3_x) - p3_y + 127)-127))/127;
	}

	
	shifter = !GEAR_SWITCH_BUTTON;
	elbow = !ELBOW_SWITCH;

	

	if(p1_sw_trig){
		pwm03 = pwm05 = p1_y;
	}else if (p2_sw_trig) {
		pwm03 = pwm05 = pid_control(&shoulder_enc, ARM_HEIGHT - Get_Encoder_3_Count());
	}else{
		pwm03 = pwm05 = 127;
	}

	

	pickUpMode = p1_sw_top;

	if (0) {
		if(pickUpMode){
			gripper_open = 1;
			gripper_close = 0;
			elbow = 1;
		}else{
			if(shoulderValue < RAISE_THRESHOLD){
				elbow = 0;
			}else{
				elbow = 1;
			}
			if(p4_sw_trig){
				gripper_open = 1;
				gripper_close = 0;
			}else{
				gripper_open = 0;
				gripper_close = 1;
			}
		}
	}else{
		gripper_open = !GRIPPER_SWITCH;
		gripper_close = GRIPPER_SWITCH;
		//!elbow = ELBOW_SWITCH;
	}

	//Lights!
	//if(p1_sw_top){
	//	relay5_fwd = 1;
	//	relay5_rev = 0;
	//}else{
	//	relay5_fwd = 0;
	//	relay5_rev = 0;
	//}

	if(irButton1 || irButton2 || irButton3 || irButton4){
		//relay6_fwd = 1;
		//relay6_rev = 0;
	}else{
		//relay6_fwd = 0;
		//relay6_rev = 0;
	}

	if (!CLOSED_DRIVE_OVERRIDE) {
		//pwm06=pwm07=pwm08=pwm09=(((int)Limit_Mix(2000 + (255 - (int)p1_x) + p1_y - 127) - 127)) + 127;
		//pwm06=pwm07=pwm08=pwm09=(-((int)Limit_Mix(2000 + (255 - (int)p3_x) - p3_y + 127) - 127)) + 127;	
		if (icnt > scnt + 7) { 
			pwm06=pwm07=(((int)Limit_Mix(2000 + (255 - (int)p3_x) + p3_y - 127) - 127)) + 127;
			pwm08=pwm09=(-((int)Limit_Mix(2000 + (255 - (int)p3_x) - p3_y + 127) - 127)) + 127;
		}else{
			pwm06=pwm07=pwm08=pwm09 = 127;
		}
	}
	//printf("%d %d\r\n", icnt, scnt);

	icnt++;

	//printf("Pick Up Mode: %i | Gripper Open: %i | Elbow Open %i\r\n", pickUpMode, gripper_open, elbow);
	prevGearSwitch = GEAR_SWITCH_BUTTON;

	//printf("R: %i/%i;  L: %i/%i G:%d\r\n", right_wheel_speed, desSpeedR, left_wheel_speed, desSpeedL, p3_y);
} /* END Default_Routine(); */



/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
