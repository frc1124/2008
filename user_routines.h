/*******************************************************************************
* FILE NAME: user_routines.h
*
* DESCRIPTION: 
*  This is the include file which corresponds to user_routines.c and
*  user_routines_fast.c
*  It contains some aliases and function prototypes used in those files.
*
* USAGE:
*  If you add your own routines to those files, this is a good place to add
*  your custom macros (aliases), type definitions, and function prototypes.
*******************************************************************************/

#ifndef __user_program_h_
#define __user_program_h_


/*******************************************************************************
                            MACRO DECLARATIONS
*******************************************************************************/
/* Add your macros (aliases and constants) here.                              */
/* Do not edit the ones in ifi_aliases.h                                      */
/* Macros are substituted in at compile time and make your code more readable */
/* as well as making it easy to change a constant value in one place, rather  */
/* than at every place it is used in your code.                               */
/*
 EXAMPLE CONSTANTS:
#define MAXIMUM_LOOPS   5
#define THE_ANSWER      42
#define TRUE            1
#define FALSE           0
#define PI_VAL          3.1415

 EXAMPLE ALIASES:
#define LIMIT_SWITCH_1  rc_dig_int1  (Points to another macro in ifi_aliases.h)
#define MAIN_SOLENOID   solenoid1    (Points to another macro in ifi_aliases.h)
*/

/* Used in limit switch routines in user_routines.c */
#define OPEN        1     /* Limit switch is open (input is floating high). */
#define CLOSED      0     /* Limit switch is closed (input connected to ground). */

#define speak(x)	printf(x)

/*******************************************************************************
                            TYPEDEF DECLARATIONS
*******************************************************************************/
extern int right_wheel_speed;
extern int left_wheel_speed;
extern long right_wheel_pos;
extern long left_wheel_pos;
extern long previous_right_wheel_pos;
extern long previous_left_wheel_pos;
extern int AUTODEBUG;

#define shifter 		relay1_fwd
#define	elbow			relay2_fwd
#define gripper_close	relay7_fwd
#define gripper_open	relay7_rev

#define GEAR_SWITCH_BUTTON	p4_sw_aux2
#define ELBOW_SWITCH		p4_sw_trig
#define GRIPPER_SWITCH		p4_sw_top

#define irButton1	rc_dig_in06
#define irButton2	rc_dig_in07
#define irButton3	rc_dig_in08
#define irButton4	rc_dig_in09

#define drive_L1	pwm06
#define drive_L2	pwm07
#define drive_R1	pwm08
#define drive_R2	pwm09

#define CLOSED_DRIVE_OVERRIDE	p4_sw_aux1

#define RAISE_THRESHOLD	127

#define gripperLimitSwitch	rc_dig_in16
/*******************************************************************************
                           FUNCTION PROTOTYPES
*******************************************************************************/

/* These routines reside in user_routines.c */
void User_Initialization(void);
void Process_Data_From_Master_uP(void);
void Default_Routine(void);
unsigned char Limit_Mix (int intermediate_value);
int abs(int x);
int max(int a, int b);
int min(int a, int b);
char cosDeg(int deg);


/* These routines reside in user_routines_fast.c */
void InterruptHandlerLow (void);  /* DO NOT CHANGE! */
void User_Autonomous_Code(void);  /* Only in full-size FRC system. */
void Process_Data_From_Local_IO(void);


#endif
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
