#ifndef _WALLCLOCK_H
#define _WALLCLOCK_H
//  Do not write to any of these variables

extern volatile unsigned long timer_clock;
extern unsigned long global_clock;
extern unsigned long previous_global_clock;
extern unsigned long prevGearChange;
extern int desSpeedR, desSpeedL;

void Timer_1_Reset(void);
void read_timer_clock(void);
void Timer_1_Initialize(void);
void Timer_1_Int_Handler(void);
void Timer_1_Load_Prescale(void);
unsigned int Timer_1_Microseconds(void);

#define MAX_SPEED_HI	67
#define MAX_SPEED_LO	31	
#define SHIFT_THRESH 16
#define DRIVE_DELAY_TIME 0		//40


#endif
