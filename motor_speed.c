#include <stdio.h>
#include "ifi_aliases.h"
#include "ifi_default.h"
#include "motor_speed.h"


/********** GLOBAL CLOCK VARIABLES ***********************************************/

volatile unsigned long timer_clock = 0L;		// milliseconds since system start (overflows after 49.7 days)
unsigned long global_clock = 0L;
unsigned long previous_global_clock = 0L;
unsigned long prevGearChange = 0L;
// for the current microseconds count, use Timer_1_Microseconds

// preload the timer so that the interrupt-on-overflow occurs 1000 microseconds later
// for microprocessor clock crystal 40MHz/4 = 10MHz tick rate = 65535 - 10000 (0xD8EF)

#define PRELOAD_LOW	 0x85  //goes to TMR1H
#define PRELOAD_HIGH 0xED  //goes to TMR1L



/*******************************************************************************
* FUNCTION NAME: Timer_1_Reset
* PURPOSE:       resets clock variables to zero
* CALLED FROM:   user_routines.c
* ARGUMENTS:     none
* RETURNS:       real-time clock variables are reset
*******************************************************************************/
void Timer_1_Reset(void)
{
	timer_clock = 0L;
}

/*******************************************************************************
* FUNCTION NAME: Timer_1_Load_Prescale
* PURPOSE:       preload the timer to overflow after 1000 microseconds
* CALLED FROM:   user_routines_fast.c, user_routines.c
* ARGUMENTS:     none
* RETURNS:       Timer1 is preloaded
*******************************************************************************/
void Timer_1_Load_Prescale(void)
{
					// NOTE:  Timer specification requires MUST write high byte
					//        first, then low byte

	TMR1H = PRELOAD_LOW;	// preload the timer so that the interrupt-on-overflow occurs 1000 microseconds later
	TMR1L = PRELOAD_HIGH;	// for microprocessor clock crystal 40MHz/4 = 10MHz tick rate = 65535 - 10000 (0xD8EF)
}


/*******************************************************************************
* FUNCTION NAME: Timer_1_Int_Handler
* PURPOSE:       Updates the real time clock variables
* CALLED FROM:   user_routines_fast.c
* ARGUMENTS:     none
* RETURNS:       real-time clock variables are updated
*******************************************************************************/
void Timer_1_Int_Handler(void)
{
	// update the millisecond-counter
	// probably will be more useful than the msec/sec/min/hours/days variables

	// avoid calling a function from within the Interrupt handler, other register
	// context will need to be saved with a #pragma in user_routines_fast.c
	//  Timer_1_Load_Prescale();

	TMR1H = PRELOAD_LOW;	// reload the timer so that the interrupt-on-overflow occurs 1000 microseconds later
	TMR1L = PRELOAD_HIGH;	// for microprocessor clock crystal 40MHz/4 = 10MHz tick rate = 65535 - 10000 (0xD8EF)

	timer_clock++;
}

/*******************************************************************************
* FUNCTION NAME: Timer_1_Initialize
* PURPOSE:       configure the Timer1 interrupt to overflow and interrupt after
*				 1000 microseconds
* CALLED FROM:   user_routines.c
* ARGUMENTS:     none
* RETURNS:       Timer1 is configured and TMR1IF interrupt enabled
*******************************************************************************/
void Timer_1_Initialize(void) {
	T1CONbits.TMR1ON = 0;
	PIR1bits.TMR1IF = 0;
	T1CON = 0x30;
	TMR1H = 0x85;
	TMR1L = 0xED;
	T1CONbits.TMR1ON = 1;

	IPR1bits.TMR1IP = 0;
	PIE1bits.TMR1IE = 1;
	INTCONbits.GIEL = 1;
}

void read_timer_clock(void) {
	INTCONbits.GIEL = 0;
	global_clock = timer_clock;	
	INTCONbits.GIEL = 1;
}
