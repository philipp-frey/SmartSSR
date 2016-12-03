/*
 * SmartSSR.c
 *
 * Created: 03.12.2016 15:00:16
 * Author : Philipp Frey
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

void init_hw(void);
void init_wdt(void);

/************************************************************************/
/* Main Function, START here                                            */
/************************************************************************/
int main(void)
{
uint8_t resetReason = MCUSR;
	
	init_hw();
	init_wdt();
	sei();
	
    while (1) 
    {
		wdt_reset();
		
    }
	
}

/************************************************************************/
/* Setting up everything with disabled interrupt,						*/
/* enable interrupts after calling function								*/
/* Clock is set to  4MHz to ensure compatibility with 3.3V				*/
/* PA0:6 are set as input, with pull-up on PA0.							*/
/* PA7 is set to output for Timer0-B									*/
/* PB0:2 are set to output with PB2 being Timer0-A. PB3 is set as input	*/
/* PCINT0 (PA0) interrupt is enabled									*/
/************************************************************************/
void init_hw(void){
	
	cli();								// Disable interrupts so registers can be changed without interruption
	
	// Clock init
	CLKPR = (1 << CLKPCE);				// Clock Prescaler Change Enable
	CLKPR = (1 << CLKPS0);				// Prescaler = 2: 4MHz CPUclk 
	
	// PortA setup
	DDRA = (1 << DDA7);								//PA7 is output 1
	PORTA = (1 << PORTA0);							// PA0 with pull-up, all other off
	
	// PortB setup
	DDRB = (1 << DDB0) | (1 << DDB1) | (1 << DDB2);	// PB0:2 are outputs 2:4
	PORTB = 0;										// All outputs off
	
	// Interrupt setup
	GIMSK = (1<<PCIE0);								// Pin Change Interrupt Enable 0 (PCINT0:7)
	PCMSK0 = (1 << PCINT0);							// Pin Change Mask Register 0 (PCINT0)	
	
	// Timer1 setup
	
	
}


/************************************************************************/
/* Initialize the Watchdog with 1s !! WOOF WOOF                         */
/************************************************************************/
void init_wdt (void){
	wdt_disable();
	WDTCSR |= (1 << WDIE);
	wdt_enable(WDTO_1S);
}


/************************************************************************/
/* Zerocrossing interrupt on PA0/PCINT0                                 */
/************************************************************************/
ISR(PCINT0_vect){
	
	
}


ISR(TIM0_COMPB_vect){
	
	
}

ISR(TIM0_COMPA_vect){
	
	
}

ISR(WATCHDOG_vect){
	
	
}
