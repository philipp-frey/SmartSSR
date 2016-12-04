/*
 * SmartSSR.c
 *
 * Created: 03.12.2016 15:00:16
 * Author : Philipp Frey
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

void init_clk(void);
void init_gpio(void);
void init_adc(void);
void init_extInterrupt(void);
void init_timer0(void);
void init_spi(void);
void init_wdt(void);

/************************************************************************/
/* Main Function, START here                                            */
/************************************************************************/
int main(void)
{
uint8_t resetReason = MCUSR;
	
	cli();				// Disable interrupts so registers can be changed without interruption

	init_clk();			// Initializes CLK to 4MHz
	init_gpio();		// Initializes I/Os
	init_adc();
	init_timer0();		// Initializes timer0
	init_wdt();			// Initializes watchdog to 1s
	init_extInterrupt();// Initializes external interrupts on I/O

	sei();				// Enable all interrupts
	
    while (1) 
    {
	//	wdt_reset();
		
    }
	
}

/************************************************************************/
/* Clock is set to  4MHz to ensure compatibility with 3.3V				*/
/************************************************************************/
void init_clk(void){
	// Clock init
	CLKPR = (1 << CLKPCE);				// Clock Prescaler Change Enable
	CLKPR = (1 << CLKPS0);				// Prescaler = 2: 4MHz CPUclk 
}

/************************************************************************/
/* PA0:6 are set as input, with pull-up on PA1.							*/
/* PA7 is set to output for Timer0-B									*/
/* PB0:2 are set to output with PB2 being Timer0-A. PB3 is set as input	*/
/************************************************************************/
void init_gpio(void){
	// PortA setup
	DDRA = (1 << DDA7) | (1<< DDA1);				//PA7 is output 1
	PORTA = (1 << PORTA1);							// PA1 with pull-up, all other off
	
	// PortB setup
	DDRB = (1 << DDB0) | (1 << DDB1) | (1 << DDB2);	// PB0:2 are outputs 2:4
	PORTB = 0;										// All outputs off
}

/************************************************************************/
/* PCINT1 (PA1) interrupt is enabled for zero crossing detection		*/
/************************************************************************/
void init_extInterrupt(void){
	// Interrupt setup
	GIMSK = (1<<PCIE0);								// Pin Change Interrupt Enable 0 (PCINT0:7)
	PCMSK0 = (1 << PCINT1);							// Pin Change Mask Register 0 (PCINT1)
}


/************************************************************************/
/* ADC for temperature sensing, because why not...                      */
/************************************************************************/
void init_adc(void){
  ADMUX = (1<<REFS1) | (1<<MUX5) | (1<<MUX1);                // Select temperature sensor with 1.1V internal voltage reference (Page 146: MUX5:0 = 100010)

  // Configure ADCSRA
  ADCSRA &= ~( (1<< ADATE ) | ( 1<<ADIE ) ); // Disable autotrigger, Disable Interrupt
  ADCSRA |= (1<<ADEN);          // Enable ADC
  ADCSRA |= (1<<ADSC);          // Start first conversion

  // Seed samples
  /*int raw_temp;
  while( ( ( raw_temp = raw() ) < 0 ) );
  for( int i = 0; i < TEMPERATURE_SAMPLES; i++ ) {
	  readings[i] = raw_temp;
  }*/
}

/************************************************************************/
/* Timer0 is initialized with 15.625kHz frequency (64us)				*/
/************************************************************************/
void init_timer0(void){
	TCCR0A = (1 << COM0A0) | (1 << COM0A1) |
	(1 << COM0B0) | (1 << COM0B1);			// Set Timer0-A and Timer0-B on compare match in normal mode

	TIMSK0 = (1 << OCIE0A) | (1<< OCIE0B) | (1<<TOIE0);
	// Enable all interrupts (compare A, compare B and overflow)

	TCNT0 = 0;									// Reset counter
	OCR0A = 0;									// Set output A compare match to 0 -> OFF
	OCR0B = 0;									// Set output B compare match to 0 -> OFF

	TCCR0B = (1<< CS02);						// Prescale 256 =  4MHz/256 = 15.625kHz (64us) Timer0 clock
}

/************************************************************************/
/* Initialize the SPI interface as slave                                */
/************************************************************************/
void init_spi(void){
	USICR = (1<<USIWM0) | (1<<USICS1);		// Three wire mode slave (SPI with external clock on positive edge)
	USISR = (1<<USISIF) | (1<<USIOIF) | (1<<USIPF) | (1<<USIDC);	// Clear all flags

	DDRA &= ~(1 << DDA4);		//set the USCK pin as input
	DDRA &= ~(1<<DDA6);			//set the DI pin as input
	DDRA |= (1<<DDA5);			//set the DO pin as output

	USICR |= (1<<USISIE) | (1<<USIOIE);			// Enable interrupts on start and overflow
}

/************************************************************************/
/* Initialize the Watchdog with 1s !! WOOF WOOF                         */
/************************************************************************/
void init_wdt (void){
	wdt_disable();
	wdt_enable(WDTO_1S);
	WDTCSR |= (1 << WDIE);				// Watchdog enable interrupt
}

/************************************************************************/
/* Zerocrossing interrupt on PA1/PCINT1                                 */
/************************************************************************/
ISR(PCINT1_vect){
	TCNT0 = 0;									// Reset counter

}

/************************************************************************/
/* Timer0 Compare B interrupt                                           */
/************************************************************************/
ISR(TIM0_COMPB_vect){
	
	
}

/************************************************************************/
/* Timer0 Compare A interrupt                                           */
/************************************************************************/
ISR(TIM0_COMPA_vect){
	
	
}

/************************************************************************/
/* Timer0 overflow interrupt                                            */
/************************************************************************/
ISR(TIM0_OVF_vect){
	
	
}

/************************************************************************/
/* Watchdog interrupt                                                   */
/************************************************************************/
ISR(WATCHDOG_vect){
	PORTA ^= (1 << PORTA1);	// Toggle LED
	
}

/************************************************************************/
/* USI Start interrupt                                                  */
/************************************************************************/
ISR(USI_STR_vect){


}

/************************************************************************/
/*  USI Overflow interrupt                                              */
/************************************************************************/
ISR(USI_OVF_vect){


}

/*
	// SPI Transfer:
	USIDR = spiData;
	USISR = _BV(USIOIF);                //clear counter and counter overflow interrupt flag
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { //ensure a consistent clock period
		while ( !(USISR & _BV(USIOIF)) ) USICR |= _BV(USITC);
	}
	return USIDR;

	// SPI End
	USICR &= ~(1<<USIWM0);

SIGNAL(SIG_USI_START) {
	uint8_t tmpUSI_STATUS;
	tmpUSI_STATUS = USI_STATUS;
	COMM_STATUS = NONE;
	// Wait for SCL to go low to ensure the "Start Condition" has completed.
	// otherwise the counter will count the transition
	while ( (PIN_USI & (1<<PORT_USI_SCL)) );
	USI_STATUS = 0xf0; // write 1 to clear flags; clear counter
	USI_CONTROL |= (1<<USIOIE);// enable USI interrupt on overflow
}

SIGNAL(SIG_USI_OVERFLOW) {
	uint8_t BUF_USI_DATA = USI_DATA;
	switch(COMM_STATUS) {
		case NONE:
		if (((BUF_USI_DATA & 0xfe) >> 1) != USI_ADDRESS) {	// if not receiving my address
			USI_CONTROL &= ~(1<<USIOIE); // disable USI interrupt on overflow
		}
		else { // else address is mine
			DDR_USI  |=  (1<<PORT_USI_SDA);
			USI_STATUS = 0x0e;	// reload counter for ACK, (SCL) high and back low
			if (BUF_USI_DATA & 0x01) COMM_STATUS = ACK_PR_TX; else COMM_STATUS = ACK_PR_RX;
		}
		break;
		case ACK_PR_RX:
		DDR_USI  &= ~(1<<PORT_USI_SDA);
		COMM_STATUS = BYTE_RX;
		break;
		case BYTE_RX:
		// Save received byte here! ... = USI_DATA
		DDR_USI  |=  (1<<PORT_USI_SDA);
		USI_STATUS = 0x0e;	// reload counter for ACK, (SCL) high and back low
		COMM_STATUS = ACK_PR_RX;
		break;
		case ACK_PR_TX:
		// Put first byte to transmit in buffer here! USI_DATA = ...
		PORT_USI |=  (1<<PORT_USI_SDA); // transparent for shifting data out
		COMM_STATUS = BYTE_TX;
		break;
		case PR_ACK_TX:
		if(BUF_USI_DATA & 0x01) {
			COMM_STATUS = NONE; // no ACK from master --> no more bytes to send
		}
		else {
			// Put next byte to transmit in buffer here! USI_DATA = ...
			PORT_USI |=  (1<<PORT_USI_SDA); // transparent for shifting data out
			DDR_USI  |=  (1<<PORT_USI_SDA);
			COMM_STATUS = BYTE_TX;
		}
		break;
		case BYTE_TX:
		DDR_USI  &= ~(1<<PORT_USI_SDA);
		PORT_USI &= ~(1<<PORT_USI_SDA);
		USI_STATUS = 0x0e;	// reload counter for ACK, (SCL) high and back low
		COMM_STATUS = PR_ACK_TX;
		break;
	}
	USI_STATUS |= (1<<USIOIF); // clear overflowinterruptflag, this also releases SCL
}
*/

