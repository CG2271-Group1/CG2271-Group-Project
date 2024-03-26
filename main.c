/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"                    // Device header

#define CLOCK_FREQ 48000000
#define PRE_SCALAR 128
#define BAUD_RATE 9600
#define TPM0_MOD 7500
#define TPM1_MOD 7500

// Define the UART pin numbers
#define UART_RX_PORTE23 23
#define UART_INIT_PRIO 128
#define PTB0_Pin 0      
#define PTD5_Pin 5
#define PTE29_Pin 29
#define PTE30_Pin 30
#define PTE31_Pin 31

// Define the LED pin numbers
#define FRONT_LED_1  7  // PortC Pin 7
#define FRONT_LED_2  8  // PortC Pin 8
#define FRONT_LED_3  9  // PortC Pin 9
#define FRONT_LED_4  10  // PortC Pin 10
#define FRONT_LED_5  11  // PortC Pin 11
#define FRONT_LED_6  12  // PortC Pin 12
#define FRONT_LED_7  13  // PortC Pin 13
#define FRONT_LED_8  4  // PortC Pin 14
#define FRONT_LED_9  5  // PortC Pin 15
#define FRONT_LED_10 16  // PortC Pin 16
#define BACK_LED     17  // PortC Pin 17
#define MASK(x)      (1 << (x))

/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/

volatile int moving_status = 0;
volatile double gear_multiplier[4] = {0.2, 0.5, 0.8, 1.2};
volatile double rl_turn[4] = {0.5, 0.7, 0.9, 1.2};
volatile int gear_level = 2;

// This part is for the audio information
uint8_t megMelody[] = {1, 3, 5, 6, 5, 0, 1, 2, 1, 3, 0, 3, 5, 6, 7, 6, 5, 3, 2, 1, 2, 3, 1};
uint8_t megDurations[] = {1, 1, 1, 3, 1, 3, 1, 2, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1, 2, 1, 1};
uint8_t megNumNotes = sizeof(megMelody) / sizeof(megMelody[0]);
	
uint8_t megMelody_1[] = {5, 6, 7, 6, 7, 5, 2, 5, 0};
uint8_t megDurations_1[] = {1, 1, 1, 2, 1, 2, 1, 3, 5};
uint8_t megNumNotes_1 = sizeof(megMelody_1) / sizeof(megMelody_1[0]);

uint8_t end_point = 0;

void InitGPIO(void)
{
	// Enable Clock to PORTC
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	
	// Configure MUX settings to make all pins GPIO
	PORTC->PCR[FRONT_LED_1] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[FRONT_LED_1] |= PORT_PCR_MUX(1);
	PORTC->PCR[FRONT_LED_2] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[FRONT_LED_2] |= PORT_PCR_MUX(1);
	PORTC->PCR[FRONT_LED_3] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[FRONT_LED_3] |= PORT_PCR_MUX(1);
	PORTC->PCR[FRONT_LED_4] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[FRONT_LED_4] |= PORT_PCR_MUX(1);
	PORTC->PCR[FRONT_LED_5] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[FRONT_LED_5] |= PORT_PCR_MUX(1);
	PORTC->PCR[FRONT_LED_6] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[FRONT_LED_6] |= PORT_PCR_MUX(1);
	PORTC->PCR[FRONT_LED_7] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[FRONT_LED_7] |= PORT_PCR_MUX(1);
	PORTC->PCR[FRONT_LED_8] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[FRONT_LED_8] |= PORT_PCR_MUX(1);
	PORTC->PCR[FRONT_LED_9] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[FRONT_LED_9] |= PORT_PCR_MUX(1);
	PORTC->PCR[FRONT_LED_10] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[FRONT_LED_10] |= PORT_PCR_MUX(1);
	PORTC->PCR[BACK_LED] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[BACK_LED] |= PORT_PCR_MUX(1);

	// Set Data Direction Registers for PortB and PortD
	PTC->PDDR |= (MASK(FRONT_LED_1) | MASK(FRONT_LED_2) | MASK(FRONT_LED_3) | MASK(FRONT_LED_4) | MASK(FRONT_LED_5) | 
								MASK(FRONT_LED_6) | MASK(FRONT_LED_7) | MASK(FRONT_LED_8) | MASK(FRONT_LED_9) | MASK(FRONT_LED_10) | 
								MASK(BACK_LED) );
}

void initUART2(uint32_t baud_rate)
{
    uint32_t divisor, bus_clock; // Define variables for the divisor and the bus clock

    // Enable the clock for UART2 module by setting the appropriate bit in the System Clock Gating Control Register 4
    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;

    // Enable the clock for Port E by setting the bit in the System Clock Gating Control Register 5
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    // Configure the pin control register for the UART2 RX pin in a similar way
    PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK; // Clear current MUX setting
    PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4); // Set MUX to UART2 RX function

    // Disable UART2 transmitter and receiver before changing the baud rate
    UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));

    // Calculate the bus clock by halving the default system clock
    bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;

    // Calculate the baud rate divisor based on the bus clock and desired baud rate
    divisor = bus_clock / (baud_rate * 16);

    // Set the high part of the baud rate divisor
    UART2->BDH = UART_BDH_SBR(divisor >> 8);
    // Set the low part of the baud rate divisor
    UART2->BDL = UART_BDL_SBR(divisor);

    // Clear the UART Control Register 1 to reset to default settings (8N1: 8 bits, No parity, 1 stop bit)
    UART2->C1 = 0;
    // Clear the UART Status Register 2 to clear any error flags and reset to default settings
    UART2->S2 = 0;
    // Clear the UART Control Register 3 for default settings
    UART2->C3 = 0;

    // Re-enable the UART2 receiver now that configuration is complete
    UART2->C2 |= (UART_C2_RE_MASK);
		
		NVIC_SetPriority(UART2_IRQn, 0); 
    NVIC_ClearPendingIRQ(UART2_IRQn); 
    NVIC_EnableIRQ(UART2_IRQn);
		
		UART2->C2 |= UART_C2_RIE_MASK;
}
 
void initPWM(void)     // Defines a function named initPWM that takes no arguments
{
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;  // Enables the clock gating for port B module by setting the corresponding bit in the System Integration Module (SIM) SCGC5 register
    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;  // Enables the clock gating for port D module by setting the corresponding bit in the System Integration Module (SIM) SCGC5 register
    SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;  // Enables the clock gating for port E module by setting the corresponding bit in the System Integration Module (SIM) SCGC5 register

    PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;      // Clears the MUX field for pin PTB0 on port B (probably to set it to a certain function)
    PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);         // Sets the MUX field for pin PTB0 on port B to the alternative function 3

    PORTD->PCR[PTD5_Pin] &= ~PORT_PCR_MUX_MASK;      // Clears the MUX field for pin PTD5 on port D
    PORTD->PCR[PTD5_Pin] |= PORT_PCR_MUX(4);         // Sets the MUX field for pin PTD5 on port D to the alternative function 4

    PORTE->PCR[PTE29_Pin] &= ~PORT_PCR_MUX_MASK;      // Clears the MUX field for pin PTE29 on port E
    PORTE->PCR[PTE29_Pin] |= PORT_PCR_MUX(3);         // Sets the MUX field for pin PTE29 on port E to the alternative function 3

    PORTE->PCR[PTE30_Pin] &= ~PORT_PCR_MUX_MASK;      // Clears the MUX field for pin PTE30 on port E
    PORTE->PCR[PTE30_Pin] |= PORT_PCR_MUX(3);         // Sets the MUX field for pin PTE30 on port E to the alternative function 3

    PORTE->PCR[PTE31_Pin] &= ~PORT_PCR_MUX_MASK;      // Clears the MUX field for pin PTE31 on port E
    PORTE->PCR[PTE31_Pin] |= PORT_PCR_MUX(3);         // Sets the MUX field for pin PTE31 on port E to the alternative function 3

    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;               // Enables the clock for TPM0 module by setting the corresponding bit in the SIM SCGC6 register
    SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;               // Enables the clock for TPM1 module by setting the corresponding bit in the SIM SCGC6 register

    SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;            // Clears the TPM clock source field in the SIM SOPT2 register
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);               // Sets the TPM clock source to option 1 in the SIM SOPT2 register

    TPM0->MOD = TPM0_MOD;                                 // Sets the modulo value for the TPM0 module;
    TPM1->MOD = TPM1_MOD;                                 // Sets the modulo value for the TPM1 module; 

    TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));   // Clears the CMOD (clock mode selection) and PS (prescale factor) fields in the TPM0 SC (status and control) register
    TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));            // Sets the CMOD to 1 (probably to enable the counter) and the prescale factor to 7

    TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));   // Clears the CMOD (clock mode selection) and PS (prescale factor) fields in the TPM1 SC (status and control) register
    TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));            // Sets the CMOD to 1 (probably to enable the counter) and the prescale factor to 7

    TPM0->SC &= ~(TPM_SC_CPWMS_MASK);                 // set the counter to up counting mode (0: upcounting, 1: up-down counting)
    TPM1->SC &= ~(TPM_SC_CPWMS_MASK);                 // set the counter to up counting mode (0: upcounting, 1: up-down counting)

    TPM0_C5SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));  // Clears the channel status and control fields for channel 0 of TPM0
    TPM0_C5SC |= ((TPM_CnSC_ELSB(1)) | TPM_CnSC_MSB(1));     // Sets the ELSB and MSB fields for channel 0 of TPM0.

    TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));  // Clears the channel status and control fields for channel 2 of TPM0
    TPM0_C2SC |= ((TPM_CnSC_ELSB(1)) | TPM_CnSC_MSB(1));     // Sets the ELSB and MSB fields for channel 2 of TPM0.

    TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));  // Clears the channel status and control fields for channel 3 of TPM0
    TPM0_C3SC |= ((TPM_CnSC_ELSB(1)) | TPM_CnSC_MSB(1));     // Sets the ELSB and MSB fields for channel 3 of TPM0.

    TPM0_C4SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));  // Clears the channel status and control fields for channel 4 of TPM0
    TPM0_C4SC |= ((TPM_CnSC_ELSB(1)) | TPM_CnSC_MSB(1));     // Sets the ELSB and MSB fields for channel 4 of TPM0.

    TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));  // Clears the channel status and control fields for channel 0 of TPM1
    TPM1_C0SC |= ((TPM_CnSC_ELSB(1)) | TPM_CnSC_MSB(1));     // Sets the ELSB and MSB fields for channel 0 of TPM1. 
}

void UART2_IRQHandler(void) {
    if (UART2->S1 & UART_S1_RDRF_MASK) {
			  moving_status = 1 - moving_status;
        uint8_t instruction = UART2->D;
        switch (instruction)
        {
					case 0x11: //w, move forward
            TPM0_C5V = 0.75 * TPM0_MOD * gear_multiplier[gear_level];  //75% duty
            TPM0_C2V = 0;
            TPM0_C3V = 0.75 * TPM0_MOD * gear_multiplier[gear_level];
            TPM0_C4V = 0;
            moving_status = 1;
            break;
          case 0x12: //a, move left
            TPM0_C5V = 0.5 * TPM0_MOD * rl_turn[gear_level];  //50% duty 
            TPM0_C2V = 0;
            TPM0_C3V = 0;
            TPM0_C4V = 0.5 * TPM0_MOD * rl_turn[gear_level];
            moving_status = 1;
            break;
          case 0x13: //s, move backward
            TPM0_C5V = 0;
            TPM0_C2V = 0.75 * TPM0_MOD * gear_multiplier[gear_level];  //75% duty
            TPM0_C3V = 0;
            TPM0_C4V = 0.75 * TPM0_MOD * gear_multiplier[gear_level];
            moving_status = 1;
            break;
          case 0x14: //d, move right
            TPM0_C5V = 0;
            TPM0_C2V = 0.5 * TPM0_MOD * rl_turn[gear_level];
            TPM0_C3V = 0.5 * TPM0_MOD * rl_turn[gear_level];  //50% duty
            TPM0_C4V = 0;
            moving_status = 1;
            break;
          case 0x15: //stop
            TPM0_C5V = 0;
            TPM0_C2V = 0;
            TPM0_C3V = 0;
            TPM0_C4V = 0;
            moving_status = 0;
					  break;
          case 0x16: //gear up
            if (gear_level < 3)
              gear_level++;
						moving_status = 0;
            break;
          case 0x17: //gear down
            if (gear_level >= 1)
              gear_level--;
						moving_status = 0;
            break;
					case 0x21:
						end_point = 1; //reached end point
					  moving_status = 0;
					  break;
					case 0x22:
						end_point = 0; //replay the running music
					  moving_status = 0;
					  break;
          default:
            TPM0_C5V = 0;
            TPM0_C2V = 0;
            TPM0_C3V = 0;
            TPM0_C4V = 0;
        }
    }
}

void front_ledOff(void)
{
	PTC->PCOR = (MASK(FRONT_LED_1) | MASK(FRONT_LED_2) | MASK(FRONT_LED_3) | MASK(FRONT_LED_4) | MASK(FRONT_LED_5) | 
							 MASK(FRONT_LED_6) | MASK(FRONT_LED_7) | MASK(FRONT_LED_8) | MASK(FRONT_LED_9) | MASK(FRONT_LED_10));
}

void back_ledOff(void)
{
	PTC->PCOR = MASK(BACK_LED);
}

void front_led (void *argument) {
  for (;;) {
		if(moving_status == 0){
			PTC->PSOR = (MASK(FRONT_LED_1) | MASK(FRONT_LED_2) | MASK(FRONT_LED_3) | MASK(FRONT_LED_4) | MASK(FRONT_LED_5) | 
									 MASK(FRONT_LED_6) | MASK(FRONT_LED_7) | MASK(FRONT_LED_8) | MASK(FRONT_LED_9) | MASK(FRONT_LED_10));
			osDelay(100);
		}
		if(moving_status == 1){
			front_ledOff();
			PTC->PSOR = MASK(FRONT_LED_1);
			osDelay(100);
			PTC->PCOR = MASK(FRONT_LED_1);
		}
		if(moving_status == 1){
			PTC->PSOR = MASK(FRONT_LED_2);
			osDelay(100);
			PTC->PCOR = MASK(FRONT_LED_2);
		}
		if(moving_status == 1){
			PTC->PSOR = MASK(FRONT_LED_3);
			osDelay(100);
			PTC->PCOR = MASK(FRONT_LED_3);
		}
		if(moving_status == 1){
			PTC->PSOR = MASK(FRONT_LED_4);
			osDelay(100);
			PTC->PCOR = MASK(FRONT_LED_4);
		}
		if(moving_status == 1){
			PTC->PSOR = MASK(FRONT_LED_5);
			osDelay(100);
			PTC->PCOR = MASK(FRONT_LED_5);
		}
		if(moving_status == 1){
			PTC->PSOR = MASK(FRONT_LED_6);
			osDelay(100);
			PTC->PCOR = MASK(FRONT_LED_6);
		}
		if(moving_status == 1){
			PTC->PSOR = MASK(FRONT_LED_7);
			osDelay(100);
			PTC->PCOR = MASK(FRONT_LED_7);
		}
		if(moving_status == 1){
			PTC->PSOR = MASK(FRONT_LED_8);
			osDelay(100);
			PTC->PCOR = MASK(FRONT_LED_8);
		}
		if(moving_status == 1){
			PTC->PSOR = MASK(FRONT_LED_9);
			osDelay(100);
			PTC->PCOR = MASK(FRONT_LED_9);
		}
		if(moving_status == 1){
			PTC->PSOR = MASK(FRONT_LED_10);
			osDelay(100);
			PTC->PCOR = MASK(FRONT_LED_10);
		}
	}
}

void back_led (void *argument) {
  for (;;) {
		if (moving_status == 1){
			PTC->PSOR = MASK(BACK_LED);
			osDelay(500);
			PTC->PCOR = MASK(BACK_LED);
			osDelay(500);
		}
		if (moving_status == 0){
			PTC->PSOR = MASK(BACK_LED);
			osDelay(250);
			PTC->PCOR = MASK(BACK_LED);
			osDelay(250);
		}
	}
}

int musicalnotes(int i)
{
	if (i == 0)
		return 0;
	else if (i == 1)
		return 262;
	else if (i == 2)
		return 294;
	else if (i == 3)
		return 330;
	else if (i == 4)
		return 349;
	else if (i == 5)
		return 392;
	else if (i == 6)
		return 440;
	else if (i == 7)
		return 494;
	else 
		return 0;
}

int freq_2_MOD(int freq)
{
	if (freq == 0)
		return 0;
	return CLOCK_FREQ/PRE_SCALAR/freq;
}

static void Delay(void) {
    volatile unsigned int a;
    for (a=0; a < 4000000; a++) {
        __NOP(); 
    }
}

void buzzer (void *argument) {
	int i = 0;
	int j = 0;
	
	for(;;)
	{
		if (end_point == 0) {
			if (i == megNumNotes)
				i = 0;
			TPM1->MOD = freq_2_MOD(musicalnotes(megMelody[i]));
			TPM1_C0V = freq_2_MOD(musicalnotes(megMelody[i])) / 2;
			osDelay(megDurations[i]*300);
			i++;
		} 
		else if (end_point == 1)
		{
			if (j == megNumNotes_1) {
				j = 0;
			    end_point ++;
			}
			TPM1->MOD = freq_2_MOD(musicalnotes(megMelody_1[j]));
			TPM1_C0V = freq_2_MOD(musicalnotes(megMelody_1[j])) / 2;
			osDelay(megDurations_1[j]*300);
			j++;
		}
		else {
			TPM1_C0V = 0;
		}
	} 
}

 int main (void) {

    SystemCoreClockUpdate();
    initPWM();
    initUART2(BAUD_RATE);
	  InitGPIO();
    
    // Initialise all the CV values to 0
    TPM0_C5V = 0;  
    TPM0_C2V = 0;
    TPM0_C3V = 0;
    TPM0_C5V = 0;
  
    osKernelInitialize();                 // Initialize CMSIS-RTOS
    osThreadNew(front_led, NULL, NULL);    // Create application main thread
	  osThreadNew(buzzer, NULL, NULL);      // Create application main thread
    osThreadNew(back_led, NULL, NULL);    // Create application main thread
    osKernelStart();                      // Start thread execution
    for (;;) {}
}