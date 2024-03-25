/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "sound.h"
#include "notes.h"
#include "basic.h"
#include "PWM.h"

 
/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"
#include "MKL25Z4.h"                    // Device header

#define CLOCK_FREQ 48000000
#define PRE_SCALAR 128
#define DURATION_UNIT 0.08

#define PTB0_Pin 0

void initPWM(void)     // Defines a function named initPWM that takes no arguments
{
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;  // Enables the clock gating for port B module by setting the corresponding bit in the System Integration Module (SIM) SCGC5 register

    PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;      // Clears the MUX field for pin PTB0 on port B (probably to set it to a certain function)
    PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);         // Sets the MUX field for pin PTB0 on port B to the alternative function 3

    SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;               // Enables the clock for TPM1 module by setting the corresponding bit in the SIM SCGC6 register

    SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;            // Clears the TPM clock source field in the SIM SOPT2 register
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);               // Sets the TPM clock source to option 1 in the SIM SOPT2 register

    TPM1->MOD = 7500;                                 // Sets the modulo value for the TPM1 module; 

    TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));   // Clears the CMOD (clock mode selection) and PS (prescale factor) fields in the TPM1 SC (status and control) register
    TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));            // Sets the CMOD to 1 (probably to enable the counter) and the prescale factor to 7

		TPM1->SC &= ~(TPM_SC_CPWMS_MASK);                 // set the counter to up counting mode (0: upcounting, 1: up-down counting)

    TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));  // Clears the channel status and control fields for channel 0 of TPM1
    TPM1_C0SC |= ((TPM_CnSC_ELSB(1)) | TPM_CnSC_MSB(1));     // Sets the ELSB and MSB fields for channel 0 of TPM1. 
}

