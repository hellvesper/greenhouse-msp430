//******************************************************************************
//   MSP430G2xx3 Demo - USCI_B0, I2C Slave multiple byte TX/RX
//
//   Description: I2C master communicates to I2C slave sending and receiving
//   3 different messages of different length. (This is the slave code). The
//   slave will be in LPM0 mode, waiting for the master to initiate the
//   communication. The slave will send/receive bytes based on the master's
//   request. The slave will handle I2C bytes sent/received using the
//   I2C interrupt.
//   ACLK = NA, MCLK = SMCLK = DCO 16MHz.
//
//
//                   MSP430G2553         3.3V
//                 -----------------   /|\ /|\
//            /|\ |                 |   |  4.7k
//             |  |                 |  4.7k |
//             ---|RST              |   |   |
//                |                 |   |   |
//                |             P1.6|---|---+- I2C Clock (UCB0SCL)
//                |                 |   |
//                |             P1.7|---+----- I2C Data (UCB0SDA)
//                |                 |
//                |                 |
//
//   Nima Eskandari
//   Texas Instruments Inc.
//   April 2017
//   Built with CCS V7.0
//******************************************************************************

#include <msp430.h>
#include <stdint.h>
#include "I2CUtils.h"

//******************************************************************************
// Example Commands ************************************************************
//******************************************************************************

/* CMD_TYPE_X_SLAVE are example commands the master sends to the slave.
 * The slave will send example SlaveTypeX buffers in response.
 *
 * CMD_TYPE_X_MASTER are example commands the master sends to the slave.
 * The slave will initialize itself to receive MasterTypeX example buffers.
 * */

#define CHECK_BIT(var, pos) (((var) >> (pos)) & 1)

//******************************************************************************
// Device Initialization *******************************************************
//******************************************************************************

void initClockTo16MHz()
{
	if (CALBC1_16MHZ == 0xFF) // If calibration constant erased
	{
		while (1)
			; // do not load, trap CPU!!
	}
	DCOCTL = 0;				// Select lowest DCOx and MODx settings
	BCSCTL1 = CALBC1_16MHZ; // Set DCO
	DCOCTL = CALDCO_16MHZ;
}

void setupInterruptForPin(uint8_t pin, uint8_t port)
{
	if (port == 1)
	{
		P1OUT = pin;   // P2.4 set, else reset
		P1REN |= pin;  // P2.4 pullup
		P1IE |= pin;   // P2.4 interrupt enabled
		P1IES &= ~pin; // P2.4 lo/Hi edge
		P1IFG &= ~pin; // P2.4 clear interrupt flag
	}
	else if (port == 2)
	{
		P2OUT = pin;   // P2.4 set, else reset
		P2REN |= pin;  // P2.4 pullup
		P2IE |= pin;   // P2.4 interrupt enabled
		P2IES &= ~pin; // P2.4 lo/Hi edge
		P2IFG &= ~pin; // P2.4 clear interrupt flag
	}
}

void initGPIO()
{
	P1SEL |= BIT6 + BIT7;  // Assign I2C pins to USCI_B0
	P1SEL2 |= BIT6 + BIT7; // Assign I2C pins to USCI_B0

	P1DIR |= 0x01; // P1.0 output, else input

	/*
	 * Setup pin 2.4 as interrupt trigger
	 */
	// P2SEL |= BIT3; // Assign pin 	P2.3 for interrupt ticks counter
	// P2DIR = 0x01;  // P1.0 output, else input
	setupInterruptForPin(BIT4, 2);
	// P2OUT &= ~BIT4; // P2.4 set, else reset
	// P2REN |= BIT4;	// P2.4 pullup
	// P2IE |= BIT4;	// P2.4 interrupt enabled
	// P2IES &= ~BIT4; // P2.4 lo/Hi edge
	// P2IFG &= ~BIT4; // P2.4 clear interrupt flag
}

void initTimer()
{
	// Configuration word
	// Bits 15-10: Unused
	// Bits 9-8: Clock source select: set to SMCLK (16MHz)
	// Bits 7-6: Input divider: set to 8
	// Bits 5-4: Mode control: Count up to TACCRO and reset
	// Bit 3: Unused
	// Bits 2: TACLR : set to initially clear timer system
	// Bit 1: Enable interrupts from TA0
	// Bit 0: Interrupt (pending) flag : set to zero (initially)
	// TA0CTL = 0b0000001011010010;
	// TACCR0 = 2000;	// Set TACCR0 = 2000 to generate a 1ms timebase @ 16MHz with a divisor of 8
	TACCTL0 = CCIE;					// Enable interrupts when TAR = TACCR0
	TACCR0 = 40000;					// Set TACCR0 = 40000 to generate a 20ms timebase @ 16MHz with a divisor of 8
	TACTL = TASSEL_2 + MC_1 + ID_3; // SMCLK, upmode
}

//******************************************************************************
// Main ************************************************************************
// Enters LPM0 and waits for I2C interrupts. The data sent from the master is  *
// then interpreted and the device will respond accordingly                    *
//******************************************************************************

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

	initClockTo16MHz();
	initGPIO();
	initI2C();
	initTimer();

	__bis_SR_register(LPM0_bits + GIE);
	return 0;
}

//******************************************************************************
// Port 2 Interrupt Service Routine ********************************************
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = PORT2_VECTOR
__interrupt void Port_2(void)
#elif defined(__GNUC__)
void __attribute__((interrupt(PORT2_VECTOR))) Port_2(void)
#else
#error Compiler not supported!
#endif
{
	for (uint_fast8_t i = 0; i < CHANNELS; i++)
	{
		if (CHECK_BIT(P2IFG, i))
		{
			P2IFG &= ~(1 << i); // P2.i IFG cleared
			TicksCount[i]++;	// Increment ticks on channel
		}
	}
}

//******************************************************************************
// Timer A0 Interrupt Service Routine ******************************************
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void timerA0ISR(void)
#elif defined(__GNUC__)
void __attribute__((interrupt(TIMER0_A0_VECTOR))) timerA0ISR(void)
#else
#error Compiler not supported!
#endif
{
	// Rised every 20ms
	for (uint8_t i = 0; i < CHANNELS; i++)
	{
		TicksPer20MS[i] = TicksCount[i]; // save ticks counted per 20ms
		// TicksPer20MS[i] = 60000U; // save ticks counted per 20ms
		TicksCount[i] = 0; // reset counter
	}

	P1OUT ^= 0x01; // P1.0 = toggle
}