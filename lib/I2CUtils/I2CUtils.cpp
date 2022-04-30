#include "I2CUtils.h"

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

//******************************************************************************
// Example Commands ************************************************************
//******************************************************************************

uint16_t TicksCount[CHANNELS] = {0};
uint16_t TicksPer20MS[CHANNELS] = {0};

/* MasterTypeX are example buffers initialized in the master, they will be
 * sent by the master to the slave.
 * SlaveTypeX are example buffers initialized in the slave, they will be
 * sent by the slave to the master.
 * */

uint8_t MasterType2[TYPE_2_LENGTH] = {0};
uint8_t MasterType1[TYPE_1_LENGTH] = {0, 0};
uint8_t MasterType0[TYPE_0_LENGTH] = {0};

uint8_t SlaveType2[TYPE_2_LENGTH] = {'A', 'B', 'C', 'D', '1', '2'};
uint8_t SlaveType1[TYPE_1_LENGTH] = {15, 16};
uint8_t SlaveType0[TYPE_0_LENGTH] = {12};

//******************************************************************************
// General I2C State Machine ***************************************************
//******************************************************************************

/* Used to track the state of the software state machine*/
I2C_Mode SlaveMode = RX_REG_ADDRESS_MODE;

/* The Register Address/Command to use*/
uint8_t ReceiveRegAddr = 0;

/* ReceiveBuffer: Buffer used to receive data in the ISR
 * RXByteCtr: Number of bytes left to receive
 * ReceiveIndex: The index of the next byte to be received in ReceiveBuffer
 * TransmitBuffer: Buffer used to transmit data in the ISR
 * TXByteCtr: Number of bytes left to transfer
 * TransmitIndex: The index of the next byte to be transmitted in TransmitBuffer
 * */
uint8_t ReceiveBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t RXByteCtr = 0;
uint8_t ReceiveIndex = 0;
uint8_t TransmitBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t TXByteCtr = 0;
uint8_t TransmitIndex = 0;

void I2C_Slave_ProcessCMD(uint8_t cmd)
{
	ReceiveIndex = 0;
	TransmitIndex = 0;
	RXByteCtr = 0;
	TXByteCtr = 0;

	switch (cmd)
	{
	case (CMD_TYPE_0_SLAVE): // Send slave device id (This device's id)
		SlaveMode = TX_DATA_MODE;
		TXByteCtr = TYPE_0_LENGTH;
		// Fill out the TransmitBuffer
		CopyArray(SlaveType0, TransmitBuffer, TYPE_0_LENGTH);
		IE2 &= ~UCB0RXIE; // Disable RX interrupt
		IE2 |= UCB0TXIE;  // Enable TX interrupt
		break;
	case (CMD_TYPE_1_SLAVE): // Send slave device time (This device's time)
		SlaveMode = TX_DATA_MODE;
		TXByteCtr = CHANNELS * 2;
		// Fill out the TransmitBuffer
		CopyArray16(TicksPer20MS, TransmitBuffer, CHANNELS);
		IE2 &= ~UCB0RXIE; // Disable RX interrupt
		IE2 |= UCB0TXIE;  // Enable TX interrupt
		break;
	case (CMD_TYPE_2_SLAVE): // Send slave device location (This device's location)
		SlaveMode = TX_DATA_MODE;
		TXByteCtr = TYPE_2_LENGTH;
		// Fill out the TransmitBuffer
		CopyArray(SlaveType2, TransmitBuffer, TYPE_2_LENGTH);
		IE2 &= ~UCB0RXIE; // Disable RX interrupt
		IE2 |= UCB0TXIE;  // Enable TX interrupt
		break;
	case (CMD_TYPE_0_MASTER):
		SlaveMode = RX_DATA_MODE;
		RXByteCtr = TYPE_0_LENGTH;
		IE2 &= ~UCB0TXIE; // Disable RX interrupt
		IE2 |= UCB0RXIE;  // Enable TX interrupt
		break;
	case (CMD_TYPE_1_MASTER):
		SlaveMode = RX_DATA_MODE;
		RXByteCtr = TYPE_1_LENGTH;
		IE2 &= ~UCB0TXIE; // Disable RX interrupt
		IE2 |= UCB0RXIE;  // Enable TX interrupt
		break;
	case (CMD_TYPE_2_MASTER):
		SlaveMode = RX_DATA_MODE;
		RXByteCtr = TYPE_2_LENGTH;
		IE2 &= ~UCB0TXIE; // Disable RX interrupt
		IE2 |= UCB0RXIE;  // Enable TX interrupt
		break;
	default:
		__no_operation();
		break;
	}
}

void I2C_Slave_TransactionDone(uint8_t cmd)
{
	switch (cmd)
	{
	case (CMD_TYPE_0_SLAVE): // Slave device id was sent(This device's id)
		break;
	case (CMD_TYPE_1_SLAVE): // Slave device time was sent(This device's time)
		break;
	case (CMD_TYPE_2_SLAVE): // Send slave device location (This device's location)
		break;
	case (CMD_TYPE_0_MASTER): // Recieve data from master
		CopyArray(ReceiveBuffer, MasterType0, TYPE_0_LENGTH);
		break;
	case (CMD_TYPE_1_MASTER): // Recieve data from master
		CopyArray(ReceiveBuffer, MasterType1, TYPE_1_LENGTH);
		break;
	case (CMD_TYPE_2_MASTER): // Recieve data from master
		CopyArray(ReceiveBuffer, MasterType2, TYPE_2_LENGTH);
		break;
	default:
		__no_operation();
		break;
	}
}

void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count)
{
	uint8_t copyIndex = 0;
	for (copyIndex = 0; copyIndex < count; copyIndex++)
	{
		dest[copyIndex] = source[copyIndex];
	}
}

void CopyArray16(uint16_t *source, uint8_t *dest, uint8_t count)
{
	uint8_t copyIndex = 0;
	for (copyIndex = 0; copyIndex < count; copyIndex++)
	{

		/*
		 * CopyIndex sequence:
		 * 0 - 0,1
		 * 1 - 2,3
		 * 2 - 4,5
		 * dest array will be x2 longer length
		 */
		dest[(copyIndex + copyIndex)] = source[copyIndex] >> 8;
		dest[(copyIndex + copyIndex + 1)] = (source[copyIndex] << 8) >> 8;
	}
}

void initI2C()
{
	UCB0CTL1 |= UCSWRST;			// Enable SW reset
	UCB0CTL0 = UCMODE_3 + UCSYNC;	// I2C Slave, synchronous mode
	UCB0I2COA = SLAVE_ADDR;			// Own Address
	UCB0CTL1 &= ~UCSWRST;			// Clear SW reset, resume operation
	UCB0I2CIE |= UCSTPIE + UCSTTIE; // Enable STT and STP interrupt
	IE2 |= UCB0RXIE;				// Enable RX interrupt
}

//******************************************************************************
// I2C Interrupt For Received and Transmitted Data******************************
//******************************************************************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
#elif defined(__GNUC__)
void __attribute__((interrupt(USCIAB0TX_VECTOR))) USCIAB0TX_ISR(void)
#else
#error Compiler not supported!
#endif
{
	if (IFG2 & UCB0RXIFG) // Receive Data Interrupt
	{
		// Must read from UCB0RXBUF
		uint8_t rx_val = UCB0RXBUF;
		switch (SlaveMode)
		{
		case (RX_REG_ADDRESS_MODE):
			ReceiveRegAddr = rx_val;
			I2C_Slave_ProcessCMD(ReceiveRegAddr);
			break;
		case (RX_DATA_MODE):
			ReceiveBuffer[ReceiveIndex++] = rx_val;
			RXByteCtr--;
			if (RXByteCtr == 0)
			{
				// Done Receiving MSG
				SlaveMode = RX_REG_ADDRESS_MODE;
				IE2 &= ~(UCB0TXIE);
				IE2 |= UCB0RXIE; // Enable RX interrupt
				I2C_Slave_TransactionDone(ReceiveRegAddr);
			}
			break;
		default:
			__no_operation();
			break;
		}
	}
	else if (IFG2 & UCB0TXIFG) // Transmit Data Interrupt
	{
		// Must write to UCB0TXBUF
		switch (SlaveMode)
		{
		case (TX_DATA_MODE):
			UCB0TXBUF = TransmitBuffer[TransmitIndex++];
			TXByteCtr--;
			if (TXByteCtr == 0)
			{
				// Done Transmitting MSG
				SlaveMode = RX_REG_ADDRESS_MODE;
				IE2 &= ~(UCB0TXIE);
				IE2 |= UCB0RXIE; // Enable RX interrupt
				I2C_Slave_TransactionDone(ReceiveRegAddr);
			}
			break;
		default:
			__no_operation();
			break;
		}
	}
}

//******************************************************************************
// I2C Interrupt For Start, Restart, Nack, Stop ********************************
//******************************************************************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)
#elif defined(__GNUC__)
void __attribute__((interrupt(USCIAB0RX_VECTOR))) USCIAB0RX_ISR(void)
#else
#error Compiler not supported!
#endif
{
	if (UCB0STAT & UCSTPIFG) // Stop or NACK Interrupt
	{
		UCB0STAT &=
			~(UCSTTIFG + UCSTPIFG + UCNACKIFG); // Clear START/STOP/NACK Flags
	}
	if (UCB0STAT & UCSTTIFG)
	{
		UCB0STAT &= ~(UCSTTIFG); // Clear START Flags
	}
}
