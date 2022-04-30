#include <msp430.h>
#include <stdint.h>

#if !defined(__I2CUTILS_H_)
#define __I2CUTILS_H_

#define SLAVE_ADDR 0x48

/* CMD_TYPE_X_SLAVE are example commands the master sends to the slave.
 * The slave will send example SlaveTypeX buffers in response.
 *
 * CMD_TYPE_X_MASTER are example commands the master sends to the slave.
 * The slave will initialize itself to receive MasterTypeX example buffers.
 * */

#define CMD_TYPE_0_SLAVE 0
#define CMD_TYPE_1_SLAVE 1 // get pwm pins value
#define CMD_TYPE_2_SLAVE 2

#define CMD_TYPE_0_MASTER 3
#define CMD_TYPE_1_MASTER 4
#define CMD_TYPE_2_MASTER 5

#define TYPE_0_LENGTH 1
#define TYPE_1_LENGTH 2
#define TYPE_2_LENGTH 6

#define MAX_BUFFER_SIZE 20

//******************************************************************************
// General I2C State Machine ***************************************************
//******************************************************************************

typedef enum I2C_ModeEnum
{
    IDLE_MODE,
    NACK_MODE,
    TX_REG_ADDRESS_MODE,
    RX_REG_ADDRESS_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    SWITCH_TO_RX_MODE,
    SWITHC_TO_TX_MODE,
    TIMEOUT_MODE
} I2C_Mode;

#define CHANNELS 8 // channels count

/*
 * Sensors resources
 */

extern uint16_t TicksCount[CHANNELS];
extern uint16_t TicksPer20MS[CHANNELS];

void initI2C();

/* Initialized the software state machine according to the received cmd
 *
 * cmd: The command/register address received
 * */
void I2C_Slave_ProcessCMD(uint8_t cmd);

/* The transaction between the slave and master is completed. Uses cmd
 * to do post transaction operations. (Place data from ReceiveBuffer
 * to the corresponding buffer based in the last received cmd)
 *
 * cmd: The command/register address corresponding to the completed
 * transaction
 */
void I2C_Slave_TransactionDone(uint8_t cmd);
void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count);
void CopyArray16(uint16_t *source, uint8_t *dest, uint8_t count);
#endif