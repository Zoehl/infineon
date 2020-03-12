#include "string.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"

#include "i2c_master.h"
#include "i2c_master_extern.h"
#include "i2c_master_conf.h"
#include "dps310.h"
#include "dps310_hal.h"
#include "optiga/pal/pal_i2c.h"

#define DPS310_ADDR  (0x77 << 1)

uint8_t tmp_data[20];
/*Instantiate driver state*/
struct dps310_state drv_state;
/*Instantiate bus connection callback holder*/
dps310_bus_connection cnn;


/* Should return -1 in case of failure otherwise valid contents*/
uint8_t dps310_read_byte(uint8_t reg_addr)
{
	tmp_data[0] = reg_addr;
	//I2C_MASTER_Transmit(&i2c_master_0,true,DPS310_ADDR,&tmp_data[0],1,false);

	// send start
	I2C_MASTER_SendStart(&i2c_master_0, DPS310_ADDR, XMC_I2C_CH_CMD_WRITE);
	while ((XMC_I2C_CH_GetStatusFlag(i2c_master_0.channel) & XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED) == 0U)
	{
		/* wait for ACK */
	}
	XMC_I2C_CH_ClearStatusFlag(i2c_master_0.channel, (uint32_t)XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED);

	I2C_MASTER_TransmitByte(&i2c_master_0, tmp_data[0]);
	/*make sure data is transmitted from FIFO*/
	while (!XMC_USIC_CH_TXFIFO_IsEmpty(i2c_master_0.channel)){}

	// i2c receive data
	//I2C_MASTER_Receive(&i2c_master_0,true,DPS310_ADDR,&tmp_data[0],1,true,true);
	// send repeated start
	I2C_MASTER_SendRepeatedStart(&i2c_master_0, DPS310_ADDR, XMC_I2C_CH_CMD_READ);
	while ((XMC_I2C_CH_GetStatusFlag(i2c_master_0.channel) & XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED) == 0U)
	{
		/* wait for ACK */
	}
	XMC_I2C_CH_ClearStatusFlag(i2c_master_0.channel, (uint32_t)XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED);

	while (XMC_USIC_CH_TXFIFO_IsFull(i2c_master_0.channel) == false)
	{
		I2C_MASTER_ReceiveNACK(&i2c_master_0);
		break;
	}
	/* wait for data to come in RX fifo */
	while (I2C_MASTER_IsRXFIFOEmpty(&i2c_master_0)){}
	tmp_data[0] = I2C_MASTER_GetReceivedByte(&i2c_master_0);

	// send stop
	I2C_MASTER_SendStop(&i2c_master_0);

	return tmp_data[0];
}

/* Should return -1 or negative value in case of failure otherwise length of
 * read contents in read_buffer
 * and shall place read contents in read_buffer
 */
uint8_t dps310_read_block(uint8_t reg_addr, uint8_t length, uint8_t *read_buffer)
{
	uint32_t buffer_index;
	uint32_t temp_index;

	if(length == 0)
		return 0;

	buffer_index = 0U;
	temp_index = 0U;
	tmp_data[0] = reg_addr;
	// send start
	I2C_MASTER_SendStart(&i2c_master_0, DPS310_ADDR, XMC_I2C_CH_CMD_WRITE);
	while ((XMC_I2C_CH_GetStatusFlag(i2c_master_0.channel) & XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED) == 0U)
	{
		/* wait for ACK */
	}
	I2C_MASTER_ClearFlag(&i2c_master_0, (uint32_t)XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED);

	I2C_MASTER_TransmitByte(&i2c_master_0, tmp_data[0]);
	/*make sure data is transmitted from FIFO*/
	while (!XMC_USIC_CH_TXFIFO_IsEmpty(i2c_master_0.channel)){}

	//I2C_MASTER_Receive(&i2c_master_0,true,DPS310_ADDR,read_buffer,length,true,true);
	I2C_MASTER_SendRepeatedStart(&i2c_master_0, DPS310_ADDR, XMC_I2C_CH_CMD_READ);
	while ((XMC_I2C_CH_GetStatusFlag(i2c_master_0.channel) & XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED) == 0U)
	{
		/* wait for ACK */
	}
	I2C_MASTER_ClearFlag(&i2c_master_0, (uint32_t)XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED);

	temp_index = buffer_index;
	while (temp_index < length)
	{
		while (XMC_USIC_CH_TXFIFO_IsFull(i2c_master_0.channel) == false)
		{
			/* transmit each byte till index reaches to the last byte */
			if(temp_index < length)
			{
				/* load the FIFO, byte by byte till either FIFO is full or all data is loaded*/
				if((temp_index + 1U) == length)
				{
					I2C_MASTER_ReceiveNACK(&i2c_master_0);
				}
				else
				{
					I2C_MASTER_ReceiveACK(&i2c_master_0);
				}
				temp_index++;
			}
			else
			{
				break;
			}
		}
		while (buffer_index < temp_index)
		{
			/* wait for data to come in RX fifo */
			while (I2C_MASTER_IsRXFIFOEmpty(&i2c_master_0)){}
			read_buffer[buffer_index++] = I2C_MASTER_GetReceivedByte(&i2c_master_0);
		}
	}

	// send stop
	I2C_MASTER_SendStop(&i2c_master_0);

	return length;
}

/* Should return -1 in case of failure otherwise non negative number*/
uint8_t dps310_write_byte(uint8_t reg_addr, uint8_t data)
{
	tmp_data[0] = reg_addr;
	tmp_data[1] = data;
	I2C_MASTER_SendStart(&i2c_master_0, DPS310_ADDR, XMC_I2C_CH_CMD_WRITE);
	while ((XMC_I2C_CH_GetStatusFlag(i2c_master_0.channel) & XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED) == 0U)
	{
		/* wait for ACK */
	}
	I2C_MASTER_ClearFlag(&i2c_master_0, (uint32_t)XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED);

	I2C_MASTER_TransmitByte(&i2c_master_0, tmp_data[0]);
	I2C_MASTER_TransmitByte(&i2c_master_0, tmp_data[1]);
	/*make sure data is transmitted from FIFO*/
	while (!XMC_USIC_CH_TXFIFO_IsEmpty(i2c_master_0.channel)){}

	I2C_MASTER_SendStop(&i2c_master_0);

	return 1;
}

/* Shall implement delay in milliseconds*/
void dps310_wait_ms(uint8_t delay)
{
	const TickType_t xDelay = pdMS_TO_TICKS(delay);
	vTaskDelay(xDelay);
}


int32_t dps310_hal_init(void)
{
	/* Assign/register platform specific bus handlers*/
	cnn.read_byte=dps310_read_byte;
	cnn.read_block=dps310_read_block;
	cnn.write_byte=dps310_write_byte;

	/*If platform doesn't support delay or sleep
	 *please assign NULL to this callback i.e cnn.delayms = NULL
	 */
	cnn.delayms = dps310_wait_ms;

	/*First call _init
	 * this function verifies chip with appropriate id and
	 * reads and stores calibration data, configures the sensor
	 * to meet default configuration set in dps310.h.
	 * This also puts the sensor in background mode
	 * making it measure both pressure and temperature continuously
	 */
	int ret = dps310_init(&drv_state, &cnn);

	/*To change configuration we first need to put sensor in
	 *idle mode by calling _standby
	 */
	if (ret < 0 )
		return -1;

	ret = dps310_standby(&drv_state);
	if (ret < 0 )
		return -1;

	//	printf("standby ret val = %d\n",ret);

	/* Now lets call _config to meet different output data rate (ODR)
	 * and oversampling rate (OSR) based on scenario and usecase
	 * For valid combinations please refer to Page 25 of datasheet
	 */
	ret = dps310_config(&drv_state,
			OSR_8,
			TMP_MR_2,
			OSR_128,
			PM_MR_128,
			drv_state.tmp_ext);
	if (ret < 0 )
		return -1;

	//	printf("config ret val = %d\n",ret);

	/*Resume the sensor in background mode again*/
	ret = dps310_resume(&drv_state);
	if (ret < 0 )
		return -1;

	double temp, press;
	ret = dps310_hal_get_temp_press(&temp, &press);
	printf("Temperature = %2.2f, Pressure = %4.2f\n",temp, press);

	return ret;
}

int32_t dps310_hal_get_temp_press(double* temp, double* press)
{
	int ret;
	if ((temp == NULL) || (press == NULL))
	{
		return -1;
	}

	ret = dps310_get_processed_data(&drv_state, press, temp);

	return ret;

}
