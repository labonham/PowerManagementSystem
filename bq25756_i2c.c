#include "bq25756_i2c.h"

#define WAIT_FOR_FLAG(flag, value, timeout, errorcode)	I2CTimeout = timeout;\
	while (I2C_GetFlagStatus(SENSORS_I2C, flag) != value) {\
		if ((I2CTimeout--) == 0)\
			return I2Cx_TIMEOUT_UserCallback(errorcode);\
	} \

#define CLEAR_ADDR_BIT	I2C_ReadRegister(SENSORS_I2C, I2C_Register_SR1);\
	I2C_ReadRegister(SENSORS_I2C, I2C_Register_SR2);\


static unsigned long ST_Sensors_I2C_WriteRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue);
static unsigned long ST_Sensors_I2C_ReadRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue);
static unsigned long ST_Sensors_I2C_WriteNoRegister(unsigned char Address, unsigned char RegisterAddr);
static unsigned long ST_Sensors_I2C_ReadNoRegister(unsigned char Address, unsigned short RegisterLen, unsigned char *RegisterValue);

void I2cMaster_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	RCC_APB1PeriphClockCmd(SENSORS_I2C_RCC_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(SENSORS_I2C_SCL_GPIO_CLK | SENSORS_I2C_SDA_GPIO_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  SENSORS_I2C_SCL_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_PinAFConfig(SENSORS_I2C_SCL_GPIO_PORT, SENSORS_I2C_SCL_GPIO_PINSOURCE, SENSORS_I2C_AF);
	GPIO_Init(SENSORS_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = SENSORS_I2C_SDA_GPIO_PIN;
	GPIO_PinAFConfig(SENSORS_I2C_SDA_GPIO_PORT, SENSORS_I2C_SDA_GPIO_PINSOURCE, SENSORS_I2C_AF);
	GPIO_Init(SENSORS_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);
	I2C_DeInit(SENSORS_I2C);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = I2C_OWN_ADDRESS;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
	I2C_Cmd(SENSORS_I2C, ENABLE);
	I2C_Init(SENSORS_I2C, &I2C_InitStructure);
	return;
}

static uint32_t I2Cx_TIMEOUT_UserCallback(char value)
{
	I2C_InitTypeDef I2C_InitStructure;

	I2C_GenerateSTOP(SENSORS_I2C, ENABLE);
	I2C_SoftwareResetCmd(SENSORS_I2C, ENABLE);
	I2C_SoftwareResetCmd(SENSORS_I2C, DISABLE);
	I2C_DeInit(SENSORS_I2C);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = I2C_OWN_ADDRESS;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
	I2C_Cmd(SENSORS_I2C, ENABLE);
	I2C_Init(SENSORS_I2C, &I2C_InitStructure);
	return 1;
}

int Sensors_I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr,
	unsigned short len, const unsigned char *data_ptr)
{
	char retries = 0;
	int ret = 0;
	unsigned short retry_in_mlsec = Get_I2C_Retry();

tryWriteAgain:
	ret = 0;
	ret = ST_Sensors_I2C_WriteRegister(slave_addr, reg_addr, len, data_ptr);
	if (ret && retry_in_mlsec) {
		if (retries++ > 4)
			return ret;
		goto tryWriteAgain;
	}
	return ret;
}

int clamp(int value, int min, int max)
{
	if (value < min)
		return min;
	else if (value > max)
		return max;
	else
		return value;
}

int Sensors_I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr,
	unsigned short len, unsigned char *data_ptr)
{
	char retries = 0;
	int ret = 0;
	unsigned short retry_in_mlsec = Get_I2C_Retry();

tryReadAgain:
	ret = 0;
	ret = ST_Sensors_I2C_ReadRegister(slave_addr, reg_addr, len, data_ptr);
	if (ret && retry_in_mlsec) {
		if (retries++ > 4)
			return ret;
		goto tryReadAgain;
	}
	return ret;
}

static unsigned long ST_Sensors_I2C_WriteRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue)
{
	uint32_t  result = 0;
	uint32_t  i = 0;
	__IO uint32_t  I2CTimeout = I2Cx_LONG_TIMEOUT;

	WAIT_FOR_FLAG(I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 1);
	I2C_GenerateSTART(SENSORS_I2C, ENABLE);
	WAIT_FOR_FLAG(I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 2);
	I2C_Send7bitAddress(SENSORS_I2C, (Address << 1), I2C_Direction_Transmitter);
	WAIT_FOR_FLAG(I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 3);
	CLEAR_ADDR_BIT
	WAIT_FOR_FLAG(I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 4);
	I2C_SendData(SENSORS_I2C, RegisterAddr);
	for (i = 0; i < (RegisterLen); i++) {
		WAIT_FOR_FLAG(I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 5);
		I2C_SendData(SENSORS_I2C, RegisterValue[i]);
	}
	WAIT_FOR_FLAG (I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 6);
	I2C_GenerateSTOP(SENSORS_I2C, ENABLE);

	return result;
}
static unsigned long ST_Sensors_I2C_ReadRegister(unsigned char Address,
	unsigned char RegisterAddr,
	unsigned short RegisterLen, unsigned char *RegisterValue)
{
	uint32_t i = 0, result = 0;
	__IO uint32_t  I2CTimeout = I2Cx_LONG_TIMEOUT;

	WAIT_FOR_FLAG(I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 7);
	I2C_GenerateSTART(SENSORS_I2C, ENABLE);
	WAIT_FOR_FLAG(I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 8);
	I2C_Send7bitAddress(SENSORS_I2C, (Address<<1), I2C_Direction_Transmitter);
	WAIT_FOR_FLAG(I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 9);
	CLEAR_ADDR_BIT;
	WAIT_FOR_FLAG(I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 10);
	I2C_SendData(SENSORS_I2C, RegisterAddr);
	WAIT_FOR_FLAG(I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 11);
	I2C_GenerateSTART(SENSORS_I2C, ENABLE);
	WAIT_FOR_FLAG(I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 12);
	I2C_Send7bitAddress(SENSORS_I2C, (Address<<1), I2C_Direction_Receiver);
	WAIT_FOR_FLAG(I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 13);
	if (RegisterLen == 1) {
		I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);
		CLEAR_ADDR_BIT;
		I2C_GenerateSTOP(SENSORS_I2C, ENABLE);
		WAIT_FOR_FLAG(I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 14);
		RegisterValue[0] = I2C_ReceiveData(SENSORS_I2C);
	} else if (RegisterLen == 2) {
		I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);
		SENSORS_I2C->CR1 |= I2C_CR1_POS;
		CLEAR_ADDR_BIT;
		WAIT_FOR_FLAG(I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 15);
		I2C_GenerateSTOP(SENSORS_I2C, ENABLE);
		RegisterValue[0] = I2C_ReceiveData(SENSORS_I2C);
		RegisterValue[1] = I2C_ReceiveData(SENSORS_I2C);
	} else if (RegisterLen == 3) {
		CLEAR_ADDR_BIT;
		WAIT_FOR_FLAG(I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 16);
		I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);
		RegisterValue[0] = I2C_ReceiveData(SENSORS_I2C);
		I2C_GenerateSTOP(SENSORS_I2C, ENABLE);
		RegisterValue[1] = I2C_ReceiveData(SENSORS_I2C);
		WAIT_FOR_FLAG(I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 17);
		RegisterValue[2] = I2C_ReceiveData(SENSORS_I2C);
	} else {
		CLEAR_ADDR_BIT;
		for (i = 0; i < (RegisterLen); i++) {
			if (i == (RegisterLen-3)) {
				WAIT_FOR_FLAG(I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 16);
				I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);
				RegisterValue[i++] = I2C_ReceiveData(SENSORS_I2C);
				I2C_GenerateSTOP(SENSORS_I2C, ENABLE);
				RegisterValue[i++] = I2C_ReceiveData(SENSORS_I2C);
				WAIT_FOR_FLAG(I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 17);
				RegisterValue[i++] = I2C_ReceiveData(SENSORS_I2C);
				goto endReadLoop;
			}
			WAIT_FOR_FLAG(I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 18);
			RegisterValue[i] = I2C_ReceiveData(SENSORS_I2C);
		}
	}
endReadLoop:
	I2C_ClearFlag(SENSORS_I2C, I2C_FLAG_BTF);
	WAIT_FOR_FLAG(I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 19);
	I2C_AcknowledgeConfig(SENSORS_I2C, ENABLE);
	SENSORS_I2C->CR1 &= ~I2C_CR1_POS;
	return result;
}

int Sensors_I2C_WriteNoRegister(unsigned char slave_addr, unsigned char reg_addr)
{
	char retries = 0;
	int ret = 0;
	unsigned short retry_in_mlsec = Get_I2C_Retry();

tryWriteAgain:
	ret = 0;
	ret = ST_Sensors_I2C_WriteNoRegister(slave_addr, reg_addr);
	if (ret && retry_in_mlsec) {
		if (retries++ > 4)
			return ret;
		goto tryWriteAgain;
	}
	return ret;
}

int Sensors_I2C_ReadNoRegister(unsigned char slave_addr,
	unsigned short len, unsigned char *data_ptr)
{
	char retries = 0;
	int ret = 0;
	unsigned short retry_in_mlsec = Get_I2C_Retry();

tryReadAgain:
	ret = 0;
	ret = ST_Sensors_I2C_ReadNoRegister(slave_addr, len, data_ptr);
	if (ret && retry_in_mlsec) {
		if (retries++ > 4)
			return ret;
		goto tryReadAgain;
	}
	return ret;
}

static unsigned long ST_Sensors_I2C_WriteNoRegister(unsigned char Address, unsigned char RegisterAddr)
{
	uint32_t  result = 0;
	__IO uint32_t  I2CTimeout = I2Cx_LONG_TIMEOUT;

	WAIT_FOR_FLAG(I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 1);
	I2C_GenerateSTART(SENSORS_I2C, ENABLE);
	WAIT_FOR_FLAG(I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 2);
	I2C_Send7bitAddress(SENSORS_I2C, (Address<<1), I2C_Direction_Transmitter);
	WAIT_FOR_FLAG(I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 3);
	CLEAR_ADDR_BIT
	WAIT_FOR_FLAG(I2C_FLAG_TXE, SET, I2Cx_FLAG_TIMEOUT, 4);
	I2C_SendData(SENSORS_I2C, RegisterAddr);
	WAIT_FOR_FLAG(I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 6);
	I2C_GenerateSTOP(SENSORS_I2C, ENABLE);
	return result;
}

static unsigned long ST_Sensors_I2C_ReadNoRegister(unsigned char Address, unsigned short RegisterLen, unsigned char *RegisterValue)
{
	uint32_t i = 0, result = 0;
	__IO uint32_t  I2CTimeout = I2Cx_LONG_TIMEOUT;

	WAIT_FOR_FLAG(I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 7);
	I2C_GenerateSTART(SENSORS_I2C, ENABLE);
	WAIT_FOR_FLAG(I2C_FLAG_SB, SET, I2Cx_FLAG_TIMEOUT, 12);
	I2C_Send7bitAddress(SENSORS_I2C, (Address << 1), I2C_Direction_Receiver);
	WAIT_FOR_FLAG(I2C_FLAG_ADDR, SET, I2Cx_FLAG_TIMEOUT, 13);
	if (RegisterLen == 1) {
		I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);
		CLEAR_ADDR_BIT;
		I2C_GenerateSTOP(SENSORS_I2C, ENABLE);
		WAIT_FOR_FLAG(I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 14);
		RegisterValue[0] = I2C_ReceiveData(SENSORS_I2C);
	} else if (RegisterLen == 2) {
		I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);
		SENSORS_I2C->CR1 |= I2C_CR1_POS;
		CLEAR_ADDR_BIT;
		WAIT_FOR_FLAG(I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 15);
		I2C_GenerateSTOP(SENSORS_I2C, ENABLE);
		RegisterValue[0] = I2C_ReceiveData(SENSORS_I2C);
		RegisterValue[1] = I2C_ReceiveData(SENSORS_I2C);
	} else if (RegisterLen == 3) {
		CLEAR_ADDR_BIT;
		WAIT_FOR_FLAG(I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 16);
		I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);
		RegisterValue[0] = I2C_ReceiveData(SENSORS_I2C);
		I2C_GenerateSTOP(SENSORS_I2C, ENABLE);
		RegisterValue[1] = I2C_ReceiveData(SENSORS_I2C);
		WAIT_FOR_FLAG(I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 17);
		RegisterValue[2] = I2C_ReceiveData(SENSORS_I2C);
	} else {
		CLEAR_ADDR_BIT;
		for (i = 0; i < (RegisterLen); i++) {
			if (i == (RegisterLen-3)) {
				WAIT_FOR_FLAG(I2C_FLAG_BTF, SET, I2Cx_FLAG_TIMEOUT, 16);
				I2C_AcknowledgeConfig(SENSORS_I2C, DISABLE);
				RegisterValue[i++] = I2C_ReceiveData(SENSORS_I2C);
				I2C_GenerateSTOP(SENSORS_I2C, ENABLE);
				RegisterValue[i++] = I2C_ReceiveData(SENSORS_I2C);
				WAIT_FOR_FLAG(I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 17);
				RegisterValue[i++] = I2C_ReceiveData(SENSORS_I2C);
				goto endReadLoop;
			}
			WAIT_FOR_FLAG(I2C_FLAG_RXNE, SET, I2Cx_FLAG_TIMEOUT, 18);
		RegisterValue[i] = I2C_ReceiveData(SENSORS_I2C);
		}
	}

endReadLoop:
	I2C_ClearFlag(SENSORS_I2C, I2C_FLAG_BTF);
	WAIT_FOR_FLAG(I2C_FLAG_BUSY, RESET, I2Cx_LONG_TIMEOUT, 19);
	I2C_AcknowledgeConfig(SENSORS_I2C, ENABLE);
	SENSORS_I2C->CR1 &= ~I2C_CR1_POS;
	return result;
}
int BQ25756_WriteReg(unsigned char ADD, u8 reg_add, u8 reg_dat)
{
	unsigned char ret;

	ret = Sensors_I2C_WriteRegister(ADD, reg_add, 1, &reg_dat);
	return ret;
}

int BQ25756_ReadReg(unsigned char ADD, u8 reg_add, unsigned char *Read, u8 num)
{
	Sensors_I2C_ReadRegister(ADD, reg_add, num, Read);
	return 0;
}

static unsigned short RETRY_IN_MLSEC  = 55;

void Set_I2C_Retry(unsigned short ml_sec)
{
	RETRY_IN_MLSEC = ml_sec;
}

unsigned short Get_I2C_Retry(void)
{
	return RETRY_IN_MLSEC;
}
