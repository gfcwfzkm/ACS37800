/*
 * acs37800.c
 *
 * Created: 18.02.2021 10:11:16
 *  Author: gfcwfzkm
 */ 

#include "acs37800.h"

#define ACS_READ	0x80
#define ACS_WRITE	0x00

void acs_setupI2C(acs37800_t* acs, uint8_t i2c_address,
		void *ioInterface, uint8_t (*startTransaction)(void*),
		uint8_t (*sendBytes)(void*,uint8_t,uint8_t*,uint16_t),
		uint8_t (*getBytes)(void*,uint8_t,uint8_t*,uint16_t),
		uint8_t (*endTransaction)(void*))
{
	acs->i2c_address = i2c_address;
	acs->ioInterface = ioInterface;
	acs->startTransaction = startTransaction;
	acs->sendBytes = sendBytes;
	acs->transceiveBytes = 0;
	acs->getBytes = getBytes;
	acs->endTransaction = endTransaction;
}

void acs_setupSPI(acs37800_t *acs,
		void *ioInterface, uint8_t (*startTransaction)(void*),
		uint8_t (*transceiveBytes)(void*, uint8_t, uint8_t*, uint16_t),
		uint8_t (*endTransaction)(void*))
{
	acs->i2c_address = 0;
	acs->ioInterface = ioInterface;
	acs->startTransaction = startTransaction;
	acs->sendBytes = 0;
	acs->transceiveBytes = transceiveBytes;
	acs->getBytes = 0;
	acs->endTransaction = endTransaction;
}

enum ACS_ERROR acs_init(acs37800_t *acs, uint16_t maxVoltage, uint8_t maxCurrent)
{
	acs->maxCurrent = maxCurrent;
	acs->maxVoltage = maxVoltage;
	acs->error = ACS_NO_ERROR;
	
	uint32_t testProbe;
	
	testProbe = acs_readReg(acs, ACS37800_R_CURRENT_TRIM);
	if ( (testProbe == 0xFFFFFFFF) || (testProbe == 0x00000000) )
	{
		acs->error = ACS_IOERROR;
	}
	
	return acs->error;
}

/* Basic Chip Access and Value Conversion */
void acs_writeReg(acs37800_t *acs_sensor, uint8_t regAddr, uint32_t regVal)
{
	uint8_t dataBuf[5];
	
	/* Setup Data Package */
	if (acs_sensor->sendBytes && acs_sensor->getBytes)
	{
		dataBuf[0] = regAddr;
	}
	else
	{
		dataBuf[0] = (regAddr & 0x7F) | ACS_WRITE;
	}
	dataBuf[1] = regVal & 0xFF;
	dataBuf[2] = (regVal >> 8) & 0xFF;
	dataBuf[3] = (regVal >> 16) & 0xFF;
	dataBuf[4] = (regVal >> 24) & 0xFF;
	
	acs_sensor->error |= acs_sensor->startTransaction(acs_sensor->ioInterface);
	if (acs_sensor->sendBytes && acs_sensor->getBytes)
	{
		acs_sensor->error |= acs_sensor->sendBytes(acs_sensor->ioInterface, acs_sensor->i2c_address | ACS_WRITE, dataBuf, 5);
	}
	else
	{
		acs_sensor->error |= acs_sensor->transceiveBytes(acs_sensor->ioInterface, 0, dataBuf, 5);
	}
	acs_sensor->error |= acs_sensor->endTransaction(acs_sensor->ioInterface);
}

uint32_t acs_readReg(acs37800_t *acs_sensor, uint8_t regAddr)
{
	uint8_t dataBuf[4] = {0,0,0,0};
	uint32_t regVal = 0;
	
	acs_sensor->error |= acs_sensor->startTransaction(acs_sensor->ioInterface);
	
	if (acs_sensor->sendBytes && acs_sensor->getBytes)
	{
		acs_sensor->error |= acs_sensor->sendBytes(acs_sensor->ioInterface, acs_sensor->i2c_address | ACS_WRITE, dataBuf, 1);
		acs_sensor->error |= acs_sensor->sendBytes(acs_sensor->ioInterface, acs_sensor->i2c_address | ACS_READ, dataBuf, 4);
	}
	else
	{
		acs_sensor->error |= acs_sensor->transceiveBytes(acs_sensor->ioInterface,
				regAddr | ACS_READ, dataBuf, 4);
		acs_sensor->endTransaction(acs_sensor->ioInterface);
		
		acs_sensor->startTransaction(acs_sensor->ioInterface);
		acs_sensor->error |= acs_sensor->transceiveBytes(acs_sensor->ioInterface,
				regAddr | ACS_READ, dataBuf, 4);
		
	}
	
	acs_sensor->error |= acs_sensor->endTransaction(acs_sensor->ioInterface);
	
	regVal = (uint32_t)dataBuf[0] | (uint32_t)dataBuf[1] << 8 | (uint32_t)dataBuf[2] << 16 | (uint32_t)dataBuf[3] << 24;
	
	return regVal;
}

/****** Following three functions written by Jiri Prokes, Allegro MicroSystems, LCC, from the Example Source Code ********/
/*
 * Convert an unsigned bitfield which is right justified, into a floating point number
 *
 *    data        - the bitfield to be converted
 *    binaryPoint - the binary point (the bit to the left of the binary point)
 *    width       - the width of the bitfield
 *    returns     - the floating point number
 */
float acs_ConvertUnsignedFixedPoint(uint32_t inVal, uint8_t binaryPoint, uint8_t width)
{
	uint32_t mask;

	if (width == 32)
	{
		mask = 0xFFFFFFFF;
	}
	else
	{
		mask = (1UL << width) - 1UL;
	}

	return (float)(inVal & mask) / (float)(1L << binaryPoint);
}

/*
 * Convert a signed bitfield which is right justified, into a floating point number
 *
 *    data        - the bitfield to be sign extended then converted
 *    binaryPoint - the binary point (the bit to the left of the binary point)
 *    width       - the width of the bitfield
 *    returns     - the floating point number
 */
float acs_ConvertSignedFixedPoint(uint32_t inVal, uint8_t binaryPoint, uint8_t width)
{
	int32_t signedValue = acs_SignExtendBitfield(inVal, width);
	return (float)signedValue / (float)(1L << binaryPoint);
}

/*
 * Sign extend a bitfield which if right justified
 *
 *    data        - the bitfield to be sign extended
 *    width       - the width of the bitfield
 *    returns     - the sign extended bitfield
 */
int32_t acs_SignExtendBitfield(uint32_t data, uint16_t width)
{
	if (width == 32)
	{
		return (int32_t)data;
	}

	int32_t x = (int32_t)data;
	int32_t mask = 1L << (width - 1);

	x = x & ((1 << width) - 1); // make sure the upper bits are zero

	return (int32_t)((x ^ mask) - mask);
}

void acs_getVIRMS(acs37800_t *acs_sensor, float *vrms, float *irms)
{
	uint32_t tempReg;
	
	acs_sensor->error = ACS_NO_ERROR;
	tempReg = acs_readReg(acs_sensor, ACS37800_R_IRMS_VRMS);
	
	*vrms = 1.19 * acs_ConvertUnsignedFixedPoint(tempReg & 0xFFFF, 16, 16) * acs_sensor->maxVoltage;
	*irms = 1.19 * acs_ConvertUnsignedFixedPoint((tempReg >> 16) & 0xFFFF, 16, 16) * acs_sensor->maxCurrent;
}

void acs_getPACTIMAG(acs37800_t *acs_sensor, float *pActive, float *pImag)
{
	uint32_t tempReg;
	
	acs_sensor->error = ACS_NO_ERROR;
	tempReg = acs_readReg(acs_sensor, ACS37800_R_PIMG_PACT);
	
	*pActive = acs_ConvertSignedFixedPoint(tempReg & 0xFFFF, 15, 16) * acs_sensor->maxVoltage * acs_sensor->maxCurrent / 1.42;
	*pImag = acs_ConvertUnsignedFixedPoint((tempReg >> 16) & 0xFFFF, 16, 16) * acs_sensor->maxVoltage * acs_sensor->maxCurrent / 1.42;
	
	*pImag = acs_ConvertUnsignedFixedPoint((tempReg >> 16) & 0xFFFF, 16, 16);
}

void acs_getPAPPFACT(acs37800_t *acs_sensor, float *pApparent, float *pFactor, enum ACS_PFACT_DETAILS *pfDetails)
{
	uint32_t tempReg;
	
	acs_sensor->error = ACS_NO_ERROR;
	tempReg = acs_readReg(acs_sensor, ACS37800_R_PFACTOR_PAPPAR);
	
	*pApparent = acs_ConvertUnsignedFixedPoint(tempReg & 0xFFFF, 15, 16) * acs_sensor->maxVoltage * acs_sensor->maxCurrent / 1.42;
	
	*pFactor = fabs(acs_ConvertSignedFixedPoint((tempReg >> 16) & 0x7FF, 10, 11));
	if (tempReg & (1L<<27))	*pfDetails = ACS_POSANGLE_LAGGING;
	if (tempReg & (1L<<28))	*pfDetails |= ACS_POSFACTOR_CONSUMED;
}

void acs_getVIAVGSEC(acs37800_t *acs_sensor, float *vrmsAvgSec, float *irmsAvgSec)
{
	uint32_t tempReg;
	
	acs_sensor->error = ACS_NO_ERROR;
	tempReg = acs_readReg(acs_sensor, ACS37800_R_VIRMSAVGONESEC);
	
	*vrmsAvgSec = acs_ConvertUnsignedFixedPoint(tempReg & 0xFFFF, 16, 16) * acs_sensor->maxVoltage;
	*irmsAvgSec = acs_ConvertUnsignedFixedPoint((tempReg >> 16) & 0xFFFF, 16, 16) * acs_sensor->maxCurrent;
}

void acs_getVIAVGMIN(acs37800_t *acs_sensor, float *vrmsAvgMin, float *irmsAvgMin)
{
	uint32_t tempReg;
	
	acs_sensor->error = ACS_NO_ERROR;
	tempReg = acs_readReg(acs_sensor, ACS37800_R_VIRMSAVGONEMIN);
	
	*vrmsAvgMin = acs_ConvertUnsignedFixedPoint(tempReg & 0xFFFF, 16, 16) * acs_sensor->maxVoltage;
	*irmsAvgMin = acs_ConvertUnsignedFixedPoint((tempReg >> 16) & 0xFFFF, 16, 16) * acs_sensor->maxCurrent;
}

void acs_getPACTAVGSEC(acs37800_t *acs_sensor, float *pActiveAvgSec)
{
	uint32_t tempReg;
	
	acs_sensor->error = ACS_NO_ERROR;
	tempReg = acs_readReg(acs_sensor, ACS37800_R_PACTAVGONESEC);
	
	*pActiveAvgSec = acs_ConvertSignedFixedPoint(tempReg & 0xFFFF, 15, 16) * acs_sensor->maxVoltage * acs_sensor->maxCurrent;
}

void acs_getPACTAVGMIN(acs37800_t *acs_sensor, float *pActiveAvgMin)
{
	uint32_t tempReg;
	
	acs_sensor->error = ACS_NO_ERROR;
	tempReg = acs_readReg(acs_sensor, ACS37800_R_PACTAVGONEMIN);
	
	*pActiveAvgMin = acs_ConvertSignedFixedPoint(tempReg & 0xFFFF, 15, 16) * acs_sensor->maxVoltage * acs_sensor->maxCurrent;
}

void acs_getInstantVI(acs37800_t *acs_sensor, float *vcodes, float *icodes)
{
	uint32_t tempReg;
	
	acs_sensor->error = ACS_NO_ERROR;
	tempReg = acs_readReg(acs_sensor, ACS37800_R_ICODES_VCODES);
	
	*vcodes = acs_ConvertSignedFixedPoint(tempReg & 0xFFFF, 15, 16) * acs_sensor->maxVoltage;
	*icodes = acs_ConvertSignedFixedPoint((tempReg >> 16) & 0xFFFF, 15, 16) * acs_sensor->maxCurrent;
}

void acs_getInstantP(acs37800_t *acs_sensor, float *pinstant)
{
	uint32_t tempReg;
	
	acs_sensor->error = ACS_NO_ERROR;
	tempReg = acs_readReg(acs_sensor, ACS37800_R_PINSTANT);
	
	*pinstant = acs_ConvertSignedFixedPoint(tempReg & 0xFFFF, 15, 16) * acs_sensor->maxVoltage * acs_sensor->maxCurrent;
}

enum ACS_STATUS acs_getStatus(acs37800_t *acs_sensor)
{
	uint32_t tempReg;
	
	acs_sensor->error = ACS_NO_ERROR;
	tempReg = acs_readReg(acs_sensor, ACS37800_R_STATUS);
	
	return (uint8_t)tempReg;
}
