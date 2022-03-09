/*
 * acs37800.h
 *
 * Created: 18.02.2021 10:11:05
 *  Author: gfcwfzkm
 */ 


#ifndef ACS37800_H_
#define ACS37800_H_

#include <inttypes.h>
#include <math.h>	// for fabs()

/* Shadow Register Access of the EEPROM
 * Add the EEPROM offset to directly access it. */
#define ACS37800_EEPROM_OFFSET		-0x10
#define ACS37800_R_CURRENT_TRIM		0x1B
#define ACS37800_R_AVERAGE_SETTING	0x1C
#define ACS37800_R_DELAY_OVERCURR	0x1D
#define ACS37800_R_SAMPLING_SETTING	0x1E
#define ACS37800_R_I2C_CONFIG		0x1F

/* Other normal, volatile memory */
#define ACS37800_R_IRMS_VRMS		0x20	// 16 bits of IRMS / VRMS
#define ACS37800_R_PIMG_PACT		0x21
#define ACS37800_R_PFACTOR_PAPPAR	0x22
#define ACS37800_R_SAMPLES_USED		0x25
#define ACS37800_R_VIRMSAVGONESEC	0x26
#define ACS37800_R_VIRMSAVGONEMIN	0x27
#define ACS37800_R_PACTAVGONESEC	0x28
#define ACS37800_R_PACTAVGONEMIN	0x29
#define ACS37800_R_ICODES_VCODES	0x2A
#define ACS37800_R_PINSTANT			0x2C
#define ACS37800_R_STATUS			0x2D
#define ACS37800_R_ACCESS_CODE		0x2F
#define ACS37800_R_ACCESS_CODE_KEY	0x4F70656E
#define ACS37800_R_CUSTOMER_ACCESS	0x30

enum ACS_ERROR{
	ACS_NO_ERROR	= 0,
	ACS_IOERROR		= 1
};

enum ACS_STATUS{
	ACS_NO_EVENT		= 0x00,
	ACS_ZEROCROSSOUT	= 0x01,
	ACS_FAULTOUT		= 0x02,
	ACS_FAULTLATCHED	= 0x04,
	ACS_OVERVOLTAGE		= 0x08,
	ACS_UNDERVOLTAGE	= 0x10
};

enum ACS_PFACT_DETAILS{
	ACS_POSANGLE_LEADING	= 0x00,
	ACS_POSANGLE_LAGGING	= 0x01,
	ACS_POSFACTOR_GENERATED	= 0x00,
	ACS_POSFACTOR_CONSUMED	= 0x02
};

typedef struct {
	enum ACS_ERROR error;
	uint8_t i2c_address;
	uint16_t maxVoltage;				// Needed to calculate V & P correctly
	uint8_t maxCurrent;					// Needed to calculate I & P correctly
	void *ioInterface;					// Pointer to the IO/Peripheral Interface library
	// Any return value by the IO interface functions have to return zero when successful or
	// non-zero when not successful.
	uint8_t (*startTransaction)(void*);	// Prepare the IO/Peripheral Interface for a transaction
	uint8_t (*sendBytes)(void*,			// Send data function pointer: InterfacePointer,
	uint8_t,		// Address of the PortExpander (8-Bit Address Format!),
	uint8_t*,		// Pointer to send buffer,
	uint16_t);		// Amount of bytes to send
	uint8_t (*transceiveBytes)(void*,			// Get data function pointer:InterfacePointer,
	uint8_t,		// Address of the PortExpander (8-Bit Address Format!),
	uint8_t*,		// Pointer to receive buffer,
	uint16_t);		// Amount of bytes to receive
	uint8_t (*getBytes)(void*,			// Get data function pointer:InterfacePointer,
	uint8_t,		// Address of the PortExpander (8-Bit Address Format!),
	uint8_t*,		// Pointer to receive buffer,
	uint16_t);		// Amount of bytes to receive
	uint8_t (*endTransaction)(void*);	// Finish the transaction / Release IO/Peripheral
} acs37800_t;

void acs_setupI2C(acs37800_t* acs, uint8_t i2c_address,
		void *ioInterface, uint8_t (*startTransaction)(void*),
		uint8_t (*sendBytes)(void*,uint8_t,uint8_t*,uint16_t),
		uint8_t (*getBytes)(void*,uint8_t,uint8_t*,uint16_t),
		uint8_t (*endTransaction)(void*));

void acs_setupSPI(acs37800_t *acs,
		void *ioInterface, uint8_t (*startTransaction)(void*),
		uint8_t (*transceiveBytes)(void*, uint8_t, uint8_t*, uint16_t),
		uint8_t (*endTransaction)(void*));

enum ACS_ERROR acs_init(acs37800_t *acs, uint16_t maxVoltage, uint8_t maxCurrent);

/* Basic Chip Access and Value Conversion */
void acs_writeReg(acs37800_t *acs_sensor, uint8_t regAddr, uint32_t regVal);

uint32_t acs_readReg(acs37800_t *acs_sensor, uint8_t regAddr);

float acs_ConvertUnsignedFixedPoint(uint32_t inVal, uint8_t binaryPoint, uint8_t width);

float acs_ConvertSignedFixedPoint(uint32_t inVal, uint8_t binaryPoint, uint8_t width);

int32_t acs_SignExtendBitfield(uint32_t data, uint16_t width);

/* Basic Read Functions - values calculated with voltage and current scale */
void acs_getVIRMS(acs37800_t *acs_sensor, float *vrms, float *irms);

void acs_getPACTIMAG(acs37800_t *acs_sensor, float *pActive, float *pImag);

void acs_getPAPPFACT(acs37800_t *acs_sensor, float *pApparent, float *pFactor, enum ACS_PFACT_DETAILS *pfDetails);

void acs_getVIAVGSEC(acs37800_t *acs_sensor, float *vrmsAvgSec, float *irmsAvgSec);

void acs_getVIAVGMIN(acs37800_t *acs_sensor, float *vrmsAvgMin, float *irmsAvgMin);

void acs_getPACTAVGSEC(acs37800_t *acs_sensor, float *pActiveAvgSec);

void acs_getPACTAVGMIN(acs37800_t *acs_sensor, float *pActiveAvgMin);

void acs_getInstantVI(acs37800_t *acs_sensor, float *vcodes, float *icodes);

void acs_getInstantP(acs37800_t *acs_sensor, float *pinstant);

enum ACS_STATUS acs_getStatus(acs37800_t *acs_sensor);

#endif /* ACS37800_H_ */