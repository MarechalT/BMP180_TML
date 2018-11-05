/*
 BMP180_TML.h - BMP180_TML library for i2c - description
 Copyright (c) 2018 Thibault MARECHAL.  All right reserved.
 @: tmarechal.projects@gmail.com
 Github: https://github.com/MarechalT/BMP180_TML
 */

// ensure this library description is only included once
#ifndef BMP180_TML_H
#define BMP180_TML_H

// include types & constants of Wiring core API
#include "Wire.h"

// library interface description
class BMP180_TML {
public:
	BMP180_TML();
	struct CompParameters {
		int16_t _AC1;
		int16_t _AC2;
		int16_t _AC3;
		uint16_t _AC4;
		uint16_t _AC5;
		uint16_t _AC6;
		int16_t _B1;
		int16_t _B2;
		int16_t _MB;
		int16_t _MC;
		int16_t _MD;
	};
private:
	//oversampling setting
	enum oss {
		ULP = 0,		//Ultra low Power value 0 Same for Temperature measurement
		STD = 1,		//Standard
		HR = 2,			//High Resolution
		UHR = 3,			//Ultra high resolution
		MAX_VAL
	};
	//Compensation parameters structure (used in calculations)

	static const double delay_oss[oss::MAX_VAL] { 4.5, 7.5, 13.5, 25.5 };	// Min Delay between Computing command and reading
	static const int BMP180_ADDR = 0x77;				//I²C Address
	static const int BMP180_ID = 0x55;					//component ID

	//
	CompParameters cp;
	int32_t _up;
	int32_t _ut;
	int32_t _B5;
	uint8_t _command;
	uint8_t _oss_mode;

	/*!Ensure the communication with the chip.*/
	bool init();
	/*!Send the order to measure the pressure.*/
	bool measurePressure();
	/*!Send the order to measure the temperature.*/
	bool measureTemperature();
	/*!Return true if the device has finished computing the value and is ready to be read.*/
	bool hasValue();
	/*!Get the pressure.*/
	double getPressure();
	/*!Get the temperature.*/
	double getTemperature();
	/*!Read the pressure (Measure + get).*/
	double readPressure();
	/*!Read the temperature (Measure + get).*/
	double readTemperature();

	/*!Read the ID.*/
	uint8_t readID();
	/*!Read a register value and apply a mask. */
	uint8_t readRegisterValue(uint8_t reg, uint8_t mask);
	/*!Read several register value and apply a mask. */
	uint8_t readRegisterValue(uint8_t reg, uint8_t mask, uint8_t length);
	/*!Read a register value.*/
	uint8_t readRegister(uint8_t reg);
	/*!Read several registers value.*/
	uint32_t readRegister(uint8_t reg, uint8_t length);
	/*! Read all compensation parameters.*/
	CompParameters readCompensationParameters();

	/*! Get masked shifts.*/
	uint8_t getMaskShift(uint8_t mask);
	/*! Set Masked bits */
	uint8_t setMaskedBits(uint8_t reg, uint8_t mask, uint8_t value);

	/*!Write a value in a part of a register.*/
	void writeRegisterValue(uint8_t reg, uint8_t mask, uint8_t value);
	/*!Write a value in a register.*/
	void writeRegister(uint8_t reg, uint8_t value);

	/*! Set all compensation registers to 0 */
	void resetCompensationParameters();
	/*! Reset the chip */
	void reset();
	/*! get the oversampling setting */
	uint8_t getOss();
	/*! set the oversampling setting */
	void setOss(uint8_t ossValue);

	/*! All available registers that will be used for measuring and setting purposes */
	enum BMP180_Registers {
		BMP180_REG_ID = 0xD0,					//Always 0x55
		BMP180_REG_RESET = 0xE0,				//Write only register (set to 0xB6 = reset)
		BMP180_REG_MEASUREMENT_CONTROL = 0xF4,	//sets data acquisition options of device
		BMP180_REG_OUTPUT = 0xF6,				//Results

		BMP180_REG_CP_AC1 = 0xAA,				//Compensation Parameter AC1 register
		BMP180_REG_CP_AC2 = 0xAC,				//Compensation Parameter AC2 register
		BMP180_REG_CP_AC3 = 0xAE,				//Compensation Parameter AC3 register
		BMP180_REG_CP_AC4 = 0xB0,				//Compensation Parameter AC4 register
		BMP180_REG_CP_AC5 = 0xB2,				//Compensation Parameter AC5 register
		BMP180_REG_CP_AC6 = 0xB4,				//Compensation Parameter AC6 register
		BMP180_REG_CP_B1 = 0xB6,				//Compensation Parameter B1 register
		BMP180_REG_CP_B2 = 0xB8,				//Compensation Parameter B2 register
		BMP180_REG_CP_MB = 0xBA,				//Compensation Parameter MB register
		BMP180_REG_CP_MC = 0xBC,				//Compensation Parameter MC register
		BMP180_REG_CP_MD = 0xBE					//Compensation Parameter MD register
	};

	/*! Masks used for device bit fields */
	enum BMP_Masks {
		BMP180_MASK_OSS = 0b11000000,					//Over sampling settings BMP180_REG_OUTPUT
		BMP180_MASK_SCO = 0b00100000, 					//Start of conversion BMP180_REG_OUTPUT 1=measuring 0=idle
		BMP180_MASK_MEASUREMENT_CONTROL = 0b00011111,	//Output BMP180_REG_OUTPUT
		BMP180_MASK_ALL = 0b11111111,
		BMP180_MASK_CP = 0xFFFF
	};

	enum BMP180_Mask_32bit_t
		: uint32_t
		{	//register 0xF6
			BMP180_MASK_PRESSURE = 0x00FFFFFF,		//20 bits
		BMP180_MASK_TEMPERATURE = 0x0000FFFF,		//16 bits
	};
	enum BMP180_Commands {
		BMP180_CMD_TEMPERATURE = 0x2E,			//start temperature conversion
		BMP180_CMD_PRESSURE = 0x34,				//start pressure conversion += MASK_OSS
		BMP180_CMD_RESET = 0xB6,				//Reset command
	};

};

#endif
