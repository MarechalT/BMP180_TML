/*
 BMP180_TML.h - BMP180_TML library for i2c - description
 Copyright (c) 2018 Thibault MARECHAL.  All right reserved.
 @: tmarechal.projects@gmail.com
 Github: https://github.com/MarechalT/BMP180_TML
 */

// include this library's description file
#include "BMP180_TML.h"
#include <math.h>
#include <Arduino.h>

const double BMP180_TML::delay_oss[] = { 4.5, 7.5, 13.5, 25.5 };

BMP180_TML::BMP180_TML() :
		_command(0x00), _oss_mode(BMP180_TML::UHR), _up(0), _ut(0), _B5(0) {
	resetCompensationParameters();
}

BMP180_TML::~BMP180_TML() {
} //empty destructor : no dynamic memory to clean

bool BMP180_TML::init() {
	if (readID() != BMP180_TML::BMP180_ID)
		return false;
	readCompensationParameters();
	return true;
}

uint8_t BMP180_TML::readID() {
	return readRegisterValue(BMP180_REG_ID, BMP180_MASK_ALL);
}

uint8_t BMP180_TML::readRegisterValue(uint8_t reg, uint8_t mask) {
	return getMaskedBits(readRegister(reg), mask);
}
uint32_t BMP180_TML::readRegisterValue(uint8_t reg, uint32_t mask, uint32_t length) {
	return getMaskedBits(readRegister(reg, length), mask);
}

uint32_t BMP180_TML::readRegister(uint8_t reg, uint8_t length) {
	if (length > 4)
		return 0L;
	uint32_t data = 0L;

	Wire.beginTransmission(BMP180_ADDR);
	Wire.write(reg);
	Wire.endTransmission(false);
	Wire.requestFrom(BMP180_ADDR, static_cast<uint8_t>(length));

	for (uint8_t i = 0; i < length; i++) {
		data <<= 8;
		data |= Wire.read();
	}
	return data;
}

uint8_t BMP180_TML::readRegister(uint8_t reg) {
	Wire.beginTransmission(BMP180_ADDR);
	Wire.write(reg);
	Wire.endTransmission(false);
	Wire.requestFrom(BMP180_ADDR, static_cast<uint8_t>(1));
	return Wire.read();
}

uint8_t BMP180_TML::getOss() {
	return _oss_mode;
}
void BMP180_TML::setOss(uint8_t ossValue) {
	switch (ossValue) {
	case oss::ULP:
	case oss::STD:
	case oss::HR:
	case oss::UHR:
		_oss_mode = ossValue;
		break;
	default:
		_oss_mode = oss::ULP;
	}
}

void BMP180_TML::resetCompensationParameters() {
	cp = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};	//define all compensation values at 0;
}

uint8_t BMP180_TML::getMaskShift(uint8_t mask) {
	uint8_t value = 0;
	//count how many times the mask must be shifted right until the lowest bit is set
	if (mask != 0) {
		while (!(mask & 1)) {
			value++;
			mask >>= 1;
		}
	}
	return value;
}

uint8_t BMP180_TML::setMaskedBits(uint8_t reg, uint8_t mask, uint8_t value) {
	//Clear all mask bits
	reg &= (~mask);
	//set masked bits in register according to value
	return ((value << getMaskShift(mask)) & mask) | reg;
}

void BMP180_TML::readCompensationParameters() {
	resetCompensationParameters();
	cp._AC1 = static_cast<int16_t>(readRegisterValue(BMP180_REG_CP_AC1, BMP180_MASK_CP, 2));
	cp._AC2 = static_cast<int16_t>(readRegisterValue(BMP180_REG_CP_AC2, BMP180_MASK_CP, 2));
	cp._AC3 = static_cast<int16_t>(readRegisterValue(BMP180_REG_CP_AC3, BMP180_MASK_CP, 2));
	cp._AC4 = readRegisterValue(BMP180_REG_CP_AC4, BMP180_MASK_CP, 2);
	cp._AC5 = readRegisterValue(BMP180_REG_CP_AC5, BMP180_MASK_CP, 2);
	cp._AC6 = readRegisterValue(BMP180_REG_CP_AC6, BMP180_MASK_CP, 2);
	cp._B1 = static_cast<int16_t>(readRegisterValue(BMP180_REG_CP_B1, BMP180_MASK_CP, 2));
	cp._B2 = static_cast<int16_t>(readRegisterValue(BMP180_REG_CP_B2, BMP180_MASK_CP, 2));
	cp._MB = static_cast<int16_t>(readRegisterValue(BMP180_REG_CP_MB, BMP180_MASK_CP, 2));
	cp._MC = static_cast<int16_t>(readRegisterValue(BMP180_REG_CP_MC, BMP180_MASK_CP, 2));
	cp._MD = static_cast<int16_t>(readRegisterValue(BMP180_REG_CP_MD, BMP180_MASK_CP, 2));
}

bool BMP180_TML::measurePressure() {
	//check if measurement in progress:
	if (readRegisterValue(BMP180_REG_MEASUREMENT_CONTROL, BMP180_MASK_SCO))
		return false;
	_command = BMP180_CMD_PRESSURE;
	writeRegisterValue(BMP180_REG_MEASUREMENT_CONTROL, BMP180_MASK_OSS | BMP180_MASK_SCO | BMP180_MASK_MEASUREMENT_CONTROL,
			(_oss_mode << 6) | _command);
	return true;
}

bool BMP180_TML::measureTemperature() {
	//check if measurement in progress:
	if (readRegisterValue(BMP180_REG_MEASUREMENT_CONTROL, BMP180_MASK_SCO))
		return false;
	_command = BMP180_CMD_TEMPERATURE;
	writeRegisterValue(BMP180_REG_MEASUREMENT_CONTROL, BMP180_MASK_SCO | BMP180_MASK_MEASUREMENT_CONTROL, _command);

	return true;
}

void BMP180_TML::writeRegisterValue(uint8_t reg, uint8_t mask, uint8_t value) {
	uint8_t reg_value = readRegister(reg);
	writeRegister(reg, setMaskedBits(reg_value, mask, value));
}

void BMP180_TML::writeRegister(uint8_t reg, uint8_t value) {
	Wire.beginTransmission(BMP180_ADDR);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
}

double BMP180_TML::getPressure() {
	int32_t pressure = 0;
	int32_t B6 = _B5 - 4000;
	int32_t X1 = (static_cast<int32_t>(cp._B2) * ((B6 * B6) >> 12)) >> 11;
	int32_t X2 = (static_cast<int32_t>(cp._AC2) * B6) >> 11;
	int32_t X3 = X1 + X2;
	int32_t B3 = ((((static_cast<int32_t>(cp._AC1) << 2) + X3) << _oss_mode) + 2) >> 2;
	X1 = (static_cast<int32_t>(cp._AC3) * B6) >> 13;
	X2 = (static_cast<int32_t>(cp._B1) * ((B6 * B6) >> 12)) >> 16;
	X3 = (X1 + X2 + 2) >> 2;
	uint32_t B4 = static_cast<uint32_t>(cp._AC4) * (static_cast<uint32_t>(X3 + 32768)) >> 15;
	uint32_t B7 = static_cast<uint32_t>(_up - B3) * (50000 >> _oss_mode);
	if (B7 < 0x80000000)
		pressure = (B7 << 1) / B4;
	else
		pressure = (B7 / B4) << 1;
	X1 = (pressure >> 8) * (pressure >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * pressure) >> 16;
	pressure = pressure + ((X1 + X2 + 3791) >> 4);
	return static_cast<double>(pressure);
}

double BMP180_TML::getTemperature() {
	int32_t X1 = (_ut - static_cast<int32_t>(cp._AC6)) * static_cast<int32_t>(cp._AC5) >> 15;
	int32_t X2 = (static_cast<int32_t>(cp._MC) << 11) / (X1 + static_cast<int32_t>(cp._MD));
	_B5 = X1 + X2;
	return static_cast<double>(((_B5 + 8) >> 4) * 0.1); //calculation gives T° in 0.1 °C
}

double BMP180_TML::readPressure() {
	if (!measurePressure())
		return NAN;
	do {
		delay(BMP180_TML::delay_oss[_oss_mode]);
	} while (!hasValue());
	return getPressure();
}

double BMP180_TML::readTemperature() {
	if (!measureTemperature())
		return NAN;
	do {
		delay(delay_oss[oss::ULP]);
	} while (!hasValue());
	return getTemperature();
}

bool BMP180_TML::hasValue() {
	if (readRegisterValue(BMP180_REG_MEASUREMENT_CONTROL, BMP180_MASK_SCO))
		return false;

	switch (_command) {
	case BMP180_CMD_PRESSURE:
		_up = static_cast<int32_t>(readRegisterValue(BMP180_REG_OUTPUT, BMP180_MASK_PRESSURE, 3));
		_up >>= (8 - _oss_mode);
		break;
	case BMP180_CMD_TEMPERATURE:
		_ut = static_cast<int32_t>(readRegisterValue(BMP180_REG_OUTPUT, BMP180_MASK_TEMPERATURE, 2));
		break;
	default:
		return false;
	}
	return true;
}

void BMP180_TML::reset() {
	_command = BMP180_CMD_RESET;
	writeRegister(BMP180_REG_RESET, _command);
}

BMP180_TML::CompParameters BMP180_TML::getCP() {
	return cp;
}
