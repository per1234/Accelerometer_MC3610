
#include <Arduino.h>
#include <Wire.h>
#include "MC3610.h"

#define MC3610_CFG_I2C_ADDR    					(0x6C)
#define MC3610_CFG_MODE_DEFAULT    				MC3610_MODE_STANDBY
#define MC3610_CFG_SAMPLE_RATE_CWAKE_DEFAULT    MC3610_CWAKE_SR_DEFAULT_50Hz
#define MC3610_CFG_SAMPLE_RATE_SNIFF_DEFAULT    MC3610_SNIFF_SR_0p4Hz
#define MC3610_CFG_RANGE_DEFAULT         		MC3610_RANGE_8G
#define MC3610_CFG_RESOLUTION_DEFAULT    		MC3610_RESOLUTION_14BIT
#define MC3610_CFG_ORIENTATION_MAP_DEFAULT    	ORIENTATION_TOP_RIGHT_UP

uint8_t CfgRange, CfgResolution;	

 // Read register bit
bool MC3610::readRegisterBit(uint8_t reg, uint8_t pos)
{
    uint8_t value;
    value = readRegister8(reg);
    return ((value >> pos) & 1);
}

// Write register bit
void MC3610::writeRegisterBit(uint8_t reg, uint8_t pos, bool state)
{
    uint8_t value;
    value = readRegister8(reg);

    if (state)
        value |= (1 << pos);
    else
        value &= ~(1 << pos);

    writeRegister8(reg, value);
}

// Read 8-bit from register
uint8_t MC3610::readRegister8(uint8_t reg)
{
	uint8_t value;
	Wire.beginTransmission(MC3610_CFG_I2C_ADDR);
	Wire.write(reg);
	Wire.endTransmission(false); //endTransmission but keep the connection active
	Wire.requestFrom(MC3610_CFG_I2C_ADDR, 1); //Once done, bus is released by default
    value = Wire.read();
    return value;
}          

// Write 8-bit to register
void MC3610::writeRegister8(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(MC3610_CFG_I2C_ADDR);
	Wire.write(reg);
	Wire.write(value);
    Wire.endTransmission();
}

// Read 16-bit from register
int16_t MC3610::readRegister16(uint8_t reg)
{
    int16_t value;
    Wire.beginTransmission(MC3610_CFG_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(MC3610_CFG_I2C_ADDR, 2);
    while(!Wire.available()) {};
    uint8_t vha = Wire.read();
    uint8_t vla = Wire.read();

    value = vha << 8 | vla;
    return value;
}

// Write 8-bit from register
void MC3610::writeRegister16(uint8_t reg, int16_t value)
{
    Wire.beginTransmission(MC3610_CFG_I2C_ADDR);
	Wire.write(reg);
	Wire.write((uint8_t)(value >> 8));
	Wire.write((uint8_t)value);
    Wire.endTransmission();
}

// Repeated Read Byte(s) from register
void MC3610::readRegisters(uint8_t reg, byte *buffer, uint8_t len)
{
	Wire.beginTransmission(MC3610_CFG_I2C_ADDR);
	Wire.write(reg);
	Wire.endTransmission(false); //endTransmission but keep the connection active
	Wire.requestFrom(MC3610_CFG_I2C_ADDR, len); //Ask for bytes, once done, bus is released by default

	while(Wire.available() < len); //Hang out until we get the # of bytes we expect
	for(int x = 0 ; x < len ; x++)
		buffer[x] = Wire.read();
}
  
//Set the operation mode  
void MC3610::SetMode(mc3610_mode_t mode)
{
   uint8_t value;
   value = readRegister8(MC3610_REG_MODE_C);
   value &= 0b11110000;
   value |= mode;
   writeRegister8(MC3610_REG_MODE_C, value);
}

//Set the range control
void MC3610::SetRangeCtrl(mc3610_range_t range)
{
	uint8_t value;    
	CfgRange = range;
	SetMode(MC3610_MODE_STANDBY);
	value = readRegister8(MC3610_REG_RANGE_C);
    value &= 0b00000111;
    value |= (range << 4)&0x70 ;
	writeRegister8(MC3610_REG_RANGE_C, value);
}

//Set the resolution control
void MC3610::SetResolutionCtrl(mc3610_resolution_t resolution)
{
	uint8_t value;
	CfgResolution = resolution;
	SetMode(MC3610_MODE_STANDBY);
	value = readRegister8(MC3610_REG_RANGE_C);
    value &= 0b01110000;
    value |= resolution;
	writeRegister8(MC3610_REG_RANGE_C, value);
}

//Set the sampling rate
void MC3610::SetCWakeSampleRate(mc3610_cwake_sr_t sample_rate)
{
	uint8_t value;
	SetMode(MC3610_MODE_STANDBY);
	value = readRegister8(MC3610_REG_WAKE_C);
	value &= 0b00000000;
	value |= sample_rate;
	writeRegister8(MC3610_REG_WAKE_C, value);
}

//Get the output sampling rate
mc3610_cwake_sr_t MC3610::GetCWakeSampleRate(void)
{
	/* Read the data format register to preserve bits */
	uint8_t value;
	value = readRegister8(MC3610_REG_WAKE_C);
	value &= 0b00000111;
	return (mc3610_cwake_sr_t) (value);
}

//Get the range control
mc3610_range_t MC3610::GetRangeCtrl(void)
{
  	/* Read the data format register to preserve bits */
  	uint8_t value;
  	value = readRegister8(MC3610_REG_RANGE_C);
  	value &= 0x70;
  	return (mc3610_range_t) (value >> 4);
}

//Get the range control
mc3610_resolution_t MC3610::GetResolutionCtrl(void)
{
  	/* Read the data format register to preserve bits */
  	uint8_t value;
  	value = readRegister8(MC3610_REG_RANGE_C);
  	value &= 0x07;
  	return (mc3610_resolution_t) (value);
}

//Initialize the MC3610 sensor and set as the default configuration
bool MC3610::start(void)
{
   	Wire.begin(); // Initialize I2C
    SetMode(MC3610_MODE_STANDBY);
     /* Check I2C connection */
	uint8_t id = readRegister8(MC3610_REG_PROD);
 	if (id != 0x70)
    {
     /* No MC3610 detected ... return false */
     	Serial.println(id, HEX);
     	return false;
    }
   	SetRangeCtrl(MC3610_RANGE_8G); //Range: 8g
   	SetResolutionCtrl(MC3610_RESOLUTION_14BIT); //Resolution: 14bit
    SetCWakeSampleRate(MC3610_CWAKE_SR_DEFAULT_50Hz); //Sampling Rate: 50Hz
    SetMode(MC3610_MODE_CWAKE); //Mode: Active
    return true;
}

void MC3610::stop()
{
  	SetMode(MC3610_MODE_SLEEP); //Mode: Sleep
}

//Read the raw counts and SI units mearsurement data
mc3610_acc_t MC3610::readRawAccel(void)
{
	float faRange[5] = { 19.614f, 39.228f, 78.456f, 156.912f, 117.684f}; //{2g, 4g, 8g, 16g, 12g}
	float faResolution[6] = { 32.0f, 64.0f, 128.0f, 512.0f, 2048.0f, 8192.0f}; //{6bit, 7bit, 8bit, 10bit, 12bit, 14bit}

	byte rawData[6];
	readRegisters(MC3610_REG_XOUT_LSB, rawData, 6);  // Read the six raw data registers into data array
	x = (short)((((unsigned short)rawData[1]) << 8) | rawData[0]);
    y = (short)((((unsigned short)rawData[3]) << 8) | rawData[2]);
    z = (short)((((unsigned short)rawData[5]) << 8) | rawData[4]);

	AccRaw.XAxis = (short) (x);
	AccRaw.YAxis = (short) (y);
	AccRaw.ZAxis = (short) (z);
    AccRaw.XAxis_g = (float) (x) / faResolution[CfgResolution]*faRange[CfgRange];
	AccRaw.YAxis_g = (float) (y) / faResolution[CfgResolution]*faRange[CfgRange];
	AccRaw.ZAxis_g = (float) (z) / faResolution[CfgResolution]*faRange[CfgRange];

	return AccRaw;
}