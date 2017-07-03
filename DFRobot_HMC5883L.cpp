/*!
 * @file DFRobot_HMC5883L.cpp
 * @brief Compatible with HMC5883L and QMC5883
 * @n 3-Axis Digital Compass IC
 *
 * @copyright	[DFRobot](http://www.dfrobot.com), 2017
 * @copyright	GNU Lesser General Public License
 *
 * @author [dexian.huang](952838602@qq.com)
 * @version  V1.0
 * @date  2017-7-3
 */

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

#include "DFRobot_HMC5883L.h"



bool DFRobot_HMC5883L::begin()
{
	delay(5000);
    Wire.begin();	
	Wire.beginTransmission(HMC5883L_ADDRESS);
    flagHmc_ = Wire.endTransmission();
	Serial.print("flagHmc_= ");
	Serial.println(flagHmc_);
	
	if(!flagHmc_){
    if ((fastRegister8(HMC5883L_REG_IDENT_A) != 0x48)
    || (fastRegister8(HMC5883L_REG_IDENT_B) != 0x34)
    || (fastRegister8(HMC5883L_REG_IDENT_C) != 0x33)){
	  return false;
    }

    setRange(HMC5883L_RANGE_1_3GA);
    setMeasurementMode(HMC5883L_CONTINOUS);
    setDataRate(HMC5883L_DATARATE_15HZ);
    setSamples(HMC5883L_SAMPLES_1);
    mgPerDigit = 0.92f;
    return true;
	}else if(flagHmc_){
	  	  
	Wire.begin();
	Wire.beginTransmission(QMC5883L_ADDRESS);
    flagQmc_ = Wire.endTransmission(); 
	Serial.print("flagQmc_= ");
	Serial.println(flagQmc_);
    if(!flagQmc_){
	if ((fastRegister8(HMC5883L_REG_IDENT_B_Q) != 0x00)
    || (fastRegister8(HMC5883L_REG_IDENT_C_Q) != 0x01)
    || (fastRegister8(HMC5883L_REG_IDENT_D_Q) != 0xFF)){
	  return false;
    }
	
    setRange(HMC5883L_RANGE_8GA);
    setMeasurementMode(HMC5883L_CONTINOUS_Q);
    setDataRate(HMC5883L_DATARATE_50HZ);
    setSamples(HMC5883L_SAMPLES_8);
	mgPerDigit = 4.25f;

    return true;
	}
  }
}

Vector DFRobot_HMC5883L::readRaw(void)
{
	if(!flagHmc_){
    v.XAxis = readRegister16(HMC5883L_REG_OUT_X_M) - xOffset;
    v.YAxis = readRegister16(HMC5883L_REG_OUT_Y_M) - yOffset;
    v.ZAxis = readRegister16(HMC5883L_REG_OUT_Z_M);

    return v;
    }else if(!flagQmc_){
	v.XAxis = readRegister16(HMC5883L_REG_OUT_X_M_Q) - xOffset;
    v.YAxis = readRegister16(HMC5883L_REG_OUT_Y_M_Q) - yOffset;
    v.ZAxis = readRegister16(HMC5883L_REG_OUT_Z_M_Q);

    return v;
  }
	
}

Vector DFRobot_HMC5883L::readNormalize(void)
{
	if(!flagHmc_){	
    v.XAxis = ((float)readRegister16(HMC5883L_REG_OUT_X_M) - xOffset) * mgPerDigit;
    v.YAxis = ((float)readRegister16(HMC5883L_REG_OUT_Y_M) - yOffset) * mgPerDigit;
    v.ZAxis = (float)readRegister16(HMC5883L_REG_OUT_Z_M) * mgPerDigit;
    return v;
  }else if(!flagQmc_){
	  v.XAxis = ((float)readRegister16(HMC5883L_REG_OUT_X_M_Q) - xOffset) * mgPerDigit;
    v.YAxis = ((float)readRegister16(HMC5883L_REG_OUT_Y_M_Q) - yOffset) * mgPerDigit;
    v.ZAxis = (float)readRegister16(HMC5883L_REG_OUT_Z_M_Q) * mgPerDigit;
    return v;
	}
	
}

void DFRobot_HMC5883L::setOffset(int xo, int yo)
{
    xOffset = xo;
    yOffset = yo;
}

void DFRobot_HMC5883L::setRange(hmc5883l_range_t range)
{
  if(!flagHmc_){
    switch(range){
    case HMC5883L_RANGE_0_88GA:
      mgPerDigit = 0.073f;
      break;

    case HMC5883L_RANGE_1_3GA:
	    mgPerDigit = 0.92f;
	    break;

    case HMC5883L_RANGE_1_9GA:
	    mgPerDigit = 1.22f;
      break;

    case HMC5883L_RANGE_2_5GA:
      mgPerDigit = 1.52f;
      break;

    case HMC5883L_RANGE_4GA:
      mgPerDigit = 2.27f;
	    break;

    case HMC5883L_RANGE_4_7GA:
	    mgPerDigit = 2.56f;
	    break;

    case HMC5883L_RANGE_5_6GA:
	    mgPerDigit = 3.03f;
	    break;

    case HMC5883L_RANGE_8_1GA:
	    mgPerDigit = 4.35f;
	    break;

    default:
	    break;
    }

    writeRegister8(HMC5883L_REG_CONFIG_B, range << 5);
	
    }else if(!flagQmc_){
    switch(range)
    {
    case HMC5883L_RANGE_2GA:
	    mgPerDigit = 1.22f;
	    break;
    case HMC5883L_RANGE_8GA:
	    mgPerDigit = 4.35f;
	    break;

    default:
	    break;
    }

    writeRegister8(HMC5883L_REG_CONFIG_2_Q, range << 4);
	}
}

hmc5883l_range_t DFRobot_HMC5883L::getRange(void)
{
    if(!flagHmc_){
	
    return (hmc5883l_range_t)((readRegister8(HMC5883L_REG_CONFIG_B) >> 5));
    }else if(!flagQmc_){
	return (hmc5883l_range_t)((readRegister8(HMC5883L_REG_CONFIG_2_Q) >> 3));
}
}

void DFRobot_HMC5883L::setMeasurementMode(hmc5883l_mode_t mode)
{
if(!flagHmc_){
    uint8_t value;

    value = readRegister8(HMC5883L_REG_MODE);
    value &= 0b11111100;
    value |= mode;

    writeRegister8(HMC5883L_REG_MODE, value);
}else if(!flagQmc_){
uint8_t value;

    value = readRegister8(HMC5883L_REG_CONFIG_1_Q);
    value &= 0xfc;
    value |= mode;

    writeRegister8(HMC5883L_REG_CONFIG_1_Q, value);
	}	
}

hmc5883l_mode_t DFRobot_HMC5883L::getMeasurementMode(void)
{
	uint8_t value;

    value = readRegister8(HMC5883L_REG_MODE);
    value &= 0b00000011;

    return (hmc5883l_mode_t)value;	
}

void DFRobot_HMC5883L::setDataRate(hmc5883l_dataRate_t dataRate)
{
    if(!flagHmc_){
      uint8_t value;

      value = readRegister8(HMC5883L_REG_CONFIG_A);
      value &= 0b11100011;
      value |= (dataRate << 2);

      writeRegister8(HMC5883L_REG_CONFIG_A, value);
    }else if(!flagQmc_){
      uint8_t value;

      value = readRegister8(HMC5883L_REG_CONFIG_1_Q);
      value &= 0xf3;
      value |= (dataRate << 2);

      writeRegister8(HMC5883L_REG_CONFIG_1_Q, value);
	}
}

hmc5883l_dataRate_t DFRobot_HMC5883L::getDataRate(void)
{
    uint8_t value;

    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b00011100;
    value >>= 2;
    return (hmc5883l_dataRate_t)value;
}
void DFRobot_HMC5883L::setSamples(hmc5883l_samples_t samples)
{
  if(!flagHmc_){
    uint8_t value;
    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b10011111;
    value |= (samples << 5);
    writeRegister8(HMC5883L_REG_CONFIG_A, value);
  }else if(!flagQmc_){
    uint8_t value;
    value = readRegister8(HMC5883L_REG_CONFIG_1_Q);
    value &= 0x3f;
    value |= (samples << 7);
    writeRegister8(HMC5883L_REG_CONFIG_1_Q, value);
  }
}
hmc5883l_samples_t DFRobot_HMC5883L::getSamples(void)
{
    uint8_t value;
    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b01100000;
    value >>= 5;
    return (hmc5883l_samples_t)value;
}
// Write byte to register
void DFRobot_HMC5883L::writeRegister8(uint8_t reg, uint8_t value)
{
  if(!flagHmc_){
    Wire.beginTransmission(HMC5883L_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
        Wire.write(value);
    #else
        Wire.send(reg);
        Wire.send(value);
    #endif
    Wire.endTransmission();
  }else if(!flagQmc_){
    Wire.beginTransmission(QMC5883L_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
        Wire.write(value);
    #else
        Wire.send(reg);
        Wire.send(value);
    #endif
    Wire.endTransmission();
	}
}
// Read byte to register
uint8_t DFRobot_HMC5883L::fastRegister8(uint8_t reg)
{
	if(!flagHmc_){
    uint8_t value;
    Wire.beginTransmission(HMC5883L_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();

    Wire.requestFrom(HMC5883L_ADDRESS, 1);
    #if ARDUINO >= 100
        value = Wire.read();
    #else
        value = Wire.receive();
    #endif
    Wire.endTransmission();

    return value;
	}else if(!flagQmc_){
		 uint8_t value;
    Wire.beginTransmission(QMC5883L_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();
    Wire.requestFrom(QMC5883L_ADDRESS, 1);
    #if ARDUINO >= 100
        value = Wire.read();
    #else
        value = Wire.receive();
    #endif
    Wire.endTransmission();
    return value;
	}
}

// Read byte from register
uint8_t DFRobot_HMC5883L::readRegister8(uint8_t reg)
{
	if(!flagHmc_){
    uint8_t value;
    Wire.beginTransmission(HMC5883L_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();

    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.requestFrom(HMC5883L_ADDRESS, 1);
    while(!Wire.available()) {};
    #if ARDUINO >= 100
        value = Wire.read();
    #else
        value = Wire.receive();
    #endif
    Wire.endTransmission();

    return value;
	}else if(!flagQmc_){
    uint8_t value;
    Wire.beginTransmission(QMC5883L_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();
    Wire.beginTransmission(QMC5883L_ADDRESS);
    Wire.requestFrom(QMC5883L_ADDRESS, 1);
    while(!Wire.available()) {};
    #if ARDUINO >= 100
        value = Wire.read();
    #else
        value = Wire.receive();
    #endif
    Wire.endTransmission();

    return value;
	}
}
// Read word from register
int16_t DFRobot_HMC5883L::readRegister16(uint8_t reg)
{
	if(!flagHmc_){	
    int16_t value;
    Wire.beginTransmission(HMC5883L_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.requestFrom(HMC5883L_ADDRESS, 2);
    while(!Wire.available()) {};
    #if ARDUINO >= 100
        uint8_t vha = Wire.read();
        uint8_t vla = Wire.read();
    #else
        uint8_t vha = Wire.receive();
        uint8_t vla = Wire.receive();
    #endif
    Wire.endTransmission();
    value = vha << 8 | vla;
    return value;
	}else if(!flagQmc_){
		  int16_t value;
    Wire.beginTransmission(QMC5883L_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();
    Wire.beginTransmission(QMC5883L_ADDRESS);
    Wire.requestFrom(QMC5883L_ADDRESS, 2);
    while(!Wire.available()) {};
    #if ARDUINO >= 100
        uint8_t vha = Wire.read();
        uint8_t vla = Wire.read();
    #else
        uint8_t vha = Wire.receive();
        uint8_t vla = Wire.receive();
    #endif
    Wire.endTransmission();
    value = vha << 8 | vla;
    return value;
	}
}

void DFRobot_HMC5883L::calibrate(int* offX, int* offY)
{
  static int minX = 0;
  static int maxX = 0;
  static int minY = 0;
  static int maxY = 0;
  Vector mag = readRaw();

  if (mag.XAxis < minX) minX = mag.XAxis;
  if (mag.XAxis > maxX) maxX = mag.XAxis;
  if (mag.YAxis < minY) minY = mag.YAxis;
  if (mag.YAxis > maxY) maxY = mag.YAxis;  
  
  *offX = (maxX + minX)/2;
  *offY = (maxY + minY)/2;  
  // delay(10000);
}
