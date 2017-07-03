/*!
 * @file DFRobot_HMC5883L.h
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

#ifndef DFROBOT_HMC5883L_H
#define DFROBOT_HMC5883L_H

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define HMC5883L_ADDRESS              (0x1E)
#define QMC5883L_ADDRESS              (0x0D)



#define HMC5883L_REG_CONFIG_A         (0x00)
#define HMC5883L_REG_CONFIG_B         (0x01)
#define HMC5883L_REG_MODE             (0x02)
#define HMC5883L_REG_OUT_X_M          (0x03)
#define HMC5883L_REG_OUT_X_L          (0x04)
#define HMC5883L_REG_OUT_Z_M          (0x05)
#define HMC5883L_REG_OUT_Z_L          (0x06)
#define HMC5883L_REG_OUT_Y_M          (0x07)
#define HMC5883L_REG_OUT_Y_L          (0x08)
#define HMC5883L_REG_STATUS           (0x09)
#define HMC5883L_REG_IDENT_A          (0x0A)
#define HMC5883L_REG_IDENT_B          (0x0B)
#define HMC5883L_REG_IDENT_C          (0x0C)


#define HMC5883L_REG_OUT_X_M_Q          (0x01)
#define HMC5883L_REG_OUT_X_L_Q          (0x00)
#define HMC5883L_REG_OUT_Z_M_Q          (0x05)
#define HMC5883L_REG_OUT_Z_L_Q          (0x04)
#define HMC5883L_REG_OUT_Y_M_Q          (0x03)
#define HMC5883L_REG_OUT_Y_L_Q          (0x02)
#define HMC5883L_REG_STATUS_Q           (0x06)
#define HMC5883L_REG_CONFIG_1_Q         (0x09)
#define HMC5883L_REG_CONFIG_2_Q         (0x0A)


#define HMC5883L_REG_IDENT_B_Q          (0x0B)

#define HMC5883L_REG_IDENT_C_Q          (0x0C)


#define HMC5883L_REG_IDENT_D_Q          (0x0D)


typedef enum
{
    HMC5883L_SAMPLES_8     = 0b11,
    HMC5883L_SAMPLES_4     = 0b10,
    HMC5883L_SAMPLES_2     = 0b01,
    HMC5883L_SAMPLES_1     = 0b00
} hmc5883l_samples_t;

typedef enum
{
    HMC5883L_DATARATE_75HZ       = 0b110,
    HMC5883L_DATARATE_30HZ       = 0b101,
    HMC5883L_DATARATE_15HZ       = 0b100,
    HMC5883L_DATARATE_7_5HZ      = 0b011,
    HMC5883L_DATARATE_3HZ        = 0b010,
    HMC5883L_DATARATE_1_5HZ      = 0b001,
    HMC5883L_DATARATE_0_75_HZ    = 0b000,
	HMC5883L_DATARATE_10HZ       = 0b00,
    HMC5883L_DATARATE_50HZ       = 0b01,
    HMC5883L_DATARATE_100HZ      = 0b10,
    HMC5883L_DATARATE_200HZ      = 0b11
} hmc5883l_dataRate_t;

typedef enum
{
    HMC5883L_RANGE_8_1GA     = 0b111,
    HMC5883L_RANGE_5_6GA     = 0b110,
    HMC5883L_RANGE_4_7GA     = 0b101,
    HMC5883L_RANGE_4GA       = 0b100,
    HMC5883L_RANGE_2_5GA     = 0b011,
    HMC5883L_RANGE_1_9GA     = 0b010,
    HMC5883L_RANGE_1_3GA     = 0b001,
    HMC5883L_RANGE_0_88GA    = 0b000,
	HMC5883L_RANGE_2GA     = 0b00,
    HMC5883L_RANGE_8GA     = 0b01 
} hmc5883l_range_t;

typedef enum
{
    HMC5883L_IDLE          = 0b10,
    HMC5883L_SINGLE        = 0b01,
    HMC5883L_CONTINOUS     = 0b00,
	HMC5883L_SINGLE_Q      = 0b00,		
    HMC5883L_CONTINOUS_Q   = 0b01		
} hmc5883l_mode_t;

#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H
struct Vector
{
    float XAxis;
    float YAxis;
    float ZAxis;
};
#endif

class DFRobot_HMC5883L
{
public:
	DFRobot_HMC5883L():flagHmc_(1),flagQmc_(1)
		{}
	bool begin(void);

	Vector readRaw(void);
	Vector readNormalize(void);
	void calibrate(int* offX, int* offY);

	void  setOffset(int xo, int yo);

	void  setRange(hmc5883l_range_t range);
	hmc5883l_range_t getRange(void);

	void  setMeasurementMode(hmc5883l_mode_t mode);
	hmc5883l_mode_t getMeasurementMode(void);

	void  setDataRate(hmc5883l_dataRate_t dataRate);
	hmc5883l_dataRate_t getDataRate(void);

	void  setSamples(hmc5883l_samples_t samples);
	hmc5883l_samples_t getSamples(void);

private:

	uint8_t flagHmc_;
	uint8_t flagQmc_;
	
	float mgPerDigit;
	Vector v;
	int xOffset, yOffset;

	void writeRegister8(uint8_t reg, uint8_t value);
	uint8_t readRegister8(uint8_t reg);
	uint8_t fastRegister8(uint8_t reg);
	int16_t readRegister16(uint8_t reg);
};

#endif