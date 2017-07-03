HMC5883L - 3-Axis Digital Compass IC
---------------------------------------------------------

### initial HMC5883L sensor
扫描设备并设置参数
    bool begin(void);


### read raw data

You can read raw data by this function:

    Vector readRaw(void);


### read Normalize data

You can read raw data by this function:
    Vector readNormalize(void);

### calibrate

You can calibrate hmc5883l sensor by this function:
    void calibrate(int* offX, int* offY);


### set offset by calibrate data

    void  setOffset(int xo, int yo);


* @n [Get the module here](等上架后添加商品购买链接)
* @n This example Set the volume size and play music snippet.
* @n [Connection and Diagram](等上架后添加wiki链接)
*
* Copyright	[DFRobot](http://www.dfrobot.com), 2016
* Copyright	GNU Lesser General Public License
*
* @author [dexian.huang](952838602@qq.com)
* version  V1.0
* date  2017-7-3
