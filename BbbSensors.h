#pragma once

#define I2C_ADDRESS_MPU9250			0x68	// MPU9250 address when ADO = 1
#define I2C_ADDRESS_AK8963			0x0C    // Address of AK8963 (MPU9250) magnetometer
#define MPU9250_ID_REG_ADDR			0x75
#define MPU9250_ID_REG_VAL 			0x71
#define AK8963_ID_REG_ADDR			0x00
#define AK8963_ID_REG_VAL			0x48

#define MPU9250_ACCEL_XOUT_H 		0x3B
//#define MPU9250_ACCEL_XOUT_L		0x3C
//#define MPU9250_ACCEL_YOUT_H 		0x3D
//#define MPU9250_ACCEL_YOUT_L 		0x3E
//#define MPU9250_ACCEL_ZOUT_H 		0x3F
//#define MPU9250_ACCEL_ZOUT_L 		0x40
#define MPU9250_TEMP_OUT_H			0x41
//#define MPU9250_TEMP_OUT_L		0x42
#define MPU9250_GYRO_XOUT_H			0x43
//#define MPU9250_GYRO_XOUT_L		0x44
//#define MPU9250_GYRO_YOUT_H		0x45
//#define MPU9250_GYRO_YOUT_L		0x46
//#define MPU9250_GYRO_ZOUT_H		0x47
//#define MPU9250_GYRO_ZOUT_L		0x48
#define MPU9250_PWR_MGMT_1			0x6B
#define MPU9250_SMPLRT_DIV       0x19
#define MPU9250_CONFIG           0x1A
#define MPU9250_GYRO_CONFIG      0x1B
#define MPU9250_ACCEL_CONFIG     0x1C
#define MPU9250_ACCEL_CONFIG2    0x1D
#define MPU9250_INT_PIN_CFG      0x37

//#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L	 0x03  // data
//#define AK8963_XOUT_H	 0x04
//#define AK8963_YOUT_L	 0x05
//#define AK8963_YOUT_H	 0x06
//#define AK8963_ZOUT_L	 0x07
//#define AK8963_ZOUT_H	 0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
//#define AK8963_ASTC      0x0C  // Self test control
//#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
//#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
//#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

// BMP280 registers
#define I2C_ADDRESS_BMP280			0x76	// Address of BMP280 altimeter
#define BMP280_ID_REG_ADDR			0xD0
#define BMP280_ID_REG_VAL			0x58
//#define BMP280_TEMP_XLSB 			0xFC
//#define BMP280_TEMP_LSB			0xFB
#define BMP280_TEMP_MSB				0xFA
//#define BMP280_PRESS_XLSB			0xF9
//#define BMP280_PRESS_LSB			0xF8
#define BMP280_PRESS_MSB			0xF7
#define BMP280_CONFIG			0xF5
#define BMP280_CTRL_MEAS		0xF4
//#define BMP280_STATUS			0xF3
//#define BMP280_RESET			0xE0
#define BMP280_CALIB00			0x88


class BbbSensors
{
public: 
	BbbSensors();

	bool InitSensors();

	double getPressureHpa();
	double getAltitude();
	double getTemp1();  // get temperature from BMP280
	double getHeading();

	bool getMagneto(double* magneto);
	bool getHeading(double* heading);
	bool getGyro(double* gyro);
	bool getAccel(double* accel);
	bool getAngle(double* angle);
	double getTemp2(); // get temperature from MPU9250


private:
	class BMP280Calib* BMPCal;
	class MPU9250Calib* MPU9250Cal;
	int file;   // The I2C handler

	bool InitMPU9250();
	bool InitBMP280();
	bool CheckSensorID(char addrToCheck, char regToCheck, char dataToCheck);

	bool getXYZWordLitEndian(char address, char reg, int16_t* data);
	bool getXYZWordBigEndian(char address, char reg, int16_t* data);

	int ReadChars(char I2cAddress, char regAddress, char* data, int nbrChars);
	int WriteChars(char I2cAddress, char regAddress, char* data, int nbrChars);
};

class BMP280Calib
{
public:
	BMP280Calib(void);
	BMP280Calib(char* calib);
	int dig_T1, dig_T2, dig_T3;
	int dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
	double t_fine = 20.0;
};

class MPU9250Calib
{
public:
	MPU9250Calib(void);
	MPU9250Calib(char* calib);
 
	//const double MPU9250aRes = 2.0 / 32768.0;
	//const double MPU9250aRes = 4.0 / 32768.0;
	//const double MPU9250aRes = 8.0 / 32768.0;
	const double MPU9250aRes = 16.0 / 32768.0;

	const double MPU9250gRes = 250.0 / 32768.0;
	//const double MPU9250gRes = 500.0 / 32768.0;
	//const double MPU9250gRes = 1000.0 / 32768.0;
	//const double MPU9250gRes = 2000.0 / 32768.0;

	// Proper scale to return milliGauss
	//MPU9250mRes = 10. * 4912. / 8190.; // 14bit
	const double MPU9250mRes = 10.0 * 4912.0 / 32760.0; // 16bit


	//const uint16_t  gyrosensitivity = 131;   // = 131 LSB/degrees/sec
	//uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

	double magCalibration[3] = { 0, 0, 0 };  // Factory mag calibration

	double MPU9250gyroBias[3] = { 0, 0, 0 };	// Bias corrections for gyro, acc and mag
	double MPU9250accelBias[3] = { 0, 0, 0 };
	double MPU9250magBias[3] = { 0, 0, 0 };      
};
