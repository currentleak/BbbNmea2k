#include "BbbSensors.h"

using namespace std;

BbbSensors::BbbSensors()
{
    const char* filename = "/dev/i2c-2";
    if ((file = open(filename, O_RDWR)) < 0)
    {
        perror("Failed to open the i2c bus\n");
    }
}

BMP280Calib::BMP280Calib()
{
}
BMP280Calib::BMP280Calib(char* calib)
{
    dig_T1 = (uint16_t)(((uint16_t)calib[1] << 8) | calib[0]);
    dig_T2 = (int16_t)(((int16_t)calib[3] << 8) | calib[2]);
    dig_T3 = (int16_t)(((int16_t)calib[5] << 8) | calib[4]);
    dig_P1 = (uint16_t)(((uint16_t)calib[7] << 8) | calib[6]);
    dig_P2 = (int16_t)(((int16_t)calib[9] << 8) | calib[8]);
    dig_P3 = (int16_t)(((int16_t)calib[11] << 8) | calib[10]);
    dig_P4 = (int16_t)(((int16_t)calib[13] << 8) | calib[12]);
    dig_P5 = (int16_t)(((int16_t)calib[15] << 8) | calib[14]);
    dig_P6 = (int16_t)(((int16_t)calib[17] << 8) | calib[16]);
    dig_P7 = (int16_t)(((int16_t)calib[19] << 8) | calib[18]);
    dig_P8 = (int16_t)(((int16_t)calib[21] << 8) | calib[20]);
    dig_P9 = (int16_t)(((int16_t)calib[23] << 8) | calib[22]);
}

MPU9250Calib::MPU9250Calib()
{
    //MPU9250aRes = 16.0 / 32768.0;
}
MPU9250Calib::MPU9250Calib(char* calib)
{
    //MPU9250aRes = 16.0 / 32768.0;
    // todo
}

bool BbbSensors::InitSensors()
{
    return InitMPU9250() && InitBMP280();
}

bool BbbSensors::InitBMP280()
{
    if (!CheckSensorID(I2C_ADDRESS_BMP280, BMP280_ID_REG_ADDR, BMP280_ID_REG_VAL))
    {
        return false;
    }
    char calibData[24];
    // Configure the BMP280
    calibData[0] = 0x93; // 1001 0011 : 8x ovesampling, normal mode; 0x90 for sleep mode
    WriteChars(I2C_ADDRESS_BMP280, BMP280_CTRL_MEAS, calibData, 1);
    calibData[0] = 0x50; // t_sb=125ms, filter=, spi=disable : 010 100 00 = 0x50
    WriteChars(I2C_ADDRESS_BMP280, BMP280_CONFIG, calibData, 1);
    calibData[0] = 0;
    // Calibration
    ReadChars(I2C_ADDRESS_BMP280, BMP280_CALIB00, calibData, 24);
    BMPCal = new BMP280Calib(calibData);

    return true;
}

bool BbbSensors::InitMPU9250()
{
    char calibData[8];

    // Check MPU9250 Sensor
    if (!CheckSensorID(I2C_ADDRESS_MPU9250, MPU9250_ID_REG_ADDR, MPU9250_ID_REG_VAL))
    {
        return false;
    }
    // wake up device
    calibData[0] = 0x80;  // reset and restore default setting
    WriteChars(I2C_ADDRESS_MPU9250, MPU9250_PWR_MGMT_1, calibData, 1);
    usleep(100000);
    calibData[0] = 0x00;  // use internal clock;   or 0x01 for best clock available 
    WriteChars(I2C_ADDRESS_MPU9250, MPU9250_PWR_MGMT_1, calibData, 1);
    usleep(200000);
    calibData[0] = 0x03;
    WriteChars(I2C_ADDRESS_MPU9250, MPU9250_CONFIG, calibData, 1);
    calibData[0] = 0x04;
    WriteChars(I2C_ADDRESS_MPU9250, MPU9250_SMPLRT_DIV, calibData, 1);

    // Set gyroscope low scale range
    calibData[0] = 0x00; // 0000 0000 : +250dps
    WriteChars(I2C_ADDRESS_MPU9250, MPU9250_GYRO_CONFIG, &calibData[0], 1); // Set full scale range for the gyro

    // Set accelerometer full-scale range configuration
    calibData[0] = 0x18; // 0001 1000 : +/-16g
    WriteChars(I2C_ADDRESS_MPU9250, MPU9250_ACCEL_CONFIG, &calibData[0], 1); // Set full scale range for the accelerometer 
    calibData[0] = 0x0B;// Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    WriteChars(I2C_ADDRESS_MPU9250, MPU9250_ACCEL_CONFIG2, &calibData[0], 1);

    MPU9250Cal = new MPU9250Calib();

    // Check AK8963 sensor, it is integrated in the MPU9250 IC
    calibData[0] = 0x02; // enable MPU9250 bypass mode
    WriteChars(I2C_ADDRESS_MPU9250, MPU9250_INT_PIN_CFG, calibData, 1);
    if (!CheckSensorID(I2C_ADDRESS_AK8963, AK8963_ID_REG_ADDR, AK8963_ID_REG_VAL))
    {
        return false;
    }
    calibData[0] = 0; // Power down magnetometer 
    WriteChars(I2C_ADDRESS_AK8963, AK8963_CNTL, calibData, 1);
    usleep(10000);
    calibData[0] = 0x0F; // Enter Fuse ROM access mode
    WriteChars(I2C_ADDRESS_AK8963, AK8963_CNTL, calibData, 1);
    usleep(10000);
    ReadChars(I2C_ADDRESS_AK8963, AK8963_ASAX, calibData, 3); // Read the x-, y-, and z-axis calibration values
 
    MPU9250Cal->magCalibration[0] = (double)(calibData[0] - 128) / 256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
    MPU9250Cal->magCalibration[1] = (double)(calibData[1] - 128) / 256. + 1.;
    MPU9250Cal->magCalibration[2] = (double)(calibData[2] - 128) / 256. + 1.;

    calibData[0] = 0; // Power down magnetometer 
    WriteChars(I2C_ADDRESS_AK8963, AK8963_CNTL, calibData, 1);
    usleep(10000);

    // Configure the magnetometer for continuous read and highest resolution, set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    calibData[0] = 0x12; // 16bit, continous mode 1 = 8Hz
    WriteChars(I2C_ADDRESS_AK8963, AK8963_CNTL, calibData, 1); // Set magnetometer data resolution and sample ODR
    usleep(10000);

    return true;
}

bool BbbSensors::CheckSensorID(char addrToCheck, char regToCheck, char dataToCheck)
{
    char buf[1] = { 0 };
    ReadChars(addrToCheck, regToCheck, buf, 1);
    if (buf[0] == dataToCheck)  // ID register value
    {
        return true;
    }
    else
    {
        perror("Wrong ID value.\n");
        return false;
    }
}

// Return PRESSURE in deg hPa, from the BMP280
double BbbSensors::getPressureHpa()
{
    char data[3] = { 0, 0, 0 };
    double pressure = 0.0;
    int pressure32 = 0;

    ReadChars(I2C_ADDRESS_BMP280, BMP280_PRESS_MSB, data, 3);

    pressure32 = (int)(((int)data[0] << 16 | (int)data[1] << 8 | data[2]) >> 4);

    long long var1, var2, p;
    var1 = ((long long)BMPCal->t_fine) - 128000;
    var2 = var1 * var1 * (long long)BMPCal->dig_P6;
    var2 = var2 + ((var1 * (long long)BMPCal->dig_P5) << 17);
    var2 = var2 + (((long long)BMPCal->dig_P4) << 35);
    var1 = ((var1 * var1 * (long long)BMPCal->dig_P3) >> 8) + ((var1 * (long long)BMPCal->dig_P2) << 12);
    var1 = (((((long long)1) << 47) + var1)) * ((long long)BMPCal->dig_P1) >> 33;
    if (var1 == 0)
    {
        return 0.0;     // avoid exception caused by division by zero
    }
    p = 1048576 - pressure32;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((long long)BMPCal->dig_P9) * (p >> 13)* (p >> 13)) >> 25;
    var2 = (((long long)BMPCal->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((long long)BMPCal->dig_P7) << 4);

    pressure = (double)p / 25600.0;
    return pressure;
}
// Calculate Altitude
double BbbSensors::getAltitude()
{
    double altitude = 145366.45f * (1.0f - pow((getPressureHpa() / 1013.25f), 0.190284f));
    return altitude;
}

// Return TEMPERATURE in deg C, from the BMP280
double BbbSensors::getTemp1()
{
    char data[3] = { 0, 0, 0 };
    double temperature = 0.0;
    int temp32 = 0;

    ReadChars(I2C_ADDRESS_BMP280, BMP280_TEMP_MSB, data, 3);
    temp32 = (int)(((int)data[0] << 16 | (int)data[1] << 8 | data[2]) >> 4);
    //temp32 = ((uint32_t)data[0] * 65535 + (uint32_t)data[1] * 256 + (uint32_t)data[0])/16;
   
    int var1, var2; 
    var1 = ((((temp32 >> 3) - (BMPCal->dig_T1 << 1)))* (BMPCal->dig_T2)) >> 11;
    var2 = (((((temp32 >> 4) - (BMPCal->dig_T1))* ((temp32 >> 4) - (BMPCal->dig_T1))) >> 12)* (BMPCal->dig_T3)) >> 14;
    temp32 = var1 + var2;
    temp32 = (temp32 * 5 + 128) >> 8;

    temperature = (double)temp32 * 0.01;
    BMPCal->t_fine = temperature;
    return temperature;
}

// Return MAG field
bool BbbSensors::getMagneto(double* magneto)
{
    int16_t magnetoWord[3] = { 0, 0, 0 };
    char c = { 0 };
    //ReadChars(I2C_ADDRESS_AK8963, AK8963_ST1, data, 1); // wait for magnetometer data ready bit to be set
    //if (data[0] & 0x01)
    //{
        ReadChars(I2C_ADDRESS_AK8963, AK8963_ST2, &c, 1);
        if (!(c & 0x08)) // Check if magnetic sensor overflow set, if not then report data
        {
            getXYZWordLitEndian(I2C_ADDRESS_AK8963, AK8963_XOUT_L, magnetoWord);
            //magneto[0] = ((int16_t)data[1] << 8) | data[0];  // Turn the MSB and LSB into a signed 16-bit value
            //magneto[1] = ((int16_t)data[3] << 8) | data[2];  // Data stored as little Endian
            //magneto[2] = ((int16_t)data[5] << 8) | data[4];
        }
    //}

    
    //getMagneto(magnetoWord);
    magneto[0] = (double)magnetoWord[0] * MPU9250Cal->MPU9250mRes * MPU9250Cal->magCalibration[0] - MPU9250Cal->MPU9250magBias[0];
    magneto[1] = (double)magnetoWord[1] * MPU9250Cal->MPU9250mRes * MPU9250Cal->magCalibration[1] - MPU9250Cal->MPU9250magBias[1];
    magneto[2] = (double)magnetoWord[2] * MPU9250Cal->MPU9250mRes * MPU9250Cal->magCalibration[2] - MPU9250Cal->MPU9250magBias[2];
    return true;
}
// Calculate Heading
double BbbSensors::getHeading()
{
    double heading;
    double magneto[3] = { 0, 0, 0 };
    getMagneto(magneto);
    heading = atan2(magneto[1], magneto[0]) * 180 / M_PI;
        
    return heading; // in deg
}
bool BbbSensors::getHeading(double* heading)
{
    double magneto[3] = { 0, 0, 0 };
    getMagneto(magneto);
    double xAngle = atan2(magneto[0], sqrt(pow(magneto[1], 2) + pow(magneto[2], 2)));
    double yAngle = atan2(magneto[1], sqrt(pow(magneto[0], 2) + pow(magneto[2], 2)));
    double zAngle = atan2(magneto[2], sqrt(pow(magneto[0], 2) + pow(magneto[1], 2)));

    xAngle = xAngle * 180 / M_PI;  // convert radian to deg
    yAngle = yAngle * 180 / M_PI;
    zAngle = zAngle * 180 / M_PI;
    heading[0] = (double)xAngle;
    heading[1] = (double)yAngle;
    heading[2] = (double)zAngle;
    return true; // in deg
}

// Return GYRO
bool BbbSensors::getGyro(double* gyro)
{
    int16_t gyroWord[3] = { 0, 0, 0 };
    getXYZWordBigEndian(I2C_ADDRESS_MPU9250, MPU9250_GYRO_XOUT_H, gyroWord);
    gyro[0] = (double)gyroWord[0] * MPU9250Cal->MPU9250gRes - MPU9250Cal->MPU9250gyroBias[0];
    gyro[1] = (double)gyroWord[1] * MPU9250Cal->MPU9250gRes - MPU9250Cal->MPU9250gyroBias[1];
    gyro[2] = (double)gyroWord[2] * MPU9250Cal->MPU9250gRes - MPU9250Cal->MPU9250gyroBias[2];
    return true;
}

// Return ACCEL
bool BbbSensors::getAccel(double* accel)
{
    int16_t accelWord[3] = { 0, 0, 0 };
    getXYZWordBigEndian(I2C_ADDRESS_MPU9250, MPU9250_ACCEL_XOUT_H, accelWord); 
    // Now we'll calculate the accleration value into actual g's, this depends on scale being set
    accel[0] = (double)accelWord[0] * MPU9250Cal->MPU9250aRes - MPU9250Cal->MPU9250accelBias[0];  
    accel[1] = (double)accelWord[1] * MPU9250Cal->MPU9250aRes - MPU9250Cal->MPU9250accelBias[1];
    accel[2] = (double)accelWord[2] * MPU9250Cal->MPU9250aRes - MPU9250Cal->MPU9250accelBias[2];
    return true;
}
// Calculate Angle
bool BbbSensors::getAngle(double* angle)
{
    double acceleration[3] = { 0, 0, 0 }; // acceleratiopn X, Y, Z
    getAccel(acceleration);
    double xAngle = atan2(acceleration[0], sqrt(pow(acceleration[1], 2) + pow(acceleration[2], 2)));
    double yAngle = atan2(acceleration[1], sqrt(pow(acceleration[0], 2) + pow(acceleration[2], 2)));
    double zAngle = atan2(acceleration[2], sqrt(pow(acceleration[0], 2) + pow(acceleration[1], 2)));
    if (xAngle > M_PI) xAngle = xAngle - (M_PI);
    if (yAngle > M_PI) yAngle = yAngle - (M_PI);
    if (zAngle > M_PI) zAngle = zAngle - (M_PI);

    xAngle = xAngle * 180 / M_PI;  // convert radian to deg
    yAngle = yAngle * 180 / M_PI;
    zAngle = zAngle * 180 / M_PI;
    angle[0] = (double)xAngle;
    angle[1] = (double)yAngle;
    angle[2] = (double)zAngle;
    return true;
}

// return temperature in deg C, from the MPU9250
double BbbSensors::getTemp2()
{
    char data[2] = { 0, 0 };
    double temperature = 0.0;
    uint16_t temp16 = 0;

    ReadChars(I2C_ADDRESS_MPU9250, MPU9250_TEMP_OUT_H, data, 2);
    temp16 = (uint16_t)data[0] * 256 + (uint16_t)data[1];
    temperature = (double)temp16 / 333.87 + 21.0;

    return temperature; // in degC
}

bool BbbSensors::getXYZWordBigEndian(char address, char reg, int16_t* data)
{
    char c[6] = { 0, 0, 0, 0, 0, 0 };
    ReadChars(address, reg, c, 6);
    data[0] = ((int16_t)c[0] << 8) | c[1];  // Turn the MSB and LSB into a signed 16-bit value
    data[1] = ((int16_t)c[2] << 8) | c[3];  // Big Endian: XMSB, XLSB;  YMSB, YLSB; ZMSB, ZLSB 
    data[2] = ((int16_t)c[4] << 8) | c[5];
    return true;
}
bool BbbSensors::getXYZWordLitEndian(char address, char reg, int16_t* data)
{
    char c[6] = { 0, 0, 0, 0, 0, 0 };
    ReadChars(address, reg, c, 6);
    data[0] = ((int16_t)c[1] << 8) | c[0];  // Turn the MSB and LSB into a signed 16-bit value
    data[1] = ((int16_t)c[3] << 8) | c[2];  // Data stored as little Endian
    data[2] = ((int16_t)c[5] << 8) | c[4];  // XLSB, XMSB;  YLSB, YMSB; ZLSB, ZMSB
    return true;
}

int BbbSensors::ReadChars(char I2cAddress, char regAddress, char* data, int nbrChars)
{
    if (ioctl(file, I2C_SLAVE, I2cAddress) < 0) // Sensor's I2C address 
    {
        perror("Failed to acquire bus access and/or talk to slave.\n");
        return -1;
    }

    for (int i = 0; i < nbrChars; i++)
    {
        if (write(file, &regAddress, 1) != 1)
        {
            perror("Failed to write the i2c bus\n");
            return -1;
        }
        if (read(file, &data[i], 1) != 1)
        {
            perror("Failed to read from the i2c bus.\n");
            return -1;
        }
        //cout << "--Data: " << to_string(data[i]) << endl;
        regAddress = regAddress + 1;
    }

    return nbrChars;
}

int BbbSensors::WriteChars(char I2cAddress, char regAddress, char* data, int nbrChars)
{
    char packet[32] = { 0 }; // reg address + data

    packet[0] = regAddress;
    for (int i = 1; i <= nbrChars; i++)
    {
        packet[i] = data[i - 1];
    }

    if (ioctl(file, I2C_SLAVE, I2cAddress) < 0) // Sensor's I2C address 
    {
        perror("Failed to acquire bus access and/or talk to slave.\n");
        return -1;
    }
    if (write(file, packet, nbrChars + 1) != nbrChars + 1) // write data
    {
        perror("Failed to write the i2c bus\n");
        return -1;
    }
    return nbrChars;
}
