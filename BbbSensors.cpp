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

    // Check and config MPU9250 Sensor
    if (!CheckSensorID(I2C_ADDRESS_MPU9250, MPU9250_ID_REG_ADDR, MPU9250_ID_REG_VAL))
    {
        return false;
    }
    // wake up device
    calibData[0] = 0x80;  // reset and restore default setting
    WriteChars(I2C_ADDRESS_MPU9250, MPU9250_PWR_MGMT_1, calibData, 1);
    usleep(100000);
    calibData[0] = 0x01;  // 0x00 use internal clock;   or 0x01 for best clock available 
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

    // Check and config AK8963 sensor, it is integrated in the MPU9250 IC
    calibData[0] = 0x22; // enable MPU9250 bypass mode to acces AK8963 at I2C 0x0C
    WriteChars(I2C_ADDRESS_MPU9250, MPU9250_INT_PIN_CFG, calibData, 1);
    calibData[0] = 0x01;  // Enable data ready (bit 0) interrupt
    WriteChars(I2C_ADDRESS_MPU9250, MPU9250_INT_ENABLE, calibData, 1);

    if (!CheckSensorID(I2C_ADDRESS_AK8963, AK8963_ID_REG_ADDR, AK8963_ID_REG_VAL))
    {
        return false;
    }
    calibData[0] = 0; // Power down magnetometer 
    WriteChars(I2C_ADDRESS_AK8963, AK8963_CNTL, calibData, 1);
    usleep(10000);
    calibData[0] = 0x1F; // Enter Fuse ROM access mode
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

bool BbbSensors::UpdateBMP280()
{
    char data[3] = { 0, 0, 0 };
    int pressure32 = 0;
    int temp32 = 0;

    // ********************
    ReadChars(I2C_ADDRESS_BMP280, BMP280_TEMP_MSB, data, 3);
    temp32 = (int)(((int)data[0] << 16 | (int)data[1] << 8 | data[2]) >> 4);
    //temp32 = ((uint32_t)data[0] * 65535 + (uint32_t)data[1] * 256 + (uint32_t)data[0])/16;
    int varTemp1, varTemp2;
    varTemp1 = ((((temp32 >> 3) - (BMPCal->dig_T1 << 1)))* (BMPCal->dig_T2)) >> 11;
    varTemp2 = (((((temp32 >> 4) - (BMPCal->dig_T1))* ((temp32 >> 4) - (BMPCal->dig_T1))) >> 12)* (BMPCal->dig_T3)) >> 14;
    temp32 = varTemp1 + varTemp2;
    temp32 = (temp32 * 5 + 128) >> 8;

    Temp1 = (double)temp32 * 0.01;
    BMPCal->t_fine = Temp1;

    // ********************
    ReadChars(I2C_ADDRESS_BMP280, BMP280_PRESS_MSB, data, 3);
    pressure32 = (int)(((int)data[0] << 16 | (int)data[1] << 8 | data[2]) >> 4);

    long long varPressure1, varPressure2, p;
    varPressure1 = ((long long)BMPCal->t_fine) - 128000;
    varPressure2 = varPressure1 * varPressure1 * (long long)BMPCal->dig_P6;
    varPressure2 = varPressure2 + ((varPressure1 * (long long)BMPCal->dig_P5) << 17);
    varPressure2 = varPressure2 + (((long long)BMPCal->dig_P4) << 35);
    varPressure1 = ((varPressure1 * varPressure1 * (long long)BMPCal->dig_P3) >> 8) + ((varPressure1 * (long long)BMPCal->dig_P2) << 12);
    varPressure1 = (((((long long)1) << 47) + varPressure1)) * ((long long)BMPCal->dig_P1) >> 33;
    if (varPressure1 == 0)
    {
        return false;     // avoid exception caused by division by zero
    }
    p = 1048576 - pressure32;
    p = (((p << 31) - varPressure2) * 3125) / varPressure1;
    varPressure1 = (((long long)BMPCal->dig_P9) * (p >> 13)* (p >> 13)) >> 25;
    varPressure2 = (((long long)BMPCal->dig_P8) * p) >> 19;
    p = ((p + varPressure1 + varPressure2) >> 8) + (((long long)BMPCal->dig_P7) << 4);

    Pressure = (double)p / 25600.0;
    PresAltitude = 145366.45f * (1.0f - pow((Pressure / 1013.25f), 0.190284f));

    return true;
}

// Return MAG field
bool BbbSensors::UpdateMagneto()
{
    int16_t magnetoWord[3] = { 0, 0, 0 };
    char c = { 0 };
    ReadChars(I2C_ADDRESS_AK8963, AK8963_ST1, &c, 1); // wait for magnetometer data ready bit to be set
    if (c & 0x01)
    {
        int tryMag = 0;
        do
        {
            tryMag = tryMag + 1;
            usleep(1000);
            getXYZWordLitEndian(I2C_ADDRESS_AK8963, AK8963_XOUT_L, magnetoWord);
            ReadChars(I2C_ADDRESS_AK8963, AK8963_ST2, &c, 1);
        } while ((c & 0x08) && tryMag < 4);
        if ((c & 0x08)) // Check if magnetic sensor overflow set, if yes clear data
        {
            magnetoWord[0] = 0; magnetoWord[1] = 0; magnetoWord[2] = 0; 
        }
        Magneto[0] = (double)magnetoWord[0] * MPU9250Cal->MPU9250mRes * MPU9250Cal->magCalibration[0] - MPU9250Cal->MPU9250magBias[0];
        Magneto[1] = (double)magnetoWord[1] * MPU9250Cal->MPU9250mRes * MPU9250Cal->magCalibration[1] - MPU9250Cal->MPU9250magBias[1];
        Magneto[2] = (double)magnetoWord[2] * MPU9250Cal->MPU9250mRes * MPU9250Cal->magCalibration[2] - MPU9250Cal->MPU9250magBias[2];
    }
    return true;
}
// Calculate Heading
bool BbbSensors::UpdateHeading()
{
    UpdateMagneto();
    MagHeading[0] = atan2(Magneto[0], sqrt(pow(Magneto[1], 2) + pow(Magneto[2], 2)));
    MagHeading[1] = atan2(Magneto[1], sqrt(pow(Magneto[0], 2) + pow(Magneto[2], 2)));
    MagHeading[2] = atan2(Magneto[2], sqrt(pow(Magneto[0], 2) + pow(Magneto[1], 2)));

    MagHeading[0] = MagHeading[0] * 180 / M_PI;  // convert radian to deg
    MagHeading[1] = MagHeading[1] * 180 / M_PI;
    MagHeading[2] = MagHeading[2] * 180 / M_PI;
    return true; // in deg
}

// Return ACCEL
bool BbbSensors::UpdateAccel()
{
    int16_t accelWord[3] = { 0, 0, 0 };
    getXYZWordBigEndian(I2C_ADDRESS_MPU9250, MPU9250_ACCEL_XOUT_H, accelWord); 
    // Now we'll calculate the accleration value into actual g's, this depends on scale being set
    Acceleration[0] = (double)accelWord[0] * MPU9250Cal->MPU9250aRes - MPU9250Cal->MPU9250accelBias[0];
    Acceleration[1] = (double)accelWord[1] * MPU9250Cal->MPU9250aRes - MPU9250Cal->MPU9250accelBias[1];
    Acceleration[2] = (double)accelWord[2] * MPU9250Cal->MPU9250aRes - MPU9250Cal->MPU9250accelBias[2];
    return true;
}
// Calculate Angle
bool BbbSensors::UpdateAccAngle()
{
    UpdateAccel();
    AccAngle[0] = atan2(Acceleration[0], sqrt(pow(Acceleration[1], 2) + pow(Acceleration[2], 2)));
    AccAngle[1] = atan2(Acceleration[1], sqrt(pow(Acceleration[0], 2) + pow(Acceleration[2], 2)));
    AccAngle[2] = atan2(Acceleration[2], sqrt(pow(Acceleration[0], 2) + pow(Acceleration[1], 2)));
    if (AccAngle[0] > M_PI) AccAngle[0] = AccAngle[0] - (M_PI);
    if (AccAngle[1] > M_PI) AccAngle[1] = AccAngle[1] - (M_PI);
    if (AccAngle[2] > M_PI) AccAngle[2] = AccAngle[2] - (M_PI);

    AccAngle[0] = AccAngle[0] * 180 / M_PI;  // convert radian to deg
    AccAngle[1] = AccAngle[1] * 180 / M_PI;
    AccAngle[2] = AccAngle[2] * 180 / M_PI;
    return true;
}

// return temperature in deg C, from the MPU9250
bool BbbSensors::UpdateTemp2()
{
    char data[2] = { 0, 0 };
    uint16_t temp16 = 0;

    ReadChars(I2C_ADDRESS_MPU9250, MPU9250_TEMP_OUT_H, data, 2);
    temp16 = (uint16_t)data[0] * 256 + (uint16_t)data[1];
    Temp2 = (double)temp16 / 333.87 + 21.0;

    return true;
}

// Return GYRO
bool BbbSensors::UpdateGyro()
{
    int16_t gyroWord[3] = { 0, 0, 0 };
    getXYZWordBigEndian(I2C_ADDRESS_MPU9250, MPU9250_GYRO_XOUT_H, gyroWord);
    Gyroscope[0] = (double)gyroWord[0] * MPU9250Cal->MPU9250gRes - MPU9250Cal->MPU9250gyroBias[0];
    Gyroscope[1] = (double)gyroWord[1] * MPU9250Cal->MPU9250gRes - MPU9250Cal->MPU9250gyroBias[1];
    Gyroscope[2] = (double)gyroWord[2] * MPU9250Cal->MPU9250gRes - MPU9250Cal->MPU9250gyroBias[2];
    return true;
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


double BbbSensors::getPressureHpa() // Return PRESSURE in deg hPa, from the BMP280
{
    return Pressure;
}
double BbbSensors::getAltitude() // Calculate Altitude, from pressure
{
    return PresAltitude;
}
double BbbSensors::getTemp1() // Return TEMPERATURE in deg C, from the BMP280
{
    return Temp1;
}
double BbbSensors::getTemp2() // Return TEMPERATURE in deg C, from the MPU9250
{
    return Temp2;
}






