#include <memory>
#include <stdexcept>
#include <array>
#include <signal.h>

#include <cstdio>
#include <cstdlib>
#include <iostream>
//#include <string.h>

#include<unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include "BbbSensors.h"
#include "BbbCan.h"

#define NO_RCV_CAN
#define NO_RPT_CAN_WRITE

using namespace std;

int main()
{
    BbbSensors* Sensors = new BbbSensors();  // Class: sensors for Beaglebone Blue I2C devices
    BbbCan* CanBus = new BbbCan();
    //void rn(); // BbbCan* can, MsgNMEA2k* msg);

    cout << "BbbNmea2k... " ;
    // init I2C sensors on Beaglebone blue
    if (!Sensors->InitSensors())
    {
        cout << "Error can not access sensors on I2C bus." << endl;
        return 1;
    }
    else
    {
        cout << "I2C Sensors ok, "; // << endl;
    }
    // init CAN bus on Beaglebone blue
    if (!CanBus->InitCan())
    {
        cout << "Error can not access CAN bus." << endl;
        return 1;
    }
    else
    {
        cout << "CAN bus ok!" << endl;
    }

    double acceleration[3] = { 0.0, 0.0, 0.0 }; // acceleratiopn X, Y, Z
    double angle[3] = { 0.0, 0.0, 0.0 };
    double gyroscope[3] = { 0.0, 0.0, 0.0 };
    double magneto[3] = { 0.0, 0.0, 0.0 };
    double heading[3] = { 0.0, 0.0, 0.0 };

    ReadAllI2cSensors();

    // CAN bus
    cout << "BeagleBone Blue I2C sensors reader to CAN bus NMEA 2000 writer" << endl;
    cout << "DST800 NMEA 2000 CAN reader to BeagleBone Data out: console, SPI LCD, SSH... " << endl;
    int counter = 0;
    pid_t pid = fork();
    if (pid == 0)
    {
#ifndef NO_RCV_CAN
        // child process
        MsgNMEA2k* message = new MsgNMEA2k();
        while (1)
        {
            CanBus->ReadNMEA2k(message);
            cout << message->parameter << " " << to_string(message->value) << endl;
            usleep(2000);
        }
#endif // NO_RCV_CAN
    }
    else if (pid > 0)
    {
        // parent process
#ifndef NO_RPT_CAN_WRITE
        while (1)
        {
#endif // NO_RPT_CAN_WRITE


            CanBus->WriteHeading(Sensors->getHeading());
            usleep(250000);

            const int avg = 8;
            double a = 0.0;
            for (int i = 0; i < avg; i++)
            {
                usleep(1000);
                Sensors->getAngle(angle);
                a = a + angle[0];
            }
            a = a / avg;

            CanBus->WriteRoll(a);
            usleep(250000);
#ifndef NO_RPT_CAN_WRITE
        }
#endif // NO_RPT_CAN_WRITE
    }
    else
    {
        // fork failed
        printf("fork() failed!\n");
        return 1;
    }

    //kill(0, SIGKILL);

    return 0;
}

void ReadAllI2cSensors()
{
    // get sensors values from BMP280
    cout << " BMP280 Temperature and Pressure Sensors" << endl;
    printf("   Temp1    : %6.1f degC\n", Sensors->getTemp1());
    printf("   Pressure : %6.1f hpa\n", Sensors->getPressureHpa());
    // get altitude
    printf("      Alt : %6.2f m\n", Sensors->getAltitude());

    // get sensors values from MPU9250
    cout << " MPU9250 Temperature, Accel, Gyro and Magneto Sensors" << endl;
    printf("   Temp2    : %6.1f degC\n", Sensors->getTemp2());

    // acceleration value in g
    Sensors->getAccel(acceleration);
    printf("   Accel X  : %6.3f g\n", acceleration[0]);
    printf("   Accel Y  : %6.3f g\n", acceleration[1]);
    printf("   Accel Z  : %6.3f g\n", acceleration[2]);
    // angle/level value calculated from acceleration
    Sensors->getAngle(angle);
    printf("      Level X : %6.3f deg\n", angle[0]);
    printf("      Level Y : %6.3f deg\n", angle[1]);
    printf("      Level Z : %6.3f deg\n", angle[2]);

    // gyro value in degrees per second
    Sensors->getGyro(gyroscope);
    printf("   Gyro X  : %6.3f dps\n", gyroscope[0]);
    printf("   Gyro Y  : %6.3f dps\n", gyroscope[1]);
    printf("   Gyro Z  : %6.3f dps\n", gyroscope[2]);

    // magnetometer values in milliGauss
    Sensors->getMagneto(magneto);
    printf("   Mag X   : %6.3f mGauss\n", magneto[0]);
    printf("   Mag Y   : %6.3f mGauss\n", magneto[1]);
    printf("   Mag Z   : %6.3f mGauss\n", magneto[2]);
    // get heading
    printf("      heading : %6.2f deg\n", Sensors->getHeading());
    Sensors->getHeading(heading);
    printf("      hx : %6.2f deg\n", heading[0]);
    printf("      hy : %6.2f deg\n", heading[1]);
    printf("      hz : %6.2f deg\n", heading[2]);
}