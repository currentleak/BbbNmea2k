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
//#define NO_RPT_CAN_WRITE

using namespace std;

int main()
{
    BbbSensors* Sensors = new BbbSensors();  // Class: sensors for Beaglebone Blue I2C devices
    BbbCan* CanBus = new BbbCan();
    void ReadAllI2cSensors(BbbSensors * Sensors);

    cout << "BbbNmea2k : BeagleBone Blue I2C sensors reader to CAN bus NMEA 2000 writer" << endl;
    cout << "DST800 NMEA 2000 CAN reader to BeagleBone Data out: console, SPI LCD, SSH... " << endl;
    // init I2C sensors on Beaglebone blue
    if (!Sensors->InitSensors())  // I2C sensors
    {
        cout << "Error can not access sensors on I2C bus." << endl;
        return 1;
    }
    else
    {
        cout << "I2C Sensors ok, ";
    }
    // init CAN bus on Beaglebone blue
    if (!CanBus->InitCan())  // CAN Bus
    {
        cout << "Error can not access CAN bus." << endl;
        return 1;
    }
    else
    {
        cout << "CAN bus ok!" << endl;
    }

    ReadAllI2cSensors(Sensors);

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

          ReadAllI2cSensors(Sensors);
            double h = 0.0;
            double a = 0.0;
            for (int i = 0; i < Sensors->avgFactor; i++)
            {
                usleep(1000);
                Sensors->UpdateAccAngle();
                a = a + Sensors->AccAngle[0]; // 0 -> X, 1 -> Y, 2 -> Z
                Sensors->UpdateHeading();
                h = h + Sensors->MagHeading[0]; // 0 -> X, 1 -> Y, 2 -> Z
            }
            a = a / Sensors->avgFactor;
            h = h / Sensors->avgFactor;

            CanBus->WriteRoll(a);
            usleep(10000);
            CanBus->WriteHeading(h);

            usleep(400000);
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

void ReadAllI2cSensors(BbbSensors* Sensors)
{
    // get sensors values from BMP280
    cout << " BMP280 Temperature and Pressure Sensors" << endl;
    Sensors->UpdateBMP280();
    printf("   Temp1    : %6.1f degC\n", Sensors->getTemp1());
    printf("   Pressure : %6.1f hpa\n", Sensors->getPressureHpa());
    // get altitude
    printf("      Alt : %6.2f m\n", Sensors->getAltitude());

    // get sensors values from MPU9250
    cout << " MPU9250 Temperature, Accel, Gyro and Magneto Sensors" << endl;
    Sensors->UpdateTemp2();
    printf("   Temp2    : %6.1f degC\n", Sensors->getTemp2());

    // Acceleration value in g
    Sensors->UpdateAccel();
    printf("   Accel X  : %6.3f g\n", Sensors->Acceleration[0]);
    printf("   Accel Y  : %6.3f g\n", Sensors->Acceleration[1]);
    printf("   Accel Z  : %6.3f g\n", Sensors->Acceleration[2]);
    // Angle/level value calculated from Acceleration
    usleep(50000);
    Sensors->UpdateAccAngle();
    printf("      Level X : %6.3f deg\n", Sensors->AccAngle[0]);
    printf("      Level Y : %6.3f deg\n", Sensors->AccAngle[1]);
    printf("      Level Z : %6.3f deg\n", Sensors->AccAngle[2]);

    // gyro value in degrees per second
    Sensors->UpdateGyro();
    printf("   Gyro X  : %6.3f dps\n", Sensors->Gyroscope[0]);
    printf("   Gyro Y  : %6.3f dps\n", Sensors->Gyroscope[1]);
    printf("   Gyro Z  : %6.3f dps\n", Sensors->Gyroscope[2]);

    // Magnetometer values in milliGauss
    Sensors->UpdateMagneto();
    printf("   Mag X   : %6.3f mGauss\n", Sensors->Magneto[0]);
    printf("   Mag Y   : %6.3f mGauss\n", Sensors->Magneto[1]);
    printf("   Mag Z   : %6.3f mGauss\n", Sensors->Magneto[2]);
    // get Heading
    usleep(50000);
    Sensors->UpdateHeading();
    printf("      hx : %6.2f deg\n", Sensors->MagHeading[0]);
    printf("      hy : %6.2f deg\n", Sensors->MagHeading[1]);
    printf("      hz : %6.2f deg\n", Sensors->MagHeading[2]);
}