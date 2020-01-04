#include <memory>
#include <stdexcept>
#include <array>
//#include <thread>
//#include <pthread.h>

#include <cstdio>
#include <cstdlib>
#include <iostream>
//#include <string.h>

#include<unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include "BbbSensors.h"
#include "BbbCan.h"

//#define DEBUG

using namespace std;

//void *rn() // BbbCan* can, MsgNMEA2k* msg)
//{
//    //can->ReadNMEA2k(msg);
//
//    cout << "Thread" << endl;
//}

int main()
{
    BbbSensors* Sensors = new BbbSensors();  // Class: sensors for Beaglebone Blue I2C devices
    BbbCan* CanBus = new BbbCan();
    //void rn(); // BbbCan* can, MsgNMEA2k* msg);

    cout << "BbbNmea2k... " ;
    // init I2C sensors on Beaglebone blue
    if (!Sensors->InitSensors())
    {
        cout << "error can not access sensors on I2C bus" << endl;
        return 1;
    }
    else
    {
        cout << "I2C Sensors ok, "; // << endl;
    }
    // init CAN bus on Beaglebone blue
    if (!CanBus->InitCan())
    {
        cout << "error can not access CAN bus" << endl;
        return 1;
    }
    else
    {
        cout << "Can bus ok!" << endl;
    }

    double acceleration[3] = { 0.0, 0.0, 0.0 }; // acceleratiopn X, Y, Z
    double angle[3] = { 0.0, 0.0, 0.0 };
    double gyroscope[3] = { 0.0, 0.0, 0.0 };
    double magneto[3] = { 0.0, 0.0, 0.0 };
    double heading[3] = { 0.0, 0.0, 0.0 };

#ifdef DEBUG
    while (1)
    {
#endif // DEBUG

        // get sensors values from BMP280
        cout << " BMP280 Temperature and Pressure Sensors" << endl;
        printf("   Temp1    : %6.1f degC\n", Sensors->getTemp1());
        printf("   Pressure : %6.1f hpa\n", Sensors->getPressureHpa());
          // get altitude
        printf("      Alt : %6.2f m\n", Sensors->getAltitude());

        // get sensors values from MPU9250
        cout << " MPU9250 Temperature, Accel, Gyro and Magneto Sensors" << endl;
        printf("   Temp2    : %6.1f degC\n", Sensors->getTemp2());

        // accleration value in g
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


        // CAN bus
        cout << " CAN bus" << endl;
        CanBus->WriteHeading(Sensors->getHeading());

        Sensors->getAngle(angle);
        CanBus->WriteRoll(angle[0]);
        





        //int rc;
        //pthread_t ThreadReadCan;
        //long t = 0;
        //rc = pthread_create(&ThreadReadCan, NULL, rn, (void*)t); // , std::ref(CanBus), std::ref(message));
        //ThreadReadCan.join();

        int counter = 0;
        pid_t pid = fork();
        if (pid == 0)
        {
            // child process
            while (1)
            {
                CanBus->WriteHeading(Sensors->getHeading());
                Sensors->getAngle(angle);
                CanBus->WriteRoll(angle[0]);
                usleep(2000000);
            }

        }
        else if (pid > 0)
        {
            // parent process
            MsgNMEA2k* message = new MsgNMEA2k();
            while (1)
            {
                CanBus->ReadNMEA2k(message);
                cout << message->parameter << " " << to_string(message->value) << endl;
            }
        }
        else
        {
            // fork failed
            printf("fork() failed!\n");
            return 1;
        }




        

#ifdef DEBUG
        //usleep(500000);
        if (getchar() == 'q')
            return 0;
    }
#endif // DEBUG

    

    return 0;
}
