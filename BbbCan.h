#pragma once

#include <memory>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string.h>
#include <sys/ioctl.h>
//#include <cstdio>
//#include <cstdlib>

#include<linux/can.h>
#include<linux/can/raw.h>
//#include<linux/can/error.h>
//#include<linux/can/bcm.h>
//#include<linux/can/gw.h>
//#include<linux/can/netlink.h>
#include<libsocketcan.h>
#include <sys/socket.h>
#include <net/if.h>

//#include <sys/types.h>
// example CAN NMEA 2000 :  DST800 transmitting depth, speed and temp
//      0DF50B23    [8] FF FF FF FF FF 00 00 FF
//      09F50323    [8] FF 4B 00 FF FF 00 FF FF
//      19F51323    [8]  20 0E FF FF FF FF FF FF
//      15FD0723    [8] FF C0 BF 71 FF 7F FF FF
//      19F51323    [8]  21 10 00 00 00 10 00 00
//      19F51323    [8]  22 00 FF FF FF FF FF FF
//      15FD0723    [8] FF C0 BF 71 FF 7F FF FF

class BbbCan
{
public:
    BbbCan();
    bool InitCan();

    bool WriteHeading(double heading);
    bool WritePressure(double pressure);
    bool WriteRoll(double roll);

    class MsgNMEA2k* message;
    bool ReadNMEA2k(MsgNMEA2k* message);

private:
    int socketHandle;       // file handle to socket CAN
    struct sockaddr_can SocketAddressCan;
    struct can_frame CanFrame;
    struct ifreq ifr;

    //const char* ifname = "can0";
    const char* ifname = "vcan0";

    const canid_t PGN128259_SPEED    = 0x09F50323U; // 128259 = 0x1F503
    const canid_t PGN128267_DEPTH    = 0x0DF50B23U;
    const canid_t PGN128275_DISTLOG  = 0x19F51323U;
    const canid_t PGN130311_WTEMP    = 0x15FD0723U;
    // todo a verfier le PGN heading...
    const canid_t PGN127250_HEADING  = 0x01F11223U;   // 127250 = 0x1F112 --> 0x??F11223
    const canid_t PGN127257_ATTITUDE = 0x01F11923U; 
    const canid_t PGN130310_PRESSURE = 0x01FD0623U;
    
    bool WriteCan(can_frame* CF);
    bool ReadCan(can_frame* CF);

    double ParseWaterTemp();
    double ParseDepth();
    double ParseSpeed();

    char* exec(const char* cmd);
   
};

class MsgNMEA2k
{
public:
    MsgNMEA2k();

    char* parameter;
    double value;
    
};
