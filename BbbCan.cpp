#include "BbbCan.h"

using namespace std;

BbbCan::BbbCan()
{

}

MsgNMEA2k::MsgNMEA2k()
{
    parameter = "";
    value = -1.0;
}

bool BbbCan::InitCan()
{
    //cout << "Init CAN" << endl;
    if ((socketHandle = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket\n");
        return -1;
    }
    strcpy(ifr.ifr_name, ifname);
    if (ioctl(socketHandle, SIOCGIFINDEX, &ifr) < 0) // access to CAN bus 
    {
        perror("Failed to acquire CAN bus access.\n");
        return -1;
    }  
    SocketAddressCan.can_family = AF_CAN;
    SocketAddressCan.can_ifindex = ifr.ifr_ifindex;
    //printf("%s at index %d\n", ifname, ifr.ifr_ifindex);
    if (bind(socketHandle, (struct sockaddr*) & SocketAddressCan, sizeof(SocketAddressCan)) < 0) {
        perror("Error in socket CAN bind\n");
        return -2;
    }
    return true;
}

bool BbbCan::WriteHeading(double heading)
{
    CanFrame.can_id = PGN127250_HEADING | CAN_EFF_FLAG;
    CanFrame.can_dlc = CAN_MAX_DLEN;


    CanFrame.data[1] = (__u8)heading;
    CanFrame.data[0] = 0xFF;
    //CanFrame.data[1] = 0x11;
    CanFrame.data[2] = 0x22;
    CanFrame.data[3] = 0x33;
    CanFrame.data[4] = 0x44;
    CanFrame.data[5] = 0x55;
    CanFrame.data[6] = 0xFF;
    CanFrame.data[7] = 0xFF;
    
    return WriteCan(&CanFrame);
}
bool BbbCan::WriteRoll(double roll)
{
    CanFrame.can_id = PGN127257_ATTITUDE | CAN_EFF_FLAG;
    CanFrame.can_dlc = CAN_MAX_DLEN;
    
    CanFrame.data[1] = (__u8)roll;
    CanFrame.data[0] = 0xFF;
    //CanFrame.data[1] = 0x11;
    CanFrame.data[2] = (__u8)roll;
    CanFrame.data[3] = 0x33;
    CanFrame.data[4] = (__u8)roll;
    CanFrame.data[5] = 0x55;
    CanFrame.data[6] = (__u8)roll;
    CanFrame.data[7] = 0xFF;

    return WriteCan(&CanFrame);
}
double BbbCan::ParseDepth()
{
    //cout << "SeqID:" << to_string(CanFrame.data[0]) << "\n";
    //cout << "DepthBelowTransducer:" << to_string(CanFrame.data[4]) << to_string(CanFrame.data[3]) << to_string(CanFrame.data[2]) << to_string(CanFrame.data[1]) << "\n";  // *0.01
    //cout << "Offset:" << to_string(CanFrame.data[5]) << to_string(CanFrame.data[6]) << "\n";  // *0.001
    //cout << "Range:" << to_string(CanFrame.data[7]) << "\n"; // *10

    uint32_t D = (uint32_t)(((uint32_t)CanFrame.data[4] << 24) | ((uint32_t)CanFrame.data[3] << 16) | ((uint32_t)CanFrame.data[2] << 8) | CanFrame.data[1]);

    if (D == 0xFFFFFFFF)
    {
        return -0.001;
    }
    return (double)D * 0.001;
}

double BbbCan::ParseSpeed() 
{
    //cout << "SeqID:" << to_string(CanFrame.data[0]) << "\n";
    //cout << "SOW:" << to_string(CanFrame.data[2]) << to_string(CanFrame.data[1]) << "\n"; // *0.01
    //cout << "SOG:" << to_string(CanFrame.data[4]) << to_string(CanFrame.data[3]) << "\n";
    //cout << "SWRT:" << to_string(CanFrame.data[5]) << "\n";

    uint16_t S = (uint16_t)(((uint16_t)CanFrame.data[2] << 8) | CanFrame.data[1]);
    return (double)S * 0.01;
}

double BbbCan::ParseWaterTemp() // return water temperature in deg C
{
    //cout << "SeqID:" << to_string(CanFrame.data[0]) << "\n";
    //cout << "vb:" << to_string(CanFrame.data[1]) << "\n";
    //cout << "Temperature:" << to_string(CanFrame.data[3]) << to_string(CanFrame.data[2]) << "\n"; //(Temperature, 0.01)
    //cout << "Humidity:" << to_string(CanFrame.data[5]) << to_string(CanFrame.data[4]) << "\n";    //(Humidity, 0.004)
    //cout << "AtmosphericPressure:" << to_string(CanFrame.data[6]) << to_string(CanFrame.data[7]) << "\n"; //(AtmosphericPressure, 100)

    uint16_t T = (uint16_t)(((uint16_t)CanFrame.data[3] << 8) | CanFrame.data[2]);
    double temperature = (double)T * 0.01 - 273.15;
    return temperature;  
}

bool BbbCan::ReadNMEA2k(MsgNMEA2k* message)
{
    if (ReadCan(&CanFrame))
    {
        if (CanFrame.can_id == (PGN127250_HEADING | CAN_EFF_FLAG))
        {
            return false;
        }
        if (CanFrame.can_id == (PGN128259_SPEED | CAN_EFF_FLAG))
        {
            message->parameter = "Speed";
            message->value = ParseSpeed();
            return true;
        }
        if (CanFrame.can_id == (PGN128267_DEPTH | CAN_EFF_FLAG))
        {
            message->parameter = "Depth";
            message->value = ParseDepth();
            return true;
        }
        if (CanFrame.can_id == (PGN128275_DISTLOG | CAN_EFF_FLAG))
        {
            return false;
        }
        if (CanFrame.can_id == (PGN130311_WTEMP | CAN_EFF_FLAG))
        {
            message->parameter = "WaterTemperature";
            message->value = ParseWaterTemp();
            return true;
        }
    }
    return false;
}

bool BbbCan::WriteCan(can_frame* CF)
{
    if (write(socketHandle, CF, sizeof(struct can_frame)) < 0)
    {
        perror("Error : CAN bus raw socket write\n");
        return false;
    }
    return true;
}

bool BbbCan::ReadCan(can_frame* CF)
{
    if (read(socketHandle, CF, sizeof(struct can_frame)) < 0)
    {
        perror("Error : CAN bus raw socket read\n");
        return false;
    }
    return true;
}


// do system call and return resulst in a string
char* BbbCan::exec(const char* cmd)
{
    array<char, 128> buffer;
    string result;
    unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe)
    {
        throw runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    {
        result += buffer.data();
    }
    cout << result << endl;
    return buffer.data();
}
//
///*
//
////*****************************************************************************
//// Vessel Heading
//// Angles should be in radians
//void SetN2kPGN127250(tN2kMsg& N2kMsg, unsigned char SID, double Heading, double Deviation, double Variation, tN2kHeadingReference ref) {
//    N2kMsg.SetPGN(127250L);
//    N2kMsg.Priority = 2;
//    N2kMsg.AddByte(SID);
//    N2kMsg.Add2ByteUDouble(Heading, 0.0001);
//    N2kMsg.Add2ByteDouble(Deviation, 0.0001);
//    N2kMsg.Add2ByteDouble(Variation, 0.0001);
//    N2kMsg.AddByte(0xfc | ref);
//}
//
//bool ParseN2kPGN127250(const tN2kMsg& N2kMsg, unsigned char& SID, double& Heading, double& Deviation, double& Variation, tN2kHeadingReference& ref) {
//    if (N2kMsg.PGN != 127250L) return false;
//
//    int Index = 0;
//
//    SID = N2kMsg.GetByte(Index);
//    Heading = N2kMsg.Get2ByteUDouble(0.0001, Index);
//    Deviation = N2kMsg.Get2ByteDouble(0.0001, Index);
//    Variation = N2kMsg.Get2ByteDouble(0.0001, Index);
//    ref = (tN2kHeadingReference)(N2kMsg.GetByte(Index) & 0x03);
//
//    return true;
//}
////*****************************************************************************
//// Attitude
//// Input:
////  - SID                   Sequence ID. If your device is e.g. boat speed and heading at same time, you can set same SID for different messages
////                          to indicate that they are measured at same time.
////  - Yaw                   Heading in radians.
////  - Pitch                 Pitch in radians. Positive, when your bow rises.
////  - Roll                  Roll in radians. Positive, when tilted right.
//// Output:
////  - N2kMsg                NMEA2000 message ready to be send.
//void SetN2kPGN127257(tN2kMsg& N2kMsg, unsigned char SID, double Yaw, double Pitch, double Roll) {
//    N2kMsg.SetPGN(127257L);
//    N2kMsg.Priority = 3;
//    N2kMsg.AddByte(SID);
//    N2kMsg.Add2ByteDouble(Yaw, 0.0001);
//    N2kMsg.Add2ByteDouble(Pitch, 0.0001);
//    N2kMsg.Add2ByteDouble(Roll, 0.0001);
//    N2kMsg.AddByte(0xff); // Reserved
//}
//
//bool ParseN2kPGN127257(const tN2kMsg& N2kMsg, unsigned char& SID, double& Yaw, double& Pitch, double& Roll) {
//    if (N2kMsg.PGN != 127257L) return false;
//
//    int Index = 0;
//    SID = N2kMsg.GetByte(Index);
//    Yaw = N2kMsg.Get2ByteDouble(0.0001, Index);
//    Pitch = N2kMsg.Get2ByteDouble(0.0001, Index);
//    Roll = N2kMsg.Get2ByteDouble(0.0001, Index);
//
//    return true;
//}
////*****************************************************************************
//// Magnetic variation
//void SetN2kPGN127258(tN2kMsg& N2kMsg, unsigned char SID, tN2kMagneticVariation Source, uint16_t DaysSince1970, double Variation) {
//    N2kMsg.SetPGN(127258L);
//    N2kMsg.Priority = 6;
//    N2kMsg.AddByte(SID);
//    N2kMsg.AddByte(Source & 0x0f);
//    N2kMsg.Add2ByteUInt(DaysSince1970);
//    N2kMsg.Add2ByteDouble(Variation, 0.0001);
//    N2kMsg.Add2ByteUInt(0xffff);
//}
//
//bool ParseN2kPGN127258(const tN2kMsg& N2kMsg, unsigned char& SID, tN2kMagneticVariation& Source, uint16_t& DaysSince1970, double& Variation) {
//    if (N2kMsg.PGN != 127258L) return false;
//
//    int Index = 0;
//    SID = N2kMsg.GetByte(Index);
//    Source = (tN2kMagneticVariation)(N2kMsg.GetByte(Index) & 0x0f);
//    DaysSince1970 = N2kMsg.Get2ByteUInt(Index);
//    Variation = N2kMsg.Get2ByteDouble(0.0001, Index);
//
//    return true;
//}
//
////*****************************************************************************
//// Boat speed
//void SetN2kPGN128259(tN2kMsg& N2kMsg, unsigned char SID, double WaterReferenced, double GroundReferenced, tN2kSpeedWaterReferenceType SWRT) {
//    N2kMsg.SetPGN(128259L);
//    N2kMsg.Priority = 2;
//    N2kMsg.AddByte(SID);
//    N2kMsg.Add2ByteUDouble(WaterReferenced, 0.01);
//    N2kMsg.Add2ByteUDouble(GroundReferenced, 0.01);
//    N2kMsg.AddByte(SWRT);
//    N2kMsg.AddByte(0xff); // Reserved
//    N2kMsg.AddByte(0xff); // Reserved
//}
//
//bool ParseN2kPGN128259(const tN2kMsg& N2kMsg, unsigned char& SID, double& WaterReferenced, double& GroundReferenced, tN2kSpeedWaterReferenceType& SWRT) {
//    if (N2kMsg.PGN != 128259L) return false;
//
//    int Index = 0;
//
//    SID = N2kMsg.GetByte(Index);
//    WaterReferenced = N2kMsg.Get2ByteUDouble(0.01, Index);
//    GroundReferenced = N2kMsg.Get2ByteUDouble(0.01, Index);
//    SWRT = (tN2kSpeedWaterReferenceType)(N2kMsg.GetByte(Index) & 0x0F);
//
//    return true;
//}
//
////*****************************************************************************
//// Water depth
//void SetN2kPGN128267(tN2kMsg& N2kMsg, unsigned char SID, double DepthBelowTransducer, double Offset, double Range) {
//    N2kMsg.SetPGN(128267L);
//    N2kMsg.Priority = 3;
//    N2kMsg.AddByte(SID);
//    N2kMsg.Add4ByteUDouble(DepthBelowTransducer, 0.01);
//    N2kMsg.Add2ByteDouble(Offset, 0.001);
//    N2kMsg.Add1ByteUDouble(Range, 10);
//}
//
//bool ParseN2kPGN128267(const tN2kMsg& N2kMsg, unsigned char& SID, double& DepthBelowTransducer, double& Offset, double& Range) {
//    if (N2kMsg.PGN != 128267L) return false;
//
//    int Index = 0;
//    SID = N2kMsg.GetByte(Index);
//    DepthBelowTransducer = N2kMsg.Get4ByteUDouble(0.01, Index);
//    Offset = N2kMsg.Get2ByteDouble(0.001, Index);
//    Range = N2kMsg.Get1ByteUDouble(10, Index);
//
//    return true;
//}
//
////*****************************************************************************
//// Distance log
//void SetN2kPGN128275(tN2kMsg& N2kMsg, uint16_t DaysSince1970, double SecondsSinceMidnight, uint32_t Log, uint32_t TripLog) {
//    N2kMsg.SetPGN(128275L);
//    N2kMsg.Priority = 6;
//    N2kMsg.Add2ByteUInt(DaysSince1970);
//    N2kMsg.Add4ByteUDouble(SecondsSinceMidnight, 0.0001);
//    N2kMsg.Add4ByteUInt(Log);
//    N2kMsg.Add4ByteUInt(TripLog);
//}
//
//bool ParseN2kPGN128275(const tN2kMsg& N2kMsg, uint16_t& DaysSince1970, double& SecondsSinceMidnight, uint32_t& Log, uint32_t& TripLog) {
//    if (N2kMsg.PGN != 128275L) return false;
//
//    int Index = 0;
//
//    DaysSince1970 = N2kMsg.Get2ByteUInt(Index);
//    SecondsSinceMidnight = N2kMsg.Get4ByteDouble(0.0001, Index);
//    Log = N2kMsg.Get4ByteUDouble(1, Index);
//    TripLog = N2kMsg.Get4ByteUDouble(1, Index);
//
//    return true;
//}
////*****************************************************************************
//// Outside Environmental parameters
//void SetN2kPGN130310(tN2kMsg& N2kMsg, unsigned char SID, double WaterTemperature,
//    double OutsideAmbientAirTemperature, double AtmosphericPressure) {
//    N2kMsg.SetPGN(130310L);
//    N2kMsg.Priority = 5;
//    N2kMsg.AddByte(SID);
//    N2kMsg.Add2ByteUDouble(WaterTemperature, 0.01);
//    N2kMsg.Add2ByteUDouble(OutsideAmbientAirTemperature, 0.01);
//    N2kMsg.Add2ByteUDouble(AtmosphericPressure, 100);
//    N2kMsg.AddByte(0xff);  // reserved
//}
//bool ParseN2kPGN130310(const tN2kMsg &N2kMsg, unsigned char &SID, double &WaterTemperature,
//                     double &OutsideAmbientAirTemperature, double &AtmosphericPressure) {
//  if (N2kMsg.PGN!=130310L) return false;
//  int Index=0;
//  SID=N2kMsg.GetByte(Index);
//  WaterTemperature=N2kMsg.Get2ByteUDouble(0.01,Index);
//  OutsideAmbientAirTemperature=N2kMsg.Get2ByteUDouble(0.01,Index);
//  AtmosphericPressure=N2kMsg.Get2ByteUDouble(100,Index);
//
//  return true;
//}
////*****************************************************************************
//// Environmental parameters
//void SetN2kPGN130311(tN2kMsg &N2kMsg, unsigned char SID, tN2kTempSource TempSource, double Temperature,
//                     tN2kHumiditySource HumiditySource, double Humidity, double AtmosphericPressure) {
//    N2kMsg.SetPGN(130311L);
//    N2kMsg.Priority=5;
//    N2kMsg.AddByte(SID);
//    N2kMsg.AddByte(((HumiditySource) & 0x03)<<6 | (TempSource & 0x3f));
//    N2kMsg.Add2ByteUDouble(Temperature,0.01);
//    N2kMsg.Add2ByteDouble(Humidity,0.004);
//    N2kMsg.Add2ByteUDouble(AtmosphericPressure,100);
//}
//
//bool ParseN2kPGN130311(const tN2kMsg &N2kMsg, unsigned char &SID, tN2kTempSource &TempSource, double &Temperature,
//                     tN2kHumiditySource &HumiditySource, double &Humidity, double &AtmosphericPressure) {
//    if (N2kMsg.PGN!=130311L) return false;
//    unsigned char vb;
//    int Index=0;
//    SID=N2kMsg.GetByte(Index);
//    vb=N2kMsg.GetByte(Index); TempSource=(tN2kTempSource)(vb & 0x3f); HumiditySource=(tN2kHumiditySource)(vb>>6 & 0x03);
//    Temperature=N2kMsg.Get2ByteUDouble(0.01,Index);
//    Humidity=N2kMsg.Get2ByteDouble(0.004,Index);
//    AtmosphericPressure=N2kMsg.Get2ByteUDouble(100,Index);
//
//    return true;
//}
//
////*****************************************************************************
//// Actual Pressure
//// Pressure should be in Pascals
//void SetN2kPGN130314(tN2kMsg &N2kMsg, unsigned char SID, unsigned char PressureInstance,
//                     tN2kPressureSource PressureSource, double ActualPressure) {
//  N2kMsg.SetPGN(130314L);
//  N2kMsg.Priority = 5;
//  N2kMsg.AddByte(SID);
//  N2kMsg.AddByte((unsigned char) PressureInstance);
//  N2kMsg.AddByte((unsigned char) PressureSource);
//  N2kMsg.Add4ByteUDouble(ActualPressure,0.1);
//  N2kMsg.AddByte(0xff); // reserved
//}
//
//bool ParseN2kPGN130314(const tN2kMsg &N2kMsg, unsigned char &SID, unsigned char &PressureInstance,
//                       tN2kPressureSource &PressureSource, double &ActualPressure) {
//  if (N2kMsg.PGN != 130314L) return false;
//  int Index = 0;
//  SID=N2kMsg.GetByte(Index);
//  PressureInstance=N2kMsg.GetByte(Index);
//  PressureSource=(tN2kPressureSource)N2kMsg.GetByte(Index);
//  ActualPressure=N2kMsg.Get4ByteUDouble(0.1, Index);
//  return true;
//}
//
//
//*/