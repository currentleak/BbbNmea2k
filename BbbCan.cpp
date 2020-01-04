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

    int16_t h;
    heading = heading / 180.0 * 3.1415926535897932384626433832795; // convert to radian
    heading = heading * (1 / 0.0001);
    h = (int16_t)heading;

    CanFrame.data[0] = 0xFF;
    CanFrame.data[1] = (__u8)h;
    CanFrame.data[2] = (__u8)(h >> 8);
    CanFrame.data[3] = 0xFF;
    CanFrame.data[4] = 0xFF;
    CanFrame.data[5] = 0xFF;
    CanFrame.data[6] = 0xFF;
    CanFrame.data[7] = 0xFF;
    
    return WriteCan(&CanFrame);
}
bool BbbCan::WriteRoll(double roll)
{
    CanFrame.can_id = PGN127257_ATTITUDE | CAN_EFF_FLAG;
    CanFrame.can_dlc = CAN_MAX_DLEN;
    
    int16_t r;
    roll = roll / 180.0 * 3.1415926535897932384626433832795;  // convert to radian
    roll = roll *(1 / 0.0001); 
    r = (int16_t)roll;

    CanFrame.data[0] = 0xFF; CanFrame.data[1] = 0xFF; CanFrame.data[2] = 0xFF; CanFrame.data[3] = 0xFF;
    CanFrame.data[4] = 0xFF;
    CanFrame.data[5] = (__u8)r;
    CanFrame.data[6] = (__u8)(r >> 8);
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
