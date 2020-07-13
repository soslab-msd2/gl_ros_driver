#include "gl_driver.h"


#define PS1             0xC3
#define PS2             0x51
#define PS3             0xA1
#define PS4             0xF8
#define SM_SET          0
#define SM_GET          1
#define SM_STREAM       2
#define SM_ERROR        255
#define BI_PC2GL310     0x21
#define BI_GL3102PC     0x12
#define PE              0xC2


#define STATE_INIT      0
#define STATE_PS1       1
#define STATE_PS2       2
#define STATE_PS3       3
#define STATE_PS4       4
#define STATE_preDATA   5
#define STATE_PE        6
#define STATE_CS        7


int recv_state = STATE_INIT;

std::vector<uint8_t> recv_packet;
std::vector<uint8_t> recv_data;
int recv_TL = 0;
int recv_SM = 0;
int recv_CAT0 = 0;
int recv_CAT1 = 0;
int recv_DTL = 0;

std::vector<uint8_t> lidar_data;
std::vector<uint8_t> serial_num;

		
serial::Serial * port_;
uint8_t cs_;
uint8_t write_cs_;


//////////////////////////////////////////////////////////////
// Constructor and Deconstructor for GL Class
//////////////////////////////////////////////////////////////

Gl::Gl(std::string &port, uint32_t baudrate)
{
    OpenSerial(port, baudrate);
}

Gl::Gl()
{
    ;
}

Gl::~Gl()
{
    SetFrameDataEnable(false);

    port_->close();
    delete port_;
    thread_running = false;
}


void Gl::OpenSerial(std::string &port, uint32_t baudrate)
{
    port_ = new serial::Serial(port, baudrate, serial::Timeout::simpleTimeout(10));
    if(port_->isOpen()) std::cout << "GL Serial is opened." << std::endl;

    th = std::thread(&Gl::ThreadCallBack,this);
}


//////////////////////////////////////////////////////////////
// Functions for Serial Comm
//////////////////////////////////////////////////////////////

void flush()
{
    port_->flush();
}

void cs_update(uint8_t data)
{
    cs_ = cs_^ (data&0xff);
}

uint8_t cs_get()
{
    return cs_&0xff;
}

void cs_clear()
{
    cs_ = 0;
}

void write(uint8_t data)
{
    port_->write(&data, 1);
    write_cs_ = write_cs_^ (data&0xff);
}

bool read(uint8_t *data)
{
    uint8_t buff;
    if (port_->read(&buff, 1) == 1)
    {
        *data = buff;
        // cs_update(buff);
        return true;
    }
    else
    {
        return false;
    }
}

void write_PS()
{   
    std::vector<uint8_t> PS = {PS1, PS2, PS3, PS4};

    for(int i=0; i<PS.size(); i++) write(PS[i]);
}

void write_packet(uint8_t PI, uint8_t PL, uint8_t SM, uint8_t CAT0, uint8_t CAT1, std::vector<uint8_t> DTn)
{
    flush();

    write_cs_ = 0;

    write_PS();

    uint16_t DTL = DTn.size();

    uint16_t TL = DTL + 14;
    uint8_t buff = TL&0xff;
    write(buff);
    buff = (TL>>8)&0xff;
    write(buff);

    write(PI);
    write(PL);
    write(SM);
    write(BI_PC2GL310);
    write(CAT0);
    write(CAT1);

    for(int i=0; i<DTL; i++) write(DTn[i]);

    write(PE);

    write(write_cs_&0xff);
}


void recv_packet_clear(void)
{
    cs_clear();
    recv_state = STATE_INIT;

    recv_packet.clear();
    recv_TL = 0;
    recv_SM = 0;
    recv_CAT0 = 0;
    recv_CAT1 = 0;
    recv_DTL = 0;
    recv_data.clear();
}


int check_PS(uint8_t data)
{
    if(recv_state==STATE_INIT)
    {
        if(data==PS1)
        {
            recv_packet_clear();
            cs_update(data);
            recv_state = STATE_PS1;
        }
    }
    else if(recv_state==STATE_PS1)
    {
        if(data==PS2)
        {
            cs_update(data);
            recv_state = STATE_PS2;
        }
        else return 0;
    }
    else if(recv_state==STATE_PS2)
    {
        if(data==PS3)
        {
            cs_update(data);
            recv_state = STATE_PS3;
        }
        else return 0;
    }
    else if(recv_state==STATE_PS3)
    {
        if(data==PS4)
        {
            cs_update(data);
            recv_state = STATE_PS4;
        }
        else return 0;
    }
    else return 2;

    return 1;
}


void save_data(std::vector<uint8_t> recv_data, int SM, int CAT0, int CAT1)
{
    // GetSerialNum()
    if(SM==SM_GET && CAT0==0x02 && CAT1==0x0A)
    {
        serial_num = recv_data;
    }
    // ReadFrameData()
    else if(SM==SM_STREAM && CAT0==0x01 && CAT1==0x02)
    {
        lidar_data = recv_data;
    }

}


void add_packet_element(uint8_t data)
{
    int PS_result = check_PS(data);
    if(PS_result==0)
    {
        recv_packet_clear();
        if(data==PS1) recv_state = STATE_PS1;
    }
    else if(PS_result==2)
    { 
        if(recv_state==STATE_PS4)
        {
            cs_update(data);
            recv_packet.push_back(data);

            if(recv_packet.size()==6 and recv_packet[5]!=BI_GL3102PC)
            {
                std::vector<uint8_t> packet = recv_packet;
                recv_packet_clear();
                for(int i=0; i<packet.size(); i++) add_packet_element(packet[i]);
            }

            if(recv_packet.size()==9)
            {
                recv_TL = recv_packet[0]&0xff;
                recv_TL = recv_TL | ((recv_packet[1]&0xff)<<8);

                recv_SM = recv_packet[4]&0xff;

                recv_CAT0 = recv_packet[6]&0xff;
                recv_CAT1 = recv_packet[7]&0xff;
                    
                recv_DTL = recv_TL - 14;
            }

            if(recv_DTL>0)
            {
                if(recv_DTL>recv_data.size()) recv_data.push_back(data);
                else recv_state = STATE_preDATA;
            }

            if(recv_state==STATE_preDATA)
            {
                if(data==PE) recv_state = STATE_PE;
                else
                {
                    std::vector<uint8_t> packet = recv_packet;
                    recv_packet_clear();
                    for(int i=0; i<packet.size(); i++) add_packet_element(packet[i]);
                }
            }
        }
        else if(recv_state==STATE_PE)
        {
            if(data==cs_get())
            {
                save_data(recv_data, recv_SM, recv_CAT0, recv_CAT1);
                recv_packet_clear();
            }
            else
            {
                std::vector<uint8_t> packet = recv_packet;
                recv_packet_clear();
                for(int i=0; i<packet.size(); i++) add_packet_element(packet[i]);
            }
        }
    }
}


void Gl::ThreadCallBack(void) 
{
    flush();
    recv_packet_clear();

    while(thread_running==true)
    {
        uint8_t data;
        while(read(&data))
        {
            add_packet_element(data);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}


//////////////////////////////////////////////////////////////
// Read GL Conditions
//////////////////////////////////////////////////////////////

std::string Gl::GetSerialNum(void)
{
    uint8_t PI = 0;
    uint8_t PL = 1;
    uint8_t SM = SM_GET;
    uint8_t CAT0 = 0x02;
    uint8_t CAT1 = 0x0A;
    std::vector<uint8_t> DTn = {1};

    serial_num.clear();
    for(int i=0; i<50; i++)
    {
        write_packet(PI, PL, SM, CAT0, CAT1, DTn);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if(serial_num.size()>0)
        {
            std::string out_str(serial_num.begin(), serial_num.end());
            return out_str;
        }
    }

    std::string out_str = "[ERROR] Serial Number is not received.";
    return out_str;
}

Gl::framedata_t ParsingFrameData(std::vector<uint8_t> data)
{
    uint16_t frame_data_size = data[0]&0xff;
    frame_data_size |= ((uint16_t)(data[1]&0xff))<<8;

    Gl::framedata_t frame_data;
    frame_data.distance.resize(frame_data_size);
    frame_data.pulse_width.resize(frame_data_size);
    frame_data.angle.resize(frame_data_size);
    for(int i=0; i<frame_data_size; i++)
    {
        uint16_t distance = data[i*4+2]&0xff;
        distance |= ((uint16_t)(data[i*4+3]&0xff))<<8;

        uint16_t pulse_width = data[i*4+4]&0xff;
        pulse_width |= ((uint16_t)(data[i*4+5]&0xff))<<8;

        frame_data.distance[i] = distance/1000.0;
        frame_data.pulse_width[i] = pulse_width;
        frame_data.angle[i] = i*180.0/(frame_data_size-1)*3.141592/180.0;
    }

    return frame_data;
}

Gl::framedata_t Gl::ReadFrameData(void)
{
    Gl::framedata_t frame_data;

    if(lidar_data.size()>0)
    {
        frame_data = ParsingFrameData(lidar_data);
    }

    lidar_data.clear();

    return frame_data;
}

//////////////////////////////////////////////////////////////
// Set GL Conditions
//////////////////////////////////////////////////////////////

void Gl::SetFrameDataEnable(uint8_t framedata_enable)
{
    uint8_t PI = 0;
    uint8_t PL = 1;
    uint8_t SM = SM_SET;
    uint8_t CAT0 = 0x1;
    uint8_t CAT1 = 0x3;

    std::vector<uint8_t> DTn = {framedata_enable};
    write_packet(PI, PL, SM, CAT0, CAT1, DTn);
}
