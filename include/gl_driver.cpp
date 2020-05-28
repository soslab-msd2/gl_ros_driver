#include <chrono>
#include <thread>
#include <iostream>

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


		
serial::Serial * port_;
uint8_t cs_;

//////////////////////////////////////////////////////////////
// Constructor and Deconstructor for GL Class
//////////////////////////////////////////////////////////////

Gl::Gl(std::string &port, uint32_t baudrate)
{
    /* initialize pointer to a new Serial port object */
    port_ = new serial::Serial(port, baudrate, serial::Timeout::simpleTimeout(100));
    if(port_->isOpen()) printf("GL Serial is opened.\n");
}

Gl::~Gl()
{
    port_->close();
    delete port_;
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
    cs_update(data);
}

bool read(uint8_t *data)
{
    uint8_t buff[1];
    if (port_->read(buff, 1) == 1)
    {
        *data = buff[0];
        cs_update(buff[0]);
        return true;
    }
    else
    {
        return false;
    }
}

void write_PS()
{   
    uint8_t PS[4] = {PS1, PS2, PS3, PS4};

    for(int i=0; i<4; i++) write(PS[i]);
}

void write_packet(uint8_t PI, uint8_t PL, uint8_t SM, uint8_t CAT0, uint8_t CAT1, uint16_t DTL, uint8_t *DTn)
{
    uint8_t buff;

    flush();
    cs_clear();

    write_PS();

    uint16_t TL = DTL + 14;
    buff = TL&0xff;
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

    write(cs_get());
}

bool check_PS(void)
{
    uint8_t data;
    
    cs_clear();

    while(read(&data)) 
    {
        if(data==PS1)
        {
            if(!read(&data)) return false;
            if(data==PS2)
            {
                if(!read(&data)) return false;
                if(data==PS3)
                {
                    if(!read(&data)) return false;
                    if(data==PS4)
                    {
                        return true;
                    }
                }    
            }
        }
        cs_clear();
    }
    
    return false;
}

bool read_packet(uint8_t SM, uint8_t CAT0, uint8_t CAT1, uint16_t *DTL, uint8_t *DTn)
{
    uint8_t buff;

    if(check_PS())
    {
        if(!read(&buff)) return false;
        uint16_t TL = buff&0xff;
        if(!read(&buff)) return false;
        TL |= (uint16_t)(buff&0xff)<<8;

        if(!read(&buff)) return false;
        if(!read(&buff)) return false;
           
        if(!read(&buff) || SM!=buff&0xff) return false;
            
        if(!read(&buff) || BI_GL3102PC!=buff&0xff) return false;
            
        if(!read(&buff) || CAT0!=buff&0xff) return false;
        if(!read(&buff) || CAT1!=buff&0xff) return false;

        *DTL = TL - 14;
            
        for(int i=0; i<*DTL; i++)
        {
            if(!read(&buff)) return false;
            DTn[i] = buff;
        }
            
        if(!read(&buff) || PE!=buff&0xff) return false;

        uint8_t cs = cs_get();
        if(!read(&buff) || cs!=buff&0xff) return false;

        return true;
    }

    return false;
}


/************************************************************
 * Read GL Conditions                                       *
 ************************************************************/

std::string Gl::GetSerialNum(void)
{
    std::string out_str;

    uint8_t PI = 0;
    uint8_t PL = 1;
    uint8_t SM = SM_GET;
    uint8_t CAT0 = 0x02;
    uint8_t CAT1 = 0x0A;

    uint16_t DTL = 1;
    uint8_t DTn[DTL] = {1};
    write_packet(PI, PL, SM, CAT0, CAT1, DTL, DTn);

    uint16_t DTL2;
    uint8_t DTn2[15];

    for(int i=0; i<10; i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if(read_packet(SM, CAT0, CAT1, &DTL2, DTn2))
        {
            out_str = std::string((char*)DTn2);
            out_str.resize(DTL2);
            return out_str;
        }
    }
    
    out_str = "[ERROR] Serial Number is not received.";
    return out_str;
}


Gl::framedata_t ParsingFrameData(uint16_t DTL, uint8_t *data)
{
    Gl::framedata_t frame_data;

    uint16_t frame_data_size;
    frame_data_size = data[0]&0xff;
    frame_data_size |= (data[1]&0xff)<<8;

    uint16_t distance[frame_data_size];
    uint16_t pulse_width[frame_data_size];

    frame_data.distance.resize(frame_data_size);
    frame_data.pulse_width.resize(frame_data_size);
    frame_data.angle.resize(frame_data_size);
    for(int i=0; i<frame_data_size; i++)
    {
        distance[i] = data[i*4+2]&0xff;
        distance[i] |= (uint16_t)(data[i*4+3]&0xff)<<8;

        pulse_width[i] = data[i*4+4]&0xff;
        pulse_width[i] |= (uint16_t)(data[i*4+5]&0xff)<<8;

        frame_data.distance[i] = distance[i];
        frame_data.pulse_width[i] = pulse_width[i];
        frame_data.angle[i] = (-90.0+(double)i*180.0/1000.0)*3.141592/180.0;
    }

    return frame_data;
}

Gl::framedata_t Gl::ReadFrameData(void)
{
    Gl::framedata_t frame_data;

    uint8_t SM = SM_STREAM;
    uint8_t CAT0 = 0x01;
    uint8_t CAT1 = 0x02;

    uint16_t DTL;
    uint8_t DTn[5000];
    if(read_packet(SM, CAT0, CAT1, &DTL, DTn)) 
    {
        frame_data = ParsingFrameData(DTL, DTn);
    }

    return frame_data;
}


//////////////////////////////////////////////////////////////
// Set GL Conditions                                       
//////////////////////////////////////////////////////////////

void Gl::SetFrameDataEnable(bool framedata_enable)
{
    uint8_t PI = 0;
    uint8_t PL = 1;
    uint8_t SM = SM_SET;
    uint8_t CAT0 = 0x01;
    uint8_t CAT1 = 0x03;

    uint16_t DTL = 1;
    uint8_t DTn[DTL] = {framedata_enable};
    write_packet(PI, PL, SM, CAT0, CAT1, DTL, DTn);
}

