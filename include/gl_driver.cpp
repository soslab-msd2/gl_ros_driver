
#include "gl_driver.h"

#include <fcntl.h>
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>


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

#define COMM_SERIAL     1
#define COMM_UDP        2


int recv_state = STATE_INIT;

uint8_t comm_type;
serial::Serial* serial_port_;
int sockfd_;
struct sockaddr_in servaddr_, clientaddr_; 

uint8_t read_cs_;
uint8_t write_cs_;

std::vector<uint8_t> send_packet;
std::vector<uint8_t> recv_packet;
std::vector<uint8_t> recv_data;


std::vector<uint8_t> serial_num;
std::vector<uint8_t> lidar_data;
Gl::framedata_t frame_data_in;


//////////////////////////////////////////////////////////////
// Constructor and Deconstructor for GL Class
//////////////////////////////////////////////////////////////

Gl::Gl(std::string& gl_ip, int gl_port, int pc_port)
{
    comm_type = COMM_UDP;

    // server setting
    if ( (sockfd_=socket(AF_INET, SOCK_DGRAM, 0)) == -1 ) 
    { 
		perror("[ERROR] Socket creation failed"); 
		exit(EXIT_FAILURE); 
	} 

    // Filling server information 
	memset(&servaddr_, 0, sizeof(servaddr_)); 
	servaddr_.sin_family = AF_INET; 
	servaddr_.sin_port = htons(gl_port); 
    const char * c = gl_ip.c_str();
	servaddr_.sin_addr.s_addr = inet_addr(c);

    memset(&clientaddr_, 0, sizeof(clientaddr_)); 
	clientaddr_.sin_family = AF_INET; 
	clientaddr_.sin_port = htons(pc_port); 
    clientaddr_.sin_addr.s_addr = htonl( INADDR_ANY );

    if ( bind(sockfd_,(struct sockaddr *)&clientaddr_,sizeof(clientaddr_)) == -1 ) 
    { 
		perror("[ERROR] Bind failed"); 
		exit(EXIT_FAILURE); 
	} 
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "Socket START [" << sockfd_ << "]" << std::endl;

    th = std::thread(&Gl::ThreadCallBack,this);
}


Gl::Gl(std::string& gl_serial_name, uint32_t gl_serial_baudrate)
{
    comm_type = COMM_SERIAL;

    serial_port_ = new serial::Serial(gl_serial_name, gl_serial_baudrate, serial::Timeout::simpleTimeout(1));
    if(serial_port_->isOpen()) std::cout << "GL Serial is opened." << std::endl;

    int fd = open(gl_serial_name.c_str(), O_RDONLY);
    struct termios2 tio;
    ioctl(fd, TCGETS2, &tio);
    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = gl_serial_baudrate;
    tio.c_ospeed = gl_serial_baudrate;
    ioctl(fd, TCSETS2, &tio);
    close(fd);

    th = std::thread(&Gl::ThreadCallBack,this);
}


Gl::~Gl()
{
    SetFrameDataEnable(false);

    if(comm_type==COMM_SERIAL)
    {
        serial_port_->close();
        delete serial_port_;
    }
    else if(comm_type==COMM_UDP) close(sockfd_); 

    thread_running = false;
    std::cout << "Socket END" << std::endl;
}


//////////////////////////////////////////////////////////////
// Functions for Comm
//////////////////////////////////////////////////////////////

void read_cs_update(uint8_t data)
{
    read_cs_ = read_cs_^ (data&0xff);
}

uint8_t read_cs_get()
{
    return read_cs_&0xff;
}

void read_cs_clear()
{
    read_cs_ = 0;
}

void write_cs_update(uint8_t data)
{
    write_cs_ = write_cs_^ (data&0xff);
}

uint8_t write_cs_get()
{
    return write_cs_&0xff;
}

void write_cs_clear()
{
    write_cs_ = 0;
}

void write(uint8_t data)
{
    send_packet.push_back(data);
    write_cs_update(data);
}

void write_PS()
{   
    std::vector<uint8_t> PS = {PS1, PS2, PS3, PS4};

    for(auto& i: PS) write(i);
}

void SendPacket(std::vector<uint8_t>& send_packet)
{
    for(auto& i: send_packet) serial_port_->write(&i, 1);
}

void write_packet(uint8_t PI, uint8_t PL, uint8_t SM, uint8_t CAT0, uint8_t CAT1, const std::vector<uint8_t>& DTn)
{
    if(comm_type==COMM_SERIAL) serial_port_->flush();
    send_packet.clear();
    write_cs_clear();

    write_PS();

    uint16_t TL = DTn.size() + 14;
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

    for(auto i: DTn) write(i);

    write(PE);

    write(write_cs_get());

    if(comm_type==COMM_SERIAL) SendPacket(send_packet);
    else if(comm_type==COMM_UDP) sendto(sockfd_, &send_packet[0], send_packet.size(), MSG_CONFIRM, (const struct sockaddr *) &servaddr_, sizeof(servaddr_));
}


void recv_packet_clear(void)
{
    read_cs_clear();
    recv_state = STATE_INIT;
    recv_packet.clear();
    recv_data.clear();
}


int check_PS(uint8_t data)
{
    if(recv_state==STATE_INIT)
    {
        if(data==PS1)
        {
            recv_packet_clear();
            read_cs_update(data);
            recv_state = STATE_PS1;
        }
    }
    else if(recv_state==STATE_PS1)
    {
        if(data==PS2)
        {
            read_cs_update(data);
            recv_state = STATE_PS2;
        }
        else return 0;
    }
    else if(recv_state==STATE_PS2)
    {
        if(data==PS3)
        {
            read_cs_update(data);
            recv_state = STATE_PS3;
        }
        else return 0;
    }
    else if(recv_state==STATE_PS3)
    {
        if(data==PS4)
        {
            read_cs_update(data);
            recv_state = STATE_PS4;
        }
        else return 0;
    }
    else return 2;

    return 1;
}

void ParsingFrameData(Gl::framedata_t& frame_data, std::vector<uint8_t>& data)
{
    if(data.size()==0) return;

    uint16_t frame_data_size = data[0]&0xff;
    frame_data_size |= ((uint16_t)(data[1]&0xff))<<8;

    if(data.size()!=(frame_data_size*4+22)) return;

    frame_data.distance.resize(frame_data_size);
    frame_data.pulse_width.resize(frame_data_size);
    frame_data.angle.resize(frame_data_size);
    for(size_t i=0; i<frame_data_size; i++)
    {
        uint16_t distance = data[i*4+2]&0xff;
        distance |= ((uint16_t)(data[i*4+3]&0xff))<<8;

        uint16_t pulse_width = data[i*4+4]&0xff;
        pulse_width |= ((uint16_t)(data[i*4+5]&0xff))<<8;

        frame_data.distance[i] = distance/1000.0;
        frame_data.pulse_width[i] = pulse_width;
        frame_data.angle[i] = i*180.0/(frame_data_size-1)*3.141592/180.0;
    }
}

void ParsingData(std::vector<uint8_t>& recv_data, int PI, int PL, int SM, int CAT0, int CAT1)
{
    // GetSerialNum()
    if(SM==SM_GET && CAT0==0x02 && CAT1==0x0A)
    {
        serial_num = recv_data;
    }
    // ReadFrameData()
    else if(SM==SM_STREAM && CAT0==0x01 && CAT1==0x02)
    {
        if(PI==0)
        {
            lidar_data = recv_data;
        }
        else
        {
            std::copy(recv_data.begin(), recv_data.end(), std::back_inserter(lidar_data));
        }

        if(PI==(PL-1))
        {
            Gl::framedata_t frame_data;
            ParsingFrameData(frame_data, lidar_data);
            frame_data_in = frame_data;
            lidar_data.clear();
        }
    }

}


void add_packet_element(uint8_t data)
{
    static int recv_PI = 0;
    static int recv_PL = 0;
    static int recv_SM = 0;
    static int recv_CAT0 = 0;
    static int recv_CAT1 = 0;
    static int recv_DTL = 0;

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
            recv_packet.push_back(data);
            read_cs_update(data);

            if(recv_packet.size()==6 and recv_packet[5]!=BI_GL3102PC)
            {
                std::vector<uint8_t> packet = recv_packet;
                recv_packet_clear();
                for(auto& v: packet) add_packet_element(v);
            }

            if(recv_packet.size()==8)
            {
                int TL = recv_packet[0]&0xff;
                TL = TL | ((recv_packet[1]&0xff)<<8);

                recv_PI = recv_packet[2]&0xff;
                recv_PL = recv_packet[3]&0xff;

                recv_SM = recv_packet[4]&0xff;
                recv_CAT0 = recv_packet[6]&0xff;
                recv_CAT1 = recv_packet[7]&0xff;
                    
                recv_DTL = TL - 14;
            }

            if(recv_packet.size()>8)
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
                    for(auto& v: packet) add_packet_element(v);
                }
            }
        }
        else if(recv_state==STATE_PE)
        {
            if(data==read_cs_get())
            {
                ParsingData(recv_data, recv_PI, recv_PL, recv_SM, recv_CAT0, recv_CAT1);
                recv_packet_clear();
            }
            else
            {
                std::vector<uint8_t> packet = recv_packet;
                recv_packet_clear();
                for(auto& v: packet) add_packet_element(v);
            }
        }
    }
}


void Gl::ThreadCallBack(void) 
{
    recv_packet_clear();
    std::vector<uint8_t> recv(2000);

    while(thread_running==true)
    {
        if(comm_type==COMM_SERIAL)
        {
            uint8_t data;
            while( serial_port_->read(&data,1)==1 ) add_packet_element(data);
        }
        else if(comm_type==COMM_UDP)
        {
            socklen_t len = sizeof(clientaddr_);
            int recv_len = recvfrom(sockfd_, &recv[0], recv.size(), MSG_WAITFORONE, (struct sockaddr *) &clientaddr_, &len);

            for(int i=0; i<recv_len; i++)
            {
                add_packet_element(recv[i]);
            }
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
    for(size_t i=0; i<50; i++)
    {
        write_packet(PI, PL, SM, CAT0, CAT1, DTn);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        if(serial_num.size()>0)
        {
            std::string out_str(serial_num.begin(), serial_num.end());
            return out_str;
        }
    }

    std::string out_str = "[ERROR] Serial Number is not received.";
    return out_str;
}

void Gl::ReadFrameData(Gl::framedata_t& frame_data, bool filter_on)
{
    if(frame_data_in.distance.size()==0) return;

    frame_data = frame_data_in;
    frame_data_in.angle.clear();
    frame_data_in.distance.clear();
    frame_data_in.pulse_width.clear();
    
    if(filter_on==true)
    {
        for(int i=0; i<(int)frame_data.distance.size()-1; i++)
        {
            if(frame_data.distance[i]>0.0 && frame_data.distance[i+1]>0.0)
            {
                double diff = (frame_data.distance[i]-frame_data.distance[i+1])/2.0;
                if(diff>0.01*frame_data.distance[i]) frame_data.distance[i] = 0.0;
                if(diff<-0.01*frame_data.distance[i]) frame_data.distance[i] = 0.0;
            }
        }
    }
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