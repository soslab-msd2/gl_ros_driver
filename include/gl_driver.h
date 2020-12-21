
#ifndef GLDRIVERUDP_H
#define GLDRIVERUDP_H

#include <stdio.h> 
#include <unistd.h> 
#include <stdlib.h>
#include <string.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>
#include <vector>
#include <math.h>

#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 

#include "serial/serial.h"


class Gl
{
public:
	struct framedata_t
	{
		std::vector<double> angle;
		std::vector<double> distance;
		std::vector<double> pulse_width;
	};

public:
	Gl(std::string& gl_ip, int gl_port, int pc_port);
	Gl(std::string& gl_serial_name, uint32_t gl_serial_baudrate);
	~Gl();

	std::string GetSerialNum(void);
	void ReadFrameData(Gl::framedata_t& frame_data, bool filter_on=true);
	void SetFrameDataEnable(uint8_t framedata_enable);

private:
	void ThreadCallBack(void);
	bool thread_running = true;
	std::thread th;
};

#endif