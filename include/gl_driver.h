
#ifndef GLDRIVER_H
#define GLDRIVER_H

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
	Gl(std::string &port, uint32_t baudrate);
	Gl();
	~Gl();

	void OpenSerial(std::string &port, uint32_t baudrate);

	std::string GetSerialNum(void);
	framedata_t ReadFrameData(void);
	void SetFrameDataEnable(uint8_t framedata_enable);

private:
	void ThreadCallBack(void);
	bool thread_running = true;
	std::thread th;
};

#endif