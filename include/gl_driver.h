
#ifndef GLDRIVER_H
#define GLDRIVER_H

#include <stdio.h> 
#include <unistd.h> 
#include <stdlib.h>
#include <string.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include <math.h>

#include "serial/serial.h"


class Gl
{
	public:
		typedef struct _FRAMEDATA_T
		{
			std::vector<double> angle;
			std::vector<double> distance;
			std::vector<double> pulse_width;
		} framedata_t;

	public:
		Gl(std::string &port, uint32_t baudrate);
		~Gl();

		std::string GetSerialNum(void);
		framedata_t ReadFrameData(void);
		void SetFrameDataEnable(uint8_t framedata_enable);
};

#endif
