
#ifndef GLDRIVER_H
#define GLDRIVER_H


#include "serial/serial.h"


class Gl
{
	public:
		typedef struct _FRAMEDATA_T
		{
			std::vector<double> angle;
			std::vector<int> distance;
			std::vector<int> pulse_width;
		} framedata_t;

	public:
		Gl(std::string &port, uint32_t baudrate);
		~Gl();

		std::string GetSerialNum(void);
		framedata_t ReadFrameData(void);
		void SetFrameDataEnable(bool framedata_enable);
};

#endif
