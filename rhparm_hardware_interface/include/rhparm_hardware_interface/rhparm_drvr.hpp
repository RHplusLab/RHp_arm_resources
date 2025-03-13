
#ifndef RHPARM_DRVR__H
#define RHPARM_DRVR__H
#include <hidapi/hidapi.h>
#include <sstream>
#include <vector>

namespace rhparm
{
	class rhparm_drvr
	{
		public:
			rhparm_drvr() {};
			virtual ~rhparm_drvr() {};

			virtual bool open(const std::string &portname) = 0;
			virtual void close() = 0;

			virtual bool getJointPosition(int id, uint16_t &pos) = 0;
			virtual bool setJointPosition(int id, uint16_t pos, uint16_t time) = 0;
			virtual bool setManualModeAll(bool enable, int count) = 0;
	};
}

#endif // RHPARM_DRVR__H
