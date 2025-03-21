
#ifndef RHPARM_SERIAL__H
#define RHPARM_SERIAL__H
#include <hidapi/hidapi.h>
#include <sstream>
#include <vector>

#include "rhparm_drvr.hpp"

namespace rhparm
{
	class rhparm_serial: public rhparm_drvr
	{
		public:
			rhparm_serial();
			~rhparm_serial();

			bool open(const std::string &portname) override;
			void close() override;

			bool getJointPosition(int id, uint16_t &pos) override;
			bool setJointPosition(int id, uint16_t pos, uint16_t time) override;
			bool setManualModeAll(bool enable, int count) override;

		private:
			int fd_;
	};
}

#endif // RHPARM_SERIAL__H
