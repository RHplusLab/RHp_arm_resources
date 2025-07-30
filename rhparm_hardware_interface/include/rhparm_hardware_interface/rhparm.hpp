#ifndef RHPARM__H
#define RHPARM__H
#include <hidapi/hidapi.h>
#include <sstream>
#include <map>
#include <vector>
#include <string>
#include <memory>

#include "rhparm_hardware_interface/rhparm_drvr.hpp"

namespace rhparm
{
	class rhparm
	{
		struct JointRangeLimits {
			float range_rad;
			int min;
			int max;
			int mid;
			int invert_factor;
		};

		public:
			rhparm();
			~rhparm();

			bool init();

			void setAllJointPositions(const std::vector<double> &commands, const std::vector<std::string> &joints);
			void getAllJointPositions(std::vector<double> &positions, const std::vector<std::string> &joints);

		private:
			double jointValueToPosition(std::string joint_name, int jointValue);
			int positionToJointValue(std::string joint_name, double position);

			double convertUnitToRad(std::string joint_name, int unit);
			int convertRadToUnit(std::string joint_name, double rad);

			bool inited_;
			std::unique_ptr<rhparm_drvr> drvr_;

			std::map<std::string, int> joint_name_map_;
			std::map<std::string, struct JointRangeLimits> joint_range_limits_;

            // [추가] 쓰기/읽기 충돌 방지를 위한 변수들
            bool is_first_write_;
            std::vector<double> last_commanded_positions_;
            std::vector<double> last_read_positions_;
	};
}

#endif
