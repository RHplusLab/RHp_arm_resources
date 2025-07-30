// Serial servo bus control for Hiwonder Bus Servo Controller

#include <stdexcept>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <chrono>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "rhparm_hardware_interface/rhparm_serial.hpp"
#include "rhparm_hardware_interface/serial_servo_bus.hpp"

static int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        fprintf(stderr, "Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

static void set_mincount(int fd, int mcount, int to_x100ms)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        fprintf(stderr, "Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = to_x100ms;

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        fprintf(stderr, "Error tcsetattr: %s\n", strerror(errno));
}

namespace rhparm
{
	rhparm_serial::rhparm_serial():
		rhparm_drvr(),
		fd_(-1)
	{
	}

	rhparm_serial::~rhparm_serial()
	{
		close();
	}

	bool rhparm_serial::open(const std::string &portname)
	{
		if (fd_ > 0) {
			return false;
		}

		fd_ = ::open(portname.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    	if (fd_ < 0) {
			RCLCPP_ERROR(rclcpp::get_logger("RHPArmSystemHardware"), "rhpArm, unable to open serial port [%s]",
				strerror(errno));
			return false;
		}

	    set_interface_attribs(fd_, B9600);
		set_mincount(fd_, 0, 5);
		RCLCPP_INFO(rclcpp::get_logger("RHPArmSystemHardware"), "rhpArm device opened ");
		return true;
	}

	void rhparm_serial::close()
	{
		if (fd_ > 0) {
			::close(fd_);
			fd_ = -1;
		}
	}

	bool rhparm_serial::getJointPosition(int id, uint16_t &pos)
	{
		if (!BusController_ReadPosition(fd_, id, pos)) {
			RCLCPP_ERROR(rclcpp::get_logger("RHPArmSystemHardware"), "Failed to read servo %d position", id);
			return false;
		}
		if(id == 2){ pos = pos + 7; }
		else if(id == 3){ pos = pos - 7; }
		else if(id == 5){ pos = pos - 3; }
		return true;
	}

	bool rhparm_serial::getMultipleJointPositions(const std::vector<uint8_t>& ids, std::map<uint8_t, uint16_t>& positions)
	{
		return BusController_ReadMultiplePositions(fd_, ids, positions);
	}

	bool rhparm_serial::setJointPosition(int id, uint16_t pos, uint16_t time)
	{
		if (!BusController_ServoMove(fd_, id, pos, time)) {
			RCLCPP_ERROR(rclcpp::get_logger("RHPArmSystemHardware"), "Failed to set servo %u position", id);
			return false;
		}
		return true;
	}

	bool rhparm_serial::setManualModeAll(bool enable, int count)
	{
		bool bOk = true;
		if (enable) {
			uint8_t* ids = new uint8_t[count];
			for(int i = 0; i < count; i++) {
				ids[i] = i + 1;
			}
			bOk = BusController_ServoUnload(fd_, ids, count);
			delete[] ids;

			if (!bOk) {
				RCLCPP_ERROR(rclcpp::get_logger("RHPArmSystemHardware"), "Failed to unload servos");
			}
		}
		return bOk;
	}
}
