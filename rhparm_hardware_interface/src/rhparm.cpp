#include <stdexcept>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <chrono>
#include <math.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rhparm_hardware_interface/rhparm.hpp"
#include "rhparm_hardware_interface/rhparm_serial.hpp"

#define A 0.026000
#define B 0.031000
#define C 0.010048
#define D 0.001552

const float RAD_RANGE = (240.0 / 180.0) * M_PI;
const int MOVE_TIME_MS = 20;
const int FIRST_SET_MOVE_TIME = 1000;
const std::string SERIAL_DEV = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0";

namespace rhparm
{
	rhparm::rhparm() : inited_(false), is_first_write_(true)
	{
	}

	rhparm::~rhparm()
	{
		if (drvr_)
		{
			drvr_->close();
		}
	}

	bool rhparm::init()
	{
		if (inited_)
		{
			return false;
		}

		std::string dev;
		drvr_ = std::make_unique<rhparm_serial>();
		dev = SERIAL_DEV;

		if (!drvr_ || !drvr_->open(dev))
		{
			RCLCPP_ERROR(rclcpp::get_logger("RHPArmSystemHardware"), "Failed to open driver");
			return false;
		}

		joint_name_map_.insert(std::make_pair("revolute_1", 1));
		joint_name_map_.insert(std::make_pair("revolute_2", 2));
		joint_name_map_.insert(std::make_pair("revolute_3", 3));
		joint_name_map_.insert(std::make_pair("revolute_4", 4));
		joint_name_map_.insert(std::make_pair("revolute_5", 5));
		joint_name_map_.insert(std::make_pair("revolute_6", 6));
		joint_name_map_.insert(std::make_pair("slider_1", 7));

		joint_range_limits_["revolute_1"] = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["revolute_2"] = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["revolute_3"] = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["revolute_4"] = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["revolute_5"] = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["revolute_6"] = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["slider_1"] = {RAD_RANGE, 0, 1000, 500, 1};

        last_read_positions_.resize(joint_name_map_.size(), 0.0);

		inited_ = true;
		return true;
	}

	void rhparm::setAllJointPositions(const std::vector<double> &commands, const std::vector<std::string> &joints)
	{
		if (std::isfinite(commands[0]))
		{
			for (uint i = 0; i < commands.size(); i++)
			{
				const std::string &name = joints[i];
				int joint_pos = positionToJointValue(name, commands[i]);
				drvr_->setJointPosition(joint_name_map_[name], joint_pos, is_first_write_ ? FIRST_SET_MOVE_TIME : MOVE_TIME_MS);
			}
			if(is_first_write_) is_first_write_ = false;
		}
	}

	// [수정됨] 항상 실제 하드웨어 값을 읽어오도록 로직 수정
	void rhparm::getAllJointPositions(std::vector<double> &positions, const std::vector<std::string> &joints)
	{
		positions.clear();
		std::vector<uint8_t> ids_to_read;
		for(const auto& joint_name : joints) {
			ids_to_read.push_back(joint_name_map_[joint_name]);
		}

		std::map<uint8_t, uint16_t> servo_positions;
		if (!drvr_->getMultipleJointPositions(ids_to_read, servo_positions)) {
			RCLCPP_WARN(rclcpp::get_logger("RHPArmSystemHardware"), "Failed to read all joint positions, returning last known values.");
			positions = last_read_positions_; // 읽기 실패 시, 마지막으로 성공했던 값 반환
			return;
		}

		// 읽기 성공 시, 새 값을 positions 벡터에 채움
		for(const auto& joint_name : joints) {
			uint8_t id = joint_name_map_[joint_name];
			double pos_rad = 0.0;
			if(servo_positions.count(id)) {
				pos_rad = jointValueToPosition(joint_name, servo_positions[id]);
			} else {
                 // 응답에 특정 조인트 값이 누락된 경우
                RCLCPP_WARN(rclcpp::get_logger("RHPArmSystemHardware"), "No position data for joint %s", joint_name.c_str());
            }
			positions.push_back(pos_rad);
		}
        last_read_positions_ = positions; // 성공적으로 읽은 값을 다음을 위해 저장
	}

	int rhparm::convertRadToUnit(std::string joint_name, double rad)
	{
		double range = joint_range_limits_[joint_name].max - joint_range_limits_[joint_name].min;
		double b = joint_range_limits_[joint_name].mid;
		return (range * rad / joint_range_limits_[joint_name].range_rad * joint_range_limits_[joint_name].invert_factor) + b;
	}

	double rhparm::convertUnitToRad(std::string joint_name, int unit)
	{
		double range = joint_range_limits_[joint_name].max - joint_range_limits_[joint_name].min;
		double b = joint_range_limits_[joint_name].mid;
		return (unit - b) * joint_range_limits_[joint_name].range_rad * joint_range_limits_[joint_name].invert_factor / range;
	}

	double rhparm::jointValueToPosition(std::string joint_name, int jointValue)
	{
		double position = 0.0;
		if (joint_name == "slider_1")
		{
			double angle = convertUnitToRad(joint_name, jointValue);
			position = std::sqrt(B * B - (A * std::cos(angle) - C) * (A * std::cos(angle) - C)) + A * std::sin(angle) - std::sqrt(B * B - (A - C) * (A - C)) + D;
		}
		else
		{
			position = convertUnitToRad(joint_name, jointValue);
		}
		return position;
	}

	int rhparm::positionToJointValue(std::string joint_name, double position)
	{
		int jointValue = 0;
		if (joint_name == "slider_1")
		{
			double y = position + std::sqrt(B * B - (A - C) * (A - C)) - D;
			double angle = std::asin((y * y + A * A + C * C - B * B) / (2 * A * std::sqrt(y * y + C * C))) -
						   std::asin(C / std::sqrt(y * y + C * C));
			jointValue = int(convertRadToUnit(joint_name, angle));
		}
		else
		{
			jointValue = int(convertRadToUnit(joint_name, position));
		}
		return jointValue;
	}
}
