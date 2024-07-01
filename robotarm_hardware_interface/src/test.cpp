#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "robotarm_hardware_interface/robotarm_serial.hpp"
#include "robotarm_hardware_interface/robotarm_drvr.hpp"


const int NUM_JOINTS = 7;

const std::string SERIAL_DEV = "/dev/ttyUSB0";

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    int i;
    std::unique_ptr<robotarm::robotarm_drvr> drvr_ = std::make_unique<robotarm::robotarm_serial>();
    uint16_t pos;

    if (!drvr_->open(SERIAL_DEV)) {
        std::cerr << "Failed to open driver";
        return false;
    }

    for (i = 0; i < 1000; i = i + 1){
        drvr_->setJointPosition(2, i, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }

	if (!drvr_->setJointPosition(2, 200, 4000)) {
        std::cerr << "Failed to set servo position" << std::endl;
    } else {
        std::cerr << "Set joint position" << std::endl;
    }



    for (i = 0; i < 5*2; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        if (drvr_->getJointPosition(1, pos)) {
            std::cout << "servo " << i << ", pos: " << pos << std::endl;
        } else {
            std::cerr << "Failed to read servo position" << std::endl;
        }
    }

    if (!drvr_->setJointPosition(1, 800, 2000)) {
        std::cerr << "Failed to set servo position" << std::endl;
    } else {
        std::cerr << "Set joint position" << std::endl;
    }

    for (i = 0; i < 5*2; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200) );

        if (drvr_->getJointPosition(1, pos)) {
            std::cout << "servo " << i << ", pos: " << pos << std::endl;
        } else {
            std::cerr << "Failed to read servo position" << std::endl;
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(3000) );

    if (drvr_->getJointPosition(1, pos)) {
        std::cout << "servo " << i << ", pos: " << pos << std::endl;
    } else {
        std::cerr << "Failed to read servo position" << std::endl;
    }

    std::cerr << "Try moving it by hand - should be free to move" << std::endl;
	drvr_->setManualModeAll(true, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10000) );

    std::cerr << "Try moving it by hand - should be holding position" << std::endl;
	drvr_->setManualModeAll(false, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10000) );

    std::cerr << "Bye" << std::endl;
    rclcpp::shutdown();
    return 0;
}
