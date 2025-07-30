// Serial servo bus control for Hiwonder Bus Servo Controller

#include <iostream>
#include <cstdint>
#include <chrono>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <map>

#include "rhparm_hardware_interface/serial_servo_bus.hpp"

// 매크로 함수들은 그대로 사용
#define GET_LOW_uint8_t(A) (uint8_t)((A))
#define GET_HIGH_uint8_t(A) (uint8_t)((A) >> 8)
#define uint8_t_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))

// 컨트롤러 프로토콜에 맞는 상수들 새로 정의
#define FRAME_HEADER 0x55
#define CMD_SERVO_MOVE 3
#define CMD_MULT_SERVO_UNLOAD 20
#define CMD_MULT_SERVO_POS_READ 21

static bool writeMsg(int fd, const uint8_t *buf, int len) {
    if (len != write(fd, buf, len)) {
        return false;
    }
    tcdrain(fd);
    return true;
}

bool BusController_ServoMove(int fd, uint8_t id, uint16_t position, uint16_t time) {
    const int num_servos = 1;
    const int msgLen = 10;
    uint8_t buf[msgLen];

    buf[0] = FRAME_HEADER;
    buf[1] = FRAME_HEADER;
    buf[2] = 8;
    buf[3] = CMD_SERVO_MOVE;
    buf[4] = num_servos;
    buf[5] = GET_LOW_uint8_t(time);
    buf[6] = GET_HIGH_uint8_t(time);
    buf[7] = id;
    buf[8] = GET_LOW_uint8_t(position);
    buf[9] = GET_HIGH_uint8_t(position);

    return writeMsg(fd, buf, msgLen);
}

bool BusController_ServoUnload(int fd, const uint8_t ids[], int count) {
    if (count <= 0) return true;
    const int msgLen = 5 + count;
    uint8_t* buf = new uint8_t[msgLen];

    buf[0] = FRAME_HEADER;
    buf[1] = FRAME_HEADER;
    buf[2] = 3 + count;
    buf[3] = CMD_MULT_SERVO_UNLOAD;
    buf[4] = count;

    for (int i = 0; i < count; ++i) {
        buf[5 + i] = ids[i];
    }

    bool result = writeMsg(fd, buf, msgLen);
    delete[] buf;
    return result;
}

static bool ReceiveResponse(int fd, uint8_t cmd, std::vector<uint8_t> &resp_data) {
    uint8_t header_count = 0;
    uint8_t rx_byte;

    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count() < 200) {
        int n = read(fd, &rx_byte, 1);
        if (n < 1) {
            continue;
        }

        if (header_count < 2) {
            if (rx_byte == FRAME_HEADER) {
                header_count++;
            } else {
                header_count = 0;
            }
            continue;
        }

        uint8_t length = rx_byte;
        uint8_t resp_cmd;
        if (read(fd, &resp_cmd, 1) < 1) return false;

        if (resp_cmd != cmd) {
            header_count = 0;
            continue;
        }

        int data_len_to_read = length - 2;
        if (data_len_to_read > 0) {
            resp_data.resize(data_len_to_read);
            int bytes_read = 0;
            auto read_start_time = std::chrono::steady_clock::now();
            while(bytes_read < data_len_to_read && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - read_start_time).count() < 50) {
                int n_read = read(fd, resp_data.data() + bytes_read, data_len_to_read - bytes_read);
                if (n_read > 0) {
                    bytes_read += n_read;
                }
            }
            if (bytes_read != data_len_to_read) return false;
        }
        return true;
    }
    return false;
}

bool BusController_ReadPosition(int fd, uint8_t id, uint16_t &position) {
    tcflush(fd, TCIFLUSH);
    const int msgLen = 6;
    uint8_t buf[msgLen];

    buf[0] = FRAME_HEADER;
    buf[1] = FRAME_HEADER;
    buf[2] = 4;
    buf[3] = CMD_MULT_SERVO_POS_READ;
    buf[4] = 1;
    buf[5] = id;

    if (!writeMsg(fd, buf, msgLen)) {
        return false;
    }

    usleep(20000);

    std::vector<uint8_t> resp_data;
    if (ReceiveResponse(fd, CMD_MULT_SERVO_POS_READ, resp_data)) {
        if (resp_data.size() >= 4 && resp_data[0] == 1 && resp_data[1] == id) {
            position = uint8_t_TO_HW(resp_data[3], resp_data[2]);
            return true;
        }
    }
    return false;
}

bool BusController_ReadMultiplePositions(int fd, const std::vector<uint8_t>& ids, std::map<uint8_t, uint16_t>& positions) {
    if (ids.empty()) {
        return true;
    }

    tcflush(fd, TCIFLUSH);
    const int count = ids.size();
    const int msgLen = 5 + count;
    uint8_t* buf = new uint8_t[msgLen];

    buf[0] = FRAME_HEADER;
    buf[1] = FRAME_HEADER;
    buf[2] = 3 + count;
    buf[3] = CMD_MULT_SERVO_POS_READ;
    buf[4] = count;
    for (int i = 0; i < count; ++i) {
        buf[5 + i] = ids[i];
    }

    if (!writeMsg(fd, buf, msgLen)) {
        delete[] buf;
        return false;
    }
    delete[] buf;

    usleep(10000);

    std::vector<uint8_t> resp_data;
    if (ReceiveResponse(fd, CMD_MULT_SERVO_POS_READ, resp_data)) {
        if (resp_data.size() >= (size_t)(1 + count * 3) && resp_data[0] == count) {
            for (int i = 0; i < count; ++i) {
                uint8_t id = resp_data[1 + i * 3];
                uint16_t pos = uint8_t_TO_HW(resp_data[1 + i * 3 + 2], resp_data[1 + i * 3 + 1]);
                positions[id] = pos;
            }
            return true;
        }
    }
    return false;
}
