#ifndef SERIAL_SERVO_BUS__H
#define SERIAL_SERVO_BUS__H

#include <cstdint>
#include <vector>
#include <map>

bool BusController_ServoMove(int fd, uint8_t id, uint16_t position, uint16_t time);
bool BusController_ServoUnload(int fd, const uint8_t ids[], int count);
bool BusController_ReadPosition(int fd, uint8_t id, uint16_t &position);
bool BusController_ReadMultiplePositions(int fd, const std::vector<uint8_t>& ids, std::map<uint8_t, uint16_t>& positions);

#endif // SERIAL_SERVO_BUS__H
