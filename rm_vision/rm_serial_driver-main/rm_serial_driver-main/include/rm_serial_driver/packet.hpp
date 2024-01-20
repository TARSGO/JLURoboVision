// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0x5A;
  uint8_t camera_choice_ :1;
  uint8_t detect_color : 1;  // 0-red 1-blue
  bool reset_tracker : 1;
  uint8_t reserved : 5;
  // float ray_distance;
  float roll;
  float pitch;
  float yaw;
  uint16_t checksum = 0;                                   
} __attribute__((packed));

struct new_SendPacket //英雄用的
{
  uint8_t header = 0xA5;
  bool tracking : 1;
  uint8_t id : 3;          // 0-outpost 6-guard 7-base
  uint8_t armors_num : 3;  // 2-balance 3-outpost 4-normal
  uint8_t reserved : 1;
  float yaw;
  float pitch;
  uint16_t checksum = 0;
} __attribute__((packed));


inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const new_SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(new_SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(new_SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
