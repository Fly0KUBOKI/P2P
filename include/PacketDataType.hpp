#ifndef PACKET_DATA_TYPE_HPP
#define PACKET_DATA_TYPE_HPP

#include <cstdint>

// 単一の enum class にまとめた PacketDataType
enum class PacketDataType : uint8_t {
  TYPE_PITCH = 0x00,
  TYPE_ROLL  = 0x01,
  TYPE_YAW   = 0x02,
  TYPE_THR   = 0x03
};

#endif // PACKET_DATA_TYPE_HPP
