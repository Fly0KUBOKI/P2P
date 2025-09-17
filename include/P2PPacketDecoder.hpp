#ifndef P2P_PACKET_DECODER_HPP
#define P2P_PACKET_DECODER_HPP

#include <array>
#include <cstdint>
#include <cstring>

#include "PacketDataType.hpp"
#include "PacketUtility.hpp"

struct P2PDecodedPacket {
    std::array<uint8_t, 256> data{};
    uint8_t current_packet_size = 0;
};

class P2PPacketDecoder {
public:
    P2PPacketDecoder() = default;

    // Set raw received buffer (caller provides pointer to received bytes and size)
    PacketError SetData(const uint8_t* buffer, uint8_t size);

    // Get typed data by passing the enum and reference to variable of correct type
    PacketError GetData(PacketDataType type, float &out);
    PacketError GetData(PacketDataType type, uint32_t &out);
    PacketError GetData(PacketDataType type, int32_t &out);

private:
    P2PDecodedPacket packet;
    bool has_data_ = false;
};

#endif // P2P_PACKET_DECODER_HPP
