#ifndef IMAGE_IO_PIXEL_H
#define IMAGE_IO_PIXEL_H

#include <cstdint>
#include <type_traits>
#include <ento-util/debug.h>

template <int BitDepth, bool IsPacked = false, int StartBit = 1, int StopBit = 0>
struct PixelType;

// Specializations for unpacked pixels
template <int BitDepth>
struct PixelType<BitDepth, false>
{
  using type = typename std::conditional<BitDepth <= 8, uint8_t, uint16_t>::type;
};

template <int BitDepth, int StartBit, int StopBit>
struct PixelType<BitDepth, true, StartBit, StopBit>
{
  static constexpr int TotalBitSize = BitDepth + 2;
  using PacketType = typename std::conditional<
    TotalBitSize <= 8, uint8_t, typename std::conditional<
    TotalBitSize <= 16, uint16_t, uint32_t>::type>::type;

  using PixelUnderlyingType = typename std::conditional<BitDepth <= 8, uint8_t, uint16_t>::type;

  class PackedPixel
  {
  public:
    using PacketType = typename PixelType<BitDepth, true, StartBit, StopBit>::PacketType;  // Define PacketType inside PackedPixel

    PackedPixel(PixelUnderlyingType value = 0) 
    {
      set_pixel(value);
    }

    void set_pixel(PixelUnderlyingType value)
    {
      packet = (static_cast<PacketType>(StartBit) << (BitDepth + 1))
        | (static_cast<PacketType>(value) << 1)
        | (static_cast<PacketType>(StopBit));
    }

    template <int UserBitDepth>
    typename std::conditional<UserBitDepth <= 8, uint8_t, uint16_t>::type get_pixel() const
    {
      using PixelReturnType =
        typename std::conditional<UserBitDepth <= 8, uint8_t, uint16_t>::type;
      constexpr int mask = (1 << UserBitDepth) - 1;
      constexpr int shift = BitDepth - UserBitDepth + 1;
      return static_cast<PixelReturnType>((packet >> shift) & mask);
    }

    PacketType get_packet() const
    {
      return packet;
    }

  private:
    PacketType packet;
  };
  using type = PackedPixel;
};

template <int BitDepth>
using Pixel = typename PixelType<BitDepth, false>::type;

// Alias for packed pixels with start/stop bits
template <int BitDepth, int StartBit = 1, int StopBit = 0>
using PackedPixel = typename PixelType<BitDepth, true, StartBit, StopBit>::type;

#endif // IMAGE_IO_PIXEL_H
