#ifndef TRIK_SENSORS_IMAGE_HPP_
#define TRIK_SENSORS_IMAGE_HPP_

#ifndef __cplusplus
#error C++-only header
#endif

#include <trik/sensors/video_format.h>
#include <cstddef>
#include <stdint.h>

namespace trik {
namespace sensors {

static const size_t FastRamSize = 0x1000;

struct ImageDesc {
  uint16_t m_width;
  uint16_t m_height;
  uint32_t m_lineLength;
  VideoFormat m_format;
};

struct ImageBuffer {
  int8_t* restrict m_ptr;
  size_t m_size;
};

}
}

#endif // !TRIK_VIDTRANSCODE_CV_INTERNAL_VIDTRANSCODE_CV_H_
