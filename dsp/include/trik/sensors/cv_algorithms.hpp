#ifndef TRIK_SENSORS_CV_ALGORITHMS_HPP_
#define TRIK_SENSORS_CV_ALGORITHMS_HPP_

#ifndef __cplusplus
#error C++-only header
#endif

#include "image.hpp"
#include "video_format.hpp"
#include <stdint.h>
#include <trik/sensors/cv_algorithm_args.h>

namespace trik {
namespace sensors {

template <typename _T>
inline _T range(_T _min, _T _val, _T _max) {
  if (_val < _min)
    return _min;
  else if (_val > _max)
    return _max;
  else
    return _val;
}

inline uint16_t pop(uint16_t x) {
  x = x - ((x >> 1) & 0x5555);
  x = (x & 0x3333) + ((x >> 2) & 0x3333);
  x = (x + (x >> 4)) & 0x0f0f;
  x = x + (x >> 8);
  x = x + (x >> 16);

  return x & 0x003f;
}

inline int makeValueRange(int _val, int _adj, int _min, int _max) {
  _val += _adj;
  if (_val > _max)
    return _max;
  else if (_val < _min)
    return _min;
  else
    return _val;
}

inline int makeValueWrap(int _val, int _adj, int _min, int _max) {
  _val += _adj;
  while (_val > _max)
    _val -= (_max - _min + 1);
  while (_val < _min)
    _val += (_max - _min + 1);

  return _val;
}

template <VideoFormat _inFormat, VideoFormat _outFormat>
class CvAlgorithm {
public:
  // CvAlgorithm(const CvAlgorithm& another) = delete;
  // CvAlgorithm& operator=(const CvAlgorithm& another) = delete;

  virtual bool setup(const ImageDesc& _inImageDesc, const ImageDesc& _outImageDesc, int8_t* _fastRam, size_t _fastRamSize) = 0;
  virtual bool run(const ImageBuffer& _inImage, ImageBuffer& _outImage, const trik_cv_algorithm_in_args& in_args, trik_cv_algorithm_out_args& out_args) = 0;

  virtual ~CvAlgorithm() {}

protected:
  CvAlgorithm() {}
};

}
}

#include "edge_line_sensor.hpp"
#include "motion_sensor.hpp"
#include "object_sensor.hpp"

#endif
