#ifndef TRIK_SENSORS_BITMAP_BUILDER_HPP_
#define TRIK_SENSORS_BITMAP_BUILDER_HPP_

#ifndef __cplusplus
#error C++-only header
#endif

#include <trik/sensors/cv_algorithms.hpp>

#include <cassert>
#include <cmath>
#include <vector>

#include "image.hpp"
#include <trik/sensors/video_format.h>

//#define HSV_CORRECTION

namespace trik {
namespace sensors {

static uint16_t s_hi2ho_bb[IMG_WIDTH];
static uint8_t s_metapixFillerShifter_bb[IMG_WIDTH];

class BitmapBuilderCvAlgorithm : public CvAlgorithm<VideoFormat::YUV422, VideoFormat::RGB565X> {
private:
  union U_Hsv8x3 {
    struct {
      uint8_t h;
      uint8_t s;
      uint8_t v;
      uint8_t none;
    } parts;

    uint32_t whole;
  };

  uint64_t m_detectRange;
  uint32_t m_detectExpected;

  ImageDesc m_inImageDesc;
  ImageDesc m_outImageDesc;

  uint32_t m_detectHueFrom;
  uint32_t m_detectHueTo;
  uint32_t m_detectSatFrom;
  uint32_t m_detectSatTo;
  uint32_t m_detectValFrom;
  uint32_t m_detectValTo;

  uint32_t m_detectHueTol;
  uint32_t m_detectSatTol;
  uint32_t m_detectValTol;

  static bool __attribute__((always_inline)) detectHsvPixel(const uint32_t _hsv, const uint64_t _hsv_range, const uint32_t _hsv_expect) {
    const uint32_t u32_hsv_det = _cmpltu4(_hsv, _hill(_hsv_range)) | _cmpgtu4(_hsv, _loll(_hsv_range));

    return (u32_hsv_det == _hsv_expect);
  }

  void resetHsvRange() {
    if (m_detectHueFrom <= m_detectHueTo) {
      m_detectRange = _itoll((m_detectValFrom << 16) | (m_detectSatFrom << 8) | m_detectHueFrom, (m_detectValTo << 16) | (m_detectSatTo << 8) | m_detectHueTo);
      m_detectExpected = 0x0;
    } else {
      assert(m_detectHueFrom > 0 && m_detectHueTo < 255);
      m_detectRange =
        _itoll((m_detectValFrom << 16) | (m_detectSatFrom << 8) | (m_detectHueTo + 1), (m_detectValTo << 16) | (m_detectSatTo << 8) | (m_detectHueFrom - 1));
      m_detectExpected = 0x1;
    }
  }

public:
  virtual bool setup(const ImageDesc& _inImageDesc, const ImageDesc& _outImageDesc, int8_t* _fastRam, size_t _fastRamSize) {
    m_inImageDesc = _inImageDesc;
    m_outImageDesc = _outImageDesc;

    if (m_inImageDesc.m_width % 32 != 0 || m_inImageDesc.m_height % 4 != 0)
      return false;

    // 0 0 0 0 320 320 320 320 640 640 640 640 ...
    uint16_t* p_hi2ho = s_hi2ho_bb;
    for (uint16_t i = 0; i < m_inImageDesc.m_height; i++)
      *(p_hi2ho++) = (i / METAPIX_SIZE) * m_outImageDesc.m_width;

    // (0 4 8 12) ...
    uint8_t* p_metapixFillerShifter = s_metapixFillerShifter_bb;
    for (uint16_t i = 0; i < m_inImageDesc.m_height; i++)
      *(p_metapixFillerShifter++) = (i % METAPIX_SIZE) * METAPIX_SIZE;

    return true;
  }

  virtual bool run(const ImageBuffer& _inImage, ImageBuffer& _outImage, const trik_cv_algorithm_in_args& _inArgs, trik_cv_algorithm_out_args& _outArgs) {
    // /if (_inArgs.setHsvRange) {

    m_detectHueFrom = range<int16_t>(0, (_inArgs.detect_hue_from * 255) / 359, 255); // scaling 0..359 to 0..255
    m_detectHueTo = range<int16_t>(0, (_inArgs.detect_hue_to * 255) / 359, 255);     // scaling 0..359 to 0..255
    m_detectSatFrom = range<int16_t>(0, (_inArgs.detect_sat_from * 255) / 100, 255); // scaling 0..100 to 0..255
    m_detectSatTo = range<int16_t>(0, (_inArgs.detect_sat_to * 255) / 100, 255);     // scaling 0..100 to 0..255
    m_detectValFrom = range<int16_t>(0, (_inArgs.detect_val_from * 255) / 100, 255); // scaling 0..100 to 0..255
    m_detectValTo = range<int16_t>(0, (_inArgs.detect_val_to * 255) / 100, 255);     // scaling 0..100 to 0..255

    resetHsvRange();
    // }

    const uint64_t u64_hsv_range = m_detectRange;
    const uint32_t u32_hsv_expect = m_detectExpected;

    /*
          METAPIX:
          15 14 12 12 11 10 9 8 7 6 5 4 3 2 1 0:
          -------------
          |0 |1 |2 |3 |
          |--|--|--|--|
          |4 |5 |6 |7 |
          |--|--|--|--|
          |8 |9 |10|11|
          |--|--|--|--|
          |12|13|14|15|
          -------------
    */
    const uint64_t* restrict p_inImg = reinterpret_cast<const uint64_t*>(_inImage.m_ptr);
    const uint16_t* restrict p_hi2ho = s_hi2ho_bb;

    int64_t detectedPoints = 0;
#ifdef HSV_CORRECTION
    uint32_t midH[256];
    uint32_t midS[256];
    memset(midH, 0, 256 * sizeof(uint32_t));
    memset(midS, 0, 256 * sizeof(uint32_t));
#endif

    // just detect and build metapixels:
    uint8_t* restrict p_metapixFillerShifter = s_metapixFillerShifter_bb;
    uint8_t metapixFiller = 0;
    U_Hsv8x3 pixel;
#pragma MUST_ITERATE(4, , 4)
    for (uint16_t srcRow = 0; srcRow < m_inImageDesc.m_height; srcRow++) {
      uint16_t* restrict p_outImg = reinterpret_cast<uint16_t*>(_outImage.m_ptr) + *(p_hi2ho++);
      const uint16_t metapixFillerShifter = *(p_metapixFillerShifter++); //(0 4 8 12)...

#pragma MUST_ITERATE(32, , 32)
      for (uint16_t srcCol = 0; srcCol < m_inImageDesc.m_width; srcCol++) {
        pixel.whole = _loll(*(p_inImg++));
        bool det = detectHsvPixel(pixel.whole, u64_hsv_range, u32_hsv_expect);

#ifdef HSV_CORRECTION
        if (det) {
          midH[pixel.parts.h]++;
          midS[pixel.parts.s]++;
          detectedPoints++;
        }
#endif
        *p_outImg += det << (metapixFillerShifter + metapixFiller++);

        if (metapixFiller == METAPIX_SIZE) {
          p_outImg++;
          metapixFiller = 0;
        }
      }
    }

#ifdef HSV_CORRECTION
    int maxHid = 0;
    int maxSid = 0;
    int maxH = 0;
    int maxS = 0;

    for (int i = 0; i < 256; i++) {
      if (midH[i] >= maxH) {
        maxHid = i;
        maxH = midH[i];
      }
      if (midS[i] >= maxS) {
        maxSid = i;
        maxS = midS[i];
      }
    }

    m_detectHueTol = (m_detectHueFrom + m_detectHueTo) / 2 - m_detectHueTo;
    m_detectSatTol = (m_detectSatFrom + m_detectSatTo) / 2 - m_detectSatTo;
    m_detectValTol = (m_detectValFrom + m_detectValTo) / 2 - m_detectValTo;

    if (detectedPoints > 32) {

      m_detectHueFrom = makeValueWrap(maxHid, -m_detectHueTol, 0, 255);
      m_detectHueTo = makeValueWrap(maxHid, +m_detectHueTol, 0, 255);
      m_detectSatFrom = makeValueRange(maxSid, -m_detectSatTol, 0, 255);
      m_detectSatTo = makeValueRange(maxSid, +m_detectSatTol, 0, 255);

      resetHsvRange();
    }
#endif
    return true;
  }
};

}

}

#endif
