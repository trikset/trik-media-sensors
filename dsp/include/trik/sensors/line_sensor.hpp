#ifndef TRIK_SENSORS_LINE_SENSOR_HPP_
#define TRIK_SENSORS_LINE_SENSOR_HPP_

#ifndef __cplusplus
#error C++-only header
#endif

#include <trik/sensors/cv_algorithms.hpp>

#include <c6x.h>
#include <cassert>
#include <cmath>

#include "hsv_range_detector.hpp"

namespace trik {
namespace sensors {

static uint64_t s_rgb888hsv_l[320 * 240];
static uint32_t s_wi2wo_l[320];
static uint32_t s_hi2ho_l[240];

class LineSensorCvAlgorithm : public CvAlgorithm<VideoFormat::YUV422P, VideoFormat::RGB565X> {
private:
  uint64_t m_detectRange;
  uint32_t m_detectExpected;

  uint32_t m_hStart;
  uint32_t m_hStop;
  uint32_t m_crossPoints;
  int8_t m_crossDetected;

  const int m_imageScaleCoeff = 1;
  int m_inImageFirstRow;

  int32_t m_targetX;
  int32_t m_targetY;
  uint32_t m_targetPoints;

  ImageDesc m_inImageDesc;
  ImageDesc m_outImageDesc;

  static uint16_t* restrict s_mult43_div;  // allocated from fast ram
  static uint16_t* restrict s_mult255_div; // allocated from fast ram

  static void __attribute__((always_inline)) writeOutputPixel(uint16_t* restrict _rgb565ptr, const uint32_t _rgb888) {
    *_rgb565ptr = ((_rgb888 >> 3) & 0x001f) | ((_rgb888 >> 5) & 0x07e0) | ((_rgb888 >> 8) & 0xf800);
  }

  void __attribute__((always_inline)) drawOutputPixelBound(const int32_t _srcCol, const int32_t _srcRow, const int32_t _srcColBot, const int32_t _srcColTop,
    const int32_t _srcRowBot, const int32_t _srcRowTop, const ImageBuffer& _outImage, const uint32_t _rgb888) const {
    const int32_t srcCol = range<int32_t>(_srcColBot, _srcCol, _srcColTop);
    const int32_t srcRow = range<int32_t>(_srcRowBot, _srcRow, _srcRowTop);

    const int32_t dstRow = s_hi2ho_l[srcRow];
    const int32_t dstCol = s_wi2wo_l[srcCol];

    const uint32_t dstOfs = dstRow * m_outImageDesc.m_lineLength + dstCol * sizeof(uint16_t);

    writeOutputPixel(reinterpret_cast<uint16_t*>(_outImage.m_ptr + dstOfs), _rgb888);
  }

  void __attribute__((always_inline))
  drawRgbTargetCenterLine(const int32_t _srcCol, const int32_t _srcRow, const ImageBuffer& _outImage, const uint32_t _rgb888) {
    const int32_t widthBot = 0;
    const int32_t widthTop = m_inImageDesc.m_width - 1;
    const int32_t heightBot = 0;
    const int32_t heightTop = m_inImageDesc.m_height - 1;

    for (int adj = 0; adj < m_inImageDesc.m_height / (m_imageScaleCoeff); ++adj) {
      drawOutputPixelBound(_srcCol - 1, _srcRow + adj, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol, _srcRow + adj, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol + 1, _srcRow + adj, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    }
  }

  void __attribute__((always_inline)) drawRgbThinLine(const int32_t _srcCol, const int32_t _srcRow, const ImageBuffer& _outImage, const uint32_t _rgb888) {
    const int32_t widthBot = 0;
    const int32_t widthTop = m_inImageDesc.m_width - 1;
    const int32_t heightBot = 0;
    const int32_t heightTop = m_inImageDesc.m_height - 1;

    for (int adj = 0; adj < m_inImageDesc.m_height / m_imageScaleCoeff; ++adj)
      drawOutputPixelBound(_srcCol, _srcRow + adj, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
  }

  void __attribute__((always_inline))
  drawRgbHorizontalLine(const int32_t _srcCol, const int32_t _srcRow, const ImageBuffer& _outImage, const uint32_t _rgb888) {
    const int32_t widthBot = 0;
    const int32_t widthTop = m_inImageDesc.m_width - 1;
    const int32_t heightBot = 0;
    const int32_t heightTop = m_inImageDesc.m_height - 1;

    for (int adj = 0; adj < m_inImageDesc.m_width; ++adj)
      drawOutputPixelBound(_srcCol + adj, _srcRow, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
  }

  static bool __attribute__((always_inline)) detectHsvPixel(const uint32_t _hsv, const uint64_t _hsv_range, const uint32_t _hsv_expect) {
    const uint32_t u32_hsv_det = _cmpltu4(_hsv, _hill(_hsv_range)) | _cmpgtu4(_hsv, _loll(_hsv_range));

    return (u32_hsv_det == _hsv_expect);
  }

  static uint64_t convert2xYuyvToRgb888(const uint32_t _yuyv) {
    const int64_t s64_yuyv1 = _mpyu4ll(_yuyv,
      (static_cast<uint32_t>(static_cast<uint8_t>(409 / 4)) << 24) | (static_cast<uint32_t>(static_cast<uint8_t>(298 / 4)) << 16) |
        (static_cast<uint32_t>(static_cast<uint8_t>(516 / 4)) << 8) | (static_cast<uint32_t>(static_cast<uint8_t>(298 / 4))));
    const uint32_t u32_yuyv2 =
      _dotpus4(_yuyv, (static_cast<uint32_t>(static_cast<uint8_t>(-208 / 4)) << 24) | (static_cast<uint32_t>(static_cast<uint8_t>(-100 / 4)) << 8));
    const uint32_t u32_rgb_h = _add2(_packh2(0, _hill(s64_yuyv1)), (static_cast<uint32_t>(static_cast<uint16_t>(128 / 4 + (-128 * 409 - 16 * 298) / 4))));
    const uint32_t u32_rgb_l = _add2(_packlh2(u32_yuyv2, _loll(s64_yuyv1)),
      (static_cast<uint32_t>(static_cast<uint16_t>(128 / 4 + (+128 * 100 + 128 * 208 - 16 * 298) / 4) << 16)) |
        (static_cast<uint32_t>(static_cast<uint16_t>(128 / 4 + (-128 * 516 - 16 * 298) / 4))));
    const uint32_t u32_y1y1 = _pack2(_loll(s64_yuyv1), _loll(s64_yuyv1));
    const uint32_t u32_y2y2 = _pack2(_hill(s64_yuyv1), _hill(s64_yuyv1));
    const uint32_t u32_rgb_p1h = _clr(_shr2(_add2(u32_rgb_h, u32_y1y1), 6), 16, 31);
    const uint32_t u32_rgb_p1l = _shr2(_add2(u32_rgb_l, u32_y1y1), 6);
    const uint32_t u32_rgb_p2h = _clr(_shr2(_add2(u32_rgb_h, u32_y2y2), 6), 16, 31);
    const uint32_t u32_rgb_p2l = _shr2(_add2(u32_rgb_l, u32_y2y2), 6);
    const uint32_t u32_rgb_p1 = _spacku4(u32_rgb_p1h, u32_rgb_p1l);
    const uint32_t u32_rgb_p2 = _spacku4(u32_rgb_p2h, u32_rgb_p2l);
    return _itoll(u32_rgb_p2, u32_rgb_p1);
  }

  static uint32_t convertRgb888ToHsv(const uint32_t _rgb888) {
    const uint32_t u32_rgb_or16 = _unpkhu4(_rgb888);
    const uint32_t u32_rgb_gb16 = _unpklu4(_rgb888);

    const uint32_t u32_rgb_max2 = _maxu4(_rgb888, _rgb888 >> 8);
    const uint32_t u32_rgb_max = _clr(_maxu4(u32_rgb_max2, u32_rgb_max2 >> 8), 8, 31); // top 3 bytes were non-zeroes!
    const uint32_t u32_rgb_max_max = _pack2(u32_rgb_max, u32_rgb_max);

    const uint32_t u32_hsv_ooo_val_x256 = u32_rgb_max << 8; // get max in 8..15 bits
    const uint32_t u32_rgb_min2 = _minu4(_rgb888, _rgb888 >> 8);
    const uint32_t u32_rgb_min = _minu4(u32_rgb_min2, u32_rgb_min2 >> 8); // top 3 bytes are zeroes
    const uint32_t u32_rgb_delta = u32_rgb_max - u32_rgb_min;

    /* optimized by table based multiplication with power-2 divisor, simulate 255*(max-min)/max */
    const uint32_t u32_hsv_sat_x256 = s_mult255_div[u32_rgb_max] * u32_rgb_delta;

    /* optimized by table based multiplication with power-2 divisor, simulate 43*(med-min)/(max-min) */
    const uint32_t u32_hsv_hue_mult43_div = _pack2(s_mult43_div[u32_rgb_delta], s_mult43_div[u32_rgb_delta]);
    int32_t s32_hsv_hue_x256;
    const uint32_t u32_rgb_cmp = _cmpeq2(u32_rgb_max_max, u32_rgb_gb16);
    if (u32_rgb_cmp == 0)
      s32_hsv_hue_x256 = static_cast<int32_t>((0x10000 * 0) / 3) + static_cast<int32_t>(_dotpn2(u32_hsv_hue_mult43_div, _packhl2(u32_rgb_gb16, u32_rgb_gb16)));
    else if (u32_rgb_cmp == 1)
      s32_hsv_hue_x256 = static_cast<int32_t>((0x10000 * 2) / 3) + static_cast<int32_t>(_dotpn2(u32_hsv_hue_mult43_div, _packlh2(u32_rgb_or16, u32_rgb_gb16)));
    else // 2, 3
      s32_hsv_hue_x256 = static_cast<int32_t>((0x10000 * 1) / 3) + static_cast<int32_t>(_dotpn2(u32_hsv_hue_mult43_div, _pack2(u32_rgb_gb16, u32_rgb_or16)));

    const uint32_t u32_hsv_hue_x256 = static_cast<uint32_t>(s32_hsv_hue_x256);
    const uint32_t u32_hsv_sat_hue_x256 = _pack2(u32_hsv_sat_x256, u32_hsv_hue_x256);
    const uint32_t u32_hsv = _packh4(u32_hsv_ooo_val_x256, u32_hsv_sat_hue_x256);
    return u32_hsv;
  }

  void convertImageYuyvToHsv(const ImageBuffer& _inImage) {
    const uint32_t srcImageRowEffectiveSize = m_inImageDesc.m_width;
    const uint32_t srcImageRowEffectiveToFullSize = m_inImageDesc.m_lineLength - srcImageRowEffectiveSize;
    const int8_t* restrict srcImageRowY = _inImage.m_ptr;
    const int8_t* restrict srcImageRowC = _inImage.m_ptr + m_inImageDesc.m_lineLength * m_inImageDesc.m_height;
    const int8_t* restrict srcImageToY = srcImageRowY + m_inImageDesc.m_lineLength * m_inImageDesc.m_height;
    uint64_t* restrict rgb888hsvptr = s_rgb888hsv_l;

    assert(m_inImageDesc.m_height % 4 == 0); // verified in setup
#pragma MUST_ITERATE(4, , 4)
    while (srcImageRowY != srcImageToY) {
      assert(reinterpret_cast<intptr_t>(srcImageRowY) % 8 == 0); // let's pray...
      assert(reinterpret_cast<intptr_t>(srcImageRowC) % 8 == 0); // let's pray...
      const uint32_t* restrict srcImageColY4 = reinterpret_cast<const uint32_t*>(srcImageRowY);
      const uint32_t* restrict srcImageColC4 = reinterpret_cast<const uint32_t*>(srcImageRowC);
      srcImageRowY += srcImageRowEffectiveSize;
      srcImageRowC += srcImageRowEffectiveSize;

      assert(m_inImageDesc.m_width % 32 == 0); // verified in setup
#pragma MUST_ITERATE(32 / 4, , 32 / 4)
      while (reinterpret_cast<const int8_t*>(srcImageColY4) != srcImageRowY) {
        assert(reinterpret_cast<const int8_t*>(srcImageColC4) != srcImageRowC);

        const uint32_t yy4x = *srcImageColY4++;
        const uint32_t uv4x = _swap4(*srcImageColC4++);

        const uint32_t yuyv12 = (_unpklu4(yy4x)) | (_unpklu4(uv4x) << 8);
        const uint32_t yuyv34 = (_unpkhu4(yy4x)) | (_unpkhu4(uv4x) << 8);

        const uint64_t rgb12 = convert2xYuyvToRgb888(yuyv12);
        *rgb888hsvptr++ = _itoll(_loll(rgb12), convertRgb888ToHsv(_loll(rgb12)));
        *rgb888hsvptr++ = _itoll(_hill(rgb12), convertRgb888ToHsv(_hill(rgb12)));

        const uint64_t rgb34 = convert2xYuyvToRgb888(yuyv34);
        *rgb888hsvptr++ = _itoll(_loll(rgb34), convertRgb888ToHsv(_loll(rgb34)));
        *rgb888hsvptr++ = _itoll(_hill(rgb34), convertRgb888ToHsv(_hill(rgb34)));
      }

      srcImageRowY += srcImageRowEffectiveToFullSize;
      srcImageRowC += srcImageRowEffectiveToFullSize;
    }
  }

  void proceedImageHsv(ImageBuffer& _outImage) {
    const uint64_t* restrict rgb888hsvptr = s_rgb888hsv_l;
    const uint32_t width = m_inImageDesc.m_width;
    const uint32_t height = m_inImageDesc.m_height;
    const uint32_t dstLineLength = m_outImageDesc.m_lineLength;
    const uint64_t u64_hsv_range = m_detectRange;
    const uint32_t u32_hsv_expect = m_detectExpected;
    uint32_t targetPointsPerRow;
    uint32_t targetPointsCol;

    const uint32_t* restrict p_hi2ho = s_hi2ho_l;
    assert(m_inImageDesc.m_height % 4 == 0); // verified in setup
#pragma MUST_ITERATE(4, , 4)
    for (uint32_t srcRow = 0; srcRow < height; ++srcRow) {
      const uint32_t dstRow = *(p_hi2ho++);
      uint16_t* restrict dstImageRow = reinterpret_cast<uint16_t*>(_outImage.m_ptr + dstRow * dstLineLength);

      targetPointsPerRow = 0;
      targetPointsCol = 0;
      const uint32_t* restrict p_wi2wo = s_wi2wo_l;
      assert(m_inImageDesc.m_width % 32 == 0); // verified in setup
#pragma MUST_ITERATE(32, , 32)
      for (uint32_t srcCol = 0; srcCol < width; ++srcCol) {
        const uint32_t dstCol = *(p_wi2wo++);
        const uint64_t rgb888hsv = *rgb888hsvptr++;

        bool det = false;
        if (srcCol >= 5 && srcCol <= width - 5) {
          det = detectHsvPixel(_loll(rgb888hsv), u64_hsv_range, u32_hsv_expect);
          targetPointsPerRow += det;
          targetPointsCol += det ? srcCol : 0;
          writeOutputPixel(dstImageRow + dstCol, det ? 0x00ffff : _hill(rgb888hsv));
        }
      }
      m_targetX += targetPointsCol;
      m_targetY += srcRow * targetPointsPerRow;
      m_targetPoints += targetPointsPerRow;
      if (srcRow >= m_hStart && srcRow <= m_hStop)
        m_crossPoints += targetPointsPerRow;
    }
  }

public:
  virtual bool setup(const ImageDesc& _inImageDesc, const ImageDesc& _outImageDesc, int8_t* _fastRam, size_t _fastRamSize) {
    m_inImageDesc = _inImageDesc;
    m_outImageDesc = _outImageDesc;

    if (m_inImageDesc.m_width % 32 != 0 || m_inImageDesc.m_height % 4 != 0)
      return false;

#define min(x, y) x < y ? x : y;
    const double srcToDstShift =
      min(static_cast<double>(m_outImageDesc.m_width) / m_inImageDesc.m_width, static_cast<double>(m_outImageDesc.m_height) / m_inImageDesc.m_height);

    const uint32_t widthIn = _inImageDesc.m_width;
    const uint32_t widthOut = _outImageDesc.m_width;
    uint32_t* restrict p_wi2wo = s_wi2wo_l;
    for (int i = 0; i < widthIn; i++)
      *(p_wi2wo++) = i * srcToDstShift;

    const uint32_t heightIn = _inImageDesc.m_height;
    const uint32_t heightOut = _outImageDesc.m_height;
    uint32_t* restrict p_hi2ho = s_hi2ho_l;
    for (uint32_t i = 0; i < heightIn; i++)
      *(p_hi2ho++) = i * srcToDstShift;

    /* Static member initialization on first instance creation */
    if (s_mult43_div == NULL || s_mult255_div == NULL) {
      if (_fastRamSize < (1u << 8) * sizeof(*s_mult43_div) + (1u << 8) * sizeof(*s_mult255_div))
        return false;

      s_mult43_div = reinterpret_cast<typeof(s_mult43_div)>(_fastRam);
      _fastRam += (1u << 8) * sizeof(*s_mult43_div);
      s_mult255_div = reinterpret_cast<typeof(s_mult255_div)>(_fastRam);
      _fastRam += (1u << 8) * sizeof(*s_mult255_div);

      s_mult43_div[0] = 0;
      s_mult255_div[0] = 0;
      for (uint32_t idx = 1; idx < (1u << 8); ++idx) {
        s_mult43_div[idx] = (43u * (1u << 8)) / idx;
        s_mult255_div[idx] = (255u * (1u << 8)) / idx;
      }
    }

    return true;
  }

  virtual bool run(const ImageBuffer& _inImage, ImageBuffer& _outImage, const trik_cv_algorithm_in_args& _inArgs, trik_cv_algorithm_out_args& _outArgs) {
    if (m_inImageDesc.m_height * m_inImageDesc.m_lineLength > _inImage.m_size)
      return false;
    if (m_outImageDesc.m_height * m_outImageDesc.m_lineLength > _outImage.m_size)
      return false;
    _outImage.m_size = m_outImageDesc.m_height * m_outImageDesc.m_lineLength;

    m_targetX = 0;
    m_targetPoints = 0;
    m_crossPoints = 0;

    m_inImageFirstRow = m_inImageDesc.m_height - m_inImageDesc.m_height / m_imageScaleCoeff;

    uint32_t detectHueFrom = range<int32_t>(0, (static_cast<int32_t>(_inArgs.detect_hue_from) * 255) / 359, 255); // scaling 0..359 to 0..255
    uint32_t detectHueTo = range<int32_t>(0, (static_cast<int32_t>(_inArgs.detect_hue_to) * 255) / 359, 255);     // scaling 0..359 to 0..255
    uint32_t detectSatFrom = range<int32_t>(0, (static_cast<int32_t>(_inArgs.detect_sat_from) * 255) / 100, 255); // scaling 0..100 to 0..255
    uint32_t detectSatTo = range<int32_t>(0, (static_cast<int32_t>(_inArgs.detect_sat_to) * 255) / 100, 255);     // scaling 0..100 to 0..255

    uint32_t detectValFrom = range<int32_t>(0, (static_cast<int32_t>(_inArgs.detect_val_from) * 255) / 100, 255); // scaling 0..100 to 0..255
    uint32_t detectValTo = range<int32_t>(0, (static_cast<int32_t>(_inArgs.detect_val_to) * 255) / 100, 255);     // scaling 0..100 to 0..255
    bool autoDetectHsv = static_cast<bool>(_inArgs.auto_detect_hsv);                                              // true or false

    if (detectHueFrom <= detectHueTo) {
      m_detectRange = _itoll((detectValFrom << 16) | (detectSatFrom << 8) | detectHueFrom, (detectValTo << 16) | (detectSatTo << 8) | detectHueTo);
      m_detectExpected = 0x0;
    } else {
      assert(detectHueFrom > 0 && detectHueTo < 255);
      m_detectRange = _itoll((detectValFrom << 16) | (detectSatFrom << 8) | (detectHueTo + 1), (detectValTo << 16) | (detectSatTo << 8) | (detectHueFrom - 1));
      m_detectExpected = 0x1;
    }

    int32_t drawY = m_inImageFirstRow - m_inImageDesc.m_height / 2 + m_inImageDesc.m_height / (2 * m_imageScaleCoeff);
    const int hWidth = m_inImageDesc.m_width / 2;
    const int hHeight = m_inImageDesc.m_height / 2;
    const int step = 40;

#ifdef DEBUG_REPEAT
    for (unsigned repeat = 0; repeat < DEBUG_REPEAT; ++repeat) {
#endif

      if (m_inImageDesc.m_height > 0 && m_inImageDesc.m_width > 0) {
        convertImageYuyvToHsv(_inImage);

        if (autoDetectHsv) {
          HsvRangeDetector rangeDetector = HsvRangeDetector(m_inImageDesc.m_width, m_inImageDesc.m_height, step);
          rangeDetector.detect(_outArgs.detect_hue_from, _outArgs.detect_hue_to, _outArgs.detect_sat_from, _outArgs.detect_sat_to, _outArgs.detect_val_from,
            _outArgs.detect_val_to, s_rgb888hsv_l);
        }

        proceedImageHsv(_outImage);
      }

#ifdef DEBUG_REPEAT
    } // repeat
#endif

    drawRgbThinLine(hWidth - step, drawY, _outImage, 0xff00ff);
    drawRgbThinLine(hWidth + step, drawY, _outImage, 0xff00ff);
    drawRgbThinLine(hWidth - 2 * step, drawY, _outImage, 0xff00ff);
    drawRgbThinLine(hWidth + 2 * step, drawY, _outImage, 0xff00ff);

    m_hStart = hHeight;
    m_hStop = hHeight + 2 * step;

    int crossSize = static_cast<uint32_t>(m_crossPoints * 100) / (m_inImageDesc.m_width * 2 * step);

    drawRgbHorizontalLine(0, m_hStart, _outImage, 0xff0000);
    drawRgbHorizontalLine(0, m_hStop, _outImage, 0xff0000);

    _outArgs.targets[0].x = 0;
    _outArgs.targets[0].y = 0;
    _outArgs.targets[0].size = 0;

    if (m_targetPoints > 10) {
      const int32_t inImagePixels = m_inImageDesc.m_height * m_inImageDesc.m_width;
      const int32_t targetX = m_targetX / m_targetPoints;

      assert(m_inImageDesc.m_height > 0 && m_inImageDesc.m_width > 0); // more or less safe since no target points would be detected otherwise

      drawRgbTargetCenterLine(targetX, drawY, _outImage, 0xff0000);

      _outArgs.targets[0].x = ((targetX - static_cast<int32_t>(m_inImageDesc.m_width) / 2) * 100 * 2) / static_cast<int32_t>(m_inImageDesc.m_width);
      _outArgs.targets[0].y = crossSize;
      _outArgs.targets[0].size = static_cast<uint32_t>(m_targetPoints * 100 * m_imageScaleCoeff) / inImagePixels;
    }

    return true;
  }
};

uint16_t* restrict LineSensorCvAlgorithm::s_mult43_div = NULL;
uint16_t* restrict LineSensorCvAlgorithm::s_mult255_div = NULL;

}
}

#endif
