#ifndef TRIK_SENSORS_MOTION_DETECTOR_HPP_
#define TRIK_SENSORS_MOTION_DETECTOR_HPP_

#ifndef __cplusplus
#error C++-only header
#endif

#include <trik/sensors/cv_algorithms.hpp>

#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/System.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <c6x.h>
#include <cassert>
#include <cmath>

namespace trik {
namespace sensors {

#define CAMERA_NOISE_S16 0x0A00     /* SQ12.3 */
#define THRESHOLD_FACTOR_S16 0x31ff /* SQ4.11 */

class MotionSensorCvAlgorithm : public CvAlgorithm<VideoFormat::YUV422, VideoFormat::RGB565X> {
private:
  uint64_t m_detectRange;
  uint32_t m_detectExpected;
  uint32_t m_srcToDstShift;

  int32_t m_targetX;
  int32_t m_targetY;
  uint32_t m_targetPoints;

  bool testifyRgbPixel(const uint32_t _rgb888, uint32_t& _out_rgb888) const {
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
    else
      s32_hsv_hue_x256 = static_cast<int32_t>((0x10000 * 1) / 3) + static_cast<int32_t>(_dotpn2(u32_hsv_hue_mult43_div, _pack2(u32_rgb_gb16, u32_rgb_or16)));

    const uint32_t u32_hsv_hue_x256 = static_cast<uint32_t>(s32_hsv_hue_x256);
    const uint32_t u32_hsv_sat_hue_x256 = _pack2(u32_hsv_sat_x256, u32_hsv_hue_x256);

    const uint32_t u32_hsv = _packh4(u32_hsv_ooo_val_x256, u32_hsv_sat_hue_x256);
    const uint64_t u64_hsv_range = m_detectRange;
    const uint32_t u32_hsv_det = _cmpltu4(u32_hsv, _hill(u64_hsv_range)) | _cmpgtu4(u32_hsv, _loll(u64_hsv_range));

    // SCORE cf5dee: 2.541, 2.485, 2.526, 2.471, 2.514
    if (u32_hsv_det != m_detectExpected)
      return false;

    _out_rgb888 = 0xffff00;
    return true;
  }

  void proceedRgbPixel(const uint32_t _srcRow, const uint32_t _srcCol, uint16_t* restrict _dstImagePix, const uint32_t _rgb888) {
    uint32_t out_rgb888 = _rgb888;
    if (testifyRgbPixel(_rgb888, out_rgb888)) {
      m_targetX += _srcCol;
      m_targetY += _srcRow;
      ++m_targetPoints;
    }

    writeOutputPixel(_dstImagePix, out_rgb888);
  }

  void proceedTwoYuyvPixels(const uint32_t _srcRow, const uint32_t _srcCol1, const uint32_t _srcCol2, uint16_t* restrict _dstImagePix1,
    uint16_t* restrict _dstImagePix2, const uint32_t _yuyv) {
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

    proceedRgbPixel(_srcRow, _srcCol1, _dstImagePix1, u32_rgb_p1);

    proceedRgbPixel(_srcRow, _srcCol2, _dstImagePix2, u32_rgb_p2);
  }

public:
  virtual bool setup(const ImageDesc& _inImageDesc, const ImageDesc& _outImageDesc, int8_t* _fastRam, size_t _fastRamSize) {
    if (!commonSetup(_inImageDesc, _outImageDesc, _fastRam, _fastRamSize))
      return false;
    return true;
  }

  virtual bool run(const ImageBuffer& _inImage, ImageBuffer& _outImage, const trik_cv_algorithm_in_args& _inArgs, trik_cv_algorithm_out_args& _outArgs) {
    if (m_inImageDesc.m_height * m_inImageDesc.m_lineLength > _inImage.m_size)
      return false;
    if (m_outImageDesc.m_height * m_outImageDesc.m_lineLength > _outImage.m_size)
      return false;
    _outImage.m_size = m_outImageDesc.m_height * m_outImageDesc.m_lineLength;

    m_targetX = 0;
    m_targetY = 0;
    m_targetPoints = 0;

    uint32_t detectHueFrom = range<int32_t>(0, (static_cast<int32_t>(_inArgs.detect_hue_from) * 255) / 359, 255); // scaling 0..359 to 0..255
    uint32_t detectHueTo = range<int32_t>(0, (static_cast<int32_t>(_inArgs.detect_hue_to) * 255) / 359, 255);     // scaling 0..359 to 0..255
    uint32_t detectSatFrom = range<int32_t>(0, (static_cast<int32_t>(_inArgs.detect_sat_from) * 255) / 100, 255); // scaling 0..100 to 0..255
    uint32_t detectSatTo = range<int32_t>(0, (static_cast<int32_t>(_inArgs.detect_sat_to) * 255) / 100, 255);     // scaling 0..100 to 0..255
    uint32_t detectValFrom = range<int32_t>(0, (static_cast<int32_t>(_inArgs.detect_val_from) * 255) / 100, 255); // scaling 0..100 to 0..255
    uint32_t detectValTo = range<int32_t>(0, (static_cast<int32_t>(_inArgs.detect_val_to) * 255) / 100, 255);     // scaling 0..100 to 0..255

    if (detectHueFrom <= detectHueTo) {
      m_detectRange = _itoll((detectValFrom << 16) | (detectSatFrom << 8) | detectHueFrom, (detectValTo << 16) | (detectSatTo << 8) | detectHueTo);
      m_detectExpected = 0x0;
    } else {
      assert(detectHueFrom > 0 && detectHueTo < 255);
      m_detectRange = _itoll((detectValFrom << 16) | (detectSatFrom << 8) | (detectHueTo + 1), (detectValTo << 16) | (detectSatTo << 8) | (detectHueFrom - 1));
      m_detectExpected = 0x1;
    }

#ifdef DEBUG_REPEAT
    for (unsigned repeat = 0; repeat < DEBUG_REPEAT; ++repeat) {
#endif

      if (m_inImageDesc.m_height > 0 && m_inImageDesc.m_width > 0) {
        const uint32_t srcToDstShift = m_srcToDstShift;
        const uint32_t width = m_inImageDesc.m_width;
        const uint32_t height = m_inImageDesc.m_height;
        const uint32_t srcLineLength = m_inImageDesc.m_lineLength;
        const uint32_t dstLineLength = m_outImageDesc.m_lineLength;

        assert(m_inImageDesc.m_height % 4 == 0); // verified in setup
        for (uint32_t srcRow = 0; srcRow < height; ++srcRow) {
          const uint32_t dstRow = srcRow >> m_srcToDstShift;

          const uint32_t srcRowOfs = srcRow * srcLineLength;
          const uint32_t* restrict srcImage = reinterpret_cast<uint32_t*>(_inImage.m_ptr + srcRowOfs);

          const uint32_t dstRowOfs = dstRow * dstLineLength;
          uint16_t* restrict dstImageRow = reinterpret_cast<uint16_t*>(_outImage.m_ptr + dstRowOfs);

          assert(m_inImageDesc.m_width % 32 == 0); // verified in setup
          for (uint32_t srcCol = 0; srcCol < width; srcCol += 2) {
            const uint32_t dstCol1 = (srcCol + 0) >> srcToDstShift;
            const uint32_t dstCol2 = (srcCol + 1) >> srcToDstShift;
            uint16_t* restrict dstImagePix1 = &dstImageRow[dstCol1]; // even if they point the same place, we don't really care
            uint16_t* restrict dstImagePix2 = &dstImageRow[dstCol2];
            proceedTwoYuyvPixels(srcRow, srcCol + 0, srcCol + 1, dstImagePix1, dstImagePix2, *srcImage++);
          }
        }
      }

#ifdef DEBUG_REPEAT
    } // repeat
#endif

    if (m_targetPoints > 0) {
      const int32_t targetX = m_targetX / m_targetPoints;
      const int32_t targetY = m_targetY / m_targetPoints;

      assert(m_inImageDesc.m_height > 0 && m_inImageDesc.m_width > 0); // more or less safe since no target points would be detected otherwise
      const uint32_t targetRadius = std::ceil(std::sqrt(static_cast<float>(m_targetPoints) / 3.1415927f));

      drawOutputCircle(targetX, targetY, targetRadius, _outImage, 0xffff00);

      _outArgs.targets[0].out_target.targetLocation.x = ((targetX - static_cast<int32_t>(m_inImageDesc.m_width) / 2) * 100 * 2) / static_cast<int32_t>(m_inImageDesc.m_width);
      _outArgs.targets[0].out_target.targetLocation.y = ((targetY - static_cast<int32_t>(m_inImageDesc.m_height) / 2) * 100 * 2) / static_cast<int32_t>(m_inImageDesc.m_height);
      _outArgs.targets[0].out_target.targetLocation.size = static_cast<uint32_t>(targetRadius * 100 * 4) / static_cast<uint32_t>(m_inImageDesc.m_width + m_inImageDesc.m_height);
    } else {
      _outArgs.targets[0].out_target.targetLocation.x = 0;
      _outArgs.targets[0].out_target.targetLocation.y = 0;
      _outArgs.targets[0].out_target.targetLocation.size = 0;
    }

    Cache_wbInv(_outImage.m_ptr, _outImage.m_size, Cache_Type_ALL, TRUE);

    return true;
  }
};

}
}

#endif
