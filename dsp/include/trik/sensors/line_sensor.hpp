#ifndef TRIK_SENSORS_LINE_SENSOR_HPP_
#define TRIK_SENSORS_LINE_SENSOR_HPP_

#ifndef __cplusplus
#error C++-only header
#endif

#include <trik/sensors/cv_algorithms.hpp>
#include <ti/sysbios/family/c64p/Cache.h>
#include <c6x.h>
#include <cassert>
#include <cmath>

#include "hsv_range_detector.hpp"

namespace trik {
namespace sensors {

class LineSensorCvAlgorithm : public CvAlgorithm<VideoFormat::YUV422, VideoFormat::RGB565X> {
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

  void proceedImageHsv(ImageBuffer& _outImage) {
    const uint64_t* restrict rgb888hsvptr = s_rgb888hsv;
    const uint32_t width = m_inImageDesc.m_width;
    const uint32_t height = m_inImageDesc.m_height;
    const uint32_t dstLineLength = m_outImageDesc.m_lineLength;
    const uint64_t u64_hsv_range = m_detectRange;
    const uint32_t u32_hsv_expect = m_detectExpected;
    uint32_t targetPointsPerRow;
    uint32_t targetPointsCol;

    int32_t sum_targetX = 0;
    int32_t sum_targetY = 0;
    uint32_t sum_targetPoints = 0;

    uint32_t local_hStart = m_hStart;
    uint32_t local_hStop = m_hStop;
    uint32_t sum_crossPoints = 0;

    const uint32_t* restrict p_hi2ho = s_hi2ho;
    assert(m_inImageDesc.m_height % 4 == 0); // verified in setup
#pragma MUST_ITERATE(4, , 4)
    for (uint32_t srcRow = 0; srcRow < height; ++srcRow) {
      const uint32_t dstRow = *(p_hi2ho++);
      uint16_t* restrict dstImageRow = reinterpret_cast<uint16_t*>(_outImage.m_ptr + dstRow * dstLineLength);

      targetPointsPerRow = 0;
      targetPointsCol = 0;
      const uint32_t* restrict p_wi2wo = s_wi2wo;
      assert(m_inImageDesc.m_width % 32 == 0); // verified in setup
#pragma MUST_ITERATE(16, , 16)
      for (uint32_t srcCol = 0; srcCol < width;) {
        const uint32_t dstCol = *(p_wi2wo++);
        const uint64_t rgb888hsv = *rgb888hsvptr++;

        bool det = false;
        if (srcCol >= 5 && srcCol <= width - 5) {
          det = detectHsvPixel(_loll(rgb888hsv), u64_hsv_range, u32_hsv_expect);
          targetPointsPerRow += det;
          targetPointsCol += det ? srcCol : 0;
          writeOutputPixel(dstImageRow + dstCol, det ? 0x00ffff : _hill(rgb888hsv));
        }
        srcCol++;

        const uint32_t dstCol2 = *(p_wi2wo++);
        const uint64_t rgb888hsv2 = *rgb888hsvptr++;
        if (srcCol >= 5 && srcCol <= width - 5) {
          det = detectHsvPixel(_loll(rgb888hsv2), u64_hsv_range, u32_hsv_expect);
          targetPointsPerRow += det;
          targetPointsCol += det ? srcCol : 0;
          writeOutputPixel(dstImageRow + dstCol2, det ? 0x00ffff : _hill(rgb888hsv2));
        }
        srcCol++;
      }
      sum_targetX += targetPointsCol;
      sum_targetY += srcRow * targetPointsPerRow;
      sum_targetPoints += targetPointsPerRow;
      if (srcRow >= local_hStart && srcRow <= local_hStop)
        sum_crossPoints += targetPointsPerRow;
    }
    m_targetX = sum_targetX;
    m_targetY = sum_targetY;
    m_targetPoints = sum_targetPoints;
    m_crossPoints = sum_crossPoints;
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
            _outArgs.detect_val_to, s_rgb888hsv);
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

      drawRgbTargetCenterLine(targetX, hHeight, _outImage, 0xff0000);

      _outArgs.targets[0].x = ((targetX - static_cast<int32_t>(m_inImageDesc.m_width) / 2) * 100 * 2) / static_cast<int32_t>(m_inImageDesc.m_width);
      _outArgs.targets[0].y = crossSize;
      _outArgs.targets[0].size = static_cast<uint32_t>(m_targetPoints * 100 * m_imageScaleCoeff) / inImagePixels;
    }
    Cache_wbInv(_outImage.m_ptr, _outImage.m_size, Cache_Type_ALL, TRUE);

    return true;
  }
};

}
}

#endif
