#ifndef TRIK_SENSORS_EDGE_LINE_SENSOR_HPP_
#define TRIK_SENSORS_EDGE_LINE_SENSOR_HPP_

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

extern "C" {
#include <ti/imglib/src/IMG_sobel_3x3_8/IMG_sobel_3x3_8.h>
#include <ti/imglib/src/IMG_thr_gt2max_8/IMG_thr_gt2max_8.h>
#include <ti/imglib/src/IMG_ycbcr422pl_to_rgb565/IMG_ycbcr422pl_to_rgb565.h>
}

namespace trik {
namespace sensors {

static uint8_t s_y[320 * 240];
static uint8_t s_y2[320 * 240];
static uint8_t s_cb[320 * 240];
static uint8_t s_cr[320 * 240];

static uint32_t s_wi2wo[640];
static uint32_t s_hi2ho[480];

static int16_t s_xGrad[320 * 240 + 1];
static int16_t s_yGrad[320 * 240 + 1];
static int16_t s_gradMag[320 * 240 + 1];
static uint16_t s_harrisScore_el[320 * 240];

static int8_t s_corners_el[320 * 240];

static uint8_t s_buffer_el[200]; // 200

static const short s_coeff_el[5] = { 0x2000, 0x2BDD, -0x0AC5, -0x1658, 0x3770 };

class EdgeLineSensorCvAlgorithm : public CvAlgorithm<VideoFormat::YUV422, VideoFormat::RGB565X> {
private:
  static const int m_detectZoneScale = 6;
  /*
    m_detectZoneScale
    The perpose of this variable is to set borders of auto color detector special zones.
    Example:
      Let's m_detectZoneScale is 6.
      Then "step" is m_width/6
      It means that inside zone of color detector looks like square wiht bounds at (x_center +- step) and (y_center +- step).
      Middle (neutral) zone has (x_center +- 2*step) and (y_center +- 2*step) bounds.

      ----------------
      |   neutral    |
      |   --------   |
      |   |inside|   |
      |   | zone |   |
      |   |      |   |
      |   --------   |
      |    zone      |
      ----------------
  */

  uint64_t m_detectRange;
  uint32_t m_detectExpected;
  double m_srcToDstShift;

  int32_t m_targetX;
  int32_t m_targetY;
  uint32_t m_targetPoints;

  void convertImageYuyvToRgb(const ImageBuffer& _inImage, ImageBuffer& _outImage) {
    const uint32_t width = m_inImageDesc.m_width;
    const uint32_t height = m_inImageDesc.m_height;
    const uint32_t imgSize = width * height;

    uint16_t targetPointsPerRow;
    uint16_t targetPointsCol;

    // separate Cb Cr
    uint8_t* restrict y2 = reinterpret_cast<uint8_t*>(s_y2);
    uint8_t* restrict cb = reinterpret_cast<uint8_t*>(s_cb);
    uint8_t* restrict cr = reinterpret_cast<uint8_t*>(s_cr);
    const uint8_t* restrict CbCr = reinterpret_cast<const uint8_t*>(_inImage.m_ptr);
    for (int i = 0; i < imgSize / 2; i++) {
      *(y2++) = *CbCr;
      CbCr++;
      *(cb++) = *CbCr;
      CbCr++;
      *(y2++) = *CbCr;
      CbCr++;
      *(cr++) = *CbCr;
      CbCr++;
    }

    // Sobel edge detection
    const unsigned char* restrict y_in_sobel = reinterpret_cast<const unsigned char*>(s_y2);
    unsigned char* restrict sobel_out = reinterpret_cast<unsigned char*>(s_y);
    IMG_sobel_3x3_8(y_in_sobel, sobel_out, width, height);

    IMG_thr_gt2max_8(reinterpret_cast<const unsigned char*>(s_y), reinterpret_cast<unsigned char*>(s_y), width, height, 50);

    // detect line
    const uint8_t* restrict sobelBin = reinterpret_cast<unsigned char*>(s_y);
    assert(m_inImageDesc.m_height % 4 == 0); // verified in setup
    for (int r = 0; r < height; r++) {
      targetPointsPerRow = 0;
      targetPointsCol = 0;

      assert(m_inImageDesc.m_width % 32 == 0); // verified in setup
#pragma MUST_ITERATE(2, , 2)
      for (int c = 0; c < width; c++) {
        if (c > 15 && c < width - 15) {
          const bool det = (*sobelBin == 0xFF);
          targetPointsPerRow += det;
          targetPointsCol += det ? c : 0;
        }
        sobelBin++;
      }
      m_targetX += targetPointsCol;
      m_targetPoints += targetPointsPerRow;
    }

#ifdef CORNERS
    // Harris corner detector
    VLIB_xyGradientsAndMagnitude(reinterpret_cast<const uint8_t*>(_inImage.m_ptr), reinterpret_cast<int16_t*>(s_xGrad), reinterpret_cast<int16_t*>(s_yGrad),
      reinterpret_cast<int16_t*>(s_gradMag), width, height);

    VLIB_harrisScore_7x7(reinterpret_cast<const int16_t*>(s_xGrad), reinterpret_cast<const int16_t*>(s_yGrad), width, height,
      reinterpret_cast<int16_t*>(s_harrisScore_el), 1500, reinterpret_cast<uint8_t*>(s_buffer_el));

    VLIB_nonMaxSuppress_7x7_S16(reinterpret_cast<const int16_t*>(s_harrisScore_el), width, height, 7000, reinterpret_cast<uint8_t*>(s_corners_el));
#endif

    // in_img to rgb565
    const short* restrict coeff = s_coeff_el;
    const unsigned char* restrict res_in = reinterpret_cast<const unsigned char*>(s_y);
    const unsigned char* restrict cb_in = reinterpret_cast<const unsigned char*>(s_cb);
    const unsigned char* restrict cr_in = reinterpret_cast<const unsigned char*>(s_cr);
    unsigned short* rgb565_out = reinterpret_cast<unsigned short*>(s_gradMag);
    IMG_ycbcr422pl_to_rgb565(coeff, res_in, cb_in, cr_in, rgb565_out, width * height);

    // lets try scaling out // & highlight corners
    const uint16_t* restrict imgRgb565ptr = reinterpret_cast<uint16_t*>(s_gradMag);
    const uint32_t dstLineLength = m_outImageDesc.m_lineLength;
    const uint32_t* restrict p_hi2ho = s_hi2ho;
    for (int r = 0; r < height; r++) {
      const uint32_t dR = *(p_hi2ho++);
      uint16_t* restrict dIR = reinterpret_cast<uint16_t*>(_outImage.m_ptr + dR * dstLineLength);
      const uint32_t* restrict p_wi2wo = s_wi2wo;
#pragma MUST_ITERATE(2, , 2)
      for (int c = 0; c < width; c++) {
        const uint32_t dC = *(p_wi2wo++);
        *(dIR + dC) = *(imgRgb565ptr++);
      }
    }

#ifdef CORNERS
    const uint8_t* restrict corners = reinterpret_cast<uint8_t*>(s_corners_el);
    p_hi2ho = s_hi2ho;
    for (int r = 0; r < height; r++) {
      const uint32_t dR = *(p_hi2ho++);
      uint16_t* dIR = reinterpret_cast<uint16_t*>(_outImage.m_ptr + dR * dstLineLength);
      const uint32_t* restrict p_wi2wo = s_wi2wo;
      for (int c = 0; c < width; c++) {
        const uint32_t dC = *(p_wi2wo++);
        if (c > 5 && c < 315 && r > 5 && r < 235)
          if (*corners != 0)
            drawCornerHighlight(c, r, _outImage, 0xff0000);
        corners++;
      }
    }
#endif
  }

  void proceedImageHsv(const ImageBuffer& _inImage, ImageBuffer& _outImage) {
    const uint8_t* restrict rgb888hsvptr = reinterpret_cast<const uint8_t*>(_inImage.m_ptr);
    const uint32_t width = m_inImageDesc.m_width;
    const uint32_t height = m_inImageDesc.m_height;
    const uint32_t dstLineLength = m_outImageDesc.m_lineLength;
    const /*uint32_t*/ double srcToDstShift = m_srcToDstShift;
    /*
          const uint64_t u64_hsv_range  = m_detectRange;
          const uint32_t u32_hsv_expect = m_detectExpected;
    */
    uint32_t targetPointsPerRow;
    uint32_t targetPointsCol;

    assert(m_inImageDesc.m_height % 4 == 0); // verified in setup
    for (uint32_t srcRow = 0; srcRow < height; ++srcRow) {
      const uint32_t dstRow = srcRow * srcToDstShift;
      uint16_t* restrict dstImageRow = reinterpret_cast<uint16_t*>(_outImage.m_ptr + dstRow * dstLineLength);

      targetPointsPerRow = 0;
      targetPointsCol = 0;
      assert(m_inImageDesc.m_width % 32 == 0); // verified in setup
      for (uint32_t srcCol = 0; srcCol < width; ++srcCol) {
        const uint32_t dstCol = srcCol * srcToDstShift;
        const uint8_t rgb888hsv = *rgb888hsvptr++;

        const bool det = rgb888hsv < 60;
        targetPointsPerRow += det;
        targetPointsCol += det ? srcCol : 0;
        if (det)
          writeOutputPixel(dstImageRow + dstCol, 0x00ffff);
      }
      m_targetX += targetPointsCol;
      m_targetY += srcRow * targetPointsPerRow;
      m_targetPoints += targetPointsPerRow;
    }
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

    uint32_t detectValFrom = range<int32_t>(0, (static_cast<int32_t>(_inArgs.detect_val_from) * 255) / 100, 255); // scaling 0..100 to 0..255
    uint32_t detectValTo = range<int32_t>(0, (static_cast<int32_t>(_inArgs.detect_val_to) * 255) / 100, 255);     // scaling 0..100 to 0..255

    m_detectRange = _itoll((detectValFrom << 16) | (0 << 8) | 0, (detectValTo << 16) | (0 << 8) | 0);
    m_detectExpected = 0x0;

#ifdef DEBUG_REPEAT
    for (unsigned repeat = 0; repeat < DEBUG_REPEAT; ++repeat) {
#endif
      if (m_inImageDesc.m_height > 0 && m_inImageDesc.m_width > 0)
        convertImageYuyvToRgb(_inImage, _outImage);
#ifdef DEBUG_REPEAT
    } // repeat
#endif

    int32_t drawY = m_inImageDesc.m_height / 2;

    if (m_targetPoints > 0) {
      const int32_t targetX = m_targetX / m_targetPoints;
      const int32_t targetY = m_targetY / m_targetPoints;

      assert(m_inImageDesc.m_height > 0 && m_inImageDesc.m_width > 0); // more or less safe since no target points would be detected otherwise
      const uint32_t targetRadius = std::ceil(std::sqrt(static_cast<float>(m_targetPoints) / 3.1415927f));

      drawRgbTargetCenterLine(targetX, drawY, _outImage, 0xff0000);

      _outArgs.targets[0].x = ((targetX - static_cast<int32_t>(m_inImageDesc.m_width) / 2) * 100 * 2) / static_cast<int32_t>(m_inImageDesc.m_width);
      _outArgs.targets[0].y = ((targetY - static_cast<int32_t>(m_inImageDesc.m_height) / 2) * 100 * 2) / static_cast<int32_t>(m_inImageDesc.m_height);
      _outArgs.targets[0].size = static_cast<uint32_t>(targetRadius * 100 * 4) / static_cast<uint32_t>(m_inImageDesc.m_width + m_inImageDesc.m_height);

    } else {
      _outArgs.targets[0].x = 0;
      _outArgs.targets[0].y = 0;
      _outArgs.targets[0].size = 0;
    }

    return true;
  }
};

}
}

#endif
