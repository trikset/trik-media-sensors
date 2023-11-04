#ifndef TRIK_SENSORS_MXN_SENSOR_HPP_
#define TRIK_SENSORS_MXN_SENSOR_HPP_

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

#include "hsv_range_detector.hpp"

namespace trik {
namespace sensors {

const int m_hueScale = 8;  // partition of axis h in 64 parts
const int m_satScale = 64; // s in 4 parts
const int m_valScale = 64; // v in 4 parts

const int m_hueClsters = 256 / m_hueScale; // partition of axis h in 64 parts
const int m_satClsters = 256 / m_satScale; // s in 4 parts
const int m_valClsters = 256 / m_valScale; // v in 4 parts

static int c_color[m_hueClsters][m_satClsters][m_valClsters]; // massiv of clusters 32x8x8

class MxnSensorCvAlgorithm : public CvAlgorithm<VideoFormat::YUV422, VideoFormat::RGB565X> {
private:
  uint8_t m_heightM;
  uint8_t m_widthN;
  uint16_t m_widthStep;
  uint16_t m_heightStep;

  uint64_t m_detectRange;
  uint32_t m_detectExpected;

  void clasterizePixel(const uint32_t _hsv) {}

  void clasterizeImage() {
    const uint64_t* restrict rgb888hsvptr = s_rgb888hsv;
    const uint32_t width = m_inImageDesc.m_width;
    const uint32_t height = m_inImageDesc.m_height;

    const uint64_t u64_hsv_range = m_detectRange;
    const uint32_t u32_hsv_expect = m_detectExpected;

    assert(m_inImageDesc.m_height % 4 == 0); // verified in setup
#pragma MUST_ITERATE(4, , 4)
    for (uint32_t srcRow = 0; srcRow < height; ++srcRow) {

      assert(m_inImageDesc.m_width % 32 == 0); // verified in setup
#pragma MUST_ITERATE(32, , 32)
      for (uint32_t srcCol = 0; srcCol < width; ++srcCol) {
        const uint64_t rgb888hsv = *rgb888hsvptr++;
        clasterizePixel(rgb888hsv);
        const bool det = detectHsvPixel(_loll(rgb888hsv), u64_hsv_range, u32_hsv_expect);
      }
    }
  }

  void proceedImageHsv(ImageBuffer& _outImage) {
    const uint64_t* restrict rgb888hsvptr = s_rgb888hsv;
    const uint32_t width = m_inImageDesc.m_width;
    const uint32_t height = m_inImageDesc.m_height;
    const uint32_t dstLineLength = m_outImageDesc.m_lineLength;

    const uint32_t* restrict p_hi2ho = s_hi2ho;
    assert(m_inImageDesc.m_height % 4 == 0); // verified in setup
#pragma MUST_ITERATE(4, , 4)
    for (uint32_t srcRow = 0; srcRow < height; ++srcRow) {
      const uint32_t dstRow = *(p_hi2ho++);
      uint16_t* restrict dstImageRow = reinterpret_cast<uint16_t*>(_outImage.m_ptr + dstRow * dstLineLength);

      const uint32_t* restrict p_wi2wo = s_wi2wo;
      assert(m_inImageDesc.m_width % 32 == 0); // verified in setup
#pragma MUST_ITERATE(32, , 32)
      for (uint32_t srcCol = 0; srcCol < width; ++srcCol) {
        const uint32_t dstCol = *(p_wi2wo++);
        const uint64_t rgb888hsv = *rgb888hsvptr++;
        writeOutputPixel(dstImageRow + dstCol, _hill(rgb888hsv));
      }
    }
  }

  void __attribute__((always_inline)) fillImage(uint16_t _row, uint16_t _col, const ImageBuffer& _outImage, const uint32_t _rgb888) {
    const uint16_t widthBot = 0;
    const uint16_t widthTop = m_inImageDesc.m_width - 1;
    const uint16_t heightBot = 0;
    const uint16_t heightTop = m_inImageDesc.m_height - 1;

    for (int row = _row; row < _row + 20; ++row)
      for (int col = _col; col < _col + 20; ++col)
        drawOutputPixelBound(col, row, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
  }

  uint32_t __attribute__((always_inline)) GetImgColor(int _rowStart, int _heightStep, int _colStart, int _widthStep) {
    uint32_t rgbResult = 0;
    const uint64_t* restrict img = s_rgb888hsv;

    int ch, cs, cv;
    memset(c_color, 0, sizeof(int) * m_hueClsters * m_satClsters * m_valClsters);

    int ch_max = 0, cs_max = 0, cv_max = 0;
    int maxColorEntry = 0;

    U_Hsv8x3 pixel;
    for (int row = _rowStart; row < _rowStart + _heightStep; row++) {
      for (int column = _colStart; column < _colStart + _widthStep; column++) {
        pixel.whole = _loll(img[row * m_inImageDesc.m_width + column]);

        ch = pixel.parts.h / m_hueScale;
        cs = pixel.parts.s / m_satScale;
        cv = pixel.parts.v / m_valScale;

        c_color[ch][cs][cv]++;
        if (c_color[ch][cs][cv] > maxColorEntry) {
          maxColorEntry = c_color[ch][cs][cv];
          ch_max = ch;
          cs_max = cs;
          cv_max = cv;
        }
      }
    }

    // return h, s and v as h_max, s_max and _max with values
    // scaled to be between 0 and 255.
    int hue = ch_max * m_hueScale;
    int sat = cs_max * m_satScale;
    int val = cv_max * m_valScale;

    return HSVtoRGB(hue, sat, val);
  }

  uint32_t __attribute__((always_inline)) GetImgColor2(uint32_t _row, uint32_t _col, uint32_t _height, uint32_t _width) {
    const uint32_t width = m_inImageDesc.m_width;
    const uint32_t gap = width - _width;
    int ch, cs, cv;
    int ch_max = 0;
    int cs_max = 0;
    int cv_max = 0;

    int maxColorEntry = 0;

    memset(c_color, 0, sizeof(int) * m_hueClsters * m_satClsters * m_valClsters);

    uint64_t* restrict subImg = s_rgb888hsv + _row * width + _col;

    for (int row = 0; row < _height; row++) {
      for (int col = 0; col < _width; col++) {
        uint32_t pixel = _loll(*(subImg++));

        ch = static_cast<uint8_t>(pixel) / m_hueScale;
        cs = static_cast<uint8_t>(pixel >> 8) / m_satScale;
        cv = static_cast<uint8_t>(pixel >> 16) / m_valScale;

        c_color[ch][cs][cv]++;
        if (c_color[ch][cs][cv] > maxColorEntry) {
          maxColorEntry = c_color[ch][cs][cv];
          ch_max = ch;
          cs_max = cs;
          cv_max = cv;
        }
      }
      subImg += gap;
    }

    // return h, s and v as h_max, s_max and _max with values
    // scaled to be between 0 and 255.
    int hue = ch_max * m_hueScale;
    int sat = cs_max * m_satScale;
    int val = cv_max * m_valScale;

    return HSVtoRGB(hue, sat, val);
  }

  void getTrueSV(double& rV, double& rS, double _v, double _s) {
    float boundV = 0.1 / (_s - 1.0) + 1.0;
    float boundS = 0.1 / (_v - 1.0) + 1.0;

    if (!((_s < boundS) && (_v < boundV))) // not ok
    {
      float A = 10.0 * _s / _v;
      float B = -10.0 * (_s / _v + 1);
      const float C = 9;

      float D = pow(B, 2) - 4 * A * C;
      float X1 = (-B - sqrt(D)) / (2 * A);
      float X2 = (-B + sqrt(D)) / (2 * A);

      rV = X1 <= X2 ? X1 : X2;
      rS = _s * rV / _v;
    } else // ok
    {
      rV = _v;
      rS = _s;
    }
  }

  uint32_t HSVtoRGB(int H, int S, int V) {
    uint32_t rgbResult;

    double r = 0;
    double g = 0;
    double b = 0;

    double h = H / 255.0f;
    double s = S / 255.0f;
    double v = V / 255.0f;

    // getTrueSV(v ,s, v ,s);
    v = v < 0.2 ? 0 : v;
    s = s < 0.2 ? 0 : 1;

    int i = h * 6;
    double f = h * 6 - i;
    double p = v * (1 - s);
    double q = v * (1 - f * s);
    double t = v * (1 - (1 - f) * s);

    switch (i % 6) {
    case 0:
      r = v;
      g = t;
      b = p;
      break;
    case 1:
      r = q;
      g = v;
      b = p;
      break;
    case 2:
      r = p;
      g = v;
      b = t;
      break;
    case 3:
      r = p;
      g = q;
      b = v;
      break;
    case 4:
      r = t;
      g = p;
      b = v;
      break;
    case 5:
      r = v;
      g = p;
      b = q;
      break;
    }

    int ri = r * 255;
    int gi = g * 255;
    int bi = b * 255;
    rgbResult = ((int32_t) ri << 16) + ((int32_t) gi << 8) + ((int32_t) bi);

    return rgbResult;
  }

#define min(x, y) x < y ? x : y;

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

    m_heightM = _inArgs.height_n;
    m_widthN = _inArgs.width_n;
    m_widthStep = m_inImageDesc.m_width / m_widthN;
    m_heightStep = m_inImageDesc.m_height / m_heightM;

#ifdef DEBUG_REPEAT
    for (unsigned repeat = 0; repeat < DEBUG_REPEAT; ++repeat) {
#endif

      if (m_inImageDesc.m_height > 0 && m_inImageDesc.m_width > 0) {
        convertImageYuyvToHsv(_inImage);
        proceedImageHsv(_outImage);
      }

#ifdef DEBUG_REPEAT
    } // repeat
#endif

    uint32_t resColor = 0;
    int colorClaster = 0;

    int counter = 0;
    int rowStart = 0;
    for (int i = 0; i < m_heightM; ++i) {
      int colStart = 0;
      for (int j = 0; j < m_widthN; ++j) {
        resColor = GetImgColor2(rowStart, colStart, m_heightStep, m_widthStep);
        fillImage(rowStart, colStart, _outImage, resColor);
        //_outArgs.outColor[counter++] = resColor;
        colStart += m_widthStep;
      }
      rowStart += m_heightStep;
    }

    return true;
  }
};

}
}

#endif
