#ifndef TRIK_SENSORS_OBJECT_SENSOR_HPP_
#define TRIK_SENSORS_OBJECT_SENSOR_HPP_

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

#include <algorithm>
#include <c6x.h>
#include <cassert>
#include <cmath>
#include <map>

#include "bitmap_builder.hpp"
#include "clusterizer.hpp"
#include "hsv_range_detector.hpp"

namespace trik {
namespace sensors {

static uint64_t s_rgb888hsv[IMG_WIDTH * IMG_HEIGHT];
static uint16_t s_bitmap[IMG_WIDTH * IMG_HEIGHT];
static uint16_t s_clustermap[IMG_WIDTH * IMG_HEIGHT];

static int32_t s_wi2wo_out[IMG_WIDTH];
static int32_t s_hi2ho_out[IMG_HEIGHT];
static int32_t s_wi2wo_cstr[IMG_WIDTH];
static int32_t s_hi2ho_cstr[IMG_HEIGHT];

#define OBJECTS 3

class ObjectSensorCvAlgorithm : public CvAlgorithm<VideoFormat::YUV422, VideoFormat::RGB565X> {
private:
  static const int m_detectZoneScale = 6;

  int32_t m_targetX;
  int32_t m_targetY;
  uint32_t m_targetPoints;

  uint16_t m_minTargetSize;

  uint16_t m_clustersAmount;

  ImageDesc m_inImageDesc;
  ImageDesc m_outImageDesc;

  ImageDesc m_bitmapDesc;
  BitmapBuilderCvAlgorithm m_bitmapBuilder;
  ImageBuffer m_bitmap;

  ImageDesc m_clustermapDesc;
  ClusterizerCvAlgorithm m_clusterizer;
  ImageBuffer m_clustermap;

  ImageDesc m_inRgb888HsvImgDesc;
  ImageBuffer m_inRgb888HsvImg;

  static uint16_t* restrict s_mult43_div;  // allocated from fast ram
  static uint16_t* restrict s_mult255_div; // allocated from fast ram

  /*
      template <typename T1, typename T2>
      struct greater_second {
          typedef std::pair<T1, T2> type;
          bool operator ()(type const& a, type const& b) const {
              return a.second > b.second;
          }
      };
  */
  static void __attribute__((always_inline)) writeOutputPixel(uint16_t* restrict _rgb565ptr, const uint32_t _rgb888) {
    *_rgb565ptr = ((_rgb888 >> 3) & 0x001f) | ((_rgb888 >> 5) & 0x07e0) | ((_rgb888 >> 8) & 0xf800);
  }

  void __attribute__((always_inline)) drawOutputPixelBound(const int32_t _srcCol, const int32_t _srcRow, const int32_t _srcColBot, const int32_t _srcColTop,
    const int32_t _srcRowBot, const int32_t _srcRowTop, const ImageBuffer& _outImage, const uint32_t _rgb888) const {
    const int32_t srcCol = range<int32_t>(_srcColBot, _srcCol, _srcColTop);
    const int32_t srcRow = range<int32_t>(_srcRowBot, _srcRow, _srcRowTop);

    const int32_t dstRow = s_hi2ho_out[srcRow];
    const int32_t dstCol = s_wi2wo_out[srcCol];

    const uint32_t dstOfs = dstRow * m_outImageDesc.m_lineLength + dstCol * sizeof(uint16_t);
    writeOutputPixel(reinterpret_cast<uint16_t*>(_outImage.m_ptr + dstOfs), _rgb888);
  }

  void __attribute__((always_inline)) drawFatPixel(const int32_t _srcCol, const int32_t _srcRow, const ImageBuffer& _outImage, const uint32_t _rgb888) {
    const int32_t widthBot = 0;
    const int32_t widthTop = m_inImageDesc.m_width - 1;
    const int32_t heightBot = 0;
    const int32_t heightTop = m_inImageDesc.m_height - 1;

    drawOutputPixelBound(_srcCol - 1, _srcRow - 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol - 1, _srcRow, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol - 1, _srcRow + 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol, _srcRow - 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol, _srcRow, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol, _srcRow + 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol + 1, _srcRow - 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol + 1, _srcRow, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol + 1, _srcRow + 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
  }

  void __attribute__((always_inline))
  drawOutputCircle(const int32_t _srcCol, const int32_t _srcRow, const int32_t _srcRadius, const ImageBuffer& _outImage, const uint32_t _rgb888) const {
    const int32_t widthBot = 0;
    const int32_t widthTop = m_inImageDesc.m_width - 1;
    const int32_t heightBot = 0;
    const int32_t heightTop = m_inImageDesc.m_height - 1;

    int32_t circleError = 1 - _srcRadius;
    int32_t circleErrorY = 1;
    int32_t circleErrorX = -2 * _srcRadius;
    int32_t circleX = _srcRadius;
    int32_t circleY = 0;

    drawOutputPixelBound(_srcCol, _srcRow + _srcRadius, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol, _srcRow - _srcRadius, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol + _srcRadius, _srcRow, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    drawOutputPixelBound(_srcCol - _srcRadius, _srcRow, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);

    while (circleY < circleX) {
      if (circleError >= 0) {
        circleX -= 1;
        circleErrorX += 2;
        circleError += circleErrorX;
      }
      circleY += 1;
      circleErrorY += 2;
      circleError += circleErrorY;

      drawOutputPixelBound(_srcCol + circleX, _srcRow + circleY, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol + circleX, _srcRow - circleY, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol - circleX, _srcRow + circleY, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol - circleX, _srcRow - circleY, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol + circleY, _srcRow + circleX, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol + circleY, _srcRow - circleX, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol - circleY, _srcRow + circleX, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol - circleY, _srcRow - circleX, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    }
  }

  void __attribute__((always_inline)) drawOutputFatRectangle(const int32_t _x1, const int32_t _x2, const int32_t _y1, const int32_t _y2,
    const ImageBuffer& _outImage, const uint32_t _rgb888) const {
    const int32_t widthBot = 0;
    const int32_t widthTop = m_inImageDesc.m_width - 1;
    const int32_t heightBot = 0;
    const int32_t heightTop = m_inImageDesc.m_height - 1;

    for (int32_t c = _x1; c < _x2; c++) {
      drawOutputPixelBound(c, _y1 - 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(c, _y1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(c, _y1 + 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);

      drawOutputPixelBound(c, _y2 - 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(c, _y2, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(c, _y2 + 1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    }

    for (int32_t r = _y1; r < _y2; r++) {
      drawOutputPixelBound(_x1 - 1, r, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_x1, r, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_x1 + 1, r, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);

      drawOutputPixelBound(_x2 - 1, r, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_x2, r, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_x2 + 1, r, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    }
  }

  void __attribute__((always_inline))
  drawOutputRectangle(const int32_t _x1, const int32_t _x2, const int32_t _y1, const int32_t _y2, const ImageBuffer& _outImage, const uint32_t _rgb888) const {
    const int32_t widthBot = 0;
    const int32_t widthTop = m_inImageDesc.m_width - 1;
    const int32_t heightBot = 0;
    const int32_t heightTop = m_inImageDesc.m_height - 1;

    for (int32_t c = _x1; c < _x2; c++) {
      drawOutputPixelBound(c, _y1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);

      drawOutputPixelBound(c, _y2, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    }

    for (int32_t r = _y1; r < _y2; r++) {
      drawOutputPixelBound(_x1, r, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);

      drawOutputPixelBound(_x2, r, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    }
  }

  void __attribute__((always_inline))
  drawRgbTargetCenterLine(const int32_t _srcCol, const int32_t _srcRow, const ImageBuffer& _outImage, const uint32_t _rgb888) {
    const int32_t widthBot = 0;
    const int32_t widthTop = m_inImageDesc.m_width - 1;
    const int32_t heightBot = 0;
    const int32_t heightTop = m_inImageDesc.m_height - 1;

    for (int adj = 0; adj < 100; ++adj) {
      drawOutputPixelBound(_srcCol, _srcRow - adj, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol, _srcRow + adj, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    }
  }

  void __attribute__((always_inline))
  drawRgbTargetHorizontalCenterLine(const int32_t _srcCol, const int32_t _srcRow, const ImageBuffer& _outImage, const uint32_t _rgb888) {
    const int32_t widthBot = 0;
    const int32_t widthTop = m_inImageDesc.m_width - 1;
    const int32_t heightBot = 0;
    const int32_t heightTop = m_inImageDesc.m_height - 1;

    for (int adj = 0; adj < 100; ++adj) {
      drawOutputPixelBound(_srcCol - adj, _srcRow, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol + adj, _srcRow, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    }
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
    uint32_t* restrict src = (uint32_t*) _inImage.m_ptr;
    uint64_t* restrict dst = s_rgb888hsv;
#pragma MUST_ITERATE(4, , 4)
    while ((uint8_t*) src < ((uint8_t*) _inImage.m_ptr) + m_inImageDesc.m_width * m_inImageDesc.m_height * 2) {
      const uint64_t rgb = convert2xYuyvToRgb888(*src);
      *dst++ = _itoll(_loll(rgb), convertRgb888ToHsv(_loll(rgb)));
      *dst++ = _itoll(_hill(rgb), convertRgb888ToHsv(_hill(rgb)));
      src++;
    }
  }

  void proceedImageHsv(ImageBuffer& _outImage) {
    const uint64_t* restrict rgb888hsvptr = s_rgb888hsv;

    const uint32_t width = m_inImageDesc.m_width;
    const uint32_t height = m_inImageDesc.m_height;
    const uint32_t dstLineLength = m_outImageDesc.m_lineLength;

    const int32_t* restrict p_hi2ho_out = s_hi2ho_out;
    const int32_t* restrict p_hi2ho_cstr = s_hi2ho_cstr;
    assert(m_outImageDesc.m_height % 4 == 0); // verified in setup
#pragma MUST_ITERATE(4, , 4)
    for (uint32_t srcRow = 0; srcRow < height; srcRow++) {
      const uint32_t dstRow = *(p_hi2ho_out++);
      const uint32_t cstrRow = *(p_hi2ho_cstr++);

      uint16_t* restrict dstImageRow = reinterpret_cast<uint16_t*>(_outImage.m_ptr + dstRow * dstLineLength);
      uint16_t* restrict clustermapRow = reinterpret_cast<uint16_t*>(s_clustermap + cstrRow * m_clustermapDesc.m_width);

      const int32_t* restrict p_wi2wo_out = s_wi2wo_out;
      const int32_t* restrict p_wi2wo_cstr = s_wi2wo_cstr;
#pragma MUST_ITERATE(32, , 32)
      for (uint32_t srcCol = 0; srcCol < width; srcCol++) {
        const uint32_t dstCol = *(p_wi2wo_out++);
        const uint32_t cstrCol = *(p_wi2wo_cstr++);
        const uint64_t rgb888hsv = *rgb888hsvptr++;

        uint16_t clusterNum = m_clusterizer.getMinEqCluster(*(clustermapRow + cstrCol));
        const bool det = clusterNum;

        writeOutputPixel(dstImageRow + dstCol, det ? 0x00ffff : _hill(rgb888hsv));
      }
    }
  }

public:
  virtual bool setup(const ImageDesc& _inImageDesc, const ImageDesc& _outImageDesc, int8_t* _fastRam, size_t _fastRamSize) {
    m_inImageDesc = _inImageDesc;
    m_outImageDesc = _outImageDesc;

    m_minTargetSize = m_inImageDesc.m_width * m_inImageDesc.m_height / 100; // 1% of screen

    m_inRgb888HsvImgDesc.m_width = m_inImageDesc.m_width;
    m_inRgb888HsvImgDesc.m_height = m_inImageDesc.m_height;
    m_inRgb888HsvImgDesc.m_lineLength = m_inImageDesc.m_width * sizeof(uint64_t);
    m_inRgb888HsvImgDesc.m_format = VideoFormat::RGB888HSV;

    m_bitmapDesc.m_width = m_inImageDesc.m_width / METAPIX_SIZE;
    m_bitmapDesc.m_height = m_inImageDesc.m_height / METAPIX_SIZE;
    m_bitmapDesc.m_lineLength = m_bitmapDesc.m_width * sizeof(uint16_t);
    m_bitmapDesc.m_format = VideoFormat::MetaBitmap;

    m_clustermapDesc = m_bitmapDesc; // i suppose

    m_bitmapBuilder.setup(m_inRgb888HsvImgDesc, m_bitmapDesc, _fastRam, _fastRamSize);
    m_clusterizer.setup(m_bitmapDesc, m_clustermapDesc, _fastRam, _fastRamSize);

    m_inRgb888HsvImg.m_ptr = reinterpret_cast<int8_t*>(s_rgb888hsv);
    m_inRgb888HsvImg.m_size = IMG_WIDTH * IMG_HEIGHT * sizeof(uint64_t);

    m_bitmap.m_ptr = reinterpret_cast<int8_t*>(s_bitmap);
    m_bitmap.m_size = IMG_WIDTH * IMG_HEIGHT * sizeof(uint16_t);

    m_clustermap.m_ptr = reinterpret_cast<int8_t*>(s_clustermap);
    m_clustermap.m_size = IMG_WIDTH * IMG_HEIGHT * sizeof(uint16_t);

    if (m_inImageDesc.m_width % 32 != 0 || m_inImageDesc.m_height % 4 != 0)
      return false;

#define min(x, y) x < y ? x : y;
    const double srcToDstShift =
      min(static_cast<double>(m_outImageDesc.m_width) / m_inImageDesc.m_width, static_cast<double>(m_outImageDesc.m_height) / m_inImageDesc.m_height);

    const uint32_t widthIn = _inImageDesc.m_width;
    // width step for out image
    int32_t* restrict p_wi2wo_out = s_wi2wo_out;
    for (int i = 0; i < widthIn; i++)
      *(p_wi2wo_out++) = i * srcToDstShift;
    // width step for cluster map
    int32_t* restrict p_wi2wo_cstr = s_wi2wo_cstr;
    for (int i = 0; i < widthIn; i++)
      *(p_wi2wo_cstr++) = i / METAPIX_SIZE;

    const uint32_t heightIn = _inImageDesc.m_height;
    // height step for out image
    int32_t* restrict p_hi2ho_out = s_hi2ho_out;
    for (int32_t i = 0; i < heightIn; i++)
      *(p_hi2ho_out++) = i * srcToDstShift;
    // height step for cluster map
    int32_t* restrict p_hi2ho_cstr = s_hi2ho_cstr;
    for (int32_t i = 0; i < heightIn; i++)
      *(p_hi2ho_cstr++) = i / METAPIX_SIZE;

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

    memset(s_clustermap, 0x00, m_clustermapDesc.m_width * m_clustermapDesc.m_height * sizeof(uint16_t));
    memset(s_bitmap, 0x00, m_bitmapDesc.m_width * m_bitmapDesc.m_height * sizeof(uint16_t));

#ifdef DEBUG_REPEAT
    for (unsigned repeat = 0; repeat < DEBUG_REPEAT; ++repeat) {
#endif

      if (m_inImageDesc.m_height > 0 && m_inImageDesc.m_width > 0) {
        convertImageYuyvToHsv(_inImage);

        bool autoDetectHsv = static_cast<bool>(_inArgs.auto_detect_hsv); // true or false
        if (autoDetectHsv) {
          HsvRangeDetector rangeDetector = HsvRangeDetector(m_inImageDesc.m_width, m_inImageDesc.m_height, m_detectZoneScale);
          rangeDetector.detect(_outArgs.detect_hue_from, _outArgs.detect_hue_to, _outArgs.detect_sat_from, _outArgs.detect_sat_to, _outArgs.detect_val_from,
            _outArgs.detect_val_to, s_rgb888hsv);
        }

        m_bitmapBuilder.run(m_inRgb888HsvImg, m_bitmap, _inArgs, _outArgs);
        m_clusterizer.run(m_bitmap, m_clustermap, _inArgs, _outArgs);

        proceedImageHsv(_outImage);
      }

#ifdef DEBUG_REPEAT
    } // repeat
#endif

    // draw taget pointer
    const int step = m_inImageDesc.m_height / m_detectZoneScale;
    const int hHeight = m_inImageDesc.m_height / 2;
    const int hWidth = m_inImageDesc.m_width / 2;

    drawRgbTargetCenterLine(hWidth - step, hHeight, _outImage, 0xff00ff);
    drawRgbTargetCenterLine(hWidth + step, hHeight, _outImage, 0xff00ff);
    drawRgbTargetCenterLine(hWidth - 2 * step, hHeight, _outImage, 0xff00ff);
    drawRgbTargetCenterLine(hWidth + 2 * step, hHeight, _outImage, 0xff00ff);

    drawRgbTargetHorizontalCenterLine(hWidth, hHeight - step, _outImage, 0xff00ff);
    drawRgbTargetHorizontalCenterLine(hWidth, hHeight + step, _outImage, 0xff00ff);
    drawRgbTargetHorizontalCenterLine(hWidth, hHeight - 2 * step, _outImage, 0xff00ff);
    drawRgbTargetHorizontalCenterLine(hWidth, hHeight + 2 * step, _outImage, 0xff00ff);

    // memset(_outArgs.target, 0, 8*sizeof(XDAS_Target));
    m_clustersAmount = m_clusterizer.getClustersAmount();
    bool noObjects = true;
    for (int i = 0; i < OBJECTS; i++) // defined in stdcpp.hpp
    {
      int size = std::sqrt(static_cast<float>(m_clusterizer.getSize(i)));
      const uint32_t targetRadius = std::ceil(size / 3.1415927f);
      size = static_cast<uint32_t>(targetRadius * 100 * 4) / static_cast<uint32_t>(m_bitmapDesc.m_width + m_bitmapDesc.m_height);
      if (size > 4) { // it's better to be about 0.5% of image
        noObjects = false;
        int x = m_clusterizer.getX(i);
        int y = m_clusterizer.getY(i);

        drawFatPixel(x, y, _outImage, 0xff0000);

        _outArgs.targets[i].size = size;
        _outArgs.targets[i].x = ((x - static_cast<int32_t>(m_inImageDesc.m_width) / 2) * 100 * 2) / static_cast<int32_t>(m_inImageDesc.m_width);
        _outArgs.targets[i].y = ((y - static_cast<int32_t>(m_inImageDesc.m_height) / 2) * 100 * 2) / static_cast<int32_t>(m_inImageDesc.m_height);
      }
    }

    if (noObjects) {
      _outArgs.targets[0].x = 0;
      _outArgs.targets[0].y = 0;
      _outArgs.targets[0].size = 0;
    }

    return true;
  }
};

uint16_t* restrict ObjectSensorCvAlgorithm::s_mult43_div = NULL;
uint16_t* restrict ObjectSensorCvAlgorithm::s_mult255_div = NULL;

}
}

#endif
