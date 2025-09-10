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
#include "hsv_range_detector_object.hpp"
#include <trik/sensors/video_format.h>

namespace trik {
namespace sensors {

static uint16_t s_bitmap[IMG_WIDTH * IMG_HEIGHT];
static uint16_t s_clustermap[IMG_WIDTH * IMG_HEIGHT];

static int32_t s_wi2wo_out[IMG_WIDTH];
static int32_t s_hi2ho_out[IMG_HEIGHT];
static int32_t s_wi2wo_cstr[IMG_WIDTH];
static int32_t s_hi2ho_cstr[IMG_HEIGHT];

#define OBJECTS 8

class ObjectSensorCvAlgorithm : public CvAlgorithm<VideoFormat::YUV422, VideoFormat::RGB565X> {
private:
  static const int m_detectZoneScale = 6;

  int32_t m_targetX;
  int32_t m_targetY;
  uint32_t m_targetPoints;

  uint16_t m_minTargetSize;

  uint16_t m_clustersAmount;

  ImageDesc m_bitmapDesc;
  BitmapBuilderCvAlgorithm m_bitmapBuilder;
  ImageBuffer m_bitmap;

  ImageDesc m_clustermapDesc;
  ClusterizerCvAlgorithm m_clusterizer;
  ImageBuffer m_clustermap;

  ImageDesc m_inRgb888HsvImgDesc;
  ImageBuffer m_inRgb888HsvImg;

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
    if (!commonSetup(_inImageDesc, _outImageDesc, _fastRam, _fastRamSize))
      return false;
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
        (this->*convertImageFormatToHSV)(_inImage);

        bool autoDetectHsv = static_cast<bool>(_inArgs.auto_detect_hsv); // true or false
        if (autoDetectHsv) {
          HsvRangeDetectorObject rangeDetector = HsvRangeDetectorObject(m_inImageDesc.m_width, m_inImageDesc.m_height, m_detectZoneScale);
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

        _outArgs.targets[i].out_target.targetLocation.size = size;
        _outArgs.targets[i].out_target.targetLocation.x = ((x - static_cast<int32_t>(m_inImageDesc.m_width) / 2) * 100 * 2) / static_cast<int32_t>(m_inImageDesc.m_width);
        _outArgs.targets[i].out_target.targetLocation.y = ((y - static_cast<int32_t>(m_inImageDesc.m_height) / 2) * 100 * 2) / static_cast<int32_t>(m_inImageDesc.m_height);
      }
    }

    if (noObjects) {
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
