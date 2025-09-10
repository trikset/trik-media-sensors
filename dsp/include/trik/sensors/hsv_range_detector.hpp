#ifndef TRIK_SENSORS_HSV_RANGE_DETECTOR_HPP_
#define TRIK_SENSORS_HSV_RANGE_DETECTOR_HPP_

#ifndef __cplusplus
#error C++-only header
#endif

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <c6x.h>
#include <cassert>
#include <cmath>

namespace trik {
namespace sensors {

typedef struct ColorRange {
  /*
    uint8_t h0;
    uint8_t h1;
    uint8_t s0;
    uint8_t s1;
  */
  uint8_t v0;
  uint8_t v1;
} ColorRange;

typedef struct Cluster {
  /*
    uint8_t h;
    uint8_t s;
  */
  uint8_t v;
} Image;

typedef struct Roi {
  uint16_t left_p;
  uint16_t left_n;
  /*
    uint16_t top_p;
    uint16_t top_n;
  */
  uint16_t right_p;
  uint16_t right_n;
  /*
    uint16_t bot_p;
    uint16_t bot_n;
  */
} Roi;

typedef struct ImageData {
  uint16_t width;
  uint16_t height;
} ImageData;

typedef union U_Hsv8x3 {
  struct {
    uint8_t h;
    uint8_t s;
    uint8_t v;
    uint8_t none;
  } parts;

  uint32_t whole;
} U_Hsv8x3;

class HsvRangeDetector {
private:
  static const int cstrs_max = 256;  // 256/1
  static const int cstrs_max_ = 255; //
  static const int pos_shift = 0;    // 1 == 2^0

  static int32_t s_hsvClusters[cstrs_max];
  ImageData m_image;
  Roi m_roi;
  Cluster m_maxFillCluster;
  int32_t m_maxFillClusterValue;

  // penalty coeffs
  static const int K = 200;
  static const int K0 = 1;
  static const int K1 = 1;
  static const int K2 = 2;

  const double T_end = 0.0005;
  const double lambda = 0.76;
  const double e = 2.718281828;

  int do_getIncrement(int _val, int _min, int _max, double _base, double _t) {
    assert(_min <= _max);
    if (_min == _max)
      return _min;
    else {
      int res = 0;
      double alpha = rand() / static_cast<double>(RAND_MAX);
      double degree = 2 * alpha - 1;
      res = _val + ((pow(_base, degree) - 1) * _t) * static_cast<double>(_max - _min);

      if ((res < _min) || (res > _max))
        return do_getIncrement(_val, _min, _max, _base, _t);
      else
        return res;
    }
  }

  ColorRange getIncrement(ColorRange _C, double _T) {

    double base = 1 + 1 / _T;
    ColorRange newC = {
      /*
              newC.h0 = do_getIncrement(_C.h0, 0, cstrs_max_, base, _T),
              newC.h1 = do_getIncrement(_C.h1, 0, cstrs_max_, base, _T),
              newC.s0 = do_getIncrement(_C.s0, 0, m_maxFillCluster.s, base, _T),
              newC.s1 = do_getIncrement(_C.s1, m_maxFillCluster.s, cstrs_max_, base, _T),
      */
      newC.v0 = do_getIncrement(_C.v0, 0, cstrs_max_, base, _T),
      newC.v1 = do_getIncrement(_C.v1, 0, cstrs_max_, base, _T),
    };

    return newC;
  }

  uint64_t F(ColorRange C) {
    //      ColorRange* C = _C;
    int64_t res = 0;

    for (int v = C.v0; v <= C.v1; v++)
      res += s_hsvClusters[v] != 0 ? s_hsvClusters[v] : -K0;
    /*
          if (C.h0 <= C.h1)
            for(int h = C.h0; h <= C.h1; h++)
              for(int s = C.s0; s <= C.s1; s++)
                for(int v = C.v0; v <= C.v1; v++)
                  res += s_hsvClusters[h][s][v] != 0 ? s_hsvClusters[h][s][v] : -K0;
            else { // h1 > h2
              for(int h = C.h0; h < cstrs_max; h++)
                for(int s = C.s0; s <= C.s1; s++)
                  for(int v = C.v0; v <= C.v1; v++)
                    res += s_hsvClusters[h][s][v] != 0 ? s_hsvClusters[h][s][v] : -K0;
              for(int h = 0; h <= C.h1; h++)
                for(int s = C.s0; s <= C.s1; s++)
                  for(int v = C.v0; v <= C.v1; v++)
                    res += s_hsvClusters[h][s][v] != 0 ? s_hsvClusters[h][s][v] : -K0;
            }
    */

    return res;
  }

  void initImg(int _imgWidth, int _imgHeight, int _step) {
    m_image.width = _imgWidth;
    m_image.height = _imgHeight;

    uint16_t hWidth = _imgWidth / 2;

    // ROI bounds
    m_roi.left_p = hWidth - _step;
    m_roi.right_p = hWidth + _step;
    /*
          m_roi.top_p   = 0;
          m_roi.bot_p   = _imgHeight;
    */

    // Ih bounds
    m_roi.left_n = m_roi.left_p - _step;
    m_roi.right_n = m_roi.right_p + _step;
    /*
          m_roi.top_n   = 0;
          m_roi.bot_n   = _imgHeight;
    */
  }

public:
  HsvRangeDetector(int _imgWidth, int _imgHeight, int _detectZoneScale) { initImg(_imgWidth, _imgHeight, _detectZoneScale); }

  void detect(uint16_t& _hFrom, uint16_t& _hTo, uint8_t& _sFrom, uint8_t& _sTo, uint8_t& _vFrom, uint8_t& _vTo, uint64_t* _rgb888hsv) {
    // initialize stuff
    srand(time(NULL));

    const uint64_t* restrict img = _rgb888hsv;

    // initialize Clusters
    memset(s_hsvClusters, 0, cstrs_max * sizeof(int32_t));

    // initialize variables for Cluster with highest occurrence
    // m_maxFillCluster
    m_maxFillClusterValue = 0;

    // Clusterize image
    U_Hsv8x3 pixel;
    Cluster currCluster;

    for (int row = 0; row < m_image.height; row++) {
      for (int col = 0; col < m_image.width; col++) {
        pixel.whole = _loll(*(img)++);
        /*
                  currCluster.h = (pixel.parts.h >> pos_shift);
                  currCluster.s = (pixel.parts.s >> pos_shift);
        */
        currCluster.v = (pixel.parts.v >> pos_shift);

        // positive part of image
        if (m_roi.left_p < col && m_roi.right_p > col) {
          s_hsvClusters[currCluster.v] += K1;

          // remember Cluster with highest positive occurrence
          if (s_hsvClusters[currCluster.v] > m_maxFillClusterValue) {
            m_maxFillClusterValue = s_hsvClusters[currCluster.v];
            m_maxFillCluster = currCluster;
          }
        } // negative part of image
        else if (m_roi.left_n > col || m_roi.right_n < col) {
          s_hsvClusters[currCluster.v] -= K2;
        }
      }
    }

    // algorithm
    ColorRange C;
    ColorRange newC;
    memset(&newC, 0, sizeof(Cluster));
    // initial hsv range
    /*
          C.h0 =
          C.h1 = m_maxFillCluster.h;
          C.s0 =
          C.s1 = m_maxFillCluster.s;
    */
    C.v0 = C.v1 = m_maxFillCluster.v;

    int64_t L = F(C);
    int64_t newL = 0;
    double T = 150;

    while (T > T_end) {
#pragma MUST_ITERATE(200, , 200)
      for (int i = 0; i < K; i++) {
        newC = getIncrement(C, T);
        newL = F(newC);

        if (rand() <= pow(e, (newL - L) / T) * RAND_MAX) {
          C = newC;
          L = newL;
        }
      }
      T *= lambda;
    }
    /*
          C.h0 = (C.h0 << pos_shift)*1.4f;
          C.h1 = (((C.h1+1) << pos_shift) - 1)*1.4f;

          C.s0 = (C.s0 << pos_shift)*0.39f;
          C.s1 = (((C.s1+1) << pos_shift) - 1)*0.39f;
    */
    C.v0 = (C.v0 << pos_shift) * 0.39f;
    C.v1 = (((C.v1 + 1) << pos_shift)) * 0.39f;
    /*
          if (C.h0 <= C.h1) {
            _h    = (C.h1 + C.h0) / 2;
            _hTol = (C.h1 - C.h0) / 2;
          }
          else {
            float hue = (C.h1 - (360.0f - C.h0)) / 2;
            float hueTolerance = (C.h1 + (360.0f - C.h0)) / 2;
            _h = hue >= 0 ? hue : (hue + 360);
            _hTol = hueTolerance;
          }

          _s = (C.s1 + C.s0) / 2;
          _sTol = (C.s1 - C.s0) / 2;
    */
    _hFrom = 0;
    _hTo = 0;
    _sFrom = 0;
    _sTo = 0;

    uint8_t _v = (C.v1 + C.v0) / 2;
    uint8_t _vTol = (C.v1 - C.v0) / 2;

    _vFrom = makeValueRange(_v, -_vTol, 0, 100);
    _vTo = makeValueRange(_v, +_vTol, 0, 100);
  }
};

int32_t restrict HsvRangeDetector::s_hsvClusters[HsvRangeDetector::cstrs_max];

}
}

#endif
