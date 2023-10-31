#ifndef TRIK_CLUSTERIZER_HPP_
#define TRIK_CLUSTERIZER_HPP_

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
#include <set>
#include <vector>

#include "image.hpp"

namespace trik {
namespace sensors {

typedef struct Target {
  int32_t x;
  int32_t y;
  int32_t size;
} Target;

static inline bool compareTargetBySize(const Target& a, const Target& b) { return a.size > b.size; }

class ClusterizerCvAlgorithm : public CvAlgorithm<VideoFormat::Unknown, VideoFormat::Unknown> {
private:
  ImageDesc m_inImageDesc;
  ImageDesc m_outImageDesc;

  std::vector<uint16_t> equalClusters;
  std::vector<Target> clusters;

  uint16_t m_maxCluster;

  uint16_t min2(uint16_t* envPixs) {
    uint16_t v = envPixs[0];
    for (int n = 1; n < 4; n++)
      if ((envPixs[n] < v && envPixs[n] != 0) || v == 0)
        v = envPixs[n];

    return v;
  }

  bool isClustersEqual(const uint16_t cluster1, const uint16_t cluster2) {
    if (cluster1 == cluster2)
      return true;
    else
      return (equalClusters[cluster1] == equalClusters[cluster2]);
  }

  /*
   ________
  |a2|a3|a4|
  |a1|p |  |
  |  |  |  |
   --------
  */

  void setPixelEnvironment(uint16_t* pixPtr, uint16_t* a, int r, int c) {
    const uint32_t width = m_inImageDesc.m_width;

    if (r != 0) {
      a[2] = *(pixPtr - width);
      if (c != 0)
        a[1] = *(pixPtr - width - 1);
      if (c != width - 1)
        a[3] = *(pixPtr - width + 1);
    }
    if (c != 0)
      a[0] = *(pixPtr - 1);
  }

  void setClusterNum(uint16_t* pixPtr, int r, int c) {
    uint16_t a[ENV_PIXS];
    memset(a, 0x0, ENV_PIXS * sizeof(uint16_t));
    setPixelEnvironment(pixPtr, a, r, c);

    uint16_t localMinCluster;
    if (localMinCluster = min2(a)) {
      *pixPtr = localMinCluster;
      clusters[localMinCluster].x += c;
      clusters[localMinCluster].y += r;
      clusters[localMinCluster].size++;

#pragma MUST_ITERATE(4, , 4)
      for (int i = 0; i < ENV_PIXS; i++)
        if (a[i]) // if no bg
          if (!isClustersEqual(a[i], localMinCluster))
            equalClusters[a[i]] = equalClusters[localMinCluster];

    } else { // no clusters around
      *pixPtr = m_maxCluster;
      equalClusters.push_back(m_maxCluster);

      Target cluster;
      memset(&cluster, 0, sizeof(Target));
      clusters.push_back(cluster);
      m_maxCluster++;
    }
  }

  void postProcessing() // link clusters
  {
    for (int i = 0; i < equalClusters.size(); i++) {
      if (i != equalClusters[i]) {
        clusters[equalClusters[i]].x += clusters[i].x;
        clusters[equalClusters[i]].y += clusters[i].y;
        clusters[equalClusters[i]].size += clusters[i].size;
        clusters[i].size = 0;
      }
    }

    std::sort(clusters.begin(), clusters.end(), compareTargetBySize);
  }

public:
  uint16_t getMinEqCluster(uint16_t i) { return equalClusters[i]; }

  uint16_t getClustersAmount() {
    int amount = 0;
    for (int i = 0; i < equalClusters.size(); i++)
      if (clusters[i].size > 0)
        amount++;
    return amount; // others have 0 size
  }

  int32_t getX(int i) { return (clusters[i].x / (clusters[i].size + 1)) * METAPIX_SIZE; }

  int32_t getY(int i) { return (clusters[i].y / (clusters[i].size + 1)) * METAPIX_SIZE; }

  uint16_t getSize(int i) { return clusters[i].size; }

  virtual bool setup(const ImageDesc& _inImageDesc, const ImageDesc& _outImageDesc, int8_t* _fastRam, size_t _fastRamSize) {
    m_inImageDesc = _inImageDesc;
    m_outImageDesc = _outImageDesc;

    if (m_inImageDesc.m_width % 32 != 0 || m_inImageDesc.m_height % 4 != 0)
      return false;

    return true;
  }

  virtual bool run(const ImageBuffer& _inImage, ImageBuffer& _outImage, const trik_cv_algorithm_in_args& _inArgs, trik_cv_algorithm_out_args& _outArgs) {
    equalClusters.clear();
    clusters.clear();

    // 0 for BG
    equalClusters.push_back(0);
    Target bg;
    memset(&bg, 0, sizeof(Target));
    clusters.push_back(bg);
    m_maxCluster = 1;

    const uint16_t* restrict srcImgPtr = reinterpret_cast<uint16_t*>(_inImage.m_ptr);
    uint16_t* restrict dstImgPtr = reinterpret_cast<uint16_t*>(_outImage.m_ptr);

    for (int srcRow = 0; srcRow < m_inImageDesc.m_height; srcRow++) {
      for (int srcCol = 0; srcCol < m_inImageDesc.m_width; srcCol++) {
        if (pop(*(srcImgPtr++)) > METAPIX_SIZE / 2) // metapix detected if there are more than N pixels
          setClusterNum(dstImgPtr, srcRow, srcCol);

        dstImgPtr++;
      }
    }

    postProcessing();
    return true;
  }
};

}
}

#endif
