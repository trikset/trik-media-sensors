#ifndef TRIK_SENSORS_VIDEOFORMAT_
#define TRIK_SENSORS_VIDEOFORMAT_

#ifndef __cplusplus
#error C++-only header
#endif

#define METAPIX_SIZE 4
#define ENV_PIXS 4

namespace trik {
namespace sensors {

enum class VideoFormat { Unknown = 0, RGB888, RGB888HSV, RGB565, RGB565X, YUV444, YUV422, YUV422P, MetaBitmap };

}
}

#endif
