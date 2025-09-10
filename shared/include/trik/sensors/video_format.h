#ifndef TRIK_SENSORS_VIDEO_FORMAT_
#define TRIK_SENSORS_VIDEO_FORMAT_

#if defined(__cplusplus)
extern "C" {
#endif

#define METAPIX_SIZE 4
#define ENV_PIXS 4

enum VideoFormat { 
    Unknown = 0, 
    RGB888, 
    RGB888HSV,
    RGB565, 
    RGB565X, 
    YUV444,
    YUV422, 
    YUV422P, 
    NV16, 
    MetaBitmap 
};


#if defined(__cplusplus)
}
#endif

#endif
