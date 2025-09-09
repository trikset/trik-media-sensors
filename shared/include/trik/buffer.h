#ifndef TRIK_BUFFER_H_
#define TRIK_BUFFER_H_

#if defined (__cplusplus)
extern "C" {
#endif

#include <stddef.h>

// TEMP
#define IMG_WIDTH 320
#define IMG_HEIGHT 240
#define BUFFER_SIZE (IMG_WIDTH * IMG_HEIGHT * 2)
#define BUFFER_SIZE_FOR_FB ((IMG_WIDTH * IMG_HEIGHT * 2) * (240.0 / 320.0))

struct buffer {
  void *start;
  size_t length;
};

#if defined (__cplusplus)
}
#endif

#endif
