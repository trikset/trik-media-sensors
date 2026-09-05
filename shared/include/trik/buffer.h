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

// Capture buffers in the DSP input region. The VPIF DMA engine fills them via
// V4L2 USERPTR while the DSP processes the previous ones. There is one
// independent region per video port (video1/video2/usb-camera), each holding
// several buffers for triple buffering, so every camera streams into its own
// memory and several cameras can run at once.
#define TRIK_INPUT_REGIONS 3
#define TRIK_INPUT_BUFFERS 3
#define TRIK_INPUT_TOTAL (TRIK_INPUT_REGIONS * TRIK_INPUT_BUFFERS)

struct buffer {
  void *start;
  size_t length;
};

#if defined (__cplusplus)
}
#endif

#endif
