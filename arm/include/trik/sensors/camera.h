#ifndef TRIK_SENSORS_CAMERA_
#define TRIK_SENSORS_CAMERA_

#include <stdint.h>
#include <trik/buffer.h>

int trik_init_camera(uint8_t buffer_count, char* dev_name);
int trik_destroy_camera(void);
int trik_camera_wait_for_frame(struct buffer* buffer);
int trik_release_frame();

#endif
