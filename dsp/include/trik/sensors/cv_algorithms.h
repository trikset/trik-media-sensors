#ifndef TRIK_SENSORS_CV_ALGORITHMS_H_
#define TRIK_SENSORS_CV_ALGORITHMS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <trik/buffer.h>
#include <trik/sensors/cv_algorithm.h>
#include <trik/sensors/cv_algorithm_args.h>
#include <trik/sensors/video_format.h>

int trik_init_cv_algorithm(enum trik_cv_algorithm algorithm, enum VideoFormat video_format, uint32_t line_length);
int trik_run_cv_algorithm(enum trik_cv_algorithm algorithm, struct buffer in_buffer, struct buffer out_buffer, struct trik_cv_algorithm_in_args in_args,
  struct trik_cv_algorithm_out_args* out_args);

#ifdef __cplusplus
}
#endif

#endif
