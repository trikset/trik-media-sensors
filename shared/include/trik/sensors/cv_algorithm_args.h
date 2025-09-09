#ifndef TRIK_SENSORS_CV_ALGORITHM_ARGS_H_
#define TRIK_SENSORS_CV_ALGORITHM_ARGS_H_

#if defined(__cplusplus)
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define TRIK_MAX_TARGET_COUNT 8

typedef struct trik_cv_algorithm_in_args {
  uint16_t detect_hue_from; // [0..359]
  uint16_t detect_hue_to;   // [0..359]
  uint8_t detect_sat_from;  // [0..100]
  uint8_t detect_sat_to;    // [0..100]
  uint8_t detect_val_from;  // [0..100]
  uint8_t detect_val_to;    // [0..100]
  bool auto_detect_hsv;     // [true|false]
  uint16_t width_n;         // [1..320]
  uint16_t height_n;        // [1..240]
} trik_cv_algorithm_in_args;

typedef struct trik_cv_algorithm_out_target {
  int16_t x;
  int16_t y;
  uint16_t size;
} trik_cv_algorithm_out_target;

typedef struct trik_cv_algorithm_out_args {
  struct trik_cv_algorithm_out_target targets[TRIK_MAX_TARGET_COUNT];
  uint16_t detect_hue_from; // [0..359]
  uint16_t detect_hue_to;   // [0..359]
  uint8_t detect_sat_from;  // [0..100]
  uint8_t detect_sat_to;    // [0..100]
  uint8_t detect_val_from;  // [0..100]
  uint8_t detect_val_to;    // [0..100]
} trik_cv_algorithm_out_args;

#if defined(__cplusplus)
}
#endif

#endif
