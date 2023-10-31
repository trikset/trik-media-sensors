#ifndef TRIK_SENSORS_LOG_H_
#define TRIK_SENSORS_LOG_H_

#include <stdio.h>

#ifdef DEBUG

#define debugf(...)                                                                                                                                            \
  do {                                                                                                                                                         \
    printf("D %s(): ", __func__);                                                                                                                              \
    printf(__VA_ARGS__);                                                                                                                                       \
    printf("\n");                                                                                                                                              \
    fflush(stdout);                                                                                                                                            \
  } while (0)
#define infof(...)                                                                                                                                             \
  do {                                                                                                                                                         \
    printf(__VA_ARGS__);                                                                                                                                       \
    printf("\n");                                                                                                                                              \
    fflush(stdout);                                                                                                                                            \
  } while (0)
#define warnf(...)                                                                                                                                             \
  do {                                                                                                                                                         \
    printf("W %s(): ", __func__);                                                                                                                              \
    printf(__VA_ARGS__);                                                                                                                                       \
    printf("\n");                                                                                                                                              \
    fflush(stdout);                                                                                                                                            \
  } while (0)
#define errorf(...)                                                                                                                                            \
  do {                                                                                                                                                         \
    fprintf(stderr, "E %s(): ", __func__);                                                                                                                     \
    fprintf(stderr, __VA_ARGS__);                                                                                                                              \
    fprintf(stderr, "\n");                                                                                                                                     \
    fflush(stderr);                                                                                                                                            \
  } while (0)

#else

#define debugf(...)
#define infof(...)                                                                                                                                             \
  do {                                                                                                                                                         \
    printf(__VA_ARGS__);                                                                                                                                       \
    printf("\n");                                                                                                                                              \
  } while (0)
#define warnf(...)                                                                                                                                             \
  do {                                                                                                                                                         \
    printf("W %s(): ", __func__);                                                                                                                              \
    printf(__VA_ARGS__);                                                                                                                                       \
    printf("\n");                                                                                                                                              \
  } while (0)
#define errorf(...)                                                                                                                                            \
  do {                                                                                                                                                         \
    fprintf(stderr, "E %s(): ", __func__);                                                                                                                     \
    fprintf(stderr, __VA_ARGS__);                                                                                                                              \
    fprintf(stderr, "\n");                                                                                                                                     \
  } while (0)

#endif

#endif