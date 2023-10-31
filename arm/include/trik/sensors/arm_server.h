#ifndef TRIK_SENSORS_ARM_SERVER_
#define TRIK_SENSORS_ARM_SERVER_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <trik/sensors/cv_algorithm.h>

int trik_init_arm_server(uint16_t rproc_id);
int trik_destroy_arm_server(void);

int trik_start_arm_server(enum trik_cv_algorithm cv_algorithm, char* dev_name, char* config_filename);

#ifdef __cplusplus
}
#endif

#endif
