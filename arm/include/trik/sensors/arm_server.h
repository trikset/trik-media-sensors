#ifndef TRIK_SENSORS_ARM_SERVER_
#define TRIK_SENSORS_ARM_SERVER_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <trik/sensors/cv_algorithm.h>

int trik_init_arm_server(uint16_t rproc_id);
int trik_destroy_arm_server(void);

void* trik_start_arm_server(void* _arg);
int trik_req_step(struct trik_cv_algorithm_out_args* out_args, struct trik_cv_algorithm_in_args in_args);
#ifdef __cplusplus
}
#endif

#endif
