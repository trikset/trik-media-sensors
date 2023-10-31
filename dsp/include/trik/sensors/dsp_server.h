#ifndef TRIK_SENSORS_DSP_SERVER_H_
#define TRIK_SENSORS_DSP_SERVER_H_

#if defined(__cplusplus)
extern "C" {
#endif

#include <xdc/std.h>

Int trik_init_dsp_server(Void);
Int trik_destroy_dsp_server(Void);

Int trik_start_dsp_server(Void);

#if defined(__cplusplus)
}
#endif

#endif
