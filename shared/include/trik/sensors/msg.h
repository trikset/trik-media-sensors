#ifndef TRIK_SENSORS_MSG_H_
#define TRIK_SENSORS_MSG_H_

#if defined(__cplusplus)
extern "C" {
#endif

#include "cmd.h"
#include "cv_algorithm_args.h"
#include <trik/sensors/video_format.h>

struct trik_msg {
  MessageQ_MsgHeader reserved;
  enum trik_cmd cmd;
};

struct trik_res_init_msg {
  struct trik_msg header;

  /* Physical address of the DSP input buffer region (TRIK_INPUT_TOTAL
   * contiguous BUFFER_SIZE slots). The host maps the whole region via
   * /dev/mem and hands the per-slot virtual addresses to V4L2 as USERPTR
   * targets, so the VPIF DMA engine writes frames straight into DSP memory. */
  void* dsp_in_buffer;

  /* Physical address of the DSP output buffer (host maps it via /dev/mem). */
  void* dsp_out_buffer;
};

struct trik_req_cv_algorithm_msg {
  struct trik_msg header;
  enum VideoFormat video_format;
  uint32_t line_length;
};

struct trik_res_step_msg {
  struct trik_msg header;

  /* Index of the input buffer (0..TRIK_INPUT_TOTAL-1) the DSP must read. */
  uint32_t buffer_idx;

  struct trik_cv_algorithm_out_args out_args;
  struct trik_cv_algorithm_in_args in_args;
};

#define max(a, b) (((a) > (b)) ? (a) : (b))
#define TRIK_MSG_SIZE max(sizeof(struct trik_req_cv_algorithm_msg), sizeof(struct trik_res_step_msg))

#define TRIK_MSG_HEAP_ID 0
#define TRIK_HOST_MSG_QUE_NAME "HOST:MsgQ:01"
#define TRIK_SLAVE_MSG_QUE_NAME "%s:MsgQ:01" /* %s is each slave's Proc Name */

#if defined(__cplusplus)
}
#endif

#endif
