#include "trik/sensors/thread_video.h"
#include "trik/buffer.h"
#include "trik/sensors/arm_server.h"
#include "trik/sensors/cv_algorithm_args.h"
#include "trik/sensors/module_fb.h"
#include "trik/sensors/module_v4l2.h"
#include "trik/sensors/runtime.h"
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <time.h>

static int threadVideoSelectLoop(Runtime* _runtime, V4L2Input* _v4l2, FBOutput* _fb) {
  int res;
  int maxFd = 0;
  fd_set fdsIn;
  static const struct timespec s_selectTimeout = { .tv_sec = 1, .tv_nsec = 0 };

  if (_runtime == NULL || _v4l2 == NULL || _fb == NULL)
    return EINVAL;

  FD_ZERO(&fdsIn);

  FD_SET(_v4l2->m_fd, &fdsIn);
  if (maxFd < _v4l2->m_fd)
    maxFd = _v4l2->m_fd;

  if ((res = pselect(maxFd + 1, &fdsIn, NULL, NULL, &s_selectTimeout, NULL)) < 0) {
    res = errno;
    fprintf(stderr, "pselect() failed: %d\n", res);
    return res;
  }

  if (!FD_ISSET(_v4l2->m_fd, &fdsIn)) {
    fprintf(stderr, "pselect() did not select V4L2\n");
    return EBUSY;
  }

  const void* frameSrcPtr;
  size_t frameSrcSize;
  size_t frameSrcIndex;
  if ((res = v4l2InputGetFrame(_v4l2, &frameSrcPtr, &frameSrcSize, &frameSrcIndex)) != 0) {
    fprintf(stderr, "v4l2InputGetFrame() failed: %d\n", res);
    return res;
  }

  void* frameDstPtr;
  size_t frameDstSize;
  if ((res = fbOutputGetFrame(_fb, &frameDstPtr, &frameDstSize)) != 0) {
    fprintf(stderr, "fbOutputGetFrame() failed: %d\n", res);
    return res;
  }

  trik_cv_algorithm_in_args targetDetectParams;

  trik_cv_algorithm_in_args targetDetectParamsResult;
  if ((res = runtimeGetTargetDetectParams(_runtime, &targetDetectParams)) != 0) {
    fprintf(stderr, "runtimeGetTargetDetectParams() failed: %d\n", res);
    return res;
  }

  TargetDetectCommand targetDetectCommand;
  if ((res = runtimeFetchTargetDetectCommand(_runtime, &targetDetectCommand)) != 0) {
    fprintf(stderr, "runtimeGetTargetDetectCommand() failed: %d\n", res);
    return res;
  }
  targetDetectParams.auto_detect_hsv = targetDetectCommand.m_cmd;

  bool videoOutEnable;
  if ((res = runtimeGetVideoOutParams(_runtime, &videoOutEnable)) != 0) {
    fprintf(stderr, "runtimeGetVideoOutParams() failed: %d\n", res);
    return res;
  }

  if ((res = runtimeGetMxnParams(_runtime, &(targetDetectParams.extra_inArgs.mxnParams))) != 0) {
    fprintf(stderr, "runtimeGetVideoOutParams() failed: %d\n", res);
    return res;
  }

  memcpy(_runtime->m_modules.m_dsp.dsp_in_buf->start, frameSrcPtr, frameSrcSize);

  trik_cv_algorithm_out_args targetArgs;


  struct timespec start, end;
  double elapsed;

  if (trik_req_step(&targetArgs, targetDetectParams) < 0) {
    printf("unable to proccess a frame on a DSP \n");
    return -1;
  }

  trik_cv_algorithm_out_target target;
  target = targetArgs.targets[0];
  if (videoOutEnable)
    memcpy(frameDstPtr, _runtime->m_modules.m_dsp.dsp_out_buf->start, BUFFER_SIZE_FOR_FB);

  if ((res = fbOutputPutFrame(_fb)) != 0) {
    fprintf(stderr, "fbOutputPutFrame() failed: %d\n", res);
    return res;
  }

  if ((res = v4l2InputPutFrame(_v4l2, frameSrcIndex)) != 0) {
    fprintf(stderr, "v4l2InputPutFrame() failed: %d\n", res);
    return res;
  }

  switch (targetDetectCommand.m_cmd) {
  case 1:
    if ((res = runtimeReportTargetDetectParams(_runtime, &targetArgs)) != 0) {
      fprintf(stderr, "runtimeReportTargetDetectParams() failed: %d\n", res);
      return res;
    }
    break;

  case 0:
  default:
    if (_runtime->m_config.m_rcConfig.m_sensorType == TRIK_CV_ALGORITHM_MXN_SENSOR) {
      if ((res = runtimeReportTargetColors(_runtime, &(target.out_target.targetColors))) != 0) {
        fprintf(stderr, "runtimeReportTargetColors() failed: %d\n", res);
        return res;
      }
    } else {
      if ((res = runtimeReportTargetLocation(_runtime, &(target.out_target.targetLocation))) != 0) {
        fprintf(stderr, "runtimeReportTargetLocation() failed: %d\n", res);
        return res;
      }
    }
    break;
  }

  return 0;
}

int threadVideo(Runtime* runtime) {
  int res = 0;
  V4L2Input* v4l2;
  FBOutput* fb;

  if (runtime == NULL) {
    res = EINVAL;
    goto exit;
  }

  if ((v4l2 = runtimeModV4L2Input(runtime)) == NULL || (fb = runtimeModFBOutput(runtime)) == NULL) {
    res = EINVAL;
    goto exit;
  }

  if ((res = v4l2InputOpen(v4l2, runtimeCfgV4L2Input(runtime))) != 0) {
    fprintf(stderr, "v4l2InputOpen() failed: %d\n", res);
    goto exit;
  }

  if ((res = fbOutputOpen(fb, runtimeCfgFBOutput(runtime))) != 0) {
    fprintf(stderr, "fbOutputOpen() failed: %d\n", res);
    goto exit_v4l2_close;
  }

  ImageDescription srcImageDesc;
  ImageDescription dstImageDesc;
  if ((res = v4l2InputGetFormat(v4l2, &srcImageDesc)) != 0) {
    fprintf(stderr, "v4l2InputGetFormat() failed: %d\n", res);
    goto exit_fb_close;
  }
  if ((res = fbOutputGetFormat(fb, &dstImageDesc)) != 0) {
    fprintf(stderr, "fbOutputGetFormat() failed: %d\n", res);
    goto exit_fb_close;
  }

  if ((res = trik_req_cv_algorithm(runtime->m_config, srcImageDesc.m_lineLength)) < 0) {
    fprintf(stderr,"failed to request a cv algorithm %d", res);
    goto exit;
  }

  if ((res = v4l2InputStart(v4l2)) != 0) {
    fprintf(stderr, "v4l2InputStart() failed: %d\n", res);
    goto exit_fb_close;
  }

  if ((res = fbOutputStart(fb)) != 0) {
    fprintf(stderr, "fbOutputStart() failed: %d\n", res);
    goto exit_v4l2_stop;
  }

  printf("Entering video thread loop\n");
  struct timespec start, end;
  double elapsed;
  while (!runtimeGetTerminate(runtime)) {
    if ((res = threadVideoSelectLoop(runtime, v4l2, fb)) != 0) {
      fprintf(stderr, "threadVideoSelectLoop() failed: %d\n", res);
      goto exit_fb_stop;
    }
  }
  printf("Exit video thread loop\n");

exit_fb_stop:
  if ((res = fbOutputStop(fb)) != 0)
    fprintf(stderr, "fbOutputStop() failed: %d\n", res);

exit_v4l2_stop:
  if ((res = v4l2InputStop(v4l2)) != 0)
    fprintf(stderr, "v4l2InputStop() failed: %d\n", res);

exit_fb_close:
  if ((res = fbOutputClose(fb)) != 0)
    fprintf(stderr, "fbOutputClose() failed: %d\n", res);

exit_v4l2_close:
  if ((res = v4l2InputClose(v4l2)) != 0)
    fprintf(stderr, "v4l2InputClose() failed: %d\n", res);

exit:
  runtimeSetTerminate(runtime);
  return res;
}
