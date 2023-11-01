#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <linux/fb.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>

#include <ti/ipc/Std.h>
/* package header files */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/MessageQ.h>
#include <ti/ipc/transports/TransportRpmsg.h>

#include <ti/ipc/MultiProc.h>

#include <trik/sensors/camera.h>
#include <trik/sensors/cmd.h>
#include <trik/sensors/cv_algorithm.h>
#include <trik/sensors/log.h>
#include <trik/sensors/msg.h>

#define PAGE_SIZE 4096
#define CAMERA_BUFFER_COUNT 2

static enum trik_cmd trik_cmd_from_cv_algorithm(enum trik_cv_algorithm cv_algorithm) {
  if (cv_algorithm == TRIK_CV_ALGORITHM_MOTION_SENSOR)
    return TRIK_CMD_MOTION_SENSOR;
  else if (cv_algorithm == TRIK_CV_ALGORITHM_EDGE_LINE_SENSOR)
    return TRIK_CMD_EDGE_LINE_SENSOR;
  else if (cv_algorithm == TRIK_CV_ALGORITHM_LINE_SENSOR)
    return TRIK_CMD_LINE_SENSOR;
  else if (cv_algorithm == TRIK_CV_ALGORITHM_OBJECT_SENSOR)
    return TRIK_CMD_OBJECT_SENSOR;
  else
    return TRIK_CMD_NOP;
}

/* module structure */
typedef struct {
  MessageQ_Handle hostQue;   // created locally
  MessageQ_QueueId slaveQue; // opened remotely
  UInt16 heapId;             // MessageQ heapId
  UInt32 msgSize;
} App_Module;

/* private data */
static App_Module Module;

static int trik_init_rpmsg(uint16_t rproc_id) {
  int status = 0;
  MessageQ_Params msgqParams;
  char msgqName[32];

  Module.hostQue = NULL;
  Module.slaveQue = MessageQ_INVALIDMESSAGEQ;
  Module.heapId = TRIK_MSG_HEAP_ID;
  Module.msgSize = TRIK_MSG_SIZE /* sizeof(struct trik_msg) */;

  MessageQ_Params_init(&msgqParams);

  Module.hostQue = MessageQ_create(TRIK_HOST_MSG_QUE_NAME, &msgqParams);

  if (Module.hostQue == NULL) {
    errorf("failed creating MessageQ");
    return -1;
  }

  sprintf(msgqName, TRIK_SLAVE_MSG_QUE_NAME, MultiProc_getName(rproc_id));

  do {
    status = MessageQ_open(msgqName, &Module.slaveQue);
    sleep(1);
  } while (status == MessageQ_E_NOTFOUND);

  if (status < 0) {
    errorf("failed opening MessageQ");
    return -1;
  }
  return 0;
}

static int trik_destroy_rpmsg() {
  if (MessageQ_close(&Module.slaveQue) < 0)
    return -1;
  if (MessageQ_delete(&Module.hostQue) < 0)
    return -1;
  return 0;
}

static struct trik_msg* trik_create_msg(enum trik_cmd cmd) {
  struct trik_msg* msg = (struct trik_msg*) MessageQ_alloc(Module.heapId, Module.msgSize);
  if (msg == NULL)
    return NULL;

  msg->cmd = cmd;

  MessageQ_setReplyQueue(Module.hostQue, (MessageQ_Msg) msg);
  return msg;
}

static int trik_send_msg(struct trik_msg* msg) {
  if (msg == NULL)
    return -EINVAL;

  if (MessageQ_put(Module.slaveQue, (MessageQ_Msg) msg) < 0)
    return -1;

  debugf("sent 0x%x", ((struct trik_msg*) msg)->cmd);
  return 0;
}

static int trik_send_cmd(enum trik_cmd cmd) {
  struct trik_msg* msg = trik_create_msg(cmd);
  if (msg == NULL)
    return -ENOMEM;

  trik_send_msg(msg);

  return 0;
}

static int trik_wait_for_msg(struct trik_msg** msg) {
  if (MessageQ_get(Module.hostQue, (MessageQ_Msg*) msg, MessageQ_FOREVER) < 0)
    return -1;

  debugf("got 0x%x", (*msg)->cmd);
  return 0;
}

static int trik_destroy_msg(void* msg) {
  MessageQ_free((MessageQ_Msg) msg);
  return 0;
}

static int trik_wait_for_cmd(enum trik_cmd cmd) {
  struct trik_msg* msg;

  if (trik_wait_for_msg(&msg) < 0)
    return -1;
  if (msg->cmd != cmd)
    return -1;

  trik_destroy_msg(msg);
  return 0;
}

static int trik_fill_pipeline() {
  for (int i = 0; i < 3; i++)
    if (trik_send_cmd(TRIK_CMD_NOP) < 0)
      return -1;
  return 0;
}

static int trik_drain_pipeline() {
  for (int i = 0; i < 3; i++)
    if (trik_wait_for_cmd(TRIK_CMD_NOP) < 0)
      return -1;
  return 0;
}

static int8_t* trik_get_ptr_for_phys_addr(void* addr) {
  uint32_t page_base = ((uint32_t) addr) / PAGE_SIZE * PAGE_SIZE;
  uint32_t page_offset = ((uint32_t) addr) - page_base;

  int memfd = open("/dev/mem", O_RDWR | O_SYNC);
  int8_t* mapped_start = mmap(0, page_offset + BUFFER_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, memfd, page_base);

  if (mapped_start == MAP_FAILED)
    return NULL;

  return (int8_t*) (((uint32_t) mapped_start) + page_offset);
}

static int trik_req_init(struct buffer* dsp_in_buf, struct buffer* dsp_out_buf) {
  if (trik_send_cmd(TRIK_CMD_INIT) < 0)
    return -1;

  struct trik_res_init_msg* res;
  if (trik_wait_for_msg((struct trik_msg**) &res) < 0)
    return -1;

  int retval = 0;
  if ((dsp_in_buf->start = trik_get_ptr_for_phys_addr(res->dsp_in_buffer)) == NULL) {
    retval = -1;
    goto cleanup;
  }
  dsp_in_buf->length = BUFFER_SIZE;

  if ((dsp_out_buf->start = trik_get_ptr_for_phys_addr(res->dsp_out_buffer)) == NULL) {
    retval = -1;
    goto cleanup;
  }
  dsp_out_buf->length = BUFFER_SIZE;

cleanup:
  trik_destroy_msg(res);
  return retval;
}

static int trik_req_cv_algorithm(enum trik_cv_algorithm cv_algorithm, struct trik_cv_algorithm_in_args in_args) {
  enum trik_cmd cmd = trik_cmd_from_cv_algorithm(cv_algorithm);
  if (cmd == TRIK_CMD_NOP)
    return -1;

  struct trik_req_cv_algorithm_msg* req = (struct trik_req_cv_algorithm_msg*) trik_create_msg(cmd);

  req->in_args = in_args;

  if (trik_send_msg((struct trik_msg*) req) < 0)
    return -1;

  if (trik_wait_for_cmd(cmd) < 0)
    return -1;
  return 0;
}

static int trik_req_step(struct trik_cv_algorithm_out_args* out_args) {
  if (trik_send_cmd(TRIK_CMD_STEP) < 0)
    return -1;

  struct trik_res_step_msg* res;
  if (trik_wait_for_msg((struct trik_msg**) &res) < 0)
    return -1;

  *out_args = res->out_args;

  trik_destroy_msg(res);
  return 0;
}

static int trik_read_cv_algorithm_in_args_from_file(char* filename, struct trik_cv_algorithm_in_args* in_args) {
  FILE* f = fopen(filename, "r");

  char param[32];
  int32_t value;

  while (fscanf(f, "%s = %d", param, &value) > 0)
    if (strcmp(param, "detect_hue_from") == 0)
      in_args->detect_hue_from = value;
    else if (strcmp(param, "detect_hue_to") == 0)
      in_args->detect_hue_to = value;
    else if (strcmp(param, "detect_sat_from") == 0)
      in_args->detect_sat_from = value;
    else if (strcmp(param, "detect_sat_to") == 0)
      in_args->detect_sat_to = value;
    else if (strcmp(param, "detect_val_from") == 0)
      in_args->detect_val_from = value;
    else if (strcmp(param, "detect_val_to") == 0)
      in_args->detect_val_to = value;
    else if (strcmp(param, "auto_detect_hsv") == 0)
      in_args->auto_detect_hsv = value;

  fclose(f);
  return 0;
}

static int trik_setup_display(int8_t** fbp) {
  int fbfd = 0;
  struct fb_var_screeninfo vinfo;
  struct fb_fix_screeninfo finfo;
  long int screensize = 0;

  *fbp = NULL;

  fbfd = open("/dev/fb0", O_RDWR);
  if (fbfd == -1) {
    errorf("cannot open framebuffer device");
    return -1;
  }

  if (ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo) == -1) {
    errorf("failed to read fixed information");
    return -1;
  }

  if (ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo) == -1) {
    errorf("failed to read variable information");
    return -1;
  }

  screensize = vinfo.xres * vinfo.yres * vinfo.bits_per_pixel / 8;

  *fbp = (int8_t*) mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
  return 0;
}

int trik_init_arm_server(uint16_t rproc_id) {
  if (trik_init_rpmsg(rproc_id) < 0) {
    errorf("failed to initialize rpmsg");
    return -1;
  }

  debugf("Initalized arm server");
  return 0;
}

int trik_destroy_arm_server(void) {
  if (trik_destroy_rpmsg() < 0)
    warnf("failed disabling rpmsg");

  if (trik_destroy_camera() < 0)
    warnf("failed disabling the camera");

  debugf("destroyed arm server");
  return 0;
}

int trik_start_arm_server(enum trik_cv_algorithm cv_algorithm, char* dev_name, char* config_filename) {
  debugf("starting arm server");

  // I don't know, why it is obsolete, but found it in examples...
  // Decided to leave it as is.
  if (trik_fill_pipeline() < 0) {
    errorf("failed to fill pipeline");
    return -1;
  }

  struct buffer dsp_in_buf;
  struct buffer dsp_out_buf;
  struct trik_cv_algorithm_in_args in_args;

  if (trik_read_cv_algorithm_in_args_from_file(config_filename, &in_args) < 0)
    warnf("failed to read config from '%s', using fallback", config_filename);
  else
    debugf("sucessfully loaded config file '%s'", config_filename);

  if (trik_req_init(&dsp_in_buf, &dsp_out_buf) < 0) {
    errorf("failed to recieve image buffer");
    return -1;
  }
  debugf("successully recieved image bufs");

  if (trik_init_camera(CAMERA_BUFFER_COUNT, dev_name) < 0) {
    errorf("failed to initialize camera (not sure ov7620 or webcam)");
    return -1;
  }
  debugf("successully init camera");

  if (trik_req_cv_algorithm(cv_algorithm, in_args) < 0) {
    errorf("failed to request a cv algorithm");
    return -1;
  }
  debugf("successully got cv algorithm");

  int8_t* fbp;
  if (trik_setup_display(&fbp) < 0)
    warnf("failed to initialize display");
  else
    debugf("successully set up the display");

  while (true) {
    struct buffer image_buf;
    if (trik_camera_wait_for_frame(&image_buf) < 0) {
      errorf("unable to recieve a camera frame");
      return -1;
    }
    memcpy(dsp_in_buf.start, image_buf.start, BUFFER_SIZE);

    struct trik_cv_algorithm_out_args out_args;
    if (trik_req_step(&out_args) < 0) {
      errorf("unable to proccess a frame on a DSP");
      return -1;
    }
    if (fbp != NULL)
      for (uint32_t i = 0; i < IMG_HEIGHT; i++)
        memcpy(fbp + i * IMG_HEIGHT * 2, dsp_out_buf.start + i * IMG_WIDTH * 2 + (IMG_WIDTH - IMG_HEIGHT), sizeof(int8_t) * IMG_HEIGHT * 2);
    trik_release_frame();
  }

  if (trik_drain_pipeline() < 0) {
    errorf("failed to drain pipeline");
    return -1;
  }

  return 0;
}
