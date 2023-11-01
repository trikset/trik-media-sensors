#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <ti/ipc/Std.h>

#include <ti/ipc/Ipc.h>
#include <ti/ipc/MessageQ.h>
#include <ti/ipc/transports/TransportRpmsg.h>

#include <ti/ipc/MultiProc.h>

#include <trik/sensors/arm_server.h>
#include <trik/sensors/camera.h>
#include <trik/sensors/cmd.h>
#include <trik/sensors/cv_algorithm.h>
#include <trik/sensors/msg.h>

#define DEFAULT_DEV_NAME "/dev/video0"
#define DEFAULT_CONFIG_FILENAME "/etc/trik/sensors";

static enum trik_cv_algorithm trik_cv_algorithm_from_string(char* string) {
  if (strcmp(string, "motion_sensor") == 0)
    return TRIK_CV_ALGORITHM_MOTION_SENSOR;
  else if (strcmp(string, "edge_line_sensor") == 0)
    return TRIK_CV_ALGORITHM_EDGE_LINE_SENSOR;
  else if (strcmp(string, "object_sensor") == 0)
    return TRIK_CV_ALGORITHM_OBJECT_SENSOR;
  else if (strcmp(string, "line_sensor") == 0)
    return TRIK_CV_ALGORITHM_LINE_SENSOR;
  else
    return TRIK_CV_ALGORITHM_NONE;
}

static void usage(void) {
  printf("usage: trik-media-sensors [-h] [-d dev_name] [-c config_path] algorithm\n");
  printf("possible algorithms: motion_sensor, edge_line_sensor, object_sensor, line_sensor\n");
}

int main(int argc, char* argv[]) {
  char* dev_name = DEFAULT_DEV_NAME;
  char* config_filename = DEFAULT_CONFIG_FILENAME;

  int c;
  while ((c = getopt(argc, argv, "hd:c:")) != -1) {
    switch (c) {
    case 'h':
      usage();
      return 0;
    case 'd':
      dev_name = optarg;
      break;
    case 'c':
      config_filename = optarg;
      break;
    case '?':
      if (optopt == 'c')
        fprintf(stderr, "option -%c requires an argument", optopt);
      return -1;
    default:
      abort();
    }
  }

  if (optind >= argc) {
    usage();
    return -1;
  }
  enum trik_cv_algorithm cv_algorithm = trik_cv_algorithm_from_string(argv[optind]);

  Ipc_transportConfig(&TransportRpmsg_Factory);

  int status = Ipc_start();
  if (status < 0) {
    printf("Ipc_start failed: status = %d\n", status);
    return 0;
  }

  uint16_t rproc_id;

  rproc_id = MultiProc_getId("DSP");

  if (trik_init_arm_server(rproc_id) < 0) {
    printf("main(): failed to initialize trik arm server\n");
    return -1;
  }

  if (trik_start_arm_server(cv_algorithm, dev_name, config_filename) < 0) {
    printf("main(): failed to start trik arm server\n");
    return -1;
  }

  if (trik_destroy_arm_server() < 0) {
    printf("main(): failed to destroy trik arm server\n");
    return -1;
  }

  Ipc_stop();

  printf("<-- main:\n");

  return 0;
}
