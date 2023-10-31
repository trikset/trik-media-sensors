/*
 *  V4L2 video capture example
 *
 *  This program can be used and distributed without restrictions.
 *
 *      This program is provided with the V4L2 API
 * see https://linuxtv.org/docs.php for more information
 */

#include <trik/sensors/camera.h>

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <getopt.h> /* getopt_long() */

#include <errno.h>
#include <fcntl.h> /* low-level i/o */
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include <trik/buffer.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

enum io_method {
  IO_METHOD_READ,
  IO_METHOD_MMAP,
  IO_METHOD_USERPTR,
};

static enum io_method io = IO_METHOD_MMAP;

static char* dev_name = "/dev/video0";
static int fd = -1;
static struct buffer* buffers;
static uint8_t buffer_count;
static int32_t buf_index;

static int force_format;

static void errno_exit(const char* s) {
  fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
  exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void* arg) {
  int r;

  do {
    r = ioctl(fh, request, arg);
  } while (-1 == r && EINTR == errno);

  return r;
}

// static void process_image(const void *p, int size)
// {
//     if (out_buf)
//         fwrite(p, size, 1, stdout);
//
//     fflush(stderr);
//     fprintf(stderr, ".");
//     fflush(stdout);
// }

static int release_frame() {
  struct v4l2_buffer buf;

  switch (io) {
  case IO_METHOD_MMAP:
    CLEAR(buf);
    if (buf_index >= buffer_count)
      return 0;
    buf.index = buf_index;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
      errno_exit("VIDIOC_QBUF");
    break;
  }

  return 0;
}

static int read_frame(struct buffer* out_buf) {
  struct v4l2_buffer buf;
  unsigned int i;

  switch (io) {
  case IO_METHOD_READ:
    if (-1 == read(fd, buffers[0].start, buffers[0].length)) {
      switch (errno) {
      case EAGAIN:
        return 0;

      case EIO:
        /* Could ignore EIO, see spec. */

        /* fall through */

      default:
        errno_exit("read");
      }
    }

    // process image
    break;

  case IO_METHOD_MMAP:
    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
      switch (errno) {
      case EAGAIN:
        return 0;

      case EIO:
        /* Could ignore EIO, see spec. */

        /* fall through */

      default:
        errno_exit("VIDIOC_DQBUF");
      }
    }

    // process_image(buffers[buf.index].start, buf.bytesused);
    if (out_buf != NULL) {
      out_buf->start = buffers[buf.index].start;
      out_buf->length = buf.bytesused;
    }

    buf_index = buf.index;

    // if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
    //     errno_exit("VIDIOC_QBUF");
    break;

  case IO_METHOD_USERPTR:
    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_USERPTR;

    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
      switch (errno) {
      case EAGAIN:
        return 0;

      case EIO:
        /* Could ignore EIO, see spec. */

        /* fall through */

      default:
        errno_exit("VIDIOC_DQBUF");
      }
    }

    for (i = 0; i < buffer_count; ++i)
      if (buf.m.userptr == (unsigned long) buffers[i].start && buf.length == buffers[i].length)
        break;

    // process_image((void *)buf.m.userptr, buf.bytesused);

    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
      errno_exit("VIDIOC_QBUF");
    break;
  }
  return 1;
}

static void wait_for_frame(struct buffer* out_buf) {
  for (;;) {
    fd_set fds;
    struct timeval tv;
    int r;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    /* Timeout. */
    tv.tv_sec = 5;
    tv.tv_usec = 0;

    r = select(fd + 1, &fds, NULL, NULL, &tv);

    if (-1 == r) {
      if (EINTR == errno)
        continue;
      errno_exit("select");
    }

    if (0 == r) {
      fprintf(stderr, "select timeout\n");
      exit(EXIT_FAILURE);
    }

    if (read_frame(out_buf))
      break;
    /* EAGAIN - continue select loop. */
  }
}

static void stop_capturing(void) {
  enum v4l2_buf_type type;

  switch (io) {
  case IO_METHOD_READ:
    /* Nothing to do. */
    break;

  case IO_METHOD_MMAP:
  case IO_METHOD_USERPTR:
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
      errno_exit("VIDIOC_STREAMOFF");
    break;
  }
}

static void start_capturing(void) {
  unsigned int i;
  enum v4l2_buf_type type;

  switch (io) {
  case IO_METHOD_READ:
    /* Nothing to do. */
    break;

  case IO_METHOD_MMAP:
    for (i = 0; i < buffer_count; ++i) {
      struct v4l2_buffer buf;

      CLEAR(buf);
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;

      if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");
    }
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
      errno_exit("VIDIOC_STREAMON");
    break;

  case IO_METHOD_USERPTR:
    for (i = 0; i < buffer_count; ++i) {
      struct v4l2_buffer buf;

      CLEAR(buf);
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;
      buf.index = i;
      buf.m.userptr = (unsigned long) buffers[i].start;
      buf.length = buffers[i].length;

      if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");
    }
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
      errno_exit("VIDIOC_STREAMON");
    break;
  }
}

static void uninit_device(void) {
  unsigned int i;

  switch (io) {
  case IO_METHOD_READ:
    free(buffers[0].start);
    break;

  case IO_METHOD_MMAP:
    for (i = 0; i < buffer_count; ++i)
      if (-1 == munmap(buffers[i].start, buffers[i].length))
        errno_exit("munmap");
    break;

  case IO_METHOD_USERPTR:
    for (i = 0; i < buffer_count; ++i)
      free(buffers[i].start);
    break;
  }

  free(buffers);
}

static void init_read(unsigned int buffer_size) {
  buffers = calloc(1, sizeof(*buffers));

  if (!buffers) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }

  buffers[0].length = buffer_size;
  buffers[0].start = malloc(buffer_size);

  if (!buffers[0].start) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }
}

static void init_mmap() {
  struct v4l2_requestbuffers req;

  CLEAR(req);

  req.count = buffer_count;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      fprintf(stderr,
        "%s does not support "
        "memory mappingn",
        dev_name);
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_REQBUFS");
    }
  }

  if (req.count < 2) {
    fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
    exit(EXIT_FAILURE);
  }

  buffers = calloc(req.count, sizeof(*buffers));

  if (!buffers) {
    fprintf(stderr, "Out of memory\n");
    exit(EXIT_FAILURE);
  }

  for (uint8_t n_buffers = 0; n_buffers < req.count; ++n_buffers) {
    struct v4l2_buffer buf;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers;

    if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
      errno_exit("VIDIOC_QUERYBUF");

    buffers[n_buffers].length = buf.length;
    buffers[n_buffers].start =
      mmap(NULL /* start anywhere */, buf.length, PROT_READ | PROT_WRITE /* required */, MAP_SHARED /* recommended */, fd, buf.m.offset);

    if (MAP_FAILED == buffers[n_buffers].start)
      errno_exit("mmap");
  }
}

static void init_userp() {
  struct v4l2_requestbuffers req;

  CLEAR(req);

  req.count = buffer_count;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;

  if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      fprintf(stderr,
        "%s does not support "
        "user pointer i/on",
        dev_name);
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_REQBUFS");
    }
  }

  // buffers = new_buffers;
  // buffer_count = new_buffer_count;
  // FIXME if needed
}

static void init_device(uint8_t new_buffer_count) {
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  unsigned int min;

  buffer_count = new_buffer_count;

  if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
    if (EINVAL == errno) {
      fprintf(stderr, "%s is no V4L2 device\n", dev_name);
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_QUERYCAP");
    }
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    fprintf(stderr, "%s is no video capture device\n", dev_name);
    exit(EXIT_FAILURE);
  }

  switch (io) {
  case IO_METHOD_READ:
    if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
      fprintf(stderr, "%s does not support read i/o\n", dev_name);
      exit(EXIT_FAILURE);
    }
    break;

  case IO_METHOD_MMAP:
  case IO_METHOD_USERPTR:
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
      fprintf(stderr, "%s does not support streaming i/o\n", dev_name);
      exit(EXIT_FAILURE);
    }
    break;
  }

  CLEAR(cropcap);

  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */

    if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
      switch (errno) {
      case EINVAL:
        /* Cropping not supported. */
        break;
      default:
        /* Errors ignored. */
        break;
      }
    }
  } else {
    /* Errors ignored. */
  }

  CLEAR(fmt);

  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (force_format) {
    fmt.fmt.pix.width = 320;
    fmt.fmt.pix.height = 240;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV422P;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
      errno_exit("VIDIOC_S_FMT");

    /* Note VIDIOC_S_FMT may change width and height. */
  } else {
    /* Preserve original settings as set by v4l2-ctl for example */
    if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
      errno_exit("VIDIOC_G_FMT");
  }

  /* Buggy driver paranoia. */
  min = fmt.fmt.pix.width * 2;
  if (fmt.fmt.pix.bytesperline < min)
    fmt.fmt.pix.bytesperline = min;
  min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
  if (fmt.fmt.pix.sizeimage < min)
    fmt.fmt.pix.sizeimage = min;

  switch (io) {
  case IO_METHOD_READ:
    // init_read(fmt.fmt.pix.sizeimage);
    break;

  case IO_METHOD_MMAP:
    init_mmap();
    break;

  case IO_METHOD_USERPTR:
    init_userp();
    break;
  }
}

static void close_device(void) {
  if (-1 == close(fd))
    errno_exit("close");

  fd = -1;
}

static void open_device(void) {
  struct stat st;

  if (-1 == stat(dev_name, &st)) {
    fprintf(stderr, "Cannot identify '%s': %d, %s\n", dev_name, errno, strerror(errno));
    exit(EXIT_FAILURE);
  }

  if (!S_ISCHR(st.st_mode)) {
    fprintf(stderr, "%s is no devicen", dev_name);
    exit(EXIT_FAILURE);
  }

  fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

  if (-1 == fd) {
    fprintf(stderr, "Cannot open '%s': %d, %s\n", dev_name, errno, strerror(errno));
    exit(EXIT_FAILURE);
  }
}

int trik_init_camera(uint8_t buffer_count, char* new_dev_name) {
  if (!new_dev_name)
    return -1;
  if (buffer_count < 2)
    return -1;

  dev_name = new_dev_name;
  open_device();
  init_device(buffer_count);
  start_capturing();
  return 0;
}

int trik_destroy_camera(void) {
  stop_capturing();
  uninit_device();
  close_device();
  return 0;
}

int trik_camera_wait_for_frame(struct buffer* out_buf) {
  wait_for_frame(out_buf);
  return 0;
}

void trik_try_to_recieve_frame() {}

int trik_release_frame() {
  release_frame();
  return 0;
}
