#include "lq_camera_ex.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <poll.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

namespace
{

static int xioctl(int fd, unsigned long request, void* arg)
{
    int ret = 0;
    do {
        ret = ioctl(fd, request, arg);
    } while (ret == -1 && errno == EINTR);
    return ret;
}

static void camera_log_error(const char* msg)
{
    std::fprintf(stderr, "lq_camera_ex: %s, errno: %d (%s)\n", msg, errno, std::strerror(errno));
}

static uint32_t choose_pixelformat(lq_camera_format_t format)
{
    switch (format) {
    case LQ_CAMERA_HIGH_MJPG:
    case LQ_CAMERA_0CPU_MJPG:
    default:
        return V4L2_PIX_FMT_MJPEG;
    }
}

} // namespace

struct lq_camera_ex::lq_camera_ex_Impl
{
    struct Buffer
    {
        void* start;
        size_t length;

        Buffer() : start(MAP_FAILED), length(0) {}
    };

    std::mutex mtx;
    int fd;
    std::string path;
    uint16_t width;
    uint16_t height;
    uint16_t fps;
    lq_camera_format_t format;
    uint32_t pixelformat;
    bool opened;
    bool collecting;
    std::vector<Buffer> buffers;

    lq_camera_ex_Impl()
        : fd(-1),
          width(0),
          height(0),
          fps(0),
          format(LQ_CAMERA_HIGH_MJPG),
          pixelformat(V4L2_PIX_FMT_MJPEG),
          opened(false),
          collecting(false)
    {
    }

    int init(uint16_t new_width, uint16_t new_height, uint16_t new_fps,
             lq_camera_format_t new_format, const std::string& new_path)
    {
        std::lock_guard<std::mutex> lock(mtx);

        release_unlocked();

        width = new_width;
        height = new_height;
        fps = new_fps;
        format = new_format;
        path = new_path;
        pixelformat = choose_pixelformat(format);

        fd = open(path.c_str(), O_RDWR | O_NONBLOCK, 0);
        if (fd < 0) {
            camera_log_error("open device failed");
            return -1;
        }

        if (configure_device_unlocked() < 0 || init_mmap_unlocked() < 0) {
            release_unlocked();
            return -1;
        }

        opened = true;
        if (start_collect_unlocked() < 0) {
            release_unlocked();
            return -1;
        }

        return 0;
    }

    int configure_device_unlocked()
    {
        struct v4l2_capability cap;
        std::memset(&cap, 0, sizeof(cap));
        if (xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
            camera_log_error("query capability failed");
            return -1;
        }

        if ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0 ||
            (cap.capabilities & V4L2_CAP_STREAMING) == 0) {
            std::fprintf(stderr, "lq_camera_ex: device does not support capture streaming\n");
            return -1;
        }

        struct v4l2_format fmt;
        std::memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = width;
        fmt.fmt.pix.height = height;
        fmt.fmt.pix.pixelformat = pixelformat;
        fmt.fmt.pix.field = V4L2_FIELD_ANY;

        if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
            fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
            if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
                camera_log_error("set format failed");
                return -1;
            }
        }

        width = static_cast<uint16_t>(fmt.fmt.pix.width);
        height = static_cast<uint16_t>(fmt.fmt.pix.height);
        pixelformat = fmt.fmt.pix.pixelformat;

        if (fps > 0) {
            struct v4l2_streamparm parm;
            std::memset(&parm, 0, sizeof(parm));
            parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (xioctl(fd, VIDIOC_G_PARM, &parm) == 0 &&
                (parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME) != 0) {
                parm.parm.capture.timeperframe.numerator = 1;
                parm.parm.capture.timeperframe.denominator = fps;
                if (xioctl(fd, VIDIOC_S_PARM, &parm) == 0 &&
                    parm.parm.capture.timeperframe.numerator != 0) {
                    fps = static_cast<uint16_t>(
                        parm.parm.capture.timeperframe.denominator /
                        parm.parm.capture.timeperframe.numerator);
                }
            }
        }

        return 0;
    }

    int init_mmap_unlocked()
    {
        struct v4l2_requestbuffers req;
        std::memset(&req, 0, sizeof(req));
        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
            camera_log_error("request buffers failed");
            return -1;
        }

        if (req.count == 0) {
            std::fprintf(stderr, "lq_camera_ex: no mmap buffers available\n");
            return -1;
        }

        buffers.resize(req.count);
        for (size_t i = 0; i < buffers.size(); ++i) {
            struct v4l2_buffer buf;
            std::memset(&buf, 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = static_cast<unsigned int>(i);

            if (xioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
                camera_log_error("query buffer failed");
                return -1;
            }

            buffers[i].length = buf.length;
            buffers[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                    MAP_SHARED, fd, buf.m.offset);
            if (buffers[i].start == MAP_FAILED) {
                camera_log_error("mmap buffer failed");
                return -1;
            }
        }

        return 0;
    }

    int start_collect()
    {
        std::lock_guard<std::mutex> lock(mtx);
        return start_collect_unlocked();
    }

    int start_collect_unlocked()
    {
        if (collecting) {
            return 0;
        }
        if (fd < 0 || buffers.empty()) {
            return -1;
        }

        for (size_t i = 0; i < buffers.size(); ++i) {
            struct v4l2_buffer buf;
            std::memset(&buf, 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = static_cast<unsigned int>(i);

            if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) {
                camera_log_error("queue buffer failed");
                return -1;
            }
        }

        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
            camera_log_error("start collect failed");
            return -1;
        }

        collecting = true;
        opened = true;
        return 0;
    }

    int stop_collect()
    {
        std::lock_guard<std::mutex> lock(mtx);
        return stop_collect_unlocked();
    }

    int stop_collect_unlocked()
    {
        if (!collecting || fd < 0) {
            collecting = false;
            return 0;
        }

        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
            const int saved_errno = errno;
            collecting = false;

            if (saved_errno == ENODEV || saved_errno == EINVAL || saved_errno == EBADF) {
                return 0;
            }

            errno = saved_errno;
            camera_log_error("stop collect failed");
            return -1;
        }

        collecting = false;
        return 0;
    }

    void release()
    {
        std::lock_guard<std::mutex> lock(mtx);
        release_unlocked();
    }

    void release_unlocked()
    {
        stop_collect_unlocked();

        for (size_t i = 0; i < buffers.size(); ++i) {
            if (buffers[i].start != MAP_FAILED && buffers[i].start != NULL) {
                if (munmap(buffers[i].start, buffers[i].length) < 0) {
                    camera_log_error("munmap buffer failed");
                }
            }
            buffers[i].start = MAP_FAILED;
            buffers[i].length = 0;
        }
        buffers.clear();

        if (fd >= 0) {
            if (close(fd) < 0) {
                camera_log_error("close device failed");
            }
            fd = -1;
        }

        opened = false;
        collecting = false;
    }

    cv::Mat get_frame_raw()
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (!opened || !collecting || fd < 0) {
            return cv::Mat();
        }

        struct pollfd fds;
        std::memset(&fds, 0, sizeof(fds));
        fds.fd = fd;
        fds.events = POLLIN;

        int poll_ret = 0;
        do {
            poll_ret = poll(&fds, 1, 1000);
        } while (poll_ret < 0 && errno == EINTR);

        if (poll_ret <= 0) {
            if (poll_ret < 0) {
                camera_log_error("poll frame failed");
            }
            return cv::Mat();
        }

        struct v4l2_buffer buf;
        std::memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (xioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
            if (errno != EAGAIN) {
                camera_log_error("dequeue frame failed");
            }
            return cv::Mat();
        }

        cv::Mat frame;
        if (buf.index < buffers.size() && buf.bytesused > 0) {
            try {
                void* data = buffers[buf.index].start;
                if (pixelformat == V4L2_PIX_FMT_MJPEG || pixelformat == V4L2_PIX_FMT_JPEG) {
                    cv::Mat encoded(1, static_cast<int>(buf.bytesused), CV_8UC1, data);
                    frame = cv::imdecode(encoded, cv::IMREAD_COLOR);
                } else if (pixelformat == V4L2_PIX_FMT_YUYV) {
                    cv::Mat yuyv(height, width, CV_8UC2, data);
                    cv::cvtColor(yuyv, frame, cv::COLOR_YUV2BGR_YUYV);
                } else if (pixelformat == V4L2_PIX_FMT_GREY) {
                    frame = cv::Mat(height, width, CV_8UC1, data).clone();
                } else {
                    std::fprintf(stderr, "lq_camera_ex: unsupported pixel format: 0x%08x\n", pixelformat);
                }
            } catch (const cv::Exception& e) {
                std::fprintf(stderr, "lq_camera_ex: decode frame failed: %s\n", e.what());
                frame.release();
            }
        }

        if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            camera_log_error("requeue frame failed");
            return cv::Mat();
        }

        return frame;
    }
};

lq_camera_ex::lq_camera_ex(uint16_t _width, uint16_t _height, uint16_t _fps,
                           lq_camera_format_t _fmt, const std::string _path)
    : pImpl(new lq_camera_ex_Impl())
{
    pImpl->init(_width, _height, _fps, _fmt, _path);
}

lq_camera_ex::~lq_camera_ex()
{
    if (pImpl) {
        pImpl->release();
    }
}

int lq_camera_ex::init(uint16_t _width, uint16_t _height, uint16_t _fps,
                       lq_camera_format_t _format, const std::string _path)
{
    if (!pImpl) {
        pImpl.reset(new lq_camera_ex_Impl());
    }
    return pImpl->init(_width, _height, _fps, _format, _path);
}

int lq_camera_ex::start_collect()
{
    return pImpl ? pImpl->start_collect() : -1;
}

int lq_camera_ex::stop_collect()
{
    return pImpl ? pImpl->stop_collect() : 0;
}

cv::Mat lq_camera_ex::get_frame_raw()
{
    return pImpl ? pImpl->get_frame_raw() : cv::Mat();
}

cv::Mat lq_camera_ex::get_frame_gray()
{
    cv::Mat raw = get_frame_raw();
    if (raw.empty()) {
        return cv::Mat();
    }

    cv::Mat gray;
    if (raw.channels() == 1) {
        gray = raw.clone();
    } else {
        cv::cvtColor(raw, gray, cv::COLOR_BGR2GRAY);
    }
    return gray;
}

bool lq_camera_ex::get_frame_raw_gray(cv::Mat& raw, cv::Mat& gray)
{
    raw = get_frame_raw();
    if (raw.empty()) {
        gray.release();
        return false;
    }

    if (raw.channels() == 1) {
        gray = raw.clone();
    } else {
        cv::cvtColor(raw, gray, cv::COLOR_BGR2GRAY);
    }
    return true;
}

bool lq_camera_ex::set_exposure_manual(int16_t expo)
{
    if (!pImpl) {
        return false;
    }

    std::lock_guard<std::mutex> lock(pImpl->mtx);
    if (pImpl->fd < 0) {
        return false;
    }

    bool ok = true;

    struct v4l2_control ctrl;
    std::memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = V4L2_CID_EXPOSURE_AUTO;
    ctrl.value = V4L2_EXPOSURE_MANUAL;
    if (xioctl(pImpl->fd, VIDIOC_S_CTRL, &ctrl) < 0) {
        ok = false;
    }

    std::memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    ctrl.value = expo;
    if (xioctl(pImpl->fd, VIDIOC_S_CTRL, &ctrl) < 0) {
        camera_log_error("set exposure failed");
        ok = false;
    }

    return ok;
}

bool lq_camera_ex::save_image_picture(const cv::Mat& frame, const std::string& filename)
{
    if (frame.empty() || filename.empty()) {
        return false;
    }
    return cv::imwrite(filename, frame);
}

uint16_t lq_camera_ex::get_camera_width() const
{
    if (!pImpl) {
        return 0;
    }
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->width;
}

uint16_t lq_camera_ex::get_camera_height() const
{
    if (!pImpl) {
        return 0;
    }
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->height;
}

uint16_t lq_camera_ex::get_camera_fps() const
{
    if (!pImpl) {
        return 0;
    }
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->fps;
}

bool lq_camera_ex::is_cam_opened() const
{
    if (!pImpl) {
        return false;
    }
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->opened && pImpl->fd >= 0;
}
