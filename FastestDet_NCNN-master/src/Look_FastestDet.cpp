#include <iostream>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "ncnn/net.h"

#include <stdio.h>
#include <unistd.h>

#include <stdlib.h>
#include <png.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <errno.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <linux/fb.h>
#include "linux/ioctl.h"
#include "signal.h"
#include <pthread.h>

//多线程变量
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t mutex2 = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond2 = PTHREAD_COND_INITIALIZER;
pthread_mutex_t mutex3 = PTHREAD_MUTEX_INITIALIZER;

//>>model内容

using namespace std;
using namespace cv;

//mode = 0表示单独测试，mode = 1表示联合摄像头使用
#define mode 1

struct Object
{
    cv::Rect  rect;
    cv::Rect crop_rect;
    int label;
    float prob;
    int track_id;
    std::vector<cv::Point2d>landmark;
    std::vector<float>angle;
    float distance_to_my_car;
  //  std::vector<float>cal_angle;
    std::string class_name;  //类别标签
};

class FastestDet
{
public:
    FastestDet(const  char*  model_bin,const  char*  model_param);
    vector<Object> detect(Mat frame);
private:
    const int inpWidth = 512;
    const int inpHeight = 512;
    vector<string> class_names;
    int num_class;

    float confThreshold;
    float nmsThreshold;
    ncnn::Net _net;
    void drawPred(float conf, int left, int top, int right, int bottom, Mat& frame, int classid);
    void qsort_descent_inplace(std::vector<Object>& faceobjects);
    void qsort_descent_inplace(std::vector<Object>& faceobjects, int left, int right);
    inline float intersection_area(const Object& a, const Object& b);
    void nms_sorted_bboxes(const std::vector<Object>& faceobjects, std::vector<int>& picked, float nms_threshold);
};

FastestDet::FastestDet(const  char*  model_bin,const  char*  model_param)
{
    //  yolov5.opt.use_int8_inference=true;

      _net.opt.use_vulkan_compute = true;
      _net.opt.use_fp16_storage = true;
     //  yolov5.register_custom_layer("Swish", Swish_layer_creator);

      _net.load_param(model_param);
      _net.load_model(model_bin);
}


void FastestDet::nms_sorted_bboxes(const std::vector<Object>& faceobjects, std::vector<int>& picked, float nms_threshold)
{
    picked.clear();

    const int n = faceobjects.size();

    std::vector<float> areas(n);
    for (int i = 0; i < n; i++)
    {
        areas[i] = faceobjects[i].rect.area();
    }

    for (int i = 0; i < n; i++)
    {
        const Object& a = faceobjects[i];

        int keep = 1;
        for (int j = 0; j < (int)picked.size(); j++)
        {
            const Object& b = faceobjects[picked[j]];

            // intersection over union
            float inter_area = intersection_area(a, b);
            float union_area = areas[i] + areas[picked[j]] - inter_area;
            // float IoU = inter_area / union_area
            if (inter_area / union_area > nms_threshold)
                keep = 0;
        }

        if (keep)
            picked.push_back(i);
    }
}
inline float FastestDet::intersection_area(const Object& a, const Object& b)
{
    cv::Rect_<float> inter = a.rect & b.rect;
    return inter.area();
}

void FastestDet::qsort_descent_inplace(std::vector<Object>& faceobjects, int left, int right)
{
    int i = left;
    int j = right;
    float p = faceobjects[(left + right) / 2].prob;

    while (i <= j)
    {
        while (faceobjects[i].prob > p)
            i++;

        while (faceobjects[j].prob < p)
            j--;

        if (i <= j)
        {
            // swap
            std::swap(faceobjects[i], faceobjects[j]);

            i++;
            j--;
        }
    }

    #pragma omp parallel sections
    {
        #pragma omp section
        {
            if (left < j) qsort_descent_inplace(faceobjects, left, j);
        }
        #pragma omp section
        {
            if (i < right) qsort_descent_inplace(faceobjects, i, right);
        }
    }
}

//貌似是只显示置信度最大的那个类别
void FastestDet::qsort_descent_inplace(std::vector<Object>& faceobjects)
{
    if (faceobjects.empty())
        return;

    qsort_descent_inplace(faceobjects, 0, faceobjects.size() - 1);
}

inline float sigmoid(float x)
{
    return 1.0 / (1 + expf(-x));
}

vector<Object> FastestDet::detect(Mat frame)
{
    std::vector<Object> proposals;
    std::vector<Object> objects;
    const int target_size = 512;
    const float prob_threshold = 0.25f;
    const float nms_threshold = 0.35f;

    const std::vector<std::string> class_names = {
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
        "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
        "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
        "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
        "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
        "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
        "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
        "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
        "hair drier", "toothbrush"
    };

    int img_w = frame.cols;
    int img_h = frame.rows;
   //  cv::resize(bgr, bgr, cv::Size(640, 640));
    ncnn::Mat inputImg = ncnn::Mat::from_pixels_resize(frame.data, ncnn::Mat::PIXEL_BGR,\
                                                       frame.cols, frame.rows, target_size, target_size);

    //Normalization of input image data
    const float mean_vals[3] = {0.f, 0.f, 0.f};
    const float norm_vals[3] = {1/255.f, 1/255.f, 1/255.f};
    inputImg.substract_mean_normalize(mean_vals, norm_vals);

    ncnn::Extractor ex = _net.create_extractor();
    ex.input("image_arrays", inputImg);
    ncnn::Mat out;
    ex.extract("outputs", out);
    int c =out.c;
    int w =out.w;
    int h = out.h;
    int num_class  =out.w -5;
    for (int i =0; i < c; i++)
    {
        for (int j = 0;j < h; j++)
        {
            const float *data_ptr =out.row(i *h+j);
            // find class index with max class score
            int class_index = 0;
            float class_score = -FLT_MAX;
            for (int k = 0; k < num_class; k++)
            {
                float score = data_ptr[5 + k];
                if (score > class_score)
                 {
                    class_index = k;
                    class_score = score;
                }
            }
            class_score *= data_ptr[0];
            if(class_score >0.8)
            {
                const int class_idx = class_index;
                float cx = (tanh(data_ptr[1]) + j) / (float)c;  ///cx
                float cy = (tanh(data_ptr[2]) + i) / (float)h;   ///cy
                float w = sigmoid(data_ptr[3]);   ///w
                float h = sigmoid(data_ptr[4]);  ///h

                cx *= float(frame.cols);
                cy *= float(frame.rows);
                w *= float(frame.cols);
                h *= float(frame.rows);

                int left = int(cx - 0.5 * w);
                int top = int(cy - 0.5 * h);
                cv::Rect rect =cv::Rect(left,top,w,h);

                Object obj;
                obj.prob =class_score;
                obj.rect =rect;
                obj.label =class_index;
                proposals.push_back(obj);
            }
        }
    }
    // sort all proposals by score from highest to lowest
    qsort_descent_inplace(proposals);

    // apply nms with nms_threshold
    std::vector<int> picked;
    nms_sorted_bboxes(proposals, picked, nms_threshold);

    int count = picked.size();

    objects.resize(count);
    for (int i = 0; i < count; i++)
    {
        objects[i] = proposals[picked[i]];
        // 获取类别标签
        int class_index = objects[i].label;
        if (class_index >= 0 && class_index < class_names.size()) {
            objects[i].class_name = class_names[class_index];
        } else {
            objects[i].class_name = "Unknown"; // 处理类别索引超出范围的情况
        }

        // 计算文本的宽度
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(objects[i].class_name, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);

        cv::rectangle(frame,objects[i].rect,cv::Scalar(0,255,0),1);
        cv::putText(frame, objects[i].class_name, cv::Point(objects[i].rect.x + objects[i].rect.width - text_size.width - 5, objects[i].rect.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }

    cv::imwrite("Fast_res_image.png", frame);

    // fprintf(stderr, "Processed image saved\n");

    return  objects; 
}

const char* imagepath = "./my_pic.png";

void test_fast_yolov2(void)
{
    cv::Mat src =cv::imread(imagepath, cv::IMREAD_UNCHANGED);

    pthread_mutex_lock(&mutex);
    //此时就可以删除目录中的图片了
    if(remove("./my_pic.png")) printf("remove ./my_pic.png failed!\n");
    pthread_cond_signal(&cond);  //唤醒主线程, cond受mutex保护
    pthread_mutex_unlock(&mutex);
    
    cv::flip(src, src, 0);

#if mode   //mode = 1
    //配合摄像头使用
    FastestDet det("../ncnn/fastdet_512x512.bin",
                   "../ncnn/fastdet_512x512.param");

#else    //mode = 0, 单独测试（用于测试运行时间，可用time命令进行测试)
    FastestDet det("./fastdet_512x512.bin",
                   "./fastdet_512x512.param");

#endif
    vector<Object> res = det.detect(src);
    

}


//---------------------------------------------------------------------------------------------------------------------------------------
//>>v4l2内容
#define FB_DEV              "/dev/fb0"      //LCD设备节点
#define FRAMEBUFFER_COUNT   3               //帧缓冲数量   

/*** 摄像头像素格式及其描述信息 ***/
typedef struct camera_format {
    unsigned char description[32];  //字符串描述信息
    unsigned int pixelformat;       //像素格式
} cam_fmt;

/*** 描述一个帧缓冲的信息 ***/
typedef struct cam_buf_info {
    unsigned short *start;      //帧缓冲起始地址
    unsigned long length;       //帧缓冲长度
} cam_buf_info;

static int width;                       //LCD宽度
static int height;                      //LCD高度
static unsigned short *screen_base = NULL, *cp_screen_base = NULL;//LCD显存基地址
static unsigned long line_length;       //LCD一行的长度（字节为单位）
static unsigned int bpp;    //像素深度bpp
static int fb_fd = -1;                  //LCD设备文件描述符
static int v4l2_fd = -1;                //摄像头设备文件描述符
static cam_buf_info buf_infos[FRAMEBUFFER_COUNT];
static cam_fmt cam_fmts[10];
static int frm_width, frm_height;   //视频帧宽度和高度
static int key_flag = 0;
static int key_cc = 0;
static int pic_idx = 0;
char name[20];
static int key_fd = 0;

int make_wite_png(const char *outfile, int width, int height, int buf_idx);

static int fb_dev_init(void)
{
    struct fb_var_screeninfo fb_var = {0};
    struct fb_fix_screeninfo fb_fix = {0};
    unsigned long screen_size;

    /* 打开framebuffer设备 */
    fb_fd = open(FB_DEV, O_RDWR);
    if (0 > fb_fd) {
        fprintf(stderr, "open error: %s: %s\n", FB_DEV, strerror(errno));
        return -1;
    }

    /* 获取framebuffer设备信息 */
    ioctl(fb_fd, FBIOGET_VSCREENINFO, &fb_var);
    ioctl(fb_fd, FBIOGET_FSCREENINFO, &fb_fix);

    line_length = fb_fix.line_length;
    bpp = fb_var.bits_per_pixel;
    screen_size = fb_fix.line_length * fb_var.yres;
    width = fb_var.xres;
    height = fb_var.yres;

    /* 内存映射 */
    screen_base = reinterpret_cast<short unsigned int*>(mmap(NULL, screen_size, PROT_READ | PROT_WRITE, MAP_SHARED, fb_fd, 0));
    cp_screen_base = screen_base;

    if (MAP_FAILED == (void *)screen_base) {
        perror("mmap error");
        close(fb_fd);
        return -1;
    }

    /* LCD背景刷白 */
    memset(screen_base, 0xFF, screen_size);
    return 0;
}

static int v4l2_dev_init(const char *device)
{
    struct v4l2_capability cap = {0};

    /* 打开摄像头 */
    v4l2_fd = open(device, O_RDWR);
    if (0 > v4l2_fd) {
        fprintf(stderr, "open error: %s: %s\n", device, strerror(errno));
        return -1;
    }

    /* 查询设备功能 */
    ioctl(v4l2_fd, VIDIOC_QUERYCAP, &cap);

    /* 判断是否是视频采集设备 */
    if (!(V4L2_CAP_VIDEO_CAPTURE & cap.capabilities)) {
        fprintf(stderr, "Error: %s: No capture video device!\n", device);
        close(v4l2_fd);
        return -1;
    }

    return 0;
}

static void v4l2_enum_formats(void)
{
    struct v4l2_fmtdesc fmtdesc = {0};

    /* 枚举摄像头所支持的所有像素格式以及描述信息 */
    fmtdesc.index = 0;
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    while (0 == ioctl(v4l2_fd, VIDIOC_ENUM_FMT, &fmtdesc)) {

        // 将枚举出来的格式以及描述信息存放在数组中
        cam_fmts[fmtdesc.index].pixelformat = fmtdesc.pixelformat;
        //显示转换
        strcpy(reinterpret_cast<char*>(cam_fmts[fmtdesc.index].description), reinterpret_cast<const char*>(fmtdesc.description));
        fmtdesc.index++;
    }
}

static void v4l2_print_formats(void)
{
    struct v4l2_frmsizeenum frmsize = {0};
    struct v4l2_frmivalenum frmival = {0};
    int i;

    frmsize.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    frmival.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    for (i = 0; cam_fmts[i].pixelformat; i++) {

        printf("format<0x%x>, description<%s>\n", cam_fmts[i].pixelformat,
                    cam_fmts[i].description);

        /* 枚举出摄像头所支持的所有视频采集分辨率 */
        frmsize.index = 0;
        frmsize.pixel_format = cam_fmts[i].pixelformat;
        frmival.pixel_format = cam_fmts[i].pixelformat;
        while (0 == ioctl(v4l2_fd, VIDIOC_ENUM_FRAMESIZES, &frmsize)) {

            printf("size<%d*%d> ",
                    frmsize.discrete.width,
                    frmsize.discrete.height);
            frmsize.index++;

            /* 获取摄像头视频采集帧率 */
            frmival.index = 0;
            frmival.width = frmsize.discrete.width;
            frmival.height = frmsize.discrete.height;
            while (0 == ioctl(v4l2_fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmival)) {

                printf("<%dfps>", frmival.discrete.denominator /
                        frmival.discrete.numerator);
                frmival.index++;
            }
            printf("\n");
        }
        printf("\n");
    }
}

static int v4l2_set_format(void)
{
    struct v4l2_format fmt = {0};
    struct v4l2_streamparm streamparm = {0};

    /* 设置帧格式 */
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;//type类型
    // fmt.fmt.pix.width = width;  //视频帧宽度
    // fmt.fmt.pix.height = height;//视频帧高度
    //试一下640*480
    fmt.fmt.pix.width = 640;  //视频帧宽度
    fmt.fmt.pix.height = 480;//视频帧高度    

    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;  //像素格式
    if (0 > ioctl(v4l2_fd, VIDIOC_S_FMT, &fmt)) {
        fprintf(stderr, "ioctl error: VIDIOC_S_FMT: %s\n", strerror(errno));
        return -1;
    }

    /*** 判断是否已经设置为我们要求的RGB565像素格式
    如果没有设置成功表示该设备不支持RGB565像素格式 */
    if (V4L2_PIX_FMT_RGB565 != fmt.fmt.pix.pixelformat) {
        fprintf(stderr, "Error: the device does not support RGB565 format!\n");
        return -1;
    }

    frm_width = fmt.fmt.pix.width;  //获取实际的帧宽度
    frm_height = fmt.fmt.pix.height;//获取实际的帧高度
    printf("视频帧大小<%d * %d>\n", frm_width, frm_height);

    /* 获取streamparm */
    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(v4l2_fd, VIDIOC_G_PARM, &streamparm);

    /** 判断是否支持帧率设置 **/
    //printf("V4L2_CAP_TIMEPERFRAME = %x\r\n", V4L2_CAP_TIMEPERFRAME);

    //printf(">>: Frame rate: %u/%u\n",streamparm.parm.capture.timeperframe.numerator,streamparm.parm.capture.timeperframe.denominator);


    if (V4L2_CAP_TIMEPERFRAME & streamparm.parm.capture.capability) {
        streamparm.parm.capture.timeperframe.numerator = 1;
        streamparm.parm.capture.timeperframe.denominator = 30;//30fps
        if (0 > ioctl(v4l2_fd, VIDIOC_S_PARM, &streamparm)) {
            fprintf(stderr, "ioctl error: VIDIOC_S_PARM: %s\n", strerror(errno));
            return -1;
        }
        else{
            printf("set fps success, fps = %d\r\n", streamparm.parm.capture.timeperframe.denominator);
        }
    }
    else{
        //printf("not go in!\r\n");
    }

    return 0;
}

static int v4l2_init_buffer(void)
{
    struct v4l2_requestbuffers reqbuf = {0};
    struct v4l2_buffer buf = {0};

    /* 申请帧缓冲 */
    reqbuf.count = FRAMEBUFFER_COUNT;       //帧缓冲的数量
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_MMAP;
    if (0 > ioctl(v4l2_fd, VIDIOC_REQBUFS, &reqbuf)) {
        fprintf(stderr, "ioctl error: VIDIOC_REQBUFS: %s\n", strerror(errno));
        return -1;
    }

    /* 建立内存映射 */
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    for (buf.index = 0; buf.index < FRAMEBUFFER_COUNT; buf.index++) {

        ioctl(v4l2_fd, VIDIOC_QUERYBUF, &buf);
        buf_infos[buf.index].length = buf.length;

        buf_infos[buf.index].start = reinterpret_cast<short unsigned int*>(mmap(NULL, buf.length,
                PROT_READ | PROT_WRITE, MAP_SHARED,
                v4l2_fd, buf.m.offset));

        if (MAP_FAILED == buf_infos[buf.index].start) {
            perror("mmap error");
            return -1;
        }
    }

    /* 入队 */
    for (buf.index = 0; buf.index < FRAMEBUFFER_COUNT; buf.index++) {

        if (0 > ioctl(v4l2_fd, VIDIOC_QBUF, &buf)) {
            fprintf(stderr, "ioctl error: VIDIOC_QBUF: %s\n", strerror(errno));
            return -1;
        }
    }

    return 0;
}

static int v4l2_stream_on(void)
{
    /* 打开摄像头、摄像头开始采集数据 */
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 > ioctl(v4l2_fd, VIDIOC_STREAMON, &type)) {
        fprintf(stderr, "ioctl error: VIDIOC_STREAMON: %s\n", strerror(errno));
        return -1;
    }

    return 0;
}


//-----------------------------------------------------------------------------------------------------------
//show_image
// static int width;                       //LCD X分辨率
// static int height;                      //LCD Y分辨率
// static unsigned short *screen_base = NULL;        //映射后的显存基地址

static int show_png_image(const char *path)
{
    png_structp png_ptr = NULL;
    png_infop info_ptr = NULL;
    FILE *png_file = NULL;
    unsigned short *fb_line_buf = NULL; //行缓冲区:用于存储写入到LCD显存的一行数据
    unsigned int min_h, min_w;
    unsigned int valid_bytes;
    unsigned int image_h, image_w;
    png_bytepp row_pointers = NULL;
    int i, j, k;

    // printf("start to show png!\n");
    // printf("path = %s\n", path);

    /* 打开png文件 */
    png_file = fopen(path, "r");    //以只读方式打开
    if (NULL == png_file) {
        perror("fopen error");
        return -1;
    }

    /* 分配和初始化png_ptr、info_ptr */
    png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png_ptr) {
        fclose(png_file);
        return -1;
    }

    info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        png_destroy_read_struct(&png_ptr, NULL, NULL);
        fclose(png_file);
        return -1;
    }

    /* 设置错误返回点 */
    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
        fclose(png_file);
        return -1;
    }

    /* 指定数据源 */
    png_init_io(png_ptr, png_file);

    /* 读取png文件 */
    png_read_png(png_ptr, info_ptr, PNG_TRANSFORM_STRIP_ALPHA, NULL);
    image_h = png_get_image_height(png_ptr, info_ptr);
    image_w = png_get_image_width(png_ptr, info_ptr);
    // printf("分辨率: %d*%d\n", image_w, image_h);

    /* 判断是不是RGB888 */
    if ((8 != png_get_bit_depth(png_ptr, info_ptr)) &&
        (PNG_COLOR_TYPE_RGB != png_get_color_type(png_ptr, info_ptr))) {
        printf("Error: Not 8bit depth or not RGB color");
        png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
        fclose(png_file);
        return -1;
    }

    /* 判断图像和LCD屏那个的分辨率更低 */
    if (image_w > width)
        min_w = width;
    else
        min_w = image_w;

    if (image_h > height)
        min_h = height;
    else
        min_h = image_h;

    valid_bytes = min_w * bpp / 8;

    // printf("min_w = %d, min_h = %d\r\n", min_w, min_h);

    /* 读取解码后的数据 */
    fb_line_buf = reinterpret_cast<short unsigned int*>(malloc(valid_bytes));

    row_pointers = png_get_rows(png_ptr, info_ptr);//获取数据

    unsigned int temp = min_w * 3;  //RGB888 一个像素3个bit位

    screen_base = cp_screen_base;

    for(i = 0; i < min_h; i++) {
        // RGB888转为RGB565
        for(j = k = 0; j < temp; j += 3, k++){
            fb_line_buf[k] = ((row_pointers[i][j] & 0xF8) << 8) |
                ((row_pointers[i][j+1] & 0xFC) << 3) |
                ((row_pointers[i][j+2] & 0xF8) >> 3);
        }

        memcpy(screen_base, fb_line_buf, valid_bytes);//将一行数据刷入显存
        screen_base += width;   //定位到显存下一行
    }

    /* 结束、销毁/释放内存 */
    png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
    free(fb_line_buf);
    fclose(png_file);

    // printf("show png success!!!!!\n");
    return 0;
}

//复制文件, 成功返回0, 否则返回-1, 用于复制目标检测的识别图片实现保存。
int copy_file(const char *srcPath, char *destPath)
{
	char Buf[1024] = {0};
	int count_read = 0;
	long fp_src_ltell = 0,fp_src_atell = 0;
	FILE* fp_src = fopen(srcPath,"r");//只读方式打开
	FILE* fp_dst = fopen(destPath,"w");//只写方式打开
	if(fp_dst ==NULL || fp_src == NULL)
	{
		printf("文件打开有问题\n");
		return -1;
	}
	while(1)
	{
		memset(Buf,0,sizeof(Buf));
		fp_src_ltell = ftell(fp_src); //上一次文件指针位置
		count_read = fread(Buf,sizeof(Buf),1,fp_src);
		fp_src_atell = ftell(fp_src); //当前文件指针位置
		if(count_read<1) //异常或到达末尾结束
		{
			if(feof(fp_src))
			{
				long temp = fp_src_atell - fp_src_ltell;
				fwrite(Buf,temp,1,fp_dst); //成功
				return 0;
			}
			else if(ferror(fp_src))
			{
				perror("fread error:");
				return -1;
			}
		}
		fwrite(Buf,sizeof(Buf),1,fp_dst);
	}
	return 0;
}


char path[30] = "";
static int ii = 0, jj = 0;

// const char *directory_path = "/path/to/directory/";
// const char *png_file_name = "example.png";

char save_path[30];
int save_idx = 0;
const char* filename = "./Fast_res_image.png";

int directory_empty = 1;

//>>多线程具体内容
//该线程为轮询，不会阻塞！  位于中间位置，是桥梁
void *model_function(void *arg) {
    while(1){  //主线程阻塞的原因是子线程还没读，所以子线程读完后要唤醒睡眠的主线程
        pthread_mutex_lock(&mutex);
        // 使用 access 函数来检查目录是否为空
        if (access(name, F_OK) == -1) {  //目录为空，
            // printf("[model thread]: Directory is empty, notifying main thread...\n");
            pthread_cond_signal(&cond);  //唤醒主线程
        }
        pthread_mutex_unlock(&mutex);

        //运行ncnn目标检测
        pthread_mutex_lock(&mutex2);
        if(access("Fast_res_image.png", F_OK) == -1){  //目录为空
            printf("[model thread]: not find ./Fast_res_image.png\n");
            //我需要写，但是如果此时目录下没有./my_pic.png，执行test_fast_yolov2就会报错！
            if(access(name, F_OK) != -1){  //存在my_pic.png
                test_fast_yolov2(); //会往目录中写Fast_res_image.png
                //识别完后删除原图
                printf("[model thread]: ncnn success!\n");
                // if(remove("./my_pic.png")) printf("remove ./my_pic.png failed!\n");  //在test_fast_yolov2中已经删除
            }
            else{
                printf("[model thread]: don't have my_pic.png, so can't ncnn!\n");
            }
        }
        else{
            printf("[model thread]: find ./Fast_res_image.png, notifying show thread...\n");
            //唤醒show_thread
            pthread_cond_signal(&cond2);  //唤醒show_thread线程
        }
        pthread_mutex_unlock(&mutex2);

        pthread_mutex_lock(&mutex3);
        if(access(filename, F_OK) != -1){  //存在
            if(key_flag == 1){
                key_flag = 0;
                sprintf(save_path, "save_%d.png", save_idx++);
                copy_file(filename, save_path);
                printf("[model thread]: <<<<=================== save png to pwd success! =================>>>>\n");
            }
        }
        pthread_mutex_unlock(&mutex3);
    }
    return NULL;
}

void *show_png_function(void *arg){
    while(1){
        pthread_mutex_lock(&mutex2);
        if(access("./Fast_res_image.png", F_OK) == -1){  //不存在, 自己进入阻塞
            printf("[show thread]: not find ./Fast_res_image.png, i'm go to sleep\n");
            pthread_cond_wait(&cond2, &mutex2); // 阻塞等待条件变为真，线程阻塞在这里
        }
        else{
            printf("[show thread]: show ./Fast_res_image.png\n");
            show_png_image("./Fast_res_image.png");
            //然后删掉./Fast_res_image.png
            if(remove("./Fast_res_image.png")) printf("remove ./Fast_res_image.png failed!\n");
            else printf("remove ./Fast_res_image.png success!\n");
        }
        pthread_mutex_unlock(&mutex2);

        pthread_mutex_lock(&mutex3);
        if(access(filename, F_OK) != -1){  //存在
            if(key_flag == 1){
                key_flag = 0;
                sprintf(save_path, "save_%d.png", save_idx++);
                copy_file(filename, save_path);
                printf("[show thread]: <<<<=================== save png to pwd success! =================>>>>\n");
            }
        }
        pthread_mutex_unlock(&mutex3);
    }
    return NULL;
}


//多线程
pthread_t model_thread_id, show_thread_id;
int ret;

static void v4l2_read_data(void)
{
    struct v4l2_buffer buf = {0};
    unsigned short *base;
    unsigned short *start;
    int min_w, min_h;
    int j = 0;

    char remove_path[30] = "";

    if (width > frm_width)
        min_w = frm_width;
    else
        min_w = width;
    if (height > frm_height)
        min_h = frm_height;
    else
        min_h = height;

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    // 创建目标检测的线程，并将参数传递给子线程
    // ret = pthread_create(&model_thread_id, NULL, model_function, path);
    ret = pthread_create(&model_thread_id, NULL, model_function, NULL);
    if (ret != 0) {
        printf("Error in pthread_create\n");
        exit(-1);
    }        

    //创建显示图片的线程，并将参数传递给子线程
    ret = pthread_create(&show_thread_id, NULL, show_png_function, NULL);
    if (ret != 0) {
        printf("Error in pthread_create\n");
        exit(-1);
    }

    for ( ; ; ) {

        for(buf.index = 0; buf.index < FRAMEBUFFER_COUNT; buf.index++) {

            printf("============start=============\n");
            ioctl(v4l2_fd, VIDIOC_DQBUF, &buf);     //出队

            // printf("======== saving png ========\r\n");
            sprintf(name, "./my_pic.png"); 
            // ii = 0;
            // path[ii++] = '.', path[ii++] = '/';
            // for(jj = 0; jj < 12; jj++) path[ii++] = name[jj];
            // printf("image path = %s\n", path);
    
            pthread_mutex_lock(&mutex);
            //判断当前目录中my_pic.png是否存在，不存在才需要写入
            if (access(name, F_OK) == -1) {  //不存在
                printf("[main thread]: Directory is empty, writing PNG file...\n");
                //主线程负责将该png写入到目录
                make_wite_png(name, 640, 480, buf.index);
                directory_empty = 0;
                pthread_cond_signal(&cond); // 通知等待的子线程去读
            } else {  //存在
                printf("[main thread]: Directory is not empty, waiting...\n");     
                directory_empty = 1;
                pthread_cond_wait(&cond, &mutex); // 阻塞等待条件变为真，主线程阻塞在这里
            }
            pthread_mutex_unlock(&mutex);

            pthread_mutex_lock(&mutex3);
            if(access(filename, F_OK) != -1){  //存在
                if(key_flag == 1){
                    key_flag = 0;
                    sprintf(save_path, "save_%d.png", save_idx++);
                    copy_file(filename, save_path);
                    printf("[main thread]: <<<<=================== save png to pwd success! =================>>>>\n");
                }
            }
            pthread_mutex_unlock(&mutex3);

            //删除上一张图片
            // sprintf(remove_path, "./my_pic_%d.png", !pic_idx);
            // if(remove(remove_path)) printf("remove failed!\n");

            // 数据处理完之后、再入队、往复
            ioctl(v4l2_fd, VIDIOC_QBUF, &buf);
        }
    }
}


int make_wite_png(const char *outfile, int width, int height, int buf_idx)
{
	FILE *fp; 
	png_structp png_ptr;
	png_infop info_ptr;
	unsigned char *row;
	int y;
	int row_width;
	int bit_depth = 1;
	int color_type = PNG_COLOR_TYPE_RGB;
    int j = 0, min_w = 640, min_h = 480;
    int i, k, v = 0;

    unsigned char *rgb888_buf = (unsigned char*)malloc(2 * min_w * 24 / 8 * min_h + 10);
    //printf("---------- malloc size = %d\n", 2 * min_w * 24 / 8 * min_h + 10);
    //unsigned char *rgb888_buf_reverse = (unsigned char*)malloc(min_w * 24 / 8 * min_h + 10);
 
	if(color_type == PNG_COLOR_TYPE_GRAY){
		bit_depth = 1;
		row_width = width;
	} else if(color_type == PNG_COLOR_TYPE_RGB) {
		bit_depth = 8;
		row_width = width*3;  //width=640时，640个像素*3(bytes)
	} else if(color_type == PNG_COLOR_TYPE_RGBA) {
		bit_depth = 8;
		row_width = width*4;
	} else {
		printf("not support color\n");
		return -1;
	}
 
	row = (unsigned char *)malloc(row_width);
	if(row == NULL) {
		printf("Failed to allocate memory.\n");
		return -1;
	}
 
	fp = fopen(outfile, "wb");
	if(fp == NULL) {
		printf("fail to create %s\n", outfile);
		free(row);
		return -1;
	}
 
	png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	if(png_ptr == NULL) {
		printf("Failed to png_create_write_struct\n");
		fclose(fp);
		free(row);
		return -1;
	}
 
	info_ptr = png_create_info_struct(png_ptr);
	if(info_ptr == NULL) {
		printf("Failed to png_create_info_struct\n");
		fclose(fp);
		free(row);
		return -1;
	}
	if(setjmp(png_jmpbuf(png_ptr)))
	{
		printf("call png_jmpbuf fail!\n");
		fclose(fp);
		free(row);
		return -1;
	}
 
	png_init_io(png_ptr, fp);
 
	png_set_IHDR(png_ptr, info_ptr,
			width, height,
			bit_depth,//bit_depth
			color_type,//color_type
			PNG_INTERLACE_NONE,//interlace or not
			PNG_COMPRESSION_TYPE_DEFAULT,//compression
			PNG_FILTER_TYPE_DEFAULT);//filter
	png_set_packing(png_ptr);
	png_write_info(png_ptr, info_ptr);

	/* data -- test   success!*/  
	// for(y = 0; y < height; y++) {
	// 	memset(row, 0xff, row_width);//white
	// 	//memset(row, 0x0, width);//black
	// 	png_write_row(png_ptr, row);
	// }

    //printf("start to change!\r\n");
    //printf("buf_infos[0].len = %d\r\n", buf_infos[0].length);  //614400

    //printf(">>>>>>>>>>>>>>>>> buf_idx = %d\n", buf_idx);
    //printf(">>>>>>>>>>> lenth = %d\n", buf_infos[buf_idx].length);
    //printf(">>>>>>> buf_infos[%d].start = %d\n", buf_idx, buf_infos[buf_idx].start);

    //格式转换
    for (i = 0; i < buf_infos[buf_idx].length; i++) {

        unsigned short *pixel = buf_infos[buf_idx].start + i;

        // 逆变换每个16位像素
        unsigned char r = ((*pixel >> 8) & 0xF8) | ((*pixel >> 13) & 0x07);
        unsigned char g = ((*pixel >> 3) & 0xFC) | ((*pixel >> 9) & 0x03);
        unsigned char b = ((*pixel << 3) & 0xF8) | ((*pixel >> 2) & 0x07);

        // if(i > 610000)
        //     printf("i = %d, r = %d, g = %d, b = %d\r\n", i, r, g, b);
        if(buf_idx == 1){
            // if(i > 300000) printf("i = %d\n", i);
        }
        // 将RGB888数据写入缓冲区
        // if(buf_idx == 1 && i > 300000) printf(">>>start\n");
        //问题在这里
        rgb888_buf[j++] = r;
        rgb888_buf[j++] = g;
        rgb888_buf[j++] = b;
        // if(buf_idx == 1 && i > 300000) printf(">>>end\n");  
    }

    // for(i = j - row_width; i >= 0; i -= row_width) {
    //     for(k = 0; k < row_width; k++){
    //         rgb888_buf_reverse[v++] = rgb888_buf[i + k];
    //     }
    // }

    //printf("change success!\r\n");

    //写入png，此时操作的是rgb888_buf
    //unsigned char *start = rgb888_buf_reverse;
    unsigned char *start = rgb888_buf;
    for(i = 0; i < min_h; i++) {  //循环每一行
        memcpy(row, start, row_width);  //row_width=width*3，表示一行的字节总数
        start += row_width;   //指向下一行数据  地址移动时以字节为单位的
        png_write_row(png_ptr, row);
    }

    //printf("write rgb888_buf success!\r\n");

 
	png_write_end(png_ptr, info_ptr);
	png_destroy_write_struct(&png_ptr, &info_ptr);
	fclose(fp);
	free(row);
//怀疑row和rgb888_buf有关系
    free(rgb888_buf);  //后来打开的
    //free(rgb888_buf_reverse);
	
	return 0;
}


//SIGIO信号处理函数
static void sigio_signal_func(int signum){

    int err = 0;
    unsigned int keyvalue = 0;

    err = read(key_fd, &keyvalue, sizeof(keyvalue));
    if(err < 0) {
        /* 读取错误 */
    } else {
        printf("sigio signal! key value=%d\r\n", keyvalue);
    }

    key_flag = 1;
    key_cc = !key_cc;  //变成1表示要进入休眠，变成0表示要唤醒
    printf("key_cc = %d\n", key_cc);
}

// static ei = 0;
//ctrl+c退出
void int_exit(int _) {
	//删除当前目录下的my_pic*图片
    // char path[20];
    // for(ei = 0; ei <= pic_idx; ei++){
    //     sprintf(path, "./my_pic_%d.png", ei);
    //     if(remove(path)) printf("remove failed!\n");
    // }
    pthread_detach(model_thread_id);   //分离线程         
    pthread_detach(show_thread_id);   //分离线程

	exit(0);
}


//>>main

//别忘了修改cmake，加入链接库
int main(int argc, char *argv[])
{
//    if(argc != 2){
// 	   fprintf(stderr, "Usage: %s [imagepath]\n", argv[0]);
// 	   return -1;
//    } 
// 	imagepath = argv[1];
//     test_fast_yolov2();
//    return 1;

    //show_png
    // struct fb_fix_screeninfo fb_fix;
    // struct fb_var_screeninfo fb_var;
    // unsigned int screen_size;
    // int fd;

    int flags = 0;

    key_fd = open("/dev/asyncnoti", O_RDWR);
    if(key_fd < 0){
        printf("Can't open file %s\r\n", key_fd);
        return -1;
    }

    //设置信号SIGIO的处理函数
    signal(SIGIO, sigio_signal_func);

    signal(SIGINT, int_exit);

    fcntl(key_fd, F_SETOWN, getpid()); //将当前进程的进程号告诉内核
    flags = fcntl(key_fd, F_GETFD);  //获取当前的进程状态
    fcntl(key_fd, F_SETFL, flags | FASYNC); //设置进程启用异步通知功能,会调用驱动中的.fasync

    if (argc != 2) {
        fprintf(stderr, "Usage: %s <video_dev>\n", argv[0]);
        exit(EXIT_FAILURE);
    }

    /* 初始化LCD */
    if (fb_dev_init())
        exit(EXIT_FAILURE);

    /* 初始化摄像头 */
    if (v4l2_dev_init(argv[1]))  //dev/video0
        exit(EXIT_FAILURE);

    /* 枚举所有格式并打印摄像头支持的分辨率及帧率 */
    v4l2_enum_formats();
    v4l2_print_formats();

    /* 设置格式 */
    if (v4l2_set_format())
        exit(EXIT_FAILURE);

    /* 初始化帧缓冲：申请、内存映射、入队 */
    if (v4l2_init_buffer())
        exit(EXIT_FAILURE);

    /* 开启视频采集 */
    if (v4l2_stream_on())
        exit(EXIT_FAILURE);

    /* 读取数据：出队 */
    v4l2_read_data();       //在函数内循环采集数据、将其显示到LCD屏

    close(key_fd);
    exit(EXIT_SUCCESS);

}



