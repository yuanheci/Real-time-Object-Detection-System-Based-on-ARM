#include <stdio.h>
#include <stdlib.h>
#include <png.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <errno.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <linux/fb.h>
#include "linux/ioctl.h"
#include "signal.h"

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
static unsigned short *screen_base = NULL;//LCD显存基地址
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

int make_wite_png(const char *outfile, int width, int height);

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

    screen_size = fb_fix.line_length * fb_var.yres;
    width = fb_var.xres;
    height = fb_var.yres;

    /* 内存映射 */
    screen_base = mmap(NULL, screen_size, PROT_READ | PROT_WRITE, MAP_SHARED, fb_fd, 0);
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
        strcpy(cam_fmts[fmtdesc.index].description, fmtdesc.description);
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
        buf_infos[buf.index].start = mmap(NULL, buf.length,
                PROT_READ | PROT_WRITE, MAP_SHARED,
                v4l2_fd, buf.m.offset);
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

char path[18] = "";
static ii = 0, jj = 0;

static void v4l2_read_data(void)
{
    struct v4l2_buffer buf = {0};
    unsigned short *base;
    unsigned short *start;
    int min_w, min_h;
    int j = 0;

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
    for ( ; ; ) {
        //按下按键后，当前进程睡眠，开启一个子进程，fork+exec去执行ncnn的内容，处理完后将结果显示到LCD，子进程结束。
        //当再次按下按键时，从睡眠中唤醒，重新进行摄像头采集。
        if(key_cc == 1){  //要休眠   
            printf("======== saving png ========\r\n");
            sprintf(name, "my_pic_%d.png", pic_idx++);
    
            make_wite_png(name, 640, 480);

            printf("======== png saved! ========\r\n");
            ii = 0;
            path[ii++] = '.', path[ii++] = '/';
            for(jj = 0; jj < 12; jj++) path[ii++] = name[jj];
            printf("image path = %s\n", path);

            //创建子进程
            pid_t pid = fork();
            if (pid == -1) {
                // 处理错误
                perror("fork");
                exit(EXIT_FAILURE);
            } else if (pid == 0) {
                // 子进程
                //printf("子进程开始执行...\n");

#if 0
                execl("../ncnn/yolov5", "yolov5", path, NULL); // 执行./yolov5 ./image.png命令
#else
                execl("../ncnn/FastestDet", "FastestDet", path, NULL); // 执行./yolov5 ./image.png命令
#endif

                perror("execl"); // 只有当execl执行失败时才会执行到这里
                exit(EXIT_FAILURE);
            } else{
                //父进程去休眠
                pause();    //等待信号
            }

            //走到这里时，父进程已经被信号唤醒
        }
        //key_cc = 0就相当于一个唤醒而已

        for(buf.index = 0; buf.index < FRAMEBUFFER_COUNT; buf.index++) {

            ioctl(v4l2_fd, VIDIOC_DQBUF, &buf);     //出队

            // for (j = 0, base=screen_base, start=buf_infos[buf.index].start;
            //             j < min_h; j++) {

            //     memcpy(base, start, min_w * 2); //RGB565 一个像素占2个字节
            //     base += width;  //LCD显示指向下一行
            //     start += frm_width;//指向下一行数据
            // }

            //更改方向
            for (j = min_h - 1, base=screen_base, start=buf_infos[buf.index].start;
                        j >= 0; j--) {

                memcpy(base + j * width, start, min_w * 2); //RGB565 一个像素占2个字节
                // base += width;  //LCD显示指向下一行
                start += frm_width;//指向下一行数据
            }

            // 数据处理完之后、再入队、往复
            ioctl(v4l2_fd, VIDIOC_QBUF, &buf);
        }
    }
}


int make_wite_png(const char *outfile, int width, int height)
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

    unsigned char *rgb888_buf = (unsigned char*)malloc(min_w * 24 / 8 * min_h + 10);
    unsigned char *rgb888_buf_reverse = (unsigned char*)malloc(min_w * 24 / 8 * min_h + 10);
 
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

    printf("start to change!\r\n");
    //printf("buf_infos[0].len = %d\r\n", buf_infos[0].length);  //614400

    //格式转换
    for (i = 0; i < buf_infos[0].length; i++) {
        unsigned short *pixel = buf_infos[0].start + i;

        // 逆变换每个16位像素
        unsigned char r = ((*pixel >> 8) & 0xF8) | ((*pixel >> 13) & 0x07);
        unsigned char g = ((*pixel >> 3) & 0xFC) | ((*pixel >> 9) & 0x03);
        unsigned char b = ((*pixel << 3) & 0xF8) | ((*pixel >> 2) & 0x07);

        // if(i > 610000)
        //     printf("i = %d, r = %d, g = %d, b = %d\r\n", i, r, g, b);

        // 将RGB888数据写入缓冲区
        rgb888_buf[j++] = r;
        rgb888_buf[j++] = g;
        rgb888_buf[j++] = b;
    }

    // for(i = j - row_width; i >= 0; i -= row_width) {
    //     for(k = 0; k < row_width; k++){
    //         rgb888_buf_reverse[v++] = rgb888_buf[i + k];
    //     }
    // }

    printf("change success!\r\n");

    //写入png，此时操作的是rgb888_buf
    //unsigned char *start = rgb888_buf_reverse;
    unsigned char *start = rgb888_buf;
    for(i = 0; i < min_h; i++) {  //循环每一行
        memcpy(row, start, row_width);  //row_width=width*3，表示一行的字节总数
        start += row_width;   //指向下一行数据  地址移动时以字节为单位的
        png_write_row(png_ptr, row);
    }

    printf("write rgb888_buf success!\r\n");

 
	png_write_end(png_ptr, info_ptr);
	png_destroy_write_struct(&png_ptr, &info_ptr);
	fclose(fp);
	free(row);
//怀疑row和rgb888_buf有关系
    // free(rgb888_buf);
    free(rgb888_buf_reverse);
	
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

static ei = 0;
//ctrl+c退出
void int_exit(int _) {
	//删除当前目录下的my_pic*图片
    char path[20];
    for(ei = 0; ei < pic_idx; ei++){
        sprintf(path, "./my_pic_%d.png", ei);
        if(remove(path)) printf("remove failed!\n");
    }
	exit(0);
}

int main(int argc, char *argv[])
{
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
    fcntl(key_fd, F_SETFL, flags | FASYNC); //设置进程启用异步通知功能

    if (argc != 2) {
        fprintf(stderr, "Usage: %s <video_dev>\n", argv[0]);
        exit(EXIT_FAILURE);
    }

    /* 初始化LCD */
    if (fb_dev_init())
        exit(EXIT_FAILURE);

    /* 初始化摄像头 */
    if (v4l2_dev_init(argv[1]))
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
