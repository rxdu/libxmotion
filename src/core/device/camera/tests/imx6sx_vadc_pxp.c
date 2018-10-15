/*
 * imx6sx_vadc_pxp.c - Sample application to process VADC output via PXP and
 * output to framebuffer. Application also demonstrates vertical and horizontal
 * flipping capabilites of the pxp. Currently we assume output display supports
 * 32 bbp, you will need to modify the code to support an LCD with 18/24 bbp.
 *
 * Output may have tearing because we don't wait for vsync on the framebuffer.
 * Furthermore because we don't employ GPU to output to the framebuffer
 * therefore the capture rate may be limited and cpu usage high.
 *
 * Copyright (C) 2015 Jasbir Matharu.
 * Copyright (C) 2010-2014 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <fcntl.h>
#include <linux/mxcfb.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <asm/types.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <linux/videodev2.h>
#include <malloc.h>
#include <signal.h>
#include <stdint.h>
#include <sys/time.h>
#include <unistd.h>

#include "pxp_lib.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define DBG_DEBUG 3
#define DBG_INFO 2
#define DBG_WARNING 1
#define DBG_ERR 0

static int debug_level = DBG_INFO;
#define dbg(flag, fmt, args...)                               \
	{                                                         \
		if (flag <= debug_level)                              \
			printf("%s:%d " fmt, __FILE__, __LINE__, ##args); \
	}

char *usage =
	"Usage: ./imx6sx_vdac_2d "
	"-H display this help \n "
	"\n"
	"options for each instance\n "
	"  -h <horizontal flip> \n "
	"  -v <vertical flip> \n "
	"  -t time in seconds to enable camera \n"
	"   -i <pixel inversion> \n ";

#define LCD_STR_ID "mxs-lcdif"

int fd_fb = 0;
unsigned short *fb0;
int g_fb0_size;

sigset_t sigset;
int quitflag;
unsigned int marker_val = 1;

struct cmd_line
{
	int hflip;
	int vflip;
	int left;
	int top;
	int pixel_inversion;
};

struct input_argument
{
	pthread_t tid;
	struct cmd_line cmd;
};

static struct input_argument input_arg;

sigset_t sigset;
int quitflag;

static char *options = "hvit:";

#define TEST_BUFFER_NUM 3
#define MAX_V4L2_DEVICE_NR 64

struct testbuffer
{
	unsigned char *start;
	size_t offset;
	unsigned int length;
};

struct testbuffer buffers[TEST_BUFFER_NUM];
struct pxp_mem_desc mem[TEST_BUFFER_NUM];

int g_out_width = 720;
int g_out_height = 480;
int g_cap_fmt = PXP_PIX_FMT_VUY444;
int g_capture_mode = 0;
int g_camera_framerate = 30; /* 30 fps */
int g_loop = 0;
int g_frame_size;
char g_v4l_device[] = "/dev/video0";
int g_timeout = 30;

int fd_v4l;
int g_fb_phys;

/* Simply memcpy the image to the framebuffer, not the most efficent but it
 * works ! */
void copy_image_to_fb(int left, int top, int width, int height, uint *img_ptr,
					  struct fb_var_screeninfo *screen_info)
{
	int i;
	uint *fb_ptr = (uint *)fb0;
	uint bytes_per_pixel;

	if ((width > screen_info->xres) || (height > screen_info->yres))
	{
		dbg(DBG_ERR, "Bad image dimensions!\n");
		return;
	}

	bytes_per_pixel = screen_info->bits_per_pixel / 8;

	dbg(DBG_DEBUG,
		" left %d top %d width %d height %d xres %d yres %d bpp %d \n", left,
		top, width, height, screen_info->xres, screen_info->yres,
		screen_info->bits_per_pixel);

	for (i = 0; i < height; i++)
	{
		memcpy(fb_ptr +
				   ((i + top) * screen_info->xres + left) * bytes_per_pixel / 4,
			   img_ptr + (i * width) * bytes_per_pixel / 4,
			   width * bytes_per_pixel);
	}
}

int start_capturing(int fd_v4l)
{
	unsigned int i;
	struct v4l2_buffer buf;
	enum v4l2_buf_type type;
	struct v4l2_requestbuffers req;

	memset(&req, 0, sizeof(req));
	req.count = TEST_BUFFER_NUM;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_USERPTR;

	if (ioctl(fd_v4l, VIDIOC_REQBUFS, &req) < 0)
	{
		dbg(DBG_ERR, "VIDIOC_REQBUFS failed\n");
		return -1;
	}

	for (i = 0; i < TEST_BUFFER_NUM; i++)
	{
		memset(&buf, 0, sizeof(buf));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_USERPTR;
		buf.index = i;
		buf.length = buffers[i].length;
		buf.m.userptr = (unsigned long)buffers[i].start;

		if (ioctl(fd_v4l, VIDIOC_QBUF, &buf) < 0)
		{
			dbg(DBG_ERR, "VIDIOC_QBUF error\n");
			return -1;
		}
	}

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(fd_v4l, VIDIOC_STREAMON, &type) < 0)
	{
		dbg(DBG_ERR, "VIDIOC_STREAMON error\n");
		return -1;
	}

	return 0;
}

int stop_capturing(int fd_v4l)
{
	enum v4l2_buf_type type;

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	return ioctl(fd_v4l, VIDIOC_STREAMOFF, &type);
}

static int open_video_device(void)
{
	struct v4l2_capability cap;
	int fd_v4l;

	if ((fd_v4l = open(g_v4l_device, O_RDWR, 0)) < 0)
	{
		dbg(DBG_ERR, "unable to open %s for capture device.\n", g_v4l_device);
	}

	if (ioctl(fd_v4l, VIDIOC_QUERYCAP, &cap) == 0)
	{
		if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)
		{
			dbg(DBG_ERR, "Found v4l2 capture device %s.\n", g_v4l_device);
			return fd_v4l;
		}
	}
	else
		close(fd_v4l);

	return fd_v4l;
}

static void print_pixelformat(char *prefix, int val)
{
	dbg(DBG_INFO, "%s: %c%c%c%c\n", prefix ? prefix : "pixelformat", val & 0xff,
		(val >> 8) & 0xff, (val >> 16) & 0xff, (val >> 24) & 0xff);
}

int v4l_capture_setup(void)
{
	struct v4l2_format fmt;
	struct v4l2_streamparm parm;
	struct v4l2_fmtdesc fmtdesc;
	struct v4l2_frmsizeenum frmsize;
	int fd_v4l = 0;

	if ((fd_v4l = open_video_device()) < 0)
	{
		dbg(DBG_ERR, "Unable to open v4l2 capture device.\n");
		return -1;
	}

	parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	parm.parm.capture.capturemode = g_capture_mode;
	parm.parm.capture.timeperframe.denominator = g_camera_framerate;
	parm.parm.capture.timeperframe.numerator = 1;
	if (ioctl(fd_v4l, VIDIOC_S_PARM, &parm) < 0)
	{
		dbg(DBG_ERR, "VIDIOC_S_PARM failed\n");
		return -1;
	}

	fmtdesc.index = 0;
	fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (ioctl(fd_v4l, VIDIOC_ENUM_FMT, &fmtdesc) < 0)
	{
		dbg(DBG_ERR, "VIDIOC ENUM FMT failed \n");
		close(fd_v4l);
		return -1;
	}

	print_pixelformat("pixelformat (output by camera)", fmtdesc.pixelformat);
	g_cap_fmt = fmtdesc.pixelformat;

	frmsize.pixel_format = fmtdesc.pixelformat;
	frmsize.index = g_capture_mode;
	if (ioctl(fd_v4l, VIDIOC_ENUM_FRAMESIZES, &frmsize) < 0)
	{
		dbg(DBG_ERR, "get capture mode %d framesize failed\n", g_capture_mode);
		return -1;
	}

	g_out_width = frmsize.discrete.width;
	g_out_height = frmsize.discrete.height;

	memset(&fmt, 0, sizeof(fmt));
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.pixelformat = g_cap_fmt;
	fmt.fmt.pix.width = g_out_width;
	fmt.fmt.pix.height = g_out_height;
	if (ioctl(fd_v4l, VIDIOC_S_FMT, &fmt) < 0)
	{
		dbg(DBG_ERR, "set format failed\n");
		return -1;
	}

	if (ioctl(fd_v4l, VIDIOC_G_FMT, &fmt) < 0)
	{
		dbg(DBG_ERR, "get format failed\n");
		return -1;
	}

	memset(&parm, 0, sizeof(parm));
	parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(fd_v4l, VIDIOC_G_PARM, &parm) < 0)
	{
		dbg(DBG_ERR, "VIDIOC_G_PARM failed\n");
		parm.parm.capture.timeperframe.denominator = g_camera_framerate;
	}
	dbg(DBG_INFO, "\t WxH@fps = %dx%d@%d \t Image size = %d \n",
		fmt.fmt.pix.width, fmt.fmt.pix.height,
		parm.parm.capture.timeperframe.denominator, fmt.fmt.pix.sizeimage);

	g_frame_size = fmt.fmt.pix.sizeimage;

	return fd_v4l;
}

void memfree(int buf_size, int buf_cnt)
{
	int i;

	for (i = 0; i < buf_cnt; i++)
	{
		if (buffers[i].start)
		{
			pxp_put_mem(&mem[i]);
			buffers[i].start = NULL;
		}
	}
}

int memalloc(int buf_size, int buf_cnt)
{
	int i, ret = 0;

	for (i = 0; i < buf_cnt; i++)
	{
		memset(&mem[i], 0, sizeof(struct pxp_mem_desc));
		buffers[i].length = mem[i].size = buf_size;
		ret = pxp_get_mem(&mem[i]);
		if (ret < 0)
		{
			dbg(DBG_ERR, "Get PHY memory failed\n");
			ret = -1;
			goto err;
		}
		buffers[i].offset = mem[i].phys_addr;
		buffers[i].start = (unsigned char *)mem[i].virt_uaddr;
		if (!buffers[i].start)
		{
			dbg(DBG_ERR, "memalloc failed\n");
			ret = -1;
			goto err;
		}
		dbg(DBG_DEBUG, "%s, buf_size=0x%x\n", __func__, buf_size);
		dbg(DBG_DEBUG, "USRP: alloc bufs va=0x%x, pa=0x%x, size %d\n",
			(unsigned int)buffers[i].start, (unsigned int)buffers[i].offset,
			buf_size);
	}

	return ret;
err:
	memfree(buf_size, buf_cnt);
	return ret;
}

int parse_args(int argc, char *argv[])
{
	int status = 0, opt;

	do
	{
		opt = getopt(argc, argv, options);
		switch (opt)
		{
		case 'h':
			input_arg.cmd.hflip = 1;
			break;
		case 'v':
			input_arg.cmd.vflip = 1;
			break;
		case 't':
			g_timeout = atoi(optarg);
			break;
		case 'i':
			input_arg.cmd.pixel_inversion = 1;
			break;
		case -1:
			break;
		default:
			status = -1;
			break;
		}
	} while ((opt != -1) && (status == 0));

	optind = 1;
	return status;
}

int vadc_test(void *arg)
{
	struct pxp_config_data *pxp_conf = NULL;
	struct pxp_proc_data *proc_data = NULL;
	int ret = 0, i;

	struct pxp_mem_desc mem_o;
	pxp_chan_handle_t pxp_chan;
	struct fb_var_screeninfo var;
	struct fb_fix_screeninfo fix;
	char fb_dev[10] = "/dev/fb";
	int fb_num = 0;
	int width, height;
	int out_w = 0, out_h = 0;
	struct v4l2_buffer buf;
	struct timeval tv1, tv2;

	struct cmd_line *cmdl = (struct cmd_line *)arg;

	dbg(DBG_DEBUG, "hflip = %d\n", cmdl->hflip);
	dbg(DBG_DEBUG, "vflip = %d\n", cmdl->vflip);
	dbg(DBG_DEBUG, "display_left = %d\n", cmdl->left);
	dbg(DBG_DEBUG, "display_top = %d\n", cmdl->top);
	dbg(DBG_DEBUG, "pixel_inversion = %d\n", cmdl->pixel_inversion);

	ret = pxp_request_channel(&pxp_chan);
	if (ret < 0)
	{
		dbg(DBG_ERR, "pxp request channel err\n");
		goto err0;
	}
	dbg(DBG_DEBUG, "requested chan handle %d\n", pxp_chan.handle);

	out_w = g_out_width;
	out_h = g_out_height;

	/* Prepare the channel parameters */
	memset(&mem_o, 0, sizeof(struct pxp_mem_desc));
	mem_o.size = out_w * out_h * 4;
	ret = pxp_get_mem(&mem_o);
	if (ret < 0)
	{
		dbg(DBG_ERR, "get mem_o err\n");
		goto err1;
	}

	dbg(DBG_DEBUG,
		"mem_o.virt_uaddr %08x, mem_o.phys_addr %08x, mem_o.size %d\n",
		mem_o.virt_uaddr, (unsigned int)mem_o.phys_addr, mem_o.size);

	if (start_capturing(fd_v4l) < 0)
	{
		dbg(DBG_ERR, "start_capturing failed\n");
		goto err2;
	}

	/* Configure the channel */
	pxp_conf = malloc(sizeof(*pxp_conf));
	memset(pxp_conf, 0, sizeof(*pxp_conf));
	proc_data = &pxp_conf->proc_data;

	/* Initialize non-channel-specific PxP parameters */
	proc_data->srect.left = 0;
	proc_data->srect.top = 0;
	proc_data->drect.left = 0;
	proc_data->drect.top = 0;
	proc_data->srect.width = out_w;
	proc_data->srect.height = out_h;
	proc_data->drect.width = out_w;
	proc_data->drect.height = out_h;
	proc_data->scaling = 0;
	proc_data->hflip = cmdl->hflip;
	proc_data->vflip = cmdl->vflip;
	proc_data->rotate = 0;
	proc_data->bgcolor = 0;

	proc_data->overlay_state = 1;
	proc_data->lut_transform =
		cmdl->pixel_inversion ? PXP_LUT_INVERT : PXP_LUT_NONE;

	/*
     * Initialize S0 parameters
     */
	pxp_conf->s0_param.pixel_fmt = PXP_PIX_FMT_VUY444;
	pxp_conf->s0_param.width = out_w;
	pxp_conf->s0_param.height = out_h;
	pxp_conf->s0_param.color_key = -1;
	pxp_conf->s0_param.color_key_enable = false;
	pxp_conf->s0_param.global_alpha_enable = false;
	pxp_conf->s0_param.global_alpha = -1;

	dbg(DBG_DEBUG, "pxp_test s0 paddr %08x\n",
		(unsigned int)pxp_conf->s0_param.paddr);

	/*
     * No overlay will be used for PxP operation
     */
	for (i = 0; i < 8; i++)
	{
		memset(&pxp_conf->ol_param[i], 0, sizeof(struct pxp_layer_param));
	}

	/*
     * Initialize Output channel parameters
     */
	pxp_conf->out_param.width = out_w;
	pxp_conf->out_param.height = out_h;
	pxp_conf->out_param.pixel_fmt = PXP_PIX_FMT_RGB32;
	;
	pxp_conf->out_param.stride = out_w;

	// while (1) {
	// fb_dev[7] = '0' + fb_num;
	// dbg(DBG_INFO, "opening this fb_dev - %s\n", fb_dev);
	// fd_fb = open(fb_dev, O_RDWR);
	// if (fd_fb < 0) {
	//    dbg(DBG_ERR, "Unable to open %s\n", fb_dev);
	//    goto err2;
	//}

	/* Check that fb device is LCD */
	/* First get screen_info */
	// ret = ioctl(fd_fb, FBIOGET_FSCREENINFO, &fix);
	// if (ret < 0) {
	//    dbg(DBG_ERR, "Unable to read fixed screeninfo for %s\n", fb_dev);
	//    goto err3;
	//}

	/* If we found LCD, exit loop */
	// if (!strcmp(LCD_STR_ID, fix.id)) {
	//    g_fb_phys = fix.smem_start;
	//    break;
	//	}

	//	fb_num++;
	//  }

	// ret = ioctl(fd_fb, FBIOGET_VSCREENINFO, &var);
	// if (ret < 0) {
	// dbg(DBG_ERR, "FBIOPUT_VSCREENINFO error\n");
	// goto err3;
	//}

	// g_fb0_size = var.xres * var.yres * var.bits_per_pixel / 8;
	// dbg(DBG_DEBUG, "g_fb0_size %d\n", g_fb0_size);

	// fb0 = (unsigned short *)mmap(0, g_fb0_size, PROT_READ | PROT_WRITE,
	// MAP_SHARED, fd_fb, 0);
	// if ((int)fb0 <= 0) {
	// dbg(DBG_ERR,
	//    "\nError: failed to map framebuffer
	//    device 0 to memory.\n");
	// goto err3;
	//}

	pxp_conf->out_param.paddr = mem_o.phys_addr;
	dbg(DBG_DEBUG, "pxp_test out paddr %08x\n",
		(unsigned int)pxp_conf->out_param.paddr);

	gettimeofday(&tv1, NULL);

	do
	{
		memset(&buf, 0, sizeof(buf));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_USERPTR;
		if (ioctl(fd_v4l, VIDIOC_DQBUF, &buf) < 0)
		{
			dbg(DBG_ERR, "VIDIOC_DQBUF failed.\n");
		}

		pxp_conf->s0_param.paddr = mem[buf.index].phys_addr;

		ret = pxp_config_channel(&pxp_chan, pxp_conf);
		if (ret < 0)
		{
			dbg(DBG_ERR, "pxp config channel err %d\n", ret);
			goto err3;
		}

		ret = pxp_start_channel(&pxp_chan);
		if (ret < 0)
		{
			dbg(DBG_ERR, "pxp start channel err %d\n", ret);
			goto err3;
		}

		ret = pxp_wait_for_completion(&pxp_chan, 3);
		if (ret < 0)
		{
			dbg(DBG_ERR, "pxp wait for completion err %d\n", ret);
			goto err3;
		}

		if (ioctl(fd_v4l, VIDIOC_QBUF, &buf) < 0)
		{
			dbg(DBG_ERR, "VIDIOC_QBUF failed\n");
			break;
		}

		width = out_w;
		height = out_h;

		// copy_image_to_fb(0, 0, width, height,
		// (uint *)mem_o.virt_uaddr,
		// &var);
		printf("saving image\n");

		IplImage *frame;
		CvMat cvmat = cvMat(480, 640, CV_8UC3, (void *)mem_o.virt_uaddr);
		frame = cvDecodeImage(&cvmat, 1);
		// cvNamedWindow("window", CV_WINDOW_AUTOSIZE);
		// cvShowImage("window", frame);
		// cvWaitKey(0);
		cvSaveImage("image.png", frame, 0);

		gettimeofday(&tv2, NULL);

	} while ((tv2.tv_sec - tv1.tv_sec < g_timeout) && !quitflag);

	if (stop_capturing(fd_v4l) < 0)
		dbg(DBG_ERR, "stop_capturing failed\n");

	dbg(DBG_INFO, "imx6sx_vdac_pxp finished!\n");

err3:
	munmap(fb0, g_fb0_size);
	close(fd_fb);
err2:
	free(pxp_conf);
	pxp_put_mem(&mem_o);
err1:
	pxp_release_channel(&pxp_chan);
err0:
	return ret;
}

static int signal_thread(void *arg)
{
	int sig;

	pthread_sigmask(SIG_BLOCK, &sigset, NULL);

	while (1)
	{
		sigwait(&sigset, &sig);
		if (sig == SIGINT)
		{
			dbg(DBG_INFO, "Ctrl-C received\n");
		}
		else
		{
			dbg(DBG_ERR, "Unknown signal. Still exiting\n");
		}
		quitflag = 1;
		break;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	pthread_t sigtid;
	int ret, err;

	err = parse_args(argc, argv);
	if (err)
	{
		goto usage;
	}

	ret = pxp_init();
	if (ret < 0)
	{
		dbg(DBG_ERR, "pxp init err %d \n", ret);
		return -1;
	}

	fd_v4l = v4l_capture_setup();
	if (fd_v4l < 0)
	{
		pxp_uninit();
		return -1;
	}

	if (memalloc(g_frame_size, TEST_BUFFER_NUM) < 0)
	{
		close(fd_v4l);
		pxp_uninit();
		dbg(DBG_ERR, "memalloc failed \n");
		return -1;
	}

	sigemptyset(&sigset);
	sigaddset(&sigset, SIGINT);
	pthread_sigmask(SIG_BLOCK, &sigset, NULL);
	pthread_create(&sigtid, NULL, (void *)&signal_thread, NULL);

	pthread_create(&input_arg.tid, NULL, (void *)&vadc_test,
				   (void *)&input_arg.cmd);

	if (input_arg.tid != 0)
	{
		pthread_join(input_arg.tid, NULL);
	}

	memfree(g_frame_size, TEST_BUFFER_NUM);

	pxp_uninit();

	return 0;
usage:
	dbg(DBG_INFO, "%s", usage);
	return -1;
}
