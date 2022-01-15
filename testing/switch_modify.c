#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <signal.h>

#include <linux/fb.h>
#include <sys/mman.h>

//OpenCV ���̺귯��
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>

#define FPGA_STEP_MOTOR_DEVICE "/dev/fpga_step_motor"
#define FPGA_PUSH_SWITCH_DEVICE "/dev/fpga_push_switch"
#define FPGA_BUZZER_DEVICE "/dev/fpga_buzzer"
#define FBDEV "/dev/fb0"
#define CAMERA_COUNT 100
#define CAM_WIDTH 640
#define CAM_HEIGHT 480

#define MAX_BUTTON 9
#define MOTOR_ATTRIBUTE_ERROR_RANGE 4

unsigned char quit = 0;

void user_signal1(int sig)
{
	quit = 1;
}

int main(void)
{
	int i;

	//����ġ ���� ����
	int push_switch;
	int buff_size;
	unsigned char push_sw_buff[MAX_BUTTON];
	unsigned char motion[MAX_BUTTON] = {0,0,0,0,0,0,0,0,0};

	//���� ���� ����
	int step_motor;
	int motor_action = 0;
	int motor_direction = 0;
	int motor_speed = 10;
	unsigned char motor_state[3];

	//��ü����(PIR) ���� ����
	int pir;
	int pir_buf[10] = { 0 };
	int count = 0;

	//���� ���� ����
	int buzzer;
	unsigned char buzzer_state = 0;
	unsigned char buzzer_retval;
	unsigned char buzzer_data;

	
	int fbfd;
	/* �����ӹ��� ���� ó���� ���� ����ü */
	struct fb_var_screeninfo vinfo;
	struct fb_fix_screeninfo finfo;

	unsigned char *buffer, rr, gg, bb;
	unsigned int xx, yy, tt, ii, screensize;
	unsigned short *pfbmap, *pOutdata, pixel;
	CvCapture* capture;                  /* ī�޶� ���� ���� */
	IplImage *frame;

	
	
	//motor_state �ʱ�ȭ
	memset(motor_state, 0, sizeof(motor_state));

	//����ġ ����
	push_switch = open(FPGA_PUSH_SWITCH_DEVICE, O_RDWR);
	if (push_switch < 0) {
		printf("Device Open Error\n");
		close(push_switch);
		return -1;
	}

	//PIR ����
	pir = open("/dev/ext_pir1_sens", O_RDWR);
	if (pir < 0) {
		perror("/dev/ext_pir1_sens error");
		exit(-1);
	}
	else {
		printf("< ext_pir1_sens device has been detected >\n");
	}

	//BUZZER ����
	buzzer = open(FPGA_BUZZER_DEVICE, O_RDWR);

	

	(void)signal(SIGINT, user_signal1);
	buff_size = sizeof(push_sw_buff);
	printf("Press <ctrl+c> to quit.\n");


	pid_t pid;
	pid = fork();
	if (pid == -1)
	{
		perror("fork error!!\n");
		exit(0);
	}
	else if (pid == 0)
	{
		while (!quit) {
			capture = cvCaptureFromCAM(2);
			cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, CAM_WIDTH);
			cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT);
			fbfd = open(FBDEV, O_RDWR);

			if (ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo) == -1) {
				perror("Error reading variable information.");
				exit(EXIT_FAILURE);
			}

			screensize = vinfo.xres * vinfo.yres * 2;
			pfbmap = (unsigned short *)mmap(NULL, screensize, PROT_READ | PROT_WRITE,
				MAP_SHARED, fbfd, 0);
			if ((int)pfbmap == -1) {
				perror("mmap() : framebuffer device to memory");
				return EXIT_FAILURE;
			}
			memset(pfbmap, 0, screensize);

			/* ����� ������ ���� ���� Ȯ�� */
			pOutdata = (unsigned short*)malloc(screensize);
			memset(pOutdata, 0, screensize);
			while (1) {
				cvGrabFrame(capture);                          /* ī�޶�� ���� ������ ������ �����´�. */
				frame = cvRetrieveFrame(capture);        /* ī�޶� ���󿡼� �̹��� �����͸� ȹ�� */
				buffer = (uchar*)frame->imageData;       /* IplImage Ŭ������ ���� ������ ȹ�� */

														 /* ������ ���۷� ��� */
				for (xx = 0; xx < frame->height; xx++) {
					tt = xx * frame->width;
					for (yy = 0; yy < frame->width; yy++) {
						rr = *(buffer + (tt + yy) * 3 + 2);
						gg = *(buffer + (tt + yy) * 3 + 1);
						bb = *(buffer + (tt + yy) * 3 + 0);
						pixel = ((rr >> 3) << 11) | ((gg >> 2) << 5) | (bb >> 3);

						pOutdata[yy + tt + (vinfo.xres - frame->width)*xx] = pixel;
					}
				}

				memcpy(pfbmap, pOutdata, screensize);
			};

		}
		munmap(pfbmap, frame->width*frame->height * 2);
		free(pOutdata);

		close(fbfd);
	}
	else {
		while (!quit) {
			//usleep(400000);
			read(push_switch, &push_sw_buff, buff_size);


			
			//��ư Ʈ����
			/*
			7���� ����
			6,8���� ���� �¿�ȸ��
			0���� ���α׷� ����
			*/
			if (push_sw_buff[7] == 1)
			{
				motor_action = 0;

				buzzer_data = 0;
				buzzer_retval = write(buzzer, &buzzer_data, 1);
				if (buzzer_retval < 0) {
					printf("Write Error! \n");
					return -1;
				}
			}
			else if (push_sw_buff[6] == 1)
			{
				motor_action = 1;
				motor_direction = 1;
			}
			else if (push_sw_buff[8] == 1)
			{
				motor_action = 1;
				motor_direction = 0;
			}
			else if (push_sw_buff[0]==1)
			{
				return 0;
			}


			//���� Ʈ����
			read(pir, pir_buf, 10);
			if ( pir_buf[0] == '1')
			{
				if (motor_action == 1)
				{
					printf("< Detected : %d>\n", ++count);
					motor_action = 0;
					buzzer_data = 1;
					buzzer_retval = write(buzzer, &buzzer_data, 1);
					if (buzzer_retval < 0) {
						printf("Write Error! \n");
						return -1;
					}

					sleep(1);
				}
			}
			

			

			motor_state[0] = (unsigned char)motor_action;
			motor_state[1] = (unsigned char)motor_direction;;
			motor_state[2] = (unsigned char)motor_speed;
			step_motor = open(FPGA_STEP_MOTOR_DEVICE, O_WRONLY);
			if (step_motor < 0) {
				printf("Device open error : %s\n", FPGA_STEP_MOTOR_DEVICE);
				exit(1);
			}
			write(step_motor, motor_state, 3);
			close(step_motor);




		}

		close(push_switch);
	}
}
