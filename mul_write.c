#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/fb.h>  //官方屏幕驱动头文件  
	 
#define FBIPUT_UPDATE   _IOW('F', 10, int)

struct update_info
{
        unsigned short x;//更新的起点x
        unsigned short y;//更新的起点y
        unsigned short w;//宽度
        unsigned short h;//高度
        void* src;//数据
}data ={
        .x = 0,
        .y = 0,
        .w =320,
        .h = 240
};

 
int main()
{
	//打开LCD屏幕 
	int lcdfd =  open("/dev/fb1",O_RDWR);
		if(lcdfd < 0) 
		{
			perror("open lcd fail\n");
		}
	printf("open success\n");
	//获取屏幕参数 
	struct  fb_var_screeninfo fbinfo;
	int r = ioctl(lcdfd,FBIOGET_VSCREENINFO , &fbinfo);  //ioctl  函数 
	if (r == -1)
	{
		perror("ioctl error:");
		return -1;
	}

	#if 0	
	/*打印屏幕参数*/
	printf("resuation: %d x %d\n", fbinfo.xres, fbinfo.yres);
	printf("bits_per_pixel: %d bits\n", fbinfo.bits_per_pixel);
	printf("A: offset %d  length %d msb_rigth: %d\n ", fbinfo.transp.offset, fbinfo.transp.length, fbinfo.transp.msb_right);
	printf("R: offset %d  length %d msb_rigth: %d\n ", fbinfo.red.offset, fbinfo.red.length, fbinfo.red.msb_right);
	printf("G: offset %d  length %d msb_rigth: %d\n ", fbinfo.green.offset, fbinfo.green.length, fbinfo.green.msb_right);
	printf("B: offset %d  length %d msb_rigth: %d\n ", fbinfo.blue.offset, fbinfo.blue.length, fbinfo.blue.msb_right);
	#endif
	
	//写入数据到lcd屏幕中  
	unsigned short int buf[320*240]={0};
	int i=0;
	lseek(lcdfd , 0 , SEEK_SET); //文件头

	for(i = 0; i < 320*240 ;i++)
		buf[i] = 0xf800;

	i = 0;
	
//	lseek(lcdfd , sizeof(buf)/4 , SEEK_SET); //文件头
	while(i < sizeof(buf)/2 )	{
	//	write(lcdfd,&buf[i],sizeof(buf)/10);
	//	write(lcdfd,&buf[i],15360);
	//	i+= 15360/2;
		write(lcdfd,&buf[i],4096);
		i+= 4096/2;
		sleep(1);
	}

//	ioctl(lcdfd,FBIPUT_UPDATE,&data);//更新屏幕
	lseek(lcdfd , 0 , SEEK_SET); //文件头
	close(lcdfd);
	
}
