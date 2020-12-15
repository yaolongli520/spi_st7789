#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/firmware.h>
#include <linux/uaccess.h>
#include <mach/gpio.h>
#include <linux/dma-mapping.h>
#include <linux/mtd/mtd.h>

#include <linux/spi/spi.h>
#include <asm/mach-types.h>
#include <asm/gpio.h>
#include <asm/delay.h>
#include <mach/soc.h>
#include <mach/platform.h>


#define DPY_W 320
#define DPY_H 240
#define DPY_BPP 16


  

//4418 友善
//#define GPIO_LCD_EN_PIN  (PAD_GPIO_C +9)	//背光?
#define GPIO_LCD_RST_PIN  (PAD_GPIO_C +8)	//复位
#define GPIO_LCD_RS_PIN  (PAD_GPIO_C +7) //数据命令
#define PIN_SET_VALUE(x,y) gpio_set_value(x,!!y)

//总共3个寄存器
enum {
	LCDP_CMD = 0x01, //命令
	LCDP_DATA = 0x00,//数据
	LCDP_DELAY = 0xff,//延时
};

//初始化数据 寄存器和要写入的值
u8 st7789_init_data[] = 
{
0x01, 0x01, // reset
0xFF, 10,


0x01, 0x11,
0xFF, 120,  //Delay 120ms
//------------------------------display and color format setting--------------------------------//
0x01, 0x36,
0x00, 0x60,//0x60=0 0x00=90  0xa0=180 0xc0=270  横屏/竖屏
0x01, 0x3a,
0x00, 0x05,
//--------------------------------ST7789V Frame rate setting----------------------------------//
0x01, 0xb2,
0x00, 0x0c,
0x00, 0x0c,
0x00, 0x00,
0x00, 0x33,
0x00, 0x33,
0x01, 0xb7,
0x00, 0x35,
//---------------------------------ST7789V Power setting--------------------------------------//
0x01, 0xbb,
0x00, 0x2c,
0x01, 0xc2,
0x00, 0x01,
0x01, 0xc3,
0x00, 0x0b,
0x01, 0xc4,

0x00, 0x20,
0x01, 0xc6,
0x00, 0x0f,
0x01, 0xd0,
0x00, 0xa4,
0x00, 0xa1,
//--------------------------------ST7789V gamma setting---------------------------------------//
0x01, 0xe0,
0x00, 0xd0,
0x00, 0x01,
0x00, 0x08,
0x00, 0x0f,
0x00, 0x11,
0x00, 0x2a,
0x00, 0x36,
0x00, 0x55,
0x00, 0x44,
0x00, 0x3a,
0x00, 0x0b,
0x00, 0x06,
0x00, 0x11,
0x00, 0x20,
0x01, 0xe1,
0x00, 0xd0,
0x00, 0x02,
0x00, 0x07,
0x00, 0x0a,
0x00, 0x0b,
0x00, 0x18,
0x00, 0x34,
0x00, 0x43,
0x00, 0x4a,
0x00, 0x2b,
0x00, 0x1b,
0x00, 0x1c,
0x00, 0x22,
0x00, 0x1f,

0x01,0x2A, 
0x00,0x00, 
0x00,0x00, 
0x00,0x00, 
0x00,0xEF, 

0x01,0x2B, 
0x00,0x00, 
0x00,0x00, 
0x00,0x01, 
0x00,0x3F,

0x01,0xb0, 
0x00,0x00,
0x00,0xF8,

// 0x01,0xE7, //DATA
// 0x00,0x00,
0x01, 0x29,


};

u8 st9341_init_data[] = 
{
	0x01,0x11,
	0xFF, 120, //Delay 120ms 

	0x01,0x12, //Partial Display Mode On

	0x01,0x36, //Memory Data Access Control
	0x00,0x60, //7:MY=0, 6:MX=0, 5:MV=1 3:ML=1

	0x01,0x3A, //Interface Pixel Format
	0x00,0x55, //MCU-16bit

	0x01,0xB2, //Porch Setting
	0x00,0x0C, //Normal BP
	0x00,0x0C, //Normal FP
	0x00,0x00, //Enable Seperate
	0x00,0x33, //idle, BP[7:4], FP[3:0]
	0x00,0x33, //partial, BP[7:4], FP[3:0]

	0x01,0xB7, //Gate Control
	0x00,0x56, //VGH=14.06V, VGL=-11.38

	0x01,0xBB, //VCOMS Setting
	0x00,0x1E, 

	0x01,0xC0, 
	0x00,0x2C, 

	0x01,0xC2, 
	0x00,0x01,

	0x01,0xC3, //VRH Set
	0x00,0x13, //4.5V

	0x01,0xC4, 
	0x00,0x20,

	0x01,0xC6, //Frame Rate Control in Normal Mode
	0x00,0x0F,

	0x01,0xD0, //Power Control 1
	0x00,0xA4, //
	0x00,0xA1, //AVDD=6.8V, AVCL=-4.8V, VDS=2.3V

	//----GAMMA------------------------------------

	0x01,0xE0,
	0x00,0xD0,
	0x00,0x03,
	0x00,0x08,
	0x00,0x0E,
	0x00,0x11,
	0x00,0x2B,
	0x00,0x3B,
	0x00,0x44,
	0x00,0x4C,
	0x00,0x2B,
	0x00,0x16,
	0x00,0x15,
	0x00,0x1E,
	0x00,0x21,

	0x01,0xE1,
	0x00,0xD0,
	0x00,0x03,
	0x00,0x08,
	0x00,0x0E,
	0x00,0x11,
	0x00,0x2B,
	0x00,0x3B,
	0x00,0x54,
	0x00,0x4C,
	0x00,0x2B,
	0x00,0x16,
	0x00,0x15,
	0x00,0x1E,
	0x00,0x21,

	0x01,0x51,
	0x00,0x20,

// 	0x01,0xf6, 
// 	0x00,0x01,
// 	0x00,0x00,
// 	0x00,0x20,

	0x01,0x29, //Display ON 
};


//更新结构 指定 起点 和宽度 高度
struct update_info
{
	unsigned short x;//x
	unsigned short y;//y
	unsigned short w;//宽
	unsigned short h;//高
	void* src;//写入的新数据
};

//帧缓冲设备私有数据结构
//本驱动没用到LCD控制器
struct myfb_par {
	struct fb_info *info;//帧缓冲指针
	struct spi_device *spi;//SPI设备用于驱动SPI总线		
	
	dma_addr_t vram_phys;//DMA总线地址
	void *vram_virt;//内存的虚拟 就是帧缓冲的地址
	int vram_size;//虚拟地址长度
	
	int gpio_reset_pin;//复位引脚
	int gpio_rs_pin;//片选
	
	int lcd_little_endian;//小端。。。。

	struct mutex mlock;//互斥锁
	
	struct spi_transfer	t;//SPI传输
	struct spi_message	m;//SPI消息
};

//固定参数 全局变量
static struct fb_fix_screeninfo myfb_fix = {
	.id =		"broadsheetfb",
	.type =		FB_TYPE_PACKED_PIXELS, //填充的像素
	.visual =	FB_VISUAL_TRUECOLOR,//真彩色
	.xpanstep =	0,
	.ypanstep =	0,
	.ywrapstep =	0,//以上3个赋值0
	.accel =	FB_ACCEL_NONE,//no
};

//可变参数 全局变量
static struct fb_var_screeninfo myfb_var = {
	.xres		= DPY_W,//可见分辨率x 320
	.yres		= DPY_H,//y 240
	.xres_virtual	= DPY_W,//虚拟分辨率320
	.yres_virtual	= DPY_H,//虚拟y 240
	.bits_per_pixel	= DPY_BPP,//每个像素16bit
	.grayscale	= 1,//灰度图 //参数分别：位域偏移 长度 MSB =0
	.red			= {11, 5, 0},//红
	.green			= {5, 6, 0},//绿
	.blue			= {0, 5, 0},//蓝
	.transp		= {0, 0, 0},//透明0
};

 //gec6818
static void myfb_setup_gpio_pin( struct fb_info *info ) 
{
	struct myfb_par* par = info->par;
	int ret;
	
	gpio_free(par->gpio_reset_pin); //引脚申请
	ret = gpio_request(par->gpio_reset_pin,"gpio_reset_pin");
	
	gpio_direction_output(par->gpio_reset_pin,1); //复位脚配置输出
	PIN_SET_VALUE( par->gpio_reset_pin, 1 );
	

	gpio_free(par->gpio_rs_pin); //引脚申请
	ret = gpio_request(par->gpio_rs_pin,"gpio_rs_pin");
	
	gpio_direction_output(par->gpio_rs_pin,1);
	PIN_SET_VALUE( par->gpio_rs_pin, 1 );



	
}



//硬件复位
static void myfb_reset( struct fb_info *info ) {
	struct myfb_par* par = info->par;
	PIN_SET_VALUE( par->gpio_reset_pin, 1 );
	msleep(5);
	PIN_SET_VALUE( par->gpio_reset_pin, 0 );
	msleep(20);
	PIN_SET_VALUE( par->gpio_reset_pin, 1 );
	msleep(120);
}





//SPI同步写命令
//把SPI再次封装入info中
static void myfb_send_cmd_sync( struct fb_info *info, u8 cmd )
{
	struct myfb_par* par = info->par;
	PIN_SET_VALUE( par->gpio_rs_pin, 0 );//命令
	spi_write( par->spi, &cmd, 1 );
	PIN_SET_VALUE( par->gpio_rs_pin, 1 );//数据
}

//SPI同步写数据
static void myfb_send_data_sync( struct fb_info *info, u8 data )
{
	struct myfb_par* par = info->par;
	spi_write( par->spi, &data, 1 );
}

//LCD帧缓冲初始化
static void myfb_init_lcd( struct fb_info *info )
{
	int i;
	struct myfb_par* par = info->par;//封装了FBI的结构体
	if(1) { //默认小端
		par->lcd_little_endian = 1;//小端
		for( i = 0; i < sizeof( st7789_init_data ); i += 2 )
		{
			//写入的是命令
			if( st7789_init_data[i] == LCDP_CMD ) 
				myfb_send_cmd_sync( info, st7789_init_data[i+1] );
			else if( st7789_init_data[i] == LCDP_DATA )//数据
				myfb_send_data_sync( info, st7789_init_data[i+1] );
			else if( st7789_init_data[i] == LCDP_DELAY )
				msleep( st7789_init_data[i+1] );//LCDP_DELAY不是用SPI发出去的
		}
	}
	else {
		par->lcd_little_endian = 0;
		for( i = 0; i < sizeof( st9341_init_data ); i += 2 )
		{
			if( st9341_init_data[i] == LCDP_CMD ) 
				myfb_send_cmd_sync( info, st9341_init_data[i+1] );
			else if( st9341_init_data[i] == LCDP_DATA ) 
				myfb_send_data_sync( info, st9341_init_data[i+1] );
			else if( st9341_init_data[i] == LCDP_DELAY )
				msleep( st9341_init_data[i+1] );
		}
	}
}

unsigned long time1;

static void update_complete(void *arg)
{
	struct fb_info * info = (struct fb_info *)arg;
	struct myfb_par* par = info->par;
	mutex_unlock(&par->mlock);//解锁
	time1 = jiffies-time1;
	//时间统计
//	printk(KERN_EMERG"spend %d ms\n",jiffies_to_msecs(time1));
}


//update_lcd_data被最终调用
////buf起点所在行第一个点 len+buf是终点所在行第一个点
#define HIBYTE(a) (u8)(a>>8)  //高8位
#define LOBYTE(a) (u8)(a&0xff)//低8位
static void update_lcd_data( struct fb_info *info, u16 x1, u16 y1, u16 x2, u16 y2, u8* data, int len )
{
	struct myfb_par* par = info->par;
	
	myfb_send_cmd_sync( info, 0x2A );		//Column Address Set
	myfb_send_data_sync( info, HIBYTE(x1) );//写x1高8位
	myfb_send_data_sync( info, LOBYTE(x1) );//写x1低8位
	myfb_send_data_sync( info, HIBYTE(x2) );//写x2高8位
	myfb_send_data_sync( info, LOBYTE(x2) );//写x2低8位
	myfb_send_cmd_sync( info, 0x2B );		//Page Address Set
	myfb_send_data_sync( info, HIBYTE(y1) );
	myfb_send_data_sync( info, LOBYTE(y1) );
	myfb_send_data_sync( info, HIBYTE(y2) );
	myfb_send_data_sync( info, LOBYTE(y2) );
	myfb_send_cmd_sync( info, 0x2C );		//Memory Write
	
//	printk("x1:%d x2:%d y1:%d y2:%d addr:%p end:%p len:%d\n",x1,x2,y1,y2,data,data+len,len);
	time1 = jiffies;
	
	memset(&par->t, 0, sizeof par->t);//清空传输结构体所占的内存
	par->t.tx_buf = data;//传输缓冲指针指向data
	par->t.len = len;//长度是len
	
	
	spi_message_init(&par->m);
	spi_message_add_tail(&par->t, &par->m);
	par->m.complete = update_complete;//解锁
	par->m.context = info;
	//同步传输 将需要改变的行输给屏幕 不用估计宽度 因为显示缓冲只有x1~x2部分被修改
	//就算把x1=0 x2=320也没所谓
	spi_async_locked( par->spi, &par->m );
}

u16 u16_to_lcd_endian( struct fb_info *info, u16 data )
{
	struct myfb_par* par = info->par;
	if( !par->lcd_little_endian ) {
		return data << 8 | data >> 8;
	}
	return data;
}



//更新显示屏
//参数2 x,y起点 
//参数3 4 是 宽度和高度 参数5是图像数据
void update_lcd( struct fb_info *info, u16 x, u16 y, u16 w, u16 h, u8* src )
{
	u16 x1, y1, x2, y2;
	u8* buf;
	int len;
	struct myfb_par* par = info->par;//获取私有数据
	
	mutex_lock(&par->mlock);//上锁
	
	if( src ) {	//src图像数据指针不是执行NULL
		int i, j;
		u16* src16, *dst16;//两个U16指针
					//xres 在定义全局可变参数时已经赋值320
		if( x > myfb_var.xres - 1 ) return;//如果X>299 即x=320或>320退出
		if( x + w > myfb_var.xres ) w = myfb_var.xres - x;//宽越界时宽度取起点到右边框
		
		if( y > myfb_var.yres - 1 ) return;//高度越界
		if( y + h > myfb_var.yres ) h = myfb_var.yres - y;//取下边框
				//转换为U16指针 并取 X行 Y列的像素点地址到src16  X行Y列为其实绘图点
		src16 = &((u16*)src)[ y * myfb_var.xres + x ];
				//获取显示缓冲区 X行 Y列的像素点地址到dst16
		dst16 = &((u16*)info->screen_base)[ y * myfb_var.xres + x ];

		if( par->lcd_little_endian ) {	//lcd_little_endian 小端点
			for( i = 0; i <= h; i++ ) {	//复制 h行的像素到显示缓冲区
				memcpy(dst16, src16, w*2);//拷贝w框的像素内容 一个像素2B
				src16 += myfb_var.xres;//数据指针跳到下一行
				dst16 += myfb_var.xres;//显示缓冲指针跳到下一行
			}
		}
		else {
			u16 register tmp;
			for( i = 0; i <= h; i++ ) {
				for( j = 0; j < w; j++ ) {
					tmp = *src16++;//获取数据
					*dst16++ = tmp << 8 | tmp >> 8;//大端交换数据并写入显示缓冲
				}
				src16 += myfb_var.xres - w;
				dst16 += myfb_var.xres - w;
			}
		}
	}
	
	x1 = 0;//x1 = 0
	x2 = myfb_var.xres - 1;//x2 = 319 ？？？不是 x1+w?
	y1 = y;
	y2 = y + h - 1;//y+h-1
	//起点y乘以一行的字节数 得出第y1行首像素地址
	buf = (u8*)&info->screen_base[ y1 * myfb_fix.line_length ];
	len = h * myfb_var.xres * 2;//h*乘以一行的字节数 得出h行的字节数 len不是指针

// printk("%d, %d, %d, %d, %d\n", x1, y1, x2, y2, len);
	//再把参数传输给update_lcd_data函数
	//x1 x2 窗口的左线和右线 y1y2窗口的上线和下线
	update_lcd_data( info, x1, y1, x2, y2, buf, len );//buf起点所在行第一个点 len+buf是终点所在行第一个点
}



/**************************************************************************
*
* @rotate:
*   旋转参数的地址
*
* @description:
*	把旋转参数更新到硬件上。
*	
*
*/
int update_lcd_rotate(struct fb_info *info,unsigned long rotate)
{

	int val = *(int *)rotate;
	switch(val) {	
	case 0:
	    	myfb_send_cmd_sync(info, 0x36);
		myfb_send_data_sync(info, 0x60);
		break;
	case 90:
	    	myfb_send_cmd_sync(info, 0x36);
		myfb_send_data_sync(info, 0x00);	
		break;
	case 180:
	    	myfb_send_cmd_sync(info, 0x36);
		myfb_send_data_sync(info, 0xa0);
		break;
	case 270:
	    	myfb_send_cmd_sync(info, 0x36);
		myfb_send_data_sync(info, 0xc0);	
		break;	
	default:
		return -1;
		break;
	}

	if(val == 0 || val == 180) {
		myfb_var.xres = info->var.xres = 320;
		myfb_var.yres = info->var.yres = 240;
	}else {
		myfb_var.xres = info->var.xres = 240;
		myfb_var.yres = info->var.yres = 320;
	}

	return 0;
}

//#define TEST_PATTEN
#ifdef TEST_PATTEN
void test_patten_lcd( struct fb_info *info )
{
	int i, x, y, w, h, r;
	u16* buf, color;
	u16 colors[] = { u16_to_lcd_endian(info, 0xF800), u16_to_lcd_endian(info, 0x07E0) };
	
	w = myfb_var.xres;
	h = myfb_var.yres;
	
	for( i = 0; i < 10; i++ ) {
		buf = (u16*)info->screen_base;
		r = i % 2;
		for( y = 0; y < h; y++ ) {
			if( y%10 == 0) r = !r;
			color = colors[ r ];
			for( x = 0; x < w; x++ )
				*buf++ = color;
		}
		update_lcd( info, 0, 0, w, h, NULL );
	//	msleep(100);
	}
}
#endif

#define FBIPUT_UPDATE	_IOW('F', 10, int)
#define FBIPUT_UPDATE_LOCK	_IOW('F', 0x300, int)
#define FBIPUT_UPDATE_UNLOCK	_IOW('F', 0x301, int)
#define FBIPUT_UPDATE_ROTATE    _IOW('F', 0x302, int)  //更新旋转参数

static int myfb_ioctl(struct fb_info *info, unsigned int cmd,
			  unsigned long arg)
{
	switch (cmd) {
	case FBIPUT_UPDATE_LOCK: //UPDATE_LOCK上锁 未实现
		return 0;
	case FBIPUT_UPDATE_UNLOCK: //UPDATE_LOCK解锁 未实现
		return 0;
	case FBIPUT_UPDATE: //更新屏幕
	{
		struct update_info update_info; //从用户空间拷贝过来
		if (copy_from_user(&update_info, (char *)arg, sizeof(update_info)))
			return -EFAULT;//调用函数 update_lcd更新
 		update_lcd( info, update_info.x, update_info.y, update_info.w, update_info.h, (u8*)update_info.src );
		break;
	}
	case FBIPUT_UPDATE_ROTATE://更新旋转参数
		return update_lcd_rotate(info,arg);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}



/*
 * desc 
 * 多行描点时,得到一个页对齐并且是行首地址,
 * 单行描点可以是页对齐但不是行首.
 * @addr 
 *
 * return 0
*/
int find_start_addr(struct fb_info *info,u8 __iomem **addr,u32 *cnt)
{
	u8 *base = (u8 *)info->screen_base; /*基地址*/
	u32 line_length = info->fix.line_length; /*行字节数*/
	u32 offset = (u32)*addr - (u32)info->screen_base;
	u32 page_addr;
	u32 count = *cnt;
	
	/* 如果可以就1行 */
	if(offset/line_length == (offset + count)/line_length) {
		u32 line_start = line_length * (offset/line_length);
		page_addr = offset & ~(PAGE_SIZE - 1);
		if(page_addr>=line_start && page_addr <= offset) {
			*addr = base + page_addr; /* 页地址作为起始地址 */
			*cnt += offset - page_addr; /* 长度增加偏移 */
			return 0;
		}
	} 	

	page_addr = offset & ~(PAGE_SIZE - 1); 
	/* 多行  页对齐 && */
	while((page_addr % PAGE_SIZE != 0) || (page_addr % line_length != 0)) {
		page_addr -= PAGE_SIZE;
		if(page_addr <= 0)
			break; 
	}

	if(page_addr < 0) { 
		printk("page_addr --%d is fail \n",page_addr);
		return -1;
	}

	*addr = page_addr + base;
	*cnt  = offset - page_addr + *cnt;
	return 0;

}




ssize_t myfb_write(struct fb_info *info, const char __user *buf,
			size_t count, loff_t *ppos)
{
	unsigned long p, total_size;
	int err = 0;
	u8 __iomem *dst;
	int x1, y1, x2, y2;
	int byte , xres; /*行分辨率*/
	struct myfb_par* par = info->par;	

	mutex_lock(&par->mlock);

	p = *ppos;
	byte = info->var.bits_per_pixel/8;
	xres = info->var.xres;
	total_size = info->screen_size;

	if (total_size == 0)
		total_size = info->fix.smem_len;
	
	if (p > total_size) { //偏移大于总字节退出
		mutex_unlock(&par->mlock);
		printk("p > total_size .....\n");
		return -EFBIG;
	}		

	if (count > total_size) { //写的字节大于总字节
		err = -EFBIG;
		count = total_size;//写入计数改为为总字节
	}	
	
	if (count + p > total_size) {
		if (!err)
			err = -ENOSPC;
		count = total_size - p;//写入控制不能越界
	}
	
	dst = (u8 __iomem *) (info->screen_base + p);
	
	if(count) {
		u8 __iomem *raw_dst;
		U8 __iomem *end_addr; //结束的显存地址
		u32 send_len;// 实际发送的长度
		
		if (copy_from_user(dst, buf, count)) { //从用户空间拷贝
			printk("copy_from user is fail count =%d \n",count);
			err = -EFAULT;
			mutex_unlock(&par->mlock);
			return err;
		}

		*ppos += count ;

		send_len = count;
		raw_dst = dst;
		err = find_start_addr(info,&dst,&send_len); 
		if(err) {
			printk("err =%d line:%d\n",err,__LINE__); //查找出错
			mutex_unlock(&par->mlock);
			return 0;
		}
		end_addr = dst + send_len - byte;
		x1 = ((uint32_t)dst - (uint32_t)info->screen_base ) % (byte * xres) / byte;
		y1 = ((uint32_t)dst - (uint32_t)info->screen_base ) / (byte * xres); 		
		x2 = ((uint32_t)end_addr - (uint32_t)info->screen_base) % (byte * xres) / byte;
		y2 = ((uint32_t)end_addr - (uint32_t)info->screen_base) / (byte * xres); 
		if(y1!=y2) { 	//写入是多行
			send_len += (xres - 1 - x2) * byte;
			x2 = xres -1;	
		}
		if(y2 > info->var.yres) {  //因为显示缓存区长度是页对齐后面有写区域不显示,计算的y可能越界
			send_len -= (y2 - info->var.yres + 1) * (byte * xres);
			y2 = info->var.yres - 1;
		}
		update_lcd_data(info, x1, y1, x2, y2, dst, send_len);
		
	
	}else {
		//count = 0
		mutex_unlock(&par->mlock);
	}
	
	return count;
}


/*
参数1是应用传入来设置的可变参数
*/
static int myfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
        int ret = 0;
	if(var->xres!=320 && var->xres!=240) 
		return -EFAULT;
	if(var->yres!=320 && var->yres!=240)
		return -EFAULT;	
	if((var->rotate%90)!=0)	
		return -EFAULT;	

	info->var.xres = var->xres;
	info->var.yres = var->yres;
	info->var.rotate = var->rotate;

        return ret;
}


static int myfb_set_par(struct fb_info *info)
{
	int rotate = info->var.rotate;

	info->fix.line_length = info->var.xres * info->var.bits_per_pixel / 8;
	update_lcd_rotate(info,(unsigned long )&rotate);
	return 0;		
}


static struct fb_ops myfb_ops = {
	.owner		= THIS_MODULE,
	.fb_ioctl = myfb_ioctl,//ioctl
	.fb_write = myfb_write, //write
	.fb_check_var = myfb_check_var, //检查可变参数
	.fb_set_par  = myfb_set_par,
};

struct fb_info *fb_info;//全局指针

//static u64 spi_lcd_dmamask = DMA_BIT_MASK(32);//DMA位掩码

static int __devinit myfb_probe(struct spi_device *spi)
{
	struct fb_info *info;
	int retval = -ENOMEM;
	int videomemorysize;//显示内存大小
	struct myfb_par *par;//私有数据
	

	printk(KERN_EMERG"%s line= %d HZ=%d\n",__func__,__LINE__,HZ);	
	printk(KERN_EMERG"spi->max_speed_hz=%d\n",spi->max_speed_hz);	
	//动态分配帧缓冲结构 私有数据长度是sizeof(struct myfb_par)
	info = framebuffer_alloc(sizeof(struct myfb_par), NULL);
	if (!info)
		goto err;

	par = info->par;//指向私有数据
	par->gpio_reset_pin = GPIO_LCD_RST_PIN;//复位
	par->gpio_rs_pin = GPIO_LCD_RS_PIN;//片选
//	par->gpio_en_pin = GPIO_LCD_EN_PIN;//使能
	par->info = info;//FBI
	par->spi = spi;//SPI
	
	mutex_init(&par->mlock);
	//与SPI无关引脚
	myfb_setup_gpio_pin(info);//配置引脚 6818 	
	myfb_reset(info);//硬件复位
	
	myfb_init_lcd( info );//帧缓冲初始化 寄存器初始化
	
	myfb_fix.line_length = myfb_var.xres * myfb_var.bits_per_pixel / 8;//一行的字节数
	videomemorysize = myfb_var.yres * myfb_fix.line_length;//一帧字节数
	videomemorysize = roundup(videomemorysize, PAGE_SIZE);//向上对齐PAGE_SIZE 不足一页当一页
	par->vram_size = videomemorysize;//虚拟内存大小
	
	info->var = myfb_var;//可变参数
	info->fix = myfb_fix;//固定参数
	printk("&info->var =%p \n",&info->var);

	par->vram_virt = (void *)__get_free_pages(GFP_KERNEL | __GFP_COMP, get_order(videomemorysize));
	if (!par->vram_virt)
		goto err_fb_rel;

	//缓存起始的物理地址	
	info->fix.smem_start = dma_map_single(NULL, (void *)par->vram_virt, videomemorysize, DMA_TO_DEVICE);
	{
		unsigned long page;
		/*设置保留的页面，以便mmap可以工作。
		这是必要的，因为我们将重新映射正常的内存。
			*/ //PAGE_ALIGN宏获取页首地址
		for (page = (unsigned long)par->vram_virt; 
			page < PAGE_ALIGN((unsigned long)par->vram_virt + videomemorysize);
			page += PAGE_SIZE) {
			SetPageReserved(virt_to_page(page));
		}
	
	}
		
	info->fbops = &myfb_ops;//操作寄存器
	
	info->screen_base = (char *)par->vram_virt;//显示缓存基地址
	
	info->fix.smem_len = videomemorysize;	//frambuffer 物理内存长度
	info->flags = FBINFO_FLAG_DEFAULT;
	
	dev_set_drvdata(&spi->dev, info);
	
	retval = register_framebuffer(info);
	if (retval < 0)
		goto err_fb_rel;
	
	printk(KERN_INFO"info->screen_base =%p \n ",info->screen_base);
	printk(KERN_INFO
	       "fb%d: frame buffer, using %dK of video memory\n",
	       info->node, videomemorysize >> 10);
		   
	

//	mutex_lock(&par->mlock);
//	update_lcd_data(info, 0 , 0, myfb_var.xres, myfb_var.yres -1,info->screen_base, info->fix.smem_len);
//	mutex_unlock(&par->mlock);

	return 0;

err_fb_rel:
	framebuffer_release(info);
err:
	return retval;

}


static int __devexit myfb_remove(struct spi_device *spi)
{
	struct fb_info *info = dev_get_drvdata(&spi->dev);

	if (info) {
		unregister_framebuffer(info);
		#if 0
		dma_free_writecombine(&spi->dev,info->fix.smem_len,
				info->screen_base,info->fix.smem_start);
		#endif
		framebuffer_release(info);
	}
	return 0;
}

static struct spi_driver myfb_driver = {
	.driver = {
		.name	= "my_spilcd",//设备驱动的驱动名称
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe	= myfb_probe,
	.remove	= __devexit_p(myfb_remove),
};



//入口
static int __init myfb_init(void)
{
	return spi_register_driver(&myfb_driver);
}

//出口
static void __exit myfb_exit(void)
{
	spi_unregister_driver(&myfb_driver);
}


module_init(myfb_init);
module_exit(myfb_exit);

MODULE_DESCRIPTION("fbdev driver for Broadsheet controller");
MODULE_AUTHOR("Jaya Kumar");
MODULE_LICENSE("GPL");
