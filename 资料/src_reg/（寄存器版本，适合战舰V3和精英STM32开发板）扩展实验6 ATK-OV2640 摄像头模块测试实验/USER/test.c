#include "sys.h"
#include "delay.h"
#include "usart.h" 
#include "led.h" 		 	 
#include "lcd.h"  
#include "key.h"     
#include "usmart.h" 
#include "sram.h"
#include "malloc.h"
#include "sdio_sdcard.h"  
#include "w25qxx.h"    
#include "ff.h"  
#include "exfuns.h"   
#include "text.h"
#include "piclib.h"	
#include "string.h"		
#include "beep.h" 
#include "timer.h" 
#include "ov2640.h"
#include "dma.h"
//ALIENTEK STM32F103开发板 扩展实验6
//OV2640照相机 实验 
//技术支持：www.openedv.com
//广州市星翼电子科技有限公司

#define OV2640_JPEG_WIDTH	1024	//JPEG拍照的宽度	
#define OV2640_JPEG_HEIGHT	768		//JPEG拍照的高度

u8* ov2640_framebuf;				//帧缓存
extern u8 ov_frame;					//在timer.c里面定义	   
 
//文件名自增（避免覆盖）
//mode:0,创建.bmp文件;1,创建.jpg文件.
//bmp组合成:形如"0:PHOTO/PIC13141.bmp"的文件名
//jpg组合成:形如"0:PHOTO/PIC13141.jpg"的文件名
void camera_new_pathname(u8 *pname,u8 mode)
{	 
	u8 res;					 
	u16 index=0;
	while(index<0XFFFF)
	{
		if(mode==0)sprintf((char*)pname,"0:PHOTO/PIC%05d.bmp",index);
		else sprintf((char*)pname,"0:PHOTO/PIC%05d.jpg",index);
		res=f_open(ftemp,(const TCHAR*)pname,FA_READ);//尝试打开这个文件
		if(res==FR_NO_FILE)break;		//该文件名不存在=正是我们需要的.
		index++;
	}
}
//OV2640速度控制
//根据LCD分辨率的不同，设置不同的参数
void ov2640_speed_ctrl(void)
{
	u8 clkdiv,pclkdiv;			//时钟分频系数和PCLK分频系数
	if(lcddev.width==240)		//2.8寸LCD
	{
		clkdiv=1;
		pclkdiv=28;
	}else if(lcddev.width==320)	//3.5寸LCD
	{
		clkdiv=3;
		pclkdiv=15;
	}else						//4.3/7寸LCD
	{
		clkdiv=15;
		pclkdiv=4;
	} 
	SCCB_WR_Reg(0XFF,0X00);		
	SCCB_WR_Reg(0XD3,pclkdiv);	//设置PCLK分频
	SCCB_WR_Reg(0XFF,0X01);
	SCCB_WR_Reg(0X11,clkdiv);	//设置CLK分频	
}
//OV2640拍照jpg图片
//pname:要保存的jpg照片路径+名字
//返回值:0,成功
//    其他,错误代码
u8 ov2640_jpg_photo(u8 *pname)
{
	FIL* f_jpg;
	u8 res=0;
	u32 bwr;
	u32 i=0;
	u32 jpeglen=0;
	u8* pbuf;
	
	f_jpg=(FIL *)mymalloc(SRAMIN,sizeof(FIL));	//开辟FIL字节的内存区域 
	if(f_jpg==NULL)return 0XFF;					//内存申请失败.
	OV2640_JPEG_Mode();							//切换为JPEG模式 
  	OV2640_OutSize_Set(OV2640_JPEG_WIDTH,OV2640_JPEG_HEIGHT); 
	SCCB_WR_Reg(0XFF,0X00);
	SCCB_WR_Reg(0XD3,30);
	SCCB_WR_Reg(0XFF,0X01);
	SCCB_WR_Reg(0X11,0X1); 
	for(i=0;i<10;i++)		//丢弃10帧，等待OV2640自动调节好（曝光白平衡之类的）
	{
		while(OV2640_VSYNC==1);	 
		while(OV2640_VSYNC==0);	  
	}  
	while(OV2640_VSYNC==1)	//开始采集jpeg数据
	{
		while(OV2640_HREF)
		{  
			while(OV2640_PCLK==0); 
			ov2640_framebuf[jpeglen]=OV2640_DATA;
			while(OV2640_PCLK==1);  
			jpeglen++;
		} 
	}		
	res=f_open(f_jpg,(const TCHAR*)pname,FA_WRITE|FA_CREATE_NEW);//模式0,或者尝试打开失败,则创建新文件	 
	if(res==0)
	{
		printf("jpeg data size:%d\r\n",jpeglen);	//串口打印JPEG文件大小
		pbuf=(u8*)ov2640_framebuf;
		for(i=0;i<jpeglen;i++)//查找0XFF,0XD8
		{
			if((pbuf[i]==0XFF)&&(pbuf[i+1]==0XD8))break;
		}
		if(i==jpeglen)res=0XFD;//没找到0XFF,0XD8
		else		//找到了
		{
			pbuf+=i;//偏移到0XFF,0XD8处
			res=f_write(f_jpg,pbuf,jpeglen-i,&bwr);
			if(bwr!=(jpeglen-i))res=0XFE; 
		}
	} 
	f_close(f_jpg);  
	OV2640_RGB565_Mode();	//RGB565模式 
	myfree(SRAMIN,f_jpg); 
	return res;
}  
int main(void)
{		
	u8 res;							 
	u8 *pname;					//带路径的文件名 
	u8 key;						//键值		   
	u8 sd_ok=1;					//0,sd卡不正常;1,SD卡正常.
 	u16 pixcnt=0;				//像素统计
	u16 linecnt=0;				//行数统计
 	Stm32_Clock_Init(9);		//系统时钟设置
	uart_init(72,115200);		//串口初始化为115200
	delay_init(72);	   	 		//延时初始化  
 	usmart_dev.init(72);		//初始化USMART		
 	LED_Init();		  			//初始化与LED连接的硬件接口
	KEY_Init();					//初始化按键
	LCD_Init();			   		//初始化LCD  
	BEEP_Init();        		//蜂鸣器初始化	 
	W25QXX_Init();				//初始化W25Q128
 	my_mem_init(SRAMIN);		//初始化内部内存池
	exfuns_init();				//为fatfs相关变量申请内存  
 	f_mount(fs[0],"0:",1); 		//挂载SD卡 
 	f_mount(fs[1],"1:",1); 		//挂载FLASH. 
	POINT_COLOR=RED;      
 	while(font_init()) 			//检查字库
	{	    
		LCD_ShowString(30,50,200,16,16,"Font Error!");
		delay_ms(200);				  
		LCD_Fill(30,50,240,66,WHITE);//清除显示	     
	}  	 
 	Show_Str(30,50,200,16,"STM32F103 开发板",16,0);				    	 
	Show_Str(30,70,200,16,"OV2640照相机实验",16,0);			    	 
	Show_Str(30,90,200,16,"KEY0:拍照(bmp格式)",16,0);			    	 
	Show_Str(30,110,200,16,"KEY1:拍照(jpg格式)",16,0);					    	 
	Show_Str(30,130,200,16,"2015年4月16日",16,0);
	res=f_mkdir("0:/PHOTO");		//创建PHOTO文件夹
	if(res!=FR_EXIST&&res!=FR_OK) 	//发生了错误
	{		    
		Show_Str(30,150,240,16,"SD卡错误,无法拍照!",16,0); 	 		  	     
		sd_ok=0;  	
	} 
	ov2640_framebuf=mymalloc(SRAMIN,52*1024);//申请帧缓存
 	pname=mymalloc(SRAMIN,30);		//为带路径的文件名分配30个字节的内存		    
 	while(!pname||!ov2640_framebuf)	//内存分配出错
 	{	    
		Show_Str(30,150,240,16,"内存分配失败!",16,0);
		delay_ms(200);				  
		LCD_Fill(30,150,240,146,WHITE);//清除显示	     
		delay_ms(200);				  
	}   											  
	while(OV2640_Init())			//初始化OV2640
	{
		Show_Str(30,150,240,16,"OV2640 错误!",16,0);
		delay_ms(200);
	    LCD_Fill(30,150,239,206,WHITE);
		delay_ms(200);
	}
 	Show_Str(30,170,200,16,"OV2640 正常",16,0);
	delay_ms(1500);	 		 
	//TIM6_Int_Init(10000,7199);	//10Khz计数频率,1秒钟中断,屏蔽则不打印帧率									  
	OV2640_RGB565_Mode();			//RGB565模式
  	OV2640_OutSize_Set(lcddev.width,lcddev.height); 
	ov2640_speed_ctrl();
	MYDMA_SRAMLCD_Init((u32)ov2640_framebuf);	 
 	while(1)
	{
		while(OV2640_VSYNC)			//等待帧信号
		{
			key=KEY_Scan(0);		//不支持连按
			if(key==KEY0_PRES||key==KEY1_PRES)
			{ 	
				LED1=0;				//DS1亮
				if(sd_ok)			//SD卡正常才可以拍照 
				{ 
					if(key==KEY0_PRES)	//BMP拍照
					{
						camera_new_pathname(pname,0);			//得到文件名
						res=bmp_encode(pname,0,0,lcddev.width,lcddev.height,0);
					}else if(key==KEY1_PRES)					//JPG拍照
					{
						camera_new_pathname(pname,1);			//得到文件名	
 						res=ov2640_jpg_photo(pname); 
						OV2640_OutSize_Set(lcddev.width,lcddev.height); 	
						ov2640_speed_ctrl(); 
					}
 					if(res)//拍照有误
					{
						Show_Str(30,130,240,16,"写入文件错误!",16,0);		 
					}else 
					{
						Show_Str(30,130,240,16,"拍照成功!",16,0);
						Show_Str(30,150,240,16,"保存为:",16,0);
						Show_Str(30+42,150,240,16,pname,16,0);		    
						BEEP=1;	//蜂鸣器短叫，提示拍照完成
						delay_ms(100);
					} 
				}else 					//提示SD卡错误
				{					    
					Show_Str(40,130,240,12,"SD卡错误!",12,0);
					Show_Str(40,150,240,12,"拍照功能不可用!",12,0);			    
				}
				LED1=1;				//DS1灭
				BEEP=0;				//关闭蜂鸣器 
				delay_ms(1800);
			}  
		} 
		LCD_SetCursor(0,0);			//设置坐标
		LCD_WriteRAM_Prepare();		//开始写入GRAM	 
 		linecnt=0;					//行统计清零
		pixcnt=0;					//像素计数器清零
		while(linecnt<lcddev.height)	
		{
			while(OV2640_HREF)
			{  
				while(OV2640_PCLK==0);
				ov2640_framebuf[pixcnt++]=OV2640_DATA;   
				while(OV2640_PCLK==1); 
				while(OV2640_PCLK==0); 
				ov2640_framebuf[pixcnt++]=OV2640_DATA; 
				while(OV2640_PCLK==1);		
			}  
			if(pixcnt)
			{
				MYDMA_SRAMLCD_Enable();	//启动DMA数据传输
				pixcnt=0;
				linecnt++;
			}
		} 
		ov_frame++;
		LED0=!LED0;		
	} 	   										    
}










