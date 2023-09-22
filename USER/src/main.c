#include "stm32f4xx.h"
#include "uart.h"
#include "rtthread.h"
#include "shell.h"
#include "spi_test.h"


#include "ff.h"
#include "diskio.h"

#include "sfud.h"
#include "spi_flash_sfud.h"

#define LED_TASK_STACK_SIZE     256 //每次至少164（160刚好到线程栈使用80%的上线）

//LED IO初始化
void LED_Init(void)
{    	 
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//使能GPIOF时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOF时钟

    //GPIOF9,F10初始化设置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOF, &GPIO_InitStructure);//初始化
        
    GPIO_SetBits(GPIOF,GPIO_Pin_9 | GPIO_Pin_10);//GPIOF9,F10设置高，灯灭
	
	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_14); /* 上拉关闭片选 */
}
void led_thread(void *arg)
{
    while(1)
    {
        GPIO_ResetBits(GPIOF,GPIO_Pin_9 );//GPIOF9,F10设置高，灯灭
        rt_thread_mdelay(500);
        GPIO_SetBits(GPIOF,GPIO_Pin_9);//GPIOF9,F10设置高，灯灭
        rt_thread_mdelay(500);
    }
}

FATFS fsobject;   //一定是一个全局变量
BYTE work[FF_MAX_SS]; //一定是一个全局变量

FIL fd;

char write_buf[15] = "hello world\r\n";
char read_buf[15] = {0};

void fatfs_test()
{
    int i = 0;
    uint32_t write_size = 0;
    uint32_t read_size = 0;
	FRESULT res;
	
    res = f_mkfs("0:", 0, work, sizeof(work));
    
	res = f_mount(&fsobject, "0:", 1);
	
	if(res != FR_OK)
	{
		rt_kprintf("res == FR_NO_FILESYSTEM error error code:%d\r\n", res);
		res = f_mkfs("0:", 0, work, sizeof(work));
		
		res = f_mount(NULL, "0:", 1);
		
		res = f_mount(&fsobject, "0:", 1);
	}
	else
		rt_kprintf("res == FR_NO_FILESYSTEM success\r\n");
	
	
	res = f_open(&fd , "0:abcd" , FA_OPEN_ALWAYS|FA_READ |FA_WRITE );
	if(res != FR_OK)
		rt_kprintf("open file error");
	else
		rt_kprintf("open file success\r\n");
    
    f_write(&fd, write_buf, 15, &write_size);
    if(write_size == 0)
        rt_kprintf("f_write error\r\n");
    else
        rt_kprintf("write :%d\r\n", write_size);
    
    f_lseek(&fd,0);
    f_read(&fd, read_buf, 15, &read_size);
    
    if(read_size != 0)
    {
        rt_kprintf("read data:%s\r\n", read_buf);
        rt_kprintf("readsize:%d\r\n", read_size);
        for(i=0; i<15; i++)
            rt_kprintf("0x%x ", read_buf[i]);
    }
    else
        rt_kprintf("read error\r\n");
}


void sfud_test()
{
	sfud_err err;
	const sfud_flash *flash = NULL;
	flash = sfud_get_device_table();
	if(flash == NULL)
		rt_kprintf("flash error\r\n");
	else
		rt_kprintf("flash success\r\n");
	sfud_erase(flash, 0, 1024);
}





int main(void)
{
	int i = 0;
	BYTE buf[100] = {0};
//    rt_thread_t th1;
    LED_Init();
    
    spi_test();
	
//	sfud__();
	fatfs_test();
//	sfud_test();
//	disk_read(0, buf, 0, 100);

//    for(i=0; i<100; i++)
//		if(buf[i] != 0 && buf[i] != 0xff)
//			rt_kprintf("0x%x ", buf[i]);
//		rt_kprintf("\r\n");
//	
	
	
	rt_kprintf("test success\r\n");
    while(1)
    {
        rt_thread_delay(1000);
    }
}




