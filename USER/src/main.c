#include "stm32f4xx.h"
#include "uart.h"
#include "rtthread.h"
#include "shell.h"
#include "spi_test.h"

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



int main(void)
{
//    rt_thread_t th1;
    LED_Init();
    
    spi_test();
    
    while(1)
    {
        rt_thread_delay(1000);
    }
}




