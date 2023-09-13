#include "stm32f4xx.h"
#include "uart.h"
#include "rtthread.h"
#include "shell.h"
#include "spi_test.h"

#define LED_TASK_STACK_SIZE     256 //ÿ������164��160�պõ��߳�ջʹ��80%�����ߣ�

//LED IO��ʼ��
void LED_Init(void)
{    	 
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//ʹ��GPIOFʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOFʱ��

    //GPIOF9,F10��ʼ������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOF, &GPIO_InitStructure);//��ʼ��
        
    GPIO_SetBits(GPIOF,GPIO_Pin_9 | GPIO_Pin_10);//GPIOF9,F10���øߣ�����
	
	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_14); /* �����ر�Ƭѡ */
}
void led_thread(void *arg)
{
    while(1)
    {
        GPIO_ResetBits(GPIOF,GPIO_Pin_9 );//GPIOF9,F10���øߣ�����
        rt_thread_mdelay(500);
        GPIO_SetBits(GPIOF,GPIO_Pin_9);//GPIOF9,F10���øߣ�����
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




