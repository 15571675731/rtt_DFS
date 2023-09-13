#include "stm32f4xx.h"
#include "uart.h"
#include "rtthread.h"
#include "shell.h"
#include "drv_spi.h"

#define LED_TASK_STACK_SIZE     256 //ÿ������164��160�պõ��߳�ջʹ��80%�����ߣ�

//LED IO��ʼ��
void LED_Init(void)
{    	 
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//ʹ��GPIOFʱ��

    //GPIOF9,F10��ʼ������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOF, &GPIO_InitStructure);//��ʼ��
        
    GPIO_SetBits(GPIOF,GPIO_Pin_9 | GPIO_Pin_10);//GPIOF9,F10���øߣ�����

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

void My_SpiInit(void)
{
    /* ʱ�ӳ�ʼ��
    spi1��Ӧ�ĵ�gpio��:
        sck:    pb3
        miso:   pb4
        mosi:   pb5
        nss:    pb14
    spi1��Ӧ��aph2ʱ��Ƶ��Ϊ84MHZ
    */
    
    /* һЩ��ʼ������ */
    GPIO_InitTypeDef GPIO_InitStruct;
    SPI_InitTypeDef SPI_InitStruct;
    
    /* ʱ��ʹ�� */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    
    /* IO�������� */
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1);
    
    
    
    /* SPI��ʼ�� */
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128; /* ��Ƶϵ������Ϊ128 */
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStruct.SPI_CRCPolynomial = 0x03; /* crc���飬����1���� */
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
    SPI_Init(SPI1, &SPI_InitStruct);
    
    /* SPIʹ�� */
    SPI_Cmd(SPI1, ENABLE);
    
//    My_SpiSendAndRecvData(0xff); /* ������˵������spiʱҪ����һ���ֽڵ����� */
}




void spi_test(void)
{
    rt_err_t ret = RT_EOK;
    uint8_t send_data = 0x90;
    uint8_t recv_data[10] = {0};
    struct rt_spi_device * spi_dev;
    GPIO_InitTypeDef  GPIO_InitStructure;
    struct rt_spi_configuration cfg;
    
    cfg.data_width = 8;
    cfg.max_hz = 2;
    cfg.mode = RT_SPI_MODE_3 | RT_SPI_MASTER | RT_SPI_MSB ;
    cfg.reserved = 0;
    
    My_SpiInit();
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    
    
    
    rt_hw_spi_device_attach("spi1", "spi10", GPIOB, GPIO_Pin_14);

//    if (RT_NULL == rt_sfud_flash_probe("W25Q128", "spi10"))
//    {
//        return -RT_ERROR;
//    }
    
    spi_dev = (struct rt_spi_device *)rt_device_find("spi10");
    
    
//    while(spi_dev)
    {
        
        rt_spi_configure(spi_dev, &cfg);
//        ret = rt_device_open((rt_device_t )spi_dev, NULL);
        if(ret == RT_EOK)
        {
            rt_spi_send_then_recv(spi_dev, &send_data, 1, recv_data, 5);
            rt_kprintf("use rt_spi_send_then_recv() read w25q ID is:%x%x\n", recv_data[3], recv_data[4]);
        }
    }
}



int main(void)
{
    rt_thread_t th1;
    LED_Init();
    
    spi_test();
    
//    th1 = rt_thread_create("led_task", led_thread, NULL, LED_TASK_STACK_SIZE, RT_THREAD_PRIORITY_MAX-5, 10);
//    rt_thread_startup(th1);
    
    while(1)
    {
        rt_thread_delay(1000);
    }
}




