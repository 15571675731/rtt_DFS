#include "stm32f4xx.h"
#include "uart.h"
#include "rtthread.h"
#include "shell.h"
#include "drv_spi.h"

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

void My_SpiInit(void)
{
    /* 时钟初始化
    spi1对应的的gpio口:
        sck:    pb3
        miso:   pb4
        mosi:   pb5
        nss:    pb14
    spi1对应的aph2时钟频率为84MHZ
    */
    
    /* 一些初始化变量 */
    GPIO_InitTypeDef GPIO_InitStruct;
    SPI_InitTypeDef SPI_InitStruct;
    
    /* 时钟使能 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    
    /* IO复用设置 */
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1);
    
    
    
    /* SPI初始化 */
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; /* 分频系数设置为128 */
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStruct.SPI_CRCPolynomial = 0x03; /* crc检验，大于1即可 */
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
    SPI_Init(SPI1, &SPI_InitStruct);
    
    /* SPI使能 */
    SPI_Cmd(SPI1, ENABLE);
    
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_SetBits(GPIOB, GPIO_Pin_14); /* 上拉关闭片选 */
    
//    My_SpiSendAndRecvData(0xff); /* 好像有说再启动spi时要发送一个字节的数据 */
}


/* 使能CS */
void My_EnableCS(void)
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_14); 
}

/* 失能CS */
void My_UnableCS(void)
{
    GPIO_SetBits(GPIOB, GPIO_Pin_14);
}

u8 My_SpiSendAndRecvData(u8 data)
{
    /* 判断是否忙碌 */
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    
    SPI_I2S_SendData(SPI1, data);
    
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    
    return SPI_I2S_ReceiveData(SPI1);
}

extern rt_err_t Stm32_Spi_Send_Recv(SPI_TypeDef *SPIx, uint32_t timeout, const uint8_t *send_data, uint8_t *recv_data);
u16 My_DEV_ID(void)
{	  
    u16 tmp = 0;
	uint8_t send_data = 0x90;
	uint8_t recv_data = 0x90;
    
	My_EnableCS();				    
//	My_SpiSendAndRecvData(0x90);//发送读取ID命令	    
//	My_SpiSendAndRecvData(0x00);
//	My_SpiSendAndRecvData(0x00);
//	My_SpiSendAndRecvData(0x00);
//	tmp|=My_SpiSendAndRecvData(0xFF) << 8;  
//	tmp|=My_SpiSendAndRecvData(0xFF);	  
	
	Stm32_Spi_Send_Recv(SPI1, 0XFF, &send_data, NULL);
	send_data = 0;
	Stm32_Spi_Send_Recv(SPI1, 0XFF, &send_data, NULL);
	Stm32_Spi_Send_Recv(SPI1, 0XFF, &send_data, NULL);
	Stm32_Spi_Send_Recv(SPI1, 0XFF, &send_data, NULL);
	Stm32_Spi_Send_Recv(SPI1, 0XFF, NULL, &recv_data);
	rt_kprintf("id:0x%x ", recv_data);
	Stm32_Spi_Send_Recv(SPI1, 0XFF, NULL, &recv_data);
	rt_kprintf("0x%x\r\n", recv_data);
	
	My_UnableCS();			    
	return tmp;
}

void spi_test(void)
{
    rt_err_t ret = RT_EOK;
    uint8_t send_data[] = {0x90, 0, 0, 0};
    uint8_t recv_data[10] = {0};
    struct rt_spi_device * spi_dev;
    GPIO_InitTypeDef  GPIO_InitStructure;
    struct rt_spi_configuration cfg;
	
	uint16_t id = 0;
    
    cfg.data_width = 8;
    cfg.max_hz = 2;
    cfg.mode = RT_SPI_MODE_3 | RT_SPI_MASTER | RT_SPI_MSB ;
    cfg.reserved = 0;
    
//    My_SpiInit();
//	
//	id = My_DEV_ID();
//	rt_kprintf("id:0x%x\r\n", id);
    
    
    
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
            rt_spi_send_then_recv(spi_dev, &send_data, 4, &recv_data, 2);
            rt_kprintf("use rt_spi_send_then_recv() read w25q ID is:0x%x 0x%x\n", recv_data[0], recv_data[1]);
//			send_data = 0;
//			rt_spi_send(spi_dev, &send_data, 1);
//            rt_kprintf("use rt_spi_send_then_recv() read w25q ID is:0x%x\n", recv_data[0]);
//			rt_spi_send(spi_dev, &send_data, 1);
//            rt_kprintf("use rt_spi_send_then_recv() read w25q ID is:0x%x\n", recv_data[0]);
//			rt_spi_send(spi_dev, &send_data, 1);
//            rt_kprintf("use rt_spi_send_then_recv() read w25q ID is:0x%x\n", recv_data[0]);
//			rt_spi_recv(spi_dev, recv_data, 1);
//            rt_kprintf("use rt_spi_send_then_recv() read w25q ID is:0x%x\n", recv_data[0]);
//			rt_spi_recv(spi_dev, recv_data, 1);
//            rt_kprintf("use rt_spi_send_then_recv() read w25q ID is:0x%x\n", recv_data[0]);
//			id = My_DEV_ID();
        }
    }
}


void raw_test()
{
	uint16_t id = 0;
    
    My_SpiInit();
	
	id = My_DEV_ID();
	rt_kprintf("id:0x%x\r\n", id);
}


int main(void)
{
    rt_thread_t th1;
    LED_Init();
    
    spi_test();
//	raw_test();
    
//    th1 = rt_thread_create("led_task", led_thread, NULL, LED_TASK_STACK_SIZE, RT_THREAD_PRIORITY_MAX-5, 10);
//    rt_thread_startup(th1);
    
    while(1)
    {
        rt_thread_delay(1000);
    }
}




