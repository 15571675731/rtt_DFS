#include "drv_spi.h"




/*--------------------------------------------------------------------------------*/
/* �����ϵͳ��ע��SPI�����豸 */
#define _DEBUG_

#define SPI_TIMEOUT     0XFF
#define DUMMY           0X00

static struct stm32_spi spi_bus_obj; //SPI���߶���

//stm32��SPI�������������
struct stm32_spi_config SPI_DEF_CONFIG = {
    .Instance = SPI1,
    .bus_name = "spi1",
    .irq_type = SPI1_IRQn
};

/*
* �Լ��ٴη�װ��SPI���亯������ΪSPI���ж�ǰ������д��д������������
* return : success->RT_EOK   error->RT_ERROR
*/
rt_err_t Stm32_Spi_Send_Recv(SPI_TypeDef *SPIx, uint32_t timeout, const uint8_t *send_data, uint8_t *recv_data)
{
    uint32_t ret = timeout;
	uint16_t recv = 0;
    
    
    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET && timeout) // 90 00 00 00 00 00
        timeout--;
    if(send_data && timeout)
        SPI_I2S_SendData(SPIx, *send_data);
    else if(timeout)
        SPI_I2S_SendData(SPIx, DUMMY);
    
    if(!timeout)
	{
		rt_kprintf("spi send timeout\r\n");
        return RT_ERROR;
	}
    
    timeout = ret;
    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET && timeout)
        timeout--;
    if(recv_data && timeout)
	{
		recv = SPI_I2S_ReceiveData(SPIx);
		*recv_data = recv;
	}
    else if(timeout)
        recv = SPI_I2S_ReceiveData(SPIx);
    
    if(!timeout)
	{
		rt_kprintf("spi recv timeout\r\n");
        return RT_ERROR;
	}
    
    return RT_EOK; //90 00 00 00  00(ef) 00(17)
}


void stm32_spi1_gpio_cfg(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    //ʱ��ʹ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOG, &GPIO_InitStructure);//��ʼ��
    
    GPIO_SetBits(GPIOG,GPIO_Pin_7);//�ر�NRFƬѡ
    
    //PB3->SCK PB4->MISO PB5->MOSI
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1);
    
    
}

/*
* stm32��SPI��ʼ������
* ������config����SPI
* ����ʼ��SPI
* return: �ɹ�->RT_EOK        ʧ��->����
*/
rt_err_t stm32_spi_init(struct stm32_spi *spi_drv, struct rt_spi_configuration *configuration)
{
    rt_err_t ret = RT_EOK;
    
    SPI_TypeDef *SPIx;
    SPI_InitTypeDef SPI_InitStruct;
    
    RT_ASSERT(spi_drv != RT_NULL);
    RT_ASSERT(configuration != RT_NULL);
    
    stm32_spi1_gpio_cfg();
    
    SPIx = spi_drv->stm32_spi_config->Instance;
//    
//    if(configuration->mode & RT_SPI_CPHA) // RT_SPI_CPHA = 1
        SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
//    else // RT_SPI_CPHA = 0
//        SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
//    
//    if(configuration->mode & RT_SPI_CPOL) // RT_SPI_CPOL = 1
        SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
//    else // RT_SPI_CPOL = 0
//        SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
//    
//    if(configuration->mode & RT_SPI_MSB)
        SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
//    else
//        SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_LSB;
    
//    if(configuration->mode & RT_SPI_SLAVE)
//        SPI_InitStruct.SPI_Mode = SPI_Mode_Slave;
//    else
        SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
    
    SPI_InitStruct.SPI_CRCPolynomial = 0x03;
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BR;
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    
//    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128; /* ��Ƶϵ������Ϊ128 */
//    SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
//    SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
//    SPI_InitStruct.SPI_CRCPolynomial = 0x03; /* crc���飬����1���� */
//    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
//    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
//    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
//    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
    spi_drv->SPI_InitStruct = SPI_InitStruct;
    SPI_Init(SPIx, &SPI_InitStruct);
    
    SPI_Cmd(SPI1, ENABLE);
    
#ifdef _DEBUG_
    rt_kprintf("spi stm32_spi_init success\r\n");
#endif
    
    return ret;
}



/*stm32_configure��������configuration��������SPI����
* �����ݿ�ȡ�ʱ�Ӽ��ԡ��������ʵȣ�������SPI��ʼ����
* return: �ɹ�->RT_EOK        ʧ��->����
*/
rt_err_t stm32_configure(struct rt_spi_device *device, struct rt_spi_configuration *configuration)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(configuration != RT_NULL);

    struct stm32_spi *spi_drv =  rt_container_of(device->bus, struct stm32_spi, spi_bus);
    spi_drv->cfg = configuration;
#ifdef _DEBUG_
    rt_kprintf("spi stm32_configure success\r\n");
#endif
    return stm32_spi_init(spi_drv, configuration);
}

extern u8 My_SpiSendAndRecvData(u8 data);
//extern u16 My_DEV_ID(void);
//rt_uint32_t stm32_xfer(struct rt_spi_device *device, struct rt_spi_message *message)
//{
//	return My_DEV_ID();
//}
#if 1
/*
* ��������xfer�������ǽ������͵����ݷ��͸����豸�Ľӿ�
*/
rt_uint32_t stm32_xfer(struct rt_spi_device *device, struct rt_spi_message *message)
{
    rt_err_t ret = RT_ERROR;
    SPI_TypeDef *SPIx;
    struct stm32_spi *spi_drv = NULL;
    struct stm32_spi_io *ns_io;
    rt_size_t already_send_len = 0;
	uint8_t _send = NULL;
    
    const uint8_t *send_buf = NULL;
    uint8_t *recv_buf = NULL;
    
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);
    RT_ASSERT(device->bus->parent.user_data != RT_NULL);
    RT_ASSERT(message != RT_NULL);
    
    send_buf = message->send_buf;
    recv_buf = message->recv_buf;
    
    //����Ƭѡ
    spi_drv =  rt_container_of(device->bus, struct stm32_spi, spi_bus);
    SPIx = spi_drv->stm32_spi_config->Instance;
    
    ns_io = device->parent.user_data;
    if(message->cs_take)
        GPIO_ResetBits(ns_io->GPIOx, ns_io->GPIO_Pin);
    
    //ѭ����/д����,��д���������� message->length ������ ����RTT�Ĺٷ����������� message->length ���ֵӦ�ò��ø��£�������ɷ��ؼ���
    
    while( already_send_len < message->length)
    {
        //�����ȫ˫��
        if(send_buf && recv_buf) //�շ�������
        {
            ret = Stm32_Spi_Send_Recv(SPIx, SPI_TIMEOUT, send_buf+already_send_len, recv_buf+already_send_len);//һ��ֻ�ܴ���һ���ֽ�(û��DMA�������)
			
        }
        else if(send_buf) //ֻ��
        {
            ret = Stm32_Spi_Send_Recv(SPIx, SPI_TIMEOUT, send_buf+already_send_len, NULL);//һ��ֻ�ܴ���һ���ֽ�(û��DMA�������)
//			_send = *(send_buf+already_send_len);
//			SPI_I2S_SendData(SPIx, _send);
        }
        else //ֻ��
        {
            ret = Stm32_Spi_Send_Recv(SPIx, SPI_TIMEOUT, NULL, recv_buf+already_send_len);//һ��ֻ�ܴ���һ���ֽ�(û��DMA�������)
//			*(recv_buf+already_send_len) = SPI_I2S_ReceiveData(SPIx);
        }
        
        already_send_len++;
        
        if(ret !=RT_EOK)
        {
            GPIO_SetBits(ns_io->GPIOx, ns_io->GPIO_Pin);
        #ifdef _DEBUG_
            rt_kprintf("spi stm32_xfer error\r\n");
        #endif
            return 0;;
        }
    }
    
    
    //����Ƭѡ
    if(message->cs_release)
        GPIO_SetBits(ns_io->GPIOx, ns_io->GPIO_Pin);
    
#ifdef _DEBUG_
    rt_kprintf("spi stm32_xfer success\r\n");
#endif
    
    return already_send_len;
}

#endif

static struct rt_spi_ops stm32_spi_ops= {
    .configure = stm32_configure,
    .xfer = stm32_xfer
};

/*
* ע��SPI���ߵ�����ϵͳ
* ��Ҫ�ǳ�ʼ�� spi_bus_obj ���󣬲�����ע�ᵽ����ϵͳ��(ʹ�� rt_spi_bus_register)
*/
static int rt_hw_spi_bus_init(void)
{
    rt_err_t ret = RT_EOK;
    
    spi_bus_obj.stm32_spi_config = &SPI_DEF_CONFIG;
    
    spi_bus_obj.cfg->data_width = 8;
    spi_bus_obj.cfg->max_hz = 2;
    spi_bus_obj.cfg->mode = RT_SPI_MODE_3 | RT_SPI_MASTER | RT_SPI_MSB ;
    spi_bus_obj.cfg->reserved = 0;
    
    spi_bus_obj.spi_dma_flag = UNUSE_SPI_DMA;
    
    spi_bus_obj.spi_bus.parent.user_data = &SPI_DEF_CONFIG;
//	spi_bus_obj.spi_bus.parent.user_data = NULL;
    

    ret = rt_spi_bus_register(&spi_bus_obj.spi_bus,
                             spi_bus_obj.stm32_spi_config->bus_name,
                             &stm32_spi_ops);
    RT_ASSERT(ret == RT_EOK);
    
    
#ifdef _DEBUG_
    rt_kprintf("spi rt_hw_spi_bus_init success\r\n");
#endif

    return ret;
}


int rt_hw_spi_init(void)
{
//    stm32_get_dma_info();
    
#ifdef _DEBUG_
    rt_kprintf("spi rt_hw_spi_init success\r\n");
#endif
    
    return rt_hw_spi_bus_init();
}
INIT_BOARD_EXPORT(rt_hw_spi_init);


/* ���SPI���ߵ�ע�� */
/*--------------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------------*/
/* 
* ���SPI�豸 
*Ϊ�˷���Ӧ�ÿ�����ʹ��SPI�����豸��ܣ�һ�������㶼���ṩһ�����ڹ���SPI���豸�ĺ�
����Ӧ�ÿ�����ʹ�á�����SPI���豸�ĺ��������������������Զ���ģ��ο�����Ϊxxx_spi_device_attach��
������xxx_spi_device_attach�����CS���ŵĳ�ʼ����Ȼ����һ��SPI���豸��struct rt_spi_device����
������SPI�豸��������е� rt_spi_bus_attach_device �����豸���ص���Ӧ��SPI�����豸�ϡ�
*/

rt_err_t rt_hw_spi_device_attach(const char *bus_name, 
                                const char *device_name, 
                                GPIO_TypeDef *cs_gpiox, 
                                uint16_t cs_gpio_pin)
{
    rt_err_t ret = RT_EOK;
    struct rt_spi_device *spi_device;
    struct stm32_spi_io *cs_pin;
    
    //��ʼ�����CS��
    GPIO_InitTypeDef  GPIO_InitStructure;

    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//��֪��ʹ���Ǹ�ʱ�ӣ������ڰ弶��ʼ����ʱ��Ͱ����е�IOʱ��ȫ��ʹ������

    //GPIOF9,F10��ʼ������
    GPIO_InitStructure.GPIO_Pin = cs_gpio_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(cs_gpiox, &GPIO_InitStructure);//��ʼ��
        
    GPIO_SetBits(cs_gpiox,cs_gpio_pin);//GPIOF9,F10���øߣ�����
    
    //����rt_spi_bus_attach_device���豸���ص�SPI����
    spi_device = (struct rt_spi_device *)rt_malloc(sizeof(struct rt_spi_device));
    RT_ASSERT(spi_device != RT_NULL);
    cs_pin = (struct stm32_spi_io *)rt_malloc(sizeof(struct stm32_spi_io));
    RT_ASSERT(cs_pin != RT_NULL);
    
    cs_pin->GPIOx = cs_gpiox;
    cs_pin->GPIO_Pin = cs_gpio_pin;
    
    ret = rt_spi_bus_attach_device(spi_device, device_name, bus_name, (void *)cs_pin);
    
    rt_kprintf("%s attach to %s done\r\n", device_name, bus_name);
    
    return ret;
}



















/*--------------------------------------------------------------------------------*/







