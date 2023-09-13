#include "uart.h"




typedef struct Bsp_Uart
{
    /* �̳�rtt�Ĵ������ԣ��øó�Ա���������Ӧ�ò�ҹ�(ͨ���ó�Ա������ҵ�struct BSP_Uart����ʼ��ַ��
     * Ȼ��ͨ����ʼ��ַ������STM32��ר�������豸���ƿ飬��ͨ���ÿ��ƿ�����ʼ��STM32�Ĵ����豸) */
    struct rt_serial_device device;

    USART_InitTypeDef uart_init;//STM32��ר���������ԣ����ڳ�ʼ��STM32��Ƭ���Ĵ���
    USART_TypeDef * huart;
    IRQn_Type irqn; //��¼�жϺ�

    rt_uint8_t dma_flag;//DMA��־λ
}_bsp_uart;

_bsp_uart _uart_cfg = {0};

void uart_gpio_clk_config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHB1PeriphClockCmd(UART_GPIO_CLK,ENABLE); //ʹ��UART_PORTʱ��
	RCC_APB2PeriphClockCmd(UART_CLK,ENABLE);//ʹ��USARTʱ��
    
    //���ڶ�Ӧ���Ÿ���ӳ��
    GPIO_PinAFConfig(UART_PORT,UART_TX_PIN_AF,UART_AF); //
    GPIO_PinAFConfig(UART_PORT,UART_RX_PIN_AF,UART_AF); //
	
	//USART�˿�����
    GPIO_InitStructure.GPIO_Pin = UART_TX_PIN | UART_RX_PIN; //
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
    GPIO_Init(UART_PORT,&GPIO_InitStructure); 
}


rt_err_t uart_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    rt_err_t ret = RT_EOK;
    _bsp_uart *uart = RT_NULL;
    
    RT_ASSERT(serial != RT_NULL);
    
    uart = rt_container_of(serial, _bsp_uart, device);
    
   //USART1 Ĭ�ϳ�ʼ������
    uart->uart_init.USART_BaudRate = cfg->baud_rate;//����������
    uart->uart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    uart->uart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    
    //���������޸�
    switch(cfg->data_bits)
    {
        case DATA_BITS_9:
            uart->uart_init.USART_WordLength = USART_WordLength_9b;//�ֳ�Ϊ8λ���ݸ�ʽ
            break;
        case DATA_BITS_8:
        default:
            uart->uart_init.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
            break;
    }
    switch(cfg->stop_bits)
    {
        case STOP_BITS_2:
            uart->uart_init.USART_StopBits = USART_StopBits_2;//2��ֹͣλ
            break;
        case STOP_BITS_1:
        default:
            uart->uart_init.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
            break;
    }
    switch(cfg->parity)
    {
        case PARITY_ODD:
            uart->uart_init.USART_Parity = USART_Parity_Odd;//����żУ��λ
            break;
        case PARITY_EVEN:
            uart->uart_init.USART_Parity = USART_Parity_Even;//����żУ��λ
            break;
        case PARITY_NONE:
        default:
            uart->uart_init.USART_Parity = USART_Parity_No;//����żУ��λ
            break;
    }
    
    USART_Init(uart->huart, &uart->uart_init); //��ʼ������1
	
    USART_Cmd(uart->huart, ENABLE);  //ʹ�ܴ���1 
	
    return ret;
}

//ֻ�����жϡ���ѯģʽ
rt_err_t uart_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    rt_err_t ret = RT_EOK;
    NVIC_InitTypeDef NVIC_InitStructure;
    _bsp_uart *uart = RT_NULL;
    
    RT_ASSERT(serial != RT_NULL);
    
    uart = rt_container_of(serial, _bsp_uart, device);
    
    //���ж�
    switch(cmd)
    {
        case RT_DEVICE_CTRL_SET_INT:
            //�����ж����ȼ�
            USART_ITConfig(uart->huart, USART_IT_RXNE, ENABLE);//��������ж�
            NVIC_InitStructure.NVIC_IRQChannel = UART_IRQ;//����1�ж�ͨ��
            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�3
            NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
            NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
            //ʹ���ж�
            break;
        case RT_DEVICE_CTRL_CLR_INT:
            USART_ITConfig(uart->huart, USART_IT_RXNE, DISABLE);//���ж�
            NVIC_InitStructure.NVIC_IRQChannel = UART_IRQ;//����1�ж�ͨ��
            NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;			//IRQͨ��ʧ��
            NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
            USART_Cmd(uart->huart, DISABLE);  //�رմ���
            USART_DeInit(uart->huart);
            break;
        default:
            ret = -RT_EINVAL;
            break;
    }
    
    return ret;
}


int uart_putc(struct rt_serial_device *serial, char c)
{
    _bsp_uart *uart = RT_NULL;
    
    RT_ASSERT(serial != RT_NULL);
    
    uart = rt_container_of(serial, _bsp_uart, device);
    
    while((uart->huart->SR&0X40)==0);//ѭ������,ֱ���������
    uart->huart->DR = (c & (uint16_t)0x01FF);
    
    return 1;
}


int uart_getc(struct rt_serial_device *serial)
{
    _bsp_uart *uart = RT_NULL;
    
    RT_ASSERT(serial != RT_NULL);
    
    uart = rt_container_of(serial, _bsp_uart, device);
    if(USART_GetFlagStatus(uart->huart, USART_FLAG_RXNE)) //��������ﲻ�жϽ��ջ������Ƿ�ǿգ���һֱ������
        return (uint16_t)(uart->huart->DR & (uint16_t)0x01FF);
    else 
        return -1;
}


rt_size_t uart_dma_transmit(struct rt_serial_device *serial, rt_uint8_t *buf, rt_size_t size, int direction)
{
    rt_size_t ret = 0;
    
    return ret;
}


static void uart_isr(struct rt_serial_device *serial)
{
    _bsp_uart *uart = RT_NULL;
    
    RT_ASSERT(serial != RT_NULL);
    
    uart = rt_container_of(serial, _bsp_uart, device);
    
    if(USART_GetITStatus(uart->huart, USART_IT_RXNE) != RESET)
    {
        rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);
    }
    
}


void USART1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr((struct rt_serial_device*)(&_uart_cfg));

    /* leave interrupt */
    rt_interrupt_leave();
}

struct rt_uart_ops _uart_ops =
{
    .configure = uart_configure,
    .control = uart_control,
    .putc = uart_putc,
    .getc = uart_getc,
    .dma_transmit = uart_dma_transmit
};





void bsp_uart_init(void)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    uart_gpio_clk_config();
    _uart_cfg.dma_flag = 0;
    _uart_cfg.device.config = config;
    _uart_cfg.device.ops = &_uart_ops;
    _uart_cfg.irqn = UART_IRQ;
    _uart_cfg.huart = UART_INSTANCE;

    rt_hw_serial_register(&(_uart_cfg.device),
                            UART_NAME,
                            RT_DEVICE_FLAG_RDWR
                           | RT_DEVICE_FLAG_INT_RX
                           | RT_DEVICE_FLAG_INT_TX
                           | _uart_cfg.dma_flag,
                           RT_NULL);
}










