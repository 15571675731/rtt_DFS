#include "uart.h"




typedef struct Bsp_Uart
{
    /* 继承rtt的串口属性，用该成员将驱动层和应用层挂钩(通过该成员反向查找到struct BSP_Uart的起始地址，
     * 然后通过起始地址来配置STM32的专属串口设备控制块，在通过该控制块来初始化STM32的串口设备) */
    struct rt_serial_device device;

    USART_InitTypeDef uart_init;//STM32的专属串口属性，用于初始化STM32单片机的串口
    USART_TypeDef * huart;
    IRQn_Type irqn; //记录中断号

    rt_uint8_t dma_flag;//DMA标志位
}_bsp_uart;

_bsp_uart _uart_cfg = {0};

void uart_gpio_clk_config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHB1PeriphClockCmd(UART_GPIO_CLK,ENABLE); //使能UART_PORT时钟
	RCC_APB2PeriphClockCmd(UART_CLK,ENABLE);//使能USART时钟
    
    //串口对应引脚复用映射
    GPIO_PinAFConfig(UART_PORT,UART_TX_PIN_AF,UART_AF); //
    GPIO_PinAFConfig(UART_PORT,UART_RX_PIN_AF,UART_AF); //
	
	//USART端口配置
    GPIO_InitStructure.GPIO_Pin = UART_TX_PIN | UART_RX_PIN; //
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
    GPIO_Init(UART_PORT,&GPIO_InitStructure); 
}


rt_err_t uart_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    rt_err_t ret = RT_EOK;
    _bsp_uart *uart = RT_NULL;
    
    RT_ASSERT(serial != RT_NULL);
    
    uart = rt_container_of(serial, _bsp_uart, device);
    
   //USART1 默认初始化设置
    uart->uart_init.USART_BaudRate = cfg->baud_rate;//波特率设置
    uart->uart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    uart->uart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    
    //根据配置修改
    switch(cfg->data_bits)
    {
        case DATA_BITS_9:
            uart->uart_init.USART_WordLength = USART_WordLength_9b;//字长为8位数据格式
            break;
        case DATA_BITS_8:
        default:
            uart->uart_init.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
            break;
    }
    switch(cfg->stop_bits)
    {
        case STOP_BITS_2:
            uart->uart_init.USART_StopBits = USART_StopBits_2;//2个停止位
            break;
        case STOP_BITS_1:
        default:
            uart->uart_init.USART_StopBits = USART_StopBits_1;//一个停止位
            break;
    }
    switch(cfg->parity)
    {
        case PARITY_ODD:
            uart->uart_init.USART_Parity = USART_Parity_Odd;//无奇偶校验位
            break;
        case PARITY_EVEN:
            uart->uart_init.USART_Parity = USART_Parity_Even;//无奇偶校验位
            break;
        case PARITY_NONE:
        default:
            uart->uart_init.USART_Parity = USART_Parity_No;//无奇偶校验位
            break;
    }
    
    USART_Init(uart->huart, &uart->uart_init); //初始化串口1
	
    USART_Cmd(uart->huart, ENABLE);  //使能串口1 
	
    return ret;
}

//只配置中断、轮询模式
rt_err_t uart_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    rt_err_t ret = RT_EOK;
    NVIC_InitTypeDef NVIC_InitStructure;
    _bsp_uart *uart = RT_NULL;
    
    RT_ASSERT(serial != RT_NULL);
    
    uart = rt_container_of(serial, _bsp_uart, device);
    
    //开中断
    switch(cmd)
    {
        case RT_DEVICE_CTRL_SET_INT:
            //配置中断优先级
            USART_ITConfig(uart->huart, USART_IT_RXNE, ENABLE);//开启相关中断
            NVIC_InitStructure.NVIC_IRQChannel = UART_IRQ;//串口1中断通道
            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级3
            NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
            NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
            //使能中断
            break;
        case RT_DEVICE_CTRL_CLR_INT:
            USART_ITConfig(uart->huart, USART_IT_RXNE, DISABLE);//关中断
            NVIC_InitStructure.NVIC_IRQChannel = UART_IRQ;//串口1中断通道
            NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;			//IRQ通道失能
            NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
            USART_Cmd(uart->huart, DISABLE);  //关闭串口
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
    
    while((uart->huart->SR&0X40)==0);//循环发送,直到发送完毕
    uart->huart->DR = (c & (uint16_t)0x01FF);
    
    return 1;
}


int uart_getc(struct rt_serial_device *serial)
{
    _bsp_uart *uart = RT_NULL;
    
    RT_ASSERT(serial != RT_NULL);
    
    uart = rt_container_of(serial, _bsp_uart, device);
    if(USART_GetFlagStatus(uart->huart, USART_FLAG_RXNE)) //如果在这里不判断接收缓冲区是否非空，会一直被调用
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










