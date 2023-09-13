#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include <drivers/spi.h>




//用于保存SPI的NS引脚信息,保存在 rt_spi_device->user_data 里面
struct stm32_spi_io
{
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
};

struct stm32_spi_config
{
    SPI_TypeDef *Instance;
    char *bus_name;
    IRQn_Type irq_type;
//    struct dma_config *dma_rx, *dma_tx;
};

struct stm32_spi
{
    SPI_InitTypeDef SPI_InitStruct;
    struct stm32_spi_config *config;
    struct rt_spi_configuration *cfg;

//    struct
//    {
//        DMA_HandleTypeDef handle_rx;
//        DMA_HandleTypeDef handle_tx;
//    } dma;

    rt_uint8_t spi_dma_flag;
    struct rt_spi_bus spi_bus;
};


#define SPI_BR          SPI_BaudRatePrescaler_2

#define USE_SPI_DMA     1
#define UNUSE_SPI_DMA   0

extern struct stm32_spi_config SPI_DEF_CONFIG; //spi默认配置


rt_err_t rt_hw_spi_device_attach(const char *bus_name, 
                                const char *device_name, 
                                GPIO_TypeDef *cs_gpiox, 
                                uint16_t cs_gpio_pin);

