#include "spi_test.h"
#include "rtthread.h"
#include "stm32f4xx.h"
#include "drv_spi.h"

void spi_test(void)
{
    rt_err_t ret = RT_EOK;
    uint8_t send_data[] = {0x90, 0, 0, 0};
    uint8_t recv_data[10] = {0};
    struct rt_spi_device * spi_dev;
    struct rt_spi_configuration cfg;
	
    cfg.data_width = 8;
    cfg.max_hz = 2;
    cfg.mode = RT_SPI_MODE_3 | RT_SPI_MASTER | RT_SPI_MSB ;
    cfg.reserved = 0;
    
    rt_hw_spi_device_attach("spi1", "spi10", GPIOB, GPIO_Pin_14);

    spi_dev = (struct rt_spi_device *)rt_device_find("spi10");
    
    if(spi_dev)
    {
        
        rt_spi_configure(spi_dev, &cfg);
        if(ret == RT_EOK)
        {
            rt_spi_send_then_recv(spi_dev, &send_data, 4, &recv_data, 2);
            rt_kprintf("use rt_spi_send_then_recv() read w25q ID is:0x%x 0x%x\n", recv_data[0], recv_data[1]);
        }
    }
}









