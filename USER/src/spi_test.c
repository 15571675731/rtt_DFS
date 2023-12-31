#include "spi_test.h"
#include "rtthread.h"
#include "stm32f4xx.h"
#include "drv_spi.h"
#include "spi_flash_sfud.h"


extern sfud_flash * g_fatfs_spi_flash;

void spi_test(void)
{
    rt_err_t ret = RT_EOK;
    uint8_t send_data[] = {0x90, 0, 0, 0};
    uint8_t recv_data[10] = {0};
    struct rt_spi_device * spi_dev;
    struct rt_spi_configuration cfg;
	rt_spi_flash_device_t spi_flash;
	
    cfg.data_width = 8;
    cfg.max_hz = 2;
    cfg.mode = RT_SPI_MODE_3 | RT_SPI_MASTER | RT_SPI_MSB ;
    cfg.reserved = 0;
    
    rt_hw_spi_device_attach("spi1", "spi10", GPIOB, GPIO_Pin_14);

    spi_dev = (struct rt_spi_device *)rt_device_find("spi10");
    
//    if(spi_dev)
//    {
//        
//        rt_spi_configure(spi_dev, &cfg);
//        if(ret == RT_EOK)
//        {
//            rt_spi_send_then_recv(spi_dev, &send_data, 4, &recv_data, 2);
//            rt_kprintf("use rt_spi_send_then_recv() read w25q ID is:0x%x 0x%x\n", recv_data[0], recv_data[1]);
//        }
//    }
	spi_flash = rt_sfud_flash_probe("W25Q128", "spi10");
	if (RT_NULL == spi_flash)  //注册块设备，这一步可以将外部                                flash抽象为系统的块设备
    {
        rt_kprintf("rt_sfud_flash_probe error\r\n");
    }
//	else
//		rt_kprintf("rt_sfud_flash_probe success\r\n");
}










