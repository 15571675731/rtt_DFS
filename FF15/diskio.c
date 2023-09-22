/*-----------------------------------------------------------------------*/
/* Low level disk I/O module SKELETON for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */

//#include "drv_spi.h"
#include "spi_flash_sfud.h"
#include "rtthread.h"

/* Definitions of physical drive number for each drive */
//#define DEV_RAM		0	/* Example: Map Ramdisk to physical drive 0 */
//#define DEV_MMC		1	/* Example: Map MMC/SD card to physical drive 1 */
//#define DEV_USB		2	/* Example: Map USB MSD to physical drive 2 */

#define SPI_FLASH	0


struct rt_spi_device * g_fatfs_spi_dev;
//sfud_flash * g_fatfs_spi_flash;

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	uint8_t send_data[] = {0x90, 0, 0, 0};
	uint8_t recv_data[4] = {0};

	rt_spi_send_then_recv(g_fatfs_spi_dev, &send_data, 4, &recv_data, 2);
	if(recv_data[0] != 0 && recv_data[0] != 0xff)
		stat = 0;
	else
		stat = STA_NOINIT;
	return stat;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/


sfud_flash *g_fatfs_spi_flash;
	

void sfud__()
{
	sfud_err result;
	uint8_t *read_data;  // 读取到的数据
	uint8_t *write_data; // 将要写入的数据
	sfud_flash *sfud_dev = NULL;

	// 或者 sfud_dev = rt_sfud_flash_find_by_dev_name("W25Q128");

	           // 擦除从 0 开始的 4096 字节

	write_data = rt_malloc(32);
	rt_memset(write_data, 1, 32);
	sfud_write(sfud_dev, 0, 32, write_data); // 将数据 32 字节的 write_data 从 0 开始写入 flash

	read_data = rt_malloc(32);
	sfud_read(sfud_dev, 0, 32, read_data);   // 读取从 0 开始的 32 字节，存入 read_data
}


DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
//	sfud_init();
	rt_kprintf("sfud_init success\r\n");
	g_fatfs_spi_dev = (struct rt_spi_device *)rt_device_find("spi10");
	
	rt_kprintf("rt_device_find success\r\n");
//    rt_sfud_flash_find_by_dev_name("");
	g_fatfs_spi_flash = rt_sfud_flash_find("spi10");
//    sfud_read(g_fatfs_spi_flash, 0x3f000, 4096, NULL);
//	sfud_erase(g_fatfs_spi_flash, 0, 4096);
//	g_fatfs_spi_flash = sfud_get_device(0);
	rt_kprintf("sfud_get_device success\r\n");
	return 0;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	LBA_t sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	DRESULT res;
	int result;
	
	result = sfud_read(g_fatfs_spi_flash, sector*4096, count*4096, buff);
    if (result == SFUD_SUCCESS)
    {
        return RES_OK;
    }

//	switch (pdrv) {
//	case DEV_RAM :
//		// translate the arguments here

//		result = RAM_disk_read(buff, sector, count);

//		// translate the reslut code here

//		return res;

//	case DEV_MMC :
//		// translate the arguments here

//		result = MMC_disk_read(buff, sector, count);

//		// translate the reslut code here

//		return res;

//	case DEV_USB :
//		// translate the arguments here

//		result = USB_disk_read(buff, sector, count);

//		// translate the reslut code here

//		return res;
//	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	LBA_t sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	DRESULT res;
	sfud_err result;
	
//	rt_spi_recv(g_fatfs_spi_dev, buff, );
	result = sfud_erase_write(g_fatfs_spi_flash, sector*4096, count*4096, buff);
	if(result == SFUD_SUCCESS)
		return RES_OK;
//	switch (pdrv) {
//	case DEV_RAM :
//		// translate the arguments here

//		result = RAM_disk_write(buff, sector, count);

//		// translate the reslut code here

//		return res;

//	case DEV_MMC :
//		// translate the arguments here

//		result = MMC_disk_write(buff, sector, count);

//		// translate the reslut code here

//		return res;

//	case DEV_USB :
//		// translate the arguments here

//		result = USB_disk_write(buff, sector, count);

//		// translate the reslut code here

//		return res;
//	}

	return RES_PARERR;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
	int result;

	switch (cmd) {
	case GET_SECTOR_COUNT :

		// 获取flash容量
		*(DWORD*)buff = 16777216;
		break;

	case GET_SECTOR_SIZE :

		// 获取扇区大小
		*(DWORD*)buff = 4096;
		break; 

	case GET_BLOCK_SIZE :

		// 获取最小擦除单位
		*(DWORD*)buff = 1;
		break;
	}

	return RES_OK;
}


////直接返回系统tick
//DWORD get_fattime(void)
//{
//	return rt_tick_get();
//}

