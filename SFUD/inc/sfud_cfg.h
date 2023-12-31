/*
 * This file is part of the Serial Flash Universal Driver Library.
 *
 * Copyright (c) 2016-2018, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: It is the configure head file for this library.
 * Created on: 2016-04-23
 */

#ifndef _SFUD_CFG_H_
#define _SFUD_CFG_H_

//打开关闭调试模式
#define SFUD_DEBUG_MODE

//打开关闭SFDP模式(若是芯片不支持SFDP，则关闭该宏，自己在 sfud_flash_def.h 中添加芯信息片)
#define SFUD_USING_SFDP

//是否使用该库自带的FLASH参数信息表(一般情况该宏与 SFUD_USING_SFDP 至少需要打开一个,也可以两个都开)
#define SFUD_USING_FLASH_INFO_TABLE

/*
* 若是 SFUD_USING_SFDP 和 SFUD_USING_FLASH_INFO_TABLE 都不定义
* 初始化方式有所改变，具体参见官方
*/


//enum {
//    SFUD_W25Q128_DEVICE_INDEX = 0,
//};

#define SFUD_FLASH_DEVICE_TABLE    	\
{				\
	0				\
}
//#define SFUD_FLASH_DEVICE_TABLE                                                \
//{                                                                              \
//    [SFUD_W25Q128_DEVICE_INDEX] = {.name = "W25Q128", .spi.name = "spi10"},           \
//}

//#define SFUD_USING_QSPI

#endif /* _SFUD_CFG_H_ */
