/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     SummerGift   first version
 */

#include "board.h"


/*将系统时钟配置为HSE
m:PLL_M,进PLL前的主分频因子 2-63
n:pll_n,进PLL后的倍频因子 192-432(超频是配这个)
p:出PLL时的分频因子  2/4/6/8
q:配置USB的分频因子(USB_clk需要配置为48MHZ) 4-15
如果配置为168M : m=8  n=336  p=2  q=7
*/
int SetSysClock_HSE(uint32_t m, uint32_t n, uint32_t p, uint32_t q)
{
/******************************************************************************/
/*            PLL (clocked by HSE) used as System clock source                */
/******************************************************************************/
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
  
    RCC_HSEConfig(RCC_HSE_ON);//使能HSE
 
   //等待HSE稳定
    RCC_WaitForHSEStartUp();

    if (RCC_WaitForHSEStartUp() != RESET)
    {
        RCC->APB1ENR |= RCC_APB1ENR_PWREN; //配置电压输出级别最大频率
        PWR->CR |= PWR_CR_VOS; 

        /*
        RCC->CFGR |= RCC_CFGR_HPRE_DIV1;//AHB分配因子

        RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;//APB2分频因子

        RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;//APB1分频因子
        */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);//AHB分配因子
        RCC_PCLK2Config(RCC_HCLK_Div2);//APB2分频因子
        RCC_PCLK1Config(RCC_HCLK_Div4);//APB1分频因子

        /*可超频*/
        RCC->PLLCFGR = m | (n << 6) | (((p >> 1) -1) << 16) |
                   (RCC_PLLCFGR_PLLSRC_HSE) | (q << 24); //pll_clk=hse/m*n/p   usb_clk=hse/m*n/q
        
        RCC_PLLCmd(ENABLE);//使能PLL
        
        while((RCC->CR & RCC_CR_PLLRDY) == 0)
        {
        }
        
        /* 配置FLASH */
        FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;
        
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); //切换系统时钟为PLL
        while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL)//等待切换成功
        {
        }
        return 0;
    }
    else
    {
        return -1;
    }
}


/*将系统时钟配置为HSI
m:PLL_M,进PLL前的主分频因子 2-63
n:pll_n,进PLL后的倍频因子 192-432(超频是配这个)
p:出PLL时的分频因子  2/4/6/8
q:配置USB的分频因子(USB_clk需要配置为48MHZ) 4-15
如果配置为168M : m=16  n=336  p=2  q=7
*/
int SetSysClock_HSI(uint32_t m, uint32_t n, uint32_t p, uint32_t q)
{
/******************************************************************************/
/*            PLL (clocked by HSE) used as System clock source                */
/******************************************************************************/
  __IO uint32_t HSIStartUpStatus = 0, HSEStatus = 0;
  
    //复位时钟
    RCC_DeInit();
    
    RCC_HSICmd(ENABLE);//使能HSI
 
   //等待HSE稳定
    do
    {
        HSIStartUpStatus = RCC->CR & RCC_CR_HSIRDY;
    }while(!HSIStartUpStatus); //

    if (HSIStartUpStatus == RCC_CR_HSIRDY)
    {
        RCC->APB1ENR |= RCC_APB1ENR_PWREN; //配置电压输出级别最大频率
        PWR->CR |= PWR_CR_VOS; 

        /*
        RCC->CFGR |= RCC_CFGR_HPRE_DIV1;//AHB分配因子

        RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;//APB2分频因子

        RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;//APB1分频因子
        */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);//AHB分配因子
        RCC_PCLK2Config(RCC_HCLK_Div2);//APB2分频因子
        RCC_PCLK1Config(RCC_HCLK_Div4);//APB1分频因子

        /*可超频*/
        RCC->PLLCFGR = m | (n << 6) | (((p >> 1) -1) << 16) |
                   (RCC_PLLCFGR_PLLSRC_HSI) | (q << 24); //pll_clk=hse/m*n/p   usb_clk=hse/m*n/q
        
        RCC_PLLCmd(ENABLE);//使能PLL
        
        while((RCC->CR & RCC_CR_PLLRDY) == 0)
        {
        }
        
        /* 配置FLASH */
        FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;
        
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); //切换系统时钟为PLL
        while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL)//等待切换成功
        {
        }
        return 0;
    }
    else
    {
        return -1;
    }
}


int set_sysClk(int mode)
{
    if(mode == 0)
    {
        //hsi
        return SetSysClock_HSI(16,336,2,7);
    }
    else if(mode == 1)
    {
        //hse
        return SetSysClock_HSE(8,336,2,7);
    }
    else
    {
        return -1;
    }
}

void Set_sysTick(uint32_t sys_tick)//参考m3-m4权威指南
{
    //选择时钟源
    SysTick->CTRL &= SysTick_CLKSource_HCLK_Div8; //使用外部8分频时钟(就是把 systick->ctrl 第2位置0)
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;       //关闭计数器
//    SysTick->LOAD = sys_tick-1; //设置重装载值(根据权威指南上说的，如果设置1000的频率，不开启中断的话，则需要将reload的值设置为1000-1)
    SysTick->LOAD = sys_tick/8 -12; /*设置重装载值(根据权威指南上说的，如果设置1000的频率，
                                            若是开启中断，则可以设置12个tick的中断补偿)*/
    SysTick->VAL = 0; //清零当前计数器的值
    SysTick->CTRL |= 2 ;//使能systick中断
    SysTick->CTRL |= 1 ;//使能计数器
}


RT_WEAK void rt_hw_board_init()                                     
{
    /* 更新系统时钟 */
    set_sysClk(1);

    /* SysTick初始化 */
    Set_sysTick(SystemCoreClock / RT_TICK_PER_SECOND);

    /* 硬件BSP初始化统统放在这里，比如LED，串口，LCD等 */      
    extern void bsp_uart_init(void);
    bsp_uart_init();
    /* 调用组件初始化函数 (use INIT_BOARD_EXPORT()) */
    
#if defined(RT_USING_CONSOLE) && defined(RT_USING_DEVICE)
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);            
#endif
    
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();                                
#endif

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
    rt_system_heap_init((void*)HEAP_BEGIN, (void*)HEAP_END);
#endif
}

void SysTick_Handler(void)
{
    /* 进入中断 */
    rt_interrupt_enter();

    /* 更新时基 */
    rt_tick_increase();

    /* 离开中断 */
    rt_interrupt_leave();
}

