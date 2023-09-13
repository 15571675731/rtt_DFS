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


/*��ϵͳʱ������ΪHSE
m:PLL_M,��PLLǰ������Ƶ���� 2-63
n:pll_n,��PLL��ı�Ƶ���� 192-432(��Ƶ�������)
p:��PLLʱ�ķ�Ƶ����  2/4/6/8
q:����USB�ķ�Ƶ����(USB_clk��Ҫ����Ϊ48MHZ) 4-15
�������Ϊ168M : m=8  n=336  p=2  q=7
*/
int SetSysClock_HSE(uint32_t m, uint32_t n, uint32_t p, uint32_t q)
{
/******************************************************************************/
/*            PLL (clocked by HSE) used as System clock source                */
/******************************************************************************/
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
  
    RCC_HSEConfig(RCC_HSE_ON);//ʹ��HSE
 
   //�ȴ�HSE�ȶ�
    RCC_WaitForHSEStartUp();

    if (RCC_WaitForHSEStartUp() != RESET)
    {
        RCC->APB1ENR |= RCC_APB1ENR_PWREN; //���õ�ѹ����������Ƶ��
        PWR->CR |= PWR_CR_VOS; 

        /*
        RCC->CFGR |= RCC_CFGR_HPRE_DIV1;//AHB��������

        RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;//APB2��Ƶ����

        RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;//APB1��Ƶ����
        */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);//AHB��������
        RCC_PCLK2Config(RCC_HCLK_Div2);//APB2��Ƶ����
        RCC_PCLK1Config(RCC_HCLK_Div4);//APB1��Ƶ����

        /*�ɳ�Ƶ*/
        RCC->PLLCFGR = m | (n << 6) | (((p >> 1) -1) << 16) |
                   (RCC_PLLCFGR_PLLSRC_HSE) | (q << 24); //pll_clk=hse/m*n/p   usb_clk=hse/m*n/q
        
        RCC_PLLCmd(ENABLE);//ʹ��PLL
        
        while((RCC->CR & RCC_CR_PLLRDY) == 0)
        {
        }
        
        /* ����FLASH */
        FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;
        
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); //�л�ϵͳʱ��ΪPLL
        while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL)//�ȴ��л��ɹ�
        {
        }
        return 0;
    }
    else
    {
        return -1;
    }
}


/*��ϵͳʱ������ΪHSI
m:PLL_M,��PLLǰ������Ƶ���� 2-63
n:pll_n,��PLL��ı�Ƶ���� 192-432(��Ƶ�������)
p:��PLLʱ�ķ�Ƶ����  2/4/6/8
q:����USB�ķ�Ƶ����(USB_clk��Ҫ����Ϊ48MHZ) 4-15
�������Ϊ168M : m=16  n=336  p=2  q=7
*/
int SetSysClock_HSI(uint32_t m, uint32_t n, uint32_t p, uint32_t q)
{
/******************************************************************************/
/*            PLL (clocked by HSE) used as System clock source                */
/******************************************************************************/
  __IO uint32_t HSIStartUpStatus = 0, HSEStatus = 0;
  
    //��λʱ��
    RCC_DeInit();
    
    RCC_HSICmd(ENABLE);//ʹ��HSI
 
   //�ȴ�HSE�ȶ�
    do
    {
        HSIStartUpStatus = RCC->CR & RCC_CR_HSIRDY;
    }while(!HSIStartUpStatus); //

    if (HSIStartUpStatus == RCC_CR_HSIRDY)
    {
        RCC->APB1ENR |= RCC_APB1ENR_PWREN; //���õ�ѹ����������Ƶ��
        PWR->CR |= PWR_CR_VOS; 

        /*
        RCC->CFGR |= RCC_CFGR_HPRE_DIV1;//AHB��������

        RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;//APB2��Ƶ����

        RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;//APB1��Ƶ����
        */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);//AHB��������
        RCC_PCLK2Config(RCC_HCLK_Div2);//APB2��Ƶ����
        RCC_PCLK1Config(RCC_HCLK_Div4);//APB1��Ƶ����

        /*�ɳ�Ƶ*/
        RCC->PLLCFGR = m | (n << 6) | (((p >> 1) -1) << 16) |
                   (RCC_PLLCFGR_PLLSRC_HSI) | (q << 24); //pll_clk=hse/m*n/p   usb_clk=hse/m*n/q
        
        RCC_PLLCmd(ENABLE);//ʹ��PLL
        
        while((RCC->CR & RCC_CR_PLLRDY) == 0)
        {
        }
        
        /* ����FLASH */
        FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;
        
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); //�л�ϵͳʱ��ΪPLL
        while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL)//�ȴ��л��ɹ�
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

void Set_sysTick(uint32_t sys_tick)//�ο�m3-m4Ȩ��ָ��
{
    //ѡ��ʱ��Դ
    SysTick->CTRL &= SysTick_CLKSource_HCLK_Div8; //ʹ���ⲿ8��Ƶʱ��(���ǰ� systick->ctrl ��2λ��0)
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;       //�رռ�����
//    SysTick->LOAD = sys_tick-1; //������װ��ֵ(����Ȩ��ָ����˵�ģ��������1000��Ƶ�ʣ��������жϵĻ�������Ҫ��reload��ֵ����Ϊ1000-1)
    SysTick->LOAD = sys_tick/8 -12; /*������װ��ֵ(����Ȩ��ָ����˵�ģ��������1000��Ƶ�ʣ�
                                            ���ǿ����жϣ����������12��tick���жϲ���)*/
    SysTick->VAL = 0; //���㵱ǰ��������ֵ
    SysTick->CTRL |= 2 ;//ʹ��systick�ж�
    SysTick->CTRL |= 1 ;//ʹ�ܼ�����
}


RT_WEAK void rt_hw_board_init()                                     
{
    /* ����ϵͳʱ�� */
    set_sysClk(1);

    /* SysTick��ʼ�� */
    Set_sysTick(SystemCoreClock / RT_TICK_PER_SECOND);

    /* Ӳ��BSP��ʼ��ͳͳ�����������LED�����ڣ�LCD�� */      
    extern void bsp_uart_init(void);
    bsp_uart_init();
    /* ���������ʼ������ (use INIT_BOARD_EXPORT()) */
    
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
    /* �����ж� */
    rt_interrupt_enter();

    /* ����ʱ�� */
    rt_tick_increase();

    /* �뿪�ж� */
    rt_interrupt_leave();
}

