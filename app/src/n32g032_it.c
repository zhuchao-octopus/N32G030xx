/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file n32g43x_it.c
 * @author Nations Solution Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "n32g032_it.h"
#include "public.h"

/** @addtogroup N32G43x_StdPeriph_Template
 * @{
 */

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief  This function handles NMI exception.
 */
void NMI_Handler(void)
{
}

/**
 * @brief  This function handles Hard Fault exception.
 */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}


/**
 * @brief  This function handles SVCall exception.
 */
void SVC_Handler(void)
{
}

/**
 * @brief  This function handles PendSV_Handler exception.
 */
void PendSV_Handler(void)
{
}

/**
 * @brief  This function handles SysTick Handler.
 */
void SysTick_Handler(void)
{
	delay_decrement();
}

/******************************************************************************/
/*                 N32G43x Peripherals Interrupt Handlers                     */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_n32g43x.s).                                                 */
/******************************************************************************/

void EXTI4_15_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_LINE7) != RESET)
	{
		/* Clear the Key Button EXTI line pending bit */
		EXTI_ClrITPendBit(EXTI_LINE7);
	}
}

void TIM1_CC_IRQHandler(void)
{
	if (TIM_GetIntStatus(TIM1, TIM_INT_CC2) == SET) {
		TIM_ClrIntPendingBit(TIM1, TIM_INT_CC2);
		ir_rx_handler();
	}
    if (TIM_GetIntStatus(TIM1, TIM_INT_CC1) == SET)
    {
    	uint16_t capture = 0;
        TIM_ClrIntPendingBit(TIM1, TIM_INT_CC1);
        capture = TIM_GetCap1(TIM1);
        TIM_SetCmp1(TIM1, capture + 500);
//		if (g_beep_info.mode != BEEP_MODE_NONE) {
//			TIM_EnableCapCmpCh(TIMER_BEEP, TIMER_CH_BEEP, TIM_CAP_CMP_ENABLE);
//		} else {
//			TIM_EnableCapCmpCh(TIMER_BEEP, TIMER_CH_BEEP, TIM_CAP_CMP_DISABLE);
//		}
    }
}

void USART1_2_IRQHandler(void)
{
	CLEAR_WATCHDOG;

	if (USART_GetFlagStatus(HOST_COMM_UART, USART_FLAG_OREF) != RESET)
	{
		host_rx_data(USART_ReceiveData(HOST_COMM_UART)&0xFF);
	}
	if (USART_GetFlagStatus(CAN_COMM_UART, USART_FLAG_OREF) != RESET)
	{
		canbox_rx(USART_ReceiveData(CAN_COMM_UART)&0xFF);
	}
	if (USART_GetFlagStatus(HOST_COMM_UART, USART_FLAG_RXDNE) != RESET)
	{
		USART_ClrFlag(HOST_COMM_UART, USART_FLAG_RXDNE);
		host_rx_data(USART_ReceiveData(HOST_COMM_UART)&0xFF);
	}
	if (USART_GetFlagStatus(CAN_COMM_UART, USART_FLAG_RXDNE) != RESET)
	{
		USART_ClrFlag(CAN_COMM_UART, USART_FLAG_RXDNE);
		canbox_rx(USART_ReceiveData(CAN_COMM_UART)&0xFF);
	}

}


/**
 * @brief  This function handles PPP interrupt request.
 */
/*void PPP_IRQHandler(void)
{
}*/

/**
 * @}
 */
