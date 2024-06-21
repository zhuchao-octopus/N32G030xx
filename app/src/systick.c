/*!
    \file    systick.c
    \brief   the systick configuration file
    
    \version 2018-03-26, V1.0.0, demo for GD32E103
    \version 2020-09-30, V1.1.0, demo for GD32E103
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "public.h"


volatile static uint32_t delay;

/*!
    \brief      configure systick
    \param[in]  none
    \param[out] none
    \retval     none
*/
void systick_config(void)
{
    /* setup systick timer for 1000Hz interrupts */
    if (SysTick_Config(SystemCoreClock / 1000U)){
        /* capture error */
        while (1){
        }
    }
    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
}

/*!
    \brief      delay a time in milliseconds
    \param[in]  count: count in milliseconds
    \param[out] none
    \retval     none
*/
void delay_1ms(uint32_t count)
{
    delay = count;

    while(0U != delay){
    }
}

/*!
    \brief      delay decrement
    \param[in]  none
    \param[out] none
    \retval     none
*/
void delay_decrement(void)
{
	++g_counter_ms;
	if(g_counter_ms%4==0)
	{
		g_trigger_4ms=1;
		Counter_4ms++;
		if (0!=g_vol_key_info.debounce_timer)
			g_vol_key_info.debounce_timer--;
		if (0!=g_tune_key_info.debounce_timer)
			g_tune_key_info.debounce_timer--;
	}
	if(g_counter_ms%12==0)
	{
		g_trigger_12ms=1;
	}
	if(g_counter_ms%40==0)
	{
		g_trigger_40ms=1;
	}
	if(g_counter_ms%100==0)
	{
		g_trigger_100ms=1;

		if (AccOffTimer != 0) {
			--AccOffTimer;
		}

		if (g_acc_wait_timer != 0) {
			--g_acc_wait_timer;
		}

	}
	if(g_counter_ms==1000)
	{
		g_trigger_1000ms=1;
		g_counter_ms=0;
	}

	if (0U != delay){
		CLEAR_WATCHDOG;
		delay--;
	}
}
