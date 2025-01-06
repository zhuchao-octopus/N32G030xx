
#define AK_ROOT
#include "public.h"

typedef void(*pFunction) (void);
pFunction		Jump_To_Application;
uint32_t		JumpAddress = 0;


static u8		g_first_love;


static void rcu_config(void)
{
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOC, ENABLE);
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOF, ENABLE);
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);

	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_USART1, ENABLE);
	RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_USART2, ENABLE);

	RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC, ENABLE);
	RCC_EnableHsi(ENABLE);
	RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8);
	ADC_ConfigClk(ADC_CTRL3_CKMOD_AHB, RCC_ADCHCLK_DIV16);

	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_TIM1, ENABLE);
	RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM3, ENABLE);
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_TIM8, ENABLE);
}


static void gpio_config(void)
{
	GPIO_InitType	GPIO_InitStructure_AFPP;
	GPIO_InitType	GPIO_InitStructure_Analog;
	GPIO_InitType	GPIO_InitStructure_Input_Pullup;
	GPIO_InitType	GPIO_InitStructure_Output;
	GPIO_InitType	gpio_init_input_af;
	GPIO_InitType	GPIO_InitStructure_Input_Float;

	GPIO_InitStruct(&GPIO_InitStructure_AFPP);
	GPIO_InitStructure_AFPP.GPIO_Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct(&GPIO_InitStructure_Analog);
	GPIO_InitStructure_Analog.GPIO_Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct(&GPIO_InitStructure_Input_Pullup);
	GPIO_InitStructure_Input_Pullup.GPIO_Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure_Input_Pullup.GPIO_Pull = GPIO_PULL_UP;
	GPIO_InitStruct(&GPIO_InitStructure_Output);
	GPIO_InitStructure_Output.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct(&gpio_init_input_af);
	gpio_init_input_af.GPIO_Mode = GPIO_MODE_AF_PP;
	gpio_init_input_af.GPIO_Pull = GPIO_PULL_UP;
	GPIO_InitStruct(&GPIO_InitStructure_Input_Float);
	GPIO_InitStructure_Input_Float.GPIO_Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure_Input_Float.GPIO_Pull = GPIO_NO_PULL;

	/* MUTE control */
	GPIO_InitStructure_Output.Pin = GPIO_MUTE_PIN;
	GPIO_InitPeripheral(GPIO_MUTE_GRP, &GPIO_InitStructure_Output);
	GPIO_SetBits(GPIO_MUTE_GRP, GPIO_MUTE_PIN);

	if (0 != g_first_love)
		{

		/* System power control */
		GPIO_InitStructure_Output.Pin = GPIO_REAL_SYS_PWR_PIN;
		GPIO_InitPeripheral(GPIO_REAL_SYS_PWR_GRP, &GPIO_InitStructure_Output);
		GPIO_ResetBits(GPIO_REAL_SYS_PWR_GRP, GPIO_REAL_SYS_PWR_PIN);

		/* Devices 5V power control */
		GPIO_InitStructure_Output.Pin = GPIO_DEVICE_5V_PWR_PIN;
		GPIO_InitPeripheral(GPIO_DEVICE_5V_PWR_GRP, &GPIO_InitStructure_Output);
		GPIO_ResetBits(GPIO_DEVICE_5V_PWR_GRP, GPIO_DEVICE_5V_PWR_PIN);

		/* Audio power control */
		GPIO_InitStructure_Output.Pin = GPIO_AUDIO_PWR_PIN;
		GPIO_InitPeripheral(GPIO_AUDIO_PWR_GRP, &GPIO_InitStructure_Output);
		GPIO_ResetBits(GPIO_AUDIO_PWR_GRP, GPIO_AUDIO_PWR_PIN);

		/* Fcam power control */
		//		GPIO_InitStructure_Output.Pin = GPIO_FCAM_PWR_PIN;
		//		GPIO_InitPeripheral(GPIO_FCAM_PWR_GRP, &GPIO_InitStructure_Output);
		//		GPIO_ResetBits(GPIO_FCAM_PWR_GRP, GPIO_FCAM_PWR_PIN);
		}

	/* Radio reset GPIO */
	GPIO_InitStructure_Output.Pin = GPIO_RADIO_RST_PIN;
	GPIO_InitPeripheral(GPIO_RADIO_RST_GRP, &GPIO_InitStructure_Output);
	GPIO_ResetBits(GPIO_RADIO_RST_GRP, GPIO_RADIO_RST_PIN);

	/* UART HOST */
	GPIO_InitStructure_AFPP.GPIO_Alternate = GPIO_AF4_USART1;
	GPIO_InitStructure_AFPP.Pin = GPIO_HOST_UART_TX_PIN;
	GPIO_InitPeripheral(GPIO_HOST_UART_TX_GRP, &GPIO_InitStructure_AFPP);
	GPIO_InitStructure_AFPP.GPIO_Alternate = GPIO_AF4_USART1;
	GPIO_InitStructure_AFPP.Pin = GPIO_HOST_UART_RX_PIN;
	GPIO_InitPeripheral(GPIO_HOST_UART_RX_GRP, &GPIO_InitStructure_AFPP);

	/* UART 1 TX confilct with SWD, so config it later */

	/* ACC */
	GPIO_InitStructure_Input_Pullup.Pin = GPIO_ACC_PIN;
	GPIO_InitPeripheral(GPIO_ACC_GRP, &GPIO_InitStructure_Input_Pullup);

	/* Bake */
	GPIO_InitStructure_Input_Pullup.Pin = GPIO_BRAKE_PIN;
	GPIO_InitPeripheral(GPIO_BRAKE_GRP, &GPIO_InitStructure_Input_Pullup);

	/* Ill detect */
	GPIO_InitStructure_Input_Pullup.Pin = GPIO_ILL_PIN;
	GPIO_InitPeripheral(GPIO_ILL_GRP, &GPIO_InitStructure_Input_Pullup);

	/* Reverse detect */
	GPIO_InitStructure_Input_Pullup.Pin = GPIO_REVERSE_PIN;
	GPIO_InitPeripheral(GPIO_REVERSE_GRP, &GPIO_InitStructure_Input_Pullup);

	/* ANT control */
	GPIO_InitStructure_Output.Pin = GPIO_ANT_CTRL_PIN;
	GPIO_InitPeripheral(GPIO_ANT_CTRL_GRP, &GPIO_InitStructure_Output);
	GPIO_ResetBits(GPIO_ANT_CTRL_GRP, GPIO_ANT_CTRL_PIN);

	/* Radio ANT control */
	GPIO_InitStructure_Output.Pin = GPIO_RADIO_ANT_CTRL_PIN;
	GPIO_InitPeripheral(GPIO_RADIO_ANT_CTRL_GRP, &GPIO_InitStructure_Output);
	GPIO_ResetBits(GPIO_RADIO_ANT_CTRL_GRP, GPIO_RADIO_ANT_CTRL_PIN);

	/* Backlight enable */
	GPIO_InitStructure_AFPP.GPIO_Alternate = GPIO_AF2_TIM8;
	GPIO_InitStructure_AFPP.Pin = GPIO_BKL_EN_PIN;
	GPIO_InitPeripheral(GPIO_BKL_EN_GRP, &GPIO_InitStructure_AFPP);

	/* SWC key pull-up control */
	GPIO_InitStructure_Output.Pin = GPIO_SWC_KEY_CTL_2K2_PIN;
	GPIO_InitPeripheral(GPIO_SWC_KEY_CTL_2K2_GRP, &GPIO_InitStructure_Output);
	GPIO_InitStructure_Output.Pin = GPIO_SWC_KEY_CTL_470R_PIN;
	GPIO_InitPeripheral(GPIO_SWC_KEY_CTL_470R_GRP, &GPIO_InitStructure_Output);


	/* ADC */
	GPIO_InitStructure_Analog.Pin = GPIO_AD_BATT_DET_PIN;
	GPIO_InitPeripheral(GPIO_AD_BATT_DET_GRP, &GPIO_InitStructure_Analog);
	GPIO_InitStructure_Analog.Pin = GPIO_AD_VOL_PIN;
	GPIO_InitPeripheral(GPIO_AD_VOL_GRP, &GPIO_InitStructure_Analog);
	GPIO_InitStructure_Analog.Pin = GPIO_AD_TUNE_PIN;
	GPIO_InitPeripheral(GPIO_AD_TUNE_GRP, &GPIO_InitStructure_Analog);
	
	GPIO_InitStructure_Analog.Pin = GPIO_AD_PANEL_KEY_DET_1_PIN | GPIO_AD_PANEL_KEY_DET_2_PIN;
	GPIO_InitPeripheral(GPIO_AD_PANEL_KEY_DET_GRP, &GPIO_InitStructure_Analog);
	
	GPIO_InitStructure_Analog.Pin = GPIO_AD_SWC_KEY_DET_1_PIN | GPIO_AD_SWC_KEY_DET_2_PIN;
	GPIO_InitPeripheral(GPIO_AD_SWC_KEY_DET_GRP, &GPIO_InitStructure_Analog);


	/* LED control */
	GPIO_InitStructure_AFPP.Pin = GPIO_LED_B_PIN | GPIO_LED_G_PIN | GPIO_LED_R_PIN;
	GPIO_InitStructure_AFPP.GPIO_Alternate = GPIO_AF2_TIM3;
	GPIO_InitPeripheral(GPIO_LED_GRP, &GPIO_InitStructure_AFPP);
	GPIO_InitStructure_Output.Pin = GPIO_LED_EN_PIN;
	GPIO_InitPeripheral(GPIO_LED_GRP, &GPIO_InitStructure_Output);
	GPIO_ResetBits(GPIO_LED_GRP, GPIO_LED_EN_PIN);


	/* HOST recovery key */
	GPIO_InitStructure_Output.Pin = GPIO_HOST_REC_KEY_PIN;
	GPIO_InitPeripheral(GPIO_HOST_REC_KEY_GRP, &GPIO_InitStructure_Output);
	GPIO_SetBits(GPIO_HOST_REC_KEY_GRP, GPIO_HOST_REC_KEY_PIN);

	/* HOST power key */
	if (0 != g_first_love)
		{
		GPIO_InitStructure_Output.Pin = GPIO_HOST_PWR_KEY_PIN;
		GPIO_InitPeripheral(GPIO_HOST_PWR_KEY_GRP, &GPIO_InitStructure_Output);
		GPIO_ResetBits(GPIO_HOST_PWR_KEY_GRP, GPIO_HOST_PWR_KEY_PIN);
		}

	/* BEEP */
	GPIO_InitStructure_AFPP.Pin = GPIO_BEEP_PIN;

	//	GPIO_InitStructure_AFPP.GPIO_Alternate = GPIO_AF2_TIM1;
	//	GPIO_InitPeripheral(GPIO_BEEP_GRP, &GPIO_InitStructure_AFPP);
	GPIO_InitStructure_Input_Float.Pin = GPIO_BEEP_PIN;
	GPIO_InitPeripheral(GPIO_BEEP_GRP, &GPIO_InitStructure_Input_Float);

	/* IR rx */
	gpio_init_input_af.Pin = GPIO_IR_RX_PIN;
	gpio_init_input_af.GPIO_Alternate = GPIO_AF3_TIM1;
	GPIO_InitPeripheral(GPIO_IR_RX_GRP, &gpio_init_input_af);

}


static void uart_config(void)
{
	USART_InitType	USART_InitStructure;
	NVIC_InitType	NVIC_InitStructure;

	/* UART for HOST comm */
	NVIC_InitStructure.NVIC_IRQChannel = HOST_COMM_UART_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_DeInit(HOST_COMM_UART);
	USART_InitStructure.BaudRate = 115200;
	USART_InitStructure.WordLength = USART_WL_8B;
	USART_InitStructure.StopBits = USART_STPB_1;
	USART_InitStructure.Parity = USART_PE_NO;
	USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
	USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;
	USART_Init(HOST_COMM_UART, &USART_InitStructure);
	USART_ConfigInt(HOST_COMM_UART, USART_INT_RXDNE, ENABLE);
	USART_Enable(HOST_COMM_UART, ENABLE);


	/* UART for CAN comm */
	// use the same irq with HOST comm
	USART_DeInit(CAN_COMM_UART);
	USART_Init(CAN_COMM_UART, &USART_InitStructure);
	USART_ConfigInt(CAN_COMM_UART, USART_INT_RXDNE, ENABLE);
	USART_Enable(CAN_COMM_UART, ENABLE);

}


static void adc_config(void)
{
	ADC_InitType	ADC_InitStructure;

	ADC_DeInit(ADC);
	ADC_InitStructure.MultiChEn = DISABLE;
	ADC_InitStructure.ContinueConvEn = DISABLE;
	ADC_InitStructure.ExtTrigSelect = ADC_EXT_TRIGCONV_NONE;
	ADC_InitStructure.DatAlign = ADC_DAT_ALIGN_R;
	ADC_InitStructure.ChsNumber = 1;
	ADC_Init(ADC, &ADC_InitStructure);
	ADC_Enable(ADC, ENABLE);

	/* Check ADC Ready */
	while (ADC_GetFlagStatusNew(ADC, ADC_FLAG_RDY) == RESET)
		;

	while (ADC_GetFlagStatusNew(ADC, ADC_FLAG_PD_RDY))
		;
}


static void timer_config(void)
{
	TIM_TimeBaseInitType TIM_TimeBaseStructure;
	OCInitType		TIM_OCInitStructure;
	TIM_ICInitType	TIM_ICInitStructure;
	NVIC_InitType	NVIC_InitStructure;

	/* Backlight */
	TIM_DeInit(TIMER_LCD_BKL);

	TIM_InitTimBaseStruct(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.Period = 999;
	TIM_TimeBaseStructure.Prescaler = 0;
	TIM_TimeBaseStructure.ClkDiv = 0;
	TIM_TimeBaseStructure.CntMode = TIM_CNT_MODE_UP;
	TIM_InitTimeBase(TIMER_LCD_BKL, &TIM_TimeBaseStructure);

	TIM_InitOcStruct(&TIM_OCInitStructure);
	TIM_OCInitStructure.OcMode = TIM_OCMODE_INACTIVE;
	TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
	TIM_OCInitStructure.Pulse = 900;
	TIM_OCInitStructure.OcPolarity = TIM_OC_POLARITY_HIGH;
	TIM_InitOc3(TIMER_LCD_BKL, &TIM_OCInitStructure);
	TIM_ConfigOc3Preload(TIMER_LCD_BKL, TIM_OC_PRE_LOAD_ENABLE);

	TIM_ConfigArPreload(TIMER_LCD_BKL, ENABLE);
	TIM_EnableCtrlPwmOutputs(TIMER_LCD_BKL, ENABLE);
	TIM_Enable(TIMER_LCD_BKL, ENABLE);


	/* Beep & IR RX */
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_DeInit(TIMER_BEEP);

	TIM_InitTimBaseStruct(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.Period = 65535;
	TIM_TimeBaseStructure.Prescaler = 95;
	TIM_TimeBaseStructure.ClkDiv = 0;
	TIM_TimeBaseStructure.CntMode = TIM_CNT_MODE_UP;
	TIM_InitTimeBase(TIMER_BEEP, &TIM_TimeBaseStructure);

	TIM_ICInitStructure.Channel = TIMER_CH_IR_RX;
	TIM_ICInitStructure.IcPolarity = TIM_IC_POLARITY_FALLING;
	TIM_ICInitStructure.IcSelection = TIM_IC_SELECTION_DIRECTTI;
	TIM_ICInitStructure.IcPrescaler = TIM_IC_PSC_DIV1;
	TIM_ICInitStructure.IcFilter = 0x0;
	TIM_ICInit(TIMER_IR_RX, &TIM_ICInitStructure);
	TIM_ConfigInt(TIMER_IR_RX, TIM_INT_CC2, ENABLE);

	TIM_InitOcStruct(&TIM_OCInitStructure);
	TIM_OCInitStructure.OcMode = TIM_OCMODE_TOGGLE;

	//	TIM_OCInitStructure.OcMode		= TIM_OCMODE_INACTIVE;
	TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
	TIM_OCInitStructure.Pulse = 500;
	TIM_OCInitStructure.OcPolarity = TIM_OC_POLARITY_LOW;
	TIM_InitOc1(TIMER_BEEP, &TIM_OCInitStructure);
	TIM_EnableOc1Preload(TIMER_BEEP, TIM_OC_PRE_LOAD_DISABLE);
	TIM_ConfigArPreload(TIMER_BEEP, ENABLE);
	TIM_EnableCtrlPwmOutputs(TIMER_BEEP, ENABLE);
	TIM_Enable(TIMER_BEEP, ENABLE);
	TIM_ConfigInt(TIMER_BEEP, TIM_INT_CC1, ENABLE);

	//	TIM_ConfigInt(TIMER_BEEP, TIM_INT_CC1, DISABLE);

	/* LED color */
	TIM_DeInit(TIMER_LED);
	TIM_InitTimBaseStruct(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.Period = 256;
	TIM_TimeBaseStructure.Prescaler = 499;
	TIM_TimeBaseStructure.ClkDiv = 0;
	TIM_TimeBaseStructure.CntMode = TIM_CNT_MODE_UP;
	TIM_InitTimeBase(TIMER_LED, &TIM_TimeBaseStructure);

	TIM_InitOcStruct(&TIM_OCInitStructure);
	TIM_OCInitStructure.OcMode = TIM_OCMODE_PWM1;
	TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
	TIM_OCInitStructure.Pulse = 0;
	TIM_OCInitStructure.OcPolarity = TIM_OC_POLARITY_HIGH;
	TIM_InitOc1(TIMER_LED, &TIM_OCInitStructure);
	TIM_InitOc2(TIMER_LED, &TIM_OCInitStructure);
	TIM_InitOc4(TIMER_LED, &TIM_OCInitStructure);

	TIM_EnableOc1Preload(TIMER_LED, TIM_OC_PRE_LOAD_ENABLE);
	TIM_ConfigOc2Preload(TIMER_LED, TIM_OC_PRE_LOAD_ENABLE);
	TIM_ConfigOc4Preload(TIMER_LED, TIM_OC_PRE_LOAD_ENABLE);
	TIM_ConfigArPreload(TIMER_LED, ENABLE);

	//	TIM_EnableCtrlPwmOutputs(TIMER_LED, ENABLE);
	TIM_Enable(TIMER_LED, ENABLE);

}


#ifdef ENABLE_WWDG


static void watchdog_enable(void)
{
	WWDG_ClrEWINTF();
	RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_WWDG, ENABLE);
	WWDG_SetPrescalerDiv(WWDG_PRESCALER_DIV8);
	WWDG_SetWValue(127);
	WWDG_Enable(127);
}


static void watchdog_disable(void)
{
	WWDG_DeInit();
	RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_WWDG, DISABLE);
}


#endif


static void mcu_init(void)
{
	g_counter_ms		= 0;
	g_trigger_4ms		= 0;
	g_trigger_12ms		= 0;
	g_trigger_40ms		= 0;
	g_trigger_100ms 	= 0;
	g_trigger_1000ms	= 0;

	systick_config();

	rcu_config();
	gpio_config();
	adc_config();
	timer_config();
	uart_config();

	i2c_init();
	hal_init();
}


#define MMU_VTOR				((__IO unsigned*)(0x40024C30))	 
#define _VTOREN()				(*MMU_VTOR = (*MMU_VTOR) | 0x88000000);
#define _VTORVALUE()			(*MMU_VTOR = (*MMU_VTOR) | 0x000E000);


void main_goto_iap(void)
{
	if (((* (vu32 *) FLASH_ADDR_UPDATER) & 0x0FFFFFFF) < 1024 * 8) //ApplicationAddress为新程序的起始地址，检查栈顶地址是否合法，即栈顶地址是否为0x2000xxxx（内置SRAM）
		{
#ifdef ENABLE_WWDG
		watchdog_disable();
#endif

		// Mute
		AUDIO_HW_MUTE;

		// turn off BEEP
		TIM_ConfigForcedOc1(TIMER_BEEP, TIM_OCMODE_INACTIVE);
		TIM_Enable(TIMER_BEEP, DISABLE);
		TIM_DeInit(TIMER_BEEP);

		USART_ConfigInt(HOST_COMM_UART, USART_INT_RXDNE, DISABLE);
		USART_Enable(HOST_COMM_UART, DISABLE);

		USART_ConfigInt(CAN_COMM_UART, USART_INT_RXDNE, DISABLE);
		USART_Enable(CAN_COMM_UART, DISABLE);

		SysTick->CTRL		= 0;

		_VTOREN();
		_VTORVALUE();
		JumpAddress 		= * (vu32 *) (FLASH_ADDR_UPDATER + 4); //用户代码区第二个字存储为新程序起始地址（新程序复位向量指针）
		Jump_To_Application = (pFunction) JumpAddress;
		__set_MSP(* (vu32 *) FLASH_ADDR_UPDATER);	//初始化APP堆栈指针(用户代码区的第一个字用于存放栈顶地址)
		Jump_To_Application();						//设置PC指针为新程序复位中断函数的地址
		}
}


static void Task4msPro()
{
	g_trigger_4ms		= 0;


	LinUart_Resend_Time();
	Usart_Resend_Time();

	encoder_key_main();
}

void resetRadio()
{
  GPIO_InitType	GPIO_InitStructure_Input_Float;

   bool b = FALSE;
   if(b)
    {
    GPIO_InitStructure_Input_Float.Pin = GPIO_RADIO_RST_PIN;
	  GPIO_InitPeripheral(GPIO_RADIO_RST_GRP, &GPIO_InitStructure_Input_Float);
    radio_init();
    }
}

static void Task12msPro()
{
	g_trigger_12ms		= 0;
	CLEAR_WATCHDOG;
	radio_main();
	channel_main();
//	Video_Main();
	Evt_Lifetime_Manage();			
	audio_main();
	Illumi_Detect();

	Voltage_Error_Detect();
	ACC_On_Detect(N_SAMPLING_ACC);

	panel_key_main();

	CLEAR_WATCHDOG;
	ACC_Off_Detect();
	PowerManage();
	LinTxWince_Service();
	beep_main();

	Canbox_Main();

	Reverse_Detect();

	ir_rx_main();

	//	can_ir_main();
}


static void Task40msPro()
{
	g_trigger_40ms		= 0;
	MMI();
	Parking_Pro();
	ABRT_Driver();
}


static void Task100msPro()
{

	g_trigger_100ms 	= 0;

	led_main();

	//	app_wd_main();
}


static void Task1sPro()
{
	g_trigger_1000ms	= 0;
	batt_main();

	if (g_startup_cntr < 250)
		{
		++g_startup_cntr;
		}

	if (g_super_watchdog_timer > 0)
		{
		--g_super_watchdog_timer;

		if ((g_startup_cntr > 10) && (0 == g_super_watchdog_timer))
			{
			// power off & on all system
			g_startup_cntr		= 0;

			// move from AUDIO_STATE_PWR_OFF_ING == g_audio_info.state
			audio_set_mute(AUDIO_MUTE_DRIVER, TRUE);
			audio_dev_deinit();
			g_audio_info.pwr_timer = 0;
			g_audio_info.bt_phone_timer = 0;
			g_audio_info.bt_phone_on = FALSE;
			g_audio_info.carplay_phone_on = FALSE;
			g_audio_info.navi_break_on = FALSE;
			g_audio_info.navi_break_on_cache = FALSE;
			g_audio_info.reverse_on = FALSE;

			// move from RADIO_STATE_PWR_OFF_ING == g_radio_info.state
			radio_dev_deinit();
			g_radio_info.state	= RADIO_STATE_UNKNOWN;

			// power down
			REAL_SYS_PWR_OFF;

			// delay
			delay_1ms(2000);

			// power up
			REAL_SYS_PWR_ON;

			audio_set_pwr_ctrl(TRUE);
			radio_set_pwr_ctrl(TRUE);

			}
		}
}


void VariableInit(void)
{
	nPowerState 		= POWER_SYSTEM_RESET;

	g_fake_pwr_off		= FALSE;

	g_app_in_charge 	= FALSE;

	g_canbox_use_uart	= FALSE;

	g_bkl_usr_ctrl		= 1;

	//	g_bt_type = 0;
	g_startup_cntr		= 0;

	g_super_watchdog_timer = 0;


	audio_init();
	radio_init();

	rtc_setup();

	led_init();
	beep_init();
	ir_rx_init();

	//	ir_tx_init();
	//	can_ir_init();
	panel_key_init();

	FrontSource 		= NUM_OF_SOURCE;

	Clr_Machine_Power();
	InitEvent();

	PowerOffReason		= POWER_STANDBY;

	g_brightness		= -1;

	// todo	g_rtc_h = 0;
	// todo	g_rtc_l = 0;
	// todo	g_rtc_cntr = 0;
}


void SetSysClockToHSE(void)
{
	ErrorStatus 	HSEStartUpStatus;

	/* SYSCLK, HCLK, PCLK2 and PCLK1 configuration
	* -----------------------------*/
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable HSE */
	RCC_ConfigHse(RCC_HSE_ENABLE);

	/* Wait till HSE is ready */
	HSEStartUpStatus	= RCC_WaitHseStable();

	if (HSEStartUpStatus == SUCCESS)
		{
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufSet(FLASH_PrefetchBuf_EN);

		if (HSE_Value <= 18000000)
			{
			/* Flash 0 wait state */
			FLASH_SetLatency(FLASH_LATENCY_0);
			}
		else 
			{
			/* Flash 1 wait state */
			FLASH_SetLatency(FLASH_LATENCY_1);
			}

		/* HCLK = SYSCLK */
		RCC_ConfigHclk(RCC_SYSCLK_DIV512);

		/* PCLK2 = HCLK */
		RCC_ConfigPclk2(RCC_HCLK_DIV1);

		/* PCLK1 = HCLK */
		RCC_ConfigPclk1(RCC_HCLK_DIV1);

		/* Select HSE as system clock source */
		RCC_ConfigSysclk(RCC_SYSCLK_SRC_HSE);

		/* Wait till HSE is used as system clock source */
		while (RCC_GetSysclkSrc() != RCC_CFG_SCLKSTS_HSE)
			{
			}
		}
	else 
		{
		/* If HSE fails to start-up, the application will have wrong clock
		configuration. User can add here some code to deal with this error */

		/* Go to infinite loop */
		//		while (1)
		//		{
		//		}
		}
}


static void mcu_stay_in_sleep(void)
{
	GPIO_InitType	GPIO_InitStructure_Input_Float;
	u16 			acc_on = 0;
	u16 			acc_off = 0;
	///NVIC_InitType	NVIC_InitStructure;
	EXTI_InitType	EXTI_InitStructure;

	/* shutdown unused mcu pin & clock & power */
	GPIO_InitStruct(&GPIO_InitStructure_Input_Float);
	GPIO_InitStructure_Input_Float.GPIO_Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure_Input_Float.GPIO_Pull = GPIO_NO_PULL;

	/* UART HOST */
	GPIO_InitStructure_Input_Float.Pin = GPIO_HOST_UART_TX_PIN;
	GPIO_InitPeripheral(GPIO_HOST_UART_TX_GRP, &GPIO_InitStructure_Input_Float);
	GPIO_InitStructure_Input_Float.Pin = GPIO_HOST_UART_RX_PIN;
	GPIO_InitPeripheral(GPIO_HOST_UART_RX_GRP, &GPIO_InitStructure_Input_Float);

	/* UART CAN */
#ifndef ENABLE_SWD
	GPIO_InitStructure_Input_Float.Pin = GPIO_CAN_UART_TX_PIN;
	GPIO_InitPeripheral(GPIO_CAN_UART_TX_GRP, &GPIO_InitStructure_Input_Float);
#endif

	GPIO_InitStructure_Input_Float.Pin = GPIO_CAN_UART_RX_PIN;
	GPIO_InitPeripheral(GPIO_CAN_UART_RX_GRP, &GPIO_InitStructure_Input_Float);

	/* Bake */
	GPIO_InitStructure_Input_Float.Pin = GPIO_BRAKE_PIN;
	GPIO_InitPeripheral(GPIO_BRAKE_GRP, &GPIO_InitStructure_Input_Float);

	/* Ill detect */
	GPIO_InitStructure_Input_Float.Pin = GPIO_ILL_PIN;
	GPIO_InitPeripheral(GPIO_ILL_GRP, &GPIO_InitStructure_Input_Float);

	/* Reverse detect */
	GPIO_InitStructure_Input_Float.Pin = GPIO_REVERSE_PIN;
	GPIO_InitPeripheral(GPIO_REVERSE_GRP, &GPIO_InitStructure_Input_Float);

	/* ANT control */
	GPIO_InitStructure_Input_Float.Pin = GPIO_ANT_CTRL_PIN;
	GPIO_InitPeripheral(GPIO_ANT_CTRL_GRP, &GPIO_InitStructure_Input_Float);

	/* ANT control */
	GPIO_InitStructure_Input_Float.Pin = GPIO_RADIO_ANT_CTRL_PIN;
	GPIO_InitPeripheral(GPIO_RADIO_ANT_CTRL_GRP, &GPIO_InitStructure_Input_Float);

	/* Radio reset control */
	GPIO_InitStructure_Input_Float.Pin = GPIO_RADIO_RST_PIN;
	GPIO_InitPeripheral(GPIO_RADIO_RST_GRP, &GPIO_InitStructure_Input_Float);

	/* Backlight enable */
	GPIO_InitStructure_Input_Float.Pin = GPIO_BKL_EN_PIN;
	GPIO_InitPeripheral(GPIO_BKL_EN_GRP, &GPIO_InitStructure_Input_Float);

	/* SWC key pull-up control */
	GPIO_InitStructure_Input_Float.Pin = GPIO_SWC_KEY_CTL_2K2_PIN;
	GPIO_InitPeripheral(GPIO_SWC_KEY_CTL_2K2_GRP, &GPIO_InitStructure_Input_Float);
	GPIO_InitStructure_Input_Float.Pin = GPIO_SWC_KEY_CTL_470R_PIN;
	GPIO_InitPeripheral(GPIO_SWC_KEY_CTL_470R_GRP, &GPIO_InitStructure_Input_Float);

	/* ADC */
	GPIO_InitStructure_Input_Float.Pin = GPIO_AD_BATT_DET_PIN;
	GPIO_InitPeripheral(GPIO_AD_BATT_DET_GRP, &GPIO_InitStructure_Input_Float);
	GPIO_InitStructure_Input_Float.Pin = GPIO_AD_VOL_PIN;
	GPIO_InitPeripheral(GPIO_AD_VOL_GRP, &GPIO_InitStructure_Input_Float);
	GPIO_InitStructure_Input_Float.Pin = GPIO_AD_TUNE_PIN;
	GPIO_InitPeripheral(GPIO_AD_TUNE_GRP, &GPIO_InitStructure_Input_Float);
	GPIO_InitStructure_Input_Float.Pin = GPIO_AD_PANEL_KEY_DET_1_PIN | GPIO_AD_PANEL_KEY_DET_2_PIN;
	GPIO_InitPeripheral(GPIO_AD_PANEL_KEY_DET_GRP, &GPIO_InitStructure_Input_Float);

	/* LED control */
	GPIO_InitStructure_Input_Float.Pin = GPIO_LED_B_PIN | GPIO_LED_G_PIN | GPIO_LED_R_PIN;
	GPIO_InitPeripheral(GPIO_LED_GRP, &GPIO_InitStructure_Input_Float);
	GPIO_InitStructure_Input_Float.Pin = GPIO_LED_EN_PIN;
	GPIO_InitPeripheral(GPIO_LED_GRP, &GPIO_InitStructure_Input_Float);

	/* HOST recovery key */
	GPIO_InitStructure_Input_Float.Pin = GPIO_HOST_REC_KEY_PIN;
	GPIO_InitPeripheral(GPIO_HOST_REC_KEY_GRP, &GPIO_InitStructure_Input_Float);

	/* BEEP */
	GPIO_InitStructure_Input_Float.Pin = GPIO_BEEP_PIN;
	GPIO_InitPeripheral(GPIO_BEEP_GRP, &GPIO_InitStructure_Input_Float);

	/* MUTE */
	GPIO_InitStructure_Input_Float.Pin = GPIO_MUTE_PIN;
	GPIO_InitPeripheral(GPIO_MUTE_GRP, &GPIO_InitStructure_Input_Float);

	/* IR rx */
	GPIO_InitStructure_Input_Float.Pin = GPIO_IR_RX_PIN;
	GPIO_InitPeripheral(GPIO_IR_RX_GRP, &GPIO_InitStructure_Input_Float);

	/* I2C */
	GPIO_InitStructure_Input_Float.Pin = GPIO_I2C_SCL_PIN;
	GPIO_InitPeripheral(GPIO_I2C_GRP, &GPIO_InitStructure_Input_Float);
	GPIO_InitStructure_Input_Float.Pin = GPIO_I2C_SDA_PIN;
	GPIO_InitPeripheral(GPIO_I2C_GRP, &GPIO_InitStructure_Input_Float);

	/* Uart */
	NVIC_DisableIRQ(HOST_COMM_UART_IRQ);
	USART_Enable(HOST_COMM_UART, DISABLE);
	NVIC_DisableIRQ(CAN_COMM_UART_IRQ);
	USART_Enable(CAN_COMM_UART, DISABLE);

	/* ADC */
	ADC_Enable(ADC, DISABLE);

	/* Timer */
	NVIC_DisableIRQ(TIM1_CC_IRQn);
	TIM_Enable(TIMER_LCD_BKL, DISABLE);
	TIM_Enable(TIMER_BEEP, DISABLE);
	TIM_Enable(TIMER_LED, DISABLE);

	/* Clock */
#ifndef ENABLE_SWD
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, DISABLE);
#endif

	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, DISABLE);
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOC, DISABLE);

	/* RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOF, DISABLE); ACC is in PF7 */
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_USART1, DISABLE);
	RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_USART2, DISABLE);

	RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC, DISABLE);

	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_TIM1, DISABLE);
	RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM3, DISABLE);
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_TIM8, DISABLE);

	EXTI_InitStructure.EXTI_Line = EXTI_LINE7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;


	while (1)
		{
		acc_on				= 0;
		acc_off 			= 0;

		for (; ; )
		{
			delay_us(2);

			if (Bit_RESET == GPIO_ReadInputDataBit(GPIO_ACC_GRP, GPIO_ACC_PIN))
				{
				acc_on++;
				acc_off 			= 0;
				}
			else 
				{
				acc_on				= 0;
				acc_off++;
				}

			if (acc_on > 10000)
				{
				break;
				}

			if (acc_off > 10000)
				{
				break;
				}
		}///for (; ; )

		if (acc_on > 10000)
		{
			break;
		}
		
    #if 0
		GPIO_ConfigEXTILine(GPIOF_PORT_SOURCE, GPIO_PIN_SOURCE7);
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_InitPeripheral(&EXTI_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
     
		SysTick->CTRL		= 0;

		//EXTI_ClrITPendBit(EXTI_LINE7);
		//lower the system clock
		SetSysClockToHSE();

		// sleep
		PWR_EnterSLEEPMode(SLEEP_OFF_EXIT, PWR_SLEEPENTRY_WFI);

		// recovery system clock
		SystemInit();
    #endif
		}////while (1)

	NVIC_DisableIRQ(EXTI4_15_IRQn);
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;
	EXTI_InitPeripheral(&EXTI_InitStructure);

	AUDIO_HW_MUTE;

	systick_config();

	rcu_config();
	gpio_config();
	adc_config();
	timer_config();
	uart_config();

	i2c_init();
}


int main(void)
{
	g_first_love		= 1;
	mcu_init();

#ifdef ENABLE_WWDG
	watchdog_enable();
#endif

	REAL_SYS_PWR_OFF;

	PowerInit();
	app_wd_init();

	USART_RxTx_Init(FALSE);

	VariableInit();
	delay_1ms(500);

	REAL_SYS_PWR_ON;
	BEGIN_WATCHDOG; 								//Watch Dog is enabled
	g_first_love		= 0;
	
	#if 1
	radio_set_pwr_ctrl(true);	
	radio_main();
	channel_main();
	#endif
	/////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////
	for (; ; )
	{
		CLEAR_WATCHDOG;

		if (g_mcu_in_sleep)
		{
			#ifdef ENABLE_WWDG
			watchdog_disable();
			#endif
			mcu_stay_in_sleep();
			#ifdef ENABLE_WWDG
			watchdog_enable();
			#endif
			g_mcu_in_sleep		= 0;
			
			//delay_1ms(500);not good
			radio_set_pwr_ctrl(true);	
			radio_main();
			channel_main();
		}
		/////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////////////
		if (g_trigger_4ms)
			{
			g_trigger_4ms		= 0;
			Task4msPro();
			}

		if (g_trigger_12ms)
			{
			g_trigger_12ms		= 0;
			Task12msPro();
			}

		if (g_trigger_40ms)
			{
			g_trigger_40ms		= 0;
			Task40msPro();
			}

		if (g_trigger_100ms)
			{
			g_trigger_100ms 	= 0;
			Task100msPro();
			}

		if (g_trigger_1000ms)
			{
			g_trigger_1000ms	= 0;
			Task1sPro();
			}
			
		/////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////////////
		USART_Data_Analyse();
		LinUart_Data_Analyse();
	}//	for (; ; )

}//int main(void)


