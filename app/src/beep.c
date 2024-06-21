
#include "public.h"

static void do_beep(void)
{
	GPIO_InitType GPIO_InitStructure_AFPP;
	GPIO_InitType GPIO_InitStructure_Input_Float;

	GPIO_InitStruct(&GPIO_InitStructure_Input_Float);
	GPIO_InitStructure_Input_Float.GPIO_Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure_Input_Float.GPIO_Pull = GPIO_NO_PULL;

	GPIO_InitStruct(&GPIO_InitStructure_AFPP);
	GPIO_InitStructure_AFPP.GPIO_Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStructure_AFPP.GPIO_Alternate = GPIO_AF2_TIM1;


	if (BEEP_MODE_SHORT==g_beep_info.mode) {
			if (0 == g_beep_info.timer) {
				
GPIO_InitStructure_AFPP.Pin = GPIO_BEEP_PIN;
GPIO_InitPeripheral(GPIO_BEEP_GRP, &GPIO_InitStructure_AFPP);
//				TIM_SetCmp1(TIMER_BEEP, 500);
//				TIM_ConfigInt(TIMER_BEEP, TIM_INT_CC1, ENABLE);
				//TIM_SelectOcMode(TIMER_BEEP, TIMER_CH_BEEP, TIM_OCMODE_TOGGLE);
//				TIM_EnableCapCmpCh(TIMER_BEEP, TIMER_CH_BEEP, TIM_CAP_CMP_ENABLE);
			} else if (g_beep_info.timer > T36MS_12) {
				g_beep_info.mode = BEEP_MODE_NONE;
			}
	} else {
//			TIM_SetCmp1(TIMER_BEEP, 0);
//			TIM_EnableCapCmpCh(TIMER_BEEP, TIMER_CH_BEEP, TIM_CAP_CMP_DISABLE);
//			TIM_ConfigInt(TIMER_BEEP, TIM_INT_CC1, DISABLE);
			//TIM_SelectOcMode(TIMER_BEEP, TIMER_CH_BEEP, TIM_OCMODE_INACTIVE);
			GPIO_InitStructure_Input_Float.Pin = GPIO_BEEP_PIN;
			GPIO_InitPeripheral(GPIO_BEEP_GRP, &GPIO_InitStructure_Input_Float);
			g_beep_info.state = BEEP_STATE_IDLE;
	}

	++g_beep_info.timer;
}

void beep_init(void)
{
	g_beep_info.state = BEEP_STATE_PWR_DOWN;
	g_beep_info.mode = BEEP_MODE_NONE;
	g_beep_info.timer = 0;
//	TIM_SetCmp1(TIMER_BEEP, 0);
//	TIM_ConfigInt(TIMER_BEEP, TIM_INT_CC1, DISABLE);
//	TIM_EnableCapCmpCh(TIMER_BEEP, TIMER_CH_BEEP, TIM_CAP_CMP_DISABLE);
	//TIM_SelectOcMode(TIMER_BEEP, TIMER_CH_BEEP, TIM_OCMODE_INACTIVE);

}

void beep_main(void)
{
	if (!Is_Machine_Power) {
		beep_init();
		return;
	}

	if(!g_sys_info_store.beep_onoff) {
		beep_init();
		return;
	}
	
	switch (g_beep_info.state) {
		case BEEP_STATE_PWR_DOWN:
			g_beep_info.state = BEEP_STATE_IDLE;
			break;
		case BEEP_STATE_IDLE:
			break;
		case BEEP_STATE_BEEP_ING:
			audio_set_mute(AUDIO_MUTE_DRIVER, FALSE);
			do_beep();
			break;
		default:
			break;
	}
}

void beep_short_mode(void)
{
	if (BEEP_STATE_IDLE!=g_beep_info.state) {
		return;
	}
	if (0==audio_get_volume()) {
		return;
	}
	g_beep_info.mode = BEEP_MODE_SHORT;
	g_beep_info.state = BEEP_STATE_BEEP_ING;
	g_beep_info.timer = 0;
}

