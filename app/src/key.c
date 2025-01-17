
#include "public.h"


static u8 is_ad_val_equal(u8 val, u8 cond)
{
	if(MAX(val, cond)-MIN(val, cond)<=5) {
		return 1;
	} else {
		return 0;
	}
}

static void encoder_do_key(ENCODER_KEY_INFO *pInfo, u8 keycode) {
	if ((0==pInfo->debounce_timer)||(pInfo->last_key_code==keycode)) {
		pInfo->debounce_timer= 8;	// 32ms
		PostEvent(MMI_MODULE,keycode,0);
		pInfo->last_key_code= keycode;
	} else {
		pInfo->debounce_timer= 8; // 32ms
		pInfo->last_key_code= keycode;
	}
}

static void encoder_key_reset(void)
{
	g_vol_key_info.debounce_timer = 0;
	g_vol_key_info.last_key_code = 0;
	g_vol_key_info.last_adc_value = 0;
	g_disabed_encoder = TRUE;

	g_tune_key_info.debounce_timer = 0;
	g_tune_key_info.last_key_code = 0;
	g_tune_key_info.last_adc_value = 0;

}

static void encoder_key_do_scan(ENCODER_KEY_INFO *pInfo, u8 ch)
{
	u8 ad_value;
	u8 ad0, ad1, ad2, ad3;
	u8 key_cw, key_ccw;

	if (AD_VOL_ENCODER==ch) {
		ad0 = 26;
		ad1 = 39;
		ad2 = 62;
		ad3 = 252;
		key_cw = UICC_VOLUME_UP;
		key_ccw = UICC_VOLUME_DOWN;
	} else {
		ad0 = 27;
		ad1 = 40;
		ad2 = 62;
		ad3 = 254;
		key_cw = UICC_STEP_UP;
		key_ccw = UICC_STEP_DOWN;
	}

	ad_value = adc_channel_sample(ch);
	if (is_ad_val_equal(ad_value, ad0)) {
		pInfo->last_adc_value= ad0;
	} else if (is_ad_val_equal(ad_value, ad1)) {
		if (ad0==pInfo->last_adc_value) {
			encoder_do_key(pInfo, key_cw);
		} else if (ad3==pInfo->last_adc_value) {
			encoder_do_key(pInfo, key_ccw);
		}
		pInfo->last_adc_value = ad1;
	} else if (is_ad_val_equal(ad_value, ad2)) {
		if (ad0==pInfo->last_adc_value) {
			encoder_do_key(pInfo, key_ccw);
		} else if (ad3==pInfo->last_adc_value) {
			encoder_do_key(pInfo, key_cw);
		}
		pInfo->last_adc_value = ad2;
	} else if (is_ad_val_equal(ad_value, ad3)) {
		pInfo->last_adc_value = ad3;
	} 
}

void encoder_key_main(void)
{

	if ( (Get_ACC_Det_Flag==0) || (!Is_Machine_Power) || g_disabed_encoder ) {
		encoder_key_reset();
		return;
	}

	encoder_key_do_scan(&g_vol_key_info, AD_VOL_ENCODER);
	//encoder_key_do_scan(&g_tune_key_info, AD_TUNE_ENCODER);

}

static const KEY_INFO g_def_panel_key[] = {
#if 1
	///{AD_PANEL_KEY_DET_1, 0, UICC_MUTE, UICC_FAKE_POWER_OFF, KEY_STUDY_SWC_PU_LARGE},
	///{AD_PANEL_KEY_DET_1, 35, UICC_NEXT, UICC_FASTF, 0},
	///{AD_PANEL_KEY_DET_1, 48, UICC_PREV, UICC_FASTR, 0},
	///{AD_PANEL_KEY_DET_1, 91, UICC_VOLUME_UP, 0x00, KEY_STUDY_SWC_PU_LARGE},
	///{AD_PANEL_KEY_DET_1, 110, UICC_VOLUME_DOWN, 0x00, KEY_STUDY_SWC_PU_LARGE},
	///{AD_PANEL_KEY_DET_1, 219, UICC_HOME, 0x00, KEY_STUDY_SWC_PU_LARGE},
	
  {AD_PANEL_KEY_DET_1, 0, 	UICC_MUTE, UICC_FAKE_POWER_OFF, KEY_STUDY_SWC_PU_LARGE},
  {AD_PANEL_KEY_DET_1, 91, 	UICC_VOLUME_UP, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 110, UICC_VOLUME_DOWN, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 219, UICC_HOME, 0x00, KEY_STUDY_SWC_PU_LARGE},
	
	{AD_PANEL_KEY_DET_2, 0, UICC_FAKE_POWER_OFF, UICC_FAKE_POWER_OFF, KEY_STUDY_SWC_PU_LARGE},
#else
	{AD_PANEL_KEY_DET_2, 0, UICC_FAKE_POWER_OFF, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 9, UICC_BACK, UICC_HOME, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 17, UICC_MUTE, UICC_TFT_STANDBY, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 25, UICC_RADIO, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 36, UICC_NEXT, UICC_FASTF, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 48, UICC_PREV, UICC_FASTR, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 61, UICC_RADIO_PS, UICC_AS, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 74, UICC_DVD, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 91, UICC_VOLUME_UP, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 110, UICC_VOLUME_DOWN, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 128, UICC_NAVI, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 147, UICC_ALL_APP, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 169, UICC_EQ, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 196, UICC_EJECT, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 219, UICC_HOME, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 0, UICC_MUTE, UICC_FAKE_POWER_OFF, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 9, UICC_EQ, UICC_FAKE_POWER_OFF, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 17, UICC_LOUDNESS, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 25, UICC_DIAL, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 36, UICC_ST_PROG, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 48, UICC_SOURCE, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 61, UICC_MUTE, UICC_TFT_STANDBY, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 74, UICC_NAVI, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 91, UICC_TFT_STANDBY, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 110, UICC_DVD, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 128, UICC_RADIO, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 147, UICC_TV, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 169, UICC_PLAY_PAUSE, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 196, UICC_SETUP, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 219, UICC_HANG, 0x00, KEY_STUDY_SWC_PU_LARGE},
	//{AD_PANEL_KEY_DET_1, 237, UICC_NISSAN_XTRAIL_CAM_SW, 0x00, KEY_STUDY_SWC_PU_LARGE},
#endif
};


static void do_swc_pu(KEY_STUDY_SWC_PU_TYPE type) {
	switch(type) {
		case KEY_STUDY_SWC_PU_LARGE:
			GPIO_SetBits(GPIO_SWC_KEY_CTL_2K2_GRP, GPIO_SWC_KEY_CTL_2K2_PIN);
			GPIO_SetBits(GPIO_SWC_KEY_CTL_470R_GRP, GPIO_SWC_KEY_CTL_470R_PIN);
			break;
		case KEY_STUDY_SWC_PU_SMALL:
			GPIO_ResetBits(GPIO_SWC_KEY_CTL_2K2_GRP, GPIO_SWC_KEY_CTL_2K2_PIN);
			GPIO_SetBits(GPIO_SWC_KEY_CTL_470R_GRP, GPIO_SWC_KEY_CTL_470R_PIN);
			break;
		case KEY_STUDY_SWC_PU_TINY:
			GPIO_SetBits(GPIO_SWC_KEY_CTL_2K2_GRP, GPIO_SWC_KEY_CTL_2K2_PIN);
			GPIO_ResetBits(GPIO_SWC_KEY_CTL_470R_GRP, GPIO_SWC_KEY_CTL_470R_PIN);
			break;
	}
}

static void panel_key_set_default_value(void)
{
	u8 cnt;
	u8 len;
	len = sizeof(g_def_panel_key)/sizeof(KEY_INFO);

	for (cnt=0; cnt<len; cnt++) {
		if (g_key_info_store.key_num >= MAX_PANEL_KEY_NUM) {
			break;
		}

		g_key_info_store.key[g_key_info_store.key_num].adc_channel = g_def_panel_key[cnt].adc_channel;
		g_key_info_store.key[g_key_info_store.key_num].adc_value = g_def_panel_key[cnt].adc_value;
		g_key_info_store.key[g_key_info_store.key_num].key_code_short = g_def_panel_key[cnt].key_code_short;
		g_key_info_store.key[g_key_info_store.key_num].key_code_long = g_def_panel_key[cnt].key_code_long;
		g_key_info_store.key[g_key_info_store.key_num].swc_pu_type = g_def_panel_key[cnt].swc_pu_type;

		++g_key_info_store.key_num;
	}

}

static void panel_key_reset(void)
{
	u8 cnt;
	g_key_handler.status = KEY_STATE_NONE;
	g_key_handler.last_idx = MAX_PANEL_KEY_NUM;
	g_key_handler.key_pressed_timer = 0;
	g_key_handler.debounce_timer = 0;
	g_key_handler.init_timer = 0;
	g_key_handler.recovery_key_timer = 0;
	g_key_handler.reset_key_timer = 0;

	g_key_handler.swc_adc1_idle_large = IDLE_ADC_VALUE;
	g_key_handler.swc_adc1_idle_small = IDLE_ADC_VALUE;
	g_key_handler.swc_adc1_idle_tiny = IDLE_ADC_VALUE;
	g_key_handler.swc_adc2_idle_large = IDLE_ADC_VALUE;
	g_key_handler.swc_adc2_idle_small = IDLE_ADC_VALUE;
	g_key_handler.swc_adc2_idle_tiny = IDLE_ADC_VALUE;

	g_key_handler.study_pressed_timer = 0;
	g_key_handler.study_swc_pu_type = KEY_STUDY_SWC_PU_LARGE;
	do_swc_pu(g_key_handler.study_swc_pu_type);

	g_key_info_store.magic_num0 = 'K';
	g_key_info_store.magic_num1 = 'E';
	g_key_info_store.magic_num2 = 'Y';
	g_key_info_store.key_num = 0;
	for (cnt=0; cnt<MAX_PANEL_KEY_NUM; cnt++) {
		g_key_info_store.key[cnt].adc_channel = 0;
		g_key_info_store.key[cnt].adc_value = IDLE_ADC_VALUE;
		g_key_info_store.key[cnt].key_code_short = NO_KEY;
		g_key_info_store.key[cnt].key_code_long = NO_KEY;
		g_key_info_store.key[cnt].swc_pu_type = KEY_STUDY_SWC_PU_LARGE;
	}
}

static u8 panel_key_get_adc_idle_value(u8 adc_ch, KEY_STUDY_SWC_PU_TYPE pu)
{
	switch (adc_ch) {
		case AD_CAR_SWC_DET_1:
			switch (pu) {
				case KEY_STUDY_SWC_PU_LARGE:
					return g_key_handler.swc_adc1_idle_large;
				case KEY_STUDY_SWC_PU_SMALL:
					return g_key_handler.swc_adc1_idle_small;
				case KEY_STUDY_SWC_PU_TINY:
					return g_key_handler.swc_adc1_idle_tiny;
			}
			break;
		case AD_CAR_SWC_DET_2:
			switch (pu) {
				case KEY_STUDY_SWC_PU_LARGE:
					return g_key_handler.swc_adc2_idle_large;
				case KEY_STUDY_SWC_PU_SMALL:
					return g_key_handler.swc_adc2_idle_small;
				case KEY_STUDY_SWC_PU_TINY:
					return g_key_handler.swc_adc2_idle_tiny;
			}
			break;
		case AD_PANEL_KEY_DET_1:
		case AD_PANEL_KEY_DET_2:
		default:
			return IDLE_ADC_VALUE;
	}
	return IDLE_ADC_VALUE;
}

static void panel_key_clear(KEY_STUDY_DEVS dev)
{
#if 1
	panel_key_reset();
	panel_key_set_default_value();
#else
	s8 cnt, tmp;
	
	g_key_handler.status = KEY_STATE_NONE;
	g_key_handler.last_idx = MAX_PANEL_KEY_NUM;
	g_key_handler.key_pressed_timer = 0;
	g_key_handler.study_pressed_timer = 0;
	g_key_handler.debounce_timer = 0;

	for (cnt=0; cnt<g_key_info_store.key_num; cnt++) {
		if (KEY_STUDY_PANEL == dev) {
			if (AD_PANEL_KEY_DET_1 != g_key_info_store.key[cnt].adc_channel) {
				continue;
			}
		} else if (KEY_STUDY_SWC == dev) {
			if ( (AD_CAR_SWC_DET_1 != g_key_info_store.key[cnt].adc_channel) &&
				(AD_CAR_SWC_DET_2 != g_key_info_store.key[cnt].adc_channel) ) {
				continue;
			}
		} else {
			continue;
		}

		// delete current key, by forward the next one
		--g_key_info_store.key_num;
		for (tmp=cnt; tmp<g_key_info_store.key_num; tmp++) {
			g_key_info_store.key[tmp].adc_channel = g_key_info_store.key[tmp+1].adc_channel;
			g_key_info_store.key[tmp].adc_value = g_key_info_store.key[tmp+1].adc_value;
			g_key_info_store.key[tmp].key_code_short = g_key_info_store.key[tmp+1].key_code_short;
			g_key_info_store.key[tmp].key_code_long = g_key_info_store.key[tmp+1].key_code_long;
		}
		--cnt;	// check this position next loop
	}

	// clear the rest empty key info
	for (cnt=g_key_info_store.key_num; cnt<MAX_PANEL_KEY_NUM; cnt++) {
		g_key_info_store.key[cnt].adc_channel = 0;
		g_key_info_store.key[cnt].adc_value = IDLE_ADC_VALUE;
		g_key_info_store.key[cnt].key_code_short = NO_KEY;
		g_key_info_store.key[cnt].key_code_long = NO_KEY;
	}

	if (KEY_STUDY_PANEL == dev) {
		panel_key_set_default_value();
	}
#endif
}

static void panel_key_load_keymap(void)
{
	bool do_recovery;

	do_recovery = FALSE;
	
	ak_memcpy((uint8_t *)&g_key_info_store, (uint8_t *)FLASH_ADDR_KEY_STORE, sizeof(KEY_INFO_STORE));

	if ( ('K' != g_key_info_store.magic_num0) ||
		('E' != g_key_info_store.magic_num1) ||
		('Y' != g_key_info_store.magic_num2) ) {
		do_recovery = TRUE;
	}

	if (g_key_info_store.key_num>MAX_PANEL_KEY_NUM) {
		do_recovery = TRUE;
	}

	if (do_recovery) {
		panel_key_reset();
		panel_key_set_default_value();
	} else {
	}
}

static void panel_key_debounce(void)
{
	g_key_handler.debounce_timer++;
	if (T200MS_12<=g_key_handler.debounce_timer) {
		g_key_handler.last_idx = MAX_PANEL_KEY_NUM;
		g_key_handler.key_pressed_timer = 0;
		g_key_handler.debounce_timer = 0;
	}
}

static void panel_key_do_send_key(u8 index, bool long_press)
{
	if (long_press) {
		if (NO_KEY != g_key_info_store.key[index].key_code_long) {
			PostEvent(MMI_MODULE,g_key_info_store.key[index].key_code_long, WORD(1, index));
		}
	} else {
		if (g_fake_pwr_off && (UICC_FAKE_POWER_OFF == g_key_info_store.key[index].key_code_long)) {
			PostEvent(MMI_MODULE,g_key_info_store.key[index].key_code_long,0);
		} else if (NO_KEY != g_key_info_store.key[index].key_code_short) {
			PostEvent(MMI_MODULE,g_key_info_store.key[index].key_code_short, WORD(1, index));
		}
	}
}

static void panel_key_do_scan(u8 adc_ch)
{
	u8 adc_value;
	u8 index;
	u8 adc_idle_value;

	adc_value = adc_channel_sample(adc_ch);

	adc_idle_value = panel_key_get_adc_idle_value(adc_ch, g_key_handler.study_swc_pu_type);
	
	if (is_ad_val_equal(adc_value, adc_idle_value)) 
	{ 	// no key pressed
			// no key pressed
		if((MAX_PANEL_KEY_NUM != g_key_handler.last_idx) &&	(adc_ch==g_key_info_store.key[g_key_handler.last_idx].adc_channel)) 
			{
				// key release
				if ((KEY_LONG_PRESS_TIME > g_key_handler.key_pressed_timer) && (KEY_SHORT_PRESS_TIME < g_key_handler.key_pressed_timer)) 
					{
						panel_key_do_send_key(g_key_handler.last_idx, FALSE);
					}
				g_key_handler.last_idx = MAX_PANEL_KEY_NUM;
			}
			return;
	}

	for (index=0; index<g_key_info_store.key_num; index++) {
		if (adc_ch != g_key_info_store.key[index].adc_channel) {
			continue;
		}

		if(!is_ad_val_equal(adc_value, g_key_info_store.key[index].adc_value)) {
			continue;
		}

		if ( ((AD_CAR_SWC_DET_1==adc_ch)||(AD_CAR_SWC_DET_2==adc_ch)) &&
			(g_key_handler.study_swc_pu_type != g_key_info_store.key[index].swc_pu_type) ) {
			continue;
		}
		break;
	}
    
	if (index>=g_key_info_store.key_num) {
		// no key detect
		return;
	}

	if (g_key_handler.last_idx != index) {
		g_key_handler.last_idx = index;
		g_key_handler.key_pressed_timer = 0;
	}
    
	g_key_handler.debounce_timer = 0;

	++g_key_handler.key_pressed_timer;

	// do repeat press for volume control
	if ((UICC_VOLUME_DOWN==g_key_info_store.key[index].key_code_short) || (UICC_VOLUME_UP==g_key_info_store.key[index].key_code_short)) 
	{
		if (0==(g_key_handler.key_pressed_timer % KEY_REPEAT_PRESS_TIME)) {
			panel_key_do_send_key(index, FALSE);
		}
	} 
	else
	{
		// do long press
		if (KEY_LONG_PRESS_TIME==g_key_handler.key_pressed_timer) {
			if (NO_KEY != g_key_info_store.key[index].key_code_long) {
				panel_key_do_send_key(index, TRUE);
			} else {
				panel_key_do_send_key(index, FALSE);
			}
		}
	}

	// handle timer, make sure not to overflow
	if ((100*KEY_REPEAT_PRESS_TIME)==g_key_handler.key_pressed_timer) {
		g_key_handler.key_pressed_timer -= KEY_REPEAT_PRESS_TIME;
	}

}

static void panel_key_do_study_det(u8 adc_ch)
{
	u8 adc_value;
	u8 adc_idle_value;
	u8 index;
	u16 temp;

	adc_value = adc_channel_sample(adc_ch);

	adc_idle_value = panel_key_get_adc_idle_value(adc_ch, g_key_handler.study_swc_pu_type);

	if (is_ad_val_equal(adc_value, adc_idle_value)) { // no key pressed
		if ( adc_ch==g_key_handler.study_adc_ch ) { // key release
			if (g_key_handler.study_pressed_timer<KEY_SHORT_PRESS_TIME) {
				g_key_handler.study_adc_ch = 0;
				g_key_handler.study_adc_value = IDLE_ADC_VALUE;
				return;
			}
			
			for (index=0; index<g_key_info_store.key_num; index++) {
				if (g_key_handler.study_adc_ch != g_key_info_store.key[index].adc_channel) {
					continue;
				}
				if(!is_ad_val_equal(g_key_handler.study_adc_value, g_key_info_store.key[index].adc_value)) {
					continue;
				}
				if ( (KEY_STUDY_SWC == g_key_handler.key_study_dev) &&
					(g_key_handler.study_swc_pu_type != g_key_info_store.key[index].swc_pu_type) ) {
					continue;
				}
				break;
			}

			if (MAX_PANEL_KEY_NUM<=index) {
				index = 0;
			}

			if (index < g_key_info_store.key_num) {
				// this key already study
				if (KEY_STUDY_ALREADY_EXIST == g_key_handler.study_status) {
					if (g_key_handler.study_index == index) {
						// user already confirmed to overwrite it
						g_key_handler.study_status = KEY_STUDY_BEGIN;
					} else {
						// user press other key
						g_key_handler.study_status = KEY_STUDY_ALREADY_EXIST;
					}
				} else {
					// mark it
					g_key_handler.study_status = KEY_STUDY_ALREADY_EXIST;
				}
			} else {
				g_key_handler.study_status = KEY_STUDY_BEGIN;
			}

			if (KEY_STUDY_BEGIN == g_key_handler.study_status) {
				g_key_handler.status = KEY_STATE_STUDY_WAITING_KEY_CODE;
			} else if (KEY_STUDY_ALREADY_EXIST == g_key_handler.study_status) {
				g_key_handler.study_adc_ch = 0;
				g_key_handler.study_adc_value = IDLE_ADC_VALUE;
				g_key_handler.study_swc_pu_type = KEY_STUDY_SWC_PU_LARGE;
				do_swc_pu(g_key_handler.study_swc_pu_type);
			}
			
			g_key_handler.study_index = index;
			g_key_handler.study_pressed_timer = 0;
			// notify HOST
			if (KEY_STUDY_PANEL == g_key_handler.key_study_dev) {
				PostEvent(WINCE_MODULE, TX_TO_GUI_PANEL_CUR_KEY_INFO, index);				
			} else if (KEY_STUDY_SWC == g_key_handler.key_study_dev) {
				PostEvent(WINCE_MODULE, TX_TO_GUI_SWC_CUR_KEY_INFO, index);				
			}
		}
		return;
	}

	g_key_handler.debounce_timer = 0;
	g_key_handler.study_adc_ch = adc_ch;

	if ( (KEY_STUDY_SWC == g_key_handler.key_study_dev) &&
		(KEY_STUDY_SWC_PU_TINY != g_key_handler.study_swc_pu_type) && 
		(adc_value < SWC_ADC_RETRY_VALUE) ) {
		g_key_handler.study_adc_value = adc_value;
		g_key_handler.study_pressed_timer = 0;
		g_key_handler.study_swc_pu_type = (KEY_STUDY_SWC_PU_TYPE)toggle_swc_pu(g_key_handler.study_swc_pu_type);
		do_swc_pu(g_key_handler.study_swc_pu_type);
	} else if (IDLE_ADC_VALUE==g_key_handler.study_adc_value) {
		g_key_handler.study_adc_value = adc_value;
		g_key_handler.study_pressed_timer = 0;
	} else if (adc_idle_value==g_key_handler.study_adc_value) {
		g_key_handler.study_adc_value = adc_value;
		g_key_handler.study_pressed_timer = 0;
	} else  if (!is_ad_val_equal(adc_value,g_key_handler.study_adc_value)) {
		g_key_handler.study_adc_value = adc_value;
		g_key_handler.study_pressed_timer = 0;
	} else {
		temp = g_key_handler.study_adc_value;
		temp += adc_value;
		g_key_handler.study_adc_value = (u8)(temp>>1);
		++g_key_handler.study_pressed_timer;
	}
}

static bool panel_key_swc_adc_idle_value_setup(void)
{
	u8 adc_value;
	u16 temp;
	
	++g_key_handler.init_timer;

	if (g_key_handler.init_timer > T1S5_12) {
		// finish setup, return true;
		g_key_handler.study_swc_pu_type = KEY_STUDY_SWC_PU_LARGE;
		do_swc_pu(g_key_handler.study_swc_pu_type);
		return TRUE;
	} else if (g_key_handler.init_timer > T1S_12) {
		// setup the idle adc value with tiny pull-up
		g_key_handler.study_swc_pu_type = KEY_STUDY_SWC_PU_TINY;
		do_swc_pu(g_key_handler.study_swc_pu_type);

		adc_value = adc_channel_sample(AD_CAR_SWC_DET_1);
		if (!is_ad_val_equal(adc_value, g_key_handler.swc_adc1_idle_tiny)) {
			g_key_handler.swc_adc1_idle_tiny = adc_value;
		} else {
			temp = g_key_handler.swc_adc1_idle_tiny;
			temp += adc_value;
			g_key_handler.swc_adc1_idle_tiny = (u8)(temp>>1);
		}
		adc_value = adc_channel_sample(AD_CAR_SWC_DET_2);
		if (!is_ad_val_equal(adc_value, g_key_handler.swc_adc2_idle_tiny)) {
			g_key_handler.swc_adc2_idle_tiny = adc_value;
		} else {
			temp = g_key_handler.swc_adc2_idle_tiny;
			temp += adc_value;
			g_key_handler.swc_adc2_idle_tiny = (u8)(temp>>1);
		}
	} else if (g_key_handler.init_timer > T500MS_12) {
		// setup the idle adc value with small pull-up
		g_key_handler.study_swc_pu_type = KEY_STUDY_SWC_PU_SMALL;
		do_swc_pu(g_key_handler.study_swc_pu_type);

		adc_value = adc_channel_sample(AD_CAR_SWC_DET_1);
		if (adc_value < (SWC_ADC_RETRY_VALUE-10)) {
			adc_value = IDLE_ADC_VALUE;
		}
		if (!is_ad_val_equal(adc_value, g_key_handler.swc_adc1_idle_small)) {
			g_key_handler.swc_adc1_idle_small = adc_value;
		} else {
			temp = g_key_handler.swc_adc1_idle_small;
			temp += adc_value;
			g_key_handler.swc_adc1_idle_small = (u8)(temp>>1);
		}
		adc_value = adc_channel_sample(AD_CAR_SWC_DET_2);
		if (adc_value < (SWC_ADC_RETRY_VALUE-10)) {
			adc_value = IDLE_ADC_VALUE;
		}
		if (!is_ad_val_equal(adc_value, g_key_handler.swc_adc2_idle_small)) {
			g_key_handler.swc_adc2_idle_small = adc_value;
		} else {
			temp = g_key_handler.swc_adc2_idle_small;
			temp += adc_value;
			g_key_handler.swc_adc2_idle_small = (u8)(temp>>1);
		}
	} else {
		// setup the idle adc value with large pull-up
		g_key_handler.study_swc_pu_type = KEY_STUDY_SWC_PU_LARGE;
		do_swc_pu(g_key_handler.study_swc_pu_type);

		adc_value = adc_channel_sample(AD_CAR_SWC_DET_1);
		if (adc_value < (SWC_ADC_RETRY_VALUE-10)) {
			adc_value = IDLE_ADC_VALUE;
		}
		if (!is_ad_val_equal(adc_value, g_key_handler.swc_adc1_idle_large)) {
			g_key_handler.swc_adc1_idle_large = adc_value;
		} else {
			temp = g_key_handler.swc_adc1_idle_large;
			temp += adc_value;
			g_key_handler.swc_adc1_idle_large = (u8)(temp>>1);
		}
		adc_value = adc_channel_sample(AD_CAR_SWC_DET_2);
		if (adc_value < (SWC_ADC_RETRY_VALUE-10)) {
			adc_value = IDLE_ADC_VALUE;
		}
		if (!is_ad_val_equal(adc_value, g_key_handler.swc_adc2_idle_large)) {
			g_key_handler.swc_adc2_idle_large = adc_value;
		} else {
			temp = g_key_handler.swc_adc2_idle_large;
			temp += adc_value;
			g_key_handler.swc_adc2_idle_large = (u8)(temp>>1);
		}
	}

	// I wil be back!
	return FALSE;
}

static void panel_key_recovery_handler(void)
{
	if (g_key_handler.recovery_key_timer < T3S_12) {
		++g_key_handler.recovery_key_timer;
		if ( (0==g_key_handler.key_pressed_timer) 
			|| (MAX_PANEL_KEY_NUM == g_key_handler.last_idx) 
			|| (NO_KEY == g_key_info_store.key[g_key_handler.last_idx].key_code_short) ) {
			GPIO_SetBits(GPIO_HOST_REC_KEY_GRP, GPIO_HOST_REC_KEY_PIN);
		} else if ( (UICC_FAKE_POWER_OFF == g_key_info_store.key[g_key_handler.last_idx].key_code_short) ||
				(UICC_FAKE_POWER_OFF == g_key_info_store.key[g_key_handler.last_idx].key_code_long) ) {
			GPIO_ResetBits(GPIO_HOST_REC_KEY_GRP, GPIO_HOST_REC_KEY_PIN);
		}
	} else if (T3S_12 == g_key_handler.recovery_key_timer) {
		GPIO_SetBits(GPIO_HOST_REC_KEY_GRP, GPIO_HOST_REC_KEY_PIN);
	}
}

#define KEY_RESET_TIMEOUT	(T10S_12)
static void panel_key_reset_handler(void)
{
	if (0==g_key_handler.key_pressed_timer) {
		g_key_handler.reset_key_timer = 0;
	} else if (MAX_PANEL_KEY_NUM == g_key_handler.last_idx) {
		if (g_key_handler.reset_key_timer > KEY_RESET_TIMEOUT) {
			AUDIO_HW_MUTE;
			REAL_SYS_PWR_OFF;
			delay_1ms(1000);
			NVIC_SystemReset();	// force reset 
		}
		g_key_handler.reset_key_timer = 0;
	} else if (NO_KEY == g_key_info_store.key[g_key_handler.last_idx].key_code_short) {
		g_key_handler.reset_key_timer = 0;
	} else if ( (UICC_FAKE_POWER_OFF == g_key_info_store.key[g_key_handler.last_idx].key_code_short) ||
			(UICC_FAKE_POWER_OFF == g_key_info_store.key[g_key_handler.last_idx].key_code_long) ) {
		if (g_key_handler.reset_key_timer<65500)
			++g_key_handler.reset_key_timer;
	}
	
	if (KEY_RESET_TIMEOUT == g_key_handler.reset_key_timer) {
		g_led_info.notify_reset = TRUE;
	}
}

static void panel_key_state_handler(void)
{
	switch (g_key_handler.status) {
		case KEY_STATE_NONE:
			g_key_handler.status = KEY_STATE_INIT;
			break;
		case KEY_STATE_INIT:
				panel_key_load_keymap();
				g_key_handler.init_timer = 0;
				g_key_handler.status = KEY_STATE_INIT_2;
			break;
		case KEY_STATE_INIT_2:
			// detect panel key now. maybe we need to make core board enter recovery mode
			panel_key_do_scan(AD_PANEL_KEY_DET_1);
			panel_key_do_scan(AD_PANEL_KEY_DET_2);

			if (panel_key_swc_adc_idle_value_setup()) {
				// finish setup, jump to next stage
				g_key_handler.status = KEY_STATE_WORKING;
			}
			break;
		case KEY_STATE_WORKING:
			panel_key_debounce();
			panel_key_do_scan(AD_PANEL_KEY_DET_1);
			panel_key_do_scan(AD_PANEL_KEY_DET_2);
			panel_key_do_scan(AD_CAR_SWC_DET_1);
			panel_key_do_scan(AD_CAR_SWC_DET_2);
			if (0==g_key_handler.key_pressed_timer) {
				g_key_handler.study_swc_pu_type = (KEY_STUDY_SWC_PU_TYPE)toggle_swc_pu(g_key_handler.study_swc_pu_type);
				do_swc_pu(g_key_handler.study_swc_pu_type);
			}
			break;
		case KEY_STATE_ENTER_STUDY:
			if (KEY_STUDY_PANEL == g_key_handler.key_study_dev) {
				PostEvent(WINCE_MODULE, TX_TO_GUI_PANEL_MODE_INFO, 0x00);
			} else if (KEY_STUDY_SWC == g_key_handler.key_study_dev) {
				PostEvent(WINCE_MODULE, TX_TO_GUI_SWC_MODE_INFO, 0x00);
				g_key_handler.study_swc_pu_type = KEY_STUDY_SWC_PU_LARGE;
				do_swc_pu(g_key_handler.study_swc_pu_type);
			}
			g_key_handler.study_index = MAX_PANEL_KEY_NUM;
			g_key_handler.study_adc_ch = 0;
			g_key_handler.study_adc_value = IDLE_ADC_VALUE;
			g_key_handler.study_status = KEY_STUDY_FINISH;
			g_key_handler.status = KEY_STATE_STUDY_WAITING_KEY_PRESSED;
			break;
		case KEY_STATE_STUDY_WAITING_KEY_PRESSED:
			if (KEY_STUDY_PANEL == g_key_handler.key_study_dev) {
				panel_key_do_study_det(AD_PANEL_KEY_DET_1);
				panel_key_do_study_det(AD_PANEL_KEY_DET_2);
			} else if (KEY_STUDY_SWC == g_key_handler.key_study_dev) {
				panel_key_do_study_det(AD_CAR_SWC_DET_1);
				panel_key_do_study_det(AD_CAR_SWC_DET_2);
//				panel_key_do_scan(AD_PANEL_KEY_DET_1);
			}
			break;
		case KEY_STATE_STUDY_WAITING_KEY_CODE:
//			if (KEY_STUDY_SWC == g_key_handler.key_study_dev) {
//				panel_key_do_scan(AD_PANEL_KEY_DET_1);
//			}
			break;
		case KEY_STATE_EXIT_STUDY:
			if (KEY_STUDY_PANEL == g_key_handler.key_study_dev) {
				PostEvent(WINCE_MODULE, TX_TO_GUI_PANEL_MODE_INFO, 0x02);
			} else if (KEY_STUDY_SWC == g_key_handler.key_study_dev) {
				PostEvent(WINCE_MODULE, TX_TO_GUI_SWC_MODE_INFO, 0x02);
				g_key_handler.study_swc_pu_type = KEY_STUDY_SWC_PU_LARGE;
				do_swc_pu(g_key_handler.study_swc_pu_type);
			}
			g_key_handler.status = KEY_STATE_WORKING;
			break;
	}
}

static void panel_key_event_handler(void)
{
	EVENT *key_evt;
	key_evt = GetEvent(PANEL_MODULE);
	if(key_evt->ID == KEY_EVT_NONE)
	{
		return;
	}
	
	switch(key_evt->ID) {
		case KEY_EVT_ENTER_STUDY:
			if ( (KEY_STUDY_PANEL==key_evt->prm) ||
				(KEY_STUDY_SWC==key_evt->prm) ) {
				g_key_handler.key_study_dev = (KEY_STUDY_DEVS)(key_evt->prm);
				g_key_handler.status = KEY_STATE_ENTER_STUDY;
			}
			break;
		case KEY_EVT_EXIT_STUDY:
			if ( (KEY_STATE_WORKING!=g_key_handler.status) && 
				(KEY_STATE_EXIT_STUDY!=g_key_handler.status) ) {
				if (1 == key_evt->prm) {
					ak_flash_save_info();
					g_key_handler.status = KEY_STATE_EXIT_STUDY;
				} else if (2 == key_evt->prm) {
					panel_key_load_keymap();
					g_key_handler.status = KEY_STATE_EXIT_STUDY;
				}
			}			
			break;
		case KEY_EVT_QUERY_KEY_CODES:
			if (KEY_STUDY_PANEL==key_evt->prm) {
				PostEvent(WINCE_MODULE, TX_TO_GUI_PANEL_KEY_CODES, NONE);
			} else if (KEY_STUDY_SWC==key_evt->prm) {
				PostEvent(WINCE_MODULE, TX_TO_GUI_SWC_KEY_CODES, NONE);
			}
			break;
		case KEY_EVT_STUDY_KEY_CODE:
			if (KEY_STATE_STUDY_WAITING_KEY_CODE == g_key_handler.status) {
				g_key_info_store.key[g_key_handler.study_index].adc_channel = g_key_handler.study_adc_ch;
				g_key_info_store.key[g_key_handler.study_index].adc_value = g_key_handler.study_adc_value;
				g_key_info_store.key[g_key_handler.study_index].key_code_short = LSB(key_evt->prm);
				g_key_info_store.key[g_key_handler.study_index].key_code_long = MSB(key_evt->prm);
				g_key_info_store.key[g_key_handler.study_index].swc_pu_type = g_key_handler.study_swc_pu_type;
				if (g_key_handler.study_index >= g_key_info_store.key_num) {
					g_key_info_store.key_num++;
					if (g_key_info_store.key_num>MAX_PANEL_KEY_NUM) {
						// we should never reach here.
						g_key_info_store.key_num=MAX_PANEL_KEY_NUM;
					}
				}
				g_key_handler.study_status = KEY_STUDY_FINISH;
				if (KEY_STUDY_PANEL == g_key_handler.key_study_dev) {
					PostEvent(WINCE_MODULE, TX_TO_GUI_PANEL_CUR_KEY_INFO, g_key_handler.study_index);
				} else if (KEY_STUDY_SWC == g_key_handler.key_study_dev) {
					PostEvent(WINCE_MODULE, TX_TO_GUI_SWC_CUR_KEY_INFO, g_key_handler.study_index);
				}
				g_key_handler.study_index = MAX_PANEL_KEY_NUM;
				g_key_handler.study_adc_ch = 0;
				g_key_handler.study_adc_value = IDLE_ADC_VALUE;
				g_key_handler.status = KEY_STATE_STUDY_WAITING_KEY_PRESSED;
				g_key_handler.study_swc_pu_type = KEY_STUDY_SWC_PU_LARGE;
				do_swc_pu(g_key_handler.study_swc_pu_type);
			}
			break;
		case KEY_EVT_CLEAR_KEYS:
			if ( (KEY_STATE_WORKING!=g_key_handler.status) && 
				(KEY_STATE_EXIT_STUDY!=g_key_handler.status) ) {
				panel_key_clear((KEY_STUDY_DEVS)(key_evt->prm));
				g_key_handler.study_status = KEY_STUDY_FINISH;
				g_key_handler.study_index = MAX_PANEL_KEY_NUM;
				g_key_handler.study_adc_ch = 0;
				g_key_handler.study_adc_value = IDLE_ADC_VALUE;
				g_key_handler.status = KEY_STATE_STUDY_WAITING_KEY_PRESSED;
			}
			break;
		default:
			break;
	}	
}

u8 panel_key_get_all_key_codes(KEY_STUDY_DEVS dev, u8 *pbuf)
{
	u8 len = 0;
	u8 index;
	
	for (index=0; index<g_key_info_store.key_num; index++) {
		// check the key devices type, according the adc channel
		if (KEY_STUDY_PANEL == dev) {
			if ( (AD_PANEL_KEY_DET_1 != g_key_info_store.key[index].adc_channel) &&
					(AD_PANEL_KEY_DET_2 != g_key_info_store.key[index].adc_channel) ) {
				continue;
			}
		} else if (KEY_STUDY_SWC == dev) {
			if ( (AD_CAR_SWC_DET_1 != g_key_info_store.key[index].adc_channel) &&
				(AD_CAR_SWC_DET_2 != g_key_info_store.key[index].adc_channel) ) {
				continue;
			}
		} else {
			continue;
		}

		pbuf[len++] = g_key_info_store.key[index].key_code_short;
		pbuf[len++] = g_key_info_store.key[index].key_code_long;
	}
	return len;
}

u8 panel_key_get_cur_key_info(KEY_STUDY_DEVS dev, u8 index, u8 *pbuf)
{
	u8 key_nums = 0;
	u8 cnt;

	// find out how much keys of this type(PANEL/SWC) before me
	for (cnt=0; cnt<index; cnt++) {
		// check the key devices type, according the adc channel
		if (KEY_STUDY_PANEL == dev) {
			if ( (AD_PANEL_KEY_DET_1 != g_key_info_store.key[cnt].adc_channel) &&
				(AD_PANEL_KEY_DET_2 != g_key_info_store.key[cnt].adc_channel) ) {
				continue;
			}
		} else if (KEY_STUDY_SWC == dev) {
			if ( (AD_CAR_SWC_DET_1 != g_key_info_store.key[cnt].adc_channel) &&
				(AD_CAR_SWC_DET_2 != g_key_info_store.key[cnt].adc_channel) ) {
				continue;
			}
		} else {
			continue;
		}
		key_nums++;
	}

	pbuf[0] = key_nums;
	pbuf[1] = g_key_handler.study_status;
	pbuf[2] = g_key_info_store.key[index].key_code_short;
	pbuf[3] = g_key_info_store.key[index].key_code_long;
	return 4;
}

#if 0
static void panel_key_fixup_keycode(u8 adc_ch, u8 adc_val, u8 keycode_short, u8 keycode_long)
{
	u8 cnt;

	// try to find the same key and fix it
	for (cnt=0; cnt<g_key_info_store.key_num; cnt++) {
		if (adc_ch == g_key_info_store.key[cnt].adc_channel) {
			if (is_ad_val_equal(g_key_info_store.key[cnt].adc_value, adc_val)) {
				g_key_info_store.key[cnt].key_code_short = keycode_short;
				g_key_info_store.key[cnt].key_code_long = keycode_long;
				break;
			}
		}
	}

	// try to add new key
	if (cnt>=g_key_info_store.key_num) {
		for (; cnt<MAX_PANEL_KEY_NUM; cnt++) {
			if ( (0==g_key_info_store.key[cnt].adc_channel) &&
				(IDLE_ADC_VALUE==g_key_info_store.key[cnt].adc_value) ) {
				g_key_info_store.key[cnt].adc_channel = adc_ch;
				g_key_info_store.key[cnt].adc_value = adc_val;
				g_key_info_store.key[cnt].key_code_short = keycode_short;
				g_key_info_store.key[cnt].key_code_long = keycode_long;
				++g_key_info_store.key_num;
				break;
			}
		}
	}
}

static void panel_swckey_fixup_keycode(u8 adc_ch, u8 adc_val, KEY_STUDY_SWC_PU_TYPE pu_type, u8 keycode_short, u8 keycode_long)
{
	u8 cnt;

	// try to find the same key and fix it
	for (cnt=0; cnt<g_key_info_store.key_num; cnt++) {
		if (pu_type != g_key_info_store.key[cnt].swc_pu_type) {
			continue;
		}
		if (adc_ch != g_key_info_store.key[cnt].adc_channel) {
			continue;
		}
		if (is_ad_val_equal(g_key_info_store.key[cnt].adc_value, adc_val)) {
			g_key_info_store.key[cnt].key_code_short = keycode_short;
			g_key_info_store.key[cnt].key_code_long = keycode_long;
			break;
		}
	}

	// try to add new key
	if (cnt>=g_key_info_store.key_num) {
		for (; cnt<MAX_PANEL_KEY_NUM; cnt++) {
			if ( (0==g_key_info_store.key[cnt].adc_channel) &&
				(IDLE_ADC_VALUE==g_key_info_store.key[cnt].adc_value) ) {
				g_key_info_store.key[cnt].adc_channel = adc_ch;
				g_key_info_store.key[cnt].adc_value = adc_val;
				g_key_info_store.key[cnt].swc_pu_type = pu_type;
				g_key_info_store.key[cnt].key_code_short = keycode_short;
				g_key_info_store.key[cnt].key_code_long = keycode_long;
				++g_key_info_store.key_num;
				break;
			}
		}
	}
}
#endif


void panel_key_init(void)
{
}

void panel_key_main(void)
{
	if(Get_ACC_Det_Flag==0) {
		panel_key_reset();
		GPIO_SetBits(GPIO_HOST_REC_KEY_GRP, GPIO_HOST_REC_KEY_PIN);
		return;
	}

	panel_key_state_handler();

	if ( (KEY_STATE_NONE!=g_key_handler.status)
		&& (KEY_STATE_INIT!=g_key_handler.status) 
		&& (KEY_STATE_INIT_2!=g_key_handler.status) ) {
		panel_key_event_handler();
	}

	panel_key_recovery_handler();
	panel_key_reset_handler();
}

