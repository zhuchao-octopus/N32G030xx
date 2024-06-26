
#include "public.h"


void SwitchSource(SOURCE TargetSRC)
{
	if(TargetSRC > NUM_OF_SOURCE)
		TargetSRC = NUM_OF_SOURCE;
	if(TargetSRC==SOURCE_NAVI || TargetSRC==SOURCE_CAMERA||TargetSRC==SOURCE_FRONT_AUX)
	{
	}
	else
	{
		PostEvent(MAIN_MODULE,EVT_SRC_FRONT_REAR_CHG,TargetSRC);
	}
}

void CtrlRadio(u8 keycode)
{

	if(FrontSource != SOURCE_TUNER) 
		return;

	switch(keycode)
	{
		case UICC_SMART_CCW:
		case UICC_SMART_CW:
			if(g_radio_info.flag.field.F_RADIO_SEEK) {
				PostEvent(TUNER_MODULE,RADIO_EVT_STOP_SEEK, NONE);
				break;
			} else {
				PostEvent(TUNER_MODULE, RADIO_EVT_SEEK, 
					(UICC_SMART_CCW==keycode)?6:5);
			}
			break;
		default:
			break;
	}
}


/******************************************************************************
  Function:       MMI
  Description:   以40ms时基在主循环中调用，定时去取MMI_MODULE
                 中的消息，解析出按键消息的ID和参数做相应的处理。 
  Input:    无     
  Output:        
  Return:      无
  Others:  customer模块主函数       
******************************************************************************/

void MMI(void)	//4// 40ms时基
{
	u8 keycode;
	EVENT *pEvt;
	pEvt=GetEvent(MMI_MODULE);
	keycode=pEvt->ID;

	if(keycode==NO_EVT)
	{
		return;
	}

	CtrlRadio(keycode);

	if(!Is_Machine_Power)
		return;

	if (g_fake_pwr_off) {
		if ( (UICC_FAKE_POWER_OFF != keycode) &&
			(UICC_VOLUME_UP != keycode) &&
			(UICC_VOLUME_DOWN != keycode) &&
			(UICC_BT != keycode) &&
			(UICC_DIAL != keycode) &&
			(UICC_HANG != keycode ) &&
			(UICC_BACK != keycode) ) {
			return;
		}
	}

	beep_short_mode();

	switch (keycode)
	{	
 		case UICC_VOLUME_DOWN:
			audio_volume_down();
			PostEvent(WINCE_MODULE, TX_TO_GUI_AUDIO_VOLUME_INFO, NONE);
			break;  
		case UICC_VOLUME_UP:
			audio_volume_up();
			PostEvent(WINCE_MODULE, TX_TO_GUI_AUDIO_VOLUME_INFO, NONE);
			break;
		case UICC_MUTE:
			if (g_audio_info.mute & AUDIO_MUTE_USER) {
				audio_set_mute(AUDIO_MUTE_USER, FALSE);
			} else {
				audio_set_mute(AUDIO_MUTE_USER, TRUE);
			}
			PostEvent(WINCE_MODULE,TX_TO_GUI_SHORT_KEY_EVENT, WORD(0xFF, keycode));
			break;
		default:
			if (pEvt->prm != 0) {
				PostEvent(WINCE_MODULE,TX_TO_GUI_SHORT_KEY_EVENT, WORD((LSB(pEvt->prm)), keycode));
			} else {
				PostEvent(WINCE_MODULE,TX_TO_GUI_SHORT_KEY_EVENT, WORD(0xFF, keycode));
			}
			break;

	}           
}                                    

