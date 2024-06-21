
#include "public.h"

static uint16_t g_bkl_duty[MAX_BRIGHTNESS_LEVEL]={0, 21, 50, 100, 224, 256, 284, 316, 344, 376, 404, 436, 476, 516, 576, 636, 708, 788, 912, 960, 999};
static uint16_t g_bkl_duty_dark[MAX_BRIGHTNESS_LEVEL]={0, 10, 30, 50, 134, 154, 170, 190, 206, 226, 242, 262, 285, 309, 345, 382, 425, 473, 547, 576, 599};



u16 g_pre_tft_backlight_level = 0;
void ABRT_Driver(void)
{
	u16 tft_backlight_level;
	u8 level;

	if (g_brightness<0) {
		level = 0;
	} else {
		level = g_brightness;
	}

	if(F_ILLUMIN_CONTORL_MODE == ILLUMIN_AUTO_MODE)
	{
		if(Det_Flag.field.F_Lighting_Det==1)
		{
			tft_backlight_level=g_bkl_duty_dark [level];	
		}
		else
		{
			tft_backlight_level=g_bkl_duty [level];				
		}  	
	}
	else if(F_ILLUMIN_CONTORL_MODE == ILLUMIN_NIGHT_MODE)
	{
		tft_backlight_level=g_bkl_duty_dark [level];
	}
	else
	{
		tft_backlight_level=g_bkl_duty [level];
	}

	//所有屏亮度控制真正动作在此处完成
	if ( ( F_TFT_ON == OFF ) || (g_bkl_usr_ctrl==0) )
	{
		disable_backlight();
		if (0!=g_pre_tft_backlight_level) {
			// notify HOST. so they can use it to control their backllight
			PostEvent(WINCE_MODULE, TX_TO_GUI_BRIGHTNESS_INFO,0 );
		}
		g_pre_tft_backlight_level = 0;
	}
	else
	{
		if (g_pre_tft_backlight_level!=tft_backlight_level) {
			if (0 == tft_backlight_level) {
				disable_backlight();
			} else {
				TIM_SetCmp3(TIMER_LCD_BKL, tft_backlight_level);
				TIM_ConfigForcedOc3(TIMER_LCD_BKL, TIM_OCMODE_PWM1);
			}
			g_pre_tft_backlight_level = tft_backlight_level;

			// notify HOST. so they can use it to control their backllight
//			PostEvent(WINCE_MODULE, TX_TO_GUI_BRIGHTNESS_INFO,level );
		}
	}
}

void Illumi_Detect(void)
{
	bool port;
	static uchar Light_Sampling_Counter=1;
	if(SYSTEM_POWER_IS_ON()==0)	//20070907 ILLUMI先置高后开机，TFT亮度错误
		return;
	port=(bool)(Bit_RESET==GPIO_ReadInputDataBit(GPIO_ILL_GRP, GPIO_ILL_PIN));
	Light_Sampling_Counter--;
	if (Light_Sampling_Counter==0)
	{
		//多次采样确认
		if(port)
		{
			Det_Flag.field.F_Lighting_Det=1;
		}
		else
		{
			Det_Flag.field.F_Lighting_Det=0;
		}
		PostEvent(WINCE_MODULE, TX_TO_GUI_BRIGHTNESS_INFO, g_brightness);
	}
	if ((port&&Det_Flag.field.F_Lighting_Det)||(!port&&(!Det_Flag.field.F_Lighting_Det)))
	{
		//端口电平与标志位一致 :=1
		Light_Sampling_Counter=N_SAMPLING_LIGHTOFF;
	}
}

// 倒车检测
// 检测到倒车信号后，视频切换到后视镜模式
#define N_SAMPLING_REVERSE	 3
void Reverse_Detect(void)	//12ms执行一次
{

	bool port;
	static uchar Reverse_Sampling_Counter=1;

	if(Get_ACC_Det_Flag==0)
	{
		Reverse_Sampling_Counter=N_SAMPLING_REVERSE;
		Clr_Reverse_Det_Flag;	
		return;
	}
	port=(bool)(Bit_RESET==GPIO_ReadInputDataBit(GPIO_REVERSE_GRP, GPIO_REVERSE_PIN));
	Reverse_Sampling_Counter--;

	if(Reverse_Sampling_Counter==0)
	{
		//多次采样确认
		if(port)
		{
			Set_Reverse_Det_Flag;
		}
		else 
		{	
			Clr_Reverse_Det_Flag;
		}
	}
	if ((port&&Get_Reverse_Det_Flag) ||(!port &&(!Get_Reverse_Det_Flag)))
	{
		//端口电平与标志位一致 :=1
		Reverse_Sampling_Counter=N_SAMPLING_REVERSE;
	}
}




#define 	N_SAMPLING_PARK	3
static uchar Park_Sampling_Counter;

void Parking_Pro(void)
{
	u8 port;

	//复位上电时PARKING_MODE 还未初始化，导致F_Stop_Car 被置1
	if(!Is_Machine_Power)
	{
		Park_Sampling_Counter=1;
		F_PARKING=0; //已开车
		return;
	}

//	if (g_canbox_use_uart) {
		port=GPIO_ReadInputDataBit(GPIO_BRAKE_GRP, GPIO_BRAKE_PIN);
//	} else {
//	fix this later	port=( (0==GPIO_CAR_BRAKE_DET_2) || (1==GPIO_CAR_BRAKE_DET) );  //采样
//		port=GPIO_ReadInputDataBit(GPIO_BRAKE_GRP, GPIO_BRAKE_PIN);
//	}
	Park_Sampling_Counter--;
	if (Park_Sampling_Counter==0)
	{
		//多次采样确认
		if (!port)
		{
			F_PARKING=1; //已刹车
		}
		else
		{
			F_PARKING=0; //已开车
		}
		PostEvent(WINCE_MODULE, TX_TO_GUI_CAR_PARKING_STATUS,F_PARKING);
	}

	if ((!port&&F_PARKING==1) ||(port&&F_PARKING==0) )
	{
		//端口电平与标志位一致 :=1
		Park_Sampling_Counter=N_SAMPLING_PARK;
	}	
}


/************************关闭或者打开PowerCtrl脚****************************/
void SystemPowerContrl(ONOFF On_Off)
{
	if(On_Off==ON)
	{
		REAL_SYS_PWR_ON;
		SYSTEM_POWER_CTRL_ON;

	}
	else
	{
		GPIO_ResetBits(GPIO_ANT_CTRL_GRP, GPIO_ANT_CTRL_PIN);
//		GPIO_ResetBits(GPIO_FCAM_PWR_GRP, GPIO_FCAM_PWR_PIN);
		SYSTEM_POWER_CTRL_OFF;
		if (g_kill_host) {
			REAL_SYS_PWR_OFF;
			g_kill_host = FALSE;
		}
	}
}

