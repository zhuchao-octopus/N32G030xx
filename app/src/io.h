#ifndef _IO_H_
#define _IO_H_


#define HOST_COMM_UART	USART1
#define HOST_COMM_UART_IRQ	USART1_2_IRQn
#define GPIO_HOST_UART_TX_GRP		GPIOA
#define GPIO_HOST_UART_TX_PIN		GPIO_PIN_9
#define GPIO_HOST_UART_RX_GRP		GPIOA
#define GPIO_HOST_UART_RX_PIN		GPIO_PIN_10

#define CAN_COMM_UART		USART2
#define CAN_COMM_UART_IRQ		USART1_2_IRQn
#define GPIO_CAN_UART_TX_GRP		GPIOA
#define GPIO_CAN_UART_TX_PIN		GPIO_PIN_14
#define GPIO_CAN_UART_RX_GRP		GPIOA
#define GPIO_CAN_UART_RX_PIN		GPIO_PIN_15


#define GPIO_REAL_SYS_PWR_GRP		GPIOC
#define GPIO_REAL_SYS_PWR_PIN		GPIO_PIN_15
#define REAL_SYS_PWR_ON                {GPIO_SetBits(GPIO_REAL_SYS_PWR_GRP, GPIO_REAL_SYS_PWR_PIN);}
#define REAL_SYS_PWR_OFF               {GPIO_ResetBits(GPIO_REAL_SYS_PWR_GRP, GPIO_REAL_SYS_PWR_PIN);}
#define REAL_SYS_PWR_IS_ON()		(Bit_SET==GPIO_ReadOutputDataBit(GPIO_REAL_SYS_PWR_GRP, GPIO_REAL_SYS_PWR_PIN))

#define GPIO_DEVICE_5V_PWR_GRP		GPIOB
#define GPIO_DEVICE_5V_PWR_PIN		GPIO_PIN_3
#define SYSTEM_POWER_CTRL_ON                {GPIO_SetBits(GPIO_DEVICE_5V_PWR_GRP, GPIO_DEVICE_5V_PWR_PIN);}
#define SYSTEM_POWER_CTRL_OFF               {GPIO_ResetBits(GPIO_DEVICE_5V_PWR_GRP, GPIO_DEVICE_5V_PWR_PIN);}
#define SYSTEM_POWER_IS_ON()		(Bit_SET==GPIO_ReadOutputDataBit(GPIO_DEVICE_5V_PWR_GRP, GPIO_DEVICE_5V_PWR_PIN))

#define GPIO_AUDIO_PWR_GRP	GPIOB
#define GPIO_AUDIO_PWR_PIN		GPIO_PIN_6

#define GPIO_RADIO_RST_GRP		GPIOB
#define GPIO_RADIO_RST_PIN		GPIO_PIN_2

//#define GPIO_FCAM_PWR_GRP		GPIO
//#define GPIO_FCAM_PWR_PIN		GPIO_PIN_


#define TIMER_LCD_BKL			TIM8
#define TIMER_CH_LCD_BKL		TIM_CH_3
#define GPIO_BKL_EN_GRP		GPIOA
#define GPIO_BKL_EN_PIN		GPIO_PIN_2
#define disable_backlight() TIM_ConfigForcedOc3(TIMER_LCD_BKL, TIM_OCMODE_INACTIVE);

#define GPIO_ANT_CTRL_GRP		GPIOB
#define GPIO_ANT_CTRL_PIN		GPIO_PIN_7

#define GPIO_RADIO_ANT_CTRL_GRP		GPIOB
#define GPIO_RADIO_ANT_CTRL_PIN		GPIO_PIN_9


#define GPIO_ACC_GRP		GPIOF
#define GPIO_ACC_PIN		GPIO_PIN_7

#define GPIO_BRAKE_GRP		GPIOA
#define GPIO_BRAKE_PIN		GPIO_PIN_11

#define GPIO_ILL_GRP		GPIOA
#define GPIO_ILL_PIN		GPIO_PIN_12

#define GPIO_REVERSE_GRP		GPIOF
#define GPIO_REVERSE_PIN		GPIO_PIN_6

#define GPIO_MUTE_GRP		GPIOB
#define GPIO_MUTE_PIN		GPIO_PIN_15
#define AUDIO_HW_MUTE		GPIO_SetBits(GPIO_MUTE_GRP, GPIO_MUTE_PIN);
#define AUDIO_HW_UNMUTE		GPIO_ResetBits(GPIO_MUTE_GRP, GPIO_MUTE_PIN);

#define GPIO_I2C_GRP		GPIOB
#define GPIO_I2C_SCL_PIN		GPIO_PIN_10
#define GPIO_I2C_SDA_PIN		GPIO_PIN_11


#define GPIO_HOST_REC_KEY_GRP		GPIOB
#define GPIO_HOST_REC_KEY_PIN		GPIO_PIN_13

#define GPIO_HOST_PWR_KEY_GRP		GPIOB
#define GPIO_HOST_PWR_KEY_PIN		GPIO_PIN_14


#define GPIO_SWC_KEY_CTL_2K2_GRP		GPIOC
#define GPIO_SWC_KEY_CTL_2K2_PIN		GPIO_PIN_13
#define GPIO_SWC_KEY_CTL_470R_GRP		GPIOC
#define GPIO_SWC_KEY_CTL_470R_PIN		GPIO_PIN_14


#define AD_PANEL_KEY_DET_GRP	ADC
#define AD_PANEL_KEY_DET_1		ADC_CH_5
#define GPIO_AD_PANEL_KEY_DET_GRP			GPIOA
#define GPIO_AD_PANEL_KEY_DET_1_PIN		GPIO_PIN_5

#define AD_PANEL_KEY_DET_GRP	ADC
#define AD_PANEL_KEY_DET_2		ADC_CH_4
//#define GPIO_AD_PANEL_KEY_DET_GRP			GPIOA
#define GPIO_AD_PANEL_KEY_DET_2_PIN		GPIO_PIN_4

#define AD_CAR_SWC_DET_GRP	ADC
#define AD_CAR_SWC_DET_1		ADC_CH_0
#define AD_CAR_SWC_DET_2		ADC_CH_1
#define GPIO_AD_SWC_KEY_DET_GRP			GPIOA
#define GPIO_AD_SWC_KEY_DET_1_PIN		GPIO_PIN_0
#define GPIO_AD_SWC_KEY_DET_2_PIN		GPIO_PIN_1

#define AD_VOL_ENCODER_GRP	ADC
#define AD_VOL_ENCODER		ADC_CH_6
#define GPIO_AD_VOL_GRP		GPIOA
#define GPIO_AD_VOL_PIN		GPIO_PIN_6

#define AD_TUNE_ENCODER_GRP	ADC
#define AD_TUNE_ENCODER		ADC_CH_7
#define GPIO_AD_TUNE_GRP		GPIOA
#define GPIO_AD_TUNE_PIN		GPIO_PIN_7


#define AD_BATT_DET_GRP	ADC
#define AD_BATT_DET		ADC_CH_8
#define GPIO_AD_BATT_DET_GRP		GPIOB
#define GPIO_AD_BATT_DET_PIN		GPIO_PIN_0


#define TIMER_LED		TIM3
#define TIMER_CH_LED_B		TIM_CH_1
#define TIMER_CH_LED_R		TIM_CH_2
#define TIMER_CH_LED_G		TIM_CH_4
#define GPIO_LED_GRP		GPIOB
#define GPIO_LED_B_PIN		GPIO_PIN_4
#define GPIO_LED_R_PIN		GPIO_PIN_5
#define GPIO_LED_G_PIN		GPIO_PIN_1
#define GPIO_LED_EN_PIN		GPIO_PIN_12


#define TIMER_BEEP		TIM1
#define TIMER_CH_BEEP		TIM_CH_1
#define GPIO_BEEP_GRP		GPIOA
#define GPIO_BEEP_PIN		GPIO_PIN_8


#define TIMER_IR_RX		TIM1
#define TIMER_CH_IR_RX	TIM_CH_2
#define GPIO_IR_RX_GRP		GPIOA
#define GPIO_IR_RX_PIN		GPIO_PIN_3


ext u8 illumin_level;
extern u8 g_panel_led_all_on;



extern void ABRT_Driver(void);
extern void Illumi_Detect(void);

extern void Reverse_Detect(void);

extern void Parking_Pro(void);

void SystemPowerContrl(ONOFF On_Off);

#endif	//_IO_H

