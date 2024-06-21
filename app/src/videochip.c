
#include "public.h"

#define OUT_PUT1 0x00   
#define OUT_PUT2 0x01
#define OUT_PUT3 0x02
#define OUT_PUT4 0x03
#define OUT_PUT5 0x04
#define OUT_PUT6 0x05

/*chip address*/
#define FMS6502_ADDR    0x06  

/*offset address*/
#define OUT_PUT1_ADDR 0x00   
#define OUT_PUT2_ADDR 0x00
#define OUT_PUT3_ADDR 0x01
#define OUT_PUT4_ADDR 0x01
#define OUT_PUT5_ADDR 0x02
#define OUT_PUT6_ADDR 0x02
#define FMS6502_CLAMP 0x03
#define FMS6502_GAIN 	0x04

#define IN_PUTOFF 0x00
#define IN_PUT01   0x01
#define IN_PUT02   0x02
#define IN_PUT03   0x03
#define IN_PUT04   0x04
#define IN_PUT05   0x05
#define IN_PUT06   0x06
#define IN_PUT07   0x07
#define IN_PUT08   0x08

#define FRONT_CVBS_CHANNEL	OUT_PUT1
#define REAR1_CVBS_CHANNEL	OUT_PUT6
#define REAR2_CVBS_CHANNEL	OUT_PUT5

#define CHANNEL_GAIN_0db	1
#define CHANNEL_GAIN_6db	0

#define NAV_CVBS_SOURCE 			IN_PUT01

#define DVD_CVBS_SOURCE 			IN_PUT02
#define RIGHT_CAM_SOURCE				IN_PUT04
#define CAMERA_CVBS_SOURCE 			IN_PUT03
#define DTV_CVBS_SOURCE				IN_PUT05
#define AUX_CVBS_SOURCE				IN_PUT06
#define AUX_FRONT_CVBS_SOURCE		IN_PUT07
#define LEFT_CAM_SOURCE			IN_PUT08
#define NO_SOURCE					IN_PUTOFF

#define DVD_SYNC_SOURCE 			DVD_CVBS_SOURCE

//extern void SetFrontVideo(u8 prm)
//{
//	if (SOURCE_FRONT_AUX == prm) {
//		GPIO_SetBits(GPIO_FCAM_PWR_GRP, GPIO_FCAM_PWR_PIN);
//	}
//
//	if (SOURCE_FRONT_AUX != prm) {
//		GPIO_ResetBits(GPIO_FCAM_PWR_GRP, GPIO_FCAM_PWR_PIN);
//	}
//}

//void video_set_pwr_ctrl(bool on)
//{
//	if (on) {
//	}
//}

//void Video_Main(void)
//{
//	EVENT *nEvt;
//	uchar tmp_Lprm;
//
//	nEvt=GetEvent(VIDEO_MODULE);
//	tmp_Lprm=LSB(nEvt->prm);
//
//	if (g_fake_pwr_off) {
//		if (EVT_VID_FRONT_SOURCE_SET == (nEvt->ID)) {
//			SetFrontVideo(tmp_Lprm);
//		}
//		return;
//	}
//
//	switch(nEvt->ID)
//	{
//		case EVT_VID_FRONT_SOURCE_SET:
//			SetFrontVideo(tmp_Lprm);
//			break;
//		default:
//			break;
//	}
//}


