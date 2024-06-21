
#include "public.h"

#if ASP_MODEL==ASP_BD37534

static const u8 g_tm2313_gains[] = {
	0x3F,	/* volume 0 */
	0x26, 0x23, 0x21, 0x1E, 0x1C,	/* volume 1~5 */
	0x1B, 0x1A, 0x19, 0x18, 0x17,	/* volume 6~10 */
	0x16, 0x15, 0x14, 0x13, 0x12,	/* volume 11~15 */
	0x11, 0x10, 0x0F, 0x0E, 0x0D,		/* volume 16~20 */
	0x0C, 0x0B, 0x0A, 0x09, 0x08,		/* volume 21~25 */
	0x07, 0x06, 0x04, 0x02, 0		/* volume 26~30 */
};

static const u8 g_tm2313_atten_table[8] = {0x00, 0x02, 0x04, 0x06, 0x08, 0x0B, 0xE, 0x1F};

static u8 g_volume;
static AUDIO_SOURCE g_src;
static u8 g_navi_mix;
static u8 g_fader;
static u8 g_balance;
static u8 g_loud;

static uint8_t g_tm2313_i2c_error = 0;
static void tm2313_i2c_wait_ack()
{
	if (i2c_wait_ack()) {
		++g_tm2313_i2c_error;
	} else {
		g_tm2313_i2c_error=0;
	}
}

static void tm2313_write(u8 cmd)
{
	if (g_tm2313_i2c_error>8) {
		return;
	}
	i2c_start();
	i2c_send_byte(0x88);
	tm2313_i2c_wait_ack();
	i2c_send_byte(cmd);
	tm2313_i2c_wait_ack();
	i2c_stop();
}

static void tm2313_update_vol_src(void)
{
	u8 fl_atten = 0;
	u8 fr_atten = 0;
	u8 rl_atten = 0;
	u8 rr_atten = 0;
	u8 val;
	u8 mute=0;

	// volume control
	tm2313_write(0x00 | g_tm2313_gains[g_volume]);

	// speaker attenuation
	if ( (g_src == AUDIO_SRC_NONE) && (0==g_navi_mix) ) {
		mute = 1;
	}
	if (mute) {
		fl_atten = 0x1F;
		fr_atten = 0x1F;
		rl_atten = 0x1F;
		rr_atten = 0x1F;
	} else {
		if (g_fader<=7) {
			rl_atten += g_tm2313_atten_table[7-g_fader];
			rr_atten += g_tm2313_atten_table[7-g_fader];
		} else {
			fl_atten += g_tm2313_atten_table[g_fader-7];
			fr_atten += g_tm2313_atten_table[g_fader-7];
		}
		if (g_balance<=7) {
			fr_atten += g_tm2313_atten_table[7-g_balance];
			rr_atten += g_tm2313_atten_table[7-g_balance];
		} else {
			fl_atten += g_tm2313_atten_table[g_balance-7];
			rl_atten += g_tm2313_atten_table[g_balance-7];
		}
		if (fl_atten>0x1F)	fl_atten=0x1F;
		if (fr_atten>0x1F)	fr_atten=0x1F;
		if (rl_atten>0x1F)	rl_atten=0x1F;
		if (rr_atten>0x1F)	rr_atten=0x1F;
	}

	tm2313_write(0x80 | fl_atten);
	tm2313_write(0xA0 | fr_atten);
	tm2313_write(0xC0 | rl_atten);
	tm2313_write(0xE0 | rr_atten);

	// audio switch
	val = 0x40;	// input +11.25dB

//	if (SOURCE_BT == FrontSource) {
//		val |= 0x18;	// input 0dB
//	}

	if (0==g_loud) {
		val |= 0x04;	// loud off
	}
	if (g_navi_mix) {
		val |= 0x01;
	} else {
		switch (g_src) {
			case AUDIO_SRC_AUXIN:
			case AUDIO_SRC_TV:
				val |= 0x00;
				break;
			case AUDIO_SRC_HOST:
				val |= 0x01;
				break;
			case AUDIO_SRC_RADIO:
				val |= 0x02;
				break;
			default:
				val |= 0x03;
				break;
		}
	}
	tm2313_write(val);
}

bool audio_dev_init(void)
{
	g_volume = 0;
	g_src = AUDIO_SRC_NONE;
	g_navi_mix = 0;
	g_fader = DEFAULT_FIELD_LEVEL;
	g_balance = DEFAULT_FIELD_LEVEL;
	g_loud = 0;
	tm2313_update_vol_src();
	return TRUE;
}

void audio_dev_deinit(void)
{
}

void audio_dev_update_source(AUDIO_SOURCE src)
{
	g_src = src;
	audio_set_mute_temporary(1000);
	tm2313_update_vol_src();
}

void audio_dev_update_volume(u8 vol)
{
	g_volume = vol;
	tm2313_update_vol_src();
}

void audio_dev_update_navi_mix(bool on)
{
	g_navi_mix = on;
	audio_set_mute_temporary(1000);
	tm2313_update_vol_src();
}

void audio_dev_update_navi_mix_vol(u8 vol)
{
}

static u8 tm2313_cal_eq_value(u8 level)
{
	if (level>85) {
		return 0x08;
	} else if (level>80) {
		return 0x09;
	} else if (level>75) {
		return 0x0A;
	} else if (level>70) {
		return 0x0B;
	} else if (level>64) {
		return 0x0C;
	} else if (level>58) {
		return 0x0D;
	} else if (level>52) {
		return 0x0E;
	} else if (level>45) {
		return 0x0F;
	} else if (level>38) {
		return 0x07;
	} else if (level>32) {
		return 0x06;
	} else if (level>26) {
		return 0x05;
	} else if (level>20) {
		return 0x04;
	} else if (level>15) {
		return 0x03;
	} else if (level>10) {
		return 0x02;
	} else if (level>5) {
		return 0x01;
	} else {
		return 0x00;
	}
}

void audio_dev_update_eq(void)
{
	u8 val;
	u8 tmp;

	/* calc the BASS gain */
	tmp = 0;
	tmp += g_audio_info.eq_visible_level[EQ_FREQ_60HZ];
	tmp += g_audio_info.eq_visible_level[EQ_FREQ_100HZ];
	tmp += g_audio_info.eq_visible_level[EQ_FREQ_120HZ];
	tmp += g_audio_info.eq_visible_level[EQ_FREQ_0K5HZ];
	if (g_audio_info.eq_visible_level[EQ_FREQ_1KHZ]<DEFAULT_EQ_LEVEL) {
		tmp += g_audio_info.eq_visible_level[EQ_FREQ_1KHZ];
	} else {
		tmp += DEFAULT_EQ_LEVEL;
	}
	val = tm2313_cal_eq_value(tmp);
	tm2313_write(0x60|(val&0x0F));

	
	/* calc the TREBLE gain */
	tmp = 0;
	if (g_audio_info.eq_visible_level[EQ_FREQ_1KHZ]>DEFAULT_EQ_LEVEL) {
		tmp += (g_audio_info.eq_visible_level[EQ_FREQ_1KHZ]-DEFAULT_EQ_LEVEL);
	} else {
		tmp += 0;
	}
	tmp += g_audio_info.eq_visible_level[EQ_FREQ_1K5HZ];
	tmp += g_audio_info.eq_visible_level[EQ_FREQ_10KHZ];
	tmp += g_audio_info.eq_visible_level[EQ_FREQ_12K5HZ];
	tmp += g_audio_info.eq_visible_level[EQ_FREQ_15KHZ];
	val = tm2313_cal_eq_value(tmp);
	tm2313_write(0x70|(val&0x0F));
}

void audio_dev_update_fader_balance(u8 fad, u8 bal)
{
	g_fader = fad;
	g_balance = bal;
	tm2313_update_vol_src();
}

void audio_dev_update_loudness(u8 loud)
{
	g_loud = loud;
	tm2313_update_vol_src();
}

void audio_dev_update_subwoofer(u8 subwoofer)
{
}

void audio_dev_update_dsp_settings_1(void)
{
}

void audio_dev_update_dsp_settings_2(void)
{
}

void audio_dev_update_spectrum_data(void)
{
}

#endif

