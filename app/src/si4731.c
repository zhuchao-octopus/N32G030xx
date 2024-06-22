
#include "public.h"

#if TUNER_MODEL==TUNER_SILICONLAB_SI4731
#include "si4731.h"

static RADIO_BAND g_si474x_band;
static RADIO_DEV_SEEK_MODE g_si474x_seek_mode = RDSM_DX;
static uint8_t  g_si474x_cmd[CMD_BUFFER_SIZE];
static uint8_t  g_si474x_rsp[RSP_BUFFER_SIZE];

static uint8_t g_si474x_i2c_error = 0;

static void si474x_i2c_wait_ack()
{
	if (i2c_wait_ack()) {
//		++g_si474x_i2c_error;
	} else {
//		g_si474x_i2c_error=0;
	}
}


void si474x_i2c_write(uint8_t* src_addr, uint8_t len)
{
	uint8_t cnt;

	if (g_si474x_i2c_error>16) {
		return;
	}
	
	i2c_start();
	
	i2c_send_byte(SI4731_I2C_ADDR_W);
	si474x_i2c_wait_ack();
	
	for (cnt=0;cnt<len;cnt++) {
		i2c_send_byte(src_addr[cnt]);
		si474x_i2c_wait_ack();
	}
 	i2c_stop();
}

void si474x_i2c_read (uint8_t* dest_addr, uint8_t len)
{
	uint8_t cnt;
	
	if (g_si474x_i2c_error>16) {
		return;
	}

	i2c_start();
	
	i2c_send_byte(SI4731_I2C_ADDR_R);
	si474x_i2c_wait_ack();

	for (cnt=0;cnt<len;cnt++) {
		if (cnt==(len-1)) {
	   	dest_addr[cnt] = i2c_read_byte(0);
		} else {
	   	dest_addr[cnt] = i2c_read_byte(1);
		}
	}
 	i2c_stop();

}

//-----------------------------------------------------------------------------
// Command that will wait for CTS before returning
//-----------------------------------------------------------------------------
bool si474x_waitForCTS()
{
	uint16_t i = 1000;
	uint8_t status;
	if (g_si474x_i2c_error>16) {
		return false;
	}

	do
	{
		CLEAR_WATCHDOG;
		si474x_i2c_read(&status, 1);
		delay_us(5);
	}while (--i && !(status & CTS));

	if ((0==i) || (!(status & CTS)) || (0xFF == status)) {
		return false;
	}
	
	return true;
//	if (0==i) {
//		++g_si474x_i2c_error;
//	} else {
//		g_si474x_i2c_error=0;
//	}
}

//-----------------------------------------------------------------------------
// Sends a command to the part and returns the reply bytes
//-----------------------------------------------------------------------------
void si474x_command(uint8_t cmd_size, uint8_t *cmd, uint8_t reply_size, uint8_t *reply)
{
	// It is always a good idea to check for cts prior to sending a command to
	// the part.
	si474x_waitForCTS();

	// Write the command to the part
	si474x_i2c_write(cmd, cmd_size);

	// Wait for CTS after sending the command
	si474x_waitForCTS();

	// If the calling function would like to have results then read them.
	if(reply_size)
	{
		si474x_i2c_read(reply, reply_size);
	}
}

void si474x_set_property(uint16_t propNumber, uint16_t propValue)
{
	// Put the ID for the command in the first byte.
	g_si474x_cmd[0] = SET_PROPERTY;

	// Initialize the reserved section to 0
	g_si474x_cmd[1] = 0;

	// Put the property number in the third and fourth bytes.
	g_si474x_cmd[2] = (uint8_t)(propNumber >> 8);
	g_si474x_cmd[3] = (uint8_t)(propNumber & 0x00FF);

	// Put the property value in the fifth and sixth bytes.
	g_si474x_cmd[4] = (uint8_t)(propValue >> 8);
	g_si474x_cmd[5] = (uint8_t)(propValue & 0x00FF);

	// Invoke the command
	si474x_command(6, g_si474x_cmd, 0, NULL);
}


void si474x_powerup(RADIO_BAND band)
{
	/* do reset */
	GPIO_ResetBits(GPIO_RADIO_RST_GRP, GPIO_RADIO_RST_PIN);
	delay_1ms(2);
	GPIO_SetBits(GPIO_RADIO_RST_GRP, GPIO_RADIO_RST_PIN);
	delay_1ms(2);

	/* power up */
	// Put the ID for the command in the first byte.
	g_si474x_cmd[0] = POWER_UP;

	// Enable the GPO2OEN on the part because it will be used to determine
	// RDS Sync timing.
//	g_si474x_cmd[1] = POWER_UP_IN_GPO2OEN;
	g_si474x_cmd[1] = 0x10;

	// The device is being powered up in FM RX mode.
	if (RADIO_BAND_FM == band) {
		g_si474x_cmd[1] |= POWER_UP_IN_FUNC_FMRX;
	} else {
		g_si474x_cmd[1] |= POWER_UP_IN_FUNC_AMRX;
	}

	// The opmode needs to be set to analog mode
	g_si474x_cmd[2] = POWER_UP_IN_OPMODE_RX_ANALOG;

	// Powerup the device
	si474x_command(3, g_si474x_cmd, 8, g_si474x_rsp);
	delay_1ms(110);               // wait for si47xx to powerup

	/* configure */
	if (RADIO_BAND_FM == band) {
		si474x_set_property(FM_SEEK_TUNE_SNR_THRESHOLD, 3);
		si474x_set_property(FM_SEEK_TUNE_RSSI_THRESHOLD, 1);

		// Setup the RDS Interrupt to interrupt when RDS data is available.
		si474x_set_property(FM_RDS_INTERRUPT_SOURCE, FM_RDS_INTERRUPT_SOURCE_RECV_MASK); 
		si474x_set_property(FM_RDS_INTERRUPT_FIFO_COUNT, 1);

		// Enable the RDS and allow all blocks so we can compute the error
		// rate later.
		si474x_set_property(FM_RDS_CONFIG, 0xFF01);
		//FM_RDS_CONFIG_RDSEN_MASK |
		//			(3 << FM_RDS_CONFIG_BLETHA_SHFT) |
		//			(3 << FM_RDS_CONFIG_BLETHB_SHFT) |
		//			(3 << FM_RDS_CONFIG_BLETHC_SHFT) |
		//			(3 << FM_RDS_CONFIG_BLETHD_SHFT));

		si474x_set_property(FM_DEEMPHASIS, FM_DEEMPH_50US); // Deemphasis
		si474x_set_property(FM_SEEK_FREQ_SPACING, 10);      // 100 kHz Spacing
	} else {
		si474x_set_property(AM_SEEK_TUNE_SNR_THRESHOLD, 4);
		si474x_set_property(AM_SEEK_TUNE_RSSI_THRESHOLD, 20);
	
		si474x_set_property(AM_SEEK_FREQ_SPACING, 10); // Set spacing to 10kHz
	}

	// Turn off the mute
//	si474x_set_property(RX_HARD_MUTE, 0);
	// Set the volume to the passed value
//	si474x_set_property(RX_VOLUME, 63);

}

#define TEST_GET_REV
bool radio_dev_init(void) 
{
	uint8_t x,y;

#ifdef TEST_GET_REV
	uint8_t partNumber;
	//char fwMajor;
	//char fwMinor;
	//uint16_t  patchID;
	//char cmpMajor;
	//char cmpMinor;
	//char chipRev;
#endif

	g_si474x_band = RADIO_BAND_FM;
	si474x_powerup(g_si474x_band);

#ifdef TEST_GET_REV
	/* Get the reversion info */
	g_si474x_cmd[0] = GET_REV;
	si474x_command(1, g_si474x_cmd, 9, g_si474x_rsp);
	partNumber = g_si474x_rsp[1];
	//fwMajor  = (char)g_si474x_rsp[2];
	//fwMinor  = (char)g_si474x_rsp[3];
	//patchID  = (uint16_t)(g_si474x_rsp[4] << 8) | (uint16_t)g_si474x_rsp[5];
	//cmpMajor = (char)g_si474x_rsp[6];
	//cmpMinor = (char)g_si474x_rsp[7];
	//chipRev  = (char)g_si474x_rsp[8];
	if ( (partNumber==0xFF) || (partNumber==0) ) {
		g_si474x_i2c_error=250;
	}
	else 
	{
		g_si474x_i2c_error = 0;
	}
#endif

	if (g_si474x_i2c_error>64) {
		for (x=PRESET_BAND_FM1; x<=PRESET_BAND_FM3; x++) {
			for(y=0;y<FM_PRESET_NUM;y++) {
				g_radio_info.preset_fm[x-PRESET_BAND_FM1][y]=2;
			}
		}
	}

	return TRUE;
}

void radio_dev_rds_init(void)
{
}

bool radio_dev_deinit(void) 
{
	return TRUE;
}
bool radio_dev_is_set_freq_done(void)
{
	g_si474x_cmd[0] = GET_INT_STATUS;
	si474x_command(1, g_si474x_cmd, 1, g_si474x_rsp);
	return (g_si474x_rsp[0] & STCINT);
}
void radio_dev_set_freq(RADIO_BAND band, u16 freq)
{
	if (band != g_si474x_band) {
		g_si474x_band = band;
		si474x_powerup(g_si474x_band);
	}
	
	if (RADIO_BAND_FM == g_si474x_band) {
		// Put the ID for the command in the first byte.
		g_si474x_cmd[0] = FM_TUNE_FREQ;

		// Initialize the reserved section to 0
		g_si474x_cmd[1] = 0;

		// Put the frequency in the second and third bytes.
		g_si474x_cmd[2] = (uint8_t)(freq >> 8);
		g_si474x_cmd[3] = (uint8_t)(freq & 0x00FF);

		// Set the antenna calibration value.
		g_si474x_cmd[4] = (uint8_t)0;  // Auto

		// Invoke the command
		si474x_command(5, g_si474x_cmd, 1, g_si474x_rsp);
	} else {
		// Put the ID for the command in the first byte.
		g_si474x_cmd[0] = AM_TUNE_FREQ;

		// Initialize the reserved section to 0
		g_si474x_cmd[1] = 0;

		// Put the frequency in the second and third bytes.
		g_si474x_cmd[2] = (uint8_t)(freq >> 8);
		g_si474x_cmd[3] = (uint8_t)(freq & 0x00FF);

		// Set the antenna calibration value.
		g_si474x_cmd[4] = (uint8_t)0;  // Auto
		g_si474x_cmd[5] = (uint8_t)0;

		// Invoke the command
		si474x_command(6, g_si474x_cmd, 1, g_si474x_rsp);
	}
}

void radio_dev_set_freq_tune(RADIO_BAND band, u16 freq)
{
	radio_dev_set_freq(band, freq);
}

bool radio_dev_is_tune_ok(bool strict)
{
	u16 offset;

	if (RADIO_BAND_FM == g_si474x_band) {
		g_si474x_cmd[0] = FM_RSQ_STATUS;
		g_si474x_cmd[1] = 0;
		si474x_command(2, g_si474x_cmd, 8, g_si474x_rsp);

		// check freq offset
		if (g_si474x_rsp[7]>=0x80) {
			offset = 256-g_si474x_rsp[7];
		} else {
			offset = g_si474x_rsp[7];
		}
		if (offset >8) {
			return 0;
		}

		if (RDSM_DX == g_si474x_seek_mode) {
			return (g_si474x_rsp[2] & 0x01);
		} else {
			if (g_si474x_rsp[2] & 0x01) {
				return ( (g_si474x_rsp[4]>18) && (g_si474x_rsp[4]<0x7F) );
			} else {
				return 0;
			}
		}
	} else {
		g_si474x_cmd[0] = AM_TUNE_STATUS;
		g_si474x_cmd[1] = 0;
		g_si474x_cmd[1] |= AM_TUNE_STATUS_IN_CANCEL;
		g_si474x_cmd[1] |= AM_TUNE_STATUS_IN_INTACK;
		si474x_command(2, g_si474x_cmd, 8, g_si474x_rsp);
		return (g_si474x_rsp[1] & 0x01);
	}
}
bool radio_dev_is_tune_status_ready(void)
{

	return TRUE;
}
u8 radio_dev_get_stereo_status(void)
{
	uint8_t blend;
	if (RADIO_BAND_FM != g_si474x_band) {
		return 1;
	}

	g_si474x_cmd[0] = FM_RSQ_STATUS;
	g_si474x_cmd[1] = 0;
	si474x_command(2, g_si474x_cmd, 4, g_si474x_rsp);

	blend = g_si474x_rsp[3];
	if ( (blend&0x80) && ((blend&0x7F)>10) ) {
		return 2;
	} else {
		return 1;
	}
}

bool radio_dev_set_seek_mode(RADIO_DEV_SEEK_MODE mode)
{
	g_si474x_seek_mode = mode;
	return TRUE;
}

bool radio_dev_set_st_mode(bool on)
{
	if (0 == on) {
		si474x_set_property(FM_BLEND_RSSI_STEREO_THRESHOLD, 127);
		si474x_set_property(FM_BLEND_RSSI_MONO_THRESHOLD, 127);
	} else {
		si474x_set_property(FM_BLEND_RSSI_STEREO_THRESHOLD, 49);
		si474x_set_property(FM_BLEND_RSSI_MONO_THRESHOLD, 30);
	}
	return TRUE;
}

bool radio_dev_set_mute(bool mute)
{
	if (mute) {
		si474x_set_property(RX_HARD_MUTE, RX_HARD_MUTE_RMUTE_MASK | RX_HARD_MUTE_LMUTE_MASK);
	} else {
		si474x_set_property(RX_HARD_MUTE, 0);
	}
	return TRUE;
}

#if RDS_FUNCTION==1
// RDS Block x Corrected Errors
#define CORRECTED_NONE		0
#define CORRECTED_SMALL	1
#define CORRECTED_LARGE	2
#define UNCORRECTABLE		3

bool radio_dev_rds_update(void)
{
	u8 bleA, bleB, bleC, bleD;
	u8 group_type;
	u8 addr;

	g_si474x_cmd[0] = FM_RDS_STATUS;
	g_si474x_cmd[1] = 0x01;       // INTACK
	si474x_command(2, g_si474x_cmd, 13, g_si474x_rsp);
	if (0==g_si474x_rsp[3]) {
		return FALSE;
	}

	bleA = (g_si474x_rsp[12] & 0x00C0) >> 6;
	bleB = (g_si474x_rsp[12] & 0x0030) >> 4;
	bleC = (g_si474x_rsp[12] & 0x000C) >> 2;
	bleD = (g_si474x_rsp[12] & 0x0003) >> 0;

	// check RDS Block B Corrected Errors
	// It's critical that block B be correct since it determines what's
	// contained in the latter blocks. For this reason, a stricter tolerance
	// is placed on block B
	if (CORRECTED_SMALL < bleB) {
		// Drop the data if more than two errors were uncorrected in block B
		return FALSE;
	}

	g_rds_info.block_a = ((u16)g_si474x_rsp[4]  << 8) | (u16)g_si474x_rsp[5];
	g_rds_info.block_b = ((u16)g_si474x_rsp[6]  << 8) | (u16)g_si474x_rsp[7];
	g_rds_info.block_c = ((u16)g_si474x_rsp[8]  << 8) | (u16)g_si474x_rsp[9];
	g_rds_info.block_d = ((u16)g_si474x_rsp[10]  << 8) | (u16)g_si474x_rsp[11];

	if (CORRECTED_LARGE >= bleA) {
		rds_update_pi(g_rds_info.block_a);
	}

	group_type = g_rds_info.block_b >> 11;

	// Update pi code.  Version B formats always have the pi code in words A and C
	if (group_type & 0x01) {
		if (CORRECTED_LARGE >= bleC) {
			rds_update_pi(g_rds_info.block_c);
		}
	}

	rds_update_pty((g_rds_info.block_b >> 5) & 0x1f);
	rds_update_tp(0!=(g_rds_info.block_b & (1<<10)));

	switch (group_type) 
	{
		case RDS_TYPE_0A:
			if (CORRECTED_LARGE >= bleC) {
				rds_update_af_list(g_rds_info.block_c);
			}
			// fallthrough
		case RDS_TYPE_0B:
			rds_update_ta(0!=(g_rds_info.block_b & (1<<4)));
			addr = (g_rds_info.block_b & 0x3) * 2;
			if (CORRECTED_LARGE >= bleD) {
				rds_update_ps(addr+0, g_rds_info.block_d >> 8  );
				rds_update_ps(addr+1, g_rds_info.block_d & 0xff);
			}
			break;

		case RDS_TYPE_2A:
			if ((CORRECTED_LARGE>=bleD) && (CORRECTED_LARGE>=bleC)) {
				addr = g_rds_info.block_b&0x0F;
				rds_update_rt(addr, 4, 
					(g_rds_info.block_b&0x10)>>4, 
					g_rds_info.block_c>>8,
					g_rds_info.block_c&0xFF,
					g_rds_info.block_d>>8,
					g_rds_info.block_d&0xFF);
			}
			break;

		case RDS_TYPE_2B:
			if (CORRECTED_LARGE>=bleD) {
				addr = g_rds_info.block_b&0x0F;
				rds_update_rt(addr, 2, 
					(g_rds_info.block_b&0x10)>>4, 
					g_rds_info.block_d>>8,
					g_rds_info.block_d&0xFF,
					0,
					0);
			}
			break;

		case RDS_TYPE_4A:
			// TODO : CT 
			break;
		default:
			break;
	}

	return FALSE;
}
#endif


bool radio_dev_is_tune_error()
{
	return !si474x_waitForCTS();
}

#endif

