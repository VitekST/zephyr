/*
 * Copyright (c) 2023 NXP Semiconductor INC.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/audio/codec.h>
#include <zephyr/devicetree/clocks.h>

#include <fsl_clock.h>

#define LOG_LEVEL CONFIG_AUDIO_WM8904_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wolfson_wm8904);

#include "wm8904.h"

#define DT_DRV_COMPAT wolfson_wm8904

#define OUTPUT_VOLUME_MAX 0
#define OUTPUT_VOLUME_MIN (-78 * 2)
#define OUTPUT_VOLUME_DEFAULT (0b101101)
#define INPUT_VOLUME_DEFAULT (0b00101)

struct wm8904_driver_config {
	struct i2c_dt_spec i2c;
	int    clock_source;
	const struct device *mclk_dev;
	clock_control_subsys_t mclk_name;
	const struct pinctrl_dev_config *pincfg;
};

#define DEV_CFG(dev) ((const struct wm8904_driver_config *const)dev->config)

static void wm8904_write_reg(const struct device *dev, uint8_t reg,
			    uint16_t val);
static void wm8904_read_reg(const struct device *dev, uint8_t reg,
			   uint16_t *val);
static void wm8904_update_reg(const struct device *dev, uint8_t reg,
			     uint16_t mask, uint16_t val);
static void wm8904_soft_reset(const struct device *dev);

static void wm8904_configure_output(const struct device *dev);

static void wm8904_configure_input(const struct device *dev);

static int wm8904_initialize(const struct device *dev)
{
	int error;
	const struct wm8904_driver_config *const dev_cfg = DEV_CFG(dev);

	error = pinctrl_apply_state(dev_cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (error) {
		LOG_ERR("WM8904 init PIN ERROR!");
		return error;
	}

	LOG_DBG("WM8904 init");
	i3c_recover_bus(dev_cfg->i2c.bus);

	return 0;
}

static int wm8904_protcol_config(const struct device *dev,
			       audio_dai_type_t dai_type)
{
	wm8904_protocol_t protocol;

	switch (dai_type) {
	case AUDIO_DAI_TYPE_I2S:
		protocol = kWM8904_ProtocolI2S;
		break;
	case AUDIO_DAI_TYPE_LEFT_JUSTIFIED:
		protocol = kWM8904_ProtocolLeftJustified;
		break;
	case AUDIO_DAI_TYPE_RIGHT_JUSTIFIED:
		protocol = kWM8904_ProtocolRightJustified;
		break;
	case AUDIO_DAI_TYPE_PCMA:
		protocol = kWM8904_ProtocolPCMA;
		break;
	case AUDIO_DAI_TYPE_PCMB:
		protocol = kWM8904_ProtocolPCMB;
		break;
	default:
		return -EINVAL;
	}

	wm8904_update_reg(dev, WM8904_AUDIO_IF_1,
			 (0x0003U | (1U << 4U)),
			 (uint16_t)protocol);

	LOG_DBG("protocol is %#x", protocol);
	return 0;
}

static int wm8904_audio_fmt_config(const struct device *dev,
		audio_dai_cfg_t *cfg, uint32_t mclk)
{
	wm8904_sample_rate_t wm_sample_rate;
	uint32_t fs;
	uint16_t wmfs_ratio;
	uint16_t mclkDiv;
	uint16_t word_size = cfg->i2s.word_size;

	switch (cfg->i2s.frame_clk_freq) {
	case 8000:
		wm_sample_rate = kWM8904_SampleRate8kHz;
		break;
	case  12000:
		wm_sample_rate = kWM8904_SampleRate12kHz;
		break;
	case 16000:
		wm_sample_rate = kWM8904_SampleRate16kHz;
		break;
	case 24000:
		wm_sample_rate = kWM8904_SampleRate24kHz;
		break;
	case 32000:
		wm_sample_rate = kWM8904_SampleRate32kHz;
		break;
	case 48000:
		wm_sample_rate = kWM8904_SampleRate48kHz;
		break;
	case 11025:
		wm_sample_rate = kWM8904_SampleRate11025Hz;
		break;
	case 22050:
		wm_sample_rate = kWM8904_SampleRate22050Hz;
		break;
	case 44100:
		wm_sample_rate = kWM8904_SampleRate44100Hz;
		break;
	default:
		LOG_WRN("sample_rate invalid %d", cfg->i2s.frame_clk_freq);
		return -EINVAL;
	}

	wm8904_read_reg(dev, WM8904_CLK_RATES_0, &mclkDiv);
	fs = (mclk >> (mclkDiv & 0x1U)) / cfg->i2s.frame_clk_freq;

	switch (fs) {
	case 64:
		wmfs_ratio = kWM8904_FsRatio64X;
		break;
	case 128:
		wmfs_ratio = kWM8904_FsRatio128X;
		break;
	case 192:
		wmfs_ratio = kWM8904_FsRatio192X;
		break;
	case 256:
		wmfs_ratio = kWM8904_FsRatio256X;
		break;
	case 384:
		wmfs_ratio = kWM8904_FsRatio384X;
		break;
	case 512:
		wmfs_ratio = kWM8904_FsRatio512X;
		break;
	case 768:
		wmfs_ratio = kWM8904_FsRatio768X;
		break;
	case 1024:
		wmfs_ratio = kWM8904_FsRatio1024X;
		break;
	case 1408:
		wmfs_ratio = kWM8904_FsRatio1408X;
		break;
	case 1536:
		wmfs_ratio = kWM8904_FsRatio1536X;
		break;
	default:
		LOG_WRN("fs Ratio invalid %d", fs);
		return -EINVAL;
	}

	LOG_DBG("fs Ratio set to %d", wmfs_ratio);
	/* Disable SYSCLK */
	wm8904_write_reg(dev, WM8904_CLK_RATES_2, 0x00);

	/* Set Clock ratio and sample rate */
	wm8904_write_reg(dev, WM8904_CLK_RATES_1,
		((wmfs_ratio) << 10U) | (uint16_t)(wm_sample_rate));

	switch (cfg->i2s.word_size) {
	case 16:
		word_size = 0;
		break;
	case 20:
		word_size = 1;
		break;
	case 24:
		word_size = 2;
		break;
	case 32:
		word_size = 3;
		break;
	default:
		LOG_ERR("word-size not support %d force to 16 bit", cfg->i2s.word_size);
		word_size = 0;
		break;
	}
	/* Set bit resolution */
	wm8904_update_reg(dev, WM8904_AUDIO_IF_1,
		(0x000CU), ((uint16_t)(word_size) << 2U));
	{
		uint16_t reg_val = 0;

		wm8904_read_reg(dev, WM8904_AUDIO_IF_1, &reg_val);
		LOG_INF("WM8904_AUDIO_IF_1 = %#x", reg_val);
	}

	/* Enable SYSCLK */
	wm8904_write_reg(dev, WM8904_CLK_RATES_2, 0x1007);
	return 0;
}

static int wm8904_out_volume_config(const struct device *dev, audio_channel_t channel, int volume)
{
	/**
	 * WM8904_ANALOG_OUT1_LEFT, WM8904_ANALOG_OUT1_RIGHT (headphone outs),
	 * WM8904_ANALOG_OUT2_LEFT and WM8904_ANALOG_OUT2_RIGHT (line outs):
	 * [8]   - MUTE: leaving intact
	 * [7]   - VU: 0 (values are "cached")
	 * [6]   - ZC: 1 (zero-crossing enabled)
	 * [5:0] - VOL: 6-bit volume value
	 */

	const uint16_t regval = (volume | 0b001000000) & 0b011111111;
	const uint16_t mask = 0b011111111;

	switch(channel)
	{
		case AUDIO_CHANNEL_FRONT_LEFT:
			wm8904_update_reg(dev, WM8904_ANALOG_OUT2_LEFT, mask, regval);
			return 0;
		
		case AUDIO_CHANNEL_FRONT_RIGHT:
			wm8904_update_reg(dev, WM8904_ANALOG_OUT2_RIGHT, mask, regval);
			return 0;

		case AUDIO_CHANNEL_HEADPHONE_LEFT:
			wm8904_update_reg(dev, WM8904_ANALOG_OUT1_LEFT, mask, regval);
			return 0;

		case AUDIO_CHANNEL_HEADPHONE_RIGHT:
			wm8904_update_reg(dev, WM8904_ANALOG_OUT1_RIGHT, mask, regval);
			return 0;
		
		case AUDIO_CHANNEL_ALL:
			/* Set volume values with VU = 0 */
			wm8904_update_reg(dev, WM8904_ANALOG_OUT1_LEFT, mask, regval);
			wm8904_update_reg(dev, WM8904_ANALOG_OUT1_RIGHT, mask, regval);
			wm8904_update_reg(dev, WM8904_ANALOG_OUT2_LEFT, mask, regval);
			wm8904_update_reg(dev, WM8904_ANALOG_OUT2_RIGHT, mask, regval);
			return 0;

		default:
			return -EINVAL;
	}
}

static int wm8904_out_mute_config(const struct device *dev, audio_channel_t channel, bool mute)
{
	/**
	 * WM8904_ANALOG_OUT1_LEFT, WM8904_ANALOG_OUT1_RIGHT (headphone outs),
	 * WM8904_ANALOG_OUT2_LEFT and WM8904_ANALOG_OUT2_RIGHT (line outs):
	 * [8]   - MUTE: mute bit
	 * [7]   - VU: 0 (values are "cached")
	 * [6]   - ZC: 1 (zero-crossing enabled)
	 * [5:0] - VOL: leaving intact
	 */

	const uint16_t regval = ((mute << 8) | 0b001000000);
	const uint16_t mask = 0b111000001;

	switch(channel)
	{
		case AUDIO_CHANNEL_FRONT_LEFT:
			wm8904_update_reg(dev, WM8904_ANALOG_OUT2_LEFT, mask, regval);
			return 0;
		
		case AUDIO_CHANNEL_FRONT_RIGHT:
			wm8904_update_reg(dev, WM8904_ANALOG_OUT2_RIGHT, mask, regval);
			return 0;

		case AUDIO_CHANNEL_HEADPHONE_LEFT:
			wm8904_update_reg(dev, WM8904_ANALOG_OUT1_LEFT, mask, regval);
			return 0;

		case AUDIO_CHANNEL_HEADPHONE_RIGHT:
			wm8904_update_reg(dev, WM8904_ANALOG_OUT1_RIGHT, mask, regval);
			return 0;

		case AUDIO_CHANNEL_ALL:
			wm8904_update_reg(dev, WM8904_ANALOG_OUT1_LEFT, mask, regval);
			wm8904_update_reg(dev, WM8904_ANALOG_OUT1_RIGHT, mask, regval);
			wm8904_update_reg(dev, WM8904_ANALOG_OUT2_LEFT, mask, regval);
			wm8904_update_reg(dev, WM8904_ANALOG_OUT1_RIGHT, mask, regval);
			return 0;

		default:
			return -EINVAL;
	}
}

static int wm8904_in_volume_config(const struct device *dev, audio_channel_t channel, int volume)
{
	/**
	 * WM8904_ANALOG_LEFT_IN_0, WM8904_ANALOG_RIGHT_IN_0:
	 * [7]   - MUTE: leaving intact
	 * [4:0] - VOL: 5 bit volume value
	 */

	const uint16_t regval = volume & 0b00011111;
	const uint16_t mask = 0b00011111;

	switch(channel)
	{
		case AUDIO_CHANNEL_FRONT_LEFT:
			wm8904_update_reg(dev, WM8904_ANALOG_LEFT_IN_0, mask, regval);
			return 0;
		
		case AUDIO_CHANNEL_FRONT_RIGHT:
			wm8904_update_reg(dev, WM8904_ANALOG_RIGHT_IN_0, mask, regval);
			return 0;
		
		case AUDIO_CHANNEL_ALL:
			wm8904_update_reg(dev, WM8904_ANALOG_LEFT_IN_0, mask, regval);
			wm8904_update_reg(dev, WM8904_ANALOG_RIGHT_IN_0, mask, regval);
			return 0;

		default:
			return -EINVAL;
	}
}

static int wm8904_in_mute_config(const struct device *dev, audio_channel_t channel, bool mute)
{
	/**
	 * WM8904_ANALOG_LEFT_IN_0, WM8904_ANALOG_RIGHT_IN_0:
	 * [7]   - MUTE: mute bit
	 * [4:0] - VOL: leaving intact
	 */

	const uint16_t regval = ((mute << 7) & 0b10000000);
	const uint16_t mask = 0b10000000;

	switch(channel)
	{
		case AUDIO_CHANNEL_FRONT_LEFT:
			wm8904_update_reg(dev, WM8904_ANALOG_LEFT_IN_0, mask, regval);
			return 0;
		
		case AUDIO_CHANNEL_FRONT_RIGHT:
			wm8904_update_reg(dev, WM8904_ANALOG_RIGHT_IN_0, mask, regval);
			return 0;

		case AUDIO_CHANNEL_ALL:
			wm8904_update_reg(dev, WM8904_ANALOG_LEFT_IN_0, mask, regval);
			wm8904_update_reg(dev, WM8904_ANALOG_RIGHT_IN_0, mask, regval);
			return 0;

		default:
			return -EINVAL;
	}
}

static int wm8904_route_input(const struct device *dev,
			audio_channel_t channel,
			uint32_t input)
{
	if(input < 1 || input > 3)
		return -EINVAL;

	uint8_t reg;
	switch(channel)
	{
		case AUDIO_CHANNEL_FRONT_LEFT:
			reg = WM8904_ANALOG_LEFT_IN_1;
			break;

		case AUDIO_CHANNEL_FRONT_RIGHT:
			reg = WM8904_ANALOG_RIGHT_IN_1;
			break;

		default:
			return -EINVAL;
	}

	uint8_t val = 0b0000000 | (((input - 1) & 0b11) << 4) | (((input - 1) & 0b11) << 2);
	uint8_t mask = 0b1111111;

	wm8904_update_reg(dev, reg, mask, val);
	return 0;
}

static void wm8904_set_master_clock(const struct device *dev,
				audio_dai_cfg_t *cfg, uint32_t sysclk)
{
	uint32_t sampleRate     = cfg->i2s.frame_clk_freq;
	uint32_t bitWidth       = cfg->i2s.word_size;
	uint32_t bclk           = sampleRate * bitWidth * 2U;
	uint32_t bclkDiv        = 0U;
	uint16_t audioInterface = 0U;
	uint16_t sysclkDiv      = 0U;

	wm8904_read_reg(dev, WM8904_CLK_RATES_0, &sysclkDiv);
	sysclk = sysclk >> (sysclkDiv & 0x1U);
	LOG_INF("codec system clk %d", sysclk);

	if ((sysclk / bclk > 48U) || (bclk / sampleRate > 2047U) || (bclk / sampleRate < 8U)) {
		LOG_ERR("Invalid BCLK clock divider configured.");
		return;
	}

	wm8904_read_reg(dev, WM8904_AUDIO_IF_2, &audioInterface);

	audioInterface &= ~(uint16_t)0x1FU;
	bclkDiv = (sysclk * 10U) / bclk;
	LOG_INF("blk %d", bclk);

	switch (bclkDiv) {
	case 10:
		audioInterface |= 0U;
		break;
	case 15:
		audioInterface |= 1U;
		break;
	case 20:
		/* Avoid MISRA 16.4 violation */
		break;
	case 30:
		/* Avoid MISRA 16.4 violation */
		break;
	case 40:
		/* Avoid MISRA 16.4 violation */
		break;
	case 50:
		audioInterface |= (uint16_t)bclkDiv / 10U;
		break;
	case 55:
		audioInterface |= 6U;
		break;
	case 60:
		audioInterface |= 7U;
		break;
	case 80:
		audioInterface |= 8U;
		break;
	case 100:
		/* Avoid MISRA 16.4 violation */
		break;
	case 110:
		/* Avoid MISRA 16.4 violation */
		break;
	case 120:
		audioInterface |= (uint16_t)bclkDiv / 10U - 1U;
		break;
	case 160:
		audioInterface |= 12U;
		break;
	case 200:
		audioInterface |= 13U;
		break;
	case 220:
		audioInterface |= 14U;
		break;
	case 240:
		audioInterface |= 15U;
		break;
	case 250:
		audioInterface |= 16U;
		break;
	case 300:
		audioInterface |= 17U;
		break;
	case 320:
		audioInterface |= 18U;
		break;
	case 440:
		audioInterface |= 19U;
		break;
	case 480:
		audioInterface |= 20U;
		break;
	default:
		LOG_ERR("invalid audio interface for wm8904 %d", bclkDiv);
		return;
	}

	/* bclk divider */
	wm8904_write_reg(dev, WM8904_AUDIO_IF_2, audioInterface);
	/* bclk direction output */
	wm8904_update_reg(dev, WM8904_AUDIO_IF_1, 1U << 6U, 1U << 6U);

	wm8904_update_reg(dev, WM8904_GPIO_CONTROL_4, 0x8FU, 1U);
	/* LRCLK direction and divider */
	audioInterface = (uint16_t)((1UL << 11U) | (bclk / sampleRate));
	wm8904_update_reg(dev, WM8904_AUDIO_IF_3, 0xFFFU, audioInterface);
}

static int wm8904_configure(const struct device *dev,
			   struct audio_codec_cfg *cfg)
{
	uint16_t value;
	const struct wm8904_driver_config *const dev_cfg = DEV_CFG(dev);

	if (cfg->dai_type >= AUDIO_DAI_TYPE_INVALID) {
		LOG_ERR("dai_type not supported");
		return -EINVAL;
	}

	wm8904_soft_reset(dev);
	
	if(cfg->dai_route == AUDIO_ROUTE_BYPASS)
		return 0;
	
	/* MCLK_INV=0, SYSCLK_SRC=0, TOCLK_RATE=0, OPCLK_ENA=1,
	 * CLK_SYS_ENA=1, CLK_DSP_ENA=1, TOCLK_ENA=1
	 */
	wm8904_write_reg(dev, WM8904_CLK_RATES_2, 0x000F);

	/* WSEQ_ENA=1, WSEQ_WRITE_INDEX=0_0000 */
	wm8904_write_reg(dev, WM8904_WRT_SEQUENCER_0, 0x0100);

	/* WSEQ_ABORT=0, WSEQ_START=1, WSEQ_START_INDEX=00_0000 */
	wm8904_write_reg(dev, WM8904_WRT_SEQUENCER_3, 0x0100);

	do {
		wm8904_read_reg(dev, WM8904_WRT_SEQUENCER_4, &value);
	} while (((value & 1U) != 0U));

	/* TOCLK_RATE_DIV16=0, TOCLK_RATE_x4=1, SR_MODE=0, MCLK_DIV=1
	 * (Required for MMCs: SGY, KRT see erratum CE000546)
	 */
	wm8904_write_reg(dev, WM8904_CLK_RATES_0, 0xA45F);

	/* INL_ENA=1, INR ENA=1 */
	wm8904_write_reg(dev, WM8904_POWER_MGMT_0, 0x0003);

	/* HPL_PGA_ENA=1, HPR_PGA_ENA=1 */
	wm8904_write_reg(dev, WM8904_POWER_MGMT_2, 0x0003);

	/* DACL_ENA=1, DACR_ENA=1, ADCL_ENA=1, ADCR_ENA=1 */
	wm8904_write_reg(dev, WM8904_POWER_MGMT_6, 0x000F);

	/* ADC_OSR128=1 */
	wm8904_write_reg(dev, WM8904_ANALOG_ADC_0, 0x0001);

	/* DACL_DATINV=0, DACR_DATINV=0, DAC_BOOST=00, LOOPBACK=0, AIFADCL_SRC=0,
	 * AIFADCR_SRC=1, AIFDACL_SRC=0, AIFDACR_SRC=1, ADC_COMP=0, ADC_COMPMODE=0,
	 * DAC_COMP=0, DAC_COMPMODE=0
	 */
	wm8904_write_reg(dev, WM8904_AUDIO_IF_0, 0x0050);

	/* DAC_MONO=0, DAC_SB_FILT-0, DAC_MUTERATE=0, DAC_UNMUTE RAMP=0,
	 * DAC_OSR128=1, DAC_MUTE=0, DEEMPH=0 (none)
	 */
	wm8904_write_reg(dev, WM8904_DAC_DIG_1, 0x0040);

	/* Enable DC servos for headphone out */
	wm8904_write_reg(dev, WM8904_DC_SERVO_0, 0x0003);

	/* HPL_RMV_SHORT=1, HPL_ENA_OUTP=1, HPL_ENA_DLY=1, HPL_ENA=1,
	 * HPR_RMV_SHORT=1, HPR_ENA_OUTP=1, HPR_ENA_DLY=1, HPR_ENA=1
	 */
	wm8904_write_reg(dev, WM8904_ANALOG_HP_0, 0x00FF);

	/* CP_DYN_PWR=1 */
	wm8904_write_reg(dev, WM8904_CLS_W_0, 0x0001);

	/* CP_ENA=1 */
	wm8904_write_reg(dev, WM8904_CHRG_PUMP_0, 0x0001);

	wm8904_protcol_config(dev, cfg->dai_type);
	wm8904_update_reg(dev, WM8904_CLK_RATES_2,
		(uint16_t)(1UL << 14U), (uint16_t)(dev_cfg->clock_source));
	
	if(dev_cfg->clock_source == 0)
	{
		int err = clock_control_on(dev_cfg->mclk_dev, dev_cfg->mclk_name);
		if(err < 0)
		{
			LOG_ERR("MCLK clock source enable fail: %d", err);
		}

		err = clock_control_get_rate(dev_cfg->mclk_dev, dev_cfg->mclk_name, &cfg->mclk_freq);
		if(err < 0)
		{
			LOG_ERR("MCLK clock source freq acquire fail: %d", err);
		}
	}

	wm8904_audio_fmt_config(dev, &cfg->dai_cfg, cfg->mclk_freq);

	if ((cfg->dai_cfg.i2s.options & I2S_OPT_FRAME_CLK_MASTER) ==
		I2S_OPT_FRAME_CLK_MASTER) {
		wm8904_set_master_clock(dev, &cfg->dai_cfg, cfg->mclk_freq);
	} else {
		/* BCLK/LRCLK default direction input */
		wm8904_update_reg(dev, WM8904_AUDIO_IF_1, 1U << 6U, 0U);
		wm8904_update_reg(dev, WM8904_AUDIO_IF_3, (uint16_t)(1UL << 11U), 0U);
	}

	switch(cfg->dai_route)
	{
		case AUDIO_ROUTE_PLAYBACK:
			wm8904_configure_output(dev);
			break;

		case AUDIO_ROUTE_CAPTURE:
			wm8904_configure_input(dev);
			break;

		case AUDIO_ROUTE_PLAYBACK_CAPTURE:
			wm8904_configure_output(dev);
			wm8904_configure_input(dev);
			break;

		default:
			break;
	}

	return 0;
}

static void wm8904_start_output(const struct device *dev)
{
}

static void wm8904_stop_output(const struct device *dev)
{
}

static int wm8904_set_property(const struct device *dev,
			      audio_property_t property,
			      audio_channel_t channel,
			      audio_property_value_t val)
{
	switch(property)
	{
		case AUDIO_PROPERTY_OUTPUT_VOLUME:
			return wm8904_out_volume_config(dev, channel, val.vol);

		case AUDIO_PROPERTY_OUTPUT_MUTE:
			return wm8904_out_mute_config(dev, channel, val.mute);
 
		case AUDIO_PROPERTY_INPUT_VOLUME:
			return wm8904_in_volume_config(dev, channel, val.vol);

		case AUDIO_PROPERTY_INPUT_MUTE:
			return wm8904_in_mute_config(dev, channel, val.mute);
	}

	return -EINVAL;
}

static int wm8904_apply_properties(const struct device *dev)
{
	/**
	 * Set VU = 1 for all output channels, VU takes effect for the whole
	 * channel pair.
	 */
	wm8904_update_reg(dev, WM8904_ANALOG_OUT1_LEFT, 0b010000000, 0b010000000);
	wm8904_update_reg(dev, WM8904_ANALOG_OUT2_LEFT, 0b010000000, 0b010000000);

	return 0;
}

static void wm8904_write_reg(const struct device *dev, uint8_t reg, uint16_t val)
{
	const struct wm8904_driver_config *const dev_cfg = DEV_CFG(dev);
	uint8_t data[3];
	int ret;

	/* data is reversed */
	data[0] = reg;
	data[1] = (val >> 8) & 0xff;
	data[2] = val & 0xff;

	ret = i2c_write(dev_cfg->i2c.bus, data, 3, dev_cfg->i2c.addr);

	if (ret != 0) {
		LOG_ERR("i2c write to codec error %d", ret);
	}

	LOG_DBG("REG:%02u VAL:%#02x", reg, val);
}

static void wm8904_read_reg(const struct device *dev, uint8_t reg, uint16_t *val)
{
	const struct wm8904_driver_config *const dev_cfg = DEV_CFG(dev);
	uint16_t value;
	int ret;

	ret = i2c_write_read(dev_cfg->i2c.bus, dev_cfg->i2c.addr, &reg,
		sizeof(reg), &value, sizeof(value));
	if (ret == 0) {
		*val = (value>>8)&0xff;
		*val += ((value & 0xff)<<8);
		/* update cache*/
		LOG_DBG("REG:%02u VAL:%#02x", reg, *val);
	}
}

static void wm8904_update_reg(const struct device *dev, uint8_t reg,
			     uint16_t mask, uint16_t val)
{
	uint16_t reg_val = 0;
	uint16_t new_value = 0;

	if (reg == 0x19) {
		LOG_INF("try write mask %#x val %#x", mask, val);
	}
	wm8904_read_reg(dev, reg, &reg_val);
	if (reg == 0x19) {
		LOG_INF("read %#x = %x", reg, reg_val);
	}
	new_value = (reg_val & ~mask) | (val & mask);
	if (reg == 0x19) {
		LOG_INF("write %#x = %x", reg, new_value);
	}
	wm8904_write_reg(dev, reg, new_value);
}

static void wm8904_soft_reset(const struct device *dev)
{
	wm8904_write_reg(dev, WM8904_RESET, 0x00);
}

static void wm8904_configure_output(const struct device *dev)
{
	wm8904_out_volume_config(dev, AUDIO_CHANNEL_ALL, OUTPUT_VOLUME_DEFAULT);
	wm8904_out_mute_config(dev, AUDIO_CHANNEL_ALL, false);
	
	wm8904_apply_properties(dev);
}

static void wm8904_configure_input(const struct device *dev)
{
	/*
	 * INL_CM_ENA=0, L_IP_SEL_N=01, L_IP_SEL_P=01, L_MODE=00
	 * INR_CM_ENA=0, R_IP_SEL_N=01, R_IP_SEL_P=01, R_MODE=00
	 */

	/*wm8904_write_reg(dev, WM8904_ANALOG_LEFT_IN_1, 0b0010100);
	wm8904_write_reg(dev, WM8904_ANALOG_RIGHT_IN_1, 0b0010100);*/

	wm8904_route_input(dev, AUDIO_CHANNEL_FRONT_LEFT, 2);
	wm8904_route_input(dev, AUDIO_CHANNEL_FRONT_RIGHT, 2);

	wm8904_in_volume_config(dev, AUDIO_CHANNEL_ALL, INPUT_VOLUME_DEFAULT);
	wm8904_in_mute_config(dev, AUDIO_CHANNEL_ALL, false);
}

static const struct audio_codec_api wm8904_driver_api = {
	.configure = wm8904_configure,
	.start_output = wm8904_start_output,
	.stop_output = wm8904_stop_output,
	.set_property = wm8904_set_property,
	.apply_properties = wm8904_apply_properties,
	.route_input = wm8904_route_input
};

#define WM8904_INIT(n)							\
	PINCTRL_DT_INST_DEFINE(n);					\
	static const struct wm8904_driver_config wm8904_device_config_##n = {	\
		.i2c = I2C_DT_SPEC_INST_GET(n),				\
		.clock_source =	DT_INST_PROP_OR(n, clk_source, 0),	\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		\
		.mclk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, mclk)), \
		.mclk_name = (clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_NAME(n, mclk, name) \
	};								\
									\
	DEVICE_DT_INST_DEFINE(n, &wm8904_initialize,	\
			      NULL,			\
			      NULL,	\
			      &wm8904_device_config_##n,	\
			      POST_KERNEL,		\
			      CONFIG_AUDIO_CODEC_INIT_PRIORITY,	\
			      &wm8904_driver_api);	\

DT_INST_FOREACH_STATUS_OKAY(WM8904_INIT)
