/*
 * File: haptic_nv.c
 *
 * Author: <chelvming@awinic.com>
 *
 * Copyright (c) 2021 AWINIC Technology CO., LTD
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include "main.h"
#include "stdio.h"
#include "haptic_nv_reg.h"
#include "haptic_nv.h"

#define HAPTIC_NV_DRIVER_VERSION "v0.4.0"


struct aw_haptic_func *g_func_haptic_nv = NULL;

struct haptic_nv haptic_nv = {
	.i2c_addr = AW862XX_I2C_ADDR,
	.gain = 0x80,
	.timer_ms_cnt = 0,
	.name = AW_CHIP_NULL,
	.is_used_irq_pin = AW_FALSE,
	.ram_vbat_comp = AW_RAM_VBAT_COMP_ENABLE,
};

static int get_base_addr(void)
{
	uint16_t last_end = 0;
	uint16_t next_start = 0;
	int i = 0;
	int ram_num = 1;

	for (i = 3; i < haptic_nv.aw_fw.len; i = i + 4)
	{
		last_end = (haptic_nv.aw_fw.data[i] << 8) | haptic_nv.aw_fw.data[i + 1];
		next_start = (haptic_nv.aw_fw.data[i + 2] << 8) | haptic_nv.aw_fw.data[i + 3];
		if ((next_start - last_end) == 1)
			ram_num++;
		else
			break;
	}

	for (i = ram_num * 4; i >= 4; i = i - 4)
	{
		last_end = (haptic_nv.aw_fw.data[i - 1] << 8) | haptic_nv.aw_fw.data[i];
		haptic_nv.ram.base_addr = (int)((haptic_nv.aw_fw.data[1] << 8) | haptic_nv.aw_fw.data[2]) - ram_num * 4 - 1;
		if ((last_end - haptic_nv.ram.base_addr + 1) == haptic_nv.aw_fw.len)
		{
			AW_LOGI("base_addr = 0x%04x", haptic_nv.ram.base_addr);
			return AW_SUCCESS;
		}
		else
		{
			ram_num--;
		}
	}
	return AW_ERROR;
}

static void ram_vbat_comp(aw_bool flag)
{
	int temp_gain = 0;

	if (flag)
	{
		if (haptic_nv.ram_vbat_comp == AW_RAM_VBAT_COMP_ENABLE)
		{

			g_func_haptic_nv->get_vbat();
			temp_gain = haptic_nv.gain * AW_VBAT_REFER / haptic_nv.vbat;
			if (temp_gain > (AW_DEFAULT_GAIN * AW_VBAT_REFER / AW_VBAT_MIN))
			{
				temp_gain = AW_DEFAULT_GAIN * AW_VBAT_REFER / AW_VBAT_MIN;
				AW_LOGI("gain limit=%d", temp_gain);
			}
			g_func_haptic_nv->set_gain(temp_gain);
			AW_LOGI("ram vbat comp open");
		}
		else
		{
			g_func_haptic_nv->set_gain(haptic_nv.gain);
			AW_LOGI("ram vbat comp close");
		}
	}
	else
	{
		g_func_haptic_nv->set_gain(haptic_nv.gain);
		AW_LOGI("ram vbat comp close");
	}
}

static void upload_lra(uint8_t flag)
{
	uint8_t reg_val;

	switch (flag)
	{
	case AW_WRITE_ZERO:
		AW_LOGI("write zero to trim_lra!");
		reg_val = 0x00;
		break;
	case AW_F0_CALI_LRA:
		AW_LOGI("write f0_cali_data to trim_lra = 0x%02X", haptic_nv.f0_cali_data);
		reg_val = haptic_nv.f0_cali_data;
		break;
	default:
		AW_LOGE("flag is error");
		reg_val = 0x00;
		break;
	}
	g_func_haptic_nv->set_trim_lra(reg_val);
}

static int long_vib_work(uint8_t index, uint32_t duration)
{
	if (!haptic_nv.ram_init)
	{
		AW_LOGE("ram init faild, ram_num = 0!");
		return AW_ERROR;
	}
	if ((duration == 0) || (index == 0))
	{
		AW_LOGE("duration = %d, index = %d, err", duration, index);
		return AW_ERROR;
	}

	AW_LOGI("start duration = %d, index = %d", duration, index);

	if (haptic_nv.timer_ms_cnt != 0)
	{
		/* stop long_vib_work */
		haptic_nv_stop_hrtimer();
		haptic_nv.timer_ms_cnt = 0;
	}

	g_func_haptic_nv->play_stop();
	haptic_nv.index = index;
	haptic_nv.duration = duration;
	upload_lra(AW_F0_CALI_LRA);
	ram_vbat_comp(AW_TRUE);
	g_func_haptic_nv->set_repeat_seq(haptic_nv.index);
	g_func_haptic_nv->play_mode(AW_RAM_LOOP_MODE);
	g_func_haptic_nv->haptic_start();
	haptic_nv_start_hrtimer();
	return AW_SUCCESS;
}

static int short_vib_work(uint8_t index, uint8_t gain, uint8_t loop)
{
	if (!haptic_nv.ram_init)
	{
		AW_LOGE("ram init faild, ram_num = 0!");
		return AW_ERROR;
	}
	if ((loop >= AW_LOOP_NUM_MAX) || (index == 0) || (index > haptic_nv.ram.ram_num))
	{
		AW_LOGE("loop = %d, index = %d, err", loop, index);
		return AW_ERROR;
	}

	AW_LOGI("start loop = %d, index = %d", loop, index);

	if (haptic_nv.timer_ms_cnt != 0)
	{
		/* stop long_vib_work */
		haptic_nv_stop_hrtimer();
		haptic_nv.timer_ms_cnt = 0;
	}

	g_func_haptic_nv->play_stop();
	haptic_nv.index = index;
	upload_lra(AW_F0_CALI_LRA);
	ram_vbat_comp(AW_FALSE);
	g_func_haptic_nv->set_wav_seq(0x00, haptic_nv.index);
	g_func_haptic_nv->set_wav_seq(0x01, 0x00);
	g_func_haptic_nv->set_wav_loop(0x00, loop - 1);
	g_func_haptic_nv->set_gain(gain);
	g_func_haptic_nv->play_mode(AW_RAM_MODE);
	g_func_haptic_nv->haptic_start();
	return AW_SUCCESS;
}

static int judge_within_cali_range(void)
{
	uint32_t f0_cali_min = 0;
	uint32_t f0_cali_max = 0;

	f0_cali_min = haptic_nv.info->f0_pre * (100 - haptic_nv.info->f0_cali_percent) / 100;
	f0_cali_max = haptic_nv.info->f0_pre * (100 + haptic_nv.info->f0_cali_percent) / 100;

	AW_LOGI("f0_pre = %d, f0_cali_min = %d, f0_cali_max = %d, f0 = %d",
			haptic_nv.info->f0_pre, f0_cali_min, f0_cali_max, haptic_nv.f0);

	if (haptic_nv.f0 < f0_cali_min)
	{
		AW_LOGE("lra f0 is too small, lra_f0 = %d!", haptic_nv.f0);
		return AW_ERROR;
	}

	if (haptic_nv.f0 > f0_cali_max)
	{
		AW_LOGE("lra f0 is too large, lra_f0 = %d!", haptic_nv.f0);
		return AW_ERROR;
	}
	return AW_SUCCESS;
}

static int f0_cali(void)
{
	int ret = 0;

	AW_LOGI("enter");
	upload_lra(AW_WRITE_ZERO);
	if (g_func_haptic_nv->get_f0())
	{
		AW_LOGE("get f0 error, user defafult f0");
	}
	else
	{
		/* max and min limit */
		ret = judge_within_cali_range();
		if (ret != AW_SUCCESS)
			return AW_ERROR;
		/* calculate cali step */
		g_func_haptic_nv->calculate_cali_data();
	}
	upload_lra(AW_F0_CALI_LRA);
	g_func_haptic_nv->play_stop();
#ifndef AW_F0_CALI_DURING_STARTUP
	haptic_nv_set_cali_to_flash();
#endif
	return ret;
}

static void f0_show(void)
{
	upload_lra(AW_WRITE_ZERO);
	g_func_haptic_nv->get_f0();
	upload_lra(AW_F0_CALI_LRA);
}

static void cali_show(void)
{
	upload_lra(AW_F0_CALI_LRA);
	g_func_haptic_nv->get_f0();
}

static int write_rtp_data(void)
{
	uint32_t buf_len = 0;
	uint32_t rtp_len = haptic_nv_rtp_len;
	uint32_t rtp_cnt = haptic_nv.rtp_cnt;
	uint32_t base_addr = haptic_nv.ram.base_addr;

	if (!rtp_len)
	{
		AW_LOGI("rtp_data is null");
		return AW_ERROR;
	}

#ifdef AW_ENABLE_RTP_PRINT_LOG
	AW_LOGI("rtp mode fifo update, cnt=%d", haptic_nv.rtp_cnt);
#endif

	if (rtp_cnt < base_addr)
	{
		if ((rtp_len - rtp_cnt) < base_addr)
			buf_len = rtp_len - rtp_cnt;
		else
			buf_len = base_addr;
	}
	else if ((rtp_len - rtp_cnt) < (base_addr >> 2))
	{
		buf_len = rtp_len - rtp_cnt;
	}
	else
	{
		buf_len = base_addr >> 2;
	}

#ifdef AW_ENABLE_RTP_PRINT_LOG
	AW_LOGI("buf_len = %d", buf_len);
#endif

	g_func_haptic_nv->set_rtp_data(&(haptic_nv_rtp_data[haptic_nv.rtp_cnt]), buf_len);
	haptic_nv.rtp_cnt += buf_len;
	return AW_SUCCESS;
}

static int judge_rtp_load_end(void)
{
	uint8_t glb_st = 0;
	int ret = AW_ERROR;

	glb_st = g_func_haptic_nv->get_glb_state();

	if ((haptic_nv.rtp_cnt == haptic_nv_rtp_len) ||
		((glb_st & AW_BIT_GLBRD_STATE_MASK) == AW_BIT_STATE_STANDBY))
	{
		if (haptic_nv.rtp_cnt != haptic_nv_rtp_len)
			AW_LOGE("rtp play suspend!");
		else
			AW_LOGI("rtp update complete!,cnt=%d", haptic_nv.rtp_cnt);
		haptic_nv.rtp_cnt = 0;
		haptic_nv.rtp_init = AW_FALSE;
		g_func_haptic_nv->set_rtp_aei(AW_FALSE);
		ret = AW_SUCCESS;
	}

	return ret;
}

static int rtp_going(void)
{
	int ret = 0;

	AW_LOGI("enter mode %d", haptic_nv.play_mode);

	haptic_nv.rtp_cnt = 0;

	while (!g_func_haptic_nv->rtp_get_fifo_afs() && (haptic_nv.play_mode == AW_RTP_MODE))
	{
		ret = write_rtp_data();
		if (ret == AW_ERROR)
			return ret;
		ret = judge_rtp_load_end();
		if (ret == AW_SUCCESS)
			return ret;
	}

	if (haptic_nv.play_mode == AW_RTP_MODE)
		g_func_haptic_nv->set_rtp_aei(AW_TRUE);

	AW_LOGI("cnt = %d, exit", haptic_nv.rtp_cnt);
	return AW_SUCCESS;
}

#ifdef AW_IRQ_CONFIG
static void irq_handle(void)
{
	int ret = 0;
	int irq_state = 0;

	AW_LOGI("enter");
	haptic_nv.irq_handle = AW_IRQ_OFF;
	haptic_nv_disable_irq();
	do
	{
		irq_state = g_func_haptic_nv->get_irq_state();

		if (haptic_nv.is_supported_rtp == AW_FALSE)
			break;

		if (irq_state == AW_IRQ_ALMOST_EMPTY)
		{
			if (haptic_nv.rtp_init)
			{
				while ((!g_func_haptic_nv->rtp_get_fifo_afs()) && (haptic_nv.play_mode == AW_RTP_MODE))
				{
					if (!haptic_nv.rtp_cnt)
					{
						AW_LOGI("haptic_nv.rtp_cnt is 0!");
						break;
					}

					ret = write_rtp_data();
					if (ret == AW_ERROR)
						break;
					ret = judge_rtp_load_end();
					if (ret == AW_SUCCESS)
						break;
				}
			}
			else
			{
				AW_LOGI("rtp_init: %d", haptic_nv.rtp_init);
			}
		}
		if (haptic_nv.play_mode != AW_RTP_MODE)
			g_func_haptic_nv->set_rtp_aei(AW_FALSE);
	} while (irq_state != AW_IRQ_NULL);
	AW_LOGI("exit");
	haptic_nv_enable_irq();
}

static void set_hw_irq_status(uint8_t aw_hw_irq_handle)
{
	haptic_nv.irq_handle = aw_hw_irq_handle;
}

static uint8_t get_hw_irq_status(void)
{
	return haptic_nv.irq_handle;
}
#endif

static int wait_enter_rtp_mode(int cnt)
{
	aw_bool rtp_work_flag = AW_FALSE;
	uint8_t ret = 0;

	while (cnt)
	{
		ret = g_func_haptic_nv->judge_rtp_going();
		if (ret == AW_SUCCESS)
		{
			rtp_work_flag = AW_TRUE;
			AW_LOGI("RTP_GO!");
			break;
		}
		cnt--;
		AW_LOGI("wait for RTP_GO, glb_state=0x%02X", ret);
		haptic_nv_mdelay(AW_RTP_DELAY);
	}

	if (!rtp_work_flag)
	{
		g_func_haptic_nv->play_stop();
		AW_LOGE("failed to enter RTP_GO status!");
		return AW_ERROR;
	}

	return AW_SUCCESS;
}

static void rtp_vib_work(void)
{
	int ret = 0;

	if (!haptic_nv.is_supported_rtp)
	{
		AW_LOGE("chip is not support rtp function.");
		return;
	}

	AW_LOGI("rtp file size = %d", haptic_nv_rtp_len);

	haptic_nv.rtp_init = AW_TRUE;
	g_func_haptic_nv->play_stop();
	g_func_haptic_nv->set_rtp_aei(AW_FALSE);
	g_func_haptic_nv->irq_clear();
	ram_vbat_comp(AW_FALSE);
	g_func_haptic_nv->play_mode(AW_RTP_MODE);
	upload_lra(AW_WRITE_ZERO);
	g_func_haptic_nv->haptic_start();
	haptic_nv_mdelay(AW_RTP_DELAY);
	ret = wait_enter_rtp_mode(200);
	if (ret == AW_ERROR)
		return;
	rtp_going();
}

static void get_ram_num(void)
{
	uint32_t first_wave_addr = 0;

	if (!haptic_nv.ram_init)
	{
		AW_LOGE("ram init faild, ram_num = 0!");
		return;
	}

	first_wave_addr = (haptic_nv.aw_fw.data[1] << 8) | haptic_nv.aw_fw.data[2];
	haptic_nv.ram.ram_num = (first_wave_addr - haptic_nv.ram.base_addr - 1) / 4;
	AW_LOGI("ram num = %d", haptic_nv.ram.ram_num);
}

static void ram_show(void)
{
	int i = 0;
	int j = 0;
	int size = 0;
	uint8_t ram_data[AW_RAMDATA_RD_BUFFER_SIZE] = {0};

	g_func_haptic_nv->ram_init(AW_TRUE);
	g_func_haptic_nv->play_stop();
	g_func_haptic_nv->set_ram_addr();
	printf("aw_haptic_ram:\r\n");
	while (i < haptic_nv.ram.len)
	{
		if ((haptic_nv.ram.len - i) < AW_RAMDATA_RD_BUFFER_SIZE)
			size = haptic_nv.ram.len - i;
		else
			size = AW_RAMDATA_RD_BUFFER_SIZE;

		g_func_haptic_nv->get_ram_data(ram_data, size);
		for (j = 0; j < size; j++)
		{
			printf("0x%02X,", ram_data[j]);
			if ((j + 1) % 16 == 0)
				printf("\r\n");
		}
		i += size;
	}
	g_func_haptic_nv->ram_init(AW_FALSE);
	printf("\r\n");
}

static void sw_reset(void)
{
	uint8_t reset = AW_SOFT_RESET;

	AW_LOGI("enter!");
	haptic_nv_i2c_writes(AW_REG_ID, &reset, AW_I2C_BYTE_ONE);
	haptic_nv_mdelay(2);
}

static void hw_reset(void)
{
	haptic_nv_pin_control(AW_RST_Pin, AW_PIN_LOW);
	haptic_nv_mdelay(2);
	haptic_nv_pin_control(AW_RST_Pin, AW_PIN_HIGH);
	haptic_nv_mdelay(8);
}

static int parse_chipid(void)
{
	uint8_t ef_id = 0;
	uint8_t cnt = 0;
	uint8_t reg = 0;
	int ret = AW_SUCCESS;

	for (cnt = 0; cnt < AW_READ_CHIPID_RETRIES; cnt++)
	{
		ret = haptic_nv_read_chipid(&reg, AW_FIRST_TRY);
		if (ret != AW_SUCCESS)
		{
			haptic_nv.i2c_addr = AW862XX_I2C_ADDR;
			AW_LOGI("try to replace i2c addr [(0x%02X)] to read chip id again",
					haptic_nv.i2c_addr);
			ret = haptic_nv_read_chipid(&reg, AW_LAST_TRY);
			if (ret != AW_SUCCESS)
				break;
		}

		switch (reg)
		{
		case AW8624_CHIP_ID:
			haptic_nv.name = AW8624;
			AW_LOGI("detected aw8624.");
			return AW_SUCCESS;
		case AW8622X_CHIP_ID:
			haptic_nv_i2c_reads(AW862XX_REG_EFRD9, &ef_id, AW_I2C_BYTE_ONE);
			if ((ef_id & 0x41) == AW86223_EF_ID)
			{
				haptic_nv.name = AW86223;
				AW_LOGI("aw86223 detected");
				return AW_SUCCESS;
			}
			if ((ef_id & 0x41) == AW86224_EF_ID)
			{
				haptic_nv.name = AW86224;
				AW_LOGI("aw86224 or aw86225 detected");
				return AW_SUCCESS;
			}
			AW_LOGI("unsupported ef_id = (0x%02X)", ef_id);
			break;
		case AW86214_CHIP_ID:
			haptic_nv_i2c_reads(AW862XX_REG_EFRD9, &ef_id, AW_I2C_BYTE_ONE);
			if ((ef_id & 0x41) == AW86214_EF_ID)
			{
				haptic_nv.name = AW86214;
				AW_LOGI("aw86214 detected");
				return AW_SUCCESS;
			}
			AW_LOGI("unsupported ef_id = (0x%02X)", ef_id);
			break;
		case AW86233_CHIP_ID_L:
			haptic_nv.name = AW86233;
			AW_LOGI("aw86233 detected");
			return 0;
		case AW86234_CHIP_ID_L:
			haptic_nv.name = AW86234;
			AW_LOGI("aw86234 detected");
			return 0;
		case AW86235_CHIP_ID_L:
			haptic_nv.name = AW86235;
			AW_LOGI("aw86235 detected");
			return 0;
		default:
			AW_LOGI("unsupport device revision (0x%02X)", reg);
			break;
		}
		haptic_nv_mdelay(2);
	}
	return AW_ERROR;
}

static void haptic_init(void)
{
	haptic_nv.f0_pre = haptic_nv.info->f0_pre;
	g_func_haptic_nv->play_mode(AW_STANDBY_MODE);
	g_func_haptic_nv->misc_para_init();
	g_func_haptic_nv->offset_cali();
	g_func_haptic_nv->vbat_mode_config(AW_CONT_VBAT_HW_COMP_MODE);
#ifdef AW_F0_CALI_DURING_STARTUP
	f0_cali();
#else
	haptic_nv_get_cali_from_flash();
#endif
}

static void write_ram_data(void)
{
	int i = 0;
	int len = 0;

	AW_LOGI("enter");
	g_func_haptic_nv->set_ram_addr();

	while (i < haptic_nv.aw_fw.len)
	{
		if ((haptic_nv.aw_fw.len - i) < AW_RAMDATA_WR_BUFFER_SIZE)
			len = haptic_nv.aw_fw.len - i;
		else
			len = AW_RAMDATA_WR_BUFFER_SIZE;

		g_func_haptic_nv->set_ram_data(&haptic_nv.aw_fw.data[i], len);
		i += len;
	}
}

static int parse_ram_data(uint32_t len, uint8_t *cont_data, uint8_t *ram_data)
{
	int i = 0;

	AW_LOGI("enter");

	for (i = 0; i < len; i++)
	{
		if (ram_data[i] != cont_data[i])
		{
			AW_LOGE("check ramdata error, addr=0x%04X, ram_data=0x%02X, file_data=0x%02X",
					i, ram_data[i], cont_data[i]);
			return AW_ERROR;
		}
	}

	return AW_SUCCESS;
}

static int check_ram_data(void)
{
	int i = 0;
	int len = 0;
	int ret = AW_SUCCESS;
	uint8_t ram_data[AW_RAMDATA_RD_BUFFER_SIZE] = {0};

	g_func_haptic_nv->set_ram_addr();
	while (i < haptic_nv.aw_fw.len)
	{
		if ((haptic_nv.aw_fw.len - i) < AW_RAMDATA_RD_BUFFER_SIZE)
			len = haptic_nv.aw_fw.len - i;
		else
			len = AW_RAMDATA_RD_BUFFER_SIZE;

		g_func_haptic_nv->get_ram_data(ram_data, len);
		ret = parse_ram_data(len, &haptic_nv.aw_fw.data[i], ram_data);
		if (ret == AW_ERROR)
			break;
		i += len;
	}
	return ret;
}

static int container_update(void)
{
	int ret = 0;

	g_func_haptic_nv->play_stop();
	g_func_haptic_nv->ram_init(AW_TRUE);
	g_func_haptic_nv->set_base_addr();
	g_func_haptic_nv->set_fifo_addr();
	g_func_haptic_nv->get_fifo_addr();
	write_ram_data();

#ifdef AW_CHECK_RAM_DATA
	ret = check_ram_data();
	if (ret)
		AW_LOGE("ram data check sum error");
	else
		AW_LOGI("ram data check sum pass");
#endif

	g_func_haptic_nv->ram_init(AW_FALSE);
	return ret;
}

static int ram_load(void)
{
	int ret = 0;
	AW_LOGI("ram load size: %d", haptic_nv.aw_fw.len);
	ret = container_update();
	if (ret)
	{
		AW_LOGE("ram firmware update failed!");
	}
	else
	{
		haptic_nv.ram_init = AW_TRUE;
		haptic_nv.ram.len = haptic_nv.aw_fw.len;
		if (haptic_nv.is_supported_trig)
			g_func_haptic_nv->trig_init();

		AW_LOGI("ram firmware update complete!");
		get_ram_num();
	}
	return AW_SUCCESS;
}

static int ram_init(void)
{
	int ret = AW_SUCCESS;
	haptic_nv.ram_init = AW_FALSE;
	haptic_nv.rtp_init = AW_FALSE;
	if (get_base_addr() != AW_SUCCESS)
	{
		AW_LOGE("base addr error, please check your ram data");
		return AW_ERROR;
	}
	ret = ram_load();

	return ret;
}

static int func_ptr_init(void)
{
	int ret = AW_SUCCESS;
	switch (haptic_nv.name)
	{
#ifdef AW8624_DRIVER
	case AW8624:
		g_func_haptic_nv = &aw8624_func_list;
		break;
#endif

#ifdef AW8622X_DRIVER
	case AW86223:
	case AW86224:
	case AW86225:
		g_func_haptic_nv = &aw862xx_func_list;
		break;
#endif

#ifdef AW86214_DRIVER
	case AW86214:
		g_func_haptic_nv = &aw862xx_func_list;
		break;
#endif

#ifdef AW8623X_DRIVER
	case AW86233:
	case AW86234:
	case AW86235:
		g_func_haptic_nv = &aw8623x_func_list;
		break;
#endif

	default:
		AW_LOGE("unexpected chip!");
		ret = AW_ERROR;
		break;
	}
	if (g_func_haptic_nv == NULL)
	{
		AW_LOGE("g_func_haptic_nv is null!");
		ret = AW_ERROR;
	}
	return ret;
}

static int create_node(void)
{
	if (!g_func_haptic_nv)
		return AW_ERROR;
#ifdef AW_IRQ_CONFIG
	g_func_haptic_nv->irq_handle = irq_handle;
	g_func_haptic_nv->set_hw_irq_status = set_hw_irq_status;
	g_func_haptic_nv->get_hw_irq_status = get_hw_irq_status;
#endif

	g_func_haptic_nv->f0_cali = f0_cali;
	g_func_haptic_nv->f0_show = f0_show;
	g_func_haptic_nv->cali_show = cali_show;
	g_func_haptic_nv->rtp_going = rtp_going;
	g_func_haptic_nv->long_vib_work = long_vib_work;
	g_func_haptic_nv->short_vib_work = short_vib_work;
	g_func_haptic_nv->rtp_vib_work = rtp_vib_work;
	g_func_haptic_nv->get_ram_num = get_ram_num;
	g_func_haptic_nv->ram_show = ram_show;
	return AW_SUCCESS;
}

static void chip_private_init(void)
{
	switch (haptic_nv.name)
	{
#ifdef AW8624_DRIVER
	case AW8624:
		haptic_nv.info = &aw8624_dts;
		haptic_nv.is_supported_rtp = AW_TRUE;
		haptic_nv.is_supported_trig = AW_TRUE;
		haptic_nv.aw_fw.data = aw8624_ram_data;
		haptic_nv.aw_fw.len = aw8624_ram_len;
		break;
#endif

#ifdef AW8622X_DRIVER
	case AW86223:
	case AW86224:
	case AW86225:
		haptic_nv.info = &aw8622x_dts;
		haptic_nv.is_supported_rtp = AW_TRUE;
		haptic_nv.is_supported_trig = AW_TRUE;
		haptic_nv.aw_fw.data = aw862xx_ram_data;
		haptic_nv.aw_fw.len = aw862xx_ram_len;
		break;
#endif

#ifdef AW86214_DRIVER
	case AW86214:
		haptic_nv.info = &aw86214_dts;
		haptic_nv.is_supported_rtp = AW_FALSE;
		haptic_nv.is_supported_trig = AW_FALSE;
		haptic_nv.aw_fw.data = aw862xx_ram_data;
		haptic_nv.aw_fw.len = aw862xx_ram_len;
		break;
#endif

#ifdef AW8623X_DRIVER
	case AW86233:
	case AW86234:
	case AW86235:
		haptic_nv.info = &aw8623x_dts;
		haptic_nv.is_supported_rtp = AW_TRUE;
		haptic_nv.is_supported_trig = AW_TRUE;
		haptic_nv.aw_fw.data = aw862xx_ram_data;
		haptic_nv.aw_fw.len = aw862xx_ram_len;
		break;
#endif

	default:
		break;
	}
}

#ifdef AW_IRQ_CONFIG
static void irq_config(void)
{
	haptic_nv.is_used_irq_pin = AW_TRUE;
	g_func_haptic_nv->interrupt_setup();
}
#endif

int haptic_nv_boot_init(void)
{
	int ret = AW_SUCCESS;
	uint8_t reg_data[]={0x51,0x52,0x53};
	uint8_t buff[3]={0};
	
	AW_LOGI("haptic_nv driver version %s", HAPTIC_NV_DRIVER_VERSION);
	haptic_nv_i2c_writes(0x0A,reg_data,3);
	AW_LOGI("reg_data1 data : %d  %d  %d", reg_data[0],reg_data[1],reg_data[2]);
	
	haptic_nv_i2c_reads(0x0A,buff,3);
	AW_LOGI("reg_data1 data : %d  %d  %d", buff[0],buff[1],buff[2]);
	hw_reset();
	ret = parse_chipid();
	if (ret != AW_SUCCESS)
	{
		AW_LOGE("read chip id failed!");
		return ret;
	}
	sw_reset();
	chip_private_init();
	ret = func_ptr_init();
	if (ret != AW_SUCCESS)
	{
		AW_LOGE("ctrl_init failed");
		return ret;
	}
	ret = g_func_haptic_nv->check_qualify();
	if (ret != AW_SUCCESS)
	{
		AW_LOGE("qualify check failed.");
		return ret;
	}

#ifdef AW_IRQ_CONFIG
	irq_config();
#endif

	haptic_init();
	ret = ram_init();
	if (ret != AW_SUCCESS)
	{
		AW_LOGE("ram init err!!!");
		return ret;
	}
	create_node();
	
	
	HAL_GPIO_WritePin(AW_RST_GPIO_Port, AW_RST_Pin, GPIO_PIN_RESET); // PG6Çå0£¬

	return ret;
}
