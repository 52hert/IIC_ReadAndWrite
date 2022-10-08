#include "main.h"
#include "haptic_nv.h"
#include "hdl_aw86224.h"
#include "timer.h"

//TIM_HandleTypeDef TIM3_Handler;      //¶¨Ê±Æ÷¾ä±ú 
//extern TIM_HandleTypeDef TIM3_Handler;
//extern I2C_HandleTypeDef hi2c1;

/*****************************************************
 * @brief i2c read function
 * @param reg_addr: register address
 * @param reg_data: register data
 * @param len: Number of read registers
 * @retval i2c read status: 0->success, 1->error
 *****************************************************/
int haptic_nv_i2c_reads(uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	uint8_t cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
	// modified
	if (AW86224FCR_ReadLenByte(haptic_nv.i2c_addr << 1, reg_addr, len, reg_data) != 0)
		return AW_SUCCESS;
//	AT24CXX_Read( reg_addr,  reg_data,len);
//		return AW_SUCCESS;
	cnt ++;
	}

	AW_LOGE("i2c read 0x%02X err!", reg_addr);
	return AW_ERROR;
}

/*****************************************************
 * @brief i2c write function
 * @param reg_addr: register address
 * @param reg_data: register data
 * @param len: Number of write registers
 * @retval i2c write status: 0->success, 1->error
 *****************************************************/
int haptic_nv_i2c_writes(uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	uint8_t cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
	if (AW86224_WriteLenByte(haptic_nv.i2c_addr << 1, reg_addr, len, reg_data) != 0)
		return AW_SUCCESS;
	cnt ++;
	}

	AW_LOGE("i2c write 0x%02X err!", reg_addr);
	return AW_ERROR;
}

/*****************************************************
 * @brief i2c write bits function
 * @param reg_addr: register address
 * @param reg_addr: register mask
 * @param reg_data: register data
 * @retval NULL
 *****************************************************/
void haptic_nv_i2c_write_bits(uint8_t reg_addr, uint32_t mask, uint8_t reg_data)
{
	uint8_t reg_val = 0;
	uint8_t reg_mask = (uint8_t)mask;

	haptic_nv_i2c_reads(reg_addr, &reg_val, AW_I2C_BYTE_ONE);
	reg_val &= reg_mask;
	reg_val |= (reg_data & (~reg_mask));
	haptic_nv_i2c_writes(reg_addr, &reg_val, AW_I2C_BYTE_ONE);
}

/*****************************************************
 * @brief read chip id function
 * @param reg_addr: chip id
 * @param type: 0->first try, 1->last try
 * @retval 0->success, 1->error
 *****************************************************/
int haptic_nv_read_chipid(uint8_t *val, uint8_t type)
{
	uint8_t cnt = 0;
	int ret = AW_ERROR;

	while (cnt < AW_I2C_RETRIES) {
		ret = haptic_nv_i2c_reads(AW_REG_CHIPIDH, val, AW_I2C_BYTE_ONE);
		if (*val == AW8623X_CHIP_ID_H)
			ret = haptic_nv_i2c_reads(AW_REG_CHIPIDL, val, AW_I2C_BYTE_ONE);
		else
			ret = haptic_nv_i2c_reads(AW_REG_ID, val, AW_I2C_BYTE_ONE);
		if (ret == AW_ERROR) {
			if (type == AW_FIRST_TRY)
				AW_LOGI("reading chip id");
			else if (type == AW_LAST_TRY)
				AW_LOGE("i2c_read cnt=%d error=%d", cnt, ret);
			else
				AW_LOGE("type is error");
		} else {
			break;
		}
		cnt++;
	}

	return ret;
}

/*****************************************************
 * @brief stop hrtimer
 * @param None
 * @retval None
 *****************************************************/
void haptic_nv_stop_hrtimer(void)
{
	//HAL_TIM_Base_Stop_IT(&TIM3_Handler);
}

/*****************************************************
 * @brief start hrtimer
 * @param None
 * @retval None
 *****************************************************/
void haptic_nv_start_hrtimer(void)
{
	//HAL_TIM_Base_Start_IT(&TIM3_Handler);
}

/*****************************************************
 * @brief hrtimer callback function, it's used to long vibrator stop. should called by HAL_TIM_PeriodElapsedCallback
 * @param htim: hrtimer
 * @retval None
 *****************************************************/
void haptic_nv_tim_periodelapsedcallback(TIM_HandleTypeDef *htim)
{
//	if (htim->Instance == TIM3_Handler.Instance) {
//		haptic_nv.timer_ms_cnt++;
//		if (haptic_nv.timer_ms_cnt == haptic_nv.duration) {
//			AW_LOGI("timer over, haptic_nv.duration:%d", haptic_nv.duration);
//			haptic_nv.duration = 0;
//			haptic_nv.timer_ms_cnt = 0;
//			haptic_nv_stop_hrtimer();
//			/* Put awinic vibrator chip into standby mode */
//			g_func_haptic_nv->play_stop();
//		}
//	}
}

/*****************************************************
 * @brief delay function
 * @param ms: millisecond
 * @retval None
 *****************************************************/
void haptic_nv_mdelay(uint32_t ms)
{
	HAL_Delay(ms);
}

/**
  * @brief factory F0 calibration value can be stored in flash
  * @retval None
  */
void haptic_nv_set_cali_to_flash(void)
{
	AW_LOGI("f0 cali data is 0x%02x", haptic_nv.f0_cali_data);
}

/**
  * @brief update calibration values to driver
  * @retval None
  */
void haptic_nv_get_cali_from_flash(void)
{
	AW_LOGI("f0 cali data is 0x%02x", haptic_nv.f0_cali_data);
	/* haptic_nv.f0_cali_data = val; */
}

#ifdef AW_IRQ_CONFIG
/*****************************************************
 * @brief interrupt callback function, should called by HAL_GPIO_EXTI_Callback
 * @param GPIO_Pin: irq gpio pin
 * @retval None
 *****************************************************/
void haptic_nv_gpio_exti_callback(uint16_t GPIO_Pin)
{
	haptic_nv.irq_handle = AW_IRQ_ON;
}
#endif

/*****************************************************
 * @brief disable interrput gpio function
 * @param None
 * @retval None
 *****************************************************/
void haptic_nv_disable_irq(void)
{
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
}

/*****************************************************
 * @brief enable interrput gpio function
 * @param None
 * @retval None
 *****************************************************/
void haptic_nv_enable_irq(void)
{
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/*****************************************************
 * @brief pin control function
 * @param GPIO_Pin: gpio pin
 * @param status: Pin status
 * @retval NULL
 *****************************************************/
void haptic_nv_pin_control(uint16_t GPIO_Pin, uint8_t status)
{
	if(status == AW_PIN_LOW)
		HAL_GPIO_WritePin(AW_RST_GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);
	else if (status == AW_PIN_HIGH)
		HAL_GPIO_WritePin(AW_RST_GPIO_Port, GPIO_Pin, GPIO_PIN_SET);
}
