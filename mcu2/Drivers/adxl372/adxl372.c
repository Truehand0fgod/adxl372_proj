/***************************************************************************//**
 *   @file   adxl372.c
 *   @brief  ADXL372 device driver.
********************************************************************************
 * Copyright 2017(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*******************************************************************************/
#include "adxl372.h"

//#define ADXL_DEBUG
adxl372_device Axdl372;

uint8_t shock_cnt[4] = {0, 0, 0, 0}; // >50g, >100g, >150g, maxShock
extern SPI_HandleTypeDef hspi2;

int adxl_read_reg_multiple(unsigned char reg,
                                  unsigned short count, unsigned char *val)
{
    reg = reg << 1 | ADXL_SPI_RNW;
    return spi_write_then_read(&reg, 1, val, count);
}

int adxl_read_reg(unsigned char reg, unsigned char *val)
{
	//rv3149DeSelect();
    return adxl_read_reg_multiple(reg, 1, val);
}

int adxl_write_reg(unsigned char reg, unsigned char val)
{
    unsigned char tmp[2];
    tmp[0] = reg << 1;
    tmp[1] = val;

#ifdef ADXL_DEBUG
    spi_write_then_read(tmp, 2, NULL, 0);
        adxl_delay(1);
    adxl_read_reg(reg, tmp);
    if (val != tmp[0])
      dnm_ucli_printf("verify failed REG 0x%X - 0x%X != 0x%X\r\n", reg, val, tmp[0]);
    
    return 0;
    
#else
    return spi_write_then_read(tmp, 2, NULL, 0);
#endif    
}

static int adxl_update_reg(unsigned char reg, unsigned char mask, unsigned char shift, unsigned char val)
{
    unsigned char tmp;
    int err;

    err = adxl_read_reg(reg, &tmp);
    if (err < 0)
        return err;

    tmp &= mask;
    tmp |= (val << shift) & ~mask;

    return adxl_write_reg(reg, tmp);
}

int adxl372_Set_Op_mode(ADXL372_OP_MODE mode)
{
    return adxl_update_reg(ADI_ADXL372_POWER_CTL, PWRCTRL_OPMODE_MASK, 0,
                           mode);
}

int adxl372_Set_ODR(ADXL372_ODR odr)
{
    return adxl_update_reg(ADI_ADXL372_TIMING, TIMING_ODR_MASK,
                           TIMING_ODR_POS, odr);
}

int adxl372_Set_WakeUp_Rate(ADXL372_WUR wur)
{
    return adxl_update_reg(ADI_ADXL372_TIMING, TIMING_WUR_MASK,
                           TIMING_WUR_POS, wur);
}

int adxl372_Set_BandWidth(ADXL372_BW bw)
{
    return adxl_update_reg(ADI_ADXL372_MEASURE, MEASURE_BANDWIDTH_MASK, 0,
                           bw);
}

int adxl372_Set_Autosleep(bool enable)
{
    return adxl_update_reg(ADI_ADXL372_MEASURE, MEASURE_AUTOSLEEP_MASK,
                           MEASURE_AUTOSLEEP_POS, enable);
}

int adxl372_Set_Act_Proc_Mode(ADXL372_ACT_PROC_MODE mode)
{
    return adxl_update_reg(ADI_ADXL372_MEASURE, MEASURE_ACTPROC_MASK,
                           MEASURE_ACTPROC_POS, mode);
}

int adxl372_Set_InstaOn_Thresh(ADXL_INSTAON_THRESH mode)
{
    return adxl_update_reg(ADI_ADXL372_POWER_CTL,
                           PWRCTRL_INSTON_THRESH_MASK, INSTAON_THRESH_POS, mode);
}

int adxl372_Set_Activity_Threshold(unsigned short thresh, bool referenced, bool enable)
{
/*    int err = adxl372_Set_Op_mode(STAND_BY);  FIXME ?
    if (err < 0)
        return err;*/
	int err = 0;
    adxl_write_reg(ADI_ADXL372_X_THRESH_ACT_H, thresh >> 3);
    adxl_write_reg(ADI_ADXL372_X_THRESH_ACT_L, (thresh << 5) | (referenced << 1) | enable);
    adxl_write_reg(ADI_ADXL372_Y_THRESH_ACT_H, thresh >> 3);
    adxl_write_reg(ADI_ADXL372_Y_THRESH_ACT_L, (thresh << 5) | enable);
    adxl_write_reg(ADI_ADXL372_Z_THRESH_ACT_H, thresh >> 3);
    adxl_write_reg(ADI_ADXL372_Z_THRESH_ACT_L, (thresh << 5) | enable);

    return err;
}

int adxl372_Set_Activity2_Threshold(unsigned short thresh, bool referenced, bool enable)
{
/*    int err = adxl372_Set_Op_mode(STAND_BY);  FIXME ?
    if (err < 0)
        return err;*/
	int err = 0;
    adxl_write_reg(ADI_ADXL372_X_THRESH_ACT2_H, thresh >> 3);
    adxl_write_reg(ADI_ADXL372_X_THRESH_ACT2_L,
                   (thresh << 5) | (referenced << 1) | enable);
    adxl_write_reg(ADI_ADXL372_Y_THRESH_ACT2_H, thresh >> 3);
    adxl_write_reg(ADI_ADXL372_Y_THRESH_ACT2_L, (thresh << 5) | enable);
    adxl_write_reg(ADI_ADXL372_Z_THRESH_ACT2_H, thresh >> 3);
    adxl_write_reg(ADI_ADXL372_Z_THRESH_ACT2_L, (thresh << 5) | enable);

    return err;
}

int adxl372_Set_Inactivity_Threshold(unsigned short thresh, bool referenced, bool enable)
{
    int err = adxl372_Set_Op_mode(STAND_BY);  /* FIXME ? */
    if (err < 0)
        return err;

    adxl_write_reg(ADI_ADXL372_X_THRESH_INACT_H, thresh >> 3);
    adxl_write_reg(ADI_ADXL372_X_THRESH_INACT_L,
                   (thresh << 5) | (referenced << 1) | enable);
    adxl_write_reg(ADI_ADXL372_Y_THRESH_INACT_H, thresh >> 3);
    adxl_write_reg(ADI_ADXL372_Y_THRESH_INACT_L, (thresh << 5) | enable);
    adxl_write_reg(ADI_ADXL372_Z_THRESH_INACT_H, thresh >> 3);
    adxl_write_reg(ADI_ADXL372_Z_THRESH_INACT_L, (thresh << 5) | enable);

    return err;
}

int adxl372_Set_Activity_Time(unsigned char time)
{
    return adxl_write_reg(ADI_ADXL372_TIME_ACT, time);
}

int adxl372_Set_Inactivity_Time(unsigned short time)
{
    adxl_write_reg(ADI_ADXL372_TIME_INACT_H, time >> 8);

    return adxl_write_reg(ADI_ADXL372_TIME_INACT_L, time & 0xFF);
}

int adxl372_Set_Filter_Settle(ADXL372_Filter_Settle mode)
{
    return adxl_update_reg(ADI_ADXL372_POWER_CTL,
                           PWRCTRL_FILTER_SETTLE_MASK, PWRCTRL_FILTER_SETTLE_POS, mode);
}

int adxl372_Get_DevID(unsigned char *DevID)
{
	//rv3149DeSelect();
    return adxl_read_reg_multiple(ADI_ADXL372_ADI_DEVID, 1, DevID);
}

int adxl372_Get_Status_Register(unsigned char *adxl_status)
{
    return adxl_read_reg(ADI_ADXL372_STATUS_1, adxl_status);
}

int adxl372_Get_ActivityStatus_Register(unsigned char *adxl_status)
{
    return adxl_read_reg(ADI_ADXL372_STATUS_2, adxl_status);
}

#define SWAP16(x) ((x) = (((x) & 0x00FF) << 8) | (((x) & 0xFF00) >> 8))
#define SHIFT4(x) ((x) = (x) >> 4)

int adxl372_Get_Highest_Peak_Accel_data(AccelTriplet_t *max_peak)
{
    int err = adxl_read_reg_multiple(ADI_ADXL372_X_MAXPEAK_H, 6,
                                     (unsigned char *) max_peak);

#ifdef L_ENDIAN
    SWAP16(max_peak->x);
    SWAP16(max_peak->y);
    SWAP16(max_peak->z);
#endif

    SHIFT4(max_peak->x);
    SHIFT4(max_peak->y);
    SHIFT4(max_peak->z);

    return err;
}

int adxl372_Get_Accel_data(AccelTriplet_t *accel_data)
{
    unsigned char status;
    int err;
    uint32_t Timeout = 0;
    do {
        adxl372_Get_Status_Register(&status);
    } while (!(status & DATA_RDY) || (++Timeout < 2000));

    err = adxl_read_reg_multiple(ADI_ADXL372_X_DATA_H, 6,
                                 (unsigned char *) accel_data);

#ifdef L_ENDIAN
  
    SWAP16(accel_data->x);
    SWAP16(accel_data->y);
    SWAP16(accel_data->z);
#endif

    SHIFT4(accel_data->x);
    SHIFT4(accel_data->y);
    SHIFT4(accel_data->z);
    return err;
}

AccelTripletG_t adxl372_ConvertAccTripletToG(const AccelTriplet_t *triplet) {

	AccelTripletG_t triplet_g = {0.0, 0.0, 0.0};
	triplet_g.x = triplet->x * 100.0 / 1000.0;
	triplet_g.y = triplet->y * 100.0 / 1000.0;
	triplet_g.z = triplet->z * 100.0 / 1000.0;
	return (triplet_g);

}

int adxl372_Reset (void)
{
    int err = adxl_write_reg(ADI_ADXL372_SRESET, 0x52);
    return err;
}

int adxl372_Configure_FIFO(unsigned short fifo_samples,
                           ADXL372_FIFO_MODE fifo_mode,
                           ADXL372_FIFO_FORMAT fifo_format)
{
    unsigned char config;
    int err;

    adxl372_Set_Op_mode(STAND_BY);

    if (fifo_samples > 512)
        return -1;

    //fifo_samples -= 1;

    config = ((unsigned char)fifo_mode << FIFO_CRL_MODE_POS) |
             ((unsigned char)fifo_format << FIFO_CRL_FORMAT_POS) |
             ((fifo_samples > 0xFF) << FIFO_CRL_SAMP8_POS);

    err = adxl_write_reg(ADI_ADXL372_FIFO_SAMPLES, fifo_samples & 0xFF);
    if (err < 0)
        return err;

    err = adxl_write_reg(ADI_ADXL372_FIFO_CTL, config);
    if (err < 0)
        return err;

    Axdl372.fifo_config.samples = fifo_samples + 1;
    Axdl372.fifo_config.mode = fifo_mode;
    Axdl372.fifo_config.format = fifo_format;

    return err;
}

int adxl372_Get_FIFO_data(int16_t *samples)
{
    int err, i;
    if(Axdl372.fifo_config.mode == BYPASSED)
        return -1;

    err = adxl_read_reg_multiple(ADI_ADXL372_FIFO_DATA,
                                 (Axdl372.fifo_config.samples - 3)* 2, (unsigned char *) samples);

#ifdef L_ENDIAN
    for (i = 0; i < Axdl372.fifo_config.samples; i++)
        SWAP16(samples[i]);
#endif

    return err;
}

int adxl372_Set_Interrupts(void)
{
  int err;
  
  err = adxl_write_reg(ADI_ADXL372_INT1_MAP, ((1 << 5)));
  err = adxl_write_reg(ADI_ADXL372_INT2_MAP, ((1 << 5)));
/*  if (err < 0)
      return err;*/

  return err;
}

void adxl372_Set_Filters(uint8_t lpf_enable, uint8_t hpf_enable)
{
	uint8_t mode = ~((uint8_t)0xFF & ((((uint8_t)0x01 & lpf_enable) << 1) | ((uint8_t)0x01 & hpf_enable)));
	
	adxl_update_reg(ADI_ADXL372_POWER_CTL,
                           PWRCTRL_LPF_HPF_MASK, PWRCTRL_LPF_HPF_POS, mode);
}

void adxl372_Set_Noise_Mode(ADXL372_NOISE_MODE mode)
{
	
	adxl_update_reg(ADI_ADXL372_MEASURE, MEASURE_LOWNOISE_MASK,
                           MEASURE_LOWNOISE_POS, (0x01 & mode));
}

uint16_t adxl372_Get_Fifo_Samples (void) 
{
	uint8_t data[2] = {0,};	
	adxl_read_reg_multiple(ADI_ADXL372_FIFO_ENTRIES_2, 2, data);
    return ((data[0] & 0x3) | data[1]);
}


uint8_t adxl372_Init (void)
{

/*	uint8_t devID = 0;
	adxl372_Get_DevID(&devID);
	
	if (devID != 173) return 1;*/

/* Configuring ADXL372 device */
	adxl372_Set_Op_mode(STAND_BY);
	adxl372_Reset();
	adxl372_Set_ODR(ODR_6400Hz);
	adxl372_Set_BandWidth(BW_3200Hz);
	adxl372_Set_Noise_Mode(MEAS_LOW_NOISE);
	adxl372_Configure_FIFO(0, STREAMED, XYZ_PEAK_FIFO);
	adxl372_Set_Act_Proc_Mode(LOOPED);
	adxl372_Set_Filters(0, 1);					// LPF, HPF
	adxl372_Set_Filter_Settle(FILTER_SETTLE_16);
	adxl372_Set_Activity_Threshold(40, 0, 1);
	adxl372_Set_Activity_Time(0);
	adxl372_Set_Inactivity_Threshold(40, 0, 1);
	adxl372_Set_Inactivity_Time(1);


	adxl372_Set_Op_mode(FULL_BW_MEASUREMENT);

	return 0;
}

uint8_t adxl372_Is_Ready (void)
{
	uint8_t adxlStatus1Reg;	
	adxl372_Get_Status_Register(&adxlStatus1Reg);
	if ((adxlStatus1Reg & DATA_RDY) != DATA_RDY) return 0;
	
	return 1;
}

/*void adxl372_Get_Data (AccelTriplet_t *accel_data)
{
	adxl372_Get_Accel_data(accel_data);
}*/

void adxl372_Get_Shock(void)
{

	uint8_t maxG = adxl372_Get_MaxG();

	if (maxG >= 150)
	{
		++shock_cnt[2];
	}
	else if(maxG >= 100)
	{
		++shock_cnt[1];
	}
	else if(maxG >= 50)
	{
		++shock_cnt[0];
	}
	if (maxG > shock_cnt[3])
	{
		shock_cnt[3]= maxG;
	}
}

void adxl372_Triplet_Abs(AccelTriplet_t *data)
{
    data->x = (data->x ^ (data->x >> 15)) - (data->x >> 15);
    data->y = (data->y ^ (data->y >> 15)) - (data->y >> 15);
    data->z = (data->z ^ (data->z >> 15)) - (data->z >> 15);
}

uint8_t adxl372_Get_MaxG(void)
{
	AccelTriplet_t ac_data;
	adxl372_Get_Highest_Peak_Accel_data(&ac_data);
	adxl372_Triplet_Abs(&ac_data);
	return max3(&ac_data)/10;
}

uint16_t max3(AccelTriplet_t *data)
{
    uint16_t max = data->x;
    if (data->y > max)
        max = data->y;
    if (data->z > max)
        max = data->z;
    return max;
}

int spi_write_then_read(	   unsigned char *txbuf,
                               unsigned n_tx,
                               unsigned char *rxbuf,
                               unsigned n_rx)
{
  int err = 0;


  HAL_GPIO_WritePin(ADXL372_CS_GPIO_Port, ADXL372_CS_Pin, GPIO_PIN_RESET);

  //HAL_Delay(1);

  for(int i = 0; i < n_tx; i++)
  {
	  if(HAL_SPI_Transmit(&hspi2, &txbuf[i], 1, 50) != HAL_OK)
	  {
		  err = 1;
	  }
  }

  //HAL_Delay(1);
/*  for(count = 0; count < n_tx; count++)
  {
    SPI.transfer((unsigned char)txbuf[count]);
  }*/


  for(int i = 0; i < n_rx; i++)
  {
	  if(HAL_SPI_Receive(&hspi2, &rxbuf[i], 1, 50) != HAL_OK)
	  {
		  err = 2;
	  }
  }

/*  for(count = 0; count < n_rx; count++)
  {
    rxbuf[count] = SPI.transfer(0xAA);
  }*/

  HAL_GPIO_WritePin(ADXL372_CS_GPIO_Port, ADXL372_CS_Pin, GPIO_PIN_SET);

  return err;
}

