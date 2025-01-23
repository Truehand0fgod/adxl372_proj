/***************************************************************************//**
 *   @file   adxl372.h
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

#ifndef ADXL372_H_
#define ADXL372_H_


#include <main.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>


//typedef SPI_HandleTypeDef adxl_spi_handle;
// #define adxl_spi_handle hspi3

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t N;
extern UART_HandleTypeDef huart1;
#define L_ENDIAN

/* Register address */
#define ADI_ADXL372_ADI_DEVID           0x00u   /* Analog Devices, Inc., accelerometer ID */
#define ADI_ADXL372_MST_DEVID          	0x01u   /* Analog Devices MEMS device ID */
#define ADI_ADXL372_DEVID               0x02u   /* Device ID */
#define ADI_ADXL372_REVID               0x03u   /* product revision ID*/
#define ADI_ADXL372_STATUS_1            0x04u   /* Status register 1 */
#define ADI_ADXL372_STATUS_2            0x05u   /* Status register 2 */
#define ADI_ADXL372_FIFO_ENTRIES_2	0x06u   /* Valid data samples in the FIFO */
#define ADI_ADXL372_FIFO_ENTRIES_1	0x07u   /* Valid data samples in the FIFO */
#define ADI_ADXL372_X_DATA_H            0x08u   /* X-axis acceleration data [11:4] */
#define ADI_ADXL372_X_DATA_L            0x09u   /* X-axis acceleration data [3:0] | dummy LSBs */
#define ADI_ADXL372_Y_DATA_H            0x0Au   /* Y-axis acceleration data [11:4] */
#define ADI_ADXL372_Y_DATA_L		0x0Bu   /* Y-axis acceleration data [3:0] | dummy LSBs */
#define ADI_ADXL372_Z_DATA_H            0x0Cu   /* Z-axis acceleration data [11:4] */
#define ADI_ADXL372_Z_DATA_L            0x0Du   /* Z-axis acceleration data [3:0] | dummy LSBs */
#define ADI_ADXL372_X_MAXPEAK_H         0x15u   /* X-axis MaxPeak acceleration data [15:8] */
#define ADI_ADXL372_X_MAXPEAK_L         0x16u   /* X-axis MaxPeak acceleration data [7:0] */
#define ADI_ADXL372_Y_MAXPEAK_H        	0x17u   /* X-axis MaxPeak acceleration data [15:8] */
#define ADI_ADXL372_Y_MAXPEAK_L         0x18u   /* X-axis MaxPeak acceleration data [7:0] */
#define ADI_ADXL372_Z_MAXPEAK_H         0x19u   /* X-axis MaxPeak acceleration data [15:8] */
#define ADI_ADXL372_Z_MAXPEAK_L         0x1Au   /* X-axis MaxPeak acceleration data [7:0] */
#define ADI_ADXL372_OFFSET_X 	        0x20u   /* X axis offset */
#define ADI_ADXL372_OFFSET_Y    	0x21u   /* Y axis offset */
#define ADI_ADXL372_OFFSET_Z	        0x22u   /* Z axis offset */
#define ADI_ADXL372_X_THRESH_ACT_H      0x23u   /* X axis Activity Threshold [15:8] */
#define ADI_ADXL372_X_THRESH_ACT_L	0x24u   /* X axis Activity Threshold [7:0] */
#define ADI_ADXL372_Y_THRESH_ACT_H      0x25u   /* Y axis Activity Threshold [15:8] */
#define ADI_ADXL372_Y_THRESH_ACT_L      0x26u   /* Y axis Activity Threshold [7:0] */
#define ADI_ADXL372_Z_THRESH_ACT_H	0x27u   /* Z axis Activity Threshold [15:8] */
#define ADI_ADXL372_Z_THRESH_ACT_L	0x28u   /* Z axis Activity Threshold [7:0] */
#define ADI_ADXL372_TIME_ACT	        0x29u   /* Activity Time */
#define ADI_ADXL372_X_THRESH_INACT_H    0x2Au   /* X axis Inactivity Threshold [15:8] */
#define ADI_ADXL372_X_THRESH_INACT_L	0x2Bu   /* X axis Inactivity Threshold [7:0] */
#define ADI_ADXL372_Y_THRESH_INACT_H    0x2Cu   /* Y axis Inactivity Threshold [15:8] */
#define ADI_ADXL372_Y_THRESH_INACT_L    0x2Du   /* Y axis Inactivity Threshold [7:0] */
#define ADI_ADXL372_Z_THRESH_INACT_H	0x2Eu   /* Z axis Inactivity Threshold [15:8] */
#define ADI_ADXL372_Z_THRESH_INACT_L	0x2Fu   /* Z axis Inactivity Threshold [7:0] */
#define ADI_ADXL372_TIME_INACT_H        0x30u   /* Inactivity Time [15:8] */
#define ADI_ADXL372_TIME_INACT_L        0x31u   /* Inactivity Time [7:0] */
#define ADI_ADXL372_X_THRESH_ACT2_H     0x32u   /* X axis Activity2 Threshold [15:8] */
#define ADI_ADXL372_X_THRESH_ACT2_L	0x33u   /* X axis Activity2 Threshold [7:0] */
#define ADI_ADXL372_Y_THRESH_ACT2_H     0x34u   /* Y axis Activity2 Threshold [15:8] */
#define ADI_ADXL372_Y_THRESH_ACT2_L     0x35u   /* Y axis Activity2 Threshold [7:0] */
#define ADI_ADXL372_Z_THRESH_ACT2_H	0x36u   /* Z axis Activity2 Threshold [15:8] */
#define ADI_ADXL372_Z_THRESH_ACT2_L	0x37u   /* Z axis Activity2 Threshold [7:0] */
#define ADI_ADXL372_HPF 	        0x38u   /* High Pass Filter */
#define ADI_ADXL372_FIFO_SAMPLES        0x39u   /* FIFO Samples */
#define ADI_ADXL372_FIFO_CTL	        0x3Au   /* FIFO Control */
#define ADI_ADXL372_INT1_MAP            0x3Bu   /* Interrupt 1 mapping control */
#define ADI_ADXL372_INT2_MAP            0x3Cu   /* Interrupt 2 mapping control */
#define ADI_ADXL372_TIMING	        0x3Du   /* Timing */
#define ADI_ADXL372_MEASURE		0x3Eu   /* Measure */
#define ADI_ADXL372_POWER_CTL           0x3Fu   /* Power control */
#define ADI_ADXL372_SELF_TEST           0x40u   /* Self Test */
#define ADI_ADXL372_SRESET              0x41u   /* Reset */
#define ADI_ADXL372_FIFO_DATA		0x42u   /* FIFO Data */

#define ADI_ADXL372_ADI_DEVID_VAL       0xADu   /* Analog Devices, Inc., accelerometer ID */
#define ADI_ADXL372_MST_DEVID_VAL       0x1Du   /* Analog Devices MEMS device ID */
#define ADI_ADXL372_DEVID_VAL           0xFAu   /* Device ID */
#define ADI_ADXL372_REVID_VAL           0x02u   /* product revision ID*/


#define MEASURE_AUTOSLEEP_MASK		0xBF
#define MEASURE_BANDWIDTH_MASK		0xF8
#define MEASURE_ACTPROC_MASK		0xCF
#define MEASURE_LOWNOISE_MASK		0xF7
#define TIMING_ODR_MASK			0x1F
#define TIMING_WUR_MASK 		0xE3
#define PWRCTRL_OPMODE_MASK		0xFC
#define PWRCTRL_INSTON_THRESH_MASK	0xDF
#define PWRCTRL_INSTON_THRESH_MASK	0xDF
#define PWRCTRL_FILTER_SETTLE_MASK	0xEF
#define PWRCTRL_LPF_HPF_MASK				0xF3


#define MEASURE_AUTOSLEEP_POS		6
#define MEASURE_ACTPROC_POS		4
#define MEASURE_LOWNOISE_POS	3
#define TIMING_ODR_POS			5
#define TIMING_WUR_POS 			2
#define INSTAON_THRESH_POS		5
#define FIFO_CRL_SAMP8_POS		0
#define FIFO_CRL_MODE_POS		1
#define FIFO_CRL_FORMAT_POS		3
#define PWRCTRL_FILTER_SETTLE_POS	4
#define PWRCTRL_LPF_HPF_POS	2

#define DATA_RDY	1
#define FIFO_RDY	2
#define FIFO_FULL	4
#define FIFO_OVR	8

#define ADXL_SPI_RNW    1

/*Acceleremoter configuration*/
#define ACT_VALUE          50     /* Activity threshold value */

#define INACT_VALUE        50     /* Inactivity threshold value */

#define ACT_TIMER          1    /* Activity timer value in multiples of 3.3ms */

#define INACT_TIMER        3     /* Inactivity timer value in multiples of 26ms */

#define ADXL_INT1_PIN     7
#define ADXL_INT2_PIN     5

//typedef unsigned char adxl_spi_handle;

typedef enum {
    STAND_BY = 0,
    WAKE_UP,
    INSTANT_ON,
    FULL_BW_MEASUREMENT
} ADXL372_OP_MODE;

typedef enum {
	MEAS_NORM_OP	=0,
	MEAS_LOW_NOISE
} ADXL372_NOISE_MODE;

typedef enum {
    ODR_400Hz = 0,
    ODR_800Hz,
    ODR_1600Hz,
    ODR_3200Hz,
    ODR_6400Hz
} ADXL372_ODR;

typedef enum {
    BW_200Hz = 0,
    BW_400Hz,
    BW_800Hz,
    BW_1600Hz,
    BW_3200Hz
} ADXL372_BW;

typedef enum {
    WUR_52ms = 0,
    WUR_104ms,
    WUR_208ms,
    WUR_512ms,
    WUR_2048ms,
    WUR_4096ms,
    WUR_8192ms,
    WUR_24576ms
} ADXL372_WUR;


typedef enum {
    DEF = 0,
    LINKED,
    LOOPED
} ADXL372_ACT_PROC_MODE;


typedef enum {
    XYZ_FIFO = 0,
    X_FIFO,
    Y_FIFO,
    XY_FIFO,
    Z_FIFO,
    XZ_FIFO,
    YZ_FIFO,
    XYZ_PEAK_FIFO
} ADXL372_FIFO_FORMAT;

typedef enum {
    BYPASSED = 0,
    STREAMED,
    TRIGGERED,
    OLDEST_SAVED
} ADXL372_FIFO_MODE;

typedef struct {
    unsigned short samples;
    ADXL372_FIFO_MODE mode;
    ADXL372_FIFO_FORMAT format;
} fifo_config_t;

typedef enum {
    ADXL_INSTAON_LOW_THRESH = 0,
    ADXL_INSTAON_HIGH_THRESH
} ADXL_INSTAON_THRESH;

typedef enum {
    FILTER_SETTLE_16 = 0,
    FILTER_SETTLE_370
} ADXL372_Filter_Settle;


typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} AccelTriplet_t;

typedef struct {
    float x;
    float y;
    float z;
} AccelTripletG_t;


typedef struct {    
    fifo_config_t fifo_config;
} adxl372_device;

int adxl_read_reg(unsigned char reg, unsigned char *val);
int adxl_write_reg(unsigned char reg, unsigned char val);

int adxl372_Set_Op_mode(ADXL372_OP_MODE mode);
int adxl372_Set_ODR(ADXL372_ODR odr);
int adxl372_Set_WakeUp_Rate(ADXL372_WUR wur);
int adxl372_Set_BandWidth(ADXL372_BW bw);
int adxl372_Set_Autosleep(bool enable);
int adxl372_Set_Act_Proc_Mode(ADXL372_ACT_PROC_MODE mode);
int adxl372_Set_InstaOn_Thresh(ADXL_INSTAON_THRESH mode);
int adxl372_Set_Activity_Threshold(unsigned short thresh, bool referenced, bool enable);
int adxl372_Set_Inactivity_Threshold(unsigned short thresh, bool referenced, bool enable);
int adxl372_Set_Activity_Time(unsigned char time);
int adxl372_Set_Inactivity_Time(unsigned short time);
int adxl372_Set_Filter_Settle(ADXL372_Filter_Settle mode);
int adxl372_Get_DevID(unsigned char *DevID);
int adxl372_Get_Status_Register(unsigned char *adxl_status);
int adxl372_Get_ActivityStatus_Register(unsigned char *adxl_status);
int adxl372_Get_Highest_Peak_Accel_data(AccelTriplet_t *max_peak);
int adxl372_Get_Accel_data(AccelTriplet_t *accel_data);
int adxl372_Reset(void);
int adxl372_Configure_FIFO(unsigned short fifo_samples,
                           ADXL372_FIFO_MODE fifo_mode,
                           ADXL372_FIFO_FORMAT fifo_format);
int adxl372_Get_FIFO_data(short *samples);
int adxl372_Set_Interrupts(void);

void adxl372_Set_Filters(uint8_t lpf_enable, uint8_t hpf_enable);
void adxl372_Set_Noise_Mode(ADXL372_NOISE_MODE mode);
uint16_t adxl372_Get_Fifo_Samples (void);

uint8_t adxl372_Init (void);
uint8_t adxl372_Is_Ready (void);
void adxl372_Get_Data (AccelTriplet_t *accel_data);
void adxl372_Triplet_Abs(AccelTriplet_t *data);
void adxl372_Get_Shock(void);
uint8_t adxl372_Get_MaxG(void);
uint16_t max3(AccelTriplet_t *data);
int spi_write_then_read(unsigned char *txbuf, unsigned n_tx,
						unsigned char *rxbuf, unsigned n_rx);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* ADXL372_H_ */
