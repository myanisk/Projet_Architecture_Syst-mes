/*
 * YNV_LIS3DSH.h
 *
 *  Created on: 12 janv. 2021
 *      Author: yanis
 */

#ifndef INC_YNV_LIS3DSH_H_
#define INC_YNV_LIS3DSH_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>



/* Register Adress Define */
#define LIS3DSH_CTRL_REG4					0x20
#define LIS3DSH_CTRL_REG5					0x24
#define LIS3DSH_STATUS						0x27
#define LIS3DSH_OUT_X						0X28
#define LIS3DSH_OUT_Y						0X2A
#define LIS3DSH_OUT_Z						0X2C


/*Configuration parameters define*/

// Configuration of the register CTRL_REG4 on an 8-bit binary code allowing to activate (1) or deactivate (0) the axes
#define LIS3DSH_CTRL_REG4_X_ENABLE				((uint8_t)0x01) //binary code :0000 0001
#define LIS3DSH_CTRL_REG4_X_DISABLE				((uint8_t)0x00) //binary code :0000 0000

#define LIS3DSH_CTRL_REG4_Y_ENABLE				((uint8_t)0x02) //binary code :0000 0010
#define LIS3DSH_CTRL_REG4_Y_DISABLE				((uint8_t)0x00) //binary code :0000 0000

#define LIS3DSH_CTRL_REG4_Z_ENABLE				((uint8_t)0x04) //binary code :0000 0100
#define LIS3DSH_CTRL_REG4_Z_DISABLE				((uint8_t)0x00) //binary code :0000 0000

// Configuration of the register CTRL_REG4 on an 8-bit binary code allowing to not update (1) or continuous update (0) the output registers
#define LIS3DSH_CTRL_REG4_BDU_ON				((uint8_t)0x00) //binary code :0000 0000
#define LIS3DSH_CTRL_REG4_BDU_OFF				((uint8_t)0x08) //binary code :0000 1000

// Configuration of the register CTRL_REG4 on an 8-bit binary code allowing to set the power mode (the frequencies)
#define LIS3DSH_CTRL_REG4_ODR_POWER_DOWN		((uint8_t)0x00) //binary code :0000 0000
#define LIS3DSH_CTRL_REG4_ODR_3_125_HZ			((uint8_t)0x10) //binary code :0001 0000
#define LIS3DSH_CTRL_REG4_ODR_6_25_HZ			((uint8_t)0x20) //binary code :0010 0000
#define LIS3DSH_CTRL_REG4_ODR_12_5_HZ			((uint8_t)0x30) //binary code :0011 0000
#define LIS3DSH_CTRL_REG4_ODR_25_HZ				((uint8_t)0x40) //binary code :0100 0000
#define LIS3DSH_CTRL_REG4_ODR_50_HZ				((uint8_t)0x50) //binary code :0101 0000
#define LIS3DSH_CTRL_REG4_ODR_100_HZ			((uint8_t)0x60) //binary code :0110 0000
#define LIS3DSH_CTRL_REG4_ODR_400_HZ			((uint8_t)0x70) //binary code :0111 0000
#define LIS3DSH_CTRL_REG4_ODR_800_HZ			((uint8_t)0x80) //binary code :1000 0000
#define LIS3DSH_CTRL_REG4_ODR_1600_HZ			((uint8_t)0x90) //binary code :1001 0000

// Configuration of the register CTRL_REG5 on an 8-bit binary code allowing to set the anti-aliasing filter bandwith
#define LIS3DSH_CTRL_REG5_BW_DEFAULT			((uint8_t)0x00) //binary code :0000 0000
#define LIS3DSH_CTRL_REG5_BW_800_HZ				((uint8_t)0x00) //binary code :0000 0000
#define LIS3DSH_CTRL_REG5_BW_200_HZ				((uint8_t)0x40) //binary code :0100 0000
#define LIS3DSH_CTRL_REG5_BW_400_HZ				((uint8_t)0x80) //binary code :1000 0000
#define LIS3DSH_CTRL_REG5_BW_50_HZ				((uint8_t)0xC0) //binary code :1100 0000

// Configuration of the register CTRL_REG5 on an 8-bit binary code allowing to select the full scale
#define LIS3DSH_CTRL_REG5_FSCALE_DEFAULT		((uint8_t)0x00) //binary code :0000 0000
#define LIS3DSH_CTRL_REG5_FSCALE_2_G			((uint8_t)0x00) //binary code :0000 0000
#define LIS3DSH_CTRL_REG5_FSCALE_4_G			((uint8_t)0x08) //binary code :0000 1000
#define LIS3DSH_CTRL_REG5_FSCALE_6_G			((uint8_t)0x10) //binary code :0001 0000
#define LIS3DSH_CTRL_REG5_FSCALE_8_G			((uint8_t)0x18) //binary code :0001 1000
#define LIS3DSH_CTRL_REG5_FSCALE_16_G			((uint8_t)0x20) //binary code :0010 0000


/* Return status define */
#define LIS3DSH_STATUS_OK			"LIS3DSH OK !\n\r"
#define LIS3DSH_STATUS_ERROR		"LIS3DSH ERROR !\n\r"


/* Init Sruct */
typedef struct
{
	uint8_t xEnable;
	uint8_t yEnable;
	uint8_t zEnable;
	uint8_t bduOn;
	uint8_t odr;
	uint8_t bw;
	uint8_t fscale;

}LIS3DSH_init_t;



/* status enum */
typedef enum
{
	LIS3DSH_OK,
	LIS3DSH_ERROR

}LIS3DSH_Status_t;


/* Write Function */
LIS3DSH_Status_t LIS3DSH_Write_reg(SPI_HandleTypeDef *acclSPI, uint8_t reg_addr, uint8_t *dataW, uint8_t size);


/* Read Function */
LIS3DSH_Status_t LIS3DSH_Read_reg(SPI_HandleTypeDef *acclSPI, uint8_t reg_addr, uint8_t *dataR, uint8_t size);

/* Init Function */
LIS3DSH_Status_t LIS3DSH_Init_CTRL_REG4(SPI_HandleTypeDef *acclSPI, uint8_t reg_addr, LIS3DSH_init_t *acclInitDef);
LIS3DSH_Status_t LIS3DSH_Init_CTRL_REG4(SPI_HandleTypeDef *acclSPI, uint8_t reg_addr, LIS3DSH_init_t *acclInitDef);


/* Get position X */
LIS3DSH_Status_t LIS3DSH_Get_X(SPI_HandleTypeDef *acclSPI, uint8_t reg_addr, float *accl);

/* Get position Y */
LIS3DSH_Status_t LIS3DSH_Get_Y(SPI_HandleTypeDef *acclSPI, uint8_t reg_addr, float *accl);

/* Get position Z */
LIS3DSH_Status_t LIS3DSH_Get_Z(SPI_HandleTypeDef *acclSPI, uint8_t reg_addr, float *accl);


#endif /* INC_YNV_LIS3DSH_H_ */
