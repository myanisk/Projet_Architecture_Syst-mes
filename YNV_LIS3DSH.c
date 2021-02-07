/*
 * YNV_LIS3DSH.c
 *
 *  Created on: 12 janv. 2021
 *      Author: yanis
 */

#include "YNV_LIS3DSH.h"

LIS3DSH_Status_t LIS3DSH_Write_reg(SPI_HandleTypeDef *acclSPI,uint8_t reg_addr,uint8_t *dataW,uint8_t size)
{
	uint8_t buffer[2] = {reg_addr,*dataW};
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	if(HAL_SPI_Transmit(acclSPI, buffer, size, 10) == HAL_OK)
	{
		return LIS3DSH_OK;
	}
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
	return LIS3DSH_ERROR;

}

/*LIS3DSH_Status_t LIS3DSH_Read_reg(SPI_HandleTypeDef *acclSPI,uint8_t reg_addr,uint8_t *dataR,uint8_t size)
{
	reg_addr |= 0x80;
	HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);
		if (HAL_SPI_Transmit(acclSPI, &reg_addr, size, 10)==HAL_OK)
			{
				if(HAL_SPI_Receive(acclSPI, dataR, size, 10) == HAL_OK)
				{
					return LIS3DSH_OK;
				}
			}
			return LIS3DSH_ERROR;
	}*/

LIS3DSH_Status_t LIS3DSH_Read_reg(SPI_HandleTypeDef *acclSPI,uint8_t reg_addr,uint8_t *dataR,uint8_t size)
{
    uint8_t spiBuf[4];
    // Mask read bit on register addr
    spiBuf[0] = reg_addr | 0x80;
    //Enable CS
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,GPIO_PIN_RESET);
    // Send register addr
    if(HAL_SPI_Transmit(acclSPI, spiBuf, 1, 10) ==HAL_OK)
    {
        // Get Register value
        if(HAL_SPI_Receive(acclSPI, spiBuf, size, 10) == HAL_OK)
        {
            for(uint8_t i = 0; i < size ; i++)
            {
                dataR[i] = spiBuf[i];
            }
            //Disable CS
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
            return LIS3DSH_OK;
        }
    }
    //Disable CS
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    return LIS3DSH_ERROR;
}



/* conf CTRL_REG4 */

LIS3DSH_Status_t LIS3DSH_Init_CTRL_REG4(SPI_HandleTypeDef *acclSPI,uint8_t reg_addr,LIS3DSH_init_t *acclInitDef)
{
	uint8_t spiData = 0x00;
	uint8_t spiCheckData = 0x00;

	//Data register conf
	spiData |= (acclInitDef->xEnable & 0x01);
	spiData |= (acclInitDef->yEnable & 0x02);
	spiData |= (acclInitDef->zEnable & 0x04);
	spiData |= (acclInitDef->bduOn & 0x00);
	spiData |= (acclInitDef->odr & 0x10);

	if(LIS3DSH_Write_reg(acclSPI,LIS3DSH_CTRL_REG4,spiData,2) == LIS3DSH_OK)
	{
		if (LIS3DSH_Read_reg(acclSPI,LIS3DSH_CTRL_REG4,spiCheckData,1) == LIS3DSH_OK)
		{
			if(spiCheckData == spiData)
			{
				return LIS3DSH_OK;
			}
		}
	}
	return LIS3DSH_ERROR;
}




/* conf CTRL_REG5 */

LIS3DSH_Status_t LIS3DSH_Init_CTRL_REG5(SPI_HandleTypeDef *acclSPI ,uint8_t reg_addr, LIS3DSH_init_t *acclInitDef)
{
	uint8_t spiData = 0x00;
	uint8_t spiCheckData = 0x00;

	//Data register conf
	spiData |= (acclInitDef->fscale & 0x00);
	spiData |= (acclInitDef->bw & 0x80);



	if(LIS3DSH_Write_reg(acclSPI,LIS3DSH_CTRL_REG5,spiData,2) == LIS3DSH_OK)
	{
		if (LIS3DSH_Read_reg(acclSPI,LIS3DSH_CTRL_REG5,spiCheckData,1) == LIS3DSH_OK)
		{
				return LIS3DSH_OK;
		}
	}
	return LIS3DSH_ERROR;
}



/* ACCL X */

LIS3DSH_Status_t LIS3DSH_Get_X(SPI_HandleTypeDef *acclSPI,uint8_t reg_addr,float *accl)
{
	uint8_t dataR[2] = {0x00, 0x00};
	uint16_t acclX = 0x0000;

	if(LIS3DSH_Read_reg(acclSPI,LIS3DSH_OUT_X,dataR,2) == LIS3DSH_OK )
	{
		acclX = ((dataR[1]<<8) | dataR[0]);
//		*accl = acclX * Sensitivity;
		return LIS3DSH_OK;
	}
	return LIS3DSH_ERROR;
}

/* ACCL Y */

LIS3DSH_Status_t LIS3DSH_Get_Y(SPI_HandleTypeDef *acclSPI,uint8_t reg_addr,float *accl)
{
	uint8_t dataR[2] = {0x00, 0x00};
	uint16_t acclY = 0x0000;

	if(LIS3DSH_Read_reg(acclSPI,LIS3DSH_OUT_Y,dataR,2) == LIS3DSH_OK )
	{
		acclY = ((dataR[0]<<8) | dataR[1]);
		for ( uint8_t n=0; n <=15 ; n++)
		{
			*accl += ((acclY >> n) &1);
		}
		return LIS3DSH_OK;
	}
	return LIS3DSH_ERROR;
}

/* ACCL Z */

LIS3DSH_Status_t LIS3DSH_Get_Z(SPI_HandleTypeDef *acclSPI,uint8_t reg_addr,float *accl)
{
	uint8_t dataR[2] = {0x00, 0x00};
	uint16_t acclZ = 0x0000;

	if(LIS3DSH_Read_reg(acclSPI,LIS3DSH_OUT_Z,dataR,2) == LIS3DSH_OK )
	{
		acclZ = ((dataR[0]<<8) | dataR[1]);
		for ( uint8_t n=0; n <=15 ; n++)
		{
			*accl += ((acclZ >> n) &1);
		}
		return LIS3DSH_OK;
	}
	return LIS3DSH_ERROR;
}
