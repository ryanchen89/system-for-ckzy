/**
  ******************************************************************************
  * File Name          : e_flash.h
  * Description        : 本文档包含flash相关的用户操作 
  *
  ******************************************************************************
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __e_flash_H
#define __e_flash_H


/* Includes ------------------------------------------------------------------*/

#include "stm32f1xx_hal.h"
//#include "string.h"
//#include "rtc.h"
//#include "stdio.h"
//#define E_MBUS 




extern void E_CFG_WriteFlash(uint8_t *Dat,uint32_t Addr,uint16_t DatLen);


#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT INLEEQ *****END OF FILE****/
