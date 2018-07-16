/**
  ******************************************************************************
  * File Name          : config.h
  * Description        : 本文档包含全局配置和需要存储的结构体定义，文件等
  *
  ******************************************************************************
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONFIG__
#define __CONFIG__


/* Includes ------------------------------------------------------------------*/

#include "stm32f1xx_hal.h"


/* define ------------------------------------------------------------------*/


#define WRITE_ADDR_CFG	0x8008000

/* Exported typedef ------------------------------------------------------------------*/
typedef struct
{

	uint16_t	DI1;
	uint16_t	DI2;
	uint16_t	DI3;
	uint16_t	DI4;
	uint16_t	DI5;
	uint16_t	DI6;
	uint16_t	DI7;
	uint16_t	DI8;
	uint16_t	DI9;
	uint16_t	DI10;
	uint16_t	DI11;
	uint16_t	DI12;
	
	
	//保留字段

}Config_Date_Read_Def;

typedef struct
{

	GPIO_PinState	DO1;
	GPIO_PinState	DO2;
	GPIO_PinState	DO3;
	GPIO_PinState	DO4;
	GPIO_PinState	DO5;
	GPIO_PinState	DO6;
	GPIO_PinState	DO7;
	GPIO_PinState	DO8;
	
}Config_Date_Set_Def;





typedef struct
{
	
	uint16_t	beg_flag;
	
	Config_Date_Set_Def Write_Data_def;
	
	uint16_t  end_flag;
	
}config_date_save_Def;











/* Private function prototypes -----------------------------------------------*/

//extern config_date_save_Def config_date;

extern Config_Date_Read_Def		*Read_data;
extern Config_Date_Set_Def		*Write_data;


extern Config_Date_Read_Def DI;
extern Config_Date_Set_Def  DO;

extern void Config_Init(void);

extern uint8_t Config_Save(void);


#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT INLEEQ *****END OF FILE****/
