/**************************************************
*
*
*
*设备参数设置文件，用于保存和初始化各类参数，读，写各类参数等
*
*
*
***************//////////////////////////////////////////////

#include "config.h"
#include "e_flash.h"




config_date_save_Def config_date_save_def=
{
	0x5555f,
	
	{
		0
	},
	
	0xaaaa
};


Config_Date_Read_Def	*Read_data;
Config_Date_Set_Def		*Write_data;

config_date_save_Def config_date={0};


Config_Date_Read_Def DI;
Config_Date_Set_Def  DO;


void Config_Init(void)
{
	config_date_save_Def *config_date_save=(config_date_save_Def *)WRITE_ADDR_CFG;
	
	config_date=*config_date_save;
	
	if((0x5555!=config_date.beg_flag)||(0xaaaa!=config_date.end_flag))
	{
		E_CFG_WriteFlash((uint8_t *)(&config_date_save_def),WRITE_ADDR_CFG,sizeof(config_date_save_Def));
		config_date=config_date_save_def;
	}
	
//	Read_data=(Config_Date_Read_Def	*)(&E_MBUS_DATA_ALL.mbus_wrreg[0]);
//	Write_data=(Config_Date_Set_Def	*)(&E_MBUS_DATA_ALL.mbus_wrreg[100]);
//	*Write_data=config_date.Write_Data_def;
//	Write_data->set_status=0;
//	config_date.Write_Data_def.set_status=0;
//	E_MBUS_DATA_ALL.mbus_id=Write_data->Dsc_Modbus_address;
}


uint8_t Config_Save(void)
{
	if(0!=memcmp(Write_data,&(config_date.Write_Data_def),sizeof(Config_Date_Set_Def)))
	{
		config_date.Write_Data_def=*Write_data;
		E_CFG_WriteFlash((uint8_t *)(&config_date),WRITE_ADDR_CFG,sizeof(config_date_save_Def));
		return 1;
	}
	else
		return 0;
}






















