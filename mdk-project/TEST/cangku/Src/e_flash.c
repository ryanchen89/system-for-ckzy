/**
  ******************************************************************************
  * File Name          : e_flash.c
  * Description        : flash相关的用户操作
  *
  ******************************************************************************
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "e_flash.h"


/* define ------------------------------------------------------------------*/

#define STM_SECTOR_SIZE	1024
#define STM32_FLASH_BASE 0x08000000 	//FLASH 起始地址
#define STM32_FLASH_SIZE 256 	 		//闪存容量

#define u16 uint16_t
#define u32 uint32_t

/* Exported typedef ------------------------------------------------------------------*/



/* struct data ------------------------------------------------------------------*/

static uint8_t STMFLASH_BUF[STM_SECTOR_SIZE];//


/* Function ------------------------------------------------------------------*/


void E_CFG_WriteFlash(uint8_t *Dat,uint32_t Addr,uint16_t DatLen)
{
	uint16_t i;
	uint16_t write_data;
	uint32_t addr=Addr;
	
	u32 secpos;	   			//页地址
	u16 offset;	   			//偏移量
//	u16 odd; 					//剩余量
	u32 offaddr;  		 	//去掉0x800000后的地址
	
	offaddr=Addr-STM32_FLASH_BASE;			//实际偏移地址
	secpos=offaddr/STM_SECTOR_SIZE;			//页地址
	offset=(offaddr%STM_SECTOR_SIZE);		//扇区内偏移地址
//	odd=STM_SECTOR_SIZE/2-offset;			//扇区内剩余空间
	
	memcpy(STMFLASH_BUF,(uint8_t *)(STM32_FLASH_BASE+secpos*STM_SECTOR_SIZE),STM_SECTOR_SIZE);
	
    //1、解锁FLASH
	HAL_FLASH_Unlock();

    //2擦除FLASH
    //初始化FLASH_EraseInitTypeDef
    FLASH_EraseInitTypeDef f;
    f.TypeErase = FLASH_TYPEERASE_PAGES;
    f.PageAddress = STM32_FLASH_BASE+secpos*STM_SECTOR_SIZE;
    f.NbPages = 1;
    //设置PageError
    uint32_t PageError = 0; 
    //调用擦除函数
    HAL_FLASHEx_Erase(&f, &PageError);
	
	for(i=0;i<DatLen;i++)
	{
		STMFLASH_BUF[i+offset]=Dat[i];
	}
	//写入数据
	for(i=0;i<STM_SECTOR_SIZE/2;){
		write_data=((STMFLASH_BUF[i+1]));
		write_data=(write_data<<8)|(STMFLASH_BUF[i]);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+i,write_data);i+=2;
	}
	//锁定FLASH
  HAL_FLASH_Lock();
}



/************************ (C) COPYRIGHT INLEEQ *****END OF FILE****/
