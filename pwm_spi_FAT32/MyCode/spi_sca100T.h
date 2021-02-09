/*
 * spi_sca100T.h
 *
 *  Created on: Feb 7, 2021
 *      Author: Administrator
 */

#ifndef SPI_SCA100T_H_
#define SPI_SCA100T_H_

#include "main.h"
#include <math.h>

//========SCA100T控制命令
#define MEAS  0x00//测量模式
#define RWTR  0x08//读写温度数据寄存器
#define RDSR  0x0a
#define RLOAD 0x0b
#define STX   0x0e//X通道自检
#define STY   0x0f//Y通道自检
#define RDAX  0x10 //读X通道数据
#define RDAY  0x11 //读Y通道数据

//SCA100T灵敏度
#define  SCA100T_D01_Resolution    1638
#define SCA100T_D02_Resolution    819

 //IO操作函数
#define SCK_SET     HAL_GPIO_WritePin(GPIOB,sca100_SCK_Pin,GPIO_PIN_SET)
#define SCK_CLR     HAL_GPIO_WritePin(GPIOB,sca100_SCK_Pin,GPIO_PIN_RESET)
#define MOSI_SET    HAL_GPIO_WritePin(GPIOB,sca100_MOSI_Pin,GPIO_PIN_SET)
#define MOSI_CLR    HAL_GPIO_WritePin(GPIOB,sca100_MOSI_Pin,GPIO_PIN_RESET)
#define CSB_SET     HAL_GPIO_WritePin(GPIOB,sca100_NSS_Pin,GPIO_PIN_SET)
#define CSB_CLR     HAL_GPIO_WritePin(GPIOB,sca100_NSS_Pin,GPIO_PIN_RESET)


void An_Tran_Init(void);
void An_Tran_WR_Byte(uint8_t command_);
uint16_t An_Tran_Read_Byte(uint8_t command_);
float AngularConvert(uint8_t Channel);
uint16_t AngularDataProcess(uint8_t Channel);
void selectSort(uint16_t *a,uint8_t n);
void delay_us(uint32_t us);
#endif /* SPI_SCA100T_H_ */
