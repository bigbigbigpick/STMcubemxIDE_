/*
 * spi_sca100T.c
 *
 *  Created on: Feb 7, 2021
 *      Author: Administrator
 */

#include "spi_sca100T.h"






uint8_t angularflag=0;

//STM32 软件模拟SPI



void An_Tran_Init(void)
{
  An_Tran_WR_Byte(STX);  //自检
  HAL_Delay(1);
  An_Tran_WR_Byte(STY);  //自检
  HAL_Delay(1);
  An_Tran_WR_Byte(MEAS); //工作在测量模式
  HAL_Delay(1);
}

void An_Tran_WR_Byte(uint8_t command_)
{
  uint8_t it=0,i=0;
  CSB_CLR;
  it=command_;

  for(i=0;i<8;i++)
  {
    SCK_CLR;
    delay_us(2);
  //  HAL_DEl
    if(it&0x80)
    {
      MOSI_SET;
    }
    else
    {
      MOSI_CLR;
    }
    SCK_SET;
    it=(it<<1);
    delay_us(2);
  }
  CSB_SET;
  SCK_CLR;
  delay_us(150);
}


uint16_t An_Tran_Read_Byte(uint8_t command_)
{
  uint8_t i=0;
  uint16_t indata=0;

  CSB_CLR;
  for(i=0;i<8;i++)   // 发送命令
  {
    SCK_CLR;
    delay_us(1);
    if(command_&0x80)
    {
      MOSI_SET;
    }
    else
    {
      MOSI_CLR;
    }
    SCK_SET;
    command_=(command_<<1);
    delay_us(1);
  }
  SCK_CLR;
  indata=0;
  for(i=0; i<11; i++)  //11位数据
  {
    SCK_SET;
    indata=indata<<1;
    if(HAL_GPIO_ReadPin(GPIOB, sca100_MISO_Pin)!=0)  // 读取输入值
    {
      indata=indata+1;
    }
    SCK_CLR;
    delay_us(1);
  }
  CSB_SET;
  delay_us(50);
  return(indata);
  }

//如果读取的数据小于1024则代表右倾即左摆，大于1024则代表左倾即右摆 ，计算方法为：角度=arcsin【（输出的数据-1024）/819】
float AngularConvert(uint8_t Channel)    //转换角度函数，45.45度为4545，最高位为1为右倾也即左摆
{
  float tempbuf;
  float tempbuf1;
  float tempbuf2;
  uint16_t rawbuf;
  An_Tran_WR_Byte(0x0E); //工作在自检模式
  An_Tran_WR_Byte(0x00); //工作在测量模式
  rawbuf = AngularDataProcess(Channel);
  if(rawbuf==0xFFFF)//无效
    return 0;
  tempbuf1 = (float)(rawbuf);
  if(tempbuf1<=1024)
  {

    tempbuf2=(asinf((1024-tempbuf1)/SCA100T_D02_Resolution))*180/3.1415926;
    tempbuf=tempbuf2;
  }
  else
  {
    tempbuf2=(asinf((tempbuf1-1024)/SCA100T_D02_Resolution))*180/3.1415926;
    tempbuf=(-1)*tempbuf2;
  }
  return  tempbuf;
}



uint16_t AngularDataProcess(uint8_t Channel)//倾角数据处理
{
  uint16_t themp;
  uint16_t AngleSensorbuf[100]={0};
  uint8_t  i,j;
  j = 0;
  for(i=0;i<100;i++)//循环读数
  {
    themp=An_Tran_Read_Byte(Channel);
    if((themp>=205)&&(themp<=1843))//有效数据
    {
      AngleSensorbuf[j]=themp;
      j++;//有效点数量
    }
  }
  if(j>0)
  {
    selectSort(AngleSensorbuf,j);
   printf("buf:%d\r\n",AngleSensorbuf[j/2]);
    return AngleSensorbuf[j/2];
  }
  else
  {
    return 0xFFFF;
  }
}
void selectSort(uint16_t *a,uint8_t n)
{
    int min_;
    for(int i=0;i<n-1;i++){
        min_=i;
        for(int j=i+1;j<n;j++){
            if(a[min_]>a[j]) min_=j;
        }
        if(min_!=i){
            int temp=a[i];
            a[i]=a[min_];
            a[min_]=temp;
        }
    }
}

void delay_us(uint32_t us)
{
    uint32_t delay = (HAL_RCC_GetHCLKFreq() / 4000000 * us);
    while (delay--)
	{
		;
	}
}

