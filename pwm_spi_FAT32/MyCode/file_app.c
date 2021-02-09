/*
 * file_app.c
 *
 *  Created on: Feb 5, 2021
 *      Author: Administrator
 */
#include "file_app.h"
// BYTE WriteBuffer[]= "asdfasdfasdfasdfasfasdf\n";/* 写缓冲区*/
 FATFS fs;													/* FatFs文件系统对象 */
void app_initfile()
{
	FRESULT f_res;
	    //在SD卡挂载文件系统，文件系统挂载时会对SD卡初始化
	    f_res = f_mount(&fs,(TCHAR const*)USERPath,1);
	 //   printf_fatfs_error(f_res);
	    /*----------------------- 格式化测�?? ---------------------------*/
	    /* 如果没有文件系统就格式化创建创建文件系统 */
	    if(f_res == FR_NO_FILESYSTEM)
	    {
	      printf("The SD card has no file system and will be formatted soon...\n");
	      /* 格式�?? */
	      f_res=f_mkfs((TCHAR const*)USERPath,0,0);

	      if(f_res == FR_OK)
	      {
	        printf("The SD card has successfully formatted the file system.\n");
	        /* 格式化后，先取消挂载 */
	        f_res = f_mount(NULL,(TCHAR const*)USERPath,0);
	        /* 重新挂载	*/
	        f_res = f_mount(&fs,(TCHAR const*)USERPath,1);
	      }
	      else
	      {
	        printf("<<Format failed.>>\n");
	        while(1);
	      }
	    }
	    else if(f_res!=FR_OK)
	    {
	      printf("！！ The SD card failed to mount the file system. (%d)\n",f_res);
	   //   printf_fatfs_error(f_res);
	      while(1);
	    }
	    else
	    {
	      printf(">>The file system is mounted successfully and can be read and write tested\n");
	    }

}

void app_FATFS_Run(FIL pf,char *buffer)
{
//	char bu[512]={0};
//	sprintf(bu,"%s\r\n",buffer);
  UINT writeBytes;
  /*文件系统基本操作：打开文件，定位到文件结尾，写入内容，关闭文件*/
  if(f_open(&pf, "test.txt", FA_READ|FA_WRITE|FA_OPEN_ALWAYS) != FR_OK)
  {
	  printf("open error\n");
    while(1);

  }
  if(f_lseek(&pf, f_size(&pf)) != FR_OK)
  {
	  printf("lseek error\n");
    while(1);
  }
  if(f_write(&pf, buffer, sizeof(buffer), &writeBytes) != FR_OK)
  {
	  printf("write error\n");
    while(1);
  }
  printf("writeBytes:%d\r\n",writeBytes);
  if(f_close(&pf) != FR_OK)
  {
	  printf("close error\n");
    while(1);
  }
}
/*
void app_writefile( FIL pf)
{

	FRESULT res;
	UINT  bw;
	  res = f_open(&pf, "test.txt", FA_OPEN_ALWAYS | FA_WRITE);
	  if (res == FR_OK)
	  {
	    printf("creat ok\n");
	  }
	  else
	  {
	    printf("creat failed\n");
	    printf("error code: %d\n",res);
	  }

	//  f_printf(&pf, "time%d-%d-%d    %d:%d:%d*",1, 2, 2, 3, 4, 5);		//	格式化输出
		f_write(&pf, WriteBuffer, sizeof(WriteBuffer), &bw);
	    if (res != FR_OK)
	    {
	      printf("close file error\n");
	      printf("error code: %d\n",res);
	    }
		else
		{
			printf("bw:%d\r\n",bw);
				printf("write data OK");
		}
		res = f_close(&pf);
		if (res != FR_OK)
		{
		printf("close file error\n");
		printf("error code: %d\n",res);
		}
}*/

uint8_t app_readfile(FIL file, uint8_t num)
{
  //FIL file;
  FRESULT res;
  unsigned int bw;
  uint8_t static rbuf[100] = {0};

  res = f_open(&file, "test.txt", FA_READ);
  if (res != FR_OK)
  {
    printf("open error: %d\n",res);
  }
  else
  {
	f_read(&file, rbuf, num, &bw);

	printf("read data:%s\n", rbuf);

	res = f_close(&file);
	if (res != FR_OK)
	{
	printf("close file error\n");
	printf("error code: %d\n",res);
	}
  }
	return rbuf;
}




