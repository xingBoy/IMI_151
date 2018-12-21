#include "uar.h"
/***************************************************************
    *函数名：Data_Turn
    *输  入：无
    *说  明：串口接收完成，返回串口查看接收情况
    *返回值：无
  **/
	

void Data_Turn(void)
{ 
	HAL_UART_Receive_DMA(&huart3,rx_buffer,BUFFER_SIZE);  				//再次打开串口3 DMA接收
  if(recv_end_flag ==1)        
  {
      printf("UART_rx_len=%d\r\n",rx_len);                           //打印接收长度
			HAL_Delay(1);
		  HAL_UART_Transmit_DMA(&huart4,rx_buffer, rx_len);         //用DMA模式将接收数据通过串口4发送到800C
			HAL_Delay(1);
		  printf("Send Date : %s \r\n",rx_buffer);                           //打印接收长度
//			HAL_Delay(1);       
//			HAL_UART_Transmit_DMA(&huart3,rx_buffer, rx_len);					//用DMA模式把接收数据通过串口3传输出来
			for(char i=0;i<rx_len;i++)		rx_buffer[i]=0;						  //清接收缓存
			rx_len=0;		 				//清楚数据长度
			recv_end_flag=0;		//清除接收结束标志位	
   }
}
/*
//DMA方式接收返回的AT结果
void Data_Turn1(void)
{ 
	HAL_UART_Receive_DMA(&huart4,rx_buffer1,BUFFER_SIZE);  				//再次打开串口1 DMA接收
  if(recv_end_flag_1 ==1)        
  {
    	HAL_Delay(1);  
		 //打印接收长度
			printf("800c_len=%d\r\n",rx_len_1);
			//printf("RX Date : %s \r\n",rx_buffer1);
      HAL_UART_Transmit_DMA(&huart3,rx_buffer1, rx_len_1);         //将接收到的800c数据通过串口3传出			
			printf("\r\nrx_buffer1 = %s\r\n",rx_buffer1);		
			for(char i=0;i<rx_len_1;i++) rx_buffer1[rx_len_1]=0;	
			rx_len_1=0;		 				//清楚数据长度
			recv_end_flag_1=0;		//清除接收结束标志位	
   }
}
*/

