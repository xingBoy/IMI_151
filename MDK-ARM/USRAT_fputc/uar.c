#include "uar.h"
/***************************************************************
    *��������Data_Turn
    *��  �룺��
    *˵  �������ڽ�����ɣ����ش��ڲ鿴�������
    *����ֵ����
  **/
	

void Data_Turn(void)
{ 
	HAL_UART_Receive_DMA(&huart3,rx_buffer,BUFFER_SIZE);  				//�ٴδ򿪴���3 DMA����
  if(recv_end_flag ==1)        
  {
      printf("UART_rx_len=%d\r\n",rx_len);                           //��ӡ���ճ���
			HAL_Delay(1);
		  HAL_UART_Transmit_DMA(&huart4,rx_buffer, rx_len);         //��DMAģʽ����������ͨ������4���͵�800C
			HAL_Delay(1);
		  printf("Send Date : %s \r\n",rx_buffer);                           //��ӡ���ճ���
//			HAL_Delay(1);       
//			HAL_UART_Transmit_DMA(&huart3,rx_buffer, rx_len);					//��DMAģʽ�ѽ�������ͨ������3�������
			for(char i=0;i<rx_len;i++)		rx_buffer[i]=0;						  //����ջ���
			rx_len=0;		 				//������ݳ���
			recv_end_flag=0;		//������ս�����־λ	
   }
}
/*
//DMA��ʽ���շ��ص�AT���
void Data_Turn1(void)
{ 
	HAL_UART_Receive_DMA(&huart4,rx_buffer1,BUFFER_SIZE);  				//�ٴδ򿪴���1 DMA����
  if(recv_end_flag_1 ==1)        
  {
    	HAL_Delay(1);  
		 //��ӡ���ճ���
			printf("800c_len=%d\r\n",rx_len_1);
			//printf("RX Date : %s \r\n",rx_buffer1);
      HAL_UART_Transmit_DMA(&huart3,rx_buffer1, rx_len_1);         //�����յ���800c����ͨ������3����			
			printf("\r\nrx_buffer1 = %s\r\n",rx_buffer1);		
			for(char i=0;i<rx_len_1;i++) rx_buffer1[rx_len_1]=0;	
			rx_len_1=0;		 				//������ݳ���
			recv_end_flag_1=0;		//������ս�����־λ	
   }
}
*/

