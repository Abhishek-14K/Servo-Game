/*
 * mfs.c
 *
 *  Created on: Apr 3, 2022
 *      Author: abhis
 */
#include "main.h"
/* Byte maps to select digit 0 to 9 */
const char SEGMENT_MAP[] = {0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0X80,0X90};
/* Byte maps to select digit 1 to 4 */
const char SEGMENT_SELECT[] = {0xF1,0xF2,0xF4,0xF8};

void set_led_1( uint32_t on ) // Function to turn on LED1 on MFS
{
	if ( on ) // a5
		HAL_GPIO_WritePin(SHLD_D13_GPIO_Port, SHLD_D13_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(SHLD_D13_GPIO_Port, SHLD_D13_Pin, GPIO_PIN_SET);
}

void set_led_2( uint32_t on ) // Function to turn on LED2 on MFS
{
	if ( on ) //a6
		HAL_GPIO_WritePin(SHLD_D12_GPIO_Port, SHLD_D12_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(SHLD_D12_GPIO_Port, SHLD_D12_Pin, GPIO_PIN_SET);
}

void set_led_3( uint32_t on ) // Function to turn on LED3 on MFS
{
	if ( on ) //a7
		HAL_GPIO_WritePin(SHLD_D11_GPIO_Port, SHLD_D11_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(SHLD_D11_GPIO_Port, SHLD_D11_Pin, GPIO_PIN_SET);
}


void shiftOut(uint8_t val)
{
      for(int ii=0x80; ii; ii>>=1) {
    	  HAL_GPIO_WritePin(SHLD_D7_SEG7_Clock_GPIO_Port,SHLD_D7_SEG7_Clock_Pin, GPIO_PIN_RESET);    // clear clock pin
      		if(ii & val)						                                                     // if this bit in `value` is set
      			HAL_GPIO_WritePin(SHLD_D8_SEG7_Data_GPIO_Port, SHLD_D8_SEG7_Data_Pin,GPIO_PIN_SET);  //   set it in shift register
      		else
      			HAL_GPIO_WritePin(SHLD_D8_SEG7_Data_GPIO_Port, SHLD_D8_SEG7_Data_Pin,GPIO_PIN_RESET); 	//   else clear it

      		HAL_GPIO_WritePin(SHLD_D7_SEG7_Clock_GPIO_Port,SHLD_D7_SEG7_Clock_Pin, GPIO_PIN_SET);       // set clock pin
      	}
}

/* Write a decimal number between 0 and 9 to one of the 4 digits of the display */
void WriteNumberToSegment(int Segment, int Value)
{
  HAL_GPIO_WritePin(SHLD_D4_SEG7_Latch_GPIO_Port, SHLD_D4_SEG7_Latch_Pin, GPIO_PIN_RESET);
  shiftOut(SEGMENT_MAP[Value]);
  shiftOut(SEGMENT_SELECT[Segment] );
  HAL_GPIO_WritePin(SHLD_D4_SEG7_Latch_GPIO_Port, SHLD_D4_SEG7_Latch_Pin, GPIO_PIN_SET);
}


void segdisplay(int x) // Function to display a number on MFS 7 segment display
{
	if (x < 10 && x >= 0)
	{
		WriteNumberToSegment(3, x);
	}

	if (x < 100 && x >= 10)
	{
		WriteNumberToSegment(3, x%10);
		WriteNumberToSegment(2, x/10);
	}

	if (x < 1000 && x >= 100)
	{
		WriteNumberToSegment(3, x%10);
		WriteNumberToSegment(2, (x/10)%10);
		WriteNumberToSegment(1, x/100);
	}

	if (x < 10000 && x >= 1000)
	{
		WriteNumberToSegment(3, x%10);
		WriteNumberToSegment(2, (x/10)%10);
		WriteNumberToSegment(1, (x/100)%10);
		WriteNumberToSegment(0, x/1000);
	}
}
