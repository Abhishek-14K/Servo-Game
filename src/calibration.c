/*
 * calibration.c
 *
 *  Created on: Apr 3, 2022
 *      Author: abhis
 */
#include "main.h"
#include "mfs.h"
#include "stm32l476xx.h"
UART_HandleTypeDef huart2;


volatile int cal1_sig, cal2_sig ;
char buffer[200];
//int button1flag, button3flag;
//int min_signal, max_signal;

int cal1() // Calibration for minimum signal
{
	int servocal1 = 0;
	int button1flag = 0;
	int button2flag = 0;
	int button3flag = 0;

		TIM2->CCR1 = 30; // Send servo to a least known position
		cal1_sig = 30;


	while (servocal1 == 0)
	{

		if (HAL_GPIO_ReadPin(SHLD_A1_GPIO_Port, SHLD_A1_Pin)==GPIO_PIN_RESET && button1flag == 0) // Button 1 pressed
			  {
				  set_led_1(1);
				  set_led_2(0);
				  set_led_3(0);
				  //cal1_sig + 30;
				  cal1_sig -= 20; // Decrement signal value
				  //TIM2->CCR1 = cal1_sig-5;
				  //cal1_sig = TIM2->CCR1;
				  //HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\n\r%d",1), 500);
				  button1flag = 1;
			  }



		if (HAL_GPIO_ReadPin(SHLD_A1_GPIO_Port, SHLD_A1_Pin)==GPIO_PIN_SET) // Button 1 released
		  {
			  set_led_1(0);
			  set_led_2(0);
			  set_led_3(0);
			  button1flag = 0;
		  }



	  if (HAL_GPIO_ReadPin(SHLD_A3_GPIO_Port, SHLD_A3_Pin)==GPIO_PIN_RESET && button3flag == 0) // Button 3 pressed
	  {
		  set_led_1(0);
		  set_led_2(0);
		  set_led_3(1);
		  cal1_sig += 20; // Increment signal value

		  //cal1_sig = TIM2->CCR1;
		  //HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\n\r%d",2), 500);
		  button3flag = 1;

	  }

	  if (HAL_GPIO_ReadPin(SHLD_A3_GPIO_Port, SHLD_A3_Pin)==GPIO_PIN_SET) //Button 3 relesed
	  {
		  set_led_1(0);
		  set_led_2(0);
		  set_led_3(0);
		  button3flag = 0;

	  }

	  if (HAL_GPIO_ReadPin(SHLD_A2_GPIO_Port, SHLD_A2_Pin)==GPIO_PIN_RESET && button2flag == 0) // Button 2 pressed
	  {
		  set_led_1(0);
		  set_led_2(1);
		  set_led_3(0);
		 // HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\n\r%d",3), 500);
		  button2flag = 1;


	  }

	  if (HAL_GPIO_ReadPin(SHLD_A2_GPIO_Port, SHLD_A2_Pin)==GPIO_PIN_SET) // Button 2 relesed
	  {
		  set_led_1(0);
		  set_led_2(0);
		  set_led_3(0);
		  if (button2flag == 1)
		  {
			  servocal1 = 1; // Condition to end loop
			  button2flag = 0;
		  }




	  }

	  TIM2->CCR1 = cal1_sig; // Move servo to position
	}
	return cal1_sig; // Return signal value
}

int cal2()
{
	int servocal1 = 0;
	int button1flag = 0;
	int button2flag = 0;
	int button3flag = 0;

		TIM2->CCR1 = 130; // Move servo to maximum known position
		cal1_sig = 130;


	while (servocal1 == 0)
	{

		if (HAL_GPIO_ReadPin(SHLD_A1_GPIO_Port, SHLD_A1_Pin)==GPIO_PIN_RESET && button1flag == 0)
			  {
				  set_led_1(1);
				  set_led_2(0);
				  set_led_3(0);
				  //cal1_sig + 30;
				  cal1_sig -= 20;
				  //TIM2->CCR1 = cal1_sig-5;
				  //cal1_sig = TIM2->CCR1;
				  //HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\n\r%d",4), 500);
				  button1flag = 1;
			  }



		if (HAL_GPIO_ReadPin(SHLD_A1_GPIO_Port, SHLD_A1_Pin)==GPIO_PIN_SET)
		  {
			  set_led_1(0);
			  set_led_2(0);
			  set_led_3(0);
			  button1flag = 0;
		  }



	  if (HAL_GPIO_ReadPin(SHLD_A3_GPIO_Port, SHLD_A3_Pin)==GPIO_PIN_RESET && button3flag == 0)
	  {
		  set_led_1(0);
		  set_led_2(0);
		  set_led_3(1);
		  cal1_sig += 20;

		  //cal1_sig = TIM2->CCR1;
		  //HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\n\r%d",5), 500);
		  button3flag = 1;

	  }

	  if (HAL_GPIO_ReadPin(SHLD_A3_GPIO_Port, SHLD_A3_Pin)==GPIO_PIN_SET)
	  {
		  set_led_1(0);
		  set_led_2(0);
		  set_led_3(0);
		  button3flag = 0;

	  }

	  if (HAL_GPIO_ReadPin(SHLD_A2_GPIO_Port, SHLD_A2_Pin)==GPIO_PIN_RESET && button2flag == 0)
	  {
		  set_led_1(0);
		  set_led_2(1);
		  set_led_3(0);
		  //HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\n\r%d",6), 500);
		  button2flag = 1;


	  }

	  if (HAL_GPIO_ReadPin(SHLD_A2_GPIO_Port, SHLD_A2_Pin)==GPIO_PIN_SET)
	  {
		  set_led_1(0);
		  set_led_2(0);
		  set_led_3(0);
		  if (button2flag == 1)
		  {
			  servocal1 = 1;
			  button2flag = 0;
		  }


	  }

	  TIM2->CCR1 = cal1_sig;
	}
	return cal1_sig;
}

/*
void test()
{
	TIM2->CCR2 = 30;
	HAL_Delay(1000);
	TIM2->CCR2 = 50;
	HAL_Delay(1000);
	TIM2->CCR2 = 70;
	HAL_Delay(1000);
	TIM2->CCR2 = 90;
	HAL_Delay(1000);
	TIM2->CCR2 = 110;
	HAL_Delay(1000);
	TIM2->CCR2 = 130;
	HAL_Delay(1000);
}
*/

