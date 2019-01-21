/*
 * ui.c
 *
 *  Created on: Dec 18, 2018
 *      Author: jeffsnyder
 */
#include "main.h"
#include "tim.h"
#include "ui.h"
#include "dac.h"

//these are some scaling arrays to try to match the brightness levels of the RGB leds
uint16_t redScaled[17] = {0, 6550, 6660, 6750, 6840, 7000, 7080, 7170, 7250, 7350, 7550, 7900, 8200, 8500, 9300, 9900, 11000};
uint16_t greenScaled[17] = {0, 8, 9, 10, 11, 13, 16, 22, 30, 40, 60, 110, 180, 300, 400, 700, 900};
uint16_t blueScaled[17] = {0, 8, 9, 10, 11, 14, 17, 22, 25, 30, 40, 50, 70, 90, 150, 250, 300};

int DAC1_active = 0;
int DAC2_active = 0;

int mode1 = 0;
int mode2 = 0;
int mode3 = 0;
int RGB_mode = 3;

void RGB_LED_setColor(uint8_t Red, uint8_t Green, uint8_t Blue) //inputs between 0-255
{
	float floatyPoint;
	uint8_t intPart;
	float fractPart;
	float endValue;

	floatyPoint = ((float)Red) * 0.0625f;
	intPart = (uint8_t)floatyPoint;
	fractPart = floatyPoint - ((float)intPart);
	endValue = (redScaled[intPart] * (1.0f - fractPart)) + (redScaled[intPart + 1] * (fractPart));
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint16_t) endValue);

	floatyPoint = ((float)Green) * 0.0625f;
	intPart = (uint8_t)floatyPoint;
	fractPart = floatyPoint - ((float)intPart);
	endValue = (greenScaled[intPart] * (1.0f - fractPart)) + (greenScaled[intPart + 1] * (fractPart));
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t) endValue);

	floatyPoint = ((float)Blue) * 0.0625f;
	intPart = (uint8_t)floatyPoint;
	fractPart = floatyPoint - ((float)intPart);
	endValue = (blueScaled[intPart] * (1.0f - fractPart)) + (blueScaled[intPart + 1] * (fractPart));
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t) endValue);

}


void configure_Jack(uint8_t jackNumber, jackModeType jackMode)
{
	 GPIO_InitTypeDef GPIO_InitStruct;

	if (jackNumber == 1)
	{

		if (jackMode == DIGITAL_INPUT) //be sure to set the jumpers on the IO board for"I/O 5" both to input
		{
			  GPIO_InitStruct.Pin = GPIO_PIN_1;
			  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
			  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		}

		else if (jackMode == DIGITAL_OUTPUT) //be sure to set the jumpers on the IO board for"I/O 5" both to output
		{
			  GPIO_InitStruct.Pin = GPIO_PIN_1;
			  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			  GPIO_InitStruct.Pull = GPIO_NOPULL;
			  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		}

		else if (jackMode == ANALOG_INPUT) //be sure to set the jumpers on the IO board for"I/O 5" both to input
		{
			//do nothing, already configured
		}

		else
		{
			Invalid_Configuration();
		}
	}

	else if (jackNumber == 2)
	{

		if (jackMode == DIGITAL_INPUT) //be sure to set the jumpers on the IO board for"I/O 6" both to input
		{
			  GPIO_InitStruct.Pin = GPIO_PIN_0;
			  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
			  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		}

		else if (jackMode == DIGITAL_OUTPUT) //be sure to set the jumpers on the IO board for"I/O 6" both to output
		{
			  GPIO_InitStruct.Pin = GPIO_PIN_0;
			  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			  GPIO_InitStruct.Pull = GPIO_NOPULL;
			  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		}

		else if (jackMode == ANALOG_INPUT) //be sure to set the jumpers on the IO board for"I/O 6" both to input
		{
			//do nothing, already configured
		}

		else
		{
			Invalid_Configuration();
		}
	}

	else if (jackNumber == 3)
	{

		if (jackMode == DIGITAL_INPUT) //be sure to set the jumpers on the IO board for"I/O 7" both to input
			//also put jumper A on 3 and jumper B on 3
		{
			  GPIO_InitStruct.Pin = GPIO_PIN_5;
			  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
			  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		}

		else if (jackMode == DIGITAL_OUTPUT) //be sure to set the jumpers on the IO board for"I/O 7" both to output
			//also put jumper A on 3 and jumper B on 3
		{
			  GPIO_InitStruct.Pin = GPIO_PIN_5;
			  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			  GPIO_InitStruct.Pull = GPIO_NOPULL;
			  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		}

		else if (jackMode == ANALOG_INPUT) //be sure to set the jumpers on the IO board for"I/O 7" both to input
			//also put jumper A on 3 and jumper B on 3
		{
			//do nothing, already configured
		}

		else if (jackMode == ANALOG_OUTPUT) //put jumper A on 1 and jumper B on 1
		{
			//keep pin in analog mode (no pull up or down) but initialize it with the DAC
			MX_DAC1_Init(2);
			GPIO_InitStruct.Pin = GPIO_PIN_4;
			GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			DAC2_active = 1;
		}

		else
		{
			Invalid_Configuration();
		}
	}

	else if (jackNumber == 4)
	{

		if (jackMode == DIGITAL_INPUT) //be sure to set the jumpers on the IO board for"I/O 8" both to input
			//also put jumper C on 3 and jumper D on 3
		{
			  GPIO_InitStruct.Pin = GPIO_PIN_4;
			  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
			  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		}

		else if (jackMode == DIGITAL_OUTPUT) //be sure to set the jumpers on the IO board for"I/O 8" both to output
			//also put jumper C on 3 and jumper D on 3
		{
			  GPIO_InitStruct.Pin = GPIO_PIN_4;
			  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			  GPIO_InitStruct.Pull = GPIO_NOPULL;
			  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		}

		else if (jackMode == ANALOG_INPUT) //be sure to set the jumpers on the IO board for"I/O 8" both to input
			//also put jumper C on 3 and jumper D on 3
		{
			//do nothing, already configured
		}

		else if (jackMode == ANALOG_OUTPUT) // put jumper C on 1 and jumper D on 1
		{
			//keep pin in analog mode (no pull up or down) but initialize it with the DAC

			MX_DAC1_Init(1);
			GPIO_InitStruct.Pin = GPIO_PIN_4;
			GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			DAC1_active = 1;
		}

		else
		{
			Invalid_Configuration();
		}
	}

	else if (jackNumber == 5)
	{

		if (jackMode == DIGITAL_INPUT) //set jumper E to 3
		{
			  GPIO_InitStruct.Pin = GPIO_PIN_12;
			  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
			  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		}

		else if (jackMode == AUDIO_INPUT) //set jumper E to 1
		{
			//do nothing, already configured
		}

		else
		{
			Invalid_Configuration();
		}
	}

	else if (jackNumber == 6)
	{

		if (jackMode == DIGITAL_INPUT) //set jumper F to 3
		{
			  GPIO_InitStruct.Pin = GPIO_PIN_13;
			  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
			  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		}

		else if (jackMode == AUDIO_INPUT) //set jumper F to 1
		{
			//do nothing, already configured
		}

		else
		{
			Invalid_Configuration();
		}
	}

	else if (jackNumber == 7)
	{

		if (jackMode == DIGITAL_OUTPUT) //set jumper G to 3
		{
			  GPIO_InitStruct.Pin = GPIO_PIN_14;
			  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			  GPIO_InitStruct.Pull = GPIO_NOPULL;
			  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		}

		else if (jackMode == AUDIO_OUTPUT) //set jumper G to 1
		{
			//do nothing, already configured
		}

		else
		{
			Invalid_Configuration();
		}
	}

	else if (jackNumber == 8)
	{

		if (jackMode == DIGITAL_OUTPUT) //set jumper H to 3
		{
			  GPIO_InitStruct.Pin = GPIO_PIN_15;
			  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			  GPIO_InitStruct.Pull = GPIO_NOPULL;
			  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		}

		else if (jackMode == AUDIO_OUTPUT) //set jumper H to 1
		{
			//do nothing, already configured
		}

		else
		{
			Invalid_Configuration();
		}
	}

	else if (jackNumber == 9)
	{

		if (jackMode == DIGITAL_INPUT) //set "I/O board 1" to input (both jumpers), and set jumper I to 3
		{
			  GPIO_InitStruct.Pin = GPIO_PIN_11;
			  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
			  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
		}

		else if (jackMode == DIGITAL_OUTPUT) //set "I/O board 1" to output (both jumpers), and set jumper I to 3
		{
			  GPIO_InitStruct.Pin = GPIO_PIN_11;
			  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			  GPIO_InitStruct.Pull = GPIO_NOPULL;
			  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
		}

		else if (jackMode == ANALOG_INPUT) //set "I/O board 1" to input (both jumpers), and set jumper I to 1 and jumper K to 1 (takes away input from knob 5)
		{
			//do nothing, already configured
		}

		else
		{
			Invalid_Configuration();
		}
	}

	else if (jackNumber == 10)
	{

		if (jackMode == DIGITAL_INPUT) //set "I/O board 2" to input (both jumpers), and set jumper J to 3
		{
			  GPIO_InitStruct.Pin = GPIO_PIN_12;
			  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
			  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
		}

		else if (jackMode == DIGITAL_OUTPUT) //set "I/O board 2" to output (both jumpers), and set jumper J to 3
		{
			  GPIO_InitStruct.Pin = GPIO_PIN_12;
			  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			  GPIO_InitStruct.Pull = GPIO_NOPULL;
			  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
		}

		else if (jackMode == ANALOG_INPUT) //set "I/O board 2" to input (both jumpers), and set jumper J to 1 and jumper L to 1 (takes away input from knob 6)
		{
			//do nothing, already configured
		}

		else
		{
			Invalid_Configuration();
		}
	}

	else if (jackNumber == 11)
	{

		if (jackMode == DIGITAL_INPUT) //set "I/O board 3" to input (both jumpers), and set jumper M to 1
		{
			  GPIO_InitStruct.Pin = GPIO_PIN_12;
			  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
			  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
		}

		else if (jackMode == DIGITAL_OUTPUT) //set "I/O board 3" to output (both jumpers), and set jumper M to 1
		{
			  GPIO_InitStruct.Pin = GPIO_PIN_12;
			  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			  GPIO_InitStruct.Pull = GPIO_NOPULL;
			  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
		}

		else if (jackMode == ANALOG_INPUT) //set "I/O board 3" to input (both jumpers), and set jumper M to 1
		{
			//do nothing, already configured
		}

		else
		{
			Invalid_Configuration();
		}
	}

	else if (jackNumber == 12)
	{

		if (jackMode == DIGITAL_INPUT) //set "I/O board 4" to input (both jumpers), set jumper O to 3, and jumper N to 1
		{
			  GPIO_InitStruct.Pin = GPIO_PIN_12;
			  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
			  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
		}

		else if (jackMode == DIGITAL_OUTPUT) //set "I/O board 4" to output (both jumpers), set jumper O to 3, and jumper N to 1
		{
			  GPIO_InitStruct.Pin = GPIO_PIN_12;
			  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			  GPIO_InitStruct.Pull = GPIO_NOPULL;
			  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
		}

		else if (jackMode == ANALOG_INPUT) //set "I/O board 4" to input (both jumpers), set jumper O to 3, and jumper N to 1
		{
			//do nothing, already configured
		}

		else
		{
			Invalid_Configuration();
		}
	}
}

void Invalid_Configuration(void)
{
	//just an error function so that you can put a breakpoint here while debugging to check for invalid jack configurations
	;
}
