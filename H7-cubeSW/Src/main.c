
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_hal.h"
#include "adc.h"
#include "bdma.h"
#include "dma.h"
#include "i2c.h"
#include "quadspi.h"
#include "rng.h"
#include "sai.h"
#include "spi.h"
#include "tim.h"
#include "usb_host.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "audiostream.h"
#include "ssd1306.h"
#include "oled.h"


// FOR BEST SPEED
// set -O2 optimization flag for GCC and add these flags -finline-functions -funswitch-loops -fpredictive-commoning -fgcse-after-reload -ftree-loop-vectorize -ftree-loop-distribution -floop-interchange -floop-unroll-and-jam -fsplit-paths -fvect-cost-model -ftree-partial-pre -fpeel-loops -ffast-math -fsingle-precision-constant

// the -ftree-slp-vectorize breaks something - MIDI, likely
// -O3 with -fno-tree-slp-vectorize should work but for some reason it isn't as good. Maybe the specific gcc version for ARM omits some of those optimization flags from -O3?

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
const unsigned char myFont[1024] =
{
	0x00, 0x3E, 0x7F, 0x41, 0x4D, 0x4F, 0x2E, 0x00,	// Char 000 (.)
	0x00, 0x20, 0x74, 0x54, 0x54, 0x7C, 0x78, 0x00,	// Char 001 (.)
	0x00, 0x7E, 0x7E, 0x48, 0x48, 0x78, 0x30, 0x00,	// Char 002 (.)
	0x00, 0x38, 0x7C, 0x44, 0x44, 0x44, 0x00, 0x00,	// Char 003 (.)
	0x00, 0x30, 0x78, 0x48, 0x48, 0x7E, 0x7E, 0x00,	// Char 004 (.)
	0x00, 0x38, 0x7C, 0x54, 0x54, 0x5C, 0x18, 0x00,	// Char 005 (.)
	0x00, 0x00, 0x08, 0x7C, 0x7E, 0x0A, 0x0A, 0x00,	// Char 006 (.)
	0x00, 0x98, 0xBC, 0xA4, 0xA4, 0xFC, 0x7C, 0x00,	// Char 007 (.)
	0x00, 0x7E, 0x7E, 0x08, 0x08, 0x78, 0x70, 0x00,	// Char 008 (.)
	0x00, 0x00, 0x48, 0x7A, 0x7A, 0x40, 0x00, 0x00,	// Char 009 (.)
	0x00, 0x00, 0x80, 0x80, 0x80, 0xFA, 0x7A, 0x00,	// Char 010 (.)
	0x00, 0x7E, 0x7E, 0x10, 0x38, 0x68, 0x40, 0x00,	// Char 011 (.)
	0x00, 0x00, 0x42, 0x7E, 0x7E, 0x40, 0x00, 0x00,	// Char 012 (.)
	0x00, 0x7C, 0x7C, 0x18, 0x38, 0x1C, 0x7C, 0x78,	// Char 013 (.)
	0x00, 0x7C, 0x7C, 0x04, 0x04, 0x7C, 0x78, 0x00,	// Char 014 (.)
	0x00, 0x38, 0x7C, 0x44, 0x44, 0x7C, 0x38, 0x00,	// Char 015 (.)
	0x00, 0xFC, 0xFC, 0x24, 0x24, 0x3C, 0x18, 0x00,	// Char 016 (.)
	0x00, 0x18, 0x3C, 0x24, 0x24, 0xFC, 0xFC, 0x00,	// Char 017 (.)
	0x00, 0x7C, 0x7C, 0x04, 0x04, 0x0C, 0x08, 0x00,	// Char 018 (.)
	0x00, 0x48, 0x5C, 0x54, 0x54, 0x74, 0x24, 0x00,	// Char 019 (.)
	0x00, 0x04, 0x04, 0x3E, 0x7E, 0x44, 0x44, 0x00,	// Char 020 (.)
	0x00, 0x3C, 0x7C, 0x40, 0x40, 0x7C, 0x7C, 0x00,	// Char 021 (.)
	0x00, 0x1C, 0x3C, 0x60, 0x60, 0x3C, 0x1C, 0x00,	// Char 022 (.)
	0x00, 0x1C, 0x7C, 0x70, 0x38, 0x70, 0x7C, 0x1C,	// Char 023 (.)
	0x00, 0x44, 0x6C, 0x38, 0x38, 0x6C, 0x44, 0x00,	// Char 024 (.)
	0x00, 0x9C, 0xBC, 0xA0, 0xE0, 0x7C, 0x3C, 0x00,	// Char 025 (.)
	0x00, 0x44, 0x64, 0x74, 0x5C, 0x4C, 0x44, 0x00,	// Char 026 (.)
	0x00, 0x00, 0x7F, 0x7F, 0x41, 0x41, 0x00, 0x00,	// Char 027 (.)
	0x40, 0x68, 0x7C, 0x5E, 0x49, 0x49, 0x22, 0x00,	// Char 028 (.)
	0x00, 0x00, 0x41, 0x41, 0x7F, 0x7F, 0x00, 0x00,	// Char 029 (.)
	0x00, 0x08, 0x0C, 0xFE, 0xFE, 0x0C, 0x08, 0x00,	// Char 030 (.)
	0x00, 0x18, 0x3C, 0x7E, 0x18, 0x18, 0x18, 0x18,	// Char 031 (.)
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// Char 032 ( )
	0x00, 0x00, 0x00, 0x4F, 0x4F, 0x00, 0x00, 0x00,	// Char 033 (!)
	0x00, 0x07, 0x07, 0x00, 0x00, 0x07, 0x07, 0x00,	// Char 034 (")
	0x14, 0x7F, 0x7F, 0x14, 0x14, 0x7F, 0x7F, 0x14,	// Char 035 (#)
	0x00, 0x24, 0x2E, 0x6B, 0x6B, 0x3A, 0x12, 0x00,	// Char 036 ($)
	0x00, 0x63, 0x33, 0x18, 0x0C, 0x66, 0x63, 0x00,	// Char 037 (%)
	0x00, 0x32, 0x7F, 0x4D, 0x4D, 0x77, 0x72, 0x50,	// Char 038 (&)
	0x00, 0x00, 0x00, 0x04, 0x06, 0x03, 0x01, 0x00,	// Char 039 (')
	0x00, 0x00, 0x1C, 0x3E, 0x63, 0x41, 0x00, 0x00,	// Char 040 (()
	0x00, 0x00, 0x41, 0x63, 0x3E, 0x1C, 0x00, 0x00,	// Char 041 ())
	0x08, 0x2A, 0x3E, 0x1C, 0x1C, 0x3E, 0x2A, 0x08,	// Char 042 (*)
	0x00, 0x08, 0x08, 0x3E, 0x3E, 0x08, 0x08, 0x00,	// Char 043 (+)
	0x00, 0x00, 0x80, 0xE0, 0x60, 0x00, 0x00, 0x00,	// Char 044 (,)
	0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00,	// Char 045 (-)
	0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00,	// Char 046 (.)
	0x00, 0x40, 0x60, 0x30, 0x18, 0x0C, 0x06, 0x02,	// Char 047 (/)
	0x00, 0x3E, 0x7F, 0x49, 0x45, 0x7F, 0x3E, 0x00,	// Char 048 (0)
	0x00, 0x40, 0x44, 0x7F, 0x7F, 0x40, 0x40, 0x00,	// Char 049 (1)
	0x00, 0x62, 0x73, 0x51, 0x49, 0x4F, 0x46, 0x00,	// Char 050 (2)
	0x00, 0x22, 0x63, 0x49, 0x49, 0x7F, 0x36, 0x00,	// Char 051 (3)
	0x00, 0x18, 0x18, 0x14, 0x16, 0x7F, 0x7F, 0x10,	// Char 052 (4)
	0x00, 0x27, 0x67, 0x45, 0x45, 0x7D, 0x39, 0x00,	// Char 053 (5)
	0x00, 0x3E, 0x7F, 0x49, 0x49, 0x7B, 0x32, 0x00,	// Char 054 (6)
	0x00, 0x03, 0x03, 0x79, 0x7D, 0x07, 0x03, 0x00,	// Char 055 (7)
	0x00, 0x36, 0x7F, 0x49, 0x49, 0x7F, 0x36, 0x00,	// Char 056 (8)
	0x00, 0x26, 0x6F, 0x49, 0x49, 0x7F, 0x3E, 0x00,	// Char 057 (9)
	0x00, 0x00, 0x00, 0x24, 0x24, 0x00, 0x00, 0x00,	// Char 058 (:)
	0x00, 0x00, 0x80, 0xE4, 0x64, 0x00, 0x00, 0x00,	// Char 059 (;)
	0x00, 0x08, 0x1C, 0x36, 0x63, 0x41, 0x41, 0x00,	// Char 060 (<)
	0x00, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x00,	// Char 061 (=)
	0x00, 0x41, 0x41, 0x63, 0x36, 0x1C, 0x08, 0x00,	// Char 062 (>)
	0x00, 0x02, 0x03, 0x51, 0x59, 0x0F, 0x06, 0x00,	// Char 063 (?)
	0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,	// Char 064 (@)
	0x00, 0x7C, 0x7E, 0x0B, 0x0B, 0x7E, 0x7C, 0x00,	// Char 065 (A)
	0x00, 0x7F, 0x7F, 0x49, 0x49, 0x7F, 0x36, 0x00,	// Char 066 (B)
	0x00, 0x3E, 0x7F, 0x41, 0x41, 0x63, 0x22, 0x00,	// Char 067 (C)
	0x00, 0x7F, 0x7F, 0x41, 0x63, 0x3E, 0x1C, 0x00,	// Char 068 (D)
	0x00, 0x7F, 0x7F, 0x49, 0x49, 0x41, 0x41, 0x00,	// Char 069 (E)
	0x00, 0x7F, 0x7F, 0x09, 0x09, 0x01, 0x01, 0x00,	// Char 070 (F)
	0x00, 0x3E, 0x7F, 0x41, 0x49, 0x7B, 0x3A, 0x00,	// Char 071 (G)
	0x00, 0x7F, 0x7F, 0x08, 0x08, 0x7F, 0x7F, 0x00,	// Char 072 (H)
	0x00, 0x00, 0x41, 0x7F, 0x7F, 0x41, 0x00, 0x00,	// Char 073 (I)
	0x00, 0x20, 0x60, 0x41, 0x7F, 0x3F, 0x01, 0x00,	// Char 074 (J)
	0x00, 0x7F, 0x7F, 0x1C, 0x36, 0x63, 0x41, 0x00,	// Char 075 (K)
	0x00, 0x7F, 0x7F, 0x40, 0x40, 0x40, 0x40, 0x00,	// Char 076 (L)
	0x00, 0x7F, 0x7F, 0x06, 0x0C, 0x06, 0x7F, 0x7F,	// Char 077 (M)
	0x00, 0x7F, 0x7F, 0x0E, 0x1C, 0x7F, 0x7F, 0x00,	// Char 078 (N)
	0x00, 0x3E, 0x7F, 0x41, 0x41, 0x7F, 0x3E, 0x00,	// Char 079 (O)
	0x00, 0x7F, 0x7F, 0x09, 0x09, 0x0F, 0x06, 0x00,	// Char 080 (P)
	0x00, 0x1E, 0x3F, 0x21, 0x61, 0x7F, 0x5E, 0x00,	// Char 081 (Q)
	0x00, 0x7F, 0x7F, 0x19, 0x39, 0x6F, 0x46, 0x00,	// Char 082 (R)
	0x00, 0x26, 0x6F, 0x49, 0x49, 0x7B, 0x32, 0x00,	// Char 083 (S)
	0x00, 0x01, 0x01, 0x7F, 0x7F, 0x01, 0x01, 0x00,	// Char 084 (T)
	0x00, 0x3F, 0x7F, 0x40, 0x40, 0x7F, 0x3F, 0x00,	// Char 085 (U)
	0x00, 0x1F, 0x3F, 0x60, 0x60, 0x3F, 0x1F, 0x00,	// Char 086 (V)
	0x00, 0x7F, 0x7F, 0x30, 0x18, 0x30, 0x7F, 0x7F,	// Char 087 (W)
	0x00, 0x63, 0x77, 0x1C, 0x1C, 0x77, 0x63, 0x00,	// Char 088 (X)
	0x00, 0x07, 0x0F, 0x78, 0x78, 0x0F, 0x07, 0x00,	// Char 089 (Y)
	0x00, 0x61, 0x71, 0x59, 0x4D, 0x47, 0x43, 0x00,	// Char 090 (Z)
	0x18, 0x18, 0x18, 0xFF, 0xFF, 0x18, 0x18, 0x18,	// Char 091 ([)
	0x33, 0x33, 0xCC, 0xCC, 0x00, 0x00, 0x00, 0x00,	// Char 092 (\)
	0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00,	// Char 093 (])
	0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x00,	// Char 094 (^)
	0x00, 0x00, 0x00, 0xF0, 0xF0, 0x00, 0x00, 0x00,	// Char 095 (_)
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,	// Char 096 (`)
	0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,	// Char 097 (a)
	0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,	// Char 098 (b)
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,	// Char 099 (c)
	0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,	// Char 100 (d)
	0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// Char 101 (e)
	0x33, 0x33, 0xCC, 0xCC, 0x33, 0x33, 0xCC, 0xCC,	// Char 102 (f)
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF,	// Char 103 (g)
	0x30, 0x30, 0xC0, 0xC0, 0x30, 0x30, 0xC0, 0xC0,	// Char 104 (h)
	0x33, 0x99, 0xCC, 0x66, 0x33, 0x99, 0xCC, 0x66,	// Char 105 (i)
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF,	// Char 106 (j)
	0x00, 0x00, 0x00, 0xFF, 0xFF, 0x18, 0x18, 0x18,	// Char 107 (k)
	0x00, 0x00, 0x00, 0x00, 0xF0, 0xF0, 0xF0, 0xF0,	// Char 108 (l)
	0x00, 0x00, 0x00, 0x1F, 0x1F, 0x18, 0x18, 0x18,	// Char 109 (m)
	0x18, 0x18, 0x18, 0xF8, 0xF8, 0x00, 0x00, 0x00,	// Char 110 (n)
	0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,	// Char 111 (o)
	0x00, 0x00, 0x00, 0xF8, 0xF8, 0x18, 0x18, 0x18,	// Char 112 (p)
	0x18, 0x18, 0x18, 0x1F, 0x1F, 0x18, 0x18, 0x18,	// Char 113 (q)
	0x18, 0x18, 0x18, 0xF8, 0xF8, 0x18, 0x18, 0x18,	// Char 114 (r)
	0x18, 0x18, 0x18, 0xFF, 0xFF, 0x00, 0x00, 0x00,	// Char 115 (s)
	0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	// Char 116 (t)
	0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,	// Char 117 (u)
	0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF,	// Char 118 (v)
	0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,	// Char 119 (w)
	0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,	// Char 120 (x)
	0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0,	// Char 121 (y)
	0x00, 0x78, 0x78, 0x30, 0x18, 0x0C, 0x06, 0x03,	// Char 122 (z)
	0xF0, 0xF0, 0xF0, 0xF0, 0x00, 0x00, 0x00, 0x00,	// Char 123 ({)
	0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x0F, 0x0F,	// Char 124 (|)
	0x18, 0x18, 0x18, 0x1F, 0x1F, 0x00, 0x00, 0x00,	// Char 125 (})
	0x0F, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00,	// Char 126 (~)
	0x0F, 0x0F, 0x0F, 0x0F, 0xF0, 0xF0, 0xF0, 0xF0
};

//register unsigned int apsr __asm("cpacr");

//FPU->CPACR |= (1<<24);

//#define SAMPLERATE96K

#define NUM_ADC_CHANNELS 5
uint16_t myADC[NUM_ADC_CHANNELS] __ATTR_RAM_D2;

uint32_t counter = 0;
int pinValue = 0;

uint8_t ball[]  = {0x00, 0x3C, 0x7E, 0x7E, 0x7E, 0x7E, 0x3C, 0x00};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void MPU_Conf(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  MPU_Conf();

  SystemInit();
  /* USER CODE END 1 */

  /* Enable I-Cache-------------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache-------------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_BDMA_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_RNG_Init();
  MX_SAI1_Init();
  MX_SPI4_Init();
  MX_I2C4_Init();
  MX_USB_HOST_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);  //LED1
  //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET); //LED2


  ssd1306_begin(&hi2c4, SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS);
  HAL_Delay(5);
  //ssd1306_home();
  ssd1306_move(0,0);
  for (int i = 0; i < 512; i++)
  {
	  buffer[i] = 0;
  }
  oled_putxy(0,0,&ball);
  HAL_Delay(500);
  //ssd1306_display_full_buffer();
  //ssd1306_command(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
  //oled_putc('H');
  ssd1306_display_full_buffer();
  //ssd1306_display_full_buffer();
  //ssd1306_display_full_buffer();
  HAL_Delay(500);
  //sdd1306_invertDisplay(1);



  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);    //LED3
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);  //LED4

  // this code sets the processor to treat denormal numbers (very tiny floats) as zero to improve performance.
  uint32_t tempFPURegisterVal = __get_FPSCR();
  tempFPURegisterVal = (1<<24); // set the FTZ (flush-to-zero) bit in the FPU control register
  __set_FPSCR(tempFPURegisterVal);

	if (HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&myADC, NUM_ADC_CHANNELS) != HAL_OK)
	{
		Error_Handler();

	}
	audioInit(&hi2c2, &hsai_BlockA1, &hsai_BlockB1, &hrng, ((uint16_t*)&myADC));
	
	//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0); //led4
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, counter); //led1
	  //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, counter); //led4
	  //__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, counter); //led3
	  counter++;
	  if (counter > 127)
	  {
		  counter = 0;
	  }
	  //HAL_Delay(1);

	  //buffer[counter] = (uint8_t)(randomNumber() * 255.0f);

	  oled_putxy(counter,0,&ball);
	  ssd1306_display_full_buffer();
/*
	  if (counter > 1000)
		{
			if (pinValue == 0)
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
				pinValue = 1;
			}
			else
			{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
				pinValue = 0;
			}
			counter = 0;
		}
		counter++;
		*/

  /* USER CODE END WHILE */
    //MX_USB_HOST_Process();

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Supply configuration update enable 
    */
  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY) 
  {
    
  }
    /**Macro to configure the PLL clock source 
    */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RNG|RCC_PERIPHCLK_SPI4
                              |RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_I2C4
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_QSPI
                              |RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.PLL2.PLL2M = 25;
  PeriphClkInitStruct.PLL2.PLL2N = 344;
  PeriphClkInitStruct.PLL2.PLL2P = 7;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.PLL3.PLL3M = 25;
  PeriphClkInitStruct.PLL3.PLL3N = 192;
  PeriphClkInitStruct.PLL3.PLL3P = 4;
  PeriphClkInitStruct.PLL3.PLL3Q = 4;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_0;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_D1HCLK;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
  PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_D3PCLK1;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_CLKP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(SystemCoreClock/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

// Returns random floating point value [0.0,1.0)
float randomNumber(void) {
	
	uint32_t rand;
	HAL_RNG_GenerateRandomNumber(&hrng, &rand);
	float num = (((float)(rand >> 16))- 32768.f) * INV_TWO_TO_15;
	return num;
}



void MPU_Conf(void)
{
	//code from Keshikan https://github.com/keshikan/STM32H7_DMA_sample
  //Thanks, Keshikan! This solves the issues with accessing the SRAM in the D2 area properly. -JS
	
	MPU_Region_InitTypeDef MPU_InitStruct;

	  HAL_MPU_Disable();

	  MPU_InitStruct.Enable = MPU_REGION_ENABLE;

	  //D2 Domain�SRAM1
	  MPU_InitStruct.BaseAddress = 0x30000000;
	  // Increased region size to 256k. In Keshikan's code, this was 512 bytes (that's all that application needed).
	  // Each audio buffer takes up the frame size * 8 (16 bits makes it *2 and stereo makes it *2 and double buffering makes it *2)
	  // So a buffer size for read/write of 4096 would take up 64k = 4096*8 * 2 (read and write).
	  // I increased that to 256k so that there would be room for the ADC knob inputs and other peripherals that might require DMA access (for instance the OLED screen).
	  // we have a total of 256k in SRAM1 (128k, 0x30000000-0x30020000) and SRAM2 (128k, 0x30020000-0x3004000) of D2 domain. 
		// There is an SRAM3 in D2 domain as well (32k, 0x30040000-0x3004800) that is currently not mapped by the MPU (memory protection unit) controller. 
	  
	  MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;

	  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;

	  //AN4838
	  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
	  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;

	  //Shared Device
//	  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
//	  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
//	  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
//	  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;


	  MPU_InitStruct.Number = MPU_REGION_NUMBER0;

	  MPU_InitStruct.SubRegionDisable = 0x00;


	  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;


	  HAL_MPU_ConfigRegion(&MPU_InitStruct);




	  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {

  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
