/**
  ******************************************************************************
  * @file    Audio_playback_and_record/inc/waveplayer.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    26-June-2014
  * @brief   Header for waveplayer.c module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */   
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AUDIOSTREAM_H
#define __AUDIOSTREAM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

#include "OOPS.h"



#define NUM_BUTTONS 4
uint8_t buttonValues[NUM_BUTTONS];
uint8_t buttonValuesPrev[NUM_BUTTONS];
uint32_t buttonCounters[NUM_BUTTONS];
uint32_t buttonPressed[NUM_BUTTONS];

extern float testFreq;
extern uint8_t buttonAPressed;




/* Exported types ------------------------------------------------------------*/
typedef enum
{
  BUFFER_OFFSET_NONE = 0,  
  BUFFER_OFFSET_HALF,  
  BUFFER_OFFSET_FULL,     
}BUFFER_StateTypeDef;


void slideValueChanged(uint16_t value);

void knobValueChanged(uint16_t value);

void buttonOneDown(void);

void buttonOneUp(void);

void buttonTwoDown(void);

void buttonTwoUp(void);

void presetButtonDown(void);

void presetButtonUp(void);

void setFundamental(float fund);


/* Exported constants --------------------------------------------------------*/

extern float fundamental_hz;
extern float fundamental_cm;
extern float fundamental_m;
extern float inv_fundamental_m;
extern float cutoff_offset;
extern float intPeak;
extern float floatPeak;
extern float testDelay;
extern float slide_tune;

extern float valPerM;
extern float mPerVal;

#ifdef SAMPLERATE96K
#define SAMPLE_RATE 96000.f
#else
#define SAMPLE_RATE 48000.f
#endif

#define INV_SAMPLE_RATE 1.f/SAMPLE_RATE 
#define SAMPLE_RATE_MS (SAMPLE_RATE / 1000.f)
#define INV_SR_MS 1.f/SAMPLE_RATE_MS
#define SAMPLE_RATE_DIV_PARAMS SAMPLE_RATE / 3
#define SAMPLE_RATE_DIV_PARAMS_MS (SAMPLE_RATE_DIV_PARAMS / 1000.f)
#define INV_SR_DIV_PARAMS_MS 1.f/SAMPLE_RATE_DIV_PARAMS_MS

typedef enum LCDModeType
{
	LCDModeDisplayPitchClass = 0,
	LCDModeDisplayPitchMidi,
	LCDModeTypeNil,
	LCDModeCount = LCDModeTypeNil
} LCDModeType;

extern uint16_t knobValue;
extern int knobMoved;
extern int calibrated;
extern LCDModeType lcdMode;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void audioInit(I2C_HandleTypeDef* hi2c, SAI_HandleTypeDef* hsaiOut, SAI_HandleTypeDef* hsaiIn, RNG_HandleTypeDef* hrandom, uint16_t* myADCArray);

void DMA1_TransferCpltCallback(DMA_HandleTypeDef *hdma);
void DMA1_HalfTransferCpltCallback(DMA_HandleTypeDef *hdma);
#endif /* __WAVEPLAYER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
