/* Includes ------------------------------------------------------------------*/
#include "audiostream.h"
#include "main.h"
#include "codec.h"
#include "tim.h"


int16_t audioOutBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;
int16_t audioInBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;

float detuneAmounts[NUM_OSC];

int16_t inBuffer[HALF_BUFFER_SIZE];
int16_t outBuffer[HALF_BUFFER_SIZE];

uint16_t* adcVals;

uint8_t buttonAPressed = 0;

uint8_t doAudio = 0;

float sample = 0.0f;

float adcx[8];

float detuneMax = 0.2f;
uint8_t audioInCV = 0;
uint8_t audioInCVAlt = 0;
float myVol = 0.0f;
float currentFreq = 1.0f;

float rightInput = 0.0f;
uint16_t string1Position;
uint8_t string1Touch;
uint8_t string1RHTouch;
uint16_t string1TouchRaw;
float string1MappedPosition = 0.0f;
float myAmplitude = 0.0f;
float openStringFrequencies[4] = {80.0f, 60.0f, 80.0f, 100.0f};
float string1Frequency = 40.0f;

float audioTickL(float audioIn); 
float audioTickR(float audioIn);
void buttonCheck(void);





HAL_StatusTypeDef transmit_status;
HAL_StatusTypeDef receive_status;

typedef enum BOOL {
	FALSE = 0,
	TRUE
} BOOL;

void audioInit(I2C_HandleTypeDef* hi2c, SAI_HandleTypeDef* hsaiOut, SAI_HandleTypeDef* hsaiIn, RNG_HandleTypeDef* hrand, uint16_t* myADCArray)
{ 
	// Initialize the audio library. OOPS.
	OOPSInit(SAMPLE_RATE, &randomNumber);
	
	//now to send all the necessary messages to the codec
	AudioCodec_init(hi2c);

	HAL_Delay(100);

	adcVals = myADCArray;
	for (int i = 0; i < NUM_OSC; i++)
	{
		osc[i] = tSawtoothInit();
		tSawtoothSetFreq(osc[i], (randomNumber() * 2000.0f) + 40.0f);
		detuneAmounts[i] = (randomNumber() * detuneMax) - (detuneMax * 0.5f);
	}

	filter = tSVFInit(SVFTypeLowpass, 900.0f, 2.0f);
	//vocoder = tTalkboxInit();
	envFollow = tEnvelopeFollowerInit(0.2f, 0.99998f);
	dcBlock = tHighpassInit(800.0f);
	myRamp = tRampInit(10.0f, 1);
	freqRamp = tRampInit(20.0f, 1);
	// set up the I2S driver to send audio data to the codec (and retrieve input as well)	
	transmit_status = HAL_SAI_Transmit_DMA(hsaiOut, (uint8_t *)&audioOutBuffer[0], AUDIO_BUFFER_SIZE);
	receive_status = HAL_SAI_Receive_DMA(hsaiIn, (uint8_t *)&audioInBuffer[0], AUDIO_BUFFER_SIZE);
	

}

float tempVal = 0.0f;
uint16_t frameCounter = 0;
float inputSum = 0.0f;

void audioFrame(uint16_t buffer_offset)
{
	uint16_t i = 0;
	int16_t current_sample = 0;

	inputSum = 0.0f;
/*
	frameCounter++;
	if (frameCounter >= 1)
	{
		frameCounter = 0;
		buttonCheck();
	}
	*/
/*
	for (int i = 0; i < NUMPARAMS; i++)
	{
		//dropping the resolution of the knobs to allow for stable positions (by making the ADC only 8 bit)
		adcx[i] = adcVals[i] / 256 * INV_TWO_TO_8;
	}
	*/

	for (i = 0; i < (HALF_BUFFER_SIZE); i++)
	{
		if ((i & 1) == 0) {
			current_sample = (int16_t)(audioTickL((float) (audioInBuffer[buffer_offset + i] * INV_TWO_TO_15)) * TWO_TO_15);
			//outBuffer[i] = current_sample;
		}
		else
		{
			//current_sample = (int16_t)(audioTickR((float) (audioInBuffer[buffer_offset + i] * INV_TWO_TO_15)) * TWO_TO_15);
		}
		audioOutBuffer[buffer_offset + i] = current_sample;
	}
	/*
	myAmplitude = (inputSum / AUDIO_FRAME_SIZE);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, myAmplitude * 6000.0f ); //led3
	if (myAmplitude > 0.5f)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, myAmplitude * 6000.0f ); //led4
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0 ); //led4
	}
	*/
}

float audioInGain = 20.0f;

float audioTickL(float audioIn) 
{
	sample = 0.0f;

	float endFreq = 0.0f;

	if (string1Position < 20000)
	{
		endFreq = string1Frequency;
	}
	else
	{
		endFreq = 40.0f;
	}

	tRampSetDest(freqRamp, endFreq);
	float endFreqMIDI = OOPS_frequencyToMidi(tRampTick(freqRamp));
	if ((string1Touch == 1) && (string1Position > 20000))
	{
		audioIn = 0.0f;
		tRampSetDest(myRamp, 0.0f);
	}
	else if (string1RHTouch == 1)
	{
		audioIn = 0.0f;
		tRampSetDest(myRamp, 0.0f);
	}
	else
	{
		tRampSetDest(myRamp, 1.0f);
	}
	float envelope = tEnvelopeFollowerTick(envFollow, tanhf(tHighpassTick(dcBlock, audioIn) * audioInGain));
	tSVFSetFreq(filter, (envelope * envelope * envelope * envelope * 500.0f) + 30.0f);
	for (uint8_t i = 0;  i < NUM_OSC; i++)
	{
		tSawtoothSetFreq(osc[i], OOPS_midiToFrequency(endFreqMIDI + detuneAmounts[i]));
		sample += OOPS_shaper(tSVFTick(filter, tSawtoothTick(osc[i]) * .07f * envelope * tRampTick(myRamp)), 1.1f);
	}

	return sample;
}

float audioTickR(float audioIn) 
{
	rightInput = audioIn;
	//sample = audioIn;
	return audioIn;
}

void buttonCheck(void)
{
	buttonValues[0] = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
	buttonValues[1] = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);
	buttonValues[2] = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	buttonValues[3] = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);

	for (int i = 0; i < 4; i++)
	{
	  if ((buttonValues[i] != buttonValuesPrev[i]) && (buttonCounters[i] < 40))
	  {
		  buttonCounters[i]++;
	  }
	  if ((buttonValues[i] != buttonValuesPrev[i]) && (buttonCounters[i] >= 40))
	  {
		  if (buttonValues[i] == 1)
		  {
			  buttonPressed[i] = 1;
		  }
		  buttonValuesPrev[i] = buttonValues[i];
		  buttonCounters[i] = 0;
	  }
	}

	if (buttonPressed[0] == 1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		buttonPressed[0] = 0;

	}
	if (buttonPressed[1] == 1)
	{
		buttonPressed[1] = 0;
	}
	if (buttonPressed[2] == 1)
	{
		buttonPressed[2] = 0;
	}

	if (buttonPressed[3] == 1)
	{
		buttonPressed[3] = 0;
	}
}


void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
	;
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  ;
}


void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
	//doAudio = 2;
	audioFrame(HALF_BUFFER_SIZE);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);;
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	//doAudio = 1;
	audioFrame(0);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);;
}

