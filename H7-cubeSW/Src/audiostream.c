/* Includes ------------------------------------------------------------------*/
#include "audiostream.h"
#include "main.h"
#include "leaf.h"
#include "codec.h"
#include "tim.h"
#include "ui.h"

//the audio buffers are put in the D2 RAM area because that is a memory location that the DMA has access to.
int32_t audioOutBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;
int32_t audioInBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;

void audioFrame(uint16_t buffer_offset);
float audioTickL(float audioIn); 
float audioTickR(float audioIn);
void buttonCheck(void);

HAL_StatusTypeDef transmit_status;
HAL_StatusTypeDef receive_status;

uint16_t* adcVals;



#define NUM_BUTTONS 4
uint8_t buttonValues[NUM_BUTTONS];
uint8_t buttonValuesPrev[NUM_BUTTONS];
uint32_t buttonCounters[NUM_BUTTONS];
uint32_t buttonPressed[NUM_BUTTONS];


float sample = 0.0f;

uint16_t frameCounter = 0;

//audio objects
tRamp adc[12];
tCycle mySine[2];
t808Snare mySnare;
t808Kick myKick;
tNoise myNoise;
/**********************************************/

typedef enum BOOL {
	FALSE = 0,
	TRUE
} BOOL;


void audioInit(I2C_HandleTypeDef* hi2c, SAI_HandleTypeDef* hsaiOut, SAI_HandleTypeDef* hsaiIn, uint16_t* adc_array)
{
	// Initialize LEAF.

	LEAF_init(SAMPLE_RATE, AUDIO_FRAME_SIZE, &randomNumber);

	adcVals = adc_array; //in audiostream.h, the array of adc values is now called adcVals.

	// adcVals array has the data organized as follows:
	// [0] = knob 1
	// [1] = knob 2
	// [2] = knob 3
	// [3] = knob 4
	// [4] = knob 5 (depending on jumper K)
	// [5] = knob 6 (depending on jumper L)
	// [6] = knob 7 or jack 11 (depending on jumper M)
	// [7] = knob 8 or jack 12 (depending on jumper N and jumper O)
	// [8] = jack 1
	// [9] = jack 2
	// [10] = jack 3 (depending on jumper A and jumper B)
	// [11] = jack 4 (depending on jumper C and jumper D)

	// note that the knobs come in with the data backwards (fully clockwise is near zero, counter-clockwise is near 65535)
	// the CVs come in as expected (0V = 0, 10V = 65535)


	for (int i = 0; i < 12; i++)
	{
		tRamp_init(&adc[i],7.0f, 1); //set all ramps for knobs/jacks to be 7ms ramp time and let the init function know they will be ticked every sample
		//if you want to read different knobs/jacks at different rates or with different smoothing times, you can reinit after this
	}

	t808Snare_init(&mySnare);
	t808Kick_init(&myKick);
	tNoise_init(&myNoise, WhiteNoise);
	//now to send all the necessary messages to the codec
	AudioCodec_init(hi2c);

	HAL_Delay(100);

	for (int i = 0; i < AUDIO_BUFFER_SIZE; i++)
	{
		audioOutBuffer[i] = 0;
	}

	// set up the I2S driver to send audio data to the codec (and retrieve input as well)
	transmit_status = HAL_SAI_Transmit_DMA(hsaiOut, (uint8_t *)&audioOutBuffer[0], AUDIO_BUFFER_SIZE);
	receive_status = HAL_SAI_Receive_DMA(hsaiIn, (uint8_t *)&audioInBuffer[0], AUDIO_BUFFER_SIZE);
}

void audioFrame(uint16_t buffer_offset)
{
	int i;
	int32_t current_sample = 0;

	frameCounter++;
	if (frameCounter >= 1)
	{
		frameCounter = 0;
		buttonCheck();
	}

	for (i = 0; i < (HALF_BUFFER_SIZE); i++)
	{
		if ((i & 1) == 0)
		{
			current_sample = (int32_t)(audioTickL((float) (audioInBuffer[buffer_offset + i] * INV_TWO_TO_31)) * TWO_TO_31);
		}
		else
		{
			current_sample = (int32_t)(audioTickR((float) (audioInBuffer[buffer_offset + i] * INV_TWO_TO_31)) * TWO_TO_31);
		}

		audioOutBuffer[buffer_offset + i] = current_sample;
	}
}

uint8_t snareTriggered = 0;
uint8_t kickTriggered = 0;

float audioTickL(float audioIn)
{
	//read the analog inputs and smooth them with ramps
	tRamp_setDest(&adc[0], 1.0f - (adcVals[0] * INV_TWO_TO_16));
	tRamp_setDest(&adc[1], 1.0f - (adcVals[1] * INV_TWO_TO_16));
	tRamp_setDest(&adc[2], 1.0f - (adcVals[2] * INV_TWO_TO_16));
	tRamp_setDest(&adc[3], 1.0f - (adcVals[3] * INV_TWO_TO_16));
	tRamp_setDest(&adc[4], 1.0f - (adcVals[4] * INV_TWO_TO_16));
	tRamp_setDest(&adc[5], 1.0f - (adcVals[5] * INV_TWO_TO_16));
	tRamp_setDest(&adc[6], 1.0f - (adcVals[6] * INV_TWO_TO_16));
	tRamp_setDest(&adc[7], 1.0f - (adcVals[7] * INV_TWO_TO_16));
	tRamp_setDest(&adc[8], (adcVals[8] * INV_TWO_TO_16));
	tRamp_setDest(&adc[9], (adcVals[9] * INV_TWO_TO_16));
	tRamp_setDest(&adc[10], (adcVals[10] * INV_TWO_TO_16));
	tRamp_setDest(&adc[11], (adcVals[11] * INV_TWO_TO_16));

	float drumGain = LEAF_clip(0.0f, tRamp_tick(&adc[0]) + tRamp_tick(&adc[8]), 2.0f);

	//if digital input on jack 5, then trigger snare
	if ((!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) == 1)
	{
		if (snareTriggered == 0)
		{
			t808Snare_on(&mySnare, drumGain);
			snareTriggered = 1;
		}
	}
	else
	{
		snareTriggered = 0;
	}

	//if digital input on jack 6, then trigger kick
	if ((!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) == 1)
	{
		if (kickTriggered == 0)
		{
			t808Kick_on(&myKick, drumGain);
			kickTriggered = 1;
		}
	}
	else
	{
		kickTriggered = 0;
	}

	//OK, now some audio stuff

	float newFreq = LEAF_clip   (0.0f, LEAF_midiToFrequency((tRamp_tick(&adc[2]) * 15.0f) + (tRamp_tick(&adc[9]) * 15.0f) + 30.0f), 24000.0f); //knob 3 sets snare freq (added with jack 2 CV input)
	t808Snare_setToneNoiseMix(&mySnare, tRamp_tick(&adc[1]));//knob 2 sets noise mix
	t808Snare_setTone1Decay(&mySnare, (tRamp_tick(&adc[3]) * 100.0f) + (tRamp_tick(&adc[10]) * 100.0f)); //knob 4 sets snare tone1 decay time (added with jack 3 CV input)
	t808Snare_setTone2Decay(&mySnare, (tRamp_tick(&adc[3]) * 150.0f) + (tRamp_tick(&adc[10]) * 150.0f)); //knob 4 sets snare tone2 decay time (added with jack 3 CV input)
	t808Snare_setNoiseDecay(&mySnare, (tRamp_tick(&adc[3]) * 100.0f) + (tRamp_tick(&adc[10]) * 100.0f)); //knob 4 sets snare noise decay time (added with jack 3 CV input)
	t808Snare_setTone1Freq(&mySnare, newFreq);
	t808Snare_setTone2Freq(&mySnare, newFreq * 1.5f);
	sample = t808Snare_tick(&mySnare);//
	LEAF_shaper(sample, 1.2f);
	return sample;
}

float audioTickR(float audioIn)
{
	t808Kick_setToneFreq(&myKick, LEAF_midiToFrequency((tRamp_tick(&adc[4]) * 30.0f) + 14.0f)); //knob 5 sets kick freq
	t808Kick_setToneDecay(&myKick, (tRamp_tick(&adc[5]) * 1000.0f) + (tRamp_tick(&adc[11]) * 1000.0f)); //knob 6 set kick decay (added to jack 4 CV in)
	sample = t808Kick_tick(&myKick);
	LEAF_shaper(sample, 1.6f);
	return sample;
}


void buttonCheck(void)
{
	buttonValues[0] = !HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13);
	buttonValues[1] = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
	buttonValues[2] = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
	buttonValues[3] = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);

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
		t808Snare_on(&mySnare, 1.0f);

		RGB_mode++;
		if (RGB_mode > 3)
		{
			RGB_mode = 0;
		}
		if (RGB_mode == 0)
		{
			RGB_LED_setColor(255, 0, 0);
		}

		else if (RGB_mode == 1)
		{
			RGB_LED_setColor(0, 255, 0);
		}

		else if (RGB_mode == 2)
		{
			RGB_LED_setColor(0, 0, 255);
		}

		else if (RGB_mode == 3)
		{

		}
		buttonPressed[0] = 0;
	}
	if (buttonPressed[1] == 1)
	{

		if (mode1 == 0)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
			mode1 = 1;
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
			mode1 = 0;
		}

		buttonPressed[1] = 0;
	}
	if (buttonPressed[2] == 1)
	{
		if (mode2 == 0)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			mode2 = 1;
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			mode2 = 0;
		}
		buttonPressed[2] = 0;
	}

	if (buttonPressed[3] == 1)
	{

		if (mode3 == 0)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
			mode3 = 1;
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
			mode3 = 0;
		}
		buttonPressed[3] = 0;
	}
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
	;
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
	audioFrame(HALF_BUFFER_SIZE);
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	audioFrame(0);
}
