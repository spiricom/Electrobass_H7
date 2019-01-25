/* Includes ------------------------------------------------------------------*/
#include "audiostream.h"
#include "main.h"
#include "leaf.h"
#include "codec.h"
#include "tim.h"
#include "ui.h"

#define HIGHBOARD

//the audio buffers are put in the D2 RAM area because that is a memory location that the DMA has access to.
int32_t audioOutBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;
int32_t audioInBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;
uint8_t spiTXBuffer[16] __ATTR_RAM_D2;
uint8_t spiRXBuffer[16] __ATTR_RAM_D2;

void audioFrame(uint16_t buffer_offset);
float audioTickL(float audioIn); 
float audioTickR(float audioIn);
float map(float value, float istart, float istop, float ostart, float ostop);


HAL_StatusTypeDef transmit_status;
HAL_StatusTypeDef receive_status;

uint16_t* adcVals;


float sample = 0.0f;

uint16_t frameCounter = 0;

float attack[2] = {0,0};
uint16_t attackDetected[2] = {0,0};

//audio objects
tRamp ampRamp[2];
tCycle mySine[2];
tSawtooth mySaw[2];
tEnvelope noiseEnv[2];
tEnvelope filterEnv[2];
tNoise myNoise[2];
tEnvelopeFollower follower[2];
tHighpass dcBlocker[2];
tSVF lowpass[2];
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

	for (int i = 0; i < 2; i++)
	{
		tRamp_init(&ampRamp[i],15.0f, 1);
		tCycle_init(&mySine[i]);
		tSawtooth_init(&mySaw[i]);
		tNoise_init(&myNoise[i], WhiteNoise);
		tEnvelopeFollower_init(&follower[i], .01f, .99995f);
		tHighpass_init(&dcBlocker[i], 80.0f);
		tEnvelope_init(&noiseEnv[i], 7.0f, 100.0f, 0);
		tEnvelope_init(&filterEnv[i], 7.0f, 1000.0f, 0);
		tSVF_init(&lowpass[i], SVFTypeLowpass, 100.0f, 0.7f);
	}
	/*
	tCycle_init(&mySine[0]);
	tCycle_init(&mySine[1]);
	tSawtooth_init(&mySaw[0]);
	tSawtooth_init(&mySaw[1]);
	tNoise_init(&myNoise[0], WhiteNoise);
	tNoise_init(&myNoise[1], WhiteNoise);
	tEnvelopeFollower_init(&follower[0], .01f, .99995f);
	tEnvelopeFollower_init(&follower[1], .01f, .99995f);
	tHighpass_init(&dcBlocker[0], 80.0f);
	tHighpass_init(&dcBlocker[1], 80.0f);
	*/
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

uint16_t stringPositions[2];
float stringMappedPositions[2];
float stringFrequencies[2];
uint8_t stringTouchLH[2] = {0,0};
uint8_t stringTouchRH[2] = {0,0};
float openStringFrequencies[4] = {41.204f, 55.0f, 73.416f, 97.999f};


void audioFrame(uint16_t buffer_offset)
{
	int i;
	int32_t current_sample = 0;

#ifdef LOWBOARD
	stringPositions[0] =  ((uint16_t)spiRXBuffer[2] << 8) + ((uint16_t)spiRXBuffer[3] & 0xff);
	if (stringPositions[0] == 65535)
	{
		stringFrequencies[0] = openStringFrequencies[0];
	}
	else
	{
		stringMappedPositions[0] = map((float)stringPositions[0], 4505.0f, 9126.0f, 0.5f, 0.66666666666f);
		stringFrequencies[0] = (1.0 / (2.0f * stringMappedPositions[0])) * openStringFrequencies[0] * 2.0f;
	}
	stringTouchLH[0] = (spiRXBuffer[8] >> 0) & 1;
	stringTouchRH[0] = (spiRXBuffer[8] >> 4) & 1;


	stringPositions[1] =  ((uint16_t)spiRXBuffer[6] << 8) + ((uint16_t)spiRXBuffer[7] & 0xff);

	if (stringPositions[1] == 65535)
	{
		stringFrequencies[1] = openStringFrequencies[1];
	}
	else
	{
		stringMappedPositions[1] = map((float)stringPositions[1], 4230.0f, 8704.0f, 0.5f, 0.66666666f);
		stringFrequencies[1] = (1.0 / (2.0f * stringMappedPositions[1])) * openStringFrequencies[1] * 2.0f;
	}
	stringTouchLH[1] = (spiRXBuffer[8] >> 1) & 1;
	stringTouchRH[1] = (spiRXBuffer[8] >> 5) & 1;
#endif

#ifdef HIGHBOARD
	stringPositions[0] =  ((uint16_t)spiRXBuffer[4] << 8) + ((uint16_t)spiRXBuffer[5] & 0xff);
	if (stringPositions[0] == 65535)
	{
		stringFrequencies[0] = openStringFrequencies[2];
	}
	else
	{
		stringMappedPositions[0] = map((float)stringPositions[0], 4462.0f, 9168.0f, 0.5f, 0.6666666666f);
		stringFrequencies[0] = (1.0 / (2.0f * stringMappedPositions[0])) * openStringFrequencies[2] * 2.0f;
	}
	stringTouchLH[0] = (spiRXBuffer[8] >> 2) & 1;
	stringTouchRH[0] = (spiRXBuffer[8] >> 6) & 1;


	stringPositions[1] =  ((uint16_t)spiRXBuffer[0] << 8) + ((uint16_t)spiRXBuffer[1] & 0xff);

	if (stringPositions[1] == 65535)
	{
		stringFrequencies[1] = openStringFrequencies[3];
	}
	else
	{
		stringMappedPositions[1] = map((float)stringPositions[1], 4294.0f, 8853.0f, 0.5f, 0.6666666666666f);
		stringFrequencies[1] = (1.0 / (2.0f * stringMappedPositions[1])) * openStringFrequencies[3] * 2.0f;
	}
	stringTouchLH[1] = (spiRXBuffer[8] >> 3) & 1;
	stringTouchRH[1] = (spiRXBuffer[8] >> 7) & 1;
#endif
	/*
	frameCounter++;
	if (frameCounter >= 1)
	{
		frameCounter = 0;
	}
	*/
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

float rightIn = 0.0f;
float envelope = 0.0f;
float envelopeMax = 0.0f;

float audioTickL(float audioIn)
{
	//process both strings
	for (int i = 0; i < 2; i++)
	{

		if ((stringTouchLH[i] == 1) && (stringPositions[i] == 65535))
		{
			//left hand mute
			audioIn = 0.0f;
			follower[i].y = 0.0f;
			tRamp_setDest(&ampRamp[i], 0.0f);
		}
		else if (stringTouchRH[i] == 1)
		{
			//right hand mute
			audioIn = 0.0f;
			follower[i].y = 0.0f;
			tRamp_setDest(&ampRamp[i], 0.0f);
		}
		else
		{
			//not muted
			tRamp_setDest(&ampRamp[i], 1.0f);
		}
		audioIn = tHighpass_tick(&dcBlocker[i], audioIn);
		envelope = tEnvelopeFollower_tick(&follower[i], audioIn);
		tSawtooth_setFreq(&mySaw[i],stringFrequencies[i]);
		tCycle_setFreq(&mySine[i],stringFrequencies[i]);
		sample = tCycle_tick(&mySine[i]) * envelope * tRamp_tick(&ampRamp[i]);
		sample += tSawtooth_tick(&mySaw[i]) * envelope* tRamp_tick(&ampRamp[i]);
	}
/*
	if ((stringTouchLH[1] == 1) && (stringPositions[1] == 65535))
	{
		//left hand mute
		rightIn = 0.0f;
		follower[1].y = 0.0f;
		tRamp_setDest(&ampRamp[1], 0.0f);
	}
	else if (stringTouchRH[1] == 1)
	{
		//right hand mute
		rightIn = 0.0f;
		follower[1].y = 0.0f;
		tRamp_setDest(&ampRamp[1], 0.0f);
	}
	else
	{
		//not muted
		tRamp_setDest(&ampRamp[1], 1.0f);
	}

	rightIn = tHighpass_tick(&dcBlocker[1], rightIn);
	envelope = tEnvelopeFollower_tick(&follower[1], rightIn);
	tCycle_setFreq(&mySine[1],stringFrequencies[1]);
	tSawtooth_setFreq(&mySaw[1],stringFrequencies[1]);
	sample += tCycle_tick(&mySine[1]) * envelope* tRamp_tick(&ampRamp[1]);
	sample += tSawtooth_tick(&mySaw[1]) * envelope* tRamp_tick(&ampRamp[1]);
*/



	//sample = 0.0f;
	LEAF_shaper(sample, 1.6f);
	return sample;
}


float audioTickR(float audioIn)
{
	rightIn = audioIn;
	return sample;
}

float map(float value, float istart, float istop, float ostart, float ostop)
{
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
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
