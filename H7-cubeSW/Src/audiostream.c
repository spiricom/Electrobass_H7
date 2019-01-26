/* Includes ------------------------------------------------------------------*/
#include "audiostream.h"
#include "main.h"
#include "leaf.h"
#include "codec.h"
#include "tim.h"
#include "ui.h"

#define LOWBOARD
#define NUM_SAWS 3
#define ATTACK_DETECTOR_BETWEEN_ATTACK_WAIT 3000
#define ATTACK_DETECTOR_POST_PEAK 2
#define SHORT_DECAY .9996f
#define LONG_DECAY .99998f
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
float detuneAmounts[2][NUM_SAWS];
float maxDetune = .09f;
float rightIn = 0.0f;
float envelope[2];
float prevEnvelope[2];
uint32_t envelopeCounter[2];
uint32_t timeSinceLastAttack[2];
uint32_t envelopeCounterFall[2];
uint32_t envelopeState[2]; //0 is resting, 1 is rising, 2 is falling
float envelopeMargin = 0.0001f;
float envelopeMemory[2][ATTACK_DETECTOR_POST_PEAK];
uint32_t noteOnHappened[2];
uint32_t noteOffHappened[2];
float velocity[2];

uint16_t stringPositions[2];
float stringMappedPositions[2];
float stringFrequencies[2];
float stringMIDIVersionOfFrequencies[2];
uint8_t stringTouchLH[2] = {0,0};
uint8_t stringTouchRH[2] = {0,0};
float openStringFrequencies[4] = {41.204f, 55.0f, 73.416f, 97.999f};


//audio objects
tRamp ampRamp[2];
tCycle mySine[2];
tSawtooth mySaw[2][NUM_SAWS];
tEnvelope noiseEnv[2];
tEnvelope filterEnv[2];
tNoise myNoise[2];
tSVF noiseFilt[2];
tEnvelopeFollower follower[2];
tHighpass dcBlocker[2];
tSVF lowpass[2];
tRamp followerRamp[2];
tADSR mainEnv[2];
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
		tRamp_init(&ampRamp[i],100.0f, 1);
		tCycle_init(&mySine[i]);
		for (int j = 0; j < NUM_SAWS; j++)
		{
			detuneAmounts[i][j] = (randomNumber() * maxDetune) - (maxDetune * .5f);
			tSawtooth_init(&mySaw[i][j]);
		}
		tNoise_init(&myNoise[i], WhiteNoise);
		tEnvelopeFollower_init(&follower[i], .01f, LONG_DECAY);
		tHighpass_init(&dcBlocker[i], 500.0f);
		tEnvelope_init(&noiseEnv[i], 7.0f, 20.0f, 0);
		tEnvelope_init(&filterEnv[i], 7.0f, 150.0f, 0);
		tSVF_init(&lowpass[i], SVFTypeLowpass, 100.0f, 1.0f);
		tSVF_init(&noiseFilt[i], SVFTypeBandpass, 1500.0f, 0.5f);
		tRamp_init(&followerRamp[i], 10.0f, 1);
		tADSR_init(&mainEnv[i], 7.0f, 5000.0f, 1.0f, 150.0f);
	}

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

#ifdef LOWBOARD
	stringPositions[0] =  ((uint16_t)spiRXBuffer[2] << 8) + ((uint16_t)spiRXBuffer[3] & 0xff);
	if (stringPositions[0] == 65535)
	{
		stringFrequencies[0] = openStringFrequencies[0];
		stringMIDIVersionOfFrequencies[0] = LEAF_frequencyToMidi(stringFrequencies[0]);
	}
	else
	{
		stringMappedPositions[0] = map((float)stringPositions[0], 4505.0f, 9126.0f, 0.5f, 0.66666666666f);
		stringFrequencies[0] = (1.0 / (2.0f * stringMappedPositions[0])) * openStringFrequencies[0] * 2.0f;
		stringMIDIVersionOfFrequencies[0] = LEAF_frequencyToMidi(stringFrequencies[0]);
	}
	stringTouchLH[0] = (spiRXBuffer[8] >> 0) & 1;
	stringTouchRH[0] = (spiRXBuffer[8] >> 4) & 1;


	stringPositions[1] =  ((uint16_t)spiRXBuffer[6] << 8) + ((uint16_t)spiRXBuffer[7] & 0xff);

	if (stringPositions[1] == 65535)
	{
		stringFrequencies[1] = openStringFrequencies[1];
		stringMIDIVersionOfFrequencies[1] = LEAF_frequencyToMidi(stringFrequencies[1]);
	}
	else
	{
		stringMappedPositions[1] = map((float)stringPositions[1], 4230.0f, 8704.0f, 0.5f, 0.66666666f);
		stringFrequencies[1] = (1.0 / (2.0f * stringMappedPositions[1])) * openStringFrequencies[1] * 2.0f;
		stringMIDIVersionOfFrequencies[1] = LEAF_frequencyToMidi(stringFrequencies[1]);
	}
	stringTouchLH[1] = (spiRXBuffer[8] >> 1) & 1;
	stringTouchRH[1] = (spiRXBuffer[8] >> 5) & 1;
#endif

#ifdef HIGHBOARD
	stringPositions[0] =  ((uint16_t)spiRXBuffer[4] << 8) + ((uint16_t)spiRXBuffer[5] & 0xff);
	if (stringPositions[0] == 65535)
	{
		stringFrequencies[0] = openStringFrequencies[2];
		stringMIDIVersionOfFrequencies[0] = LEAF_frequencyToMidi(stringFrequencies[0]);
	}
	else
	{
		stringMappedPositions[0] = map((float)stringPositions[0], 4462.0f, 9168.0f, 0.5f, 0.6666666666f);
		stringFrequencies[0] = (1.0 / (2.0f * stringMappedPositions[0])) * openStringFrequencies[2] * 2.0f;
		stringMIDIVersionOfFrequencies[0] = LEAF_frequencyToMidi(stringFrequencies[0]);
	}
	stringTouchLH[0] = (spiRXBuffer[8] >> 2) & 1;
	stringTouchRH[0] = (spiRXBuffer[8] >> 6) & 1;


	stringPositions[1] =  ((uint16_t)spiRXBuffer[0] << 8) + ((uint16_t)spiRXBuffer[1] & 0xff);

	if (stringPositions[1] == 65535)
	{
		stringFrequencies[1] = openStringFrequencies[3];
		stringMIDIVersionOfFrequencies[1] = LEAF_frequencyToMidi(stringFrequencies[1]);
	}
	else
	{
		stringMappedPositions[1] = map((float)stringPositions[1], 4294.0f, 8853.0f, 0.5f, 0.6666666666666f);
		stringFrequencies[1] = (1.0 / (2.0f * stringMappedPositions[1])) * openStringFrequencies[3] * 2.0f;
		stringMIDIVersionOfFrequencies[1] = LEAF_frequencyToMidi(stringFrequencies[1]);
	}
	stringTouchLH[1] = (spiRXBuffer[8] >> 3) & 1;
	stringTouchRH[1] = (spiRXBuffer[8] >> 7) & 1;
#endif

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



float audioTickL(float audioIn)
{
	sample = 0.0f;
	//process both strings
	for (int i = 0; i < 2; i++)
	{
		if (i == 1)
		{
			audioIn = rightIn;
		}

		LEAF_softClip(audioIn * 5.0f, .8f);

		if ((stringTouchLH[i] == 1) && (stringPositions[i] == 65535))
		{
			//left hand mute
			audioIn = 0.0f;
			follower[i].d_coeff  = SHORT_DECAY;
			tADSR_off(&mainEnv[i]);
			tRamp_setDest(&ampRamp[i], 0.0f);
		}
		else if (stringTouchRH[i] == 1)
		{
			//right hand mute
			audioIn = 0.0f;
			follower[i].d_coeff  = SHORT_DECAY;
			tADSR_off(&mainEnv[i]);
			tRamp_setDest(&ampRamp[i], 0.0f);
		}
		else
		{
			//not muted
			tRamp_setDest(&ampRamp[i], 1.0f);

			follower[i].d_coeff  = LONG_DECAY;
		}
		audioIn = tHighpass_tick(&dcBlocker[i], audioIn);
		envelope[i] = tEnvelopeFollower_tick(&follower[i], audioIn);



		timeSinceLastAttack[i]++;
		//trying to detect an attack
		if ((envelope[i] > prevEnvelope[i] + envelopeMargin) && (timeSinceLastAttack[i] > ATTACK_DETECTOR_BETWEEN_ATTACK_WAIT))
		{
			envelopeState[i] = 1; //rising
			envelopeCounter[i] = 0;
			envelopeCounterFall[i] = 0;
			timeSinceLastAttack[i] = 0;
		}

		if (timeSinceLastAttack[i] > 65535)
		{
			timeSinceLastAttack[i] = 65535;
		}
		if ((envelopeCounter[i] < ATTACK_DETECTOR_POST_PEAK) && (envelopeState[i] == 1))
		{
			envelopeMemory[i][envelopeCounter[i]] = envelope[i];
			envelopeCounter[i]++;
		}
		if (envelope[i] < prevEnvelope[i])
		{
			envelopeCounterFall[i]++;
			if ((envelopeCounterFall[i] >= ATTACK_DETECTOR_POST_PEAK) && (envelopeState[i] == 1)) //you were rising, and now you've been falling for at least ATTACK_DETECTOR_WAIT samples
			{
				envelopeState[i] = 2; // falling
				// send a noteOn, and record the velocity
				velocity[i] = 0.0f;
				/*
				for (int j = 0; j < ATTACK_DETECTOR_POST_PEAK; j++)
				{

					if (envelopeMemory[i][j] > velocity[i])
					{
						velocity[i] = envelopeMemory[i][j];
					}

				}
				*/
				velocity[i] = 0.1f;
				noteOnHappened[i] = 1;
			}
		}

		prevEnvelope[i] = envelope[i];



		if (noteOnHappened[i] == 1)
		{
			noteOnHappened[i] = 0;
			tEnvelope_on(&noiseEnv[i], velocity[i]);
			tEnvelope_on(&filterEnv[i], velocity[i]);
			tADSR_on(&mainEnv[i], 1.0f);
		}

		float filtFreq = tEnvelope_tick(&filterEnv[i]) * 40000.0f + stringFrequencies[i] * 2.0f;
		if (filtFreq > 18000.0f)
		{
			filtFreq = 18000.0f;
		}
		tSVF_setFreq(&lowpass[i], filtFreq);
		tSVF_setFreq(&noiseFilt[i], filtFreq * .2f);
		sample += tSVF_tick(&noiseFilt[i], (tNoise_tick(&myNoise[i]) * tEnvelope_tick(&noiseEnv[i])));

		tCycle_setFreq(&mySine[i],stringFrequencies[i]);
		tSVF_setFreq(&lowpass[i], (envelope[i] * 20000.0f) + 1000.0f);
		//sample += tCycle_tick(&mySine[i]) * envelope[i] * tRamp_tick(&ampRamp[i]);
		sample += tCycle_tick(&mySine[i]) * tADSR_tick(&mainEnv[i]);

		float sawtooths = 0.0f;
		for (int j = 0; j < NUM_SAWS; j++)
		{
			if (j == 0)
			{
				tSawtooth_setFreq(&mySaw[i][j], LEAF_midiToFrequency(stringMIDIVersionOfFrequencies[i] + detuneAmounts[i][j]) * 2.0f);
			}
			else
			{
				tSawtooth_setFreq(&mySaw[i][j], LEAF_midiToFrequency(stringMIDIVersionOfFrequencies[i] + detuneAmounts[i][j]));
			}
			//sawtooths += tSawtooth_tick(&mySaw[i][j]) * envelope[i] * tRamp_tick(&ampRamp[i]);
			sawtooths += tSawtooth_tick(&mySaw[i][j]) * tADSR_tick(&mainEnv[i]);
		}


		//sample += tSVF_tick(&lowpass[i], tSawtooth_tick(&mySaw[i])) * envelope[i]* tRamp_tick(&ampRamp[i]);
		sample += tSVF_tick(&lowpass[i], sawtooths);
	}

	//sample = 0.0f;
	sample *= .5f;
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
