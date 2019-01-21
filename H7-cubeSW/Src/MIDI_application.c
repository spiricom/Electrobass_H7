/*
 * MIDI_application.c
 *
 *  Created on: 6 d�c. 2014
 *      Author: Xavier Halgand
 *
 *	Modified on: 9/12/16 by C.P. to handle the MIDI_IDLE state properly, and 
 *	added required code to be compatible with "NucleoSynth"
 *
 *	11/8/17 by C.P.: Version 0.7.7 - Use for Casio CTK-6200 Keyboard
 */

/* Includes ------------------------------------------------------------------*/
#include "audiostream.h"
#include "main.h"
#include "MIDI_application.h"
#include "usbh_core.h"
#include "usbh_MIDI.h"
#include "usb_host.h"


MIDI_ApplicationTypeDef MIDI_Appli_state = MIDI_APPLICATION_READY;
extern ApplicationTypeDef Appli_state;
extern USBH_HandleTypeDef hUsbHostFS;
uint8_t MIDI_RX_Buffer[RX_BUFF_SIZE] __ATTR_RAM_D2; // MIDI reception buffer

uint8_t key, velocity, ctrl, data;

uint8_t paramvalue[256], firstkey, h;
/* Private define ------------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/




/*-----------------------------------------------------------------------------*/
/**
 * @brief  Main routine for MIDI application, looped in main.c
 * @param  None
 * @retval none
 */
void MIDI_Application(void)
{
	if(Appli_state == APPLICATION_READY)
	{
		if(MIDI_Appli_state == MIDI_APPLICATION_READY)
		{

			USBH_MIDI_Receive(&hUsbHostFS, MIDI_RX_Buffer, RX_BUFF_SIZE); // just once at the beginning, start the first reception
			MIDI_Appli_state = MIDI_APPLICATION_RUNNING;
		}
	}
	if(Appli_state == APPLICATION_DISCONNECT)
	{
		MIDI_Appli_state = MIDI_APPLICATION_READY;
		USBH_MIDI_Stop(&hUsbHostFS);
	}
}

/*-----------------------------------------------------------------------------*/
void ProcessReceivedMidiDatas(uint32_t myLength)
{
	
}

void LocalMidiHandler(uint8_t param, uint8_t data)
{
	switch(param)
	{
		case (0): // pitch wheel
			break;
		case (1): // modulation wheel
			break;
		case (2): // Tuning
			
			break;
		case (3): // Wave Select
			
			break;
		case (4): // OSC Mix
			
			break;
		case (5): // De-Tune 
			
			break;
		case (6): // Scale
			
			break;
		case (7): // Resonance
			
			break;
		case (8): // Pulse Width Value
			
			break;
		case (9): // PWM Level
			
			break;
		case (10): // VCF Attack
			
			break;
		case (11): // VCF Decay
			
			break;
		case (12): // VCF Sustain
			
			break;
		case (13): // VCF Release
			
			break;
		case (14): // VCA Attack
			
			break;
		case (15): // VCA Decay
			
			break;
		case (16): // VCA Sustain
			
			break;
		case (17): // VCA Release
			
			break;
		case (18): // VCF Follow Level
			
			break;
		case (19): // ENV Follow Level
			
			break;
		case (20): // Velocity Select
			
			break;
		case (21): // VCF Envelope Level
			
			break;
		case (22): // Mod LFO rate
			
			break;
		case (23): // Pwm LFO rate
			
			break;
	}
	paramvalue[param] = data;
}

/*-----------------------------------------------------------------------------*/
/**
 * @brief  MIDI data receive callback.
 * @param  phost: Host handle
 * @retval None
 */
void USBH_MIDI_ReceiveCallback(USBH_HandleTypeDef *phost, uint32_t myLength)
{
	ProcessReceivedMidiDatas(myLength);
	USBH_MIDI_Receive(&hUsbHostFS, MIDI_RX_Buffer, RX_BUFF_SIZE); // start a new reception
}

