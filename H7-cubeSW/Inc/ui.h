/*
 * ui.h
 *
 *  Created on: Dec 18, 2018
 *      Author: jeffsnyder
 */

#ifndef UI_H_
#define UI_H_


extern int mode1;
extern int mode2;
extern int mode3;
extern int RGB_mode;

extern int DAC1_active;
extern int DAC2_active;


//the typedef for assigning jack functions
typedef enum
{
	DIGITAL_INPUT = 0,
	DIGITAL_OUTPUT,
	ANALOG_INPUT,
	ANALOG_OUTPUT,
	AUDIO_INPUT,
	AUDIO_OUTPUT
}jackModeType;

void RGB_LED_setColor(uint8_t R, uint8_t G, uint8_t B);
void configure_Jack(uint8_t jackNumber, jackModeType jackMode);
void Invalid_Configuration(void);
void CV_DAC_Output(uint8_t DACnum, uint16_t value);

#endif /* UI_H_ */
