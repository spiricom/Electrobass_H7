/*
  ==============================================================================

    LEAFInstrument.h
    Created: 20 Jan 2017 12:01:54pm
    Author:  Michael R Mulshine

  ==============================================================================
*/

#ifndef LEAFINSTRUMENT_H_INCLUDED
#define LEAFINSTRUMENT_H_INCLUDED

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

#include "leaf-globals.h"
#include "leaf-math.h"

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

/* tTalkbox */
#define NUM_TALKBOX_PARAM 4
#define TALKBOX_BUFFER_LENGTH 1024 //1600

typedef struct _tTalkbox
{
    float param[NUM_TALKBOX_PARAM];
    
    ///global internal variables
    float car0[TALKBOX_BUFFER_LENGTH], car1[TALKBOX_BUFFER_LENGTH];
    float window[TALKBOX_BUFFER_LENGTH];
    float buf0[TALKBOX_BUFFER_LENGTH], buf1[TALKBOX_BUFFER_LENGTH];
    
    float emphasis;
    int32_t K, N, O, pos;
    float wet, dry, FX;
    float d0, d1, d2, d3, d4;
    float u0, u1, u2, u3, u4;
    
} tTalkbox;

void        tTalkbox_init        (tTalkbox* const);
void        tTalkbox_free        (tTalkbox* const);
float       tTalkbox_tick        (tTalkbox* const, float synth, float voice);
void        tTalkbox_update      (tTalkbox* const);
void        tTalkbox_suspend     (tTalkbox* const);
void        tTalkbox_lpcDurbin   (float *r, int p, float *k, float *g);
void        tTalkbox_lpc         (float *buf, float *car, int32_t n, int32_t o);
void		tTalkbox_setQuality  (tTalkbox* const, float quality);

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

/* tVocoder */
#define NUM_VOCODER_PARAM 8
#define NBANDS 16

typedef struct _tVocoder
{
    float param[NUM_VOCODER_PARAM];
    
    float gain;         //output level
    float thru, high;   //hf thru
    float kout;         //downsampled output
    int32_t  kval;      //downsample counter
    int32_t  nbnd;      //number of bands
    
    //filter coeffs and buffers - seems it's faster to leave this global than make local copy
    float f[NBANDS][13]; //[0-8][0 1 2 | 0 1 2 3 | 0 1 2 3 | val rate]
    
} tVocoder;

void        tVocoder_init        (tVocoder* const);
void        tVocoder_free        (tVocoder* const);
float       tVocoder_tick        (tVocoder* const, float synth, float voice);
void        tVocoder_update      (tVocoder* const);
void        tVocoder_suspend     (tVocoder* const);

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

#endif  // LEAFINSTRUMENT_H_INCLUDED
