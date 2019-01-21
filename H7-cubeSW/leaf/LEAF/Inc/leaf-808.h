/*
  ==============================================================================

    leaf_808.h
    Created: 30 Nov 2018 10:24:44am
    Author:  airship

  ==============================================================================
*/

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

#include "leaf-globals.h"
#include "leaf-math.h"

#include "leaf-oscillator.h"
#include "leaf-utilities.h"
#include "leaf-filter.h"

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

// 808 Cowbell
typedef struct _t808Cowbell {
    
    tSquare p[2];
    tNoise stick;
    tSVF bandpassOsc;
    tSVF bandpassStick;
    tEnvelope envGain;
    tEnvelope envStick;
    tEnvelope envFilter;
    tHighpass highpass;
    float oscMix;
    float filterCutoff;
    
} t808Cowbell;

void    t808Cowbell_init             (t808Cowbell* const);
void    t808Cowbell_free             (t808Cowbell* const);
float   t808Cowbell_tick             (t808Cowbell* const);
void    t808Cowbell_on               (t808Cowbell* const, float vel);
void    t808Cowbell_setDecay         (t808Cowbell* const, float decay);
void    t808Cowbell_setHighpassFreq  (t808Cowbell* const, float freq);
void    t808Cowbell_setBandpassFreq  (t808Cowbell* const, float freq);
void    t808Cowbell_setFreq          (t808Cowbell* const, float freq);
void    t808Cowbell_setOscMix        (t808Cowbell* const, float oscMix);


// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

// 808 Hihat
typedef struct _t808Hihat {
    
    // 6 Square waves
    tSquare p[6];
    tNoise n;
    tSVF bandpassOsc;
    tSVF bandpassStick;
    tEnvelope envGain;
    tEnvelope envStick;
    tEnvelope noiseFMGain;
    tHighpass highpass;
    tNoise stick;

    float freq;
    float stretch;
    float FM_amount;
    float oscNoiseMix;
    
} t808Hihat;

void        t808Hihat_init               (t808Hihat* const);
void        t808Hihat_free               (t808Hihat* const);

float       t808Hihat_tick               (t808Hihat* const);
void        t808Hihat_on                 (t808Hihat* const, float vel);
void        t808Hihat_setOscNoiseMix     (t808Hihat* const, float oscNoiseMix);
void        t808Hihat_setDecay           (t808Hihat* const, float decay);
void        t808Hihat_setHighpassFreq    (t808Hihat* const, float freq);
void        t808Hihat_setOscBandpassFreq  (t808Hihat* const, float freq);
void 		t808Hihat_setOscBandpassQ		(t808Hihat* const hihat, float Q);
void        t808Hihat_setStickBandPassFreq  (t808Hihat* const, float freq);
void 		t808Hihat_setStickBandPassQ		(t808Hihat* const hihat, float Q);
void        t808Hihat_setOscFreq         (t808Hihat* const, float freq);
void 		t808Hihat_setStretch				(t808Hihat* const hihat, float stretch);
void 		t808Hihat_setFM					(t808Hihat* const hihat, float FM_amount);

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

// 808 Snare
typedef struct _t808Snare {
    
    // Tone 1, Tone 2, Noise
    tTriangle tone[2]; // Tri (not yet antialiased or wavetabled)
    tNoise noiseOsc;
    tSVF toneLowpass[2];
    tSVF noiseLowpass; // Lowpass from SVF filter
    tEnvelope toneEnvOsc[2];
    tEnvelope toneEnvGain[2];
    tEnvelope noiseEnvGain;
    tEnvelope toneEnvFilter[2];
    tEnvelope noiseEnvFilter;
    
    float toneGain[2];
    float noiseGain;
    
    float toneNoiseMix;
    
    float tone1Freq, tone2Freq;
    
    float noiseFilterFreq;
    
    
} t808Snare;

void        t808Snare_init                  (t808Snare* const);
void        t808Snare_free                  (t808Snare* const);

float       t808Snare_tick                  (t808Snare* const);
void        t808Snare_on                    (t808Snare* const, float vel);
void        t808Snare_setTone1Freq          (t808Snare* const, float freq);
void        t808Snare_setTone2Freq          (t808Snare* const, float freq);
void        t808Snare_setTone1Decay         (t808Snare* const, float decay);
void        t808Snare_setTone2Decay         (t808Snare* const, float decay);
void        t808Snare_setNoiseDecay         (t808Snare* const, float decay);
void        t808Snare_setToneNoiseMix       (t808Snare* const, float toneNoiseMix);
void        t808Snare_setNoiseFilterFreq    (t808Snare* const, float noiseFilterFreq);
void        t808Snare_setNoiseFilterQ       (t808Snare* const, float noiseFilterQ);

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

// 808 Kick
typedef struct _t808Kick {


    tCycle tone; // Tri
    tNoise noiseOsc;
    tSVF toneLowpass;
    tEnvelope toneEnvOscChirp;
    tEnvelope toneEnvOscSigh;
    tEnvelope toneEnvGain;
    tEnvelope noiseEnvGain;
    tEnvelope toneEnvFilter;

    float toneGain;
    float noiseGain;

    float toneInitialFreq;
    float sighAmountInHz;
    float chirpRatioMinusOne;
    float noiseFilterFreq;


} t808Kick;

void        t808Kick_init                  (t808Kick* const);
void        t808Kick_free                  (t808Kick* const);

float       t808Kick_tick                  (t808Kick* const);
void        t808Kick_on                    (t808Kick* const, float vel);
void        t808Kick_setToneFreq          (t808Kick* const, float freq);
void        t808Kick_setToneDecay         (t808Kick* const, float decay);
void        t808Kick_setNoiseDecay         (t808Kick* const, float decay);
void        t808Kick_setSighAmount         (t808Kick* const, float sigh);
void        t808Kick_setChirpAmount         (t808Kick* const, float chirp);
void        t808Kick_setToneNoiseMix       (t808Kick* const, float toneNoiseMix);
void        t808Kick_setNoiseFilterFreq    (t808Kick* const, float noiseFilterFreq);
void        t808Kick_setNoiseFilterQ       (t808Kick* const, float noiseFilterQ);

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

