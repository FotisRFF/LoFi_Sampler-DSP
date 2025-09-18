/*
 * InfiniFX - Simple Overdrive (no filters)
 *
 * Author: ChatGPT
 *
 * Usage:
 *   IFX_Overdrive od;
 *   IFX_Overdrive_Init(&od, 10.0f, 0.33f);
 *   float y = IFX_Overdrive_Update(&od, x);
 */

#ifndef IFX_OVERDRIVE_H
#define IFX_OVERDRIVE_H

#include <stdint.h>

typedef struct {
    float preGain;     // Input gain before distortion
    float threshold;   // Soft clipping threshold
} IFX_Overdrive;

// Initialize the overdrive with gain and threshold
void IFX_Overdrive_Init(IFX_Overdrive *od,float preGain, float threshold);

// Apply the overdrive effect to one sample (float in -1.0f..+1.0f range)
float IFX_Overdrive_Update(IFX_Overdrive *od, float inp);

float IFX_Overdrive_Set_preGain(IFX_Overdrive *od,float preGain);

#endif // IFX_OVERDRIVE_H
