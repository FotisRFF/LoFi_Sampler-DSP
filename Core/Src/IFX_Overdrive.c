#include <math.h>
#include <stdint.h>

typedef struct {
    float preGain;     // input gain before distortion
    float threshold;   // soft clipping threshold
} IFX_Overdrive;

// Initialize the overdrive with gain and threshold
void IFX_Overdrive_Init(IFX_Overdrive *od, float preGain, float threshold) {
    od->preGain   = preGain;
    od->threshold = threshold; // typically ~0.33f
}

// Update function: apply soft clipping
float IFX_Overdrive_Update(IFX_Overdrive *od, float inp) {
    float x = od->preGain * inp;

    // hard clip around threshold
    if (x > od->threshold)  x =  od->threshold;
    if (x < -od->threshold) x = -od->threshold;

    // normalize back to -1..+1
    return x / od->threshold;
}

float IFX_Overdrive_Set_preGain(IFX_Overdrive *od,float preGain){

	 od->preGain=preGain;

}


