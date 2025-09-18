/*
 * Bitcrusher.c
 *
 *  Created on: Sep 7, 2025
 *      Author: stama
 */


float Bitcrusher(float in, int bitDepth, int downsampleFactor,
                 float *hold_sample, int *hold_count)
{
    // --- Bit depth reduction ---
    int levels = 1 << bitDepth;  // 2^bitDepth
    float crushed = roundf(((in + 1.0f) * 0.5f) * (levels - 1)) / (levels - 1);
    crushed = crushed * 2.0f - 1.0f;

    // --- Sample rate reduction (sample & hold) ---
    if (downsampleFactor > 1) {
        if (*hold_count == 0) {
            *hold_sample = crushed;
            *hold_count = downsampleFactor;
        }
        crushed = *hold_sample;
        (*hold_count)--;
    }

    return crushed;
}
