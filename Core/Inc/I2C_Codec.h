/*
 * I2C_Codec.h
 *
 *  Created on: Jun 8, 2025
 *      Author: Stamatakis Fotis
 *      Codec: TLV320AIC3104
 *      Datasheet: https://www.ti.com/lit/ds/symlink/tlv320aic3104.pdf
 */

#ifndef INC_I2C_CODEC_H_
#define INC_I2C_CODEC_H_

#include "stm32f4xx_hal.h"

/*
 * I2C 7-bit address from datasheet: 0x18 (page 45)
 * HAL expects 8-bit address, so shift left by 1
 */
#define CODEC_I2C_ADDRESS1 (0x18 << 1)

/*
 * TLV_CODEC struct definition
 */
typedef struct {
    I2C_HandleTypeDef *i2cHandle;
    GPIO_TypeDef *Codec_reset_pin_bank;
    uint16_t Codec_reset_pin;
} TLV_CODEC;

/*
 * Public function prototypes
 */

// Initialize codec with I2C and reset pin
HAL_StatusTypeDef Codec_Init(TLV_CODEC *codec,I2C_HandleTypeDef *i2cHandle);

// Write a value to a codec register
HAL_StatusTypeDef Codec_WriteRegister(TLV_CODEC *codec, uint8_t reg, uint8_t value);

// Read a value from a codec register
HAL_StatusTypeDef Codec_ReadRegister(TLV_CODEC *codec, uint8_t reg, uint8_t *value);


#endif /* INC_I2C_CODEC_H_ */
