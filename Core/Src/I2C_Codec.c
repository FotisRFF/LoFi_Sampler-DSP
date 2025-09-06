/*
 * I2C_Codec.c
 *
 *  Created on: Jun 8, 2025
 *      Author: Stamatakis Fotis
 */

#include "I2C_Codec.h"

// Initialize the codec
HAL_StatusTypeDef Codec_Init(TLV_CODEC *codec, I2C_HandleTypeDef *i2cHandle )
{
    codec->i2cHandle = i2cHandle;

    //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
   //HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // release reset
    HAL_Delay(10);


    HAL_StatusTypeDef status;

    // Software Reset
    status = Codec_WriteRegister(codec, 0x01, 0x01);
    HAL_Delay(10);

    // Clock settings
    status = Codec_WriteRegister(codec, 0x03, 0x10);

    // Audio routing
    status = Codec_WriteRegister(codec, 0x07, 0x08);

    status = Codec_WriteRegister(codec, 0x0F, 0x00);

    status = Codec_WriteRegister(codec, 0x10, 0x80);

    status = Codec_WriteRegister(codec, 0x11, 0x0F);

    status = Codec_WriteRegister(codec, 0x13, 0x7C);

    status = Codec_WriteRegister(codec, 0x25, 0x80);

    status = Codec_WriteRegister(codec, 0x28, 0x80);

    status = Codec_WriteRegister(codec, 0x2B, 0x00);

    status = Codec_WriteRegister(codec, 0x52, 0x80);

    status = Codec_WriteRegister(codec, 0x56, 0x0B);

    status = Codec_WriteRegister(codec, 0x65, 0x01);

    return status;
}

// Write to codec register
HAL_StatusTypeDef Codec_WriteRegister(TLV_CODEC *codec, uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(codec->i2cHandle, CODEC_I2C_ADDRESS1, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

// Read from codec register
HAL_StatusTypeDef Codec_ReadRegister(TLV_CODEC *codec, uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(codec->i2cHandle, CODEC_I2C_ADDRESS1, reg,I2C_MEMADD_SIZE_8BIT,  value, 1, HAL_MAX_DELAY);
}

