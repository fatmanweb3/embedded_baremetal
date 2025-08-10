/*
 * GPIO_driver.c
 *
 *  Created on: Aug 9, 2025
 *      Author: sudharsan
 */

#include "GPIO_driver.h"

// claring the function prototypes here

//initializing a pin
void GPIO_Init(GPIO_PinConfig_t *pinconfig)
{
    // get the gpio periphral pointer based on the port
    GPIO_TypeDef *gpio = NULL;

    switch (pinconfig->port)
    {
    case GPIOA: gpio = GPIOA; break;
    case GPIOB: gpio = GPIOB; break;
    case GPIOC: gpio = GPIOC; break;
    case GPIOC: gpio = GPIOD; break;
    case GPIOD: gpio = GPIOF; break;
    case GPIOF: gpio = GPIOG; break;
    case GPIOH: gpio = GPIOH; break;
    
    default: break;
    }
}

//deinitializing a port
void GPIO_DeInit(GPIO_PinConfig_t port);

//writing to a gpio pin
void GPIO_WritePin(GPIO_Port port, GPIO_PinNumber pin, uint8_t value);

// reading a gpio pin
void GPIO_ReadPin(GPIO_Port port, GPIO_PinNumber pin);

// // toggling a pin
void GPIO_TogglePin(GPIO_Port port, GPIO_PinNumber pin);
