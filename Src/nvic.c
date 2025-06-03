/*
 * nvic.c
 *
 *  Created on: May 28, 2025
 *      Author: doanh
 */


#include "stm32f407xx.h"

void NVIC_EnableIRQ(uint8_t IRQn) {
    if (IRQn <= 31)
        *NVIC_ISER0 |= (1 << IRQn);
    else if (IRQn <= 63)
        *NVIC_ISER1 |= (1 << (IRQn % 32));
    else if (IRQn <= 95)
        *NVIC_ISER2 |= (1 << (IRQn % 64));
}

void NVIC_SetPriority(uint8_t IRQn, uint8_t priority) {
    uint8_t iprx = IRQn / 4;
    uint8_t iprx_section = IRQn % 4;
    *(NVIC_PR_BASE_ADDR + iprx) |= (priority << ((8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED)));
}
