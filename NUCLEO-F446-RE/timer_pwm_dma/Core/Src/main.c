#include "stm32f446xx.h"
#include "stdint.h"

#define FADE_STEPS 1001
#define TOTAL_STEPS (FADE_STEPS * 2 - 2) // 0->1000->0

static uint16_t fade_pattern[TOTAL_STEPS];

void generate_fade_pattern(void)
{
    // Determine top duty value (safe fallback if TIM2 not initialized)
    uint32_t top = (TIM2->ARR != 0) ? (uint32_t)TIM2->ARR : 999U;

    // Fill ascending part: indices 0 .. FADE_STEPS-1
    // We scale i in range [0 .. FADE_STEPS-1] to [0 .. top]
    for (uint32_t i = 0; i < FADE_STEPS; i++) {
        fade_pattern[i] = (uint16_t)((i * top) / (FADE_STEPS - 1));
    }

    // Fill descending part by mirroring the ascending values
    // indices FADE_STEPS .. TOTAL_STEPS-1
    for (uint32_t i = FADE_STEPS; i < TOTAL_STEPS; i++) {
        // TOTAL_STEPS = 2*FADE_STEPS - 2
        // Mirror index -> gives descending sequence and avoids duplicating endpoints
        fade_pattern[i] = fade_pattern[TOTAL_STEPS - i];
    }
}

void rcc_init(void)
{
    // Enable GPIOA, TIM2, DMA1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    (void)RCC->AHB1ENR; // dummy read to ensure clock ready
}

void gpio_init(void)
{
    // PA5 as Alternate Function (AF1 = TIM2_CH1)
    GPIOA->MODER &= ~(GPIO_MODER_MODER5_Msk);
    GPIOA->MODER |= (2U << GPIO_MODER_MODER5_Pos); // AF mode

    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL5_Msk);
    GPIOA->AFR[0] |= (1U << GPIO_AFRL_AFSEL5_Pos); // AF1
}

void tim2_init(void)
{
    // Timer clock: adjust PSC for desired PWM frequency
    TIM2->PSC = 15;      // Prescaler
    TIM2->ARR = 999;     // 1000 steps => 1 kHz if timer tick = 1 MHz
    TIM2->CCR1 = 0;      // start at 0% duty

    // PWM Mode 1 on CH1
    TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk);
    TIM2->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos); // PWM mode 1
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;            // preload enable

    // Enable CH1 output
    TIM2->CCER |= TIM_CCER_CC1E;

    // Enable ARR preload
    TIM2->CR1 |= TIM_CR1_ARPE;

    // Enable DMA request on update event
    TIM2->DIER |= TIM_DIER_UDE;
}

void dma_init(void)
{
    // TIM2_UP usually maps to DMA1_Stream1, Channel 3 (check RM)
    DMA1_Stream1->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream1->CR & DMA_SxCR_EN);

    DMA1_Stream1->PAR = (uint32_t)&TIM2->CCR1;  // peripheral: CCR1
    DMA1_Stream1->M0AR = (uint32_t)fade_pattern; // memory: pattern array
    DMA1_Stream1->NDTR = TOTAL_STEPS;

    // Configure DMA: Channel 3, high priority, 16-bit mem & periph
    DMA1_Stream1->CR =
        (3U << DMA_SxCR_CHSEL_Pos) |   // Channel 3 = TIM2_UP
        DMA_SxCR_PL_1 |                // Priority high
        DMA_SxCR_MSIZE_0 |             // Memory size 16-bit
        DMA_SxCR_PSIZE_0 |             // Peripheral size 16-bit
        DMA_SxCR_MINC |                // Increment memory
        DMA_SxCR_CIRC |                // Circular mode
        DMA_SxCR_DIR_0;                // Mem-to-periph

    // Direct mode (FIFO disabled)
    DMA1_Stream1->FCR = 0;

    // Enable DMA stream
    DMA1_Stream1->CR |= DMA_SxCR_EN;
}

int main(void)
{

    rcc_init();
    gpio_init();
    tim2_init();
    generate_fade_pattern();
    dma_init();

    // Start TIM2
    TIM2->CR1 |= TIM_CR1_CEN;

    while (1)
    {
        // sleep until event (optional)
    }
}
