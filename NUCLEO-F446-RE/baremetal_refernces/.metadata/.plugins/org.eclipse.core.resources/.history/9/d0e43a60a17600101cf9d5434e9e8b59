#include "stm32f446xx.h"
#include <stdint.h>


#define LED_PORT  GPIOA
#define LED_PIN   5

#define TIMER     TIM3 


// Define the system clock frequency (e.g., 16 MHz for the internal HSI)

#define SYS_CLK_FREQ 16000000 

// Define the desired PWM frequency (1 kHz)
#define PWM_FREQ   1000

// Define the prescaler value based on the system clock and desired timer clock
// PSC = (SYS_CLK_FREQ / TIMER_CLK_FREQ) - 1
// We want a timer clock of 1 kHz, so TIMER_CLK_FREQ = 1000 Hz.
// To achieve this, the timer's ARR should be the number of ticks per PWM cycle.
// If we want a 1 kHz PWM, and our counter counts up to ARR,
// then the counter should tick faster than 1 kHz.
// Let's aim for a timer clock of 1000000 Hz.
// Then PSC = (16000000 / 1000000) - 1 = 15.
#define PWM_PRESCALER ((SYS_CLK_FREQ / 1000000) - 1)

// Define the Auto-Reload Register value for the desired PWM frequency
// ARR = (TIMER_CLK_FREQ / PWM_FREQ) - 1
// If TIMER_CLK_FREQ = 1000000 Hz, and PWM_FREQ = 1000 Hz
// ARR = (1000000 / 1000) - 1 = 999.
#define PWM_ARR ((1000000 / PWM_FREQ) - 1)


// Global variables for duty cycle control
volatile int duty_cycle_step = 0;
volatile int ramp_direction = 1; // 1 for ramping up, -1 for ramping down


void init_gpio(void);
void TIM2_PWM_Init(void);
void TIM6_Timebase_Init(void);


void init_gpio(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enabling the clock for gpioa
	
	GPIOA->MODER &= ~(GPIO_MODER_MODER5_Msk); // clearing the pin for pa5

	GPIOA->MODER |= GPIO_MODER_MODER5_1; //setting the moder for alternate function mode

	GPIOA->AFR[0] &=  ~GPIO_AFRL_AFSEL5_Msk; // clears the bit first.

	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL5_1; // SET AF2
}

void TIM2_PWM_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // ENABLING THE TIMER 2 FOR APB 1 BUS

	// Set the prescaler and period for 1000 Hz PWM
    // Assuming 16MHz clock:
    // Period = (16,000,000 / (1000 * 100)) - 1 = 159
    // Prescaler = 0 (no prescaling, we use a single timer for frequency and resolution)
	TIM2->PSC = 159; // No prescaler, since we want a good duty cycle resolution
    TIM2->ARR = 99; // Period for 1000 Hz (16MHz / 16000 = 1000)

	//CONFIGURE THE TIMER 2 CHANNEL 1 FOR PWM MODE
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk);
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;

	TIM2->CCMR1 |= TIM_CCMR1_OC1PE;

	TIM2->CCER |= TIM_CCER_CC1E;

	TIM2->CCR1 = 0;

	TIM2->CR1 |= TIM_CR1_CEN;

}

// Timer 6 Timebase Initialization (20ms interrupt)
void TIM6_Timebase_Init(void) {
    // Enable the clock for TIM6
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    // Configure TIM6 for a 20ms interrupt
    // Assuming 16MHz clock:
    // Prescaler: 15999
    // Period: 19
    // (16,000,000 / (15999+1)) / (19+1) = 16,000,000 / 16,000 / 20 = 1000 / 20 = 50 Hz (20ms)
    TIM6->PSC = 15999;
    TIM6->ARR = 19;

    // Enable the Update Interrupt
    TIM6->DIER |= TIM_DIER_UIE;

    // Enable TIM6 interrupt in the NVIC
    NVIC_SetPriority(TIM6_DAC_IRQn, 0);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);

    // Start the timer
    TIM6->CR1 |= TIM_CR1_CEN;
}

// Timer 6 Interrupt Service Routine (ISR)
void TIM6_DAC_IRQHandler(void) {
    // Check if the update interrupt flag is set
    if (TIM6->SR & TIM_SR_UIF) {
        // Clear the interrupt flag
        TIM6->SR &= ~TIM_SR_UIF;

        // Update the duty cycle step
        duty_cycle_step += ramp_direction;

        // Update the PWM duty cycle
        TIM2->CCR1 = duty_cycle_step;

        // Check for ramp boundaries
        if (duty_cycle_step >= 99) { // At 100% duty cycle
            ramp_direction = -1; // Change direction to ramp down
        } else if (duty_cycle_step <= 0) { // At 0% duty cycle
            ramp_direction = 1; // Change direction to ramp up
        }
    }
}


int main(void)
{
	init_gpio();

	TIM2_PWM_Init();

	TIM6_Timebase_Init();

	while(1)
	{

	}
}