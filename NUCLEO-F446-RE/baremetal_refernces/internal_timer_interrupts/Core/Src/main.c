#include "stm32f446xx.h"

// define the gpio port and pin for the onboard led
#define LED_PORT GPIOA

// Define the timer we'll use
#define TIMER       TIM2
#define TIMER_IRQ   TIM2_IRQn

// define the interrupt period in milliseconds
#define PERIOD_MS  500


 //initializing the gpio functions
 void init_gpio(void)
 {
     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enablin the rccc clock for the ahb1 peripheral bus

     // configure the PAS% as gneral purpos input-output pin
     // clear the mode bits(bits 10 and 11)
     LED_PORT->MODER &= ~(GPIO_MODER_MODE5_Msk);// i'm choosing a port then choosing register for that port then resetting them using &= ~() mask
     LED_PORT->MODER |= GPIO_MODER_MODER5_0; // setting as output mode
 }

 //initializing the timer functions

 void init_timer(void)
 {
     // Enable the clock for TIM2
     RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // enabling the rcc for apb1 bus


     // Set the Prescaler to get a 1 kHz counter clock from 16 MHz
     TIMER->PSC = 15999;

     // Set the Auto-Reload Register for a 500ms period
     // 1000 Hz / (1 / 0.5s) = 500. So ARR = 500 - 1 = 499
     TIMER->ARR = 499; // auto reload register
     //arr means auto reload register

     // Enable the update interrupt (UIE)

     TIMER->DIER |= TIM_DIER_UIE; // dma/interrupt reload register, eie menas update interrupt enable.

     // Enable the TIM2 interrupt in the Nested Vector Interrupt Controller (NVIC)
     NVIC_EnableIRQ(TIM2_IRQn); // irq means interrupt request.

     // start the timer
     TIMER->CR1 |= TIM_CR1_CEN; // counter enable
 }

 void TIM2_IRQHandler(void)
 {
     // Check if the update interrupt flag is set

     if (TIMER->SR & TIM_SR_UIF) // uif means update inteerupt flag
     {
         TIMER->SR &= ~(TIM_SR_UIF);// clearing the interrupt flag in this

         //TOGGLE THE LED PIN USING THE XOR OPERATOR
         GPIOA->ODR ^= GPIO_ODR_OD5;
     }
 }


 int main(void)
 {
     // initialize the gpio and time peripherals
     init_gpio();
     init_timer();

     while (1)
     {

     }

     return 0;
 }

