#include "stm32f446xx.h"
#include <stdint.h>


static void rcc_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // ENABLING THE GPIOA
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // ENABLING THE TIMER 
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;// ENABLING THE ADC
}

static void gpio_init(void)
{
   GPIOA->MODER &= ~(GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER0_Msk); // CLREARING BITS FOR PWM PIN AND ADC PIN 

   GPIOA->MODER |= (2U << GPIO_MODER_MODER5_Pos) | (3U << GPIO_MODER_MODER0_Pos); // SETTIGN THE PIN MODES 2U IS FOR ALTERNATE FUNCTION MODE AF1, FOR PA0 ANLOG INPUT SO NO NEED FOR OTHER FUNCTIONS 

   GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL5_Msk); // CLEARING TEH PREVIOSU AF MODE
   
   GPIOA->AFR[0] |= ( 1U << GPIO_AFRL_AFSEL5_Pos);
}

static void adc1_init(void)
{

  //settign the adc prescaler 
  ADC->CCR &= ~(ADC_CCR_ADCPRE_Msk); // CLEARED THE PRESCALER SO IT SETTED TO 00 

  //SETTING THE SAMPLE TIME FOR CHANNEL 0 : CHOOSE THE LONG TO EASE SOURCE IMPEDANCE (e.g., 84 cycles) */

  ADC1->SMPR2 &= ~(ADC_SMPR2_SMP0_Msk); // CLEARING THE BITS
  ADC1->SMPR2 |= (7U << ADC_SMPR2_SMP0_Pos); // SETTING 480 CYCLES FOR ACCURACY.

  // SETTING THE SEQUENC REGISTER BY USING THIS REGISTER WE CAN CHOOSE THE LENGTH & THE CHANNEL FOR OUTR TARGET 
  ADC1->SQR1 &= ~(ADC_SQR1_L_Msk); //CLEARING TEH SEQUENC REGISTER.
  ADC1->SQR1 |= (0U << ADC_SQR1_L_Pos); // SETTING 16 CONVERSIONS FOR ACCURACY

  //SETTING THE CHANNEL 
  ADC1->SQR3 &= ~(ADC_SQR3_SQ1_Msk);
  ADC1->SQR3 |= (0U << ADC_SQR3_SQ1_Pos); // SETTED FOR CHANNEL 0 

  //SETTING THE CHANNEL FOR CONTINUOS CONVERSION MODE
  ADC1->CR2 |= (ADC_CR2_CONT);// CHOOSING FOR CONTINUOS CONVERSION MODE.
  ADC1->CR2 |= (ADC_CR2_ADON); //TURNING ON THE ADC.
}

volatile uint16_t adc_value;

uint16_t adc1_read(void)
{
  //start conversion (if not in continuos mode)
  ADC1->CR2 |= ADC_CR2_SWSTART;

  //WAIT UNTIL CONVERSION IS COMPLETE
  while(!(ADC1->SR & ADC_SR_EOC));//

  //read the data register
  return (uint16_t)ADC1->DR;
}


void tim2_init(void)
{
  //setting up the tim2 registers.
  
  //etting the prescaler

  TIM2->PSC = 15; //SETTING THE PRESCALER SO I CAN ADJUST EH VALUE.

  TIM2->ARR = 19999; // SETTING TEH ARR FOR THESE VALEUS CAN GENERATE A 50HZ PWM

  //SETTING THE DUTY CYCLE TO 0 

  TIM2->CCR1 = 1500 ; // Set initial duty cycle (1.5 ms -> center position)

  //PWM MODE 1  ON CHANNEL 1
  TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk);
  TIM2->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos); // PWM MODE 1 6U MEANS 110
  TIM2->CCMR1 |= TIM_CCMR1_OC1PE; // SETTING THE PRELOAD ENABLE

  // ENABLE THE CAPTURE/COMPARE MODE
  TIM2->CCER |= TIM_CCER_CC1E; // ENABLING TH CAPTURE COMPARE 1 LOCATION.

  // ENABLING THE AUTO-RELOAD PRELOAD
  TIM2->CR1 |= TIM_CR1_ARPE;

  //ENABLING THE TIMER
  TIM2->CR1 |= TIM_CR1_CEN; // ENABLING THE TIMER2 COUNTER.
}

static uint16_t map_adc_to_serve(uint16_t adc_value)
{
  // Map 0–4095 ADC value to 1000–2000 µs pulse width
  return 500 + ((uint32_t)adc_value * 2000)/4095;

}

int main(void)
{
  rcc_init();
  gpio_init();
  tim2_init();
  adc1_init();

  while (1)
  {
    //gettignt eh adc_value
    uint16_t adc_value = adc1_read();

    //gettign the duty cycle
    uint16_t pulse = map_adc_to_serve(adc_value);

    //settign the duty cycle to the tim2

    TIM2->CCR1 = pulse; // U[PDATE PWM DUTY CYCLE FOR SERVEO;
  }
  
}
