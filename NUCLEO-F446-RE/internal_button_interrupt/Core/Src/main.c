#include "stm32f446xx.h"

// define the gpio port and pin for the onboard led
#define LED_PORT GPIOA
#define LED_PIN  5
#define LED_PIN_POS  (1U << LED_PIN) // setting the led pin to enable

// defining port for internal button
#define BUTTON_PORT GPIOC
#define BUTTON_PIN  13
#define BUTTON_PIN_POS (1U << BUTTON_PIN)

// define the EXTI LINE FOR THE BUTTON
#define BUTTON_EXTI_IRQ EXTI15_10_IRQn

// Define the timer we'll use
#define TIMER       TIM2
#define TIMER_IRQ   TIM2_IRQn

// define the interrupt period in milliseconds
#define PERIOD_MS  500


// //initializing the gpio functions
// void init_gpio(void)
// {
//     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enablin the rccc clock for the ahb1 peripheral bus

//     // configure the PAS% as gneral purpos input-output pin
//     // clear the mode bits(bits 10 and 11)
//     LED_PORT->MODER &= ~(GPIO_MODER_MODE5_Msk);// i'm choosing a port then choosing register for that port then resetting them using &= ~() mask 
//     LED_PORT->MODER |= GPIO_MODER_MODER5_0; // setting as output mode 
// }

// //initializing the timer functions

// void init_timer(void)
// {
//     // Enable the clock for TIM2
//     RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // enabling the rcc for apb1 bus

    
//     // Set the Prescaler to get a 1 kHz counter clock from 16 MHz
//     TIMER->PSC = 15999;

//     // Set the Auto-Reload Register for a 500ms period
//     // 1000 Hz / (1 / 0.5s) = 500. So ARR = 500 - 1 = 499
//     TIMER->ARR = 499; // auto reload register
//     //arr means auto reload register

//     // Enable the update interrupt (UIE)

//     TIMER->DIER |= TIM_DIER_UIE; // dma/interrupt reload register, eie menas update interrupt enable.
    
//     // Enable the TIM2 interrupt in the Nested Vector Interrupt Controller (NVIC)
//     NVIC_EnableIRQ(TIMER_IRQ); // irq means interrupt request.

//     // start the timer
//     TIMER->CR1 |= TIM_CR1_CEN;
// }

// void TIM2_IRQHandler(void)
// {
//     // Check if the update interrupt flag is set

//     if (TIMER->SR & TIM_SR_UIF) // uif means update inteerupt flag
//     {
//         TIMER->SR &= ~(TIM_SR_UIF);// clearing the interrupt flag in this 

//         //TOGGLE THE LED PIN USING THE XOR OPERATOR
//         LED_PORT->ODR ^= LED_PIN_POS;
//     }
// }


// int main(void)
// {
//     // initialize the gpio and time peripherals
//     init_gpio();
//     init_timer();

//     while (1)
//     {

//     }
    
//     return 0;
// }



//function prototypes
void init_gpio(void);
void init_button_exti(void);

// main function
int main(void)
{
    init_gpio();
    init_button_exti();

    while (1)
    {
        /* code */
    }
   
}

void init_gpio(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;// ENABLING THE AHB 1 BUS FOR GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // ENABLING THE AHB1 BUS GPIOC PORT

    //CONFIGURING THE PAS5 INTERNAL LED AS THE OUTPUT PIN
    LED_PORT->MODER &= ~(GPIO_MODER_MODER5_Msk); // THIS SETS THE PORT A MASK FOR IT 

    LED_PORT->MODER |= GPIO_MODER_MODER5_0;// THIS sETS THE PIN TO OUTPUT MODE

    //configuring the button for default input mode adn setting the pull up down configurations

    BUTTON_PORT->MODER &= ~(GPIO_MODER_MODER13_Msk);// CLEARING THE 13 PIN SO I CAN SET THE PIN TO DEFAULT INPUT MODE

    BUTTON_PORT->PUPDR &= ~(GPIO_PUPDR_PUPD13_Msk); // CLEARING THE PULL UP DOWN CONFIGURATION SO I CAN CLEAR THE SETTING
    BUTTON_PORT->PUPDR |= GPIO_PUPDR_PUPD13_0; // this is basically (1U << 26U)

}



void init_button_exti(void)
{
    // enabling the clock for ssyconfig becuase it is connected in apb2 buss
    RCC->APB1ENR |= RCC_APB2ENR_SYSCFGEN;

    //SYSCFG CONTROLS THE EXTI 
    // NOW I HAVE CHOOSE THE13 PIN FOR THE INTERUTP TRIGGERING 
    // SINCE EACH CR REGISTER CONTROLS 4 EXTI INTERRUPTS I wanted to control them perfectly.
    //cr4 has the control of pin 12- pin 15
    SYSCFG->EXTICR[3] &= ~(SYSCFG_EXTICR4_EXTI13_Msk);// SETTING THE SYSCFG A MASK TO CLEAR THE REGISTER

    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;

    // CONFIGURING THE EXTI LINE TO TRIGGER ON A FALLING EDGE.
    //ftsr means falling edge selection trigger
    // The button connects to ground when pressed, causing a high-to-low transition.

    EXTI->FTSR |= EXTI_FTSR_TR13; // SETTING THE FALLING EDGE TRIGGER FOR PIN 13

    //CONFIGURING THE INTERRUPT SIGNAL TO BE SENT 
    // 5. Unmask the interrupt for EXTI line 13
    // This allows the interrupt signal to be sent to the NVIC
    EXTI->IMR |= EXTI_IMR_IM13;// SETTING THE INTERRUPT MASK REGIST FOR 13 PIN TO ENABLE THE PIN FOR ALLOWING IT TO SEND SIGNAL TO NVIC
    // IMR MEANS INTERRUPT MASK REGISTER
    NVIC_EnableIRQ(BUTTON_EXTI_IRQ);
}


void EXTI15_10_IRQHandler(void)
{
    if(EXTI->PR & EXTI_PR_PR13)
    {
        LED_PORT->ODR ^= GPIO_ODR_OD5_Pos;

        // Clear the interrupt pending flag by writing a '1' to it

        EXTI->PR |= EXTI_PR_PR13;
    }
}