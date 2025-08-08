//#define RCC_BASE   0x40023800 // addressing the peripheral rcc clock
//#define GPIOA_BASE 0x40020000 // addressing the GPIO ports.
//

//#define RCC_AHB1ENR_ADDR     (RCC_BASE + 0x30) // register mapping
//#define GPIOA_MODER_ADDR     (GPIOA_BASE + 0x00) // register mapping
//#define GPIOA_ODR_ADDR		 (GPIOA_BASE + 0x14)
//#define GPIOA_BSRR_ADDR      (GPIOA_BASE + 0x18)

//volatile unsigned int* pRCC_AHB1ENR = (volatile unsigned int*)RCC_AHB1ENR_ADDR;
//volatile unsigned int* pGPIOA_MODER = (volatile unsigned int*)GPIOA_MODER_ADDR;
//volatile unsigned int* pGPIOA_ODR   = (volatile unsigned int*)GPIOA_ODR_ADDR;
//volatile unsigned int* pGPIOA_BSRR  = (volatile unsigned int*)GPIOA_BSRR_ADDR;
//
//
//void delay(volatile int count)
//{
//	while (count--); // for delay
//}
//
//
//int main(void)
//{
//	*pRCC_AHB1ENR |= (1<< 0); // bit manipulation is to set and reset the particualr pins right.
//
//	*pGPIOA_MODER &= ~(0b11 << 10);// resetting the pin 5
//
//	*pGPIOA_MODER |= (1<<10); // setting the pin 5
//
//	while(1)
//	{
//		*pGPIOA_BSRR = (1<<5);
//
//		delay(500000);
//
//		*pGPIOA_BSRR = (1<<(5+16));
//
//		delay(500000);
//	}
//}
//



#include <stdint.h> // for standard library to include uint8_t like those variables

// defining base addresses for peripherals
#define RCC_BASE   0x40023800 // base address for rcc peripheral
#define GPIOA_BASE 0x40020000 // base address for GPIOA

// rcc register structure ( only AHB1ENR )

typedef struct
{
	uint32_t reserved[12];   // padding for offset
	volatile uint32_t AHB1ENR;
} RCC_Typedef;


// GPIOA register Structure
typedef struct
{
	volatile uint32_t MODER;  // 0x00: mode register
	uint32_t reservved[5];    // padding for offset 0x04 to 0x10
	volatile uint32_t BSRR;   //
}GPIOA_TypeDef;



// Map peripheral to base address

#define RCC     ((RCC_Typedef *)RCC_BASE)
#define GPIOA   ((GPIOA_TypeDef *)GPIOA_BASE)

// Delay function with precise type
void delay(volatile uint32_t count) {
    while (count--) {
        __asm volatile("nop"); // Prevent compiler optimization
    }
}

// Bit masks for clarity
#define GPIOA_EN        (1U << 0)  // RCC AHB1ENR: Enable GPIOA clock
#define GPIOA_MODER5_OUT (1U << 10) // GPIOA MODER: Pin 5 output mode
#define GPIOA_MODER5_MASK (3U << 10) // GPIOA MODER: Pin 5 mode mask
#define GPIOA_BSRR_SET5  (1U << 5)  // GPIOA BSRR: Set Pin 5
#define GPIOA_BSRR_RESET5 (1U << (5 + 16)) // GPIOA BSRR: Reset Pin 5


int main(void) {
    // Enable GPIOA clock
    RCC->AHB1ENR |= GPIOA_EN; //

    // Configure GPIOA Pin 5 as output
    GPIOA->MODER &= ~GPIOA_MODER5_MASK; // Clear Pin 5 mode
    GPIOA->MODER |= GPIOA_MODER5_OUT;   // Set Pin 5 as output

    // Blink loop
    while (1) {
        GPIOA->BSRR = GPIOA_BSRR_SET5;  // Set Pin 5 high
        delay(500000);                  // Delay
        GPIOA->BSRR = GPIOA_BSRR_RESET5; // Set Pin 5 low
        delay(500000);                  // Delay
    }

    return 0; // Unreachable, but included for completeness
}
