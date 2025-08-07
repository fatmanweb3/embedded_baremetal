#include <stdint.h>
#include <stdio.h>

// --- Base Addresses ---
#define RCC_BASE      0x40023800
#define GPIOA_BASE    0x40020000
#define GPIOB_BASE    0x40020400
#define USART2_BASE   0x40004400
#define I2C1_BASE     0x40005400

// --- MPU6050 I2C Address ---
#define MPU6050_ADDR  (0x68 << 1)
#define MPU6050_WHO_AM_I_REG 0x75
#define MPU6050_PWR_MGMT_1_REG 0x6B
#define MPU6050_ACCEL_XOUT_H_REG 0x3B

// --- Register Structures (Typedefs) ---

typedef struct {
    volatile uint32_t CR; volatile uint32_t PLLCFGR; volatile uint32_t CFGR;
    volatile uint32_t CIR; volatile uint32_t AHB1RSTR; volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR; uint32_t RESERVED0; volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR; uint32_t RESERVED1[2]; volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR; volatile uint32_t AHB3ENR; uint32_t RESERVED2;
    volatile uint32_t APB1ENR; volatile uint32_t APB2ENR; uint32_t RESERVED3[2];
    volatile uint32_t AHB1LPENR; volatile uint32_t AHB2LPENR; volatile uint32_t AHB3LPENR;
    uint32_t RESERVED4; volatile uint32_t APB1LPENR; volatile uint32_t APB2LPENR;
} RCC_Typedef;

typedef struct {
    volatile uint32_t MODER; volatile uint32_t OTYPER; volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR; volatile uint32_t IDR; volatile uint32_t ODR;
    volatile uint32_t BSRR; volatile uint32_t LCKR; volatile uint32_t AFRL;
    volatile uint32_t AFRH;
} GPIO_Typedef;

typedef struct {
    volatile uint32_t SR; volatile uint32_t DR; volatile uint32_t BRR;
    volatile uint32_t CR1; volatile uint32_t CR2; volatile uint32_t CR3;
    volatile uint32_t GTPR;
} USART_Typedef;

typedef struct {
    volatile uint32_t CR1; volatile uint32_t CR2; volatile uint32_t OAR1;
    volatile uint32_t OAR2; volatile uint32_t DR; volatile uint32_t SR1;
    volatile uint32_t SR2; volatile uint32_t CCR; volatile uint32_t TRISE;
    volatile uint32_t FLTR;
} I2C_Typedef;


// --- Peripheral Pointers ---
#define RCC     ((RCC_Typedef *)RCC_BASE)
#define GPIOA   ((GPIO_Typedef *)GPIOA_BASE)
#define GPIOB   ((GPIO_Typedef *)GPIOB_BASE)
#define USART2  ((USART_Typedef *)USART2_BASE)
#define I2C1    ((I2C_Typedef *)I2C1_BASE)

// --- Bit Masks and Values ---
#define GPIOA_EN        (1U << 0)
#define GPIOB_EN        (1U << 1)
#define USART2_EN       (1U << 17)
#define I2C1_EN         (1U << 21)

// I2C GPIO
#define GPIOB_MODER6_AF (2U << 12)
#define GPIOB_MODER7_AF (2U << 14)
#define GPIOB_OTYPER6_OD (1U << 6)
#define GPIOB_OTYPER7_OD (1U << 7)
#define GPIOB_AFRL6_I2C1 (4U << 24)
#define GPIOB_AFRL7_I2C1 (4U << 28)

// UART GPIO
#define GPIOA_MODER2_AF (2U << 4)
#define GPIOA_MODER3_AF (2U << 6)
#define GPIOA_AFRL2_UART (7U << 8)
#define GPIOA_AFRL3_UART (7U << 12)

// --- Function Prototypes ---
void delay(volatile uint32_t count);
void uart_init(void);
void uart_write(char *str);
void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_write_addr(uint8_t addr);
void i2c_write_data(uint8_t data);
uint8_t i2c_read_byte(uint8_t ack);
void mpu6050_init(void);
void mpu6050_read_accel(int16_t *accel_data);


// --- Main Function ---
int main(void) {
    int16_t accel_data[3];
    char buffer[100];

    // --- System Initialization ---
    uart_init();
    i2c_init();

    delay(500000); // Give the sensor time to power up
    mpu6050_init();

    while(1) {
        // --- Read MPU6050 data ---
        mpu6050_read_accel(accel_data);

        // --- Format and send data via UART ---
        sprintf(buffer, "X: %6d, Y: %6d, Z: %6d\r\n",
                accel_data[0], accel_data[1], accel_data[2]);
        uart_write(buffer);

        delay(1000000);
    }
}


// --- Function Implementations ---

void delay(volatile uint32_t count) {
    while(count--) {
        __asm volatile("nop");
    }
}

void uart_init(void) {
    RCC->AHB1ENR |= GPIOA_EN;
    RCC->APB1ENR |= USART2_EN;

    GPIOA->MODER &= ~(3U << (2*2)); GPIOA->MODER |= GPIOA_MODER2_AF;
    GPIOA->AFRL &= ~(0xFU << (4*2)); GPIOA->AFRL |= GPIOA_AFRL2_UART;

    GPIOA->MODER &= ~(3U << (2*3)); GPIOA->MODER |= GPIOA_MODER3_AF;
    GPIOA->AFRL &= ~(0xFU << (4*3)); GPIOA->AFRL |= GPIOA_AFRL3_UART;

    USART2->BRR = 0x8B;
    USART2->CR1 = (1U << 13) | (1U << 3) | (1U << 2);
}

void uart_write(char *str) {
    while(*str) {
        while(!(USART2->SR & (1U << 7)));
        USART2->DR = *str++;
    }
}

void i2c_init(void) {
    RCC->AHB1ENR |= GPIOB_EN;
    RCC->APB1ENR |= I2C1_EN;

    GPIOB->MODER &= ~(3U << 12); GPIOB->MODER |= GPIOB_MODER6_AF;
    GPIOB->OTYPER |= GPIOB_OTYPER6_OD;
    GPIOB->PUPDR |= (1U << 12);
    GPIOB->AFRL |= GPIOB_AFRL6_I2C1;

    GPIOB->MODER &= ~(3U << 14); GPIOB->MODER |= GPIOB_MODER7_AF;
    GPIOB->OTYPER |= GPIOB_OTYPER7_OD;
    GPIOB->PUPDR |= (1U << 14);
    GPIOB->AFRL |= GPIOB_AFRL7_I2C1;

    I2C1->CR1 &= ~(1U << 0);
    I2C1->CR2 = 16U;
    I2C1->CCR = 80;
    I2C1->TRISE = 17;

    I2C1->CR1 |= (1U << 0);
}

void i2c_start(void) {
    I2C1->CR1 |= (1U << 8);
    while(!(I2C1->SR1 & (1U << 0)));
}

void i2c_stop(void) {
    I2C1->CR1 |= (1U << 9);
    while(I2C1->SR2 & (1U << 1)); // Wait until bus is not busy
}

void i2c_write_addr(uint8_t addr) {
    I2C1->DR = addr;
    while(!(I2C1->SR1 & (1U << 1)));
    (void)I2C1->SR2; // Clear ADDR flag by reading SR2
}

void i2c_write_data(uint8_t data) {
    while(!(I2C1->SR1 & (1U << 7)));
    I2C1->DR = data;
    while(!(I2C1->SR1 & (1U << 2)));
}

uint8_t i2c_read_byte(uint8_t ack) {
    if (ack) {
        I2C1->CR1 |= (1U << 10);
    } else {
        I2C1->CR1 &= ~(1U << 10);
    }
    while(!(I2C1->SR1 & (1U << 6)));
    return I2C1->DR;
}

void mpu6050_init(void) {
    uint8_t who_am_i;
    char buffer[50];

    i2c_start();
    i2c_write_addr(MPU6050_ADDR | 0x00);
    i2c_write_data(MPU6050_PWR_MGMT_1_REG);
    i2c_write_data(0x00);
    i2c_stop();

    // Check WHO_AM_I register to confirm communication
    i2c_start();
    i2c_write_addr(MPU6050_ADDR | 0x00);
    i2c_write_data(MPU6050_WHO_AM_I_REG);
    i2c_stop();
    i2c_start();
    i2c_write_addr(MPU6050_ADDR | 0x01);
    who_am_i = i2c_read_byte(0);
    i2c_stop();

    if (who_am_i != 0x68) {
        sprintf(buffer, "Error: MPU6050 not found. WHO_AM_I: 0x%X\r\n", who_am_i);
        uart_write(buffer);
        while(1); // Stop execution
    } else {
        uart_write("MPU6050 initialized successfully!\r\n");
    }
    delay(100000);
}

void mpu6050_read_accel(int16_t *accel_data) {
    uint8_t buffer[6];

    i2c_start();
    i2c_write_addr(MPU6050_ADDR | 0x00);
    i2c_write_data(MPU6050_ACCEL_XOUT_H_REG);
    i2c_stop();

    i2c_start();
    i2c_write_addr(MPU6050_ADDR | 0x01);
    buffer[0] = i2c_read_byte(1);
    buffer[1] = i2c_read_byte(1);
    buffer[2] = i2c_read_byte(1);
    buffer[3] = i2c_read_byte(1);
    buffer[4] = i2c_read_byte(1);
    buffer[5] = i2c_read_byte(0);
    i2c_stop();

    accel_data[0] = (int16_t)(buffer[0] << 8 | buffer[1]);
    accel_data[1] = (int16_t)(buffer[2] << 8 | buffer[3]);
    accel_data[2] = (int16_t)(buffer[4] << 8 | buffer[5]);
}
