#include "stm32f0xx.h"
#include <stdint.h>
#include <stdlib.h>

#define TEST_MODE 1
#define WATCHDOG_MAX 300000 
#define SYSTEM_CLOCK 48000000 

// Use the robust clock config we discussed
void init_clock(void) {
    // 1. Enable HSE
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)); // Wait for external crystal to stabilize

    // 2. Flash Latency (Required for 48MHz)
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY;

    // 3. Configure PLL to use HSE
    // Assuming an 8MHz crystal: 8MHz / 1 (Prediv) * 6 (PLLMUL) = 48MHz
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL);
    RCC->CFGR |= (RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_PLLMUL6);
    
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)); 
    
    // 4. Switch System Clock to PLL
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); 
}

void set_motor_a(uint32_t percent, uint8_t is_rev) {
    if (percent == 0) {
        TIM1->CCR2 = 0; 
        return;
    }
    // PA8 Direction
    if (is_rev) GPIOA->BRR = GPIO_BRR_BR_8;  
    else        GPIOA->BSRR = GPIO_BSRR_BS_8; 

    uint32_t target_hz = percent * 2000; 
    uint32_t arr_val = (SYSTEM_CLOCK / target_hz) - 1;
    TIM1->ARR = arr_val;
    TIM1->CCR2 = (arr_val + 1) / 2; 
    TIM1->EGR |= TIM_EGR_UG; 
}

void set_motor_b(uint32_t percent, uint8_t is_rev) {
    if (percent == 0) {
        TIM3->CCR3 = 0; 
        return;
    }
    // PC7 Direction
    if (is_rev) GPIOC->BRR = GPIO_BRR_BR_7;  
    else        GPIOC->BSRR = GPIO_BSRR_BS_7; 

    uint32_t target_hz = percent * 2000; 
    uint32_t arr_val = (SYSTEM_CLOCK / target_hz) - 1;
    TIM3->ARR = arr_val;
    TIM3->CCR3 = (arr_val + 1) / 2; 
    TIM3->EGR |= TIM_EGR_UG; 
}

void init_uart(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART5EN; 

    // PD2 RX (AF2)
    GPIOD->MODER &= ~GPIO_MODER_MODER2;
    GPIOD->MODER |= GPIO_MODER_MODER2_1;        
    GPIOD->AFR[0] &= ~(0xF << (2 * 4));         
    GPIOD->AFR[0] |= (2 << (2 * 4));            

    // PC12 TX (AF2)
    GPIOC->MODER &= ~GPIO_MODER_MODER12;
    GPIOC->MODER |= GPIO_MODER_MODER12_1;       
    GPIOC->AFR[1] &= ~(0xF << ((12 - 8) * 4));  
    GPIOC->AFR[1] |= (2 << ((12 - 8) * 4));     

    USART5->BRR = SYSTEM_CLOCK / 115200;            
    USART5->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE; 
}

int main(void) {
    // 1. RECOVERY DELAY: If something goes wrong, you have 2 seconds 
    // to connect the debugger before the peripherals/clocks start.
    for(volatile int i = 0; i < 2000000; i++);

    init_clock();

    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    
    init_uart();

    // --- Motor A Setup (PA8 Dir, PA9 PWM AF2) ---
    GPIOA->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOA->MODER |= (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_1); 
    GPIOA->AFR[1] &= ~(0xF << (1 * 4)); // Clear AF for PA9
    GPIOA->AFR[1] |= (2 << (1 * 4));    // Set AF2 for PA9

    // --- Motor B Setup (PC7 Dir, PC8 PWM AF0) ---
    GPIOC->MODER &= ~(GPIO_MODER_MODER7 | GPIO_MODER_MODER8);
    GPIOC->MODER |= (GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_1);
    GPIOC->AFR[1] &= ~(0xF << 0);       // AF0 for PC8 is standard

    // --- Buttons (PA0, PB2) ---
    GPIOA->MODER &= ~GPIO_MODER_MODER0;
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1; // Pull-down
    GPIOB->MODER &= ~GPIO_MODER_MODER2;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR2_1;

    // Timer Init
    TIM1->BDTR |= TIM_BDTR_MOE; // Main Output Enable for TIM1
    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos);
    TIM1->CCER  |= TIM_CCER_CC2E;
    TIM1->CR1   |= TIM_CR1_CEN;

    TIM3->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos);
    TIM3->CCER  |= TIM_CCER_CC3E;
    TIM3->CR1   |= TIM_CR1_CEN;

    uint8_t pending_motor = 0, pending_dir = 0;
    uint32_t watchdog_timer = 0;

    while (1) {
        if (USART5->ISR & USART_ISR_RXNE) {
            uint8_t rx = USART5->RDR;
            if (rx & 0x80) { 
                pending_motor = (rx >> 6) & 0x01;
                pending_dir   = (rx >> 5) & 0x01;
            } else { 
                uint8_t percent = rx & 0x7F; 
                if (percent <= 100) { 
                    if (pending_motor == 0) set_motor_a(percent, pending_dir);
                    else                    set_motor_b(percent, pending_dir);
                    watchdog_timer = 0;
                }
            }
        } 
        
        if (TEST_MODE == 0) {
            if (++watchdog_timer > WATCHDOG_MAX) {
                set_motor_a(0, 0);
                set_motor_b(0, 0);
                watchdog_timer = WATCHDOG_MAX; 
            }
        }
    }
}