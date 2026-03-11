#include "stm32f0xx.h"
#include <stdint.h>

#define SYSTEM_CLOCK 8000000 

// --- Motor A Abstraction (TIM1_CH2 on PA9) ---
void set_motor_a(uint32_t percent, uint8_t is_rev) {
    if (percent == 0) {
        TIM1->CCR2 = 0; 
        TIM1->EGR |= TIM_EGR_UG;
        return;
    }
    if (is_rev) GPIOA->BRR = (1 << 8);  
    else        GPIOA->BSRR = (1 << 8); 

    uint32_t target_hz = percent * 2000; 
    uint32_t arr_val = (SYSTEM_CLOCK / target_hz) - 1;
    TIM1->ARR = arr_val;
    TIM1->CCR2 = (arr_val + 1) / 2; 
    TIM1->EGR |= TIM_EGR_UG; 
}

// --- Motor B Abstraction (TIM3_CH3 on PC8) ---
void set_motor_b(uint32_t percent, uint8_t is_rev) {
    if (percent == 0) {
        TIM3->CCR3 = 0; 
        TIM3->EGR |= TIM_EGR_UG;
        return;
    }
    if (is_rev) GPIOC->BRR = (1 << 7);  
    else        GPIOC->BSRR = (1 << 7); 

    uint32_t target_hz = percent * 2000; 
    uint32_t arr_val = (SYSTEM_CLOCK / target_hz) - 1;
    TIM3->ARR = arr_val;
    TIM3->CCR3 = (arr_val + 1) / 2; 
    TIM3->EGR |= TIM_EGR_UG; 
}

void init_uart_rx(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART5EN; 
    GPIOD->MODER &= ~GPIO_MODER_MODER2;
    GPIOD->MODER |= GPIO_MODER_MODER2_1;        
    GPIOD->AFR[0] &= ~(0xF << (2 * 4));         
    GPIOD->AFR[0] |= (2 << (2 * 4));            
    USART5->BRR = 8000000 / 115200;            
    USART5->CR1 = USART_CR1_RE | USART_CR1_UE; 
}

int main(void) {
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    init_uart_rx();

    // Motor A Pins (PA8 DIR+, PC9 EN+, PA9 PUL+)
    GPIOA->MODER &= ~((3 << 16) | (3 << 18));
    GPIOA->MODER |= (1 << 16) | (2 << 18); 
    GPIOA->AFR[1] |= (2 << 4);     
    GPIOC->MODER &= ~(3 << 18);
    GPIOC->MODER |= (1 << 18);     
    GPIOA->BSRR = (1 << 8);        
    GPIOC->BRR  = (1 << 9);        

    // Motor B Pins (PC7 DIR+, PC6 EN+, PC8 PUL+)
    GPIOC->MODER &= ~((3 << 12) | (3 << 14) | (3 << 16));
    GPIOC->MODER |= (1 << 12) | (1 << 14) | (2 << 16);
    GPIOC->AFR[1] &= ~(0xF << 0);  
    GPIOC->BSRR = (1 << 7);        
    GPIOC->BRR  = (1 << 6);        

    // Init Timers
    TIM1->PSC = 0;
    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC1PE;
    TIM1->CCER  |= TIM_CCER_CC2E;
    TIM1->BDTR  |= TIM_BDTR_MOE;   
    TIM1->CR1   |= TIM_CR1_CEN;

    TIM3->PSC = 0;
    TIM3->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
    TIM3->CCER  |= TIM_CCER_CC3E;  
    TIM3->CR1   |= TIM_CR1_CEN;

    // --- State Variables for the 2-Byte Protocol ---
    uint8_t pending_motor = 0;
    uint8_t pending_dir = 0;

    // 6. Main RX Loop
    while (1) {
        if (USART5->ISR & USART_ISR_RXNE) {
            uint8_t rx = USART5->RDR;
            
            // Check the 8th bit (MSB)
            if (rx & 0x80) { 
                // It's a Control Byte (MSB is 1)
                pending_motor = (rx >> 6) & 0x01;
                pending_dir   = (rx >> 5) & 0x01;
            } 
            else { 
                // It's a Payload Byte (MSB is 0)
                uint8_t percent = rx & 0x7F; // Strip MSB just in case
                
                if (percent <= 100) { // Safety check
                    if (pending_motor == 0) {
                        set_motor_a(percent, pending_dir);
                    } else {
                        set_motor_b(percent, pending_dir);
                    }
                }
            }
        }
    }
}