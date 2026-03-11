#include "stm32f0xx.h"
#include <stdint.h>

//===========================================================================
// CONFIGURATION
//===========================================================================
#define SYSTEM_CLOCK       8000000 
#define INITIAL_FREQ_KHZ   150
#define DUTY_CYCLE_PERCENT 50

volatile uint32_t current_freq_khz = INITIAL_FREQ_KHZ;

void update_pwm_frequency(uint32_t freq_khz) {
    if (freq_khz < 1) freq_khz = 1; 
    
    uint32_t target_hz = freq_khz * 1000;
    uint32_t arr_val = (SYSTEM_CLOCK / target_hz) - 1;
    
    // Calculate the new pulse width and apply it
    uint32_t ccr_val = (arr_val + 1) * DUTY_CYCLE_PERCENT / 100;

    TIM1->ARR = arr_val;
    TIM1->CCR2 = ccr_val;
    
    TIM1->EGR |= TIM_EGR_UG; 
}

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 1000; i++) {
        __NOP(); 
    }
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
    // 1. Enable Clocks
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    init_uart_rx();

    // 2. Setup Buttons (PA0 Down, PB2 Up)
    GPIOA->MODER &= ~(3 << 0);     
    GPIOA->PUPDR |= (2 << 0);      
    GPIOB->MODER &= ~(3 << 4);     
    GPIOB->PUPDR |= (2 << 4);      

    // 3. Setup DIR+ (PA8) and EN+ (PC9)
    GPIOA->MODER |= (1 << 16); 
    GPIOC->MODER |= (1 << 18); 
    
    GPIOA->BSRR = (1 << 8);        // DIR+ HIGH (Default Forward)
    GPIOC->BRR  = (1 << 9);        // EN+ LOW  (Driver Enabled - Active Low)

    // 4. Setup PUL+ (PA9) as PWM (TIM1_CH2)
    GPIOA->MODER |= (2 << 18); 
    GPIOA->AFR[1] |= (2 << 4); 

    // 5. Initial Timer Configuration
    TIM1->PSC   = 0;
    update_pwm_frequency(INITIAL_FREQ_KHZ);
    
    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC1PE;
    TIM1->CCER  |= TIM_CCER_CC2E;
    TIM1->BDTR  |= TIM_BDTR_MOE;   
    TIM1->CR1   |= TIM_CR1_CEN;

    // 6. Main Loop
    while (1) {
        // --- Process Single-Byte UART Packet ---
        if (USART5->ISR & USART_ISR_RXNE) {
            uint8_t rx_byte = USART5->RDR;
            
            uint8_t is_reverse = (rx_byte & 0x80); 
            uint8_t raw_mag = (rx_byte & 0x7F);
            
            // Check for the special "OFF" command
            if (raw_mag == 127) {
                TIM1->CCR2 = 0;          // Set duty cycle to 0% to stop pulses
                TIM1->EGR |= TIM_EGR_UG; // Force update
            } 
            else {
                uint32_t decoded_freq = raw_mag + 100;
                
                if (decoded_freq >= 100 && decoded_freq <= 200) {
                    if (is_reverse) {
                        GPIOA->BRR = (1 << 8);  // PA8 Low (Reverse)
                    } else {
                        GPIOA->BSRR = (1 << 8); // PA8 High (Forward)
                    }
                    
                    current_freq_khz = decoded_freq;
                    // Calling this recalculates CCR2 > 0, instantly turning the motor back on
                    update_pwm_frequency(current_freq_khz); 
                }
            }
        }

        // --- Hardware Buttons ---
        // Note: Pressing a button while the motor is "OFF" via Python 
        // will automatically recalculate the PWM and turn it back on.
        if (GPIOA->IDR & (1 << 0)) {
            if (current_freq_khz > 100) {
                current_freq_khz -= 10;
                update_pwm_frequency(current_freq_khz);
            }
            delay_ms(200); 
        }

        if (GPIOB->IDR & (1 << 2)) {
            if (current_freq_khz < 200) {
                current_freq_khz += 10;
                update_pwm_frequency(current_freq_khz);
            }
            delay_ms(200); 
        }
    }
}