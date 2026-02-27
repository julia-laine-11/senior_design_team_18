#include "stm32f0xx.h"

//===========================================================================
// CONFIGURATION
//===========================================================================
#define SYSTEM_CLOCK       8000000 
#define INITIAL_FREQ_KHZ   150
#define DUTY_CYCLE_PERCENT 50

// Global variable to track current frequency
volatile uint32_t current_freq_khz = INITIAL_FREQ_KHZ;

// Function to calculate and update Timer ARR/CCR registers
void update_pwm_frequency(uint32_t freq_khz) {
    if (freq_khz < 1) freq_khz = 1; // Safety floor
    
    uint32_t target_hz = freq_khz * 1000;
    uint32_t arr_val = (SYSTEM_CLOCK / target_hz) - 1;
    uint32_t ccr_val = (arr_val + 1) * DUTY_CYCLE_PERCENT / 100;

    TIM1->ARR = arr_val;
    TIM1->CCR2 = ccr_val;
    
    // Force an update event to apply new values immediately
    TIM1->EGR |= TIM_EGR_UG; 
}

// Simple delay for debouncing buttons
void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 1000; i++) {
        __NOP(); 
    }
}

int main(void) {
    // 1. Enable Clocks: GPIOA, GPIOB, GPIOC, and TIM1
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // 2. Setup Buttons: PA0 and PB2 as Inputs with Pull-Down
    // PA0 (Down)
    GPIOA->MODER &= ~(3 << 0);     // Input mode
    GPIOA->PUPDR |= (2 << 0);      // Pull-down
    
    // PB2 (Up)
    GPIOB->MODER &= ~(3 << 4);     // Input mode
    GPIOB->PUPDR |= (2 << 4);      // Pull-down

    // 3. Setup DIR+ (PA8) and EN+ (PC9) as Outputs
    GPIOA->MODER |= (1 << 16); 
    GPIOC->MODER |= (1 << 18); 
    
    GPIOA->BSRR = (1 << 8);        // DIR+ HIGH
    GPIOC->BRR  = (1 << 9);        // EN+ LOW

    // 4. Setup PUL+ (PA9) as PWM (TIM1_CH2)
    GPIOA->MODER |= (2 << 18); 
    GPIOA->AFR[1] |= (2 << 4); 

    // 5. Initial Timer 1 Configuration
    TIM1->PSC   = 0;
    update_pwm_frequency(INITIAL_FREQ_KHZ);
    
    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC1PE;
    TIM1->CCER  |= TIM_CCER_CC2E;
    TIM1->BDTR  |= TIM_BDTR_MOE;   
    TIM1->CR1   |= TIM_CR1_CEN;

    // 6. Main Loop: Check Buttons
    while (1) {
        // PA0: Decrease Frequency (-10kHz)
        if (GPIOA->IDR & (1 << 0)) {
            if (current_freq_khz > 10) current_freq_khz -= 10;
            update_pwm_frequency(current_freq_khz);
            delay_ms(200); // Debounce
        }

        // PB2: Increase Frequency (+10kHz)
        if (GPIOB->IDR & (1 << 2)) {
            current_freq_khz += 10;
            update_pwm_frequency(current_freq_khz);
            delay_ms(200); // Debounce
        }
    }
}