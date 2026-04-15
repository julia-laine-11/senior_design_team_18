// #include "stm32f0xx.h"
// #include <stdint.h>

// // Configures the PLL to multiply the 8MHz HSI to 48MHz
// void init_clock(void) {
//     // 1. Set Flash latency to 1 wait state and enable prefetch buffer (Required for >24 MHz)
//     FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;
    
//     // 2. Configure PLL: Source = HSI/2 (4 MHz), Multiplier = 12 -> 48 MHz
//     RCC->CFGR &= ~(RCC_CFGR_PLLMUL | RCC_CFGR_PLLSRC);
//     RCC->CFGR |= RCC_CFGR_PLLMUL12;
    
//     // 3. Enable PLL and wait for it to lock
//     RCC->CR |= RCC_CR_PLLON;
//     while (!(RCC->CR & RCC_CR_PLLRDY)); 
    
//     // 4. Select PLL as the system clock source and wait for switch
//     RCC->CFGR |= RCC_CFGR_SW_PLL;
//     while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); 
// }

// // Simple blocking delay for probing test
// // Global variable to count milliseconds
// volatile uint32_t ms_ticks = 0;

// // This is a hardware interrupt handler built into the ARM Cortex core.
// // It automatically fires exactly once per millisecond.
// void SysTick_Handler(void) {
//     ms_ticks++;
// }

// // Configures the hardware timer based on our 48MHz clock
// void init_systick(void) {
//     // 48,000,000 Hz / 1000 = 48,000 ticks per millisecond
//     SysTick_Config(48000000 / 1000);
// }

// // A hardware-accurate delay function
// void delay_ms(uint32_t ms) {
//     uint32_t start_time = ms_ticks;
//     while ((ms_ticks - start_time) < ms) {
//         // Wait here until the hardware timer increments enough times
//         __asm("wfi"); // Optional: "Wait For Interrupt" saves power while looping
//     }
// }

// int main(void) {
//     // Boot up to 48 MHz immediately
//     init_clock();

//     // Enable Peripheral Clock for GPIOB
//     RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

//     // Configure PB3, PB5, PB7, PB9 as General Purpose Output (Mode 01)
//     // 1. Clear the MODER bits for these specific pins
//     GPIOB->MODER &= ~((3 << (3 * 2)) | 
//                       (3 << (5 * 2)) | 
//                       (3 << (7 * 2)) | 
//                       (3 << (9 * 2)));
                      
//     // 2. Set the MODER bits to 01 (Output Mode)
//     GPIOB->MODER |=  ((1 << (3 * 2)) | 
//                       (1 << (5 * 2)) | 
//                       (1 << (7 * 2)) | 
//                       (1 << (9 * 2)));

//     // Set pins HIGH initially using the Bit Set/Reset Register
//     GPIOB->BSRR = (1 << 3) | (1 << 5) | (1 << 7) | (1 << 9);

//     while (1) {
//         // Toggle the pins so you can easily verify activity with a multimeter
//         GPIOB->ODR ^= (1 << 3) | (1 << 5) | (1 << 7) | (1 << 9);
        
//         // Wait ~500ms before toggling again
//         delay_ms(500);
//     }
// }