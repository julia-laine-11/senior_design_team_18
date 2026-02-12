// /**
// ******************************************************************************
// * @file main.c
// * @author Weili An, Niraj Menon
// * @date Feb 7, 2024
// * @brief ECE 362 Lunar lander piezo
// ******************************************************************************
// */

// /*******************************************************************************/

// // Fill out your username! Even though we're not using an autotest,
// // it should be a habit to fill out your username in this field now.
// const char* username = "ananth3";

// /*******************************************************************************/

// #include "stm32f0xx.h"
// #include <stdint.h>
// void internal_clock();



// //extern void internal_clock(void); // Clock from clock.s

// void setup_dac(void);
// void setup_button_interrupt(void);
// void generateTone(uint16_t duration_ms);
// void delay_ms(uint32_t ms);
// void test();

// int main(void) {
// internal_clock(); // System clock ~48MHz
// setup_dac(); // Setup DAC for piezo
// setup_button_interrupt(); // Setup PA0 interrupt for button
// //test();



// while (1) {
// __WFI(); // Wait for interrupt — puts CPU to sleep until interrupt
// }
// }




// void test()
// {
// RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable GPIOA
// GPIOA->MODER |= GPIO_MODER_MODER4_0; //testing for piezo without dac on pa4
// setup_button_interrupt();

// }

// void setup_dac(void) {
// RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable GPIOA
// GPIOA->MODER |= (3 << (4 * 2)); // Set PA4 to analog mode
// RCC->APB1ENR |= RCC_APB1ENR_DACEN; // Enable DAC clock
// DAC->CR &= ~DAC_CR_EN1; // Ensure DAC is off
// DAC->CR &= ~DAC_CR_TEN1; // Disable trigger
// DAC->CR |= DAC_CR_EN1; // Enable DAC
// }

// void setup_button_interrupt(void) {
// RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable GPIOB clock
// GPIOB->MODER &= ~(3 << (2 * 2));
// GPIOB->PUPDR|= (2 << (2 * 2)); // Pull-down on PB2

// // Configure EXTI line 2 for PB2
// RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; // Enable SYSCFG
// SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PB;
// EXTI->IMR |= EXTI_IMR_MR2; // Unmask EXTI0
// EXTI->RTSR |=EXTI_RTSR_TR2; // Trigger on falling edge

// NVIC_EnableIRQ(EXTI2_3_IRQn); // Enable interrupt in NVIC
// }



// void EXTI2_3_IRQHandler(void){
// if (EXTI->PR && EXTI_PR_PR2) { // Check if EXTI0 caused the interrupt
// EXTI->PR |= EXTI_PR_PR2; // Clear the pending interrupt
// generateTone(200); // Buzz piezo for 200 ms
// }
// }
// void generateTone(uint16_t duration_ms) {
// DAC->DHR8R1 = 200; //turn buzzer on
// delay_ms(duration_ms); //keep buzzer on for certain duration
// DAC->DHR8R1 = 0; //turn off

// }

// void delay_ms(uint32_t ms) {
// for (uint32_t i = 0; i < ms * 4800; i++) {
// __asm__("nop");
// }
// }







// /////code done/////////

