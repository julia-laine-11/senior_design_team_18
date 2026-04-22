// #include "stm32f0xx.h"
// #include <stdint.h>
// #include <stdlib.h> // For abs()

// #define TEST_MODE 1
// #define WATCHDOG_MAX 300000  // Scaled up 6x for 48MHz
// #define SYSTEM_CLOCK 48000000 

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

// void set_motor_a(uint32_t percent, uint8_t is_rev) {
//     if (percent == 0) {
//         TIM1->CCR2 = 0; 
//         TIM1->EGR |= TIM_EGR_UG;
//         return;
//     }
//     if (is_rev) GPIOA->BRR = (1 << 8);  
//     else        GPIOA->BSRR = (1 << 8); 

//     uint32_t target_hz = percent * 2000; 
//     uint32_t arr_val = (SYSTEM_CLOCK / target_hz) - 1;
//     TIM1->ARR = arr_val;
//     TIM1->CCR2 = (arr_val + 1) / 2; 
//     TIM1->EGR |= TIM_EGR_UG; 
// }

// void set_motor_b(uint32_t percent, uint8_t is_rev) {
//     if (percent == 0) {
//         TIM3->CCR3 = 0; 
//         TIM3->EGR |= TIM_EGR_UG;
//         return;
//     }
//     if (is_rev) GPIOC->BRR = (1 << 7);  
//     else        GPIOC->BSRR = (1 << 7); 

//     uint32_t target_hz = percent * 2000; 
//     uint32_t arr_val = (SYSTEM_CLOCK / target_hz) - 1;
//     TIM3->ARR = arr_val;
//     TIM3->CCR3 = (arr_val + 1) / 2; 
//     TIM3->EGR |= TIM_EGR_UG; 
// }

// void init_uart(void) {
//     // Enable GPIOC (TX) and GPIOD (RX)
//     RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN;
//     RCC->APB1ENR |= RCC_APB1ENR_USART5EN; 

//     // Setup PD2 as RX (Alternate Function 2)
//     GPIOD->MODER &= ~GPIO_MODER_MODER2;
//     GPIOD->MODER |= GPIO_MODER_MODER2_1;        
//     GPIOD->AFR[0] &= ~(0xF << (2 * 4));         
//     GPIOD->AFR[0] |= (2 << (2 * 4));            

//     // Setup PC12 as TX (Alternate Function 2)
//     GPIOC->MODER &= ~GPIO_MODER_MODER12;
//     GPIOC->MODER |= GPIO_MODER_MODER12_1;       
//     GPIOC->AFR[1] &= ~(0xF << ((12 - 8) * 4));  
//     GPIOC->AFR[1] |= (2 << ((12 - 8) * 4));     

//     // Dynamically calculate Baud Rate based on new 48MHz SYSCLK
//     USART5->BRR = SYSTEM_CLOCK / 115200;            
    
//     // Enable Receiver (RE), Transmitter (TE), and UART (UE)
//     USART5->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE; 
// }

// // Packages the counter into our 2-Byte Protocol and sends it to Python
// void send_counter(int32_t count) {
//     // Byte 1: Control (MSB = 1). Use Bit 6 to flag negative numbers.
//     uint8_t byte1 = 0x80; 
//     if (count < 0) {
//         byte1 |= 0x40; 
//     }
    
//     // Byte 2: Payload (MSB = 0). Send the absolute value.
//     uint8_t byte2 = abs(count) & 0x7F;

//     // Transmit Byte 1
//     while (!(USART5->ISR & USART_ISR_TXE)); // Wait for Transmit Data Register to be empty
//     USART5->TDR = byte1;

//     // Transmit Byte 2
//     while (!(USART5->ISR & USART_ISR_TXE));
//     USART5->TDR = byte2;
// }

// int main(void) {
//     // Boot up to 48 MHz immediately
//     init_clock();

//     // Enable Peripheral Clocks
//     RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
//     RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
//     RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    
//     init_uart();

//     // Motor A Pins 
//     GPIOA->MODER &= ~((3 << 16) | (3 << 18));
//     GPIOA->MODER |= (1 << 16) | (2 << 18); 
//     GPIOA->AFR[1] |= (2 << 4);     
//     GPIOC->MODER &= ~(3 << 18);
//     GPIOC->MODER |= (1 << 18);     
//     GPIOA->BSRR = (1 << 8);        
//     GPIOC->BRR  = (1 << 9);        

//     // Motor B Pins 
//     GPIOC->MODER &= ~((3 << 12) | (3 << 14) | (3 << 16));
//     GPIOC->MODER |= (1 << 12) | (1 << 14) | (2 << 16);
//     GPIOC->AFR[1] &= ~(0xF << 0);  
//     GPIOC->BSRR = (1 << 7);        
//     GPIOC->BRR  = (1 << 6);        

//     // Setup Buttons (PA0 and PB2)
//     GPIOA->MODER &= ~(3 << 0);     
//     GPIOA->PUPDR |= (2 << 0);      
//     GPIOB->MODER &= ~(3 << 4);     
//     GPIOB->PUPDR |= (2 << 4);  

//     // Init Timers
//     TIM1->PSC = 0;
//     TIM1->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC1PE;
//     TIM1->CCER  |= TIM_CCER_CC2E;
//     TIM1->BDTR  |= TIM_BDTR_MOE;   
//     TIM1->CR1   |= TIM_CR1_CEN;

//     TIM3->PSC = 0;
//     TIM3->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
//     TIM3->CCER  |= TIM_CCER_CC3E;  
//     TIM3->CR1   |= TIM_CR1_CEN;

//     uint8_t pending_motor = 0;
//     uint8_t pending_dir = 0;
//     uint32_t watchdog_timer = 0;
    
//     // Game State / Counter Variables
//     int32_t game_counter = 0;
//     uint32_t pa0_debounce = 0;
//     uint32_t pb2_debounce = 0;

//     while (1) {
//         // --- 1. Handle UART RX (From Camera/Python) ---
//         if (USART5->ISR & USART_ISR_RXNE) {
//             uint8_t rx = USART5->RDR;
//             if (rx & 0x80) { 
//                 pending_motor = (rx >> 6) & 0x01;
//                 pending_dir   = (rx >> 5) & 0x01;
//             } else { 
//                 uint8_t percent = rx & 0x7F; 
//                 if (percent <= 100) { 
//                     if (pending_motor == 0) set_motor_a(percent, pending_dir);
//                     else                    set_motor_b(percent, pending_dir);
                    
//                     if (TEST_MODE == 0) watchdog_timer = 0;
//                 }
//             }
//         } 
//         else if (TEST_MODE == 0) {
//             watchdog_timer++;
//             if (watchdog_timer > WATCHDOG_MAX) {
//                 set_motor_a(0, 0);
//                 set_motor_b(0, 0);
//                 watchdog_timer = WATCHDOG_MAX; 
//             }
//         }

//         // --- 2. Handle Non-Blocking Buttons (To Python) ---
//         // Cool down the debounce timers every loop
//         if (pa0_debounce > 0) pa0_debounce--;
//         if (pb2_debounce > 0) pb2_debounce--;

//         // PA0 (+1)
//         // if ((GPIOA->IDR & (1 << 0)) && (pa0_debounce == 0)) {
//         //     game_counter++;
//         //     send_counter(game_counter);
//         //     pa0_debounce = 300000; // Scaled up 6x for 48MHz
//         // }

//         // // PB2 (-1)
//         // if ((GPIOB->IDR & (1 << 2)) && (pb2_debounce == 0)) {
//         //     game_counter--;
//         //     send_counter(game_counter);
//         //     pb2_debounce = 300000; // Scaled up 6x for 48MHz
//         // }
//     }
// }