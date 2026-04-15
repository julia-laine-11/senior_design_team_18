// #include "stm32f0xx.h"
// #include <stdint.h>

// //===========================================================================
// // DEFINITIONS
// //===========================================================================

// // UI States
// #define UI_STATE_SPLASH     0
// #define UI_STATE_MENU       1
// #define UI_STATE_PLAY       2

// // Game Modes
// #define MODE_BOT            0
// #define MODE_PLAYER         1

// //===========================================================================
// // TIMING FUNCTIONS
// //===========================================================================

// void delay_ms(uint32_t ms) {
//     for (uint32_t i = 0; i < ms; i++) {
//         SysTick->LOAD = 48000 - 1;                  
//         SysTick->VAL = 0;
//         SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
//         while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
//         SysTick->CTRL = 0; 
//     }
// }

// void small_delay(void) {
//     for(volatile int i=0; i<15; i++);
// }

// //===========================================================================
// // ADC (Joystick VRY -> PC0)
// //===========================================================================

// void init_adc(void) {
//     RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    
//     // PC0 (IN10) Analog Mode
//     GPIOC->MODER |= GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1; 
    
//     // Enable HSI14 Clock for ADC
//     RCC->CR2 |= RCC_CR2_HSI14ON;
//     while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0);
    
//     // Enable ADC
//     ADC1->CR |= ADC_CR_ADEN;
//     while ((ADC1->ISR & ADC_ISR_ADRDY) == 0);
// }

// uint16_t read_adc(uint32_t channel_mask) {
//     // Stop any ongoing conversion before changing channels
//     if (ADC1->CR & ADC_CR_ADSTART) {
//         ADC1->CR |= ADC_CR_ADSTP;
//         while (ADC1->CR & ADC_CR_ADSTP);
//     }
    
//     // Set the requested channel
//     ADC1->CHSELR = channel_mask;
    
//     // Start conversion
//     ADC1->CR |= ADC_CR_ADSTART;
//     while ((ADC1->ISR & ADC_ISR_EOC) == 0);
//     return ADC1->DR;
// }

// //===========================================================================
// // CONTROLS (PC2 Button - Active Low for HW-504 SW pin)
// //===========================================================================

// void init_controls(void) {
//     // PC2 Input (Start/Select)
//     GPIOC->MODER &= ~(GPIO_MODER_MODER2);
    
//     // Configure with an internal PULL-UP resistor (holds line at 3.3V)
//     GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR2);
//     GPIOC->PUPDR |= (GPIO_PUPDR_PUPDR2_0); 
// }

// //===========================================================================
// // OLED PINS & SPI Bit-Banging (SEH1602A)
// //===========================================================================

// void init_oled_pins(void) {
//     // SCL = PA15 
//     GPIOA->MODER &= ~(GPIO_MODER_MODER15);
//     GPIOA->MODER |= (GPIO_MODER_MODER15_0);

//     // SDI = PC10 | nCS = PC11 
//     GPIOC->MODER &= ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11);
//     GPIOC->MODER |= (GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0);

//     // Initialize nCS High (Deselected), SCL Low
//     GPIOC->BSRR = (1U << 11); // nCS = 1
//     GPIOA->BRR = (1U << 15);  // SCL = 0
// }

// void spi_send_10bit(uint16_t data) {
//     GPIOC->BRR = (1U << 11); // Pull nCS Low (Active)
//     small_delay();

//     for (int i = 9; i >= 0; i--) {
//         if ((data >> i) & 1) GPIOC->BSRR = (1U << 10);
//         else                 GPIOC->BRR = (1U << 10);
//         small_delay();
        
//         GPIOA->BSRR = (1U << 15); // Clock High
//         small_delay();
//         GPIOA->BRR = (1U << 15);  // Clock Low
//         small_delay();
//     }

//     GPIOC->BSRR = (1U << 11); // Push nCS High (Deselected)
//     small_delay();
// }

// void spi_cmd(unsigned int data) { 
//     spi_send_10bit(data & 0xFF); 
//     delay_ms(1); // Give the slow OLED logic time to process
// }

// void spi_data(unsigned int data) { 
//     spi_send_10bit(data | 0x200); 
//     delay_ms(1); 
// }

// void spi1_init_oled(void) {
//     // CRITICAL: Wait for OLED's internal logic to power up
//     delay_ms(100); 

//     spi_cmd(0x38); // Function set: 8-bit interface, 2 lines
//     spi_cmd(0x08); // Display off
//     spi_cmd(0x17); // 3V internal DC/DC Power ON (Crucial for SEH1602A)
//     spi_cmd(0x01); // Clear display
//     delay_ms(5);   // Clear command takes extra time
//     spi_cmd(0x06); // Entry mode
//     spi_cmd(0x02); // Cursor home
//     spi_cmd(0x0C); // Display ON, Cursor OFF
// }

// void spi1_display1(const char *string) {
//     spi_cmd(0x02); // Move cursor to home (Line 1)
//     while(*string != '\0') {
//         spi_data(*string);
//         string++;
//     }
// }

// void spi1_display2(const char *string) {
//     spi_cmd(0xC0); // Move cursor to start of Line 2
//     while(*string != '\0') {
//         spi_data(*string);
//         string++;
//     }
// }

// //===========================================================================
// // MAIN
// //===========================================================================

// int main(void) {
//     // 1. Enable Clocks for Port A and C
//     RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN;

//     // 2. Initialize Hardware
//     init_oled_pins();
//     init_adc();
//     init_controls();

//     // 3. Initialize OLED
//     spi1_init_oled();

//     // 4. UI Variables
//     uint8_t ui_state = UI_STATE_SPLASH;
//     uint8_t selected_mode = MODE_BOT; 
//     uint8_t oled_needs_update = 1;
//     uint32_t pc2_debounce = 0;

//     // 5. Main Loop
//     while (1) {
        
//         // --- Input Handling ---
//         if (pc2_debounce > 0) pc2_debounce--;

//         // Read Y-Axis on PC0 (CH10)
//         uint16_t joy_y = read_adc(ADC_CHSELR_CHSEL10);
        
//         // Joystick Menu Navigation (Y-Axis)
//         if (ui_state == UI_STATE_MENU) {
//             // SWAPPED LOGIC: < 1000 sets to PLAYER, > 3000 sets to BOT
//             if (joy_y < 1000 && selected_mode != MODE_PLAYER) {
//                 selected_mode = MODE_PLAYER;
//                 oled_needs_update = 1;
//             } else if (joy_y > 3000 && selected_mode != MODE_BOT) {
//                 selected_mode = MODE_BOT;
//                 oled_needs_update = 1;
//             }
//         }

//         // Button Press (PC2 - Active Low)
//         // NOTICE the "!" - Checks if the line drops to Ground when pressed
//         if (!(GPIOC->IDR & (1 << 2)) && (pc2_debounce == 0)) {
//             pc2_debounce = 100000; // Software debounce delay
            
//             if (ui_state == UI_STATE_SPLASH) {
//                 ui_state = UI_STATE_MENU;
//                 oled_needs_update = 1;
//             } else if (ui_state == UI_STATE_MENU) {
//                 ui_state = UI_STATE_PLAY;
//                 oled_needs_update = 1;
//             } else if (ui_state == UI_STATE_PLAY) {
//                 ui_state = UI_STATE_SPLASH; 
//                 selected_mode = MODE_BOT;
//                 oled_needs_update = 1;
//             }
//         }

//         // --- Screen Rendering ---
//         if (oled_needs_update) {
//             oled_needs_update = 0;
            
//             spi_cmd(0x01); 
//             delay_ms(2); 

//             if (ui_state == UI_STATE_SPLASH) {
//                 spi1_display1("Push Button     ");
//                 spi1_display2("to start...     ");
//             } 
//             else if (ui_state == UI_STATE_MENU) {
//                 if (selected_mode == MODE_PLAYER) {
//                     spi1_display1("> Human         ");
//                     spi1_display2("  Bot           ");
//                 } else {
//                     spi1_display1("  Human         ");
//                     spi1_display2("> Bot           ");
//                 }
//             } 
//             else if (ui_state == UI_STATE_PLAY) {
//                 if (selected_mode == MODE_PLAYER) {
//                     spi1_display1("Playing:        ");
//                     spi1_display2("Human Mode      ");
//                 } else {
//                     spi1_display1("Playing:        ");
//                     spi1_display2("Bot Mode        ");
//                 }
//             }
//         }
//     }
// }