// #include "stm32f0xx.h"
// #include <stdint.h>
// #include <stdbool.h>

// //===========================================================================
// // TEST CONFIGURATION
// //===========================================================================
// #define MODE_BOTH       0
// #define MODE_BLUE_ONLY  1
// #define MODE_RED_ONLY   2

// // Change this to isolate sensors: MODE_BOTH, MODE_BLUE_ONLY, or MODE_RED_ONLY
// #define DISPLAY_MODE    MODE_BOTH

// #define SYSTEM_CLOCK        48000000 
// #define MATRIX_HEIGHT       32
// #define MATRIX_WIDTH        32
// #define MATRIX_SCAN_ROWS    16
// #define COLOR_RED   1
// #define COLOR_BLUE  4
// #define NUM_HEIGHT 14
// #define NUM_WIDTH 10

// // Display Buffer
// volatile uint8_t canvas[16][32];
// static volatile uint8_t current_display_row = 0;
// volatile uint32_t sys_ticks = 0;

// //===========================================================================
// // SYSTEM UTILITIES
// //===========================================================================

// void init_clock(void) {
//     FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;
//     RCC->CFGR &= ~(RCC_CFGR_PLLMUL | RCC_CFGR_PLLSRC);
//     RCC->CFGR |= RCC_CFGR_PLLMUL12;
//     RCC->CR |= RCC_CR_PLLON;
//     while (!(RCC->CR & RCC_CR_PLLRDY)); 
//     RCC->CFGR |= RCC_CFGR_SW_PLL;
//     while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); 
// }

// void SysTick_Handler(void) { sys_ticks++; }

// //===========================================================================
// // TRUE HUB75 MATRIX DRIVE
// //===========================================================================
// // Data: PA4(R1), PA5(G1), PA6(B1), PA7(R2), PC4(G2), PC5(B2)
// // Control: PB12(CLK), PB13(LAT), PB14(OE)
// // Address: PB0(A), PB1(B), PB10(C), PB11(D)

// static inline void Matrix_Scan(uint8_t row) {
//     // 1. SHIFT DATA IN BACKGROUND
//     // Because LAT is now properly wired, we can shift the NEXT row's data 
//     // into the chip while the CURRENT row is actively shining on the screen!
//     for (int col = 0; col < 32; col++) {
//         uint8_t p = canvas[row][col];
        
//         GPIOA->BSRR = ((p & 0xF) << 4) | ((~(p & 0xF) & 0xF) << 20);
//         GPIOC->BSRR = ((p & 0x30)) | ((~(p & 0x30) & 0x30) << 16);

//         // Clock Pulse (PB12)
//         GPIOB->BSRR = (1U << 12); 
//         GPIOB->BRR  = (1U << 12); 
//     }

//     // 2. BLANK SCREEN: OE HIGH (PB14)
//     // Turn off the screen so we can safely change rows without ghosting
//     GPIOB->BSRR = (1U << 14); 

//     // 3. LATCH DATA: LAT HIGH then LOW (PB13)
//     // This locks the 32 bits we just shifted into the output registers
//     GPIOB->BSRR = (1U << 13);
//     for(volatile int i = 0; i < 2; i++); // Tiny delay for Latch lock
//     GPIOB->BRR  = (1U << 13);

//     // 4. UPDATE ADDRESS: A:PB0, B:PB1, C:PB10, D:PB11
//     uint32_t b_set = 0;
//     if (row & 0x01) b_set |= (1U << 0);
//     if (row & 0x02) b_set |= (1U << 1);
//     if (row & 0x04) b_set |= (1U << 10);
//     if (row & 0x08) b_set |= (1U << 11);
    
//     GPIOB->BRR = (1U << 0) | (1U << 1) | (1U << 10) | (1U << 11);
//     GPIOB->BSRR = b_set;

//     // 5. DISPLAY ENABLE: OE LOW (PB14)
//     // Turn the screen back on to reveal the new row!
//     GPIOB->BRR = (1U << 14);
    
//     // 6. ROW DWELL TIME (Controls Brightness)
//     for(volatile int i = 0; i < 400; i++); 
// }

// //===========================================================================
// // SPRITES & GRAPHICS
// //===========================================================================

// const uint8_t sprite_0[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,0,0}};
// const uint8_t sprite_1[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,0,0,0,0,0,0}};

// void SetPixel(int x, int y, uint8_t color) {
//     if (x < 0 || x >= 32 || y < 0 || y >= 32) return;
//     uint8_t r = y % 16;
//     if (y < 16) canvas[r][x] = (canvas[r][x] & ~0x07) | (color & 0x07);
//     else canvas[r][x] = (canvas[r][x] & ~0x38) | ((color & 0x07) << 3);
// }

// void ClearScreen(void) {
//     for (int r = 0; r < 16; r++) {
//         for (int c = 0; c < 32; c++) canvas[r][c] = 0;
//     }
// }

// void DrawSprite(int x, int y, const uint8_t data[NUM_HEIGHT][NUM_WIDTH], uint8_t color) {
//     for (int r = 0; r < NUM_HEIGHT; r++) {
//         for (int c = 0; c < NUM_WIDTH; c++) {
//             if (data[r][c] != 0) SetPixel(x + c, y + r, color);
//         }
//     }
// }

// //===========================================================================
// // INITIALIZATION
// //===========================================================================

// void init_hardware(void) {
//     RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;

//     // DATA PA4-7
//     GPIOA->MODER &= ~(0xFF00);
//     GPIOA->MODER |= 0x5500;
    
//     // DATA PC4-5
//     GPIOC->MODER &= ~(0xF00);
//     GPIOC->MODER |= 0x500;

//     // PORT B: ADDR A(0), B(1), C(10), D(11), CLK(12), LAT(13), OE(14)
//     GPIOB->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER10 | 
//                       GPIO_MODER_MODER11 | GPIO_MODER_MODER12 | GPIO_MODER_MODER13 | GPIO_MODER_MODER14);
//     GPIOB->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER10_0 | 
//                      GPIO_MODER_MODER11_0 | GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0);

//     // Initial Safe States
//     GPIOB->BSRR = (1U << 14); // Start with screen Blanked (OE High)
//     GPIOB->BRR  = (1U << 13); // Start with Latch Low
//     GPIOB->BRR  = (1U << 12); // Start with Clock Low

//     // SENSORS: PA11 (Red), PA12 (Blue) - Inputs with Pull-up
//     GPIOA->MODER &= ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER12);
//     GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR11_0 | GPIO_PUPDR_PUPDR12_0);
// }

// //===========================================================================
// // MAIN LOOP
// //===========================================================================

// int main(void) {
//     init_clock();
//     SysTick_Config(48000); // 1ms tick at 48MHz
//     init_hardware();

//     uint32_t player_underline_timeout = 0;
//     uint32_t bot_underline_timeout = 0;

//     while (1) {
//         // Poll Pins: Active low, 0V means beam broken
//         bool player_is_broken = (GPIOA->IDR & (1 << 12)) == 0; 
//         bool bot_is_broken    = (GPIOA->IDR & (1 << 11)) == 0; 

//         // Schedule underlines
//         if (player_is_broken) player_underline_timeout = sys_ticks + 250; 
//         if (bot_is_broken)    bot_underline_timeout = sys_ticks + 250;

//         ClearScreen();

//         // LEFT DIGIT: BLUE / PLAYER
//         if (DISPLAY_MODE == MODE_BOTH || DISPLAY_MODE == MODE_BLUE_ONLY) {
//             DrawSprite(2, 9, player_is_broken ? sprite_0 : sprite_1, COLOR_BLUE);
//             if (sys_ticks < player_underline_timeout) {
//                 for (int i = 0; i < NUM_WIDTH; i++) SetPixel(2 + i, 24, COLOR_BLUE);
//             }
//         }

//         // RIGHT DIGIT: RED / BOT
//         if (DISPLAY_MODE == MODE_BOTH || DISPLAY_MODE == MODE_RED_ONLY) {
//             DrawSprite(18, 9, bot_is_broken ? sprite_0 : sprite_1, COLOR_RED);
//             if (sys_ticks < bot_underline_timeout) {
//                 for (int i = 0; i < NUM_WIDTH; i++) SetPixel(18 + i, 24, COLOR_RED);
//             }
//         }

//         // Drive the matrix free-running
//         Matrix_Scan(current_display_row);
//         current_display_row = (current_display_row + 1) & 0x0F;
//     }
// }