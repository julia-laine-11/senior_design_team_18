// #include "stm32f0xx.h"
// #include <stdint.h>

// #define MATRIX_WIDTH        32
// #define MATRIX_SCAN_ROWS    16

// // Color Definitions
// #define COLOR_RED   1
// #define COLOR_BLUE  4

// // Canvas Buffer
// volatile uint8_t canvas[16][32];

// // --- Sprites ---
// const uint8_t s_A[5][4] = {{0,7,7,0},{7,0,0,7},{7,7,7,7},{7,0,0,7},{7,0,0,7}};
// const uint8_t s_U[5][4] = {{7,0,0,7},{7,0,0,7},{7,0,0,7},{7,0,0,7},{0,7,7,0}};
// const uint8_t s_T[5][4] = {{7,7,7,7},{0,7,7,0},{0,7,7,0},{0,7,7,0},{0,7,7,0}};
// const uint8_t s_O[5][4] = {{0,7,7,0},{7,0,0,7},{7,0,0,7},{7,0,0,7},{0,7,7,0}};
// const uint8_t s_N[5][4] = {{7,7,0,7},{7,7,0,7},{7,7,7,7},{7,0,7,7},{7,0,0,7}};
// const uint8_t s_I[5][4] = {{7,7,7,7},{0,7,7,0},{0,7,7,0},{0,7,7,0},{7,7,7,7}};
// const uint8_t s_R[5][4] = {{7,7,7,0},{7,0,0,7},{7,7,7,0},{7,0,7,0},{7,0,0,7}};

// void SetPixel(int x, int y, uint8_t color) {
//     if (x < 0 || x >= 32 || y < 0 || y >= 32) return;
//     uint8_t row = y % 16;
//     if (y < 16) canvas[row][x] = (canvas[row][x] & ~0x07) | (color & 0x07);
//     else canvas[row][x] = (canvas[row][x] & ~0x38) | ((color & 0x07) << 3);
// }

// void DrawSprite(int x, int y, const uint8_t data[5][4], uint8_t color) {
//     for (int r = 0; r < 5; r++)
//         for (int c = 0; c < 4; c++)
//             if (data[r][c]) SetPixel(x + c, y + r, color);
// }

// // --- Stabilized Matrix Scan ---
// // Using PB3:OE, PB6:CLK, PB7:ADDR_D (LAT tied to 3.3V)
// static inline void Matrix_Scan(uint8_t row) {
//     // 1. HARD BLANKING: OE HIGH (PB3)
//     GPIOB->BSRR = (1U << 3); 
//     // Wait for the display to fully turn off to hide the transparent shift
//     for(volatile int i = 0; i < 30; i++); 

//     // 2. SHIFT DATA: (CLK on PB6)
//     for (int col = 0; col < 32; col++) {
//         uint8_t p = canvas[row][col];
        
//         // Data Pins: PA4-7 (R1,G1,B1,R2) and PC4-5 (G2,B2)
//         GPIOA->BSRR = ((p & 0xF) << 4) | ((~(p & 0xF) & 0xF) << 20);
//         GPIOC->BSRR = ((p & 0x30)) | ((~(p & 0x30) & 0x30) << 16);

//         // SLOW CLOCK: This stops the horizontal smearing/moving!
//         for(volatile int i = 0; i < 5; i++); 
//         GPIOB->BSRR = (1U << 6); // CLK HIGH
//         for(volatile int i = 0; i < 15; i++); 
//         GPIOB->BRR  = (1U << 6); // CLK LOW
//     }

//     // 3. ADDRESS UPDATE: (A:PB0, B:PB1, C:PB10, D:PB7)
//     uint32_t b_bits = 0;
//     if (row & 0x01) b_bits |= (1U << 0);
//     if (row & 0x02) b_bits |= (1U << 1);
//     if (row & 0x04) b_bits |= (1U << 10);
//     if (row & 0x08) b_bits |= (1U << 7);
    
//     // Atomically set and reset Address bits on Port B
//     GPIOB->BRR = (1U << 0) | (1U << 1) | (1U << 10) | (1U << 7);
//     GPIOB->BSRR = b_bits;

//     // Let the address lines physically settle before turning the screen back on
//     for(volatile int i = 0; i < 15; i++); 

//     // 4. DISPLAY ENABLE: OE LOW (PB3)
//     GPIOB->BRR = (1U << 3);
    
//     // 5. ROW DWELL TIME: 
//     // Increased to give a stable frame rate without strobe effects
//     for(volatile int i = 0; i < 200; i++); 
// }

// int main(void) {
//     // Enable Peripherals
//     RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
    
//     // GPIO Config
//     GPIOA->MODER |= 0x5500;
//     GPIOB->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER10_0 | 
//                      GPIO_MODER_MODER3_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | 
//                      GPIO_MODER_MODER7_0);
//     GPIOC->MODER |= 0x500;

//     // Slew Rate Control: Force LOW SPEED to prevent ringing on long cables
//     GPIOB->OSPEEDR &= ~(0xFFFFFFFF);
//     GPIOA->OSPEEDR &= ~(0xFFFFFFFF);

//     // Initial Splash Screen
//     DrawSprite(3, 4, s_A, COLOR_RED);        
//     DrawSprite(8, 4, s_U, COLOR_RED);
//     DrawSprite(13, 4, s_T, COLOR_RED);
//     DrawSprite(18, 4, s_O, COLOR_RED);
//     DrawSprite(23, 4, s_N, COLOR_RED);
    
//     DrawSprite(17, 12, s_A, COLOR_BLUE);
//     DrawSprite(22, 12, s_I, COLOR_BLUE);
//     DrawSprite(27, 12, s_R, COLOR_BLUE);

//     uint8_t current_row = 0;
//     while (1) {
//         Matrix_Scan(current_row);
//         current_row = (current_row + 1) & 0x0F; // Cycle 0-15
//     }
// }