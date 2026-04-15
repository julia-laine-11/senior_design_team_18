// #include "stm32f0xx.h"
// #include <stdint.h>
// #include <stdbool.h>

// #define MATRIX_HEIGHT       32
// #define MATRIX_WIDTH        32
// #define MATRIX_SCAN_ROWS    16

// #define COLOR_RED   1
// #define COLOR_BLUE  4

// #define NUM_HEIGHT 14
// #define NUM_WIDTH 10

// volatile uint8_t canvas[MATRIX_SCAN_ROWS][MATRIX_WIDTH];
// static volatile uint8_t current_display_row = 0;

// const uint8_t sprite_0[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,0,0}};
// const uint8_t sprite_1[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,0,0,0,0,0,0}};

// void SetPixel(int x, int y, uint8_t color) {
//     if (x < 0 || x >= MATRIX_WIDTH || y < 0 || y >= MATRIX_HEIGHT) return;
//     uint8_t row = y % 16;
//     if (y < 16) canvas[row][x] = (canvas[row][x] & ~0x07) | (color & 0x07);
//     else canvas[row][x] = (canvas[row][x] & ~0x38) | ((color & 0x07) << 3);
// }

// void ClearScreen(void) {
//     for (int r = 0; r < MATRIX_SCAN_ROWS; r++) {
//         for (int c = 0; c < MATRIX_WIDTH; c++) canvas[r][c] = 0;
//     }
// }

// void DrawSprite(int x, int y, int height, int width, const uint8_t sprite_data[][width], uint8_t color) {
//     for (int r = 0; r < height; r++) {
//         for (int c = 0; c < width; c++) {
//             if (sprite_data[r][c] != 0) SetPixel(x + c, y + r, color);
//         }
//     }
// }

// static inline void Matrix_Scan(uint8_t row) {
//     // 1. HARD BLANKING
//     GPIOB->BSRR = (1U << 3); 
//     for(volatile int i = 0; i < 10; i++); 

//     // 2. SHIFT DATA
//     for (int col = 0; col < 32; col++) {
//         uint8_t p = canvas[row][col];
//         GPIOA->BSRR = ((p & 0xF) << 4) | ((~(p & 0xF) & 0xF) << 20);
//         GPIOC->BSRR = ((p & 0x30)) | ((~(p & 0x30) & 0x30) << 16);

//         for(volatile int i = 0; i < 2; i++); 
//         GPIOB->BSRR = (1U << 6); 
//         for(volatile int i = 0; i < 8; i++); 
//         GPIOB->BRR  = (1U << 6); 
//     }

//     // 3. ADDRESS UPDATE
//     uint32_t b_bits = 0;
//     if (row & 0x01) b_bits |= (1U << 0);
//     if (row & 0x02) b_bits |= (1U << 1);
//     if (row & 0x04) b_bits |= (1U << 10);
//     if (row & 0x08) b_bits |= (1U << 7); 
    
//     GPIOB->BRR = (1U << 0) | (1U << 1) | (1U << 10) | (1U << 7);
//     GPIOB->BSRR = b_bits;

//     for(volatile int i = 0; i < 5; i++); 

//     // 4. DISPLAY ENABLE
//     GPIOB->BRR = (1U << 3);
    
//     // 5. ROW DWELL TIME
//     for(volatile int i = 0; i < 50; i++); 
// }

// void delay_ms(uint32_t ms) {
//     for (uint32_t i = 0; i < ms; i++) {
//         SysTick->LOAD = 48000 - 1;                  
//         SysTick->VAL = 0;
//         SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
        
//         while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) {
//             Matrix_Scan(current_display_row);
//             current_display_row = (current_display_row + 1) & 0x0F;
//         }
//         SysTick->CTRL = 0;  
//     }
// }

// void init_matrix_gpio(void) {
//     RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;

//     GPIOA->MODER &= ~(0xFF00);
//     GPIOA->MODER |= 0x5500; 
//     GPIOC->MODER &= ~(0xF00);
//     GPIOC->MODER |= 0x500;
    
//     GPIOB->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER3 | 
//                       GPIO_MODER_MODER6 | GPIO_MODER_MODER7 | GPIO_MODER_MODER10);
//     GPIOB->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER3_0 | 
//                      GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER10_0);

//     GPIOA->OSPEEDR &= ~(0xFFFFFFFF);
//     GPIOB->OSPEEDR &= ~(0xFFFFFFFF);
//     GPIOC->OSPEEDR &= ~(0xFFFFFFFF);
    
//     GPIOB->BSRR = (1U << 3); // OE HIGH
//     GPIOB->BRR  = (1U << 6); // CLK LOW
// }

// void init_sensors(void) {
//     GPIOA->MODER &= ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER12);
//     GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR11 | GPIO_PUPDR_PUPDR12);
//     GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR11_1 | GPIO_PUPDR_PUPDR12_1);
// }

// int main(void) {
//     init_matrix_gpio();
//     init_sensors();

//     // Timers for the 500ms visual hold
//     int player_hold_frames = 0;
//     int bot_hold_frames = 0;

//     while (1) {
//         // Read pins. Since they are active low, 0V means the beam is broken.
//         bool player_is_broken = (GPIOA->IDR & (1 << 12)) == 0; // PA12 (Blue)
//         bool bot_is_broken    = (GPIOA->IDR & (1 << 11)) == 0; // PA11 (Red)

//         // If a puck breaks the beam, lock in the 500ms visual hold
//         if (player_is_broken) player_hold_frames = 31; // 31 frames * 16ms = ~496ms
//         if (bot_is_broken)    bot_hold_frames = 31;

//         ClearScreen();

//         // --- Left Digit: Blue / Player ---
//         if (player_hold_frames > 0) {
//             DrawSprite(2, 9, NUM_HEIGHT, NUM_WIDTH, sprite_0, COLOR_BLUE); // Show 0
//             player_hold_frames--; // Count down the frames
//         } else {
//             DrawSprite(2, 9, NUM_HEIGHT, NUM_WIDTH, sprite_1, COLOR_BLUE); // Return to 1
//         }

//         // --- Right Digit: Red / Bot ---
//         if (bot_hold_frames > 0) {
//             DrawSprite(18, 9, NUM_HEIGHT, NUM_WIDTH, sprite_0, COLOR_RED); // Show 0
//             bot_hold_frames--; // Count down the frames
//         } else {
//             DrawSprite(18, 9, NUM_HEIGHT, NUM_WIDTH, sprite_1, COLOR_RED); // Return to 1
//         }

//         // 16ms delay powers the matrix scan
//         delay_ms(16);
//     }
// }