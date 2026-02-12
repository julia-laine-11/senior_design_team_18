// /**
//   ******************************************************************************
//   * @file    main.c
//   * @author  uppalapa
//   * @date    03/28/2025
//   * @brief   Main program body for STM32F091 driving a 32x32 RGB Matrix.
//   ******************************************************************************
// */

// // Pin Locations
// /*
// R1  PC0
// G1  PC1
// B1  PC2
// R2  PC3
// B2  PC4
// G2  PC5
// A   PC6
// B   PC7
// C   PC8
// D   PC9
// OE  PC10
// LAT PC11
// CLK PC13
// */

// //Includes
// #include "stm32f0xx.h"
// #include <stdint.h>
// #include <stddef.h>
// #include <stdlib.h>
// #include <time.h>

// // Matrix Constants
// #define MATRIX_HEIGHT       32
// #define MATRIX_WIDTH        32
// #define MATRIX_SCAN_ROWS    (MATRIX_HEIGHT / 2)

// // Color Constants Definitions
// #define COLOR_BLACK         0   // Value 0: R=0, G=0, B=0 (LEDs OFF)
// #define COLOR_RED           1   // Value 1: R=1, G=0, B=0
// #define COLOR_BLUE          2   // Value 2: R=0, G=0, B=1 (Required for White)
// #define COLOR_GREEN         4   // Value 4: R=0, G=1, B=0
// #define COLOR_YELLOW        5   //(COLOR_RED | COLOR_GREEN)               // Value 5: R=1, G=1, B=0
// #define COLOR_WHITE         7   //(COLOR_RED | COLOR_BLUE | COLOR_GREEN)  // Value 7: R=1, G=1, B=1

// //Global Variables
// volatile uint8_t canvas[MATRIX_SCAN_ROWS][MATRIX_WIDTH];
// static volatile uint8_t current_display_row = 0;

// static void nano_wait(int t) {
//     asm("       mov r0,%0\n"
//         "repeat:\n"
//         "       sub r0,#83\n"
//         "       bgt repeat\n"
//         : : "r"(t) : "r0", "cc");
// }

// // Sprite Arrays
// // Lander Sprite Dimensions & Array
// #define LANDER_HEIGHT 6
// #define LANDER_WIDTH  6
// const uint8_t lander_sprite[LANDER_HEIGHT][LANDER_WIDTH] = {
//     { 0, 0, 7, 7, 0, 0 }, // White=7
//     { 0, 7, 7, 7, 7, 0 },
//     { 0, 7, 7, 7, 7, 0 },
//     { 7, 7, 7, 7, 7, 7 },
//     { 7, 7, 7, 7, 7, 7 },
//     { 7, 7, 0, 0, 7, 7 },
// };

// #define THRUST_HEIGHT 5
// #define THRUST_WIDTH  4
// const uint8_t thrust_sprite[THRUST_HEIGHT][THRUST_WIDTH] = {
//     { 0, 5, 5, 0 }, // White=7
//     { 5, 2, 2, 5 },
//     { 5, 2, 2, 5 },
//     { 0, 5, 5, 0 },
//     { 0, 5, 5, 0 }
// };

// #define LBOOST_HEIGHT 3
// #define LBOOST_WIDTH 4
// const uint8_t lboost_sprite[LBOOST_HEIGHT][LBOOST_WIDTH] = {
//     { 0, 0, 5, 5 }, // White=7
//     { 0, 5, 2, 2 },
//     { 5, 5, 5, 5 }
// };

// #define RBOOST_HEIGHT 3
// #define RBOOST_WIDTH 4
// const uint8_t rboost_sprite[RBOOST_HEIGHT][RBOOST_WIDTH] = {
//     { 5, 5, 0, 0 }, // White=7
//     { 2, 2, 5, 0 },
//     { 5, 5, 5, 5 }
// };

// // Obstacle Sprite Dimensions & Array
// #define OBSTACLE_HEIGHT 10
// #define OBSTACLE_WIDTH  10
// const uint8_t obstacle_sprite[OBSTACLE_HEIGHT][OBSTACLE_WIDTH] = {
//     { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
//     { 1, 0, 0, 0, 0, 0, 0, 0, 1, 1 },
//     { 1, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
//     { 1, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
//     { 1, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
//     { 1, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
//     { 1, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
//     { 1, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
//     { 1, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
//     { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }
// };

// #define OVERT_HEIGHT 10
// #define OVERT_WIDTH  1
// const uint8_t overt_sprite[OVERT_HEIGHT][OVERT_WIDTH] = {
//     { 1 },
//     { 1 },
//     { 1 },
//     { 1 },
//     { 1 },
//     { 1 },
//     { 1 },
//     { 1 },
//     { 1 },
//     { 1 }
// };

// #define OHOR_HEIGHT 1
// #define OHOR_WIDTH  10
// const uint8_t ohor_sprite [OHOR_HEIGHT][OHOR_WIDTH] = {
//     { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
// };

// // Text Sprite Dimensions & Array
// #define FONT_HEIGHT 5
// #define FONT_WIDTH  3
// const uint8_t font_char_A[FONT_HEIGHT][FONT_WIDTH] = {
//     { 4, 4, 4 }, // Green=4
//     { 4, 0, 4 },
//     { 4, 4, 4 },
//     { 4, 0, 4 },
//     { 4, 0, 4 }
// };
// const uint8_t font_char_O[FONT_HEIGHT][FONT_WIDTH] = {
//     { 4, 4, 4 }, // Green=4
//     { 4, 0, 4 },
//     { 4, 0, 4 },
//     { 4, 0, 4 },
//     { 4, 4, 4 }
// };

// // Functions
// void init_matrix_gpio(void);
// void init_refresh_timer(void);
// void ClearScreen(uint8_t color_value);
// void SetPixel(int x, int y, uint8_t color_value);
// void DrawSprite(int start_x, int start_y, int height, int width, const uint8_t sprite_data[][width]); // Added prototype
// static void Matrix_Scan(void);

// // Main Function
// int main(void)
// {
//     /* Initialize peripherals */
//     internal_clock();
//     init_matrix_gpio();
//     init_refresh_timer();

//     /* Initialize Framebuffer */
//     ClearScreen(0); // Start with Off/Black

//     /* Enable Global Interrupts */
//     __enable_irq(); // Start the timer interrupt processing

//     // --- Animation Variables ---
//     int lander_x = 14; // Center horizontally
//     int lander_y = 16;  // Start near the top
//     //int lander_dy = 16; // Initial direction (down)

//     int lateral_boost = 2;

//     int lboost_x = lander_x - LBOOST_WIDTH;
//     int lboost_y = lander_y;

//     int rboost_x = lander_x + LANDER_WIDTH;
//     int rboost_y = lander_y;

//     int tactive = -2;

//     int thrust_x = lander_x + 1;
//     int thrust_y = lander_y - LANDER_HEIGHT;

//     int time_left = 20;
//     int score = 0;

//     srand(time(NULL));
//     int obstacle_x = rand() % 21;
//     int obstacle_y = rand() & 21; // Near bottom

//     while ((lander_x >= 0 && lander_x < 32) && (lander_y >= 0 && lander_y < 32) && (time_left >= 0))
//     {
//         lboost_x = lander_x - LBOOST_WIDTH;
//         lboost_y = lander_y;
    
//         rboost_x = lander_x + LANDER_WIDTH;
//         rboost_y = lander_y;
    
        
    
//         obstacle_x = rand() % 22;
//         obstacle_y = rand() % 22; // Near bottom
//         thrust_x = lander_x + 1;
//         thrust_y = lander_y + LANDER_HEIGHT;

//         //Screen Inits
//         ClearScreen(0x0);

//         //Initial Obstacle
//         DrawSprite(obstacle_x,obstacle_y,OBSTACLE_HEIGHT, OBSTACLE_WIDTH, obstacle_sprite);

//         //Lander
//         DrawSprite(lander_x, lander_y, LANDER_HEIGHT, LANDER_WIDTH, lander_sprite);
        
//         //Boost sprite conditionals
//         if(lateral_boost > 0) {
//             DrawSprite(lboost_x, lboost_y, LBOOST_HEIGHT, LBOOST_WIDTH, lboost_sprite);
//         }
//         if(lateral_boost < 0) {
//             DrawSprite(rboost_x, rboost_y, RBOOST_HEIGHT, RBOOST_WIDTH, rboost_sprite);
//         }
//         if(tactive != 0) {
//             DrawSprite(thrust_x, thrust_y, THRUST_HEIGHT, THRUST_WIDTH, thrust_sprite);
//         }

//         //Obstacle Layering Code
//         if((obstacle_x >= lander_x) && (obstacle_x <= (lander_x + 5))) {
//             DrawSprite(obstacle_x, obstacle_y, OVERT_HEIGHT, OVERT_WIDTH, overt_sprite);
//         }
//         if((obstacle_y >= (lander_y - 5)) && ((obstacle_y <= (lander_y)))) {
//             DrawSprite(obstacle_x, obstacle_y, OHOR_HEIGHT, OHOR_WIDTH, ohor_sprite);
//         }

//         if((lander_x >= obstacle_x) && ((lander_x + 5) <= (obstacle_x + 9)) && (lander_y <= obstacle_y) && ((lander_y - 5) >= (obstacle_y - 9))) {
//             obstacle_x = rand() % 22;
//             obstacle_y = rand() % 22; // Near bottom
//             time_left += 10;
//         }








//         lander_x += lateral_boost;
//         lander_y += (tactive);
//         score += 1;
//         nano_wait(200000000);
//     }
// }

// // --- Function Definitions ---

// /**
//  * @brief Initializes GPIOC pins PC0-PC12 for matrix control.
//  */
// void init_matrix_gpio(void)
// {
//     RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
//     GPIOC->MODER &= ~(0x0CFFFFFF);
//     GPIOC->MODER |= (0x4555555);
//     GPIOC->OSPEEDR &= ~(0x0CFFFFFF);
//     GPIOC->OSPEEDR |= 0x45555555;
//     GPIOC->OTYPER &= ~(0x2FFF);
//     GPIOC->PUPDR  &= ~0x03FFFFFF;
//     GPIOC->BSRR = (1U << 11); // Set OE Pin 12 High
//     GPIOC->BRR  = 0x0FFF;     // Reset Pins 0-11 Low
// }

// /**
//  * @brief Initializes TIM6 to generate an update interrupt at ~1600Hz.
//  */
// void init_refresh_timer(void) {
//     RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
//     // Assumes 48MHz SystemCoreClock
//     TIM6->PSC = 48 - 1;
//     TIM6->ARR = 625 - 1;
//     TIM6->DIER |= TIM_DIER_UIE;
//     NVIC_SetPriority(TIM6_DAC_IRQn, 1);
//     NVIC_EnableIRQ(TIM6_DAC_IRQn);
//     TIM6->CR1 |= TIM_CR1_CEN;
// }

// void ClearScreen(uint8_t color_value) {
//     color_value &= 0x07;
//     uint8_t packed_color = color_value | (color_value << 3);
//     uint32_t primask_bit = __get_PRIMASK();
//     __disable_irq();
//     for (int r = 0; r < MATRIX_SCAN_ROWS; r++) {
//         for (int c = 0; c < MATRIX_WIDTH; c++) {
//             canvas[r][c] = packed_color;
//         }
//     }
//     if (!primask_bit) {
//         __enable_irq();
//     }
// }

// /**
//  * @brief Sets the color of a single pixel in the framebuffer.
//  * @param x: Column coordinate (0 to MATRIX_WIDTH - 1)
//  * @param y: Row coordinate (0 to MATRIX_HEIGHT - 1)
//  * @param color_value: 3-bit color value (0-7) following RBG format.
//  */
// void SetPixel(int x, int y, uint8_t color_value) {
//     // Bounds check
//     if (x < 0 || x >= MATRIX_WIDTH || y < 0 || y >= MATRIX_HEIGHT) {
//         return;
//     }

//     uint8_t row_index = y % MATRIX_SCAN_ROWS; // Canvas row (0-15)
//     uint8_t col_index = x;
//     color_value &= 0x7; // Ensure 3-bit color

//     // --- Critical Section Start ---
//     uint32_t primask_bit = __get_PRIMASK();
//     __disable_irq();

//     // Read the current packed byte, modify only the relevant 3 bits, write back
//     uint8_t current_packed_color = canvas[row_index][col_index];
//     uint8_t new_packed_color;

//     if (y < MATRIX_SCAN_ROWS) { // Top half (y = 0 to 15) -> Modify bits 0-2
//         new_packed_color = (current_packed_color & ~0x07) | color_value;
//     } else { // Bottom half (y = 16 to 31) -> Modify bits 3-5
//         new_packed_color = (current_packed_color & ~0x38) | (color_value << 3);
//     }
//     canvas[row_index][col_index] = new_packed_color;

//     // --- Critical Section End ---
//     if (!primask_bit) {
//         __enable_irq();
//     }
// }

// /**
//  * @brief Draws a sprite onto the canvas framebuffer.
//  * @param start_x: Top-left column coordinate for the sprite.
//  * @param start_y: Top-left row coordinate for the sprite.
//  * @param height: Height of the sprite array.
//  * @param width: Width of the sprite array.
//  * @param sprite_data: Pointer to the 2D sprite array.
//  * @note Pixels with value 0 in the sprite data are treated as transparent (not drawn).
//  */
// void DrawSprite(int start_x, int start_y, int height, int width, const uint8_t sprite_data[][width]) {
//     for (int r = 0; r < height; r++) {
//         for (int c = 0; c < width; c++) {
//             uint8_t pixel_color = sprite_data[r][c];
//             // Treat color 0 (black) in the sprite as transparent
//             if (pixel_color != 0) {
//                 SetPixel(start_x + c, start_y + r, pixel_color);
//             }
//         }
//     }
// }


// /**
//  * @brief TIM6 Interrupt Service Routine. Calls Matrix_Scan.
//  */
// void TIM6_DAC_IRQHandler(void) {
//     if (TIM6->SR & TIM_SR_UIF) {
//         TIM6->SR &= ~TIM_SR_UIF; // Clear flag
//         Matrix_Scan();          // Call scan function
//     }
// }

// /**
//  * @brief Sends data for one row pair to the matrix. Called by TIM6 ISR.
//  */
// static void Matrix_Scan(void) {
//     // 1. Disable output
//     // 3. Clock in Data
//     for (int col = 0; col < MATRIX_WIDTH; col++) {
//         uint8_t packed_color = canvas[current_display_row][col];
//         uint8_t rgb1 = packed_color & 0x07;        // Top row color (RBG)
//         uint8_t rgb2 = (packed_color >> 3) & 0x07; // Bottom row color (RBG)
//         uint32_t rgb_bits_set = 0, rgb_bits_reset = 0;
//         // Top Row (R1=PC0, B1=PC1, G1=PC2)
//         if (rgb1 & 1) rgb_bits_set |= (1U << 0); else rgb_bits_reset |= (1U << 0); // R1
//         if (rgb1 & 2) rgb_bits_set |= (1U << 1); else rgb_bits_reset |= (1U << 1); // B1
//         if (rgb1 & 4) rgb_bits_set |= (1U << 2); else rgb_bits_reset |= (1U << 2); // G1
//         // Bottom Row (R2=PC3, B2=PC4, G2=PC5)
//         if (rgb2 & 1) rgb_bits_set |= (1U << 3); else rgb_bits_reset |= (1U << 3); // R2
//         if (rgb2 & 2) rgb_bits_set |= (1U << 4); else rgb_bits_reset |= (1U << 4); // B2
//         if (rgb2 & 4) rgb_bits_set |= (1U << 5); else rgb_bits_reset |= (1U << 5); // G2
//         // Set/Reset all 6 RGB lines
//         GPIOC->BSRR = rgb_bits_set | (rgb_bits_reset << 16);
//         // Pulse Clock (CLK = PC10)
        
//         GPIOC->BSRR = (1U << 13); 
//         nano_wait(5000);
//         GPIOC->BRR = (1U << 13);
//         nano_wait(5000);

//     }

//     GPIOC->BSRR = (1U << 11); // OE High

//     // 4. Pulse Latch (LAT = PC11)
//     GPIOC->BSRR = (1U << 10); 
//     nano_wait(5000);
//     GPIOC->BRR = (1U << 10);
//     nano_wait(5000);

//     // 2. Set Address Lines
//     uint8_t addr = current_display_row;
//     uint32_t addr_bits_set = 0, addr_bits_reset = 0;
//     if (addr & 0x01) addr_bits_set |= (1U << 6); else addr_bits_reset |= (1U << 6); // A
//     if (addr & 0x02) addr_bits_set |= (1U << 7); else addr_bits_reset |= (1U << 7); // B
//     if (addr & 0x04) addr_bits_set |= (1U << 8); else addr_bits_reset |= (1U << 8); // C
//     if (addr & 0x08) addr_bits_set |= (1U << 9); else addr_bits_reset |= (1U << 9); // D
//     GPIOC->BSRR = addr_bits_set | (addr_bits_reset << 16);

//     // 5. Enable output
//     GPIOC->BRR = (1U << 11); // OE Low

//     // 6. Advance to the next row pair
//     current_display_row = (current_display_row + 1) % MATRIX_SCAN_ROWS;
// }

// // /**
// //  * @brief Simple blocking delay loop (approximate).
// //  * @param count: Number of loop iterations. Adjust based on clock speed.
// //  */
// // void DelayApprox(volatile uint32_t count) {
// //     while (count--) {
// //         __NOP();
// //     }
// // }