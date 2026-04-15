#include "stm32f0xx.h"
#include <stdint.h>
#include <stdbool.h>

//===========================================================================
// TEST CONFIGURATION PARAMETERS
//===========================================================================

#define MODE_BOTH       0
#define MODE_BLUE_ONLY  1
#define MODE_RED_ONLY   2

// ---> CHANGE THIS TO ISOLATE A SENSOR <---
#define DISPLAY_MODE    MODE_BOTH

//===========================================================================
// DEFINITIONS
//===========================================================================

#define SYSTEM_CLOCK        48000000 
#define MATRIX_HEIGHT       32
#define MATRIX_WIDTH        32
#define MATRIX_SCAN_ROWS    16

#define COLOR_RED   1
#define COLOR_BLUE  4

#define NUM_HEIGHT 14
#define NUM_WIDTH 10

volatile uint8_t canvas[MATRIX_SCAN_ROWS][MATRIX_WIDTH];
static volatile uint8_t current_display_row = 0;

// Global Millisecond Counter (Updated by hardware timer)
volatile uint32_t sys_ticks = 0;

//===========================================================================
// SYSTEM CLOCK & TIMING
//===========================================================================

// Boots the CPU to 48MHz for maximum polling speed
void init_clock(void) {
    FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;
    RCC->CFGR &= ~(RCC_CFGR_PLLMUL | RCC_CFGR_PLLSRC);
    RCC->CFGR |= RCC_CFGR_PLLMUL12;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)); 
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); 
}

// Triggers exactly once per millisecond
void SysTick_Handler(void) {
    sys_ticks++;
}

//===========================================================================
// SPRITES & GRAPHICS
//===========================================================================

const uint8_t sprite_0[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,0,0}};
const uint8_t sprite_1[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,0,0,0,0,0,0}};

void SetPixel(int x, int y, uint8_t color) {
    if (x < 0 || x >= MATRIX_WIDTH || y < 0 || y >= MATRIX_HEIGHT) return;
    uint8_t row = y % 16;
    if (y < 16) canvas[row][x] = (canvas[row][x] & ~0x07) | (color & 0x07);
    else canvas[row][x] = (canvas[row][x] & ~0x38) | ((color & 0x07) << 3);
}

void ClearScreen(void) {
    for (int r = 0; r < MATRIX_SCAN_ROWS; r++) {
        for (int c = 0; c < MATRIX_WIDTH; c++) canvas[r][c] = 0;
    }
}

void DrawSprite(int x, int y, int height, int width, const uint8_t sprite_data[][width], uint8_t color) {
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            if (sprite_data[r][c] != 0) SetPixel(x + c, y + r, color);
        }
    }
}

//===========================================================================
// STABILIZED MATRIX SCAN
//===========================================================================

static inline void Matrix_Scan(uint8_t row) {
    GPIOB->BSRR = (1U << 3); 
    for(volatile int i = 0; i < 10; i++); 

    for (int col = 0; col < 32; col++) {
        uint8_t p = canvas[row][col];
        GPIOA->BSRR = ((p & 0xF) << 4) | ((~(p & 0xF) & 0xF) << 20);
        GPIOC->BSRR = ((p & 0x30)) | ((~(p & 0x30) & 0x30) << 16);

        for(volatile int i = 0; i < 2; i++); 
        GPIOB->BSRR = (1U << 6); 
        for(volatile int i = 0; i < 8; i++); 
        GPIOB->BRR  = (1U << 6); 
    }

    uint32_t b_bits = 0;
    if (row & 0x01) b_bits |= (1U << 0);
    if (row & 0x02) b_bits |= (1U << 1);
    if (row & 0x04) b_bits |= (1U << 10);
    if (row & 0x08) b_bits |= (1U << 7); 
    
    GPIOB->BRR = (1U << 0) | (1U << 1) | (1U << 10) | (1U << 7);
    GPIOB->BSRR = b_bits;

    for(volatile int i = 0; i < 5; i++); 

    GPIOB->BRR = (1U << 3);
    for(volatile int i = 0; i < 50; i++); 
}

//===========================================================================
// HARDWARE SETUP
//===========================================================================

void init_matrix_gpio(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;

    GPIOA->MODER &= ~(0xFF00);
    GPIOA->MODER |= 0x5500; 
    GPIOC->MODER &= ~(0xF00);
    GPIOC->MODER |= 0x500;
    
    GPIOB->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER3 | 
                      GPIO_MODER_MODER6 | GPIO_MODER_MODER7 | GPIO_MODER_MODER10);
    GPIOB->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER3_0 | 
                     GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER10_0);

    GPIOA->OSPEEDR &= ~(0xFFFFFFFF);
    GPIOB->OSPEEDR &= ~(0xFFFFFFFF);
    GPIOC->OSPEEDR &= ~(0xFFFFFFFF);
    
    GPIOB->BSRR = (1U << 3); // OE HIGH
    GPIOB->BRR  = (1U << 6); // CLK LOW
}

void init_sensors(void) {
    GPIOA->MODER &= ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER12);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR11 | GPIO_PUPDR_PUPDR12);
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR11_1 | GPIO_PUPDR_PUPDR12_1);
}

//===========================================================================
// MAIN LOOP
//===========================================================================

int main(void) {
    // 1. Boot up to max speed
    init_clock();
    
    // 2. Start the Hardware Millisecond Timer (48000 ticks = 1ms at 48MHz)
    SysTick_Config(48000); 

    init_matrix_gpio();
    init_sensors();

    // Timers for the 250ms underline hold
    uint32_t player_underline_timeout = 0;
    uint32_t bot_underline_timeout = 0;

    while (1) {
        // --- 1. INSTANTANEOUS POLLING ---
        // This runs instantly with ZERO blocking delays. 
        bool player_is_broken = (GPIOA->IDR & (1 << 12)) == 0; // PA12 (Blue)
        bool bot_is_broken    = (GPIOA->IDR & (1 << 11)) == 0; // PA11 (Red)

        // If a puck breaks the beam, schedule the underline to disappear 250ms from NOW
        if (player_is_broken) player_underline_timeout = sys_ticks + 250; 
        if (bot_is_broken)    bot_underline_timeout = sys_ticks + 250;

        // --- 2. RENDER GRAPHICS TO BUFFER ---
        ClearScreen();

        // LEFT DIGIT: BLUE / PLAYER
        if (DISPLAY_MODE == MODE_BOTH || DISPLAY_MODE == MODE_BLUE_ONLY) {
            if (player_is_broken) {
                DrawSprite(2, 9, NUM_HEIGHT, NUM_WIDTH, sprite_0, COLOR_BLUE);
            } else {
                DrawSprite(2, 9, NUM_HEIGHT, NUM_WIDTH, sprite_1, COLOR_BLUE);
            }
            
            // Check true hardware time to see if we should still draw the underline
            if (sys_ticks < player_underline_timeout) {
                for (int i = 0; i < NUM_WIDTH; i++) {
                    SetPixel(2 + i, 24, COLOR_BLUE); 
                }
            }
        }

        // RIGHT DIGIT: RED / BOT
        if (DISPLAY_MODE == MODE_BOTH || DISPLAY_MODE == MODE_RED_ONLY) {
            if (bot_is_broken) {
                DrawSprite(18, 9, NUM_HEIGHT, NUM_WIDTH, sprite_0, COLOR_RED);
            } else {
                DrawSprite(18, 9, NUM_HEIGHT, NUM_WIDTH, sprite_1, COLOR_RED);
            }
            
            // Check true hardware time to see if we should still draw the underline
            if (sys_ticks < bot_underline_timeout) {
                for (int i = 0; i < NUM_WIDTH; i++) {
                    SetPixel(18 + i, 24, COLOR_RED);
                }
            }
        }

        // --- 3. DRIVE THE MATRIX ---
        // Instead of doing 16 rows at once and pausing for 16ms, 
        // we push exactly ONE row to the matrix and immediately loop back to check the sensors.
        Matrix_Scan(current_display_row);
        current_display_row = (current_display_row + 1) & 0x0F;
    }
}