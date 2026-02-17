#include "stm32f0xx.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

//===========================================================================
// DEFINITIONS
//===========================================================================

// Matrix
#define MATRIX_HEIGHT       32
#define MATRIX_WIDTH        32
#define MATRIX_SCAN_ROWS    (MATRIX_HEIGHT / 2)

// Game
#define WINNING_SCORE       7
#define GOAL_COOLDOWN_TICKS 50 // 2 seconds

// Sprites
#define CHAR_HEIGHT 5
#define CHAR_WIDTH 4
#define NUM_HEIGHT 14
#define NUM_WIDTH 10

//===========================================================================
// GLOBAL VARIABLES
//===========================================================================

volatile int player_score = 0;
volatile int bot_score = 0;
volatile int sensor_cooldown = 0;
volatile bool game_active = false;

// Display Buffer
volatile uint8_t canvas[MATRIX_SCAN_ROWS][MATRIX_WIDTH];
static volatile uint8_t current_display_row = 0;

//===========================================================================
// HELPER FUNCTIONS
//===========================================================================

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        SysTick->LOAD = 48000 - 1;                  // 1ms @ 48MHz
        SysTick->VAL = 0;
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
        SysTick->CTRL = 0;  // Stop SysTick
    }
}

void enable_ports(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
}

//===========================================================================
// HARDWARE SETUP
//===========================================================================

void init_start_reset(void) {
    // PA0 Input (Reset) - Pull Down
    GPIOA->MODER &= ~(GPIO_MODER_MODER0);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR0_1);

    // PB2 Input (Start) - Pull Down
    GPIOB->MODER &= ~(GPIO_MODER_MODER2);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR2);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPDR2_1);
}

void init_sensors(void) {
    // PA11, PA12 as Input
    GPIOA->MODER &= ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER12);
    
    // Reset Pull-Up/Pull-Down bits
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR11 | GPIO_PUPDR_PUPDR12);
    
    // SET PULL-UP (01 binary) instead of Pull-Down (10 binary)
    // Using _0 suffix usually targets the first bit of the pair, creating '01'
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR11_0 | GPIO_PUPDR_PUPDR12_0);
}

void init_matrix_gpio(void) {
    // Port A: R1(PA4), G1(PA5), B1(PA6), R2(PA7)
    GPIOA->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOA->MODER |= (GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0);
    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR4 | GPIO_OSPEEDR_OSPEEDR5 | GPIO_OSPEEDR_OSPEEDR6 | GPIO_OSPEEDR_OSPEEDR7);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4 | GPIO_PUPDR_PUPDR5 | GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7);

    // Port B: A(PB0), B(PB1), C(PB10), D(PB11), CLK(PB12), LAT(PB13), OE(PB14)
    GPIOB->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER10 | 
                      GPIO_MODER_MODER11 | GPIO_MODER_MODER12 | GPIO_MODER_MODER13 | GPIO_MODER_MODER14);
    GPIOB->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER10_0 | 
                     GPIO_MODER_MODER11_0 | GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0);
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR0 | GPIO_OSPEEDR_OSPEEDR1 | GPIO_OSPEEDR_OSPEEDR10 | 
                       GPIO_OSPEEDR_OSPEEDR11 | GPIO_OSPEEDR_OSPEEDR12 | GPIO_OSPEEDR_OSPEEDR13 | GPIO_OSPEEDR_OSPEEDR14);
    
    // Port C: G2(PC4), B2(PC5)
    GPIOC->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
    GPIOC->MODER |= (GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0);
    GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR4 | GPIO_OSPEEDR_OSPEEDR5);
    
    // Inits
    GPIOB->BSRR = (1U << 14); // OE High (Disable)
    GPIOB->BRR = (1U << 13);  // LAT Low
    GPIOB->BRR = (1U << 12);  // CLK Low
    
    GPIOA->BRR = (0xF0); 
    GPIOC->BRR = (0x30);
    GPIOB->BRR = (0x0C03); 
}

// Timer 6: Matrix Refresh (High Priority)
void init_refresh_timer(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = 48 - 1;   // 1MHz
    TIM6->ARR = 100 - 1;  // 100us
    TIM6->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM6_DAC_IRQn, 0); 
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    TIM6->CR1 |= TIM_CR1_CEN;
}

// Timer 14: Game Logic (Low Priority)
void setup_tim14(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    TIM14->PSC = 4800 - 1; 
    TIM14->ARR = 100 - 1; 
    TIM14->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM14_IRQn, 2);
    NVIC_EnableIRQ(TIM14_IRQn);
    TIM14->CR1 |= TIM_CR1_CEN;
}

//===========================================================================
// GRAPHICS
//===========================================================================

void SetPixel(int x, int y, uint8_t color_value) {
    if (x < 0 || x >= MATRIX_WIDTH || y < 0 || y >= MATRIX_HEIGHT) return;
    
    uint8_t row = y % MATRIX_SCAN_ROWS;
    uint8_t col = x;
    color_value &= 0x7;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint8_t current = canvas[row][col];
    if (y < MATRIX_SCAN_ROWS) {
        canvas[row][col] = (current & ~0x07) | color_value;
    } else {
        canvas[row][col] = (current & ~0x38) | (color_value << 3);
    }

    if (!primask) __enable_irq();
}

void ClearScreen(void) {
    for (int r = 0; r < MATRIX_SCAN_ROWS; r++) {
        for (int c = 0; c < MATRIX_WIDTH; c++) {
            canvas[r][c] = 0;
        }
    }
}

void DrawSprite(int start_x, int start_y, int height, int width, const uint8_t sprite_data[][width]) {
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            uint8_t pixel_color = sprite_data[r][c];
            if (pixel_color != 0) {
                SetPixel(start_x + c, start_y + r, pixel_color);
            }
        }
    }
}

//===========================================================================
// SPRITES
//===========================================================================

const uint8_t sprite_A[CHAR_HEIGHT][CHAR_WIDTH] = { {0, 7, 7, 0}, {7, 0, 0, 7}, {7, 7, 7, 7}, {7, 0, 0, 7}, {7, 0, 0, 7}};
const uint8_t sprite_U[CHAR_HEIGHT][CHAR_WIDTH] = { {7, 0, 0, 7}, {7, 0, 0, 7}, {7, 0, 0, 7}, {7, 0, 0, 7}, {0, 7, 7, 0}};
const uint8_t sprite_T[CHAR_HEIGHT][CHAR_WIDTH] = { {7, 7, 7, 7}, {0, 7, 7, 0}, {0, 7, 7, 0}, {0, 7, 7, 0}, {0, 7, 7, 0}};
const uint8_t sprite_O[CHAR_HEIGHT][CHAR_WIDTH] = { {0, 7, 7, 0}, {7, 0, 0, 7}, {7, 0, 0, 7}, {7, 0, 0, 7}, {0, 7, 7, 0}};
const uint8_t sprite_N[CHAR_HEIGHT][CHAR_WIDTH] = { {7, 7, 0, 7}, {7, 7, 0, 7}, {7, 7, 7, 7}, {7, 0, 7, 7}, {7, 0, 0, 7}};
const uint8_t sprite_I[CHAR_HEIGHT][CHAR_WIDTH] = { {7, 7, 7, 7}, {0, 7, 7, 0}, {0, 7, 7, 0}, {0, 7, 7, 0}, {7, 7, 7, 7}};
const uint8_t sprite_R[CHAR_HEIGHT][CHAR_WIDTH] = { {7, 7, 7, 0}, {7, 0, 0, 7}, {7, 7, 7, 0}, {7, 0, 7, 0}, {7, 0, 0, 7}};

// --- NUMBERS 0-9 ---

const uint8_t sprite_0[NUM_HEIGHT][NUM_WIDTH] = {
    {0,0,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},
    {0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},
    {0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},
    {0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,0,0}
};

const uint8_t sprite_1[NUM_HEIGHT][NUM_WIDTH] = {
    {0,0,0,0,0,0,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},
    {0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},
    {0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},
    {0,0,0,0,7,7,0,0,0,0},{0,0,0,0,0,0,0,0,0,0}
};

const uint8_t sprite_2[NUM_HEIGHT][NUM_WIDTH] = {
    {0,0,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},
    {0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,7,7,7,7,7,7,7,7,0},{0,7,0,0,0,0,0,0,0,0},
    {0,7,0,0,0,0,0,0,0,0},{0,7,0,0,0,0,0,0,0,0},{0,7,0,0,0,0,0,0,0,0},{0,7,0,0,0,0,0,0,0,0},
    {0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,0,0}
};

const uint8_t sprite_3[NUM_HEIGHT][NUM_WIDTH] = {
    {0,0,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},
    {0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,7,0},
    {0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},
    {0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,0,0}
};

const uint8_t sprite_4[NUM_HEIGHT][NUM_WIDTH] = {
    {0,0,0,0,0,0,0,0,0,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},
    {0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,7,0},
    {0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},
    {0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,0,0}
};

const uint8_t sprite_5[NUM_HEIGHT][NUM_WIDTH] = {
    {0,0,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,0,0,0,0,0,0,0,0},{0,7,0,0,0,0,0,0,0,0},
    {0,7,0,0,0,0,0,0,0,0},{0,7,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,7,0},
    {0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},
    {0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,0,0}
};

const uint8_t sprite_6[NUM_HEIGHT][NUM_WIDTH] = {
    {0,0,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,0,0,0,0,0,0,0,0},{0,7,0,0,0,0,0,0,0,0},
    {0,7,0,0,0,0,0,0,0,0},{0,7,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,0,0,0,0,0,0,7,0},
    {0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},
    {0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,0,0}
};

const uint8_t sprite_7[NUM_HEIGHT][NUM_WIDTH] = {
    {0,0,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},
    {0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},
    {0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},
    {0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,0,0}
};

const uint8_t sprite_8[NUM_HEIGHT][NUM_WIDTH] = {
    {0,0,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},
    {0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,7,7,7,7,7,7,7,0},{0,7,0,0,0,0,0,0,7,0},
    {0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},
    {0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,0,0}
};

const uint8_t sprite_9[NUM_HEIGHT][NUM_WIDTH] = {
    {0,0,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},
    {0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,7,0},
    {0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},
    {0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,0,0}
};

const uint8_t (*sprite_numbers[])[NUM_WIDTH] = {
    sprite_0, sprite_1, sprite_2, sprite_3, sprite_4, sprite_5, sprite_6, sprite_7, sprite_8, sprite_9
};

void start_screen(void) {
        DrawSprite(3, 4, CHAR_HEIGHT, CHAR_WIDTH, sprite_A);        
        DrawSprite(8, 4, CHAR_HEIGHT, CHAR_WIDTH, sprite_U);
        DrawSprite(13, 4, CHAR_HEIGHT, CHAR_WIDTH, sprite_T);
        DrawSprite(18, 4, CHAR_HEIGHT, CHAR_WIDTH, sprite_O);
        DrawSprite(23, 4, CHAR_HEIGHT, CHAR_WIDTH, sprite_N);
        DrawSprite(17, 12, CHAR_HEIGHT, CHAR_WIDTH, sprite_A);
        DrawSprite(22, 12, CHAR_HEIGHT, CHAR_WIDTH, sprite_I);
        DrawSprite(27, 12, CHAR_HEIGHT, CHAR_WIDTH, sprite_R);  
}

// --- NEW DEFINITIONS ---
#define TITLE_WIDTH  14
#define COLON_WIDTH  4
#define COLON_HEIGHT 14  // Matches NUM_HEIGHT

// --- NEW SPRITES ---

// "YOU" (Width 14, Height 5)
const uint8_t sprite_word_YOU[CHAR_HEIGHT][TITLE_WIDTH] = {
    {7,0,0,7, 0, 0,7,7,0, 0, 7,0,0,7}, // Y   O   U
    {7,0,0,7, 0, 7,0,0,7, 0, 7,0,0,7},
    {0,7,7,0, 0, 7,0,0,7, 0, 7,0,0,7},
    {0,0,7,0, 0, 7,0,0,7, 0, 7,0,0,7},
    {0,0,7,0, 0, 0,7,7,0, 0, 0,7,7,0}
};

// "BOT" (Width 14, Height 5)
const uint8_t sprite_word_BOT[CHAR_HEIGHT][TITLE_WIDTH] = {
    {7,7,7,0, 0, 0,7,7,0, 0, 7,7,7,7}, // B   O   T
    {7,0,0,7, 0, 7,0,0,7, 0, 0,7,7,0},
    {7,7,7,0, 0, 7,0,0,7, 0, 0,7,7,0},
    {7,0,0,7, 0, 7,0,0,7, 0, 0,7,7,0},
    {7,7,7,0, 0, 0,7,7,0, 0, 0,7,7,0}
};

// "::" (Width 4, Height 14) - Vertically Centered relative to numbers
const uint8_t sprite_colon[COLON_HEIGHT][COLON_WIDTH] = {
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0},
    {0,7,7,0}, // Top Dot
    {0,7,7,0},
    {0,0,0,0},
    {0,0,0,0}, // Gap
    {0,7,7,0}, // Bottom Dot
    {0,7,7,0},
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0},
    {0,0,0,0}
};

// "WON" (Width 14, Height 5)
const uint8_t sprite_word_WON[CHAR_HEIGHT][TITLE_WIDTH] = {
    {7,0,0,7, 0, 0,7,7,0, 0, 7,0,0,7}, // W   O   N
    {7,0,0,7, 0, 7,0,0,7, 0, 7,7,0,7},
    {7,0,0,7, 0, 7,0,0,7, 0, 7,7,7,7},
    {7,7,7,7, 0, 7,0,0,7, 0, 7,0,7,7},
    {7,0,0,7, 0, 0,7,7,0, 0, 7,0,0,7}
};

//===========================================================================
// SCAN FUNCTION
//===========================================================================
static void Matrix_Scan(void) {
    // 1. Disable Display
    GPIOB->BSRR = (1U << 14);

    // 2. Shift Data
    for (int col = 0; col < MATRIX_WIDTH; col++) {
        uint8_t packed_color = canvas[current_display_row][col];
        
        uint32_t porta_set = 0, porta_reset = 0;
        uint32_t portc_set = 0, portc_reset = 0;

        if (packed_color & 0x01) porta_set |= (1U << 4); else porta_reset |= (1U << 4);
        if (packed_color & 0x02) porta_set |= (1U << 5); else porta_reset |= (1U << 5);
        if (packed_color & 0x04) porta_set |= (1U << 6); else porta_reset |= (1U << 6);
        if (packed_color & 0x08) porta_set |= (1U << 7); else porta_reset |= (1U << 7);
        
        if (packed_color & 0x10) portc_set |= (1U << 4); else portc_reset |= (1U << 4);
        if (packed_color & 0x20) portc_set |= (1U << 5); else portc_reset |= (1U << 5);

        GPIOA->BSRR = porta_set | (porta_reset << 16);
        GPIOC->BSRR = portc_set | (portc_reset << 16);

        // Clock Pulse
        GPIOB->BSRR = (1U << 12); 
        GPIOB->BRR  = (1U << 12);
    }

    // 3. Set Address
    uint8_t addr = current_display_row;
    uint32_t addr_set = 0, addr_reset = 0;
    
    if (addr & 0x01) addr_set |= (1U << 0);  else addr_reset |= (1U << 0);
    if (addr & 0x02) addr_set |= (1U << 1);  else addr_reset |= (1U << 1);
    if (addr & 0x04) addr_set |= (1U << 10); else addr_reset |= (1U << 10);
    if (addr & 0x08) addr_set |= (1U << 11); else addr_reset |= (1U << 11);
    GPIOB->BSRR = addr_set | (addr_reset << 16);

    // 4. Latch
    GPIOB->BSRR = (1U << 13); 
    asm("nop"); asm("nop");
    GPIOB->BRR  = (1U << 13);

    // 5. Enable Display
    GPIOB->BRR = (1U << 14); 

    // 6. Next Row
    current_display_row = (current_display_row + 1) % MATRIX_SCAN_ROWS;
}

//===========================================================================
// INTERRUPTS
//===========================================================================

void TIM6_DAC_IRQHandler(void) {
    if (TIM6->SR & TIM_SR_UIF) {
        TIM6->SR &= ~TIM_SR_UIF; 
        Matrix_Scan();           
    }
}

void TIM14_IRQHandler(void) {
    if (TIM14->SR & TIM_SR_UIF) {
        TIM14->SR &= ~TIM_SR_UIF;

        // If game not active, do nothing
        if (!game_active) return;

        // Debounce / Cooldown logic
        if (sensor_cooldown > 0) {
            sensor_cooldown--;
            return;
        }

        bool scored = false;
        
        // --- CHANGED LOGIC START ---
        
        // PA12 (Player Score) 
        // We now check if the bit is 0 (LOW) using the logic NOT operator (!)
        if (!(GPIOA->IDR & (1 << 12))) {
            player_score++;
            scored = true;
        } 
        // PA11 (Bot Score)
        // We now check if the bit is 0 (LOW)
        else if (!(GPIOA->IDR & (1 << 11))) {
            bot_score++;
            scored = true;
        }

        // --- CHANGED LOGIC END ---

        if (scored) {
            sensor_cooldown = GOAL_COOLDOWN_TICKS;
            // Check Win Condition (7 points)
            if (player_score >= WINNING_SCORE || bot_score >= WINNING_SCORE) {
                game_active = false;
            }
        }
    }
}

//===========================================================================
// MAIN
//===========================================================================

int main(void) {
    enable_ports(); 
    
    init_matrix_gpio();
    init_refresh_timer();
    setup_tim14();
    init_sensors();      
    init_start_reset();  

    while (1) {
        // --- BUTTON INPUTS ---
        
        // PB2 = Start
        if (GPIOB->IDR & (1<<2)) {
            game_active = true;
            // If starting a new game after a win, reset scores
            if (player_score >= WINNING_SCORE || bot_score >= WINNING_SCORE) {
                 player_score = 0;
                 bot_score = 0;
            }
        }
        
        // PA0 = Reset
        if (GPIOA->IDR & (1<<0)) {
            game_active = false;
            player_score = 0;
            bot_score = 0;
        }

        // --- DRAWING LOGIC ---

        ClearScreen();

        if (game_active) {
            // === ACTIVE GAME STATE ===
            
            // Draw Titles
            DrawSprite(0, 2, CHAR_HEIGHT, TITLE_WIDTH, sprite_word_YOU);
            DrawSprite(18, 2, CHAR_HEIGHT, TITLE_WIDTH, sprite_word_BOT);

            // Draw Scores & Separator
            if (player_score < 10)
                DrawSprite(4, 9, NUM_HEIGHT, NUM_WIDTH, sprite_numbers[player_score]);
            
            DrawSprite(14, 9, COLON_HEIGHT, COLON_WIDTH, sprite_colon);

            if (bot_score < 10)
                DrawSprite(18, 9, NUM_HEIGHT, NUM_WIDTH, sprite_numbers[bot_score]);

        } else {
            // === INACTIVE STATE (Start or End Screen) ===
            
            if (player_score >= WINNING_SCORE) {
                // PLAYER WON SCREEN
                // Draw "YOU" at Top (Centered horizontally relative to screen width 32)
                // "YOU" is 14px wide. Screen 32. Center x = (32-14)/2 = 9
                DrawSprite(9, 2, CHAR_HEIGHT, TITLE_WIDTH, sprite_word_YOU);
                
                // Draw "WON" below it
                DrawSprite(9, 9, CHAR_HEIGHT, TITLE_WIDTH, sprite_word_WON);
                
            } else if (bot_score >= WINNING_SCORE) {
                // BOT WON SCREEN
                // Draw "BOT" at Top
                DrawSprite(9, 2, CHAR_HEIGHT, TITLE_WIDTH, sprite_word_BOT);
                
                // Draw "WON" below it
                DrawSprite(9, 9, CHAR_HEIGHT, TITLE_WIDTH, sprite_word_WON);
                
            } else {
                // START SCREEN (Scores are 0 or low)
                start_screen();
            }
        }
        
        delay_ms(16); 
    }
}
