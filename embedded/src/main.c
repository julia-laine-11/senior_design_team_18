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
#define GOAL_COOLDOWN_TICKS 50 

// Sprites
#define CHAR_HEIGHT 5
#define CHAR_WIDTH 4
#define NUM_HEIGHT 14
#define NUM_WIDTH 10

// OLED Protocol (10-bit)
#define OLED_RS_CMD 0
#define OLED_RS_DATA 1

// Joystick Thresholds
#define JOY_LOW_THRESHOLD   0.66f
#define JOY_HIGH_THRESHOLD  2.64f

//===========================================================================
// GLOBAL VARIABLES
//===========================================================================

volatile int player_score = 0;
volatile int bot_score = 0;
volatile int sensor_cooldown = 0;
volatile bool game_active = false;

typedef enum {
    MODE_AI,
    MODE_HUMAN,
    MODE_X
} GameMode;

// Default to AI
GameMode current_mode = MODE_AI;

// Display Buffer
volatile uint8_t canvas[MATRIX_SCAN_ROWS][MATRIX_WIDTH];
static volatile uint8_t current_display_row = 0;

//===========================================================================
// OLED DRIVER (Lab 6 Protocol Emulation)
//===========================================================================

void small_delay(void) {
    // Increased delay to ensure stable clock edges for OLED
    for(volatile int i=0; i<50; i++); 
}

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        SysTick->LOAD = 48000 - 1;
        SysTick->VAL = 0;
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
        SysTick->CTRL = 0; 
    }
}

void init_oled_gpio(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;
    
    // PC10 (SDI), PC11 (SCL) Output
    GPIOC->MODER &= ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11);
    GPIOC->MODER |= (GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0); 
    
    // PA15 (CS) Output
    GPIOA->MODER &= ~(GPIO_MODER_MODER15);
    GPIOA->MODER |= (GPIO_MODER_MODER15_0);
    
    // Set Idle States
    GPIOA->BSRR = (1 << 15); 
    GPIOC->BRR  = (1 << 11); 
}

// 10-bit SPI: [RS] [RW] [D7..D0]
void OLED_Send10Bit(uint16_t value) {
    GPIOA->BRR = (1 << 15); // CS Low
    small_delay();

    // Send 10 bits, MSB first
    for (int i = 9; i >= 0; i--) {
        if ((value >> i) & 0x01) GPIOC->BSRR = (1 << 10); 
        else                     GPIOC->BRR = (1 << 10);
        small_delay();
        
        GPIOC->BSRR = (1 << 11); // Clock High
        small_delay();
        GPIOC->BRR = (1 << 11);  // Clock Low
    }

    small_delay();
    GPIOA->BSRR = (1 << 15); // CS High
    delay_ms(1);
}

void OLED_Command(uint8_t cmd) {
    // RS=0, RW=0
    OLED_Send10Bit((uint16_t)cmd);
}

void OLED_Data(uint8_t data) {
    // RS=1 (Bit 9), RW=0
    // 0x200 = 10 0000 0000
    OLED_Send10Bit(0x200 | (uint16_t)data);
}

void OLED_String(char *str) {
    while (*str) OLED_Data(*str++);
}

void OLED_Clear_Screen(void) {
    OLED_Command(0x01); // Clear
    delay_ms(3);
    OLED_Command(0x02); // Home
}

void OLED_Init(void) {
    delay_ms(100);      
    OLED_Command(0x38); // Function Set
    OLED_Command(0x08); // OFF
    OLED_Command(0x01); // Clear
    delay_ms(3);        
    OLED_Command(0x06); // Entry Mode
    OLED_Command(0x02); // Home
    OLED_Command(0x0C); // ON
}

//===========================================================================
// JOYSTICK & ADC DRIVER
//===========================================================================

void init_adc(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    
    // Clear first to be safe, then set PC1 to Analog (11)
    GPIOC->MODER |= (GPIO_MODER_MODER1); 
    
    // PC2 Input with Pull-Up (Switch)
    GPIOC->MODER &= ~(GPIO_MODER_MODER2);
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR2);
    GPIOC->PUPDR |= (GPIO_PUPDR_PUPDR2_0);

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->CR |= ADC_CR_ADCAL;
    while(ADC1->CR & ADC_CR_ADCAL);
    ADC1->CR |= ADC_CR_ADEN;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
}

float read_adc_voltage(int channel) {
    ADC1->CHSELR = (1 << channel);
    ADC1->CR |= ADC_CR_ADSTART;
    while(!(ADC1->ISR & ADC_ISR_EOC));
    return (ADC1->DR * 3.3f) / 4095.0f;
}

int read_joystick_sw(void) {
    return (GPIOC->IDR & (1 << 2)) ? 1 : 0;
}

//===========================================================================
// HARDWARE SETUP
//===========================================================================

void enable_ports(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
}

void init_start_reset(void) {
    GPIOA->MODER &= ~(GPIO_MODER_MODER0); 
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0); 
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR0_1);
}

void init_sensors(void) {
    GPIOA->MODER &= ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER12);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR11 | GPIO_PUPDR_PUPDR12);
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR11_0 | GPIO_PUPDR_PUPDR12_0);
}

void init_matrix_gpio(void) {
    GPIOA->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOA->MODER |= (GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0);
    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR4 | GPIO_OSPEEDR_OSPEEDR5 | GPIO_OSPEEDR_OSPEEDR6 | GPIO_OSPEEDR_OSPEEDR7);
    
    GPIOB->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER10 | GPIO_MODER_MODER11 | GPIO_MODER_MODER12 | GPIO_MODER_MODER13 | GPIO_MODER_MODER14);
    GPIOB->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0 | GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0);
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR0 | GPIO_OSPEEDR_OSPEEDR1 | GPIO_OSPEEDR_OSPEEDR10 | GPIO_OSPEEDR_OSPEEDR11 | GPIO_OSPEEDR_OSPEEDR12 | GPIO_OSPEEDR_OSPEEDR13 | GPIO_OSPEEDR_OSPEEDR14);
    
    // Matrix uses PC4, PC5. Clear first, then Set Output.
    GPIOC->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
    GPIOC->MODER |= (GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0);
    GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR4 | GPIO_OSPEEDR_OSPEEDR5);
    
    GPIOB->BSRR = (1U << 14); 
    GPIOB->BRR = (1U << 13); 
    GPIOB->BRR = (1U << 12);
    GPIOA->BRR = (0xF0); 
    GPIOC->BRR = (0x30); 
    GPIOB->BRR = (0x0C03); 
}

void init_refresh_timer(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = 40 - 1; TIM6->ARR = 100 - 1; 
    TIM6->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM6_DAC_IRQn, 0); NVIC_EnableIRQ(TIM6_DAC_IRQn);
    TIM6->CR1 |= TIM_CR1_CEN;
}

void setup_tim14(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    TIM14->PSC = 4800 - 1; TIM14->ARR = 100 - 1; 
    TIM14->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM14_IRQn, 2); NVIC_EnableIRQ(TIM14_IRQn);
    TIM14->CR1 |= TIM_CR1_CEN;
}

//===========================================================================
// GRAPHICS & SPRITES
//===========================================================================

void SetPixel(int x, int y, uint8_t color_value) {
    if (x < 0 || x >= MATRIX_WIDTH || y < 0 || y >= MATRIX_HEIGHT) return;
    uint8_t row = y % MATRIX_SCAN_ROWS;
    uint8_t col = x;
    color_value &= 0x7;
    
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t current = canvas[row][col];
    if (y < MATRIX_SCAN_ROWS) canvas[row][col] = (current & ~0x07) | color_value;
    else canvas[row][col] = (current & ~0x38) | (color_value << 3);
    if (!primask) __enable_irq();
}

void ClearScreen(void) {
    for (int r = 0; r < MATRIX_SCAN_ROWS; r++) 
        for (int c = 0; c < MATRIX_WIDTH; c++) canvas[r][c] = 0;
}

void DrawSprite(int start_x, int start_y, int height, int width, const uint8_t sprite_data[][width]) {
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            if (sprite_data[r][c] != 0) SetPixel(start_x + c, start_y + r, sprite_data[r][c]);
        }
    }
}

// Sprites
const uint8_t sprite_A[CHAR_HEIGHT][CHAR_WIDTH] = {{0,7,7,0},{7,0,0,7},{7,7,7,7},{7,0,0,7},{7,0,0,7}};
const uint8_t sprite_U[CHAR_HEIGHT][CHAR_WIDTH] = {{7,0,0,7},{7,0,0,7},{7,0,0,7},{7,0,0,7},{0,7,7,0}};
const uint8_t sprite_T[CHAR_HEIGHT][CHAR_WIDTH] = {{7,7,7,7},{0,7,7,0},{0,7,7,0},{0,7,7,0},{0,7,7,0}};
const uint8_t sprite_O[CHAR_HEIGHT][CHAR_WIDTH] = {{0,7,7,0},{7,0,0,7},{7,0,0,7},{7,0,0,7},{0,7,7,0}};
const uint8_t sprite_N[CHAR_HEIGHT][CHAR_WIDTH] = {{7,7,0,7},{7,7,0,7},{7,7,7,7},{7,0,7,7},{7,0,0,7}};
const uint8_t sprite_I[CHAR_HEIGHT][CHAR_WIDTH] = {{7,7,7,7},{0,7,7,0},{0,7,7,0},{0,7,7,0},{7,7,7,7}};
const uint8_t sprite_R[CHAR_HEIGHT][CHAR_WIDTH] = {{7,7,7,0},{7,0,0,7},{7,7,7,0},{7,0,7,0},{7,0,0,7}};

const uint8_t sprite_0[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,0,0}};
const uint8_t sprite_1[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,0,0,0,0,0,0}};
const uint8_t sprite_2[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,7,7,7,7,7,7,7,7,0},{0,7,0,0,0,0,0,0,0,0},{0,7,0,0,0,0,0,0,0,0},{0,7,0,0,0,0,0,0,0,0},{0,7,0,0,0,0,0,0,0,0},{0,7,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,0,0}};
// Placeholders for 3-9
const uint8_t sprite_3[NUM_HEIGHT][NUM_WIDTH] = {{0},{0}}; const uint8_t sprite_4[NUM_HEIGHT][NUM_WIDTH] = {{0},{0}};
const uint8_t sprite_5[NUM_HEIGHT][NUM_WIDTH] = {{0},{0}}; const uint8_t sprite_6[NUM_HEIGHT][NUM_WIDTH] = {{0},{0}};
const uint8_t sprite_7[NUM_HEIGHT][NUM_WIDTH] = {{0},{0}}; const uint8_t sprite_8[NUM_HEIGHT][NUM_WIDTH] = {{0},{0}};
const uint8_t sprite_9[NUM_HEIGHT][NUM_WIDTH] = {{0},{0}};
const uint8_t (*sprite_numbers[])[NUM_WIDTH] = { sprite_0, sprite_1, sprite_2, sprite_3, sprite_4, sprite_5, sprite_6, sprite_7, sprite_8, sprite_9 };

#define TITLE_WIDTH  14
#define COLON_WIDTH  4
#define COLON_HEIGHT 14 
const uint8_t sprite_word_YOU[CHAR_HEIGHT][TITLE_WIDTH] = {{7,0,0,7, 0, 0,7,7,0, 0, 7,0,0,7}, {7,0,0,7, 0, 7,0,0,7, 0, 7,0,0,7}, {0,7,7,0, 0, 7,0,0,7, 0, 7,0,0,7}, {0,0,7,0, 0, 7,0,0,7, 0, 7,0,0,7}, {0,0,7,0, 0, 0,7,7,0, 0, 0,7,7,0}};
const uint8_t sprite_word_BOT[CHAR_HEIGHT][TITLE_WIDTH] = {{7,7,7,0, 0, 0,7,7,0, 0, 7,7,7,7}, {7,0,0,7, 0, 7,0,0,7, 0, 0,7,7,0}, {7,7,7,0, 0, 7,0,0,7, 0, 0,7,7,0}, {7,0,0,7, 0, 7,0,0,7, 0, 0,7,7,0}, {7,7,7,0, 0, 0,7,7,0, 0, 0,7,7,0}};
const uint8_t sprite_colon[COLON_HEIGHT][COLON_WIDTH] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,7,7,0},{0,7,7,0},{0,0,0,0},{0,0,0,0},{0,7,7,0},{0,7,7,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
const uint8_t sprite_word_WON[CHAR_HEIGHT][TITLE_WIDTH] = {{7,0,0,7, 0, 0,7,7,0, 0, 7,0,0,7}, {7,0,0,7, 0, 7,0,0,7, 0, 7,7,0,7}, {7,0,0,7, 0, 7,0,0,7, 0, 7,7,7,7}, {7,7,7,7, 0, 7,0,0,7, 0, 7,0,7,7}, {7,0,0,7, 0, 0,7,7,0, 0, 7,0,0,7}};

void start_screen(void) {
    DrawSprite(3, 4, CHAR_HEIGHT, CHAR_WIDTH, sprite_A); DrawSprite(8, 4, CHAR_HEIGHT, CHAR_WIDTH, sprite_U); DrawSprite(13, 4, CHAR_HEIGHT, CHAR_WIDTH, sprite_T); DrawSprite(18, 4, CHAR_HEIGHT, CHAR_WIDTH, sprite_O); DrawSprite(23, 4, CHAR_HEIGHT, CHAR_WIDTH, sprite_N);
    DrawSprite(17, 12, CHAR_HEIGHT, CHAR_WIDTH, sprite_A); DrawSprite(22, 12, CHAR_HEIGHT, CHAR_WIDTH, sprite_I); DrawSprite(27, 12, CHAR_HEIGHT, CHAR_WIDTH, sprite_R);  
}

//===========================================================================
// SCAN FUNCTION
//===========================================================================
static void Matrix_Scan(void) {
    GPIOB->BSRR = (1U << 14); 
    for (int col = 0; col < MATRIX_WIDTH; col++) {
        uint8_t packed_color = canvas[current_display_row][col];
        uint32_t porta_set=0,porta_reset=0, portc_set=0,portc_reset=0;
        if (packed_color & 0x01) porta_set |= (1U<<4); else porta_reset |= (1U<<4);
        if (packed_color & 0x02) porta_set |= (1U<<5); else porta_reset |= (1U<<5);
        if (packed_color & 0x04) porta_set |= (1U<<6); else porta_reset |= (1U<<6);
        if (packed_color & 0x08) porta_set |= (1U<<7); else porta_reset |= (1U<<7);
        if (packed_color & 0x10) portc_set |= (1U<<4); else portc_reset |= (1U<<4);
        if (packed_color & 0x20) portc_set |= (1U<<5); else portc_reset |= (1U<<5);
        GPIOA->BSRR = porta_set | (porta_reset << 16); GPIOC->BSRR = portc_set | (portc_reset << 16);
        GPIOB->BSRR = (1U << 12); GPIOB->BRR  = (1U << 12);
    }
    uint8_t addr = current_display_row;
    uint32_t addr_set=0, addr_reset=0;
    if (addr & 0x01) addr_set |= (1U<<0); else addr_reset |= (1U<<0);
    if (addr & 0x02) addr_set |= (1U<<1); else addr_reset |= (1U<<1);
    if (addr & 0x04) addr_set |= (1U<<10); else addr_reset |= (1U<<10);
    if (addr & 0x08) addr_set |= (1U<<11); else addr_reset |= (1U<<11);
    GPIOB->BSRR = addr_set | (addr_reset << 16);
    GPIOB->BSRR = (1U << 13); asm("nop"); asm("nop"); GPIOB->BRR  = (1U << 13);
    GPIOB->BRR = (1U << 14); 
    current_display_row = (current_display_row + 1) % MATRIX_SCAN_ROWS;
}

void TIM6_DAC_IRQHandler(void) {
    if (TIM6->SR & TIM_SR_UIF) { TIM6->SR &= ~TIM_SR_UIF; Matrix_Scan(); }
}

void TIM14_IRQHandler(void) {
    if (TIM14->SR & TIM_SR_UIF) {
        TIM14->SR &= ~TIM_SR_UIF;
        if (!game_active) return;
        if (sensor_cooldown > 0) { sensor_cooldown--; return; }

        bool scored = false;
        if (!(GPIOA->IDR & (1 << 12))) { player_score++; scored = true; } 
        else if (!(GPIOA->IDR & (1 << 11))) { bot_score++; scored = true; }

        if (scored) {
            sensor_cooldown = GOAL_COOLDOWN_TICKS;
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
    init_oled_gpio();
    OLED_Init(); 
    init_adc();
    init_matrix_gpio();
    init_refresh_timer();
    setup_tim14();
    init_sensors();      
    init_start_reset();  

    // --- 1. MENU STATE ---
    
    // --- 1. MENU STATE ---
start_screen(); // Display title on LED Matrix

OLED_Clear_Screen();
OLED_Command(0x80); 
OLED_String("Select Mode:");

current_mode = MODE_AI; // Default start selection
bool last_displayed_mode = !current_mode; // Force first OLED update

// Threshold constants from your reference snippet
const float b1_AI    = 2.4f; // Left Threshold
const float b4_HUMAN = 2.4f; // Right Threshold

while(1) {
    // Read X-Axis (PC1 -> ADC Channel 11)
    float vx = read_adc_voltage(11); 

    /* LATCHING LOGIC:
       Only change the mode if we cross the extreme thresholds.
       If the stick is centered (between 0.66V and 2.64V), 
       do nothing and keep the last selection.
    */
    if ((vx > 0) & (vx < 0.66f)) {
        // Stick pushed Left -> Select AI
        current_mode = MODE_AI;
    } 
    else if ((vx > 3.0f)) {
        // Stick pushed Right -> Select HUMAN
        current_mode = MODE_HUMAN;
    }
    

    // Only update the OLED if the selection has actually changed
    if (current_mode != last_displayed_mode) {
        OLED_Command(0xC0); // Move to Line 2
        if (current_mode == MODE_AI) {
            OLED_String("[AI]   HUMAN  ");
        } else if (current_mode == MODE_HUMAN) {
            OLED_String(" AI   [HUMAN] ");
        } else {
            OLED_String(" AI    HUMAN  ");
        }
        last_displayed_mode = current_mode;
    }

    // Check for "Select" button press (Joystick SW on PC2)
    // The datasheet confirms this is typically an active-low switch [cite: 643, 741]
    if (read_joystick_sw() == 0) { 
        delay_ms(200); // Debounce delay
        break; // Exit menu and proceed to Game Ready
    }
    
    delay_ms(20); // Small poll delay to save CPU cycles
}

    // --- 2. GAME READY STATE ---
    OLED_Clear_Screen(); 
    OLED_Command(0x80); OLED_String("Game Ready      ");
    OLED_Command(0xC0); 
    if (current_mode == MODE_AI) OLED_String("Mode: AI        ");
    else                         OLED_String("Mode: HUMAN     ");
    
    while(1) {
        if (read_joystick_sw() == 0) {
            delay_ms(200); 
            game_active = true;
            break;
        }
        delay_ms(50);
    }

    // --- 3. GAME RUNNING STATE ---
    bool last_game_state = !game_active; 
    
    while (1) {
        if (GPIOA->IDR & (1<<0)) { // Reset Button
            game_active = false;
            player_score = 0;
            bot_score = 0;
        }

        if (game_active != last_game_state) {
            OLED_Clear_Screen();
            
            OLED_Command(0x80); 
            if (game_active) OLED_String("Game Running    ");
            else             OLED_String("Game Finished   ");
            
            OLED_Command(0xC0); 
            if (current_mode == MODE_AI) OLED_String("Mode: AI        ");
            else                         OLED_String("Mode: HUMAN     ");
            
            last_game_state = game_active;
        }

        ClearScreen();

        if (game_active) {
            DrawSprite(0, 2, CHAR_HEIGHT, TITLE_WIDTH, sprite_word_YOU);
            DrawSprite(18, 2, CHAR_HEIGHT, TITLE_WIDTH, sprite_word_BOT);
            if (player_score < 10) DrawSprite(4, 9, NUM_HEIGHT, NUM_WIDTH, sprite_numbers[player_score]);
            DrawSprite(14, 9, COLON_HEIGHT, COLON_WIDTH, sprite_colon);
            if (bot_score < 10) DrawSprite(18, 9, NUM_HEIGHT, NUM_WIDTH, sprite_numbers[bot_score]);
            
        } else {
             if (player_score >= WINNING_SCORE) {
                DrawSprite(9, 2, CHAR_HEIGHT, TITLE_WIDTH, sprite_word_YOU);
                DrawSprite(9, 9, CHAR_HEIGHT, TITLE_WIDTH, sprite_word_WON);
            } else if (bot_score >= WINNING_SCORE) {
                DrawSprite(9, 2, CHAR_HEIGHT, TITLE_WIDTH, sprite_word_BOT);
                DrawSprite(9, 9, CHAR_HEIGHT, TITLE_WIDTH, sprite_word_WON);
            } else {
                start_screen();
            }
        }
        delay_ms(16); 
    }
}