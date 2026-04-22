#include "stm32f0xx.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

//===========================================================================
// DEFINITIONS
//===========================================================================

#define MATRIX_HEIGHT       32
#define MATRIX_WIDTH        32
#define MATRIX_SCAN_ROWS    16

#define WINNING_SCORE       7
#define GOAL_COOLDOWN_TICKS 1500 // 1.5 seconds of total lockout

#define MODE_BOT            0
#define MODE_PLAYER         1

#define UI_STATE_SPLASH     0
#define UI_STATE_MENU       1
#define UI_STATE_PLAY       2

#define COLOR_RED   1
#define COLOR_BLUE  4

#define CHAR_HEIGHT 5
#define CHAR_WIDTH 4
#define NUM_HEIGHT 14
#define NUM_WIDTH 10

#define TEST_MODE 0     
#define WATCHDOG_MAX 25 
#define SYSTEM_CLOCK 48000000 

#define ABS(x) ((x) < 0 ? -(x) : (x))

//===========================================================================
// GLOBAL VARIABLES
//===========================================================================

volatile int player_score = 0;
volatile int bot_score = 0;
volatile int sensor_cooldown = 0;
volatile bool game_active = false;
volatile uint8_t game_mode = MODE_BOT; 

volatile uint8_t canvas[MATRIX_SCAN_ROWS][MATRIX_WIDTH];
static volatile uint8_t current_display_row = 0;
volatile uint32_t anim_tick = 0; 

// UART / Motor State Variables
volatile uint8_t pending_motor = 0;
volatile uint8_t pending_dir = 0;
volatile uint32_t watchdog_timer = 0;

// Firework Particle System
#define NUM_FW 5
typedef struct {
    int x, y, frame, color, max_r;
} Firework;
Firework fws[NUM_FW] = {0};

//===========================================================================
// FORWARD DECLARATIONS
//===========================================================================
void set_motor_a(uint32_t percent, uint8_t is_rev);
void set_motor_b(uint32_t percent, uint8_t is_rev);

//===========================================================================
// SYSTEM CLOCK (48 MHz)
//===========================================================================

void init_clock(void) {
    FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;
    RCC->CFGR &= ~(RCC_CFGR_PLLMUL | RCC_CFGR_PLLSRC);
    RCC->CFGR |= RCC_CFGR_PLLMUL12;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)); 
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); 
}

//===========================================================================
// TRUE HUB75 MATRIX SCAN (64-Pin Layout)
//===========================================================================

static inline void Matrix_Scan(uint8_t row) {
    for (int col = 0; col < 32; col++) {
        uint8_t p = canvas[row][col];
        
        GPIOA->BSRR = ((p & 0xF) << 4) | ((~(p & 0xF) & 0xF) << 20);
        GPIOC->BSRR = ((p & 0x30)) | ((~(p & 0x30) & 0x30) << 16);

        GPIOB->BSRR = (1U << 12); 
        GPIOB->BRR  = (1U << 12); 
    }

    GPIOB->BSRR = (1U << 14); 

    GPIOB->BSRR = (1U << 13);
    for(volatile int i = 0; i < 2; i++); 
    GPIOB->BRR  = (1U << 13);

    uint32_t b_set = 0;
    if (row & 0x01) b_set |= (1U << 0);
    if (row & 0x02) b_set |= (1U << 1);
    if (row & 0x04) b_set |= (1U << 10);
    if (row & 0x08) b_set |= (1U << 11);
    
    GPIOB->BRR = (1U << 0) | (1U << 1) | (1U << 10) | (1U << 11);
    GPIOB->BSRR = b_set;

    GPIOB->BRR = (1U << 14);
    
    for(volatile int i = 0; i < 400; i++); 
}

//===========================================================================
// HELPER FUNCTIONS 
//===========================================================================

void USART3_8_IRQHandler(void) {
    if (USART5->ISR & USART_ISR_ORE) {
        USART5->ICR |= USART_ICR_ORECF;
    }

    if (USART5->ISR & USART_ISR_RXNE) {
        uint8_t rx = USART5->RDR;
        if (rx & 0x80) { 
            pending_motor = (rx >> 6) & 0x01;
            pending_dir   = (rx >> 5) & 0x01;
        } else { 
            uint8_t percent = rx & 0x7F; 
            if (percent <= 100) { 
                if (pending_motor == 0) set_motor_a(percent, pending_dir);
                else                    set_motor_b(percent, pending_dir);
                
                if (TEST_MODE == 0) watchdog_timer = 0;
            }
        }
    }
}

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        SysTick->LOAD = 48000 - 1;                  
        SysTick->VAL = 0;
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
        
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) {
            Matrix_Scan(current_display_row);
            current_display_row = (current_display_row + 1) & 0x0F;
        }
        SysTick->CTRL = 0;  
    }
}

void small_delay(void) {
    for(volatile int i=0; i<15; i++);
}

//===========================================================================
// ADC & OLED & HARDWARE SETUP 
//===========================================================================

void init_adc(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    GPIOC->MODER |= GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1; 
    RCC->CR2 |= RCC_CR2_HSI14ON;
    while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0);
    ADC1->CR |= ADC_CR_ADEN;
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0);
    ADC1->CHSELR = ADC_CHSELR_CHSEL10;
}

uint16_t read_adc(void) {
    ADC1->CR |= ADC_CR_ADSTART;
    while ((ADC1->ISR & ADC_ISR_EOC) == 0);
    return ADC1->DR;
}

void init_oled_pins(void) {
    GPIOA->MODER &= ~(GPIO_MODER_MODER15); GPIOA->MODER |= (GPIO_MODER_MODER15_0);
    GPIOC->MODER &= ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11); GPIOC->MODER |= (GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0);
    GPIOC->BSRR = (1U << 11); GPIOA->BRR = (1U << 15);  
}

void spi_send_10bit(uint16_t data) {
    GPIOC->BRR = (1U << 11); small_delay();
    for (int i = 9; i >= 0; i--) {
        if ((data >> i) & 1) GPIOC->BSRR = (1U << 10); else GPIOC->BRR = (1U << 10);
        small_delay(); GPIOA->BSRR = (1U << 15); small_delay(); GPIOA->BRR = (1U << 15); small_delay();
    }
    GPIOC->BSRR = (1U << 11); small_delay();
}

void spi_cmd(unsigned int data) { spi_send_10bit(data & 0xFF); delay_ms(1); }
void spi_data(unsigned int data) { spi_send_10bit(data | 0x200); delay_ms(1); }

void spi1_init_oled(void) {
    delay_ms(100); 
    spi_cmd(0x38); spi_cmd(0x08); spi_cmd(0x17); spi_cmd(0x01); 
    delay_ms(5);   
    spi_cmd(0x06); spi_cmd(0x02); spi_cmd(0x0C); 
}

void spi1_display1(const char *string) {
    spi_cmd(0x02); while(*string != '\0') { spi_data(*string); string++; }
}
void spi1_display2(const char *string) {
    spi_cmd(0xC0); while(*string != '\0') { spi_data(*string); string++; }
}

void init_controls(void) {
    GPIOC->MODER &= ~(GPIO_MODER_MODER2); GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR2); GPIOC->PUPDR |= (GPIO_PUPDR_PUPDR2_0);
}

void init_sensors(void) {
    GPIOA->MODER &= ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER12);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR11 | GPIO_PUPDR_PUPDR12);
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR11_0 | GPIO_PUPDR_PUPDR12_0);
}

void init_matrix_gpio(void) {
    GPIOA->MODER &= ~(0xFF00); GPIOA->MODER |= 0x5500;
    GPIOC->MODER &= ~(0xF00); GPIOC->MODER |= 0x500;
    GPIOB->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER10 | 
                      GPIO_MODER_MODER11 | GPIO_MODER_MODER12 | GPIO_MODER_MODER13 | GPIO_MODER_MODER14);
    GPIOB->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER10_0 | 
                     GPIO_MODER_MODER11_0 | GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0);
    GPIOB->BSRR = (1U << 14); GPIOB->BRR  = (1U << 13); GPIOB->BRR  = (1U << 12); 
}

void setup_tim14(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    TIM14->PSC = 4800 - 1; 
    TIM14->ARR = 10 - 1;   
    TIM14->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM14_IRQn, 1);
    NVIC_EnableIRQ(TIM14_IRQn);
    TIM14->CR1 |= TIM_CR1_CEN;
}

void init_uart(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART5EN; 
    GPIOD->MODER &= ~GPIO_MODER_MODER2; GPIOD->MODER |= GPIO_MODER_MODER2_1;        
    GPIOD->AFR[0] &= ~(0xF << (2 * 4)); GPIOD->AFR[0] |= (2 << (2 * 4));            
    GPIOC->MODER &= ~GPIO_MODER_MODER12; GPIOC->MODER |= GPIO_MODER_MODER12_1;       
    GPIOC->AFR[1] &= ~(0xF << ((12 - 8) * 4)); GPIOC->AFR[1] |= (2 << ((12 - 8) * 4));     
    USART5->BRR = SYSTEM_CLOCK / 115200;            
    USART5->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_RXNEIE; 
    NVIC_EnableIRQ(USART3_8_IRQn);
    NVIC_SetPriority(USART3_8_IRQn, 0); 
}

void init_motors(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    GPIOA->MODER &= ~((3 << 16) | (3 << 18)); GPIOA->MODER |= (1 << 16) | (2 << 18); 
    GPIOA->AFR[1] |= (2 << 4); GPIOC->MODER &= ~(3 << 18); GPIOC->MODER |= (1 << 18);     
    GPIOA->BSRR = (1 << 8); GPIOC->BRR  = (1 << 9); 
    GPIOC->MODER &= ~((3 << 12) | (3 << 14) | (3 << 16)); GPIOC->MODER |= (1 << 12) | (1 << 14) | (2 << 16);
    GPIOC->AFR[1] &= ~(0xF << 0);  
    GPIOC->BSRR = (1 << 7); GPIOC->BRR  = (1 << 6); 
    TIM1->PSC = 0; TIM1->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC1PE;
    TIM1->CCER  |= TIM_CCER_CC2E; TIM1->BDTR  |= TIM_BDTR_MOE; TIM1->CR1   |= TIM_CR1_CEN;
    TIM3->PSC = 0; TIM3->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
    TIM3->CCER  |= TIM_CCER_CC3E; TIM3->CR1   |= TIM_CR1_CEN;
}

void set_motor_a(uint32_t percent, uint8_t is_rev) {
    if (percent == 0) { TIM1->CCR2 = 0; TIM1->EGR |= TIM_EGR_UG; return; }
    GPIOC->BRR = (1 << 9); 
    if (is_rev) GPIOA->BRR = (1 << 8); else GPIOA->BSRR = (1 << 8); 
    uint32_t target_hz = percent * 2000; 
    uint32_t arr_val = (SYSTEM_CLOCK / target_hz) - 1;
    TIM1->ARR = arr_val; TIM1->CCR2 = (arr_val + 1) / 2; TIM1->EGR |= TIM_EGR_UG; 
}

void set_motor_b(uint32_t percent, uint8_t is_rev) {
    if (percent == 0) { TIM3->CCR3 = 0; TIM3->EGR |= TIM_EGR_UG; return; }
    GPIOC->BRR = (1 << 6); 
    if (is_rev) GPIOC->BRR = (1 << 7); else GPIOC->BSRR = (1 << 7); 
    uint32_t target_hz = percent * 2000; 
    uint32_t arr_val = (SYSTEM_CLOCK / target_hz) - 1;
    TIM3->ARR = arr_val; TIM3->CCR3 = (arr_val + 1) / 2; TIM3->EGR |= TIM_EGR_UG; 
}

void send_state_byte(void) {
    uint8_t state_byte = 0;
    if (game_active && (sensor_cooldown == 0)) state_byte |= (1 << 7);
    if (game_mode == MODE_PLAYER) state_byte |= (1 << 6);
    state_byte |= ((player_score & 0x07) << 3);
    state_byte |= (bot_score & 0x07);
    while (!(USART5->ISR & USART_ISR_TXE)); 
    USART5->TDR = state_byte;
}

//===========================================================================
// GRAPHICS & SPRITES
//===========================================================================

void SetPixel(int x, int y, uint8_t color) {
    if (x <= 0 || x >= MATRIX_WIDTH - 1 || y <= 0 || y >= MATRIX_HEIGHT - 1) return;
    
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

void DrawSprite5(int x, int y, const uint8_t sprite_data[5][5], uint8_t color) {
    for (int r = 0; r < 5; r++) {
        for (int c = 0; c < 5; c++) {
            if (sprite_data[r][c] != 0) SetPixel(x + c, y + r, color);
        }
    }
}

// Blocky 5x5 Font for Start Screen
const uint8_t s5_A[5][5] = {{0,7,7,7,0},{7,7,0,7,7},{7,7,7,7,7},{7,7,0,7,7},{7,7,0,7,7}};
const uint8_t s5_U[5][5] = {{7,7,0,7,7},{7,7,0,7,7},{7,7,0,7,7},{7,7,0,7,7},{0,7,7,7,0}};
const uint8_t s5_T[5][5] = {{7,7,7,7,7},{7,7,7,7,7},{0,0,7,0,0},{0,0,7,0,0},{0,0,7,0,0}};
const uint8_t s5_O[5][5] = {{0,7,7,7,0},{7,7,0,7,7},{7,7,0,7,7},{7,7,0,7,7},{0,7,7,7,0}};
const uint8_t s5_N[5][5] = {{7,7,0,0,7},{7,7,7,0,7},{7,0,7,0,7},{7,0,7,7,7},{7,0,0,7,7}};
const uint8_t s5_I[5][5] = {{7,7,7,7,7},{0,0,7,0,0},{0,0,7,0,0},{0,0,7,0,0},{7,7,7,7,7}};
const uint8_t s5_R[5][5] = {{7,7,7,7,0},{7,7,0,7,7},{7,7,7,7,0},{7,7,0,7,0},{7,7,0,0,7}};

// 4x5 Font for Win Screen Text
const uint8_t s4_H[5][4] = {{7,0,0,7},{7,0,0,7},{7,7,7,7},{7,0,0,7},{7,0,0,7}};
const uint8_t s4_U[5][4] = {{7,0,0,7},{7,0,0,7},{7,0,0,7},{7,0,0,7},{0,7,7,0}};
const uint8_t s4_M[5][4] = {{7,0,0,7},{7,7,7,7},{7,7,7,7},{7,0,0,7},{7,0,0,7}};
const uint8_t s4_A[5][4] = {{0,7,7,0},{7,0,0,7},{7,7,7,7},{7,0,0,7},{7,0,0,7}};
const uint8_t s4_N[5][4] = {{7,7,0,7},{7,7,0,7},{7,7,7,7},{7,0,7,7},{7,0,0,7}};
const uint8_t s4_W[5][4] = {{7,0,0,7},{7,0,0,7},{7,0,0,7},{7,7,7,7},{0,7,7,0}};
const uint8_t s4_I[5][4] = {{7,7,7,7},{0,7,7,0},{0,7,7,0},{0,7,7,0},{7,7,7,7}};
const uint8_t s4_S[5][4] = {{0,7,7,7},{7,0,0,0},{0,7,7,0},{0,0,0,7},{7,7,7,0}};
const uint8_t s4_B[5][4] = {{7,7,7,0},{7,0,0,7},{7,7,7,0},{7,0,0,7},{7,7,7,0}};
const uint8_t s4_O[5][4] = {{0,7,7,0},{7,0,0,7},{7,0,0,7},{7,0,0,7},{0,7,7,0}};
const uint8_t s4_T[5][4] = {{7,7,7,7},{0,7,7,0},{0,7,7,0},{0,7,7,0},{0,7,7,0}};

// Thick Arcade Numbers
const uint8_t sprite_0[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,0,7,7,7,7,7,7,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,7,7,7,7,7,7,0,0},{0,0,0,0,0,0,0,0,0,0}};
const uint8_t sprite_1[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,7,7,7,0,0,0,0},{0,0,7,7,7,7,0,0,0,0},{0,7,7,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,7,7,7,7,7,7,0,0},{0,0,7,7,7,7,7,7,0,0},{0,0,0,0,0,0,0,0,0,0}};
const uint8_t sprite_2[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,0,7,7,7,7,7,7,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,0,0,0,0,0,0,7,7,0},{0,0,0,0,0,0,7,7,0,0},{0,0,0,0,0,7,7,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,7,7,0,0,0,0,0},{0,0,7,7,0,0,0,0,0,0},{0,7,7,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,0,0}};
const uint8_t sprite_3[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,0,7,7,7,7,7,7,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,0,0,0,0,0,0,7,7,0},{0,0,0,0,7,7,7,7,0,0},{0,0,0,0,7,7,7,7,0,0},{0,0,0,0,0,0,0,7,7,0},{0,0,0,0,0,0,0,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,7,7,7,7,7,7,0,0},{0,0,0,0,0,0,0,0,0,0}};
const uint8_t sprite_4[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,7,7,0,0},{0,0,0,0,0,7,7,7,0,0},{0,0,0,0,7,7,7,7,0,0},{0,0,0,7,7,0,7,7,0,0},{0,0,7,7,0,0,7,7,0,0},{0,7,7,0,0,0,7,7,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,7,7,0,0},{0,0,0,0,0,0,7,7,0,0},{0,0,0,0,0,0,7,7,0,0},{0,0,0,0,0,0,7,7,0,0},{0,0,0,0,0,0,0,0,0,0}};
const uint8_t sprite_5[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,7,7,7,7,7,7,7,0},{0,7,7,0,0,0,0,0,0,0},{0,7,7,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,0,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,7,7,0},{0,0,0,0,0,0,0,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,7,7,7,7,7,7,0,0},{0,0,0,0,0,0,0,0,0,0}};
const uint8_t sprite_6[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,0,0,7,7,7,7,7,0,0},{0,0,7,7,7,7,7,7,7,0},{0,7,7,0,0,0,0,0,0,0},{0,7,7,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,7,7,7,7,7,7,0,0},{0,0,0,0,0,0,0,0,0,0}};
const uint8_t sprite_7[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,7,7,0},{0,0,0,0,0,0,7,7,0,0},{0,0,0,0,0,7,7,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,7,7,0,0,0,0,0},{0,0,7,7,0,0,0,0,0,0},{0,0,7,7,0,0,0,0,0,0},{0,0,7,7,0,0,0,0,0,0},{0,0,7,7,0,0,0,0,0,0},{0,0,7,7,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0}};
const uint8_t sprite_8[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,0,7,7,7,7,7,7,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,0,7,7,7,7,7,7,0,0},{0,0,7,7,7,7,7,7,0,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,7,7,7,7,7,7,0,0},{0,0,0,0,0,0,0,0,0,0}};
const uint8_t sprite_9[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,0,7,7,7,7,7,7,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,0,0,0,0,7,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,7,7,0},{0,0,0,0,0,0,0,7,7,0},{0,0,0,0,0,0,0,7,7,0},{0,7,7,7,7,7,7,7,0,0},{0,0,7,7,7,7,7,0,0,0},{0,0,0,0,0,0,0,0,0,0}};

const uint8_t (*sprite_numbers[])[NUM_WIDTH] = {
    sprite_0, sprite_1, sprite_2, sprite_3, sprite_4, sprite_5, sprite_6, sprite_7, sprite_8, sprite_9
};

// Computes a vertical offset for the bubble wave effect
int get_bubble_offset(int x_center) {
    int wave_x = (anim_tick / 3) % 50; 
    int dist = ABS(wave_x - x_center);
    if (dist == 0) return -2;
    if (dist == 1) return -1;
    return 0;
}

void start_screen(void) {
    int ya = 6;
    DrawSprite5(1,  ya + get_bubble_offset(3), s5_A, COLOR_RED);
    DrawSprite5(7,  ya + get_bubble_offset(9), s5_U, COLOR_RED);
    DrawSprite5(13, ya + get_bubble_offset(15), s5_T, COLOR_RED);
    DrawSprite5(19, ya + get_bubble_offset(21), s5_O, COLOR_RED);
    DrawSprite5(25, ya + get_bubble_offset(27), s5_N, COLOR_RED);
    
    int yb = 18;
    DrawSprite5(14, yb + get_bubble_offset(16), s5_A, COLOR_BLUE);
    DrawSprite5(20, yb + get_bubble_offset(22), s5_I, COLOR_BLUE);
    DrawSprite5(26, yb + get_bubble_offset(28), s5_R, COLOR_BLUE);  
}

void draw_zipper_border(uint8_t color) {
    int p = 0;
    int offset = (anim_tick / 4) % 2; // Shifts the zigzag pattern to rotate
    
    // Top Edge
    for (int x = 1; x <= 30; x++) { p++; SetPixel(x, ((p + offset) % 2 == 0) ? 1 : 2, color); }
    // Right Edge
    for (int y = 2; y <= 30; y++) { p++; SetPixel(((p + offset) % 2 == 0) ? 30 : 29, y, color); }
    // Bottom Edge
    for (int x = 29; x >= 1; x--) { p++; SetPixel(x, ((p + offset) % 2 == 0) ? 30 : 29, color); }
    // Left Edge
    for (int y = 29; y >= 2; y--) { p++; SetPixel(((p + offset) % 2 == 0) ? 1 : 2, y, color); }
}

void draw_win_screen(void) {
    // 1. Process and draw fireworks in the background
    for (int i = 0; i < NUM_FW; i++) {
        if (fws[i].frame == 0) {
            if ((rand() % 100) < 5) { 
                fws[i].x = 4 + (rand() % 24);
                fws[i].y = 4 + (rand() % 24);
                
                // Shift fireworks to the top/bottom edges to not obstruct text
                if (fws[i].x > 4 && fws[i].x < 28 && fws[i].y > 6 && fws[i].y < 24) {
                    if (rand()%2 == 0) fws[i].y = 2 + rand()%4;
                    else fws[i].y = 25 + rand()%4;
                }
                
                uint8_t colors[] = {1, 2, 3, 4, 5, 6, 7};
                fws[i].color = colors[rand() % 7];
                fws[i].max_r = 2 + (rand() % 5); 
                fws[i].frame = 1;
            }
        } else {
            int r = fws[i].frame / 3; 
            int x = fws[i].x, y = fws[i].y;
            uint8_t c = fws[i].color;
            
            if (r > fws[i].max_r) { fws[i].frame = 0; continue; }
            
            if (r == 0) {
                SetPixel(x, y, c);
            } else {
                int r_diag = (r * 7) / 10;
                if (r_diag == 0 && r > 0) r_diag = 1;
                
                SetPixel(x+r, y, c); SetPixel(x-r, y, c);
                SetPixel(x, y+r, c); SetPixel(x, y-r, c);
                SetPixel(x+r_diag, y+r_diag, c); SetPixel(x-r_diag, y+r_diag, c);
                SetPixel(x+r_diag, y-r_diag, c); SetPixel(x-r_diag, y-r_diag, c);
            }
            fws[i].frame++;
        }
    }

    uint8_t win_color = (player_score >= WINNING_SCORE) ? COLOR_BLUE : COLOR_RED;
    draw_zipper_border(win_color);
    
    // 3. Draw Winning Text with Bubble Wave
    int base_y1 = 10;
    int base_y2 = 17;

    if (player_score >= WINNING_SCORE) {
        DrawSprite(4,  base_y1 + get_bubble_offset(6),  CHAR_HEIGHT, CHAR_WIDTH, s4_H, COLOR_BLUE);
        DrawSprite(9,  base_y1 + get_bubble_offset(11), CHAR_HEIGHT, CHAR_WIDTH, s4_U, COLOR_BLUE);
        DrawSprite(14, base_y1 + get_bubble_offset(16), CHAR_HEIGHT, CHAR_WIDTH, s4_M, COLOR_BLUE);
        DrawSprite(19, base_y1 + get_bubble_offset(21), CHAR_HEIGHT, CHAR_WIDTH, s4_A, COLOR_BLUE);
        DrawSprite(24, base_y1 + get_bubble_offset(26), CHAR_HEIGHT, CHAR_WIDTH, s4_N, COLOR_BLUE);
    } else {
        DrawSprite(9,  base_y1 + get_bubble_offset(11), CHAR_HEIGHT, CHAR_WIDTH, s4_B, COLOR_RED);
        DrawSprite(14, base_y1 + get_bubble_offset(16), CHAR_HEIGHT, CHAR_WIDTH, s4_O, COLOR_RED);
        DrawSprite(19, base_y1 + get_bubble_offset(21), CHAR_HEIGHT, CHAR_WIDTH, s4_T, COLOR_RED);
    }
    
    DrawSprite(6,  base_y2 + get_bubble_offset(8),  CHAR_HEIGHT, CHAR_WIDTH, s4_W, win_color);
    DrawSprite(11, base_y2 + get_bubble_offset(13), CHAR_HEIGHT, CHAR_WIDTH, s4_I, win_color);
    DrawSprite(16, base_y2 + get_bubble_offset(18), CHAR_HEIGHT, CHAR_WIDTH, s4_N, win_color);
    DrawSprite(21, base_y2 + get_bubble_offset(23), CHAR_HEIGHT, CHAR_WIDTH, s4_S, win_color);
}

//===========================================================================
// TIM14 INTERRUPT (IR Sensors) - Shift Register Debounce
//===========================================================================

void TIM14_IRQHandler(void) {
    if (TIM14->SR & TIM_SR_UIF) {
        TIM14->SR &= ~TIM_SR_UIF;

        if (!game_active) return;

        static uint16_t p_db = 0xFFFF;
        static uint16_t b_db = 0xFFFF;

        // If in lockout, ignore sensors entirely and freeze history at unbroken
        if (sensor_cooldown > 0) {
            sensor_cooldown--;
            p_db = 0xFFFF; 
            b_db = 0xFFFF;
            if (sensor_cooldown == 0) send_state_byte(); 
            return;
        }

        // Read pins. 1 = Clear/Unbroken, 0 = Broken
        uint8_t p_read = (GPIOA->IDR & (1 << 12)) ? 1 : 0; 
        uint8_t b_read = (GPIOA->IDR & (1 << 11)) ? 1 : 0; 

        // Shift left and insert the new 1ms reading
        p_db = (p_db << 1) | p_read;
        b_db = (b_db << 1) | b_read;

        bool scored = false;
        
        // 0x07FF = 11 bits (0b0000_0111_1111_1111). 
        // We want the oldest bit to be 1 (unbroken), and the newest 10 bits to be 0 (solidly broken for 10ms).
        // If there is ANY noise (a '1' sneaks into the bottom 10 bits), this statement will reject it.
        if ((p_db & 0x07FF) == 0x0400) {
            player_score++;
            scored = true;
        } else if ((b_db & 0x07FF) == 0x0400) {
            bot_score++;
            scored = true;
        }

        // Only trigger the lockout if a clean goal was registered
        if (scored) {
            sensor_cooldown = GOAL_COOLDOWN_TICKS;
            
            // Wipe history immediately so concurrent or bouncy hits don't register
            p_db = 0xFFFF; 
            b_db = 0xFFFF;

            if (player_score >= WINNING_SCORE || bot_score >= WINNING_SCORE) {
                game_active = false;
            }
            send_state_byte(); 
        }
    }
}

//===========================================================================
// MAIN
//===========================================================================

int main(void) {
    init_clock(); 
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN;

    init_matrix_gpio(); 
    setup_tim14();
    init_sensors();      
    init_controls();  
    init_adc();
    init_uart();
    init_motors();

    init_oled_pins();
    spi1_init_oled();
    
    uint8_t ui_state = UI_STATE_SPLASH;
    uint8_t selected_mode = MODE_BOT; 
    uint8_t oled_needs_update = 1;
    
    uint8_t pc2_last_state = 1; 
    uint32_t pc2_debounce = 0; 
    
    // Animation Trackers
    const int bounce_lut[12] = {0, -1, -2, -3, -4, -5, -6, -5, -4, -3, -2, -1};
    int last_p_score = 0;
    int last_b_score = 0;
    int p_bounce = 0;
    int b_bounce = 0;
    
    send_state_byte();

    while (1) {
        // --- 1. RX: Motor Control ---
        if (TEST_MODE == 0) {
            watchdog_timer++;
            if (watchdog_timer > WATCHDOG_MAX) {
                set_motor_a(0, 0);
                set_motor_b(0, 0);
                watchdog_timer = WATCHDOG_MAX; 
            }
        }

        // --- 2. Hardware Button & Joystick Handling ---
        if (pc2_debounce > 0) pc2_debounce--;

        if (ui_state == UI_STATE_MENU) {
            uint16_t joy_y = read_adc();
            if (joy_y < 1000 && selected_mode != MODE_PLAYER) {
                selected_mode = MODE_PLAYER;
                oled_needs_update = 1;
            } else if (joy_y > 3000 && selected_mode != MODE_BOT) {
                selected_mode = MODE_BOT;
                oled_needs_update = 1;
            }
        }

        uint8_t pc2_current = (GPIOC->IDR & (1 << 2)) ? 1 : 0;
        
        if (pc2_current == 0 && pc2_last_state == 1 && pc2_debounce == 0) {
            pc2_debounce = 5; 
            
            if (ui_state == UI_STATE_SPLASH) {
                ui_state = UI_STATE_MENU;
                oled_needs_update = 1;
            } else if (ui_state == UI_STATE_MENU) {
                ui_state = UI_STATE_PLAY;
                game_mode = selected_mode;
                game_active = true;
                player_score = 0;
                bot_score = 0;
                last_p_score = 0;
                last_b_score = 0;
                p_bounce = 0;
                b_bounce = 0;
                sensor_cooldown = 0; // Wipe any lingering cooldown from the previous game
                for(int i=0; i<NUM_FW; i++) fws[i].frame = 0; // Clear fireworks
                oled_needs_update = 1;
                send_state_byte(); 
            } else if (ui_state == UI_STATE_PLAY) {
                ui_state = UI_STATE_SPLASH;
                game_active = false;
                oled_needs_update = 1;
                send_state_byte();
            }
        }
        pc2_last_state = pc2_current;
        
        // --- 3. OLED Menu Rendering ---
        if (oled_needs_update) {
            oled_needs_update = 0;
            spi_cmd(0x01); 
            delay_ms(2); 

            if (ui_state == UI_STATE_SPLASH) {
                spi1_display1("Push Button     ");
                spi1_display2("to start...     ");
            } 
            else if (ui_state == UI_STATE_MENU) {
                if (selected_mode == MODE_PLAYER) {
                    spi1_display1("> Human         ");
                    spi1_display2("  Bot           ");
                } else {
                    spi1_display1("  Human         ");
                    spi1_display2("> Bot           ");
                }
            } 
            else if (ui_state == UI_STATE_PLAY) {
                if (game_mode == MODE_PLAYER) {
                    spi1_display1("Playing:        ");
                    spi1_display2("Human Mode      ");
                } else {
                    spi1_display1("Playing:        ");
                    spi1_display2("Bot Mode        ");
                }
            }
        }

        // --- 4. LED Matrix Rendering & Score Animation ---
        ClearScreen();

        if (ui_state == UI_STATE_SPLASH || ui_state == UI_STATE_MENU) {
            start_screen(); 
        } 
        else if (ui_state == UI_STATE_PLAY) {
            if (!game_active && (player_score >= WINNING_SCORE || bot_score >= WINNING_SCORE)) {
                draw_win_screen();
            } else {
                if (player_score > last_p_score) { p_bounce = 11; last_p_score = player_score; }
                if (bot_score > last_b_score) { b_bounce = 11; last_b_score = bot_score; }
                
                if (p_bounce > 0) p_bounce--;
                if (b_bounce > 0) b_bounce--;

                int p_y = 9 + bounce_lut[p_bounce];
                int b_y = 9 + bounce_lut[b_bounce];

                if (player_score < 10)
                    DrawSprite(3, p_y, NUM_HEIGHT, NUM_WIDTH, sprite_numbers[player_score], COLOR_BLUE);
                if (bot_score < 10)
                    DrawSprite(19, b_y, NUM_HEIGHT, NUM_WIDTH, sprite_numbers[bot_score], COLOR_RED);
            }
        }
        
        // --- 5. GAME ENGINE LOOP ---
        anim_tick++;
        delay_ms(16); 
    }
}