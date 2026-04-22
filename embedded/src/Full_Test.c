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
#define GOAL_COOLDOWN_TICKS 500 // Scaled up for 10kHz TIM14 

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

#define TEST_MODE 0     // Turns the safety watchdog back on
#define WATCHDOG_MAX 25 // 25 loops * 16ms = ~400ms timeout
#define SYSTEM_CLOCK 48000000 

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

// UART / Motor State Variables
volatile uint8_t pending_motor = 0;
volatile uint8_t pending_dir = 0;
volatile uint32_t watchdog_timer = 0;

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
// Data: PA4(R1), PA5(G1), PA6(B1), PA7(R2), PC4(G2), PC5(B2)
// Control: PB12(CLK), PB13(LAT), PB14(OE)
// Address: PB0(A), PB1(B), PB10(C), PB11(D)

static inline void Matrix_Scan(uint8_t row) {
    // 1. SHIFT DATA IN BACKGROUND
    for (int col = 0; col < 32; col++) {
        uint8_t p = canvas[row][col];
        
        GPIOA->BSRR = ((p & 0xF) << 4) | ((~(p & 0xF) & 0xF) << 20);
        GPIOC->BSRR = ((p & 0x30)) | ((~(p & 0x30) & 0x30) << 16);

        // Clock Pulse (PB12)
        GPIOB->BSRR = (1U << 12); 
        GPIOB->BRR  = (1U << 12); 
    }

    // 2. BLANK SCREEN: OE HIGH (PB14)
    GPIOB->BSRR = (1U << 14); 

    // 3. LATCH DATA: LAT HIGH then LOW (PB13)
    GPIOB->BSRR = (1U << 13);
    for(volatile int i = 0; i < 2; i++); 
    GPIOB->BRR  = (1U << 13);

    // 4. UPDATE ADDRESS: A:PB0, B:PB1, C:PB10, D:PB11
    uint32_t b_set = 0;
    if (row & 0x01) b_set |= (1U << 0);
    if (row & 0x02) b_set |= (1U << 1);
    if (row & 0x04) b_set |= (1U << 10);
    if (row & 0x08) b_set |= (1U << 11);
    
    GPIOB->BRR = (1U << 0) | (1U << 1) | (1U << 10) | (1U << 11);
    GPIOB->BSRR = b_set;

    // 5. DISPLAY ENABLE: OE LOW (PB14)
    GPIOB->BRR = (1U << 14);
    
    // 6. ROW DWELL TIME (Brightness)
    for(volatile int i = 0; i < 400; i++); 
}

//===========================================================================
// HELPER FUNCTIONS (UART Polling & Hijacked Delay)
//===========================================================================

void USART3_8_IRQHandler(void) {
    // 1. Clear Overrun Error (ORE) safely
    if (USART5->ISR & USART_ISR_ORE) {
        USART5->ICR |= USART_ICR_ORECF;
    }

    // 2. Read incoming motor commands instantly
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
// ADC (Joystick VRY -> PC0)
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

//===========================================================================
// OLED PINS & SPI (SEH1602A 3V)
//===========================================================================

void init_oled_pins(void) {
    GPIOA->MODER &= ~(GPIO_MODER_MODER15);
    GPIOA->MODER |= (GPIO_MODER_MODER15_0);
    GPIOC->MODER &= ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11);
    GPIOC->MODER |= (GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0);
    GPIOC->BSRR = (1U << 11); 
    GPIOA->BRR = (1U << 15);  
}

void spi_send_10bit(uint16_t data) {
    GPIOC->BRR = (1U << 11); 
    small_delay();
    for (int i = 9; i >= 0; i--) {
        if ((data >> i) & 1) GPIOC->BSRR = (1U << 10);
        else                 GPIOC->BRR = (1U << 10);
        small_delay();
        GPIOA->BSRR = (1U << 15); 
        small_delay();
        GPIOA->BRR = (1U << 15);  
        small_delay();
    }
    GPIOC->BSRR = (1U << 11); 
    small_delay();
}

void spi_cmd(unsigned int data) { 
    spi_send_10bit(data & 0xFF); 
    delay_ms(1);
}

void spi_data(unsigned int data) { 
    spi_send_10bit(data | 0x200); 
    delay_ms(1);
}

void spi1_init_oled(void) {
    delay_ms(100); 
    spi_cmd(0x38); 
    spi_cmd(0x08); 
    spi_cmd(0x17); 
    spi_cmd(0x01); 
    delay_ms(5);   
    spi_cmd(0x06); 
    spi_cmd(0x02); 
    spi_cmd(0x0C); 
}

void spi1_display1(const char *string) {
    spi_cmd(0x02); 
    while(*string != '\0') {
        spi_data(*string);
        string++;
    }
}

void spi1_display2(const char *string) {
    spi_cmd(0xC0); 
    while(*string != '\0') {
        spi_data(*string);
        string++;
    }
}

//===========================================================================
// HARDWARE SETUP: CONTROLS & TIMERS
//===========================================================================

void init_controls(void) {
    // PC2 Input (UI Select)
    GPIOC->MODER &= ~(GPIO_MODER_MODER2);
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR2);
    GPIOC->PUPDR |= (GPIO_PUPDR_PUPDR2_0);
}

void init_sensors(void) {
    // PA11 (Red), PA12 (Blue)
    GPIOA->MODER &= ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER12);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR11 | GPIO_PUPDR_PUPDR12);
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR11_0 | GPIO_PUPDR_PUPDR12_0); // Pull-ups
}

void init_matrix_gpio(void) {
    // DATA PA4-7
    GPIOA->MODER &= ~(0xFF00);
    GPIOA->MODER |= 0x5500;
    
    // DATA PC4-5
    GPIOC->MODER &= ~(0xF00);
    GPIOC->MODER |= 0x500;

    // PORT B: ADDR A(0), B(1), C(10), D(11), CLK(12), LAT(13), OE(14)
    GPIOB->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER10 | 
                      GPIO_MODER_MODER11 | GPIO_MODER_MODER12 | GPIO_MODER_MODER13 | GPIO_MODER_MODER14);
    GPIOB->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER10_0 | 
                     GPIO_MODER_MODER11_0 | GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0);

    // Start Safe
    GPIOB->BSRR = (1U << 14); // Screen Blanked
    GPIOB->BRR  = (1U << 13); // Latch Low
    GPIOB->BRR  = (1U << 12); // Clock Low
}

void setup_tim14(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    TIM14->PSC = 4800 - 1; // 10kHz tick for fast sensor response
    TIM14->ARR = 10 - 1;   // Fire interrupt every 1ms
    TIM14->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM14_IRQn, 1);
    NVIC_EnableIRQ(TIM14_IRQn);
    TIM14->CR1 |= TIM_CR1_CEN;
}

//===========================================================================
// HARDWARE SETUP: MOTORS & UART
//===========================================================================

void init_uart(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART5EN; 

    // Setup PD2 as RX (Alternate Function 2)
    GPIOD->MODER &= ~GPIO_MODER_MODER2;
    GPIOD->MODER |= GPIO_MODER_MODER2_1;        
    GPIOD->AFR[0] &= ~(0xF << (2 * 4));         
    GPIOD->AFR[0] |= (2 << (2 * 4));            

    // Setup PC12 as TX (Alternate Function 2)
    GPIOC->MODER &= ~GPIO_MODER_MODER12;
    GPIOC->MODER |= GPIO_MODER_MODER12_1;       
    GPIOC->AFR[1] &= ~(0xF << ((12 - 8) * 4));  
    GPIOC->AFR[1] |= (2 << ((12 - 8) * 4));     

    USART5->BRR = SYSTEM_CLOCK / 115200;            
    
    // Enable Receiver, Transmitter, UART, AND RX Interrupts
    USART5->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_RXNEIE; 
    
    // Enable the Interrupt in the NVIC (USART5 shares IRQ line 3-8 on STM32F0)
    NVIC_EnableIRQ(USART3_8_IRQn);
    NVIC_SetPriority(USART3_8_IRQn, 0); // Highest priority to never miss a motor command
}

void init_motors(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    
    // Motor A: PA8 (DIR), PC9 (ENA), PA9 (PUL)
    GPIOA->MODER &= ~((3 << 16) | (3 << 18));
    GPIOA->MODER |= (1 << 16) | (2 << 18); 
    GPIOA->AFR[1] |= (2 << 4);     
    GPIOC->MODER &= ~(3 << 18);
    GPIOC->MODER |= (1 << 18);     
    
    GPIOA->BSRR = (1 << 8); // DIR High
    GPIOC->BRR  = (1 << 9); // ENA Low (Enabled)

    // Motor B: PC7 (DIR), PC6 (ENA), PC8 (PUL)
    GPIOC->MODER &= ~((3 << 12) | (3 << 14) | (3 << 16));
    GPIOC->MODER |= (1 << 12) | (1 << 14) | (2 << 16);
    GPIOC->AFR[1] &= ~(0xF << 0);  
    
    GPIOC->BSRR = (1 << 7); // DIR High
    GPIOC->BRR  = (1 << 6); // ENA Low (Enabled)

    TIM1->PSC = 0;
    TIM1->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC1PE;
    TIM1->CCER  |= TIM_CCER_CC2E;
    TIM1->BDTR  |= TIM_BDTR_MOE;   
    TIM1->CR1   |= TIM_CR1_CEN;

    TIM3->PSC = 0;
    TIM3->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
    TIM3->CCER  |= TIM_CCER_CC3E;  
    TIM3->CR1   |= TIM_CR1_CEN;
}

void set_motor_a(uint32_t percent, uint8_t is_rev) {
    if (percent == 0) {
        TIM1->CCR2 = 0; 
        TIM1->EGR |= TIM_EGR_UG;
        return;
    }
    GPIOC->BRR = (1 << 9); // Keep ENA Low
    if (is_rev) GPIOA->BRR = (1 << 8);  
    else        GPIOA->BSRR = (1 << 8); 

    uint32_t target_hz = percent * 2000; 
    uint32_t arr_val = (SYSTEM_CLOCK / target_hz) - 1;
    TIM1->ARR = arr_val;
    TIM1->CCR2 = (arr_val + 1) / 2; 
    TIM1->EGR |= TIM_EGR_UG; 
}

void set_motor_b(uint32_t percent, uint8_t is_rev) {
    if (percent == 0) {
        TIM3->CCR3 = 0; 
        TIM3->EGR |= TIM_EGR_UG;
        return;
    }
    GPIOC->BRR = (1 << 6); // Keep ENA Low
    if (is_rev) GPIOC->BRR = (1 << 7);  
    else        GPIOC->BSRR = (1 << 7); 

    uint32_t target_hz = percent * 2000; 
    uint32_t arr_val = (SYSTEM_CLOCK / target_hz) - 1;
    TIM3->ARR = arr_val;
    TIM3->CCR3 = (arr_val + 1) / 2; 
    TIM3->EGR |= TIM_EGR_UG; 
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

const uint8_t sprite_A[CHAR_HEIGHT][CHAR_WIDTH] = { {0, 7, 7, 0}, {7, 0, 0, 7}, {7, 7, 7, 7}, {7, 0, 0, 7}, {7, 0, 0, 7}};
const uint8_t sprite_U[CHAR_HEIGHT][CHAR_WIDTH] = { {7, 0, 0, 7}, {7, 0, 0, 7}, {7, 0, 0, 7}, {7, 0, 0, 7}, {0, 7, 7, 0}};
const uint8_t sprite_T[CHAR_HEIGHT][CHAR_WIDTH] = { {7, 7, 7, 7}, {0, 7, 7, 0}, {0, 7, 7, 0}, {0, 7, 7, 0}, {0, 7, 7, 0}};
const uint8_t sprite_O[CHAR_HEIGHT][CHAR_WIDTH] = { {0, 7, 7, 0}, {7, 0, 0, 7}, {7, 0, 0, 7}, {7, 0, 0, 7}, {0, 7, 7, 0}};
const uint8_t sprite_N[CHAR_HEIGHT][CHAR_WIDTH] = { {7, 7, 0, 7}, {7, 7, 0, 7}, {7, 7, 7, 7}, {7, 0, 7, 7}, {7, 0, 0, 7}};
const uint8_t sprite_I[CHAR_HEIGHT][CHAR_WIDTH] = { {7, 7, 7, 7}, {0, 7, 7, 0}, {0, 7, 7, 0}, {0, 7, 7, 0}, {7, 7, 7, 7}};
const uint8_t sprite_R[CHAR_HEIGHT][CHAR_WIDTH] = { {7, 7, 7, 0}, {7, 0, 0, 7}, {7, 7, 7, 0}, {7, 0, 7, 0}, {7, 0, 0, 7}};

const uint8_t sprite_0[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,0,0}};
const uint8_t sprite_1[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,7,7,0,0,0,0},{0,0,0,0,0,0,0,0,0,0}};
const uint8_t sprite_2[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,7,7,7,7,7,7,7,7,0},{0,7,0,0,0,0,0,0,0,0},{0,7,0,0,0,0,0,0,0,0},{0,7,0,0,0,0,0,0,0,0},{0,7,0,0,0,0,0,0,0,0},{0,7,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,0,0}};
const uint8_t sprite_3[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,0,0}};
const uint8_t sprite_4[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,0,0}};
const uint8_t sprite_5[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,0,0,0,0,0,0,0,0},{0,7,0,0,0,0,0,0,0,0},{0,7,0,0,0,0,0,0,0,0},{0,7,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,0,0}};
const uint8_t sprite_6[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,0,0,0,0,0,0,0,0},{0,7,0,0,0,0,0,0,0,0},{0,7,0,0,0,0,0,0,0,0},{0,7,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,0,0}};
const uint8_t sprite_7[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,0,0}};
const uint8_t sprite_8[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,7,7,7,7,7,7,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,0,0}};
const uint8_t sprite_9[NUM_HEIGHT][NUM_WIDTH] = {{0,0,0,0,0,0,0,0,0,0},{0,7,7,7,7,7,7,7,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,0,0,0,0,0,0,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,0,0,0,0,0,0,0,7,0},{0,7,7,7,7,7,7,7,7,0},{0,0,0,0,0,0,0,0,0,0}};

const uint8_t (*sprite_numbers[])[NUM_WIDTH] = {
    sprite_0, sprite_1, sprite_2, sprite_3, sprite_4, sprite_5, sprite_6, sprite_7, sprite_8, sprite_9
};

void start_screen(void) {
    DrawSprite(3, 4, CHAR_HEIGHT, CHAR_WIDTH, sprite_A, COLOR_RED);        
    DrawSprite(8, 4, CHAR_HEIGHT, CHAR_WIDTH, sprite_U, COLOR_RED);
    DrawSprite(13, 4, CHAR_HEIGHT, CHAR_WIDTH, sprite_T, COLOR_RED);
    DrawSprite(18, 4, CHAR_HEIGHT, CHAR_WIDTH, sprite_O, COLOR_RED);
    DrawSprite(23, 4, CHAR_HEIGHT, CHAR_WIDTH, sprite_N, COLOR_RED);
    
    DrawSprite(17, 12, CHAR_HEIGHT, CHAR_WIDTH, sprite_A, COLOR_BLUE);
    DrawSprite(22, 12, CHAR_HEIGHT, CHAR_WIDTH, sprite_I, COLOR_BLUE);
    DrawSprite(27, 12, CHAR_HEIGHT, CHAR_WIDTH, sprite_R, COLOR_BLUE);  
}

void draw_win_screen(void) {
    if (player_score >= WINNING_SCORE) {
        DrawSprite(11, 9, NUM_HEIGHT, NUM_WIDTH, sprite_numbers[player_score], COLOR_BLUE);
        for(int i = 0; i < MATRIX_WIDTH; i++) {
            SetPixel(i, 0, COLOR_BLUE);                 
            SetPixel(i, MATRIX_HEIGHT - 1, COLOR_BLUE);  
            SetPixel(0, i, COLOR_BLUE);                 
            SetPixel(MATRIX_WIDTH - 1, i, COLOR_BLUE);   
        }
    } else if (bot_score >= WINNING_SCORE) {
        DrawSprite(11, 9, NUM_HEIGHT, NUM_WIDTH, sprite_numbers[bot_score], COLOR_RED);
        for(int i = 0; i < MATRIX_WIDTH; i++) {
            SetPixel(i, 0, COLOR_RED);
            SetPixel(i, MATRIX_HEIGHT - 1, COLOR_RED);
            SetPixel(0, i, COLOR_RED);
            SetPixel(MATRIX_WIDTH - 1, i, COLOR_RED);
        }
    }
}

//===========================================================================
// TIM14 INTERRUPT (IR Sensors - Trigger on Exit)
//===========================================================================

void TIM14_IRQHandler(void) {
    if (TIM14->SR & TIM_SR_UIF) {
        TIM14->SR &= ~TIM_SR_UIF;

        if (!game_active) return;

        // IR Sensors are Active Low: 0V means beam is broken
        bool player_is_broken = (GPIOA->IDR & (1 << 12)) == 0; 
        bool bot_is_broken    = (GPIOA->IDR & (1 << 11)) == 0; 

        static bool player_was_broken = false;
        static bool bot_was_broken = false;

        if (player_is_broken) player_was_broken = true;
        if (bot_is_broken)    bot_was_broken = true;

        if (sensor_cooldown > 0) {
            sensor_cooldown--;
            if (sensor_cooldown == 0) send_state_byte(); 
            return;
        }

        bool scored = false;
        
        // Wait until the puck LEAVES the beam to score
        if (player_was_broken && !player_is_broken) {
            player_score++;
            scored = true;
            player_was_broken = false;
        } 
        else if (bot_was_broken && !bot_is_broken) {
            bot_score++;
            scored = true;
            bot_was_broken = false;
        }

        if (scored) {
            sensor_cooldown = GOAL_COOLDOWN_TICKS;
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
    // 1. Boot up to 48 MHz 
    init_clock(); 
    
    // Enable GPIO Ports
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN;

    // Hardware Initialization
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

        // --- 4. LED Matrix Rendering ---
        ClearScreen();

        if (ui_state == UI_STATE_SPLASH || ui_state == UI_STATE_MENU) {
            start_screen(); 
        } 
        else if (ui_state == UI_STATE_PLAY) {
            if (!game_active && (player_score >= WINNING_SCORE || bot_score >= WINNING_SCORE)) {
                draw_win_screen();
            } else {
                if (player_score < 10)
                    DrawSprite(2, 9, NUM_HEIGHT, NUM_WIDTH, sprite_numbers[player_score], COLOR_BLUE);
                if (bot_score < 10)
                    DrawSprite(18, 9, NUM_HEIGHT, NUM_WIDTH, sprite_numbers[bot_score], COLOR_RED);
            }
        }
        
        // --- 5. GAME ENGINE LOOP ---
        // This powers the Matrix. Instead of freezing, it repaints the screen rapidly.
        delay_ms(16); 
    }
}