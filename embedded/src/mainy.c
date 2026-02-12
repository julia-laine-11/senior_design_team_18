
// //Include Files
// #include "stm32f0xx.h"
// #include <math.h>   // for M_PI
// #include <stdint.h>
// #include <stdio.h>
// #include <stdbool.h>
// #include <stddef.h>
// #include <stdlib.h>
// #include <time.h>
// //Pinouts
// /*
// //Programming Pins
// PD2     -> PROGRAMMER
// PC12    -> PROGRAMMER

// //Joystick Pins
// PA1:2   -> X AND Y AXIS CONTROL

// //Piezoelectric Buzzer
// PA4 DAC Output to Buzzer

// //Score Display Pins
// PA15    -> PIN 16 LCD
// PA7     -> PIN 14 LCD
// PA5     -> PIN 12 LCD

// //LED Array Pins
// PC0  R1
// PC1  B1
// PC2  G1
// PC3  R2
// PC4  B2
// PC5  G2
// PC6  A
// PC7  B
// PC8  C
// PC9  D
// PC10  OE
// PC11  LAT
// PC13  CLK
// */

// //Function Declarations
// //Univeral Functions
// void internal_clock();

// //adc (controls) support.c
// extern void setup_adc();
// extern double read_adc_channel1();
// extern double read_adc_channel2();
// //spi (lcd) support.c
// extern void init_spi1();
// extern void spi_cmd(unsigned int data);
// extern void spi_data(unsigned int data);
// extern void spi1_init_oled();
// //Piezo Functions
// void setup_dac(void);
// //void setup_button_interrupt(void);
// void generateTone(uint16_t duration_ms);
// void delay_ms(uint32_t ms);
// //void test();
// //LED Functions:
// void init_matrix_gpio(void);
// void init_refresh_timer(void);
// void ClearScreen(uint8_t color_value);
// void SetPixel(int x, int y, uint8_t color_value);
// void DrawSprite(int start_x, int start_y, int height, int width, const uint8_t sprite_data[][width]); // Added prototype
// static void Matrix_Scan(void);
// //Function Definitions
// extern void nano_wait(int t);
// // extern void delay_ms(uint32_t ms);
// extern void small_delay(void);
// // //variables
// // uint8_t col          = 0;           
// // uint8_t mode         = 'A';         //this is for the display
// extern int8_t thrust_down;           //same as before
// extern int8_t thrust_side;           //or do we need 2, one for left and one for right?
// // int16_t fuel         = 800;
// // int16_t alt          = 4500;
// // int16_t velo_down    = 0;
// // int16_t velo_side    = 0;
// extern uint16_t display[34];
// int score = 0;
// int time_left = 15;
// volatile bool game_running = false;
// bool game_started = false;

// //===========================================================================
// // enable ports
// //===========================================================================
// extern void enable_ports();
// //===========================================================================
// // Controls timer
// //===========================================================================
// extern void setup_tim14();
// //===========================================================================
// // Configure the SPI1 peripheral for OLED
// //===========================================================================
// extern void spi1_display1(const char *string);
// extern void spi1_display2(const char *string);
// extern void spi1_setup_dma(void);
// extern void spi1_enable_dma(void);
// //===========================================================================
// // Scoreboard Display, some game logic to be moved to main()
// //===========================================================================
// // simulate score update
// //Never used
// extern void update_score(void);
// extern void game_over(const char *end);
// // crash subtraction
// //never used
// extern void crash(void);
// extern void TIM17_IRQHandler(void);
// extern void init_tim17(void);
// //===========================================================================
// // Configure Piezoelectric Buzzer
// //===========================================================================
// //Used for button testing
// // void test()
// // {
// // RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable GPIOA
// // GPIOA->MODER |= GPIO_MODER_MODER4_0; //testing for piezo without dac on pa4
// // setup_button_interrupt();
// // }
// // void EXTI2_3_IRQHandler(void){
// // if (EXTI->PR && EXTI_PR_PR2) { // Check if EXTI0 caused the interrupt
// // EXTI->PR |= EXTI_PR_PR2; // Clear the pending interrupt
// // generateTone(200); // Buzz piezo for 200 ms
// // }
// // }
// // void setup_button_interrupt(void) {
// //     RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable GPIOB clock
// //     GPIOB->MODER &= ~(3 << (2 * 2));
// //     GPIOB->PUPDR|= (2 << (2 * 2)); // Pull-down on PB2
// //
// //     // Configure EXTI line 2 for PB2
// //     RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; // Enable SYSCFG
// //     SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PB;
// //     EXTI->IMR |= EXTI_IMR_MR2; // Unmask EXTI0
// //     EXTI->RTSR |=EXTI_RTSR_TR2; // Trigger on falling edge
// //
// //     NVIC_EnableIRQ(EXTI2_3_IRQn); // Enable interrupt in NVIC
// // }
// // void setup_dac(void) {
// //     RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable GPIOA
// //     GPIOA->MODER |= (3 << (4 * 2)); // Set PA4 to analog mode
// //     RCC->APB1ENR |= RCC_APB1ENR_DACEN; // Enable DAC clock
// //     DAC->CR &= ~DAC_CR_EN1; // Ensure DAC is off
// //     DAC->CR &= ~DAC_CR_TEN1; // Disable trigger
// //     DAC->CR |= DAC_CR_EN1; // Enable DAC
// // }
// extern void generateTone(uint16_t duration_ms);
// //Defined in support.c
// // void delay_ms(uint32_t ms) {
// // for (uint32_t i = 0; i < ms * 4800; i++) {
// // __asm__("nop");
// // }
// // }
// //===========================================================================
// // LED Array Logic
// //===========================================================================
// //Led Variables
// //LED Vars
// //LED Constants
// // Matrix Constants


// //Matrix Parameters

// //Top Left: (0,0), Bottom Right: (31,31)
// #define MATRIX_HEIGHT       32
// #define MATRIX_WIDTH        32
// #define MATRIX_SCAN_ROWS    (MATRIX_HEIGHT / 2)

// // Color Constants Definitions
// #define COLOR_BLACK   0   // Value 0: R=0, G=0, B=0
// #define COLOR_RED     1   // Value 1: R=1, G=0, B=0
// #define COLOR_BLUE    2   // Value 2: R=0, G=0, B=1
// #define COLOR_GREEN   4   // Value 4: R=0, G=1, B=0
// #define COLOR_PURPLE  3   // (COLOR_RED | COLOR_BLUE)    // Value 3: R=1, G=0, B=1
// #define COLOR_YELLOW  5   // (COLOR_RED | COLOR_GREEN)   // Value 5: R=1, G=1, B=0
// #define COLOR_CYAN    6   // (COLOR_BLUE | COLOR_GREEN)  // Value 6: R=0, G=1, B=1
// #define COLOR_WHITE   7   // (COLOR_RED | COLOR_BLUE | COLOR_GREEN) // Value 7: R=1, G=1, B=1
// //Global Variables
// volatile uint8_t canvas[MATRIX_SCAN_ROWS][MATRIX_WIDTH];
// static volatile uint8_t current_display_row = 0;

// // Sprite Arrays

// // Charachter Sprite Parameters
// #define CHAR_HEIGHT 5
// #define CHAR_WIDTH 4

// // Charachter Sprites

// //A
// const uint8_t sprite_A[CHAR_HEIGHT][CHAR_WIDTH] = {
//     {0, 7, 7, 0}, // White=7
//     {7, 0, 0, 7},
//     {7, 7, 7, 7},
//     {7, 0, 0, 7},
//     {7, 0, 0, 7},
// };

// //U
// const uint8_t sprite_U[CHAR_HEIGHT][CHAR_WIDTH] = {
//     {7, 0, 0, 7}, // White=7
//     {7, 0, 0, 7},
//     {7, 0, 0, 7},
//     {7, 0, 0, 7},
//     {0, 7, 7, 0},
// };

// //T
// const uint8_t sprite_T[CHAR_HEIGHT][CHAR_WIDTH] = {
//     {7, 7, 7, 7}, // White=7
//     {0, 7, 7, 0},
//     {0, 7, 7, 0},
//     {0, 7, 7, 0},
//     {0, 7, 7, 0},
// };

// //O
// const uint8_t sprite_O[CHAR_HEIGHT][CHAR_WIDTH] = {
//     {0, 7, 7, 0}, // White=7
//     {7, 0, 0, 7},
//     {7, 0, 0, 7},
//     {7, 0, 0, 7},
//     {0, 7, 7, 0},
// };

// //N
// const uint8_t sprite_N[CHAR_HEIGHT][CHAR_WIDTH] = {
//     {7, 7, 0, 7}, // White=7
//     {7, 7, 0, 7},
//     {7, 7, 7, 7},
//     {7, 0, 7, 7},
//     {7, 0, 0, 7},
// };

// //I
// const uint8_t sprite_I[CHAR_HEIGHT][CHAR_WIDTH] = {
//     {7, 7, 7, 7}, // White=7
//     {0, 7, 7, 0},
//     {0, 7, 7, 0},
//     {0, 7, 7, 0},
//     {7, 7, 7, 7},
// };

// //R
// const uint8_t sprite_R[CHAR_HEIGHT][CHAR_WIDTH] = {
//     {7, 7, 7, 0}, // White=7
//     {7, 0, 0, 7},
//     {7, 7, 7, 0},
//     {7, 0, 7, 0},
//     {7, 0, 0, 7},
// };

// //H
// const uint8_t sprite_H[CHAR_HEIGHT][CHAR_WIDTH] = {
//     {7, 0, 0, 7}, // White=7
//     {7, 0, 0, 7},
//     {7, 7, 7, 7},
//     {7, 0, 0, 7},
//     {7, 0, 0, 7},
// };

// //C
// const uint8_t sprite_C[CHAR_HEIGHT][CHAR_WIDTH] = {
//     {7, 7, 7, 7}, // White=7
//     {7, 0, 0, 0},
//     {7, 0, 0, 0},
//     {7, 0, 0, 0},
//     {7, 7, 7, 7},
// };

// //W
// const uint8_t sprite_W[CHAR_HEIGHT][CHAR_WIDTH] = {
//     {7, 0, 0, 7}, // White=7
//     {7, 0, 0, 7},
//     {7, 0, 0, 7},
//     {7, 7, 7, 7},
//     {7, 0, 0, 7},
// };

// //S
// const uint8_t sprite_S[CHAR_HEIGHT][CHAR_WIDTH] = {
//     {7, 7, 7, 7}, // White=7
//     {7, 0, 0, 0},
//     {7, 7, 7, 7},
//     {0, 0, 0, 7},
//     {7, 7, 7, 7},
// };


// //Numbers
// //Number Sprite Parameters
// #define NUM_HEIGHT 14
// #define NUM_WIDTH 10

// //Number Sprites
// const uint8_t sprite_0[NUM_HEIGHT][NUM_WIDTH] = {
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // White=7
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // White=7
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
// };

// const uint8_t sprite_1[NUM_HEIGHT][NUM_WIDTH] = {
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // White=7
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // White=7
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
// };

// const uint8_t sprite_2[NUM_HEIGHT][NUM_WIDTH] = {
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // White=7
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // White=7
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
// };

// const uint8_t sprite_3[NUM_HEIGHT][NUM_WIDTH] = {
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // White=7
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // White=7
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
// };

// const uint8_t sprite_4[NUM_HEIGHT][NUM_WIDTH] = {
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // White=7
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // White=7
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
// };

// const uint8_t sprite_5[NUM_HEIGHT][NUM_WIDTH] = {
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // White=7
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // White=7
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
// };

// const uint8_t sprite_6[NUM_HEIGHT][NUM_WIDTH] = {
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // White=7
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // White=7
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
// };


// const uint8_t starfield[MATRIX_HEIGHT][MATRIX_WIDTH] = {
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 7, 0, 0, 0, 0},
//     {7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 7, 7, 0, 0, 0},
//     {0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 7, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 7, 7, 7, 0, 0, 0, 7, 0, 0, 0, 0, 0, 7, 7, 7, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 7, 0, 0, 0, 7, 7, 7, 0, 0, 0, 0, 0, 7, 0, 0, 0, 7, 0, 0, 0},
//     {0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {7, 7, 7, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 7, 0, 0, 7, 0, 0, 0, 0, 0, 7, 7, 7, 0, 0, 0, 0, 0, 7, 0, 7, 0, 7, 0, 7, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0},
//     {0, 0, 0, 7, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 7, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 7, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7},
//     {0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 7, 7, 7, 7, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 7, 7, 0, 7, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 7, 7, 0, 7, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 7, 0, 7, 7, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 7, 7, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 7, 0, 7, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 7, 0, 0, 7, 7, 7, 7, 7, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 7, 0, 0, 7, 0, 0, 0, 7, 7, 7, 7, 7, 7, 7, 0, 0, 0, 0, 0, 0},
//     {0, 0, 7, 0, 0, 0, 0, 7, 7, 7, 0, 7, 7, 7, 0, 0, 0, 0, 0, 0, 7, 0, 0, 7, 7, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 7, 0, 0, 0, 7, 7, 7, 7, 0, 0, 7, 0, 7, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 7, 7, 0, 0, 0, 0, 7, 0, 0, 0, 7, 0, 0, 0, 7, 0, 0, 0, 0, 0, 7, 0, 0, 0, 7},
//     {7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 7, 7, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 7, 7, 7, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 7},
//     {0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0},
//     {0, 0, 7, 7, 7, 7, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 7, 7, 0},
//     {0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 7, 0, 0, 7, 0, 7, 7, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0},
//     {0, 0, 0, 0, 7, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0},
//     {0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 7, 0, 0, 0, 7, 7, 7, 0, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 7, 7, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 7, 0, 0, 0},
//     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 7, 7, 0, 0, 7, 7, 7, 0, 0},
//     {0, 7, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 7, 0, 0, 0}
//   };


// //GPIO Inits
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

// //IR Inits
// // void init_IR_gpio(void)
// // {
// //     RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
// //     GPIOB->MODER &= ~(0x3);
// //     GPIOB->MODER |= (0x0);
// // }

// //Periodic Timer Update
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

// //Clears screen
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

// //(x,y) of pixel and color, (0,31) bottom right, (31, 0) top right
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

// //x coordinate, y coordinate, array height, array width, array pointer
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

// //Tim6 ISR
// void TIM6_DAC_IRQHandler(void) {
//     if (TIM6->SR & TIM_SR_UIF) {
//         TIM6->SR &= ~TIM_SR_UIF; // Clear flag
//        // IR_Scan();
//         Matrix_Scan();          // Call scan function
//     }
// }

// // static void IR_Scan(void) {
// //     if (GPIOB->IDR & 0x1)
// //     {
// //         score += 1;
// //     }    
// // }

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

// void start_screen(void) {
//     while (thrust_side == 0) {
//         //DrawSprite(0,0, MATRIX_HEIGHT, MATRIX_WIDTH, starfield);
//         DrawSprite(3, 4, CHAR_HEIGHT, CHAR_WIDTH, sprite_A);        
//         DrawSprite(8, 4, CHAR_HEIGHT, CHAR_WIDTH, sprite_U);
//         DrawSprite(13, 4, CHAR_HEIGHT, CHAR_WIDTH, sprite_T);
//         DrawSprite(18, 4, CHAR_HEIGHT, CHAR_WIDTH, sprite_O);
//         DrawSprite(23, 4, CHAR_HEIGHT, CHAR_WIDTH, sprite_N);
        
//         DrawSprite(17, 12, CHAR_HEIGHT, CHAR_WIDTH, sprite_A);
//         DrawSprite(22, 12, CHAR_HEIGHT, CHAR_WIDTH, sprite_I);
//         DrawSprite(27, 12, CHAR_HEIGHT, CHAR_WIDTH, sprite_R);  
//     }
//     ClearScreen(0);
// }

// void end_screen(void) {

// }

// //===========================================================================
// // Main
// //===========================================================================
// int main(void){
//     internal_clock();

//     // setup controls
//     setup_adc();
//     setup_tim14();    
//     // Initialize OLED
//     init_spi1();
//     spi1_init_oled();
//     init_tim17();

//     /* Initialize peripherals */
//     init_matrix_gpio();
//     init_refresh_timer();

//     //Piezo
//     setup_dac(); // Setup DAC for piezo
    
//     // welcome message
//     spi1_display1("Welcome to...");
//     spi1_display2("Team 30's Game!");
// while (true)
// {
//     /* code */
//     start_screen();
// }

//     start_screen();

//     //start
//     int refresh_rate = 100; //(ms)
//     game_running = true;
//     score = 0;
//     time_left = 30;
//     // clear display
//     spi1_display1("                 ");
//     spi1_display2("                 ");
//     // new display
//     spi1_display1("Game Started!");
//     spi1_display2("Score: 0");

//     /* Initialize Framebuffer */
//     //ClearScreen(0); // Start with Off/Black

//     /* Enable Global Interrupts */
//     __enable_irq(); // Start the timer interrupt processing
   
//     //thrust_side
//     //int lateral_boost = 2;
//     //thrust_down
//     //int tactive = -2;

//     //generateTone(1000);

//     time_left = 0;
//     //ClearScreen(0x0);
// }

