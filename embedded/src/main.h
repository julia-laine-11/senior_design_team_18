#ifndef MAIN_H
#define MAIN_H

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
#define GOAL_COOLDOWN_TICKS 1500

#define MODE_BOT            0
#define MODE_PLAYER         1

#define UI_STATE_SPLASH     0
#define UI_STATE_MENU       1
#define UI_STATE_PLAY       2

#define COLOR_RED   1
#define COLOR_BLUE  4
#define COLOR_WHITE 7

#define CHAR_HEIGHT 5
#define CHAR_WIDTH 4
#define NUM_HEIGHT 14
#define NUM_WIDTH 10

#define TEST_MODE 0     
#define WATCHDOG_MAX 25 
#define SYSTEM_CLOCK 48000000 

#define ABS(x) ((x) < 0 ? -(x) : (x))

//===========================================================================
// EXTERN GLOBAL VARIABLES (Defined in main.c or graphics.c)
//===========================================================================
extern volatile int player_score;
extern volatile int bot_score;
extern volatile int sensor_cooldown;
extern volatile bool game_active;
extern volatile uint8_t game_mode; 
extern volatile uint32_t anim_tick; 

extern volatile uint8_t canvas[MATRIX_SCAN_ROWS][MATRIX_WIDTH];
extern volatile uint8_t current_display_row;

extern volatile uint8_t pending_motor;
extern volatile uint8_t pending_dir;
extern volatile uint32_t watchdog_timer;

//===========================================================================
// PROTOTYPES: Hardware & Setup
//===========================================================================
void init_clock(void);
void init_adc(void);
uint16_t read_adc(void);
void init_oled_pins(void);
void spi_cmd(unsigned int data);
void spi_data(unsigned int data);
void spi1_init_oled(void);
void spi1_display1(const char *string);
void spi1_display2(const char *string);
void init_controls(void);
void init_sensors(void);
void init_matrix_gpio(void);
void setup_tim14(void);
void init_uart(void);
void init_motors(void);
void set_motor_a(uint32_t percent, uint8_t is_rev);
void set_motor_b(uint32_t percent, uint8_t is_rev);
void delay_ms(uint32_t ms);
void small_delay(void);

//===========================================================================
// PROTOTYPES: Graphics Engine
//===========================================================================
void Matrix_Scan(uint8_t row);
void SetPixel(int x, int y, uint8_t color);
void ClearScreen(void);
void DrawSprite(int x, int y, int height, int width, const void *sprite_data, uint8_t color);
void DrawSprite5(int x, int y, const uint8_t sprite_data[5][5], uint8_t color);

#endif // MAIN_H