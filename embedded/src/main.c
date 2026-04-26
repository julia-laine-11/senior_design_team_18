#include "main.h"
#include "sprites.h"

//===========================================================================
// GLOBAL VARIABLES
//===========================================================================
volatile int player_score = 0;
volatile int bot_score = 0;
volatile int sensor_cooldown = 0;
volatile bool game_active = false;
volatile uint8_t game_mode = MODE_BOT; 
volatile uint32_t anim_tick = 0; 

volatile uint8_t pending_motor = 0;
volatile uint8_t pending_dir = 0;
volatile uint32_t watchdog_timer = 0;

#define NUM_FW 5
typedef struct {
    int x, y, frame, color, max_r;
} Firework;
Firework fws[NUM_FW] = {0};

//===========================================================================
// GAME HELPERS & INTERRUPTS
//===========================================================================

void send_state_byte(void) {
    uint8_t state_byte = 0;
    if (game_active && (sensor_cooldown == 0)) state_byte |= (1 << 7);
    if (game_mode == MODE_PLAYER) state_byte |= (1 << 6);
    state_byte |= ((player_score & 0x07) << 3);
    state_byte |= (bot_score & 0x07);
    while (!(USART5->ISR & USART_ISR_TXE)); 
    USART5->TDR = state_byte;
}

void USART3_8_IRQHandler(void) {
    if (USART5->ISR & USART_ISR_ORE) USART5->ICR |= USART_ICR_ORECF;

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

void TIM14_IRQHandler(void) {
    if (TIM14->SR & TIM_SR_UIF) {
        TIM14->SR &= ~TIM_SR_UIF;

        if (!game_active) return;

        static uint16_t p_db = 0xFFFF;
        static uint16_t b_db = 0xFFFF;

        if (sensor_cooldown > 0) {
            sensor_cooldown--;
            p_db = 0xFFFF; 
            b_db = 0xFFFF;
            if (sensor_cooldown == 0) send_state_byte(); 
            return;
        }

        uint8_t p_read = (GPIOA->IDR & (1 << 12)) ? 1 : 0; 
        uint8_t b_read = (GPIOA->IDR & (1 << 11)) ? 1 : 0; 

        p_db = (p_db << 1) | p_read;
        b_db = (b_db << 1) | b_read;

        bool scored = false;
        
        if ((p_db & 0x07FF) == 0x0400) { player_score++; scored = true; } 
        else if ((b_db & 0x07FF) == 0x0400) { bot_score++; scored = true; }

        if (scored) {
            sensor_cooldown = GOAL_COOLDOWN_TICKS;
            p_db = 0xFFFF; b_db = 0xFFFF;

            if (player_score >= WINNING_SCORE || bot_score >= WINNING_SCORE) {
                game_active = false;
            }
            send_state_byte(); 
        }
    }
}

int get_bubble_offset(int index) {
    int wave_pos = (anim_tick / 4) % 40; 
    int dist = ABS(wave_pos - index);
    if (dist == 0) return -2;
    if (dist == 1) return -1;
    return 0;
}

void start_screen(void) {
    int ya = 6;
    DrawSprite5(1,  ya + get_bubble_offset(0), s5_A, COLOR_RED);
    DrawSprite5(7,  ya + get_bubble_offset(1), s5_U, COLOR_RED);
    DrawSprite5(13, ya + get_bubble_offset(2), s5_T, COLOR_RED);
    DrawSprite5(19, ya + get_bubble_offset(3), s5_O, COLOR_RED);
    DrawSprite5(25, ya + get_bubble_offset(4), s5_N, COLOR_RED);
    
    int yb = 18;
    DrawSprite5(14, yb + get_bubble_offset(5), s5_A, COLOR_BLUE);
    DrawSprite5(20, yb + get_bubble_offset(6), s5_I, COLOR_BLUE);
    DrawSprite5(26, yb + get_bubble_offset(7), s5_R, COLOR_BLUE);  
}

void draw_zipper_border(uint8_t color) {
    int p = 0;
    int offset = (anim_tick / 10) % 2; 
    
    for (int x = 1; x <= 30; x++) { p++; SetPixel(x, ((p + offset) % 2 == 0) ? 1 : 2, color); }
    for (int y = 2; y <= 30; y++) { p++; SetPixel(((p + offset) % 2 == 0) ? 30 : 29, y, color); }
    for (int x = 29; x >= 1; x--) { p++; SetPixel(x, ((p + offset) % 2 == 0) ? 30 : 29, color); }
    for (int y = 29; y >= 2; y--) { p++; SetPixel(((p + offset) % 2 == 0) ? 1 : 2, y, color); }
}

void draw_win_screen(void) {
    for (int i = 0; i < NUM_FW; i++) {
        if (fws[i].frame == 0) {
            if ((rand() % 100) < 5) { 
                fws[i].x = 4 + (rand() % 24);
                fws[i].y = 4 + (rand() % 24);
                
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
    
    int base_y1 = 10;
    int base_y2 = 17;

    if (player_score >= WINNING_SCORE) {
        DrawSprite(6,  base_y1 + get_bubble_offset(0), CHAR_HEIGHT, CHAR_WIDTH, s4_B, COLOR_BLUE);
        DrawSprite(11, base_y1 + get_bubble_offset(1), CHAR_HEIGHT, CHAR_WIDTH, s4_L, COLOR_BLUE);
        DrawSprite(16, base_y1 + get_bubble_offset(2), CHAR_HEIGHT, CHAR_WIDTH, s4_U, COLOR_BLUE);
        DrawSprite(21, base_y1 + get_bubble_offset(3), CHAR_HEIGHT, CHAR_WIDTH, s4_E, COLOR_BLUE);
    } else {
        DrawSprite(8,  base_y1 + get_bubble_offset(0), CHAR_HEIGHT, CHAR_WIDTH, s4_R, COLOR_RED);
        DrawSprite(13, base_y1 + get_bubble_offset(1), CHAR_HEIGHT, CHAR_WIDTH, s4_E, COLOR_RED);
        DrawSprite(18, base_y1 + get_bubble_offset(2), CHAR_HEIGHT, CHAR_WIDTH, s4_D, COLOR_RED);
    }
    
    DrawSprite(6,  base_y2 + get_bubble_offset(4), CHAR_HEIGHT, CHAR_WIDTH, s4_W, win_color);
    DrawSprite(11, base_y2 + get_bubble_offset(5), CHAR_HEIGHT, CHAR_WIDTH, s4_I, win_color);
    DrawSprite(16, base_y2 + get_bubble_offset(6), CHAR_HEIGHT, CHAR_WIDTH, s4_N, win_color);
    DrawSprite(21, base_y2 + get_bubble_offset(7), CHAR_HEIGHT, CHAR_WIDTH, s4_S, win_color);
}

//===========================================================================
// MAIN LOOP
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
    
    const int bounce_lut[12] = {0, -1, -2, -3, -4, -5, -6, -5, -4, -3, -2, -1};
    int last_p_score = 0; int last_b_score = 0;
    int p_bounce = 0; int b_bounce = 0;
    int p_splash_frame = 0; int b_splash_frame = 0;
    
    send_state_byte();

    while (1) {
        if (TEST_MODE == 0) {
            watchdog_timer++;
            if (watchdog_timer > WATCHDOG_MAX) {
                set_motor_a(0, 0);
                set_motor_b(0, 0);
                watchdog_timer = WATCHDOG_MAX; 
            }
        }

        if (pc2_debounce > 0) pc2_debounce--;

        if (ui_state == UI_STATE_MENU) {
            uint16_t joy_y = read_adc();
            if (joy_y < 1000 && selected_mode != MODE_PLAYER) {
                selected_mode = MODE_PLAYER; oled_needs_update = 1;
            } else if (joy_y > 3000 && selected_mode != MODE_BOT) {
                selected_mode = MODE_BOT; oled_needs_update = 1;
            }
        }

        uint8_t pc2_current = (GPIOC->IDR & (1 << 2)) ? 1 : 0;
        
        if (pc2_current == 0 && pc2_last_state == 1 && pc2_debounce == 0) {
            pc2_debounce = 5; 
            
            if (ui_state == UI_STATE_SPLASH) {
                ui_state = UI_STATE_MENU; oled_needs_update = 1;
            } else if (ui_state == UI_STATE_MENU) {
                ui_state = UI_STATE_PLAY; game_mode = selected_mode; game_active = true;
                player_score = 0; bot_score = 0; last_p_score = 0; last_b_score = 0;
                p_bounce = 0; b_bounce = 0; p_splash_frame = 0; b_splash_frame = 0;
                sensor_cooldown = 0; 
                for(int i=0; i<NUM_FW; i++) fws[i].frame = 0; 
                oled_needs_update = 1; send_state_byte(); 
            } else if (ui_state == UI_STATE_PLAY) {
                ui_state = UI_STATE_SPLASH; game_active = false;
                oled_needs_update = 1; send_state_byte();
            }
        }
        pc2_last_state = pc2_current;
        
        if (oled_needs_update) {
            oled_needs_update = 0;
            spi_cmd(0x01); delay_ms(2); 
            if (ui_state == UI_STATE_SPLASH) {
                spi1_display1("Push Button     "); spi1_display2("to start...     ");
            } else if (ui_state == UI_STATE_MENU) {
                if (selected_mode == MODE_PLAYER) {
                    spi1_display1("> Blue (Human)  "); spi1_display2("  Red (Bot)     ");
                } else {
                    spi1_display1("  Blue (Human)  "); spi1_display2("> Red (Bot)     ");
                }
            } else if (ui_state == UI_STATE_PLAY) {
                if (game_mode == MODE_PLAYER) {
                    spi1_display1("Playing:        "); spi1_display2("Blue (Human)    ");
                } else {
                    spi1_display1("Playing:        "); spi1_display2("Red (Bot)       ");
                }
            }
        }

        ClearScreen();

        if (ui_state == UI_STATE_SPLASH || ui_state == UI_STATE_MENU) {
            start_screen(); 
        } else if (ui_state == UI_STATE_PLAY) {
            if (!game_active && (player_score >= WINNING_SCORE || bot_score >= WINNING_SCORE)) {
                draw_win_screen();
            } else {
                if (player_score > last_p_score) { p_bounce = 11; p_splash_frame = 1; last_p_score = player_score; }
                if (bot_score > last_b_score) { b_bounce = 11; b_splash_frame = 1; last_b_score = bot_score; }
                
                if (p_bounce > 0) p_bounce--;
                if (b_bounce > 0) b_bounce--;

                int p_y = 9 + bounce_lut[p_bounce];
                int b_y = 9 + bounce_lut[b_bounce];

                if (p_splash_frame > 0) {
                    int r = p_splash_frame / 2; int cx = 8, cy = 16;
                    if (r <= 6) {
                        SetPixel(cx + r, cy, COLOR_WHITE); SetPixel(cx - r, cy, COLOR_WHITE);
                        SetPixel(cx, cy + r, COLOR_WHITE); SetPixel(cx, cy - r, COLOR_WHITE);
                        if (r > 1) {
                            int d = r - 1;
                            SetPixel(cx + d, cy + d, COLOR_WHITE); SetPixel(cx - d, cy - d, COLOR_WHITE);
                            SetPixel(cx + d, cy - d, COLOR_WHITE); SetPixel(cx - d, cy + d, COLOR_WHITE);
                        }
                        p_splash_frame++;
                    } else p_splash_frame = 0;
                }

                if (b_splash_frame > 0) {
                    int r = b_splash_frame / 2; int cx = 24, cy = 16;
                    if (r <= 6) {
                        SetPixel(cx + r, cy, COLOR_WHITE); SetPixel(cx - r, cy, COLOR_WHITE);
                        SetPixel(cx, cy + r, COLOR_WHITE); SetPixel(cx, cy - r, COLOR_WHITE);
                        if (r > 1) {
                            int d = r - 1;
                            SetPixel(cx + d, cy + d, COLOR_WHITE); SetPixel(cx - d, cy - d, COLOR_WHITE);
                            SetPixel(cx + d, cy - d, COLOR_WHITE); SetPixel(cx - d, cy + d, COLOR_WHITE);
                        }
                        b_splash_frame++;
                    } else b_splash_frame = 0;
                }

                if (player_score < 10) DrawSprite(3, p_y, NUM_HEIGHT, NUM_WIDTH, sprite_numbers[player_score], COLOR_BLUE);
                if (bot_score < 10) DrawSprite(19, b_y, NUM_HEIGHT, NUM_WIDTH, sprite_numbers[bot_score], COLOR_RED);
            }
        }
        
        anim_tick++;
        delay_ms(16); 
    }
}