
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

void DrawSprite(int start_x, int start_y, int height, int width, const uint8_t sprite_data[][width], uint8_t color) {
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            if (sprite_data[r][c] != 0) {
                SetPixel(start_x + c, start_y + r, color);
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
// SCAN FUNCTION
//===========================================================================
static void Matrix_Scan(void) {
    GPIOB->BSRR = (1U << 14);

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

        GPIOB->BSRR = (1U << 12); 
        GPIOB->BRR  = (1U << 12);
    }

    uint8_t addr = current_display_row;
    uint32_t addr_set = 0, addr_reset = 0;
    
    if (addr & 0x01) addr_set |= (1U << 0);  else addr_reset |= (1U << 0);
    if (addr & 0x02) addr_set |= (1U << 1);  else addr_reset |= (1U << 1);
    if (addr & 0x04) addr_set |= (1U << 10); else addr_reset |= (1U << 10);
    if (addr & 0x08) addr_set |= (1U << 11); else addr_reset |= (1U << 11);
    GPIOB->BSRR = addr_set | (addr_reset << 16);

    GPIOB->BSRR = (1U << 13); 
    asm("nop"); asm("nop");
    GPIOB->BRR  = (1U << 13);

    GPIOB->BRR = (1U << 14); 

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

        if (!game_active) return;

        // Goal Cooldown logic
        if (sensor_cooldown > 0) {
            sensor_cooldown--;
            // If the cooldown JUST finished, tell Python the game is active again
            if (sensor_cooldown == 0) {
                send_state_byte(); 
            }
            return;
        }

        bool scored = false;
        
        // PA12 = Player Goal
        if (GPIOA->IDR & (1 << 12)) {
            player_score++;
            scored = true;
        } 
        // PA11 = Bot Goal
        else if (GPIOA->IDR & (1 << 11)) {
            bot_score++;
            scored = true;
        }

        if (scored) {
            sensor_cooldown = GOAL_COOLDOWN_TICKS;
            if (player_score >= WINNING_SCORE || bot_score >= WINNING_SCORE) {
                game_active = false;
            }
            // Tell Python: Score changed AND motors should pause
            send_state_byte(); 
        }
    }
}

//===========================================================================
// MAIN
//===========================================================================

int main(void) {
    enable_ports(); 
    
    // Init Matrix & Game
    init_matrix_gpio();
    init_refresh_timer();
    setup_tim14();
    init_sensors();      
    init_start_reset();  
    
    // Init UART & Motors
    init_uart();
    init_motors();

    uint8_t pending_motor = 0;
    uint8_t pending_dir = 0;
    uint32_t watchdog_timer = 0;
    
    // Button Debouncing
    uint32_t pa0_debounce = 0;
    uint32_t pb2_debounce = 0;

    // Send initial state to Python on boot
    send_state_byte();

    while (1) {
        // --- 1. RX: Motor Control ---
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
        else if (TEST_MODE == 0) {
            watchdog_timer++;
            if (watchdog_timer > WATCHDOG_MAX) {
                set_motor_a(0, 0);
                set_motor_b(0, 0);
                watchdog_timer = WATCHDOG_MAX; 
            }
        }

        // --- 2. Hardware Buttons ---
        if (pb2_debounce > 0) pb2_debounce--;
        if (pa0_debounce > 0) pa0_debounce--;

        // PB2 = Start Game
        if ((GPIOB->IDR & (1<<2)) && (pb2_debounce == 0)) {
            game_active = true;
            pb2_debounce = 50000;
            
            // Clear scores if starting fresh after a win
            if (player_score >= WINNING_SCORE || bot_score >= WINNING_SCORE) {
                 player_score = 0;
                 bot_score = 0;
            }
            send_state_byte(); 
        }
        
        // PA0 = Reset (In-Game) OR Mode Toggle (Menu)
        if ((GPIOA->IDR & (1<<0)) && (pa0_debounce == 0)) {
            pa0_debounce = 50000;
            
            if (game_active) {
                // Play Mode: Act as Reset
                game_active = false;
                player_score = 0;
                bot_score = 0;
                sensor_cooldown = 0;
            } else {
                // Menu Mode: Toggle Bot/Human
                if (game_mode == MODE_PLAYER) {
                    game_mode = MODE_BOT;
                } else {
                    game_mode = MODE_PLAYER;
                }
            }
            send_state_byte(); 
        }

        // --- 3. Screen Rendering ---
        ClearScreen();

        if (!game_active) {
            if (player_score >= WINNING_SCORE || bot_score >= WINNING_SCORE) {
                draw_win_screen();
            } else {
                start_screen(); 
            }
        } else {
            if (player_score < 10)
                DrawSprite(2, 9, NUM_HEIGHT, NUM_WIDTH, sprite_numbers[player_score], COLOR_BLUE);
            
            if (bot_score < 10)
                DrawSprite(18, 9, NUM_HEIGHT, NUM_WIDTH, sprite_numbers[bot_score], COLOR_RED);
        }
        
        delay_ms(16); 
    }
}