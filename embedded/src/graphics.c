#include "main.h"

// Define the global matrix buffer and display row tracker
volatile uint8_t canvas[MATRIX_SCAN_ROWS][MATRIX_WIDTH] = {0};
volatile uint8_t current_display_row = 0;

void Matrix_Scan(uint8_t row) {
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

void DrawSprite(int x, int y, int height, int width, const void *sprite_ptr, uint8_t color) {
    // Cast the incoming 2D array to a flat 1D pointer
    const uint8_t *sprite_data = (const uint8_t *)sprite_ptr;
    
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            // Use flat memory math to find the pixel
            if (sprite_data[r * width + c] != 0) {
                SetPixel(x + c, y + r, color);
            }
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