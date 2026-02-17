// #include "stm32f0xx.h"
// #include <stdint.h>

// // consts
// #define BAUD_RATE 115200
// #define START_BYTE 0xA5  // Arbitrary start byte to sync packets

// // funcs
// void init_uart5(void);
// void init_leds(void);
// void uart5_send_char(char c);
// uint8_t uart5_recv_char(void);
// void process_packet(uint8_t led_idx, uint8_t state);

// int main(void) {
//     // 1. Initialize Hardware
//     init_leds();
//     init_uart5();

//     uint8_t byte;
    
//     // Simple state machine variables
//     // 0 = Waiting for Start, 1 = Waiting for LED Index, 2 = Waiting for State
//     int state_machine = 0; 
//     uint8_t cmd_led = 0;
//     uint8_t cmd_state = 0;

//     while (1) {
//         // Blocking receive (wait for a character)
//         byte = uart5_recv_char();

//         switch (state_machine) {
//             case 0: // Waiting for START_BYTE
//                 if (byte == START_BYTE) {
//                     state_machine = 1;
//                 }
//                 break;
            
//             case 1: // Waiting for LED Index (0-3)
//                 cmd_led = byte;
//                 state_machine = 2;
//                 break;
            
//             case 2: // Waiting for State (0=OFF, 1=ON, 2=TOGGLE)
//                 cmd_state = byte;
//                 process_packet(cmd_led, cmd_state);
//                 state_machine = 0; // Reset to wait for next packet
//                 break;
                
//             default:
//                 state_machine = 0;
//                 break;
//         }
//     }
// }

// // --- Initialization Functions ---

// void init_leds(void) {
//     // gpioc clock en
//     RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

//     // pc6-pc9 as outputs
//     GPIOC->MODER |=  ((1 << 12) | (1 << 14) | (1 << 16) | (1 << 18)); // output mode
//     GPIOC->MODER &= ~((3 << 12) | (3 << 14) | (3 << 16) | (3 << 18)); // initialize as 0s
    
// }

// void init_uart5(void) {
//     // Enable Clocks for GPIOC and GPIOD
//     RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN;
    
//     // Enable UART5 Clock
//     RCC->APB1ENR |= RCC_APB1ENR_USART5EN;

//     // Configure PC12 (TX) as Alternate Function (AF2)
//     GPIOC->MODER  &= ~(3 << 24);      // Clear
//     GPIOC->MODER  |=  (2 << 24);      // Set AF mode
//     GPIOC->AFR[1] |=  (2 << 16);      // AF2 for pin 12 (AFR[1] covers pins 8-15)

//     // Configure PD2 (RX) as Alternate Function (AF2)
//     GPIOD->MODER  &= ~(3 << 4);       // Clear
//     GPIOD->MODER  |=  (2 << 4);       // Set AF mode
//     GPIOD->AFR[0] |=  (2 << 8);       // AF2 for pin 2 (AFR[0] covers pins 0-7)

//     // Configure UART5: 115200 baud @ 48MHz system clock (Assuming 48MHz default)
//     // BRR = SystemClock / BaudRate
//     // If SystemClock is 48MHz: 48000000 / 115200 = 416.6 -> 417
//     // If using 8MHz HSI, adjust accordingly. This calculation assumes standard setup.
//     USART5->BRR = 48000000 / BAUD_RATE; 

//     // Enable TE (Transmitter), RE (Receiver), and UE (UART Enable)
//     USART5->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
// }

// // --- Helper Functions ---

// // Blocking receive
// uint8_t uart5_recv_char(void) {
//     while (!(USART5->ISR & USART_ISR_RXNE)); // Wait until RX Not Empty
//     return USART5->RDR;
// }

// // Blocking transmit (for debugging echo)
// void uart5_send_char(char c) {
//     while (!(USART5->ISR & USART_ISR_TXE)); // Wait until TX Empty
//     USART5->TDR = c;
// }

// void process_packet(uint8_t led_idx, uint8_t state) {
//     // Map index 0-3 to Pins 6-9
//     // PC6 = Red, PC7 = Blue, PC8 = Orange, PC9 = Green (Example mapping)
    
//     uint32_t pin_mask = (1 << (6 + led_idx)); 

//     if (led_idx > 3) return; // Safety check

//     if (state == 1) {
//         GPIOC->BSRR = pin_mask; // Turn ON
//     } else if (state == 0) {
//         GPIOC->BRR = pin_mask;  // Turn OFF
//     } else if (state == 2) {
//         GPIOC->ODR ^= pin_mask; // Toggle
//     }
    
//     // Optional: Echo back 'K' to acknowledge
//     uart5_send_char('K');
// }