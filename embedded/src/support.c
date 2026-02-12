// // CODE ADAPTED FROM LAB 6 //

// #include "stm32f0xx.h"
// #include <string.h> // for memmove()
// #include <stdlib.h> // for srandom() and random()
// #include <stdio.h>
// #include <stdbool.h>

// uint16_t msg[8];
// uint16_t display[34];  

// void nano_wait(unsigned int n) {
//     asm(    "        mov r0,%0\n"
//             "repeat: sub r0,#83\n"
//             "        bgt repeat\n" : : "r"(n) : "r0", "cc");
// }

// void small_delay(void) {
//     nano_wait(50000);
// }

// void enable_ports(){

// }

// void delay_ms(uint32_t ms) {
//     for (uint32_t i = 0; i < ms; i++) {
//         SysTick->LOAD = 48000 - 1;                 // 1ms @ 48MHz
//         SysTick->VAL = 0;
//         SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
//         while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
//         SysTick->CTRL = 0;  // Stop SysTick
//     }
// }

// extern int score;
// extern int time_left;
// extern bool game_running;
// extern bool game_started;
// //extern void spi1_display2(char *message);
// extern void game_over(const char *end);

// //game run? i think

// int8_t thrust_down   = 0;           //same as before
// int8_t thrust_side   = 0;           //or do we need 2, one for left and one for right?

// void setup_tim14() {
//     RCC -> APB1ENR |= RCC_APB1ENR_TIM14EN;
  
//     // invoke an update interrupt 
//     TIM14 -> PSC = 2400 - 1; //
//     TIM14 -> ARR = 100 - 1; //
  
//     //Enable UIE bit in DIER (use TIM_DIER_UIE mask)
//     TIM14 -> DIER |= TIM_DIER_UIE;
//     //Enable the TIM7 interrupt (NVIC ISER)
//     NVIC -> ISER[0] |= (1 << TIM14_IRQn);
//     //Enable TIM7 by setting CEN bit in TIM7 CR)  - set TIM_CR1_CEN in TIM7_CR1 (dont set the TIM7_ARR to 0)
//     TIM14 -> CR1 |= TIM_CR1_CEN;
// }

// void setup_dac(void) {
//     RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable GPIOA
//     GPIOA->MODER |= (3 << (4 * 2)); // Set PA4 to analog mode
//     RCC->APB1ENR |= RCC_APB1ENR_DACEN; // Enable DAC clock
//     DAC->CR &= ~DAC_CR_EN1; // Ensure DAC is off
//     DAC->CR &= ~DAC_CR_TEN1; // Disable trigger
//     DAC->CR |= DAC_CR_EN1; // Enable DAC
// }


// /*----------------------------------------------------------------------------------------------
// ADC SETUP FOR CONTROLS
// ----------------------------------------------------------------------------------------------*/
// void setup_adc(){ 
//     RCC -> AHBENR |= RCC_AHBENR_GPIOAEN; 
//     RCC -> AHBENR |= RCC_AHBENR_GPIOCEN;

//     GPIOA -> MODER |= 0x3C; //1100 -> 12 -> C pa1 and 2
 
//     RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; 

//     RCC->CR2 |= RCC_CR2_HSI14ON;
//     while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) { }
 
//     ADC1 -> CR |= ADC_CR_ADEN; 
 
//     while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) { }
 
//     ADC1 -> CHSELR = ADC_CHSELR_CHSEL1;
 
//     while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) { }

//  }

// double read_adc_channel1() {
//     ADC1 -> CHSELR = ADC_CHSELR_CHSEL1;
//     ADC1 -> CR |= ADC_CR_ADSTART;
//     while ((ADC1->ISR & ADC_ISR_EOC) == 0) {}
//     return ((double)ADC1->DR) * 3.3  / 4095.0;
// }

// double read_adc_channel2() {
//     ADC1 -> CHSELR = ADC_CHSELR_CHSEL2;
//     ADC1 -> CR |= ADC_CR_ADSTART;
//     while ((ADC1->ISR & ADC_ISR_EOC) == 0) {}
//     return ((double)ADC1->DR) * 3.3  / 4095.0;
// }

// void TIM14_IRQHandler(){
//     //acknowledge the interrupt
//     TIM14 -> SR &= ~TIM_SR_UIF;  

// //Thrust Side Logic

// ///////////////////////////////////////////////////////////////////////////////////

//     //boundaries for the voltage provided by the potentiometers
//     int8_t b0 = 0;
//     float  b1 = 0.66;
//     float  b2 = 1.4;
//     float  b3 = 1.9;
//     float  b4 = 2.64;
//     int8_t b5 = 3.5;

//     //int8_t thrust_down = 0;
//     int8_t a =  2; // thrust  2x
//     int8_t b =  1; // thrust  1x
//     int8_t c =  0; // thrust  0x
//     int8_t d = -1; // thrust -1x
//     int8_t e = -2; // thrust -2x    

//     if     (read_adc_channel1() < b1 && read_adc_channel1() > b0) {
//         // GPIOB -> BRR  = 0x1F00;
//         // GPIOB -> BSRR = 0x100;  //PA8
//         thrust_side = a;
//     }
 
//     else if(read_adc_channel1() < b2 && read_adc_channel1() > b1) {
//         // GPIOB -> BRR  = 0x1F00;
//         // GPIOB -> BSRR = 0x200;  //PA9
//         thrust_side = b;
//     }
 
//     else if(read_adc_channel1() < b3 && read_adc_channel1() > b2) {
//         // GPIOB -> BRR  = 0x1F00;
//         // GPIOB -> BSRR = 0x400;  //PA10
//         thrust_side = c;
//     }
 
//     else if(read_adc_channel1() < b4 && read_adc_channel1() > b3) {
//         // GPIOB -> BRR  = 0x1F00;
//         // GPIOB -> BSRR = 0x800;  //PA11
//         thrust_side = d;
//     }
 
//     else if(read_adc_channel1() < b5 && read_adc_channel1() > b4) {     
//         // GPIOB -> BRR  = 0x1F00;
//         // GPIOB -> BSRR = 0x1000; //PA12
//         thrust_side = e;
//     }
// ///////////////////////////////////////////////////////////////////////////////////

// //Thrust Down Logic

// ///////////////////////////////////////////////////////////////////////////////////
//     //boundaries for the voltage provided by the potentiometers
//     int8_t v0 = 0;
//     float  v1 = 0.2;
//     float  v2 = 1.4;
//     float  v3 = 2.2;
//     int8_t v4 = 3.5;

//     //int8_t thrust_side = 0;
//     int8_t d0 = 0; // thrust  0x
//     int8_t d1 = 1; // thrust  1x
//     int8_t d2 = 2; // thrust  2x 
//     int8_t d3 = 3; // thrust  3x

//     if(read_adc_channel2() < v1 && read_adc_channel2() > v0) {
//         // GPIOC -> BRR  = 0x7C; //reset
//         // GPIOC -> BSRR = 0x4;  //Pc2
//         thrust_down = d0;
//     }

//     else if(read_adc_channel2() < v2 && read_adc_channel2() > v1) {
//         // GPIOC -> BRR  = 0x7C; //reset
//         // GPIOC -> BSRR = 0x4;  //Pc2
//         // GPIOC -> BSRR = 0x8;  //Pc3
//         thrust_down = d1;
//     }

//     else if(read_adc_channel2() < v3 && read_adc_channel2() > v2) {
//         // GPIOC -> BRR  = 0x7C;  //reset
//         // GPIOC -> BSRR = 0x4;   //Pc2
//         // GPIOC -> BSRR = 0x8;   //Pc3
//         // GPIOC -> BSRR = 0x10;  //Pc4
//         thrust_down = d2;
//     }

//     else if(read_adc_channel2() < v4 && read_adc_channel2() > v3) {
//         // GPIOC -> BRR  = 0x7C;  //reset
//         // GPIOC -> BSRR = 0x4;   //Pc2
//         // GPIOC -> BSRR = 0x8;   //Pc3
//         // GPIOC -> BSRR = 0x10;  //Pc4
//         // GPIOC -> BSRR = 0x20;  //Pc5
//         thrust_down = d3;

//     }
// ///////////////////////////////////////////////////////////////////////////////////
// }
// /*----------------------------------------------------------------------------------------------
// SPI SETUP FOR LCD
// ----------------------------------------------------------------------------------------------*/
// void init_spi1() {
//     // enable clock
//     RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
//     RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

//     // configure PA15, PA5, PA7
//     GPIOA->MODER &= ~(GPIO_MODER_MODER15 | GPIO_MODER_MODER5 | GPIO_MODER_MODER7);
//     GPIOA->MODER |= (GPIO_MODER_MODER15_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER7_1);

//     GPIOA->AFR[1] &= ~(0xF0000000);
//     GPIOA->AFR[0] &= ~(0xF0F00000);

//     // configure SPI1 in master mode
//     SPI1->CR1 &= ~SPI_CR1_SPE;

//     SPI1->CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2;
//     SPI1->CR1 |= SPI_CR1_MSTR;

//     SPI1->CR2 = SPI_CR2_DS_3 | SPI_CR2_DS_0;
//     SPI1->CR2 |= SPI_CR2_SSOE | SPI_CR2_NSSP;
//     SPI1->CR2 |= SPI_CR2_TXDMAEN;
//     SPI1->CR1 |= SPI_CR1_SPE;
// }

// void spi_cmd(unsigned int data) {
//     // wait until SPI1 buffer is empty
//     while (!(SPI1->SR & SPI_SR_TXE)) {};
    
//     // copy data
//     SPI1->DR = data;
// }

// void spi_data(unsigned int data) {
//     // call spi_cmd
//     spi_cmd(data | 0x200);
// }

// void spi1_init_oled() {
//     // wait 1 ms
//     nano_wait(1000000);

//     // initialize OLED display
//     spi_cmd(0x38);
//     spi_cmd(0x08);
//     spi_cmd(0x01);
//     nano_wait(2000000); // wait 2 ms

//     spi_cmd(0x06);
//     spi_cmd(0x02);
//     spi_cmd(0x0c); 
// }

// void spi1_display1(const char *string) {
//     // move cursor to home
//     spi_cmd(0x02);

//     // loop through the string
//     for (int i = 0; string[i] != '\0'; i++) {
//         spi_data(string[i]);
//     } 
// }

// void spi1_display2(const char *string) {
//     spi_cmd(0xc0);
//     for (int i = 0; string[i] != '\0'; i++) {
//         spi_data(string[i]);
//     }   
// }


// void spi1_setup_dma(void) {
//     // enable clock
//     RCC->AHBENR |= RCC_AHBENR_DMA1EN;

//     // disable DMA channel
//     DMA1_Channel3->CCR &= ~DMA_CCR_EN;

//     // set memory address
//     DMA1_Channel3->CMAR = (uint32_t)&display;

//     // set peripheral address
//     DMA1_Channel3->CPAR = (uint32_t)&SPI1->DR;

//     // set number of data transfers
//     DMA1_Channel3->CNDTR = 34;

//     // configure DMA channel
//     DMA1_Channel3->CCR &= ~DMA_CCR_DIR;
//     DMA1_Channel3->CCR |= DMA_CCR_DIR;
//     DMA1_Channel3->CCR |= DMA_CCR_MINC;
//     DMA1_Channel3->CCR &= ~DMA_CCR_PINC;

//     DMA1_Channel3->CCR &= ~DMA_CCR_MSIZE;
//     DMA1_Channel3->CCR |= DMA_CCR_MSIZE_0;
//     DMA1_Channel3->CCR &= ~DMA_CCR_PSIZE;
//     DMA1_Channel3->CCR |= DMA_CCR_PSIZE_0;
//     DMA1_Channel3->CCR |= DMA_CCR_CIRC;

//     SPI1->CR2 |= SPI_CR2_TXDMAEN;
// }

// void spi1_enable_dma(void) {
//     // enable DMA channel 3
//     DMA1_Channel3->CCR |= DMA_CCR_EN;
// }

// //Never Used
// void update_score(void) {
//     //score++;
//     char buffer[17];
//     snprintf(buffer, sizeof(buffer), "Score: %3d", score);
//     spi1_display2(buffer);
// }


// void game_over(const char *end) {
//     game_running = false;
//     game_started = false;
//     // clear display
//     spi1_display1("                 ");
//     spi1_display2("                 ");
//     // new display
//     spi1_display1("Game Over!");
//     spi1_display2(end);
// }

// void crash(void) {
//     // if (time_left >=2) {
//     //     time_left -= 2;
//     // } else {
//     //     time_left = 0;
//     // }

//     // update OLED
//     char buffer[17];
//     snprintf(buffer, sizeof(buffer), "Score: %d", score);
//     spi1_display2(buffer);

//     if (game_running && game_started && time_left == 0) {
//         game_over("You Lose :(");
//     }
// }

// void TIM17_IRQHandler(void) {
//     if (TIM17->SR & TIM_SR_UIF) {
//         TIM17->SR &= ~TIM_SR_UIF; 

//         // check game state
//         if (game_running) {
//             //score++;

//             // game started if score above 0
//             if (!game_started && time_left > 0) {
//                 game_started = true;
//             }

//             // update display
//             char buffer[17];
//             snprintf(buffer, sizeof(buffer), "Score: %d", score);
//             spi1_display2(buffer);

//             // win condition
//             if (score >= 50) {
//               game_over("You Win!");
//             }

//             // lose condition
//             else if (game_started && time_left == 0) {
//                 game_over("You Lose :(");
//                 return;
//             }
//         }
//     }
// }

// void init_tim17(void)
// {
//     RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
//     TIM17->PSC = 48000 - 1;
//     TIM17->ARR = 1000 - 1;
//     TIM17->CR1 |= TIM_CR1_ARPE;
//     TIM17->DIER |= TIM_DIER_UIE;
//     TIM17->CR1 |= TIM_CR1_CEN;
//     NVIC_EnableIRQ(TIM17_IRQn);
// }

// void generateTone(uint16_t duration_ms) {
//     DAC->DHR8R1 = 200; //turn buzzer on
//     delay_ms(duration_ms); //keep buzzer on for certain duration
//     DAC->DHR8R1 = 0; //turn off
// }