#include "main.h"

void init_clock(void) {
    FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;
    RCC->CFGR &= ~(RCC_CFGR_PLLMUL | RCC_CFGR_PLLSRC);
    RCC->CFGR |= RCC_CFGR_PLLMUL12;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)); 
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); 
}

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

void small_delay(void) {
    for(volatile int i=0; i<15; i++);
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