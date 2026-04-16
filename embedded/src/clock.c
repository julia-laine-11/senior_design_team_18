#include "stm32f0xx.h"

void internal_clock(void)
{
    // 1. Enable Prefetch Buffer and set Flash Latency to 1 Wait State
    // Crucial: This must happen before switching to 48MHz.
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY; // 001: 1 wait state (required for 24MHz < SYSCLK <= 48MHz)

    // 2. Ensure HSI is ON and ready
    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY));

    // 3. Reset PLL configuration
    // Clearing PLLSRC, PLLXTPRE, and PLLMUL bits
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMUL);
    
    // 4. Configure PLL: (HSI / 2) * 12 = 48 MHz
    // HSI is 8MHz, so HSI/2 = 4MHz. 4MHz * 12 = 48MHz.
    RCC->CFGR |= (RCC_CFGR_PLLSRC_HSI_DIV2 | RCC_CFGR_PLLMUL12);

    // 5. Enable PLL and wait for lock
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));

    // 6. Select PLL as System Clock source
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    // 7. Wait until PLL is actually being used
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // 8. Final Bus Divider Checks (Ensure they are Reset to 1)
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // HCLK = 48MHz
    RCC->CFGR |= RCC_CFGR_PPRE_DIV1; // PCLK = 48MHz
}