#include "stm32f407xx.h"

void delay(void) {
    for (uint32_t i = 0; i < 500000; i++);
}

void GPIO_Button_Init(void) {
    // Enable clock for GPIOC (button) and SYSCFG
    GPIOC_PCLK_EN();
    SYSCFG_PCLK_EN();

    // Set PC13 as input (00)
    GPIOC->MODER &= ~(3 << (13 * 2));

    // Configure EXTI13 line
    SYSCFG->EXTICR[3] &= ~(0xF << 4);      // Clear EXTI13 bits
    SYSCFG->EXTICR[3] |= (2 << 4);         // Select port C for EXTI13

    EXTI->IMR |= (1 << 13);                // Unmask EXTI13
    EXTI->FTSR |= (1 << 13);               // Falling edge trigger

    NVIC_SetPriority(IRQ_NO_EXTI15_10, 2);
    NVIC_EnableIRQ(IRQ_NO_EXTI15_10);
}

void GPIO_LED_Init(void) {
    GPIOA_PCLK_EN();

    // Set PA5 as output (01)
    GPIOA->MODER &= ~(3 << (5 * 2));
    GPIOA->MODER |= (1 << (5 * 2));
}

int main(void) {
    GPIO_LED_Init();
    GPIO_Button_Init();

    while (1) {
        // Do nothing, wait for interrupt
    }
}

// EXTI15_10 interrupt handler
void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & (1 << 13)) {
        EXTI->PR |= (1 << 13);  // Clear pending bit by writing 1

        // Toggle PA5 (LED)
        GPIOA->ODR ^= (1 << 5);

    }




{

	delay();//200ms
	GPIO_IRQHandling(GPIO_PIN_NO_13);
	/*
	 * GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5)
	 */
	GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);

    }
    }
