#include "stm32f100xb.h"

void init_pin();
void init_NVIC();
void msDelay(int ms);


void EXTI9_5_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR5){
        /* Прерывание от PC5 - SW */
        GPIOC->ODR ^= GPIO_ODR_ODR8;
        msDelay(500);
        EXTI->PR |= EXTI_PR_PR5;
    }
    else {
        GPIOC->ODR ^= GPIO_ODR_ODR9;
    }
}

int main() {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;

    init_pin();
    init_NVIC();

    while(1) {

    }
}

void init_pin() {
    /* PC8 - BLUE LED */
    GPIOC->CRH |= GPIO_CRH_MODE8_1 | GPIO_CRH_MODE8_0;
    GPIOC->CRH &= ~GPIO_CRH_CNF8_1 & ~GPIO_CRH_CNF8_0;

    /* PC9 - GREEN LED */
    GPIOC->CRH |= GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0;
    GPIOC->CRH &= ~GPIO_CRH_CNF9_1 & ~GPIO_CRH_CNF9_0;
    
    /* PA6 - A, Input Pull-up */
    GPIOA->CRL |= GPIO_CRL_CNF6_1;
    GPIOA->CRL &= ~GPIO_CRL_CNF6_0 & ~GPIO_CRL_MODE6_1 & ~GPIO_CRL_MODE6_0;
    GPIOA->ODR |= GPIO_ODR_ODR6;

    /* PA7 - B, Input Pull-up */
    GPIOA->CRL |= GPIO_CRL_CNF7_1;
    GPIOA->CRL &= ~GPIO_CRL_CNF7_0 & ~GPIO_CRL_MODE7_1 & ~GPIO_CRL_MODE7_0;
    GPIOA->ODR |= GPIO_ODR_ODR7;
    
    /* PC5 - SW, Input Pull-up */
    GPIOC->CRL |= GPIO_CRL_CNF5_1;
    GPIOC->CRL &= ~GPIO_CRL_CNF5_0 & ~GPIO_CRL_MODE5_1 & ~GPIO_CRL_MODE5_0;
    GPIOC->ODR |= GPIO_ODR_ODR5;
}

void init_NVIC() {
    /* Настойка SW, кнопки */
    /* Разешить прерывания от 5 линии */
    EXTI->IMR |= EXTI_IMR_IM5;
    /* По нарастающему фронту сигнала */
    EXTI->RTSR |= EXTI_RTSR_RT5;
    /* Соединить 5 линию с портом C (т.е. PC5) */
    AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI5_PC;
    NVIC_EnableIRQ(EXTI9_5_IRQn);


}

void msDelay(int ms){
   while(ms-- > 0) {
      volatile int x=500;
      while (x-- > 0)
         __asm("nop");
   }
}