#include "stm32f100xb.h"

void init_pin();
void init_NVIC();
void msDelay(int ms);
void init_TIM3();
void init_TIM4();
void init_UART();
void send_byte(uint8_t byte);

void EXTI9_5_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR5){
        /* Прерывание от PC5 - SW */
        GPIOC->ODR ^= GPIO_ODR_ODR9;
        msDelay(500);
        EXTI->PR |= EXTI_PR_PR5;
    }
}

void TIM4_IRQHandler(void) {
    if (TIM4->SR & TIM_SR_TIF) {
        send_byte(TIM4->CNT);
        GPIOC->ODR ^= GPIO_ODR_ODR9;
        TIM3->CCR3 = (TIM4->CNT >> 1);
        msDelay(150);
        // Сбросить флаг прерывания
        TIM4->SR &= ~TIM_SR_TIF;
    }
}

int main() {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | RCC_APB1ENR_USART3EN;

    init_pin();
    init_NVIC();
    init_TIM3(); init_TIM4();
    init_UART();
    
    TIM3->CR1 |= TIM_CR1_CEN;
    TIM4->CR1 |= TIM_CR1_CEN;
    
    while(1) {
    }
}

void init_pin() {
    /* PC8 - BLUE LED, TIM3_Ch3 */
    GPIOC->CRH |= GPIO_CRH_CNF8_1 | GPIO_CRH_MODE8_1 | GPIO_CRH_MODE8_0;
    GPIOC->CRH &= ~GPIO_CRH_CNF8_0;
    AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP;

    /* PC9 - GREEN LED */
    GPIOC->CRH |= GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0;
    GPIOC->CRH &= ~GPIO_CRH_CNF9_1 & ~GPIO_CRH_CNF9_0;
    
    /* PB6 - A, Input Pull-up */
    GPIOB->CRL |= GPIO_CRL_CNF6_1;
    GPIOB->CRL &= ~GPIO_CRL_CNF6_0 & ~GPIO_CRL_MODE6_1 & ~GPIO_CRL_MODE6_0;
    GPIOB->ODR |= GPIO_ODR_ODR6;

    /* PB7 - B, Input Pull-up */
    GPIOB->CRL |= GPIO_CRL_CNF7_1;
    GPIOB->CRL &= ~GPIO_CRL_CNF7_0 & ~GPIO_CRL_MODE7_1 & ~GPIO_CRL_MODE7_0;
    GPIOB->ODR |= GPIO_ODR_ODR7;
    
    /* PC5 - SW, Input Pull-up */
    GPIOC->CRL |= GPIO_CRL_CNF5_1;
    GPIOC->CRL &= ~GPIO_CRL_CNF5_0 & ~GPIO_CRL_MODE5_1 & ~GPIO_CRL_MODE5_0;
    GPIOC->ODR |= GPIO_ODR_ODR5;

    /* PB10 - USART3_TX, передающий канал, AF push-pull */
    GPIOB->CRH |= GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_1 | GPIO_CRH_MODE10_0;
    GPIOB->CRH &= ~GPIO_CRH_CNF10_0;
}

void init_NVIC() {
    /* Настойка SW, кнопки */
    /* Разешить прерывания от 5 линии */
    EXTI->IMR |= EXTI_IMR_IM5;
    /* По нарастающему фронту сигнала */
    EXTI->RTSR |= EXTI_RTSR_RT5;
    /* Соединить 5 линию с портом C (т.е. PC5) */
    AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI5_PC;
    
    NVIC_SetPriority(EXTI9_5_IRQn, 1);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    
    NVIC_SetPriority(TIM4_IRQn, 0);
    NVIC_EnableIRQ(TIM4_IRQn);
}

void init_TIM3() {
    TIM3->ARR = 100;
    TIM3->CCR3 = 0;
    TIM3->CCER |= TIM_CCER_CC3E; // включение выхода канала 3
    TIM3->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1; // режим 2 PWM
}

void init_TIM4() {
    /* Конфигурирование каналов 1(PA6), 2(PA7) таймера 
     * 01 - канал на вход */
    TIM4->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
    TIM4->CCMR1 &= ~TIM_CCMR1_CC1S_1 & ~TIM_CCMR1_CC2S_1;
    /* Режим работы энкодера */
    TIM4->SMCR |=  TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;
    /* Для прерывыния необходимо сигнал TI1 снять триггером */
    TIM4->SMCR |= TIM_SMCR_TS_2;
    TIM4->DIER |= TIM_DIER_TIE;
    
    TIM4->ARR = 39;
    TIM4->CNT = 0;
}

void init_UART() {
    // Включение USART3
    USART3->CR1 |= USART_CR1_UE;
    
    // Нaстрока USART3
    // 9600, 1 стоп бит, 9 бит данных, без проверки четности
    USART3->BRR = 0x341; // Скорость работы 9600
    USART3->CR1 &= ~USART_CR1_M; // 8 бит данных
    USART3->CR1 &= ~USART_CR1_PCE; // без контроля четности
    USART3->CR2 &= ~USART_CR2_STOP_1 & ~USART_CR2_STOP_0; // 1 стоп бит

    // Разрешить передачу
    USART3->CR1 |= USART_CR1_TE;
}

void send_byte(uint8_t byte) {
    /* Ждем завершения предыдущей перелачи */
    while(!(USART3->SR & USART_SR_TC));
    /* Отправляем байт */
    USART3->DR = byte;
}

void msDelay(int ms){
   while(ms-- > 0) {
      volatile int x=500;
      while (x-- > 0)
         __asm("nop");
   }
}
