#include "stm32f1xx.h"

#define START (TIM3->CR1 |= TIM_CR1_CEN)

void init_pin(); /* Настройка пинов */
void init_NVIC(); /* Система прерываний */
void init_TIM3(); /* Формирование PWM для управления двигателем */
void init_TIM4(); /* Счет испульсов от энкодера */
void msDelay(uint32_t ms); /* Функция задержки */


void TIM4_IRQHandler() {
    TIM3->CCR3 = TIM4->CNT;
    GPIOC->ODR ^= GPIO_ODR_ODR9;
    msDelay(150);
    // Сбросить флаг прерывания
    TIM4->SR &= ~TIM_SR_TIF;
}

int main(void) {
    // Включение тактирования портов B, C, AFIOEN
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;
    // Включение тактирования TIM3, TIM4 
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
    
    init_pin();
    init_NVIC();
    init_TIM3();
    init_TIM4();

    START;

    while(1) {

    }
    return 0;
}

void init_pin() {
    /* PC8 - Alternative Function Push-Pull,  TIM3_Ch3 */
    GPIOC->CRH |= GPIO_CRH_CNF8_1 | GPIO_CRH_MODE8_1 | GPIO_CRH_MODE8_0;
    GPIOC->CRH &= ~GPIO_CRH_CNF8_0;

    AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP;


    GPIOC->CRH |= GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0;
    GPIOC->CRH &= ~GPIO_CRH_CNF9_1 & ~GPIO_CRH_CNF9_0;

    /* PB6(A) - TIM4_Ch1, Input PU */
    GPIOB->CRL |= GPIO_CRL_CNF6_1;
    GPIOB->CRL &= ~GPIO_CRL_CNF6_0 & ~GPIO_CRL_MODE6_1  & ~GPIO_CRL_MODE6_0;
    GPIOB->ODR |= GPIO_ODR_ODR6;
    /* PB7(B) - TIM4_Ch2, Input PU */
    GPIOB->CRL |= GPIO_CRL_CNF7_1;
    GPIOB->CRL &= ~GPIO_CRL_CNF7_0 & ~GPIO_CRL_MODE7_1 & ~GPIO_CRL_MODE7_0;
    GPIOB->ODR |= GPIO_ODR_ODR7;
}

void init_NVIC() {
    NVIC_SetPriority(TIM4_IRQn, 1);
    NVIC_EnableIRQ(TIM4_IRQn);
}

void init_TIM3() {
    //TIM3->PSC = 7999;
    TIM3->ARR = 100;
    TIM3->CCR3 = 70;
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
    TIM4->ARR |= 100;
    TIM4->CNT = 70;
    TIM4->CR1 |= TIM_CR1_CEN;
}

void msDelay(uint32_t ms){
   while(ms-- > 0) {
      volatile short x = 500;
      while (x-- > 0)
         __asm("nop");
   }
}