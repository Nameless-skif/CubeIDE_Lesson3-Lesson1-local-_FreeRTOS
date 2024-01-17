#include "Timer3_PWM.h"
#include "stm32f1xx.h"
//----------------------------------------------

void CMSIS_Timer3_PWM_Init(){
	
	
		//Вкл тактирование таймера (стр 48)
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);//Вкл тактирования таймера
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);//Вкл тактирования альтернативных фун- ций 
	
	//Настройка таймера 3 (стр 404)
	//15.4.1 TIMx Control register 1(TIMx_CR1)
	
	
	CLEAR_BIT(TIM3->CR1, TIM_CR1_URS);//Генерировать событие Update
	CLEAR_BIT(TIM3->CR1, TIM_CR1_UDIS);//Генерировать прерывание
	CLEAR_BIT(TIM3->CR1, TIM_CR1_OPM);//One-pulse mode выкл
	CLEAR_BIT(TIM3->CR1, TIM_CR1_DIR);//Счет идет вверх
	CLEAR_BIT(TIM3->CR1, TIM_CR1_CMS);//00: Выравнивание по краю
	SET_BIT(TIM3->CR1, TIM_CR1_ARPE);//Auto-reload preload enable  - включим
	CLEAR_BIT(TIM3->CR1, TIM_CR1_CKD);//00: Предделитель выключим
	
	
	//Настроим прерывания по таймеру (стр 408)
	//15.4.4 TIMx DMA/Interrupt enable register (TIMx_DIER)
	SET_BIT(TIM3->DIER, TIM_DIER_UIE);//Включение прерываний
	
	//Настройка регистра статуса TIMx status register (TIMx_SR)


	TIM3->PSC = 7200-1;//(6 -> f=1MHz); (7200 - 1 -> 1kHz) Регистр предделителя, можно регулировать частоту выходного сигнала
	TIM3->ARR = 11-1;//Счетчик будет считать до 10   Регистр перезагрузки
	//Разрешаем прерывания по таймеру 3
	//NVIC_EnableIRQ(TIM3_IRQn);
	
	SET_BIT(TIM3->CR1, TIM_CR1_CEN);//Запуск таймера
	
	                                /*Насройка ножки PA6 под ШИМ*/
																	
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);//Вкл тактирование порта А
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF6, GPIO_CRL_CNF6_1);//10: Настраиваем на альтернативную функцию, выход с push- pull
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE6, GPIO_CRL_MODE6);//11: Вкл на макс частот 50МГц
	
															/*Настройка ШИМ на PA6(канал 1)  (стр387)*/
																	
	CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_CC1S);//Канал 1 на выход
	CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_OC1FE);//Fast mode выкл
	SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC1PE);//Предзагрузка вкл
	CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_OC1M);//Очистим бит OC1M
    SET_BIT(TIM3->CCMR1, ((TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1) & TIM_CCMR1_OC1M));//PWM mode 1(в бит OC1M надо выставить 0b110)
	SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC1CE);//Внешний сигнал с канала ETRF не действует на выкл ШИМ
	
	/*Запуск ШИМ на PA6*/
	//15.4.9 TIMx capture/compare enable register (TIMx_CCER)
	//	SET_BIT(TIM3->CCER, TIM_CCER_CC1E);//1: On - OC1 signal is output on the corresponding output pin.

	SET_BIT(TIM3->CCER, TIM_CCER_CC1P);//1: OC1 active high.
	
	TIM3->CCR1 = 5;//Управление скважностью
	
														/*Настроем ножки PA7(канал 2) ШИМ с One-pulse mode вкл*/
															
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF7, GPIO_CRL_CNF7_1);//10: Настраиваем на альтернативную функцию, выход с push- pull
	MODIFY_REG(GPIOA->CRL, GPIO_CRL_MODE7, GPIO_CRL_MODE7);//11: Вкл на макс частот 50МГц
	
														/*Настройка ШИМ на PA7  (стр387)*/
	CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_CC2S);//Канал 2 на выход
	CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_OC2FE);//Fast mode выкл
	SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC2PE);//Предзагрузка вкл
	CLEAR_BIT(TIM3->CCMR1, TIM_CCMR1_OC2M);//Очистим бит OC1M
    SET_BIT(TIM3->CCMR1, ((TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1) & TIM_CCMR1_OC2M));//PWM mode 1(в бит OC1M надо выставить 0b110)
	SET_BIT(TIM3->CCMR1, TIM_CCMR1_OC2CE);//Внешний сигнал с канала ETRF не действует на выкл ШИМ												
	
	/*Запуск ШИМ на PA7*/
	//	SET_BIT(TIM3->CCER, TIM_CCER_CC2E);//1: On - OC1 signal is output on the corresponding output pin.

	CLEAR_BIT(TIM3->CCER, TIM_CCER_CC2P);//0: OC1 active low.
	
	TIM3->CCR2 = 5;//Управление скважностью
}

void CMSIS_Timer3_PWM_PA6_ON()
	{
	SET_BIT(TIM3->CCER, TIM_CCER_CC1E);//1: On - OC1 signal is output on the corresponding output pin.
	}

void CMSIS_Timer3_PWM_PA6_Off()
	{
	CLEAR_BIT(TIM3->CCER, TIM_CCER_CC1E);//1: On - OC1 signal is output on the corresponding output pin.
	}

void CMSIS_Timer3_PWM_PA7_ON()
	{
	SET_BIT(TIM3->CCER, TIM_CCER_CC2E);//1: On - OC1 signal is output on the corresponding output pin.
	}

void CMSIS_Timer3_PWM_PA7_Off()
	{
	CLEAR_BIT(TIM3->CCER, TIM_CCER_CC2E);//1: On - OC1 signal is output on the corresponding output pin.
	}

void CMSIS_Timer3_PWM_DutyCycleControl(uint8_t dutyCycle1, uint8_t dutyCycle2, uint32_t frequence, uint32_t count){
	// uint8_t dutyCycle1 - скважность 1 канала PA6
	// uint8_t dutyCycle2 - скважность 2 канала PA7
	//uint32_t frequence - (6 -> f=1MHz); (7200 - 1 -> 1kHz) Регистр предделителя, можно регулировать частоту выходного сигнала
	//uint32_t count - Счетчик будет считать count
	TIM3->CCR1 = dutyCycle1;//Управление скважностью PA6
	TIM3->CCR2 = dutyCycle2;//Управление скважностью PA7
	TIM3->PSC = frequence;//
	TIM3->ARR = count;//Счетчик будет считать до 10   Регистр перезагрузки	
	}
