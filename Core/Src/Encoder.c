#include "Encoder.h"
#include "stm32f1xx.h"

uint32_t counterEncoder = 0;//Счетчик энкодера
uint32_t counterTimer= 0;

void EncoderInit(void){
	
/*Используем таймер 3 для подключения энкодера
	TIM3_CH1 -> PA6
	TIM3_CH2 -> PA7 
*/
	
	/*Включение тактирования*/	
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);//
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);//

	/*Настройка порта А(PA6, PA7) на альтернативные функции*/
	/*Input analog -> 00:CNF; 00:MODE*/
	
	CLEAR_BIT(GPIOA->CRL,GPIO_CRL_CNF6);	//00:Input floating
	CLEAR_BIT(GPIOA->CRL,GPIO_CRL_CNF7);	//00:Input floating
	CLEAR_BIT(GPIOA->CRL,GPIO_CRL_MODE6);	//
	CLEAR_BIT(GPIOA->CRL,GPIO_CRL_MODE7);	//
	
	CLEAR_BIT(AFIO->MAPR, AFIO_MAPR_TIM3_REMAP);//00:	TIM3_REMAP -> no remap(оставляем порты PA6 - CH1, PA7 - CH2)
  
	/*Настройка таймера 3 на работу с энкодером*/
	//15.3.12 стр 392
	
	SET_BIT(EXTI->IMR, EXTI_IMR_MR6);//Регистр маски прерываний 1:ВКЛ
	SET_BIT(EXTI->RTSR, EXTI_RTSR_TR6);//Регистр включения детектора нарастающего края импульса
	SET_BIT(AFIO->EXTICR[2], AFIO_EXTICR2_EXTI6_PA);//
	
	TIM3->ARR  = 30;//Счет до 30
	SET_BIT(TIM3->CCMR1, TIM_CCMR1_CC1S_1);//01:CC1S
	SET_BIT(TIM3->CCMR1, TIM_CCMR1_CC2S_0);//10:CC2S
		
	
	//MODIFY_REG(TIM3->CCMR1, TIM_CCMR1_IC1F, (TIM_CCMR1_IC1F^TIM_CCMR1_IC1F_1^TIM_CCMR1_IC1F_3));// 0101:IC1F (0101: fSAMPLING=fDTS/2, N=8)
	MODIFY_REG(TIM3->CCMR1, TIM_CCMR1_IC1F, (TIM_CCMR1_IC1F^TIM_CCMR1_IC1F_2^TIM_CCMR1_IC1F_3));// 0011:IC1F (0011: fSAMPLING=fCK_INT, N=8)
	MODIFY_REG(TIM3->CCMR1, TIM_CCMR1_IC2F, (TIM_CCMR1_IC2F^TIM_CCMR1_IC2F_2^TIM_CCMR1_IC2F_3));// 0011:IC2F (0011: fSAMPLING=fCK_INT, N=8)
	
	SET_BIT(TIM3->SMCR, (TIM_SMCR_SMS^TIM_SMCR_SMS_2));// 011:SMS (it is counting on both TI1 and TI2 edges.)
	
	SET_BIT(TIM3->CR1, TIM_CR1_CEN);//Counter is enable
}

void EncoderCounter(int limit){
	//limit - переменная до которой считает энкодер
	counterEncoder = counterEncoder > limit ? 0: counterEncoder;
	
	if(  (READ_BIT(TIM3->CR1, TIM_CR1_DIR) == 0) && (counterTimer != READ_BIT(TIM3->CNT, TIM_CNT_CNT))){ //DIR - направление счета
		Delay_ms(500);
		counterEncoder++;
		counterTimer =  TIM3->CNT;
	} else if( (READ_BIT(TIM3->CR1, TIM_CR1_DIR) != 0) && (counterTimer != READ_BIT(TIM3->CNT, TIM_CNT_CNT)) ) {
		Delay_ms(500);
		counterEncoder--;	
		counterTimer =  TIM3->CNT;
	} 
		counterEncoder = counterEncoder == -1 ? limit: counterEncoder;
}
