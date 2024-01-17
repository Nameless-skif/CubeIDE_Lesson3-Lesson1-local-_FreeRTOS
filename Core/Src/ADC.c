#include "ADC.h"
#include "stm32f1xx.h"

void CMSIS_ADC1(void){
// конфигурирование выводов для аналоговых сигналов (PA0...PA3)
RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // разрешение тактирования порта A
GPIOA->CRL &= ~ (GPIO_CRL_MODE0 | GPIO_CRL_CNF0); // PA0 - аналоговый вход
GPIOA->CRL &= ~ (GPIO_CRL_MODE1 | GPIO_CRL_CNF1); // PA1 - аналоговый вход
GPIOA->CRL &= ~ (GPIO_CRL_MODE2 | GPIO_CRL_CNF2); // PA2 - аналоговый вход
GPIOA->CRL &= ~ (GPIO_CRL_MODE3 | GPIO_CRL_CNF3); // PA3 - аналоговый вход

RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // разрешение тактирование АЦП

	
RCC->CFGR &= ~RCC_CFGR_ADCPRE_0;  // предделитель АЦП = 10 (/6)
RCC->CFGR |=  RCC_CFGR_ADCPRE_1;	
	
ADC1->CR1 = 0;      // запрещаем все в управляющих регистрах
ADC1->CR2 = 0;
	

ADC1->SMPR2 |= ADC_SMPR2_SMP0_0 | ADC_SMPR2_SMP0_1 ; // время выборки 28,5 циклов
ADC1->SMPR2 &= ~ADC_SMPR2_SMP0_2 ;
ADC1->SMPR2 |= ADC_SMPR2_SMP1_0 | ADC_SMPR2_SMP1_1 ; // время выборки 28,5 циклов
ADC1->SMPR2 &= ~ADC_SMPR2_SMP1_2 ;
ADC1->SMPR2 |= ADC_SMPR2_SMP2_0 | ADC_SMPR2_SMP2_1 ; // время выборки 28,5 циклов
ADC1->SMPR2 &= ~ADC_SMPR2_SMP2_2 ;
ADC1->SMPR2 |= ADC_SMPR2_SMP3_0 | ADC_SMPR2_SMP3_1 ; // время выборки 28,5 циклов
ADC1->SMPR2 &= ~ADC_SMPR2_SMP3_2 ;

//--------------- Режим один регулярный канал, однократное преобразование

ADC1->CR2 &= ~ADC_CR2_ADON; // запретить АЦП

// выбор каналов
ADC1->SQR1 =0; // 1 регулярный канал
ADC1->SQR3 =0; // 1 преобразование - канал 0

ADC1->CR2 |= ADC_CR2_CONT; // непрерывный режим
ADC1->CR1 &= ~ADC_CR1_SCAN; // запрет режима сканирования

ADC1->CR2 |= ADC_CR2_ADON; // разрешить АЦП
ADC1->CR2 |= ADC_CR2_CAL; // запуск калибровки
while ((ADC1->CR2 & ADC_CR2_CAL) != 0) ; // ожидание окончания калибровки

ADC1->CR2 |= ADC_CR2_EXTSEL; // источник запуска - SWSTART
ADC1->CR2 |= ADC_CR2_EXTTRIG; // разрешение внешнего запуска для регулярных каналов

ADC1-> CR2|= ADC_CR1_EOCIE;//Разрешение прерываний для бита EOC
}
