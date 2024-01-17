/*
 * Lesson2_USART1_DMA1_ADC2.c
 *
 *  Created on: 17 янв. 2024 г.
 *      Author: Win_7
 */

#include "stm32f1xx.h"
#include "stdbool.h"
#include "stdio.h"
#include "Lesson2_USART1_DMA1_ADC2.h"

bool CommandReceived = false;
char RxBuffer[100];
char TxBuffer[100];

uint16_t channel_Adc_Convert(uint8_t channel);

//char RxBuffer[100] = {"ADC"};
//bool CommandReceived = true;

void USART1_IRQHandler(void){
	  if(USART1->SR & USART_SR_RXNE){ 	   // Проверка, что данные пришли
		  uint8_t len = strlen(RxBuffer);  // Значение последнего свободного элемента в RxBuffer
		  RxBuffer[len] = USART1->DR;      // В последний свободный элемент RxBuffer записываем считанное значение из регистра DR
		  if((RxBuffer[len] == 0x0A) && (RxBuffer[len - 1] ==0x0D)){  		//Значит строка пришла целиком
			  CommandReceived = true;
			  return;
		  }
	  }
 }

void init_USART1(void){
	 		  RCC->APB2ENR |= RCC_APB2ENR_USART1EN; //
	 		  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;	//вкл альтернативных функций
	 		  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

	 		  //Настройка портов под USART1:  PA9 - TX(выход),  PA10 - RX(вход)
	 		  CLEAR_BIT(GPIOA-> CRH, GPIO_CRH_CNF9);    										 // очищаем биты для PA9
	 		  CLEAR_BIT(GPIOA-> CRH, GPIO_CRH_CNF10) | CLEAR_BIT(GPIOA-> CRH, GPIO_CRH_MODE10);  // очищаем биты для PA10

	 		  SET_BIT(GPIOA-> CRH, GPIO_CRH_CNF9_1);	// PA9 - выход на алтер. функцию
	 		  SET_BIT(GPIOA-> CRH, GPIO_CRH_MODE9_0);	// 10 MHz

	 		  SET_BIT(GPIOA-> CRH, GPIO_CRH_CNF10_1);	// PA10 - Input (input pull- up)
	 		  SET_BIT(GPIOA-> CRH, GPIO_CRH_MODE10_0);	// PA10 - 10 MHz (уже настроен на вход алтер функции)

	 		  GPIOA->CRH &= (~GPIO_CRH_CNF9_0);
	 		  GPIOA->CRH |= (GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9);

	 		  // Настройка USART1

	 		  //Настройка скорости передачи - 115200 бод (см стр 798)
	 		  // 72000000/16/115200 = 39.1
	 		  //mantissa = 0d39 (hex -> 0x27); fraction = 0.1 * 16 = 2 ->hex 0x272

	 		  USART1->BRR = 7500; // для 9600 бод

	 		  USART1->CR1 |= USART_CR1_UE;    	  // On USART
	 		  USART1->CR1 |= USART_CR1_TE;    	  // On transmitter
	 		  USART1->CR1 |= USART_CR1_RE;    	  // On receiver
	 		  USART1->CR1 |= USART_CR1_RXNEIE;    // Разрешает прерывания по приему данных из USART

	 		  NVIC_EnableIRQ(USART1_IRQn);	      //Разрешаем прерывания по USART1
 }

void init_DMA1_USART1(){
	RCC->AHBENR|= RCC_AHBENR_DMA1EN;
	//	Для USART2_TX нам нужен 4 канал DMA
	DMA1_Channel4->CPAR = (uint32_t)& USART1->DR; //
	DMA1_Channel4->CCR|= DMA_CCR_MINC;			  // Инкремент указателя для памяти
	DMA1_Channel4->CCR|= DMA_CCR_DIR;			  //Читаем из памяти
	USART1->CR3|= USART_CR3_DMAT;				  //Разрешить DMA для передатчика USART (см стр 825 Datasheet)
}

void send_String_DMA1(char *str ){
	strcat(str,"\r\n"); //
	DMA1_Channel4-> CCR &=~ DMA_CCR_EN_Msk;	//Выкл канал
	DMA1_Channel4-> CMAR = (uint32_t)str;	//Регистр указатель на область памяти. Просто записали нашу строку
	// str - это массив(и указатель на 1-ый элемент массива)
	DMA1_Channel4-> CNDTR = (uint32_t)strlen(str);//Кол-во передаваемых данных(т.е. длина этой строки)
	DMA1-> IFCR = DMA_IFCR_CTCIF4;	//Очищаем флаг окончания передачи по DMA
	DMA1_Channel4-> CCR |= DMA_CCR_EN_Msk;//Разрешаем передачу по DMA
}

void txStr(char *str, bool crlf){
	if(crlf)
		strcat(str, "\r\n");		// \r -> символ D, \n -> символ А (возврат каретки и перенос строки)

	//побайтово отправляем строку
	for(uint8_t i = 0; i< strlen(str); i++){
		USART1->DR = str[i];
		Delay_ms(1);
		while(USART1->SR & USART_SR_TC){}		// Статус бит, когда передача закончилась
	}
}

void ExecuteCommand(void){
	char ADC_char_res[100];											// Переменная для записи результата ADC(для вывода в USART)
	memset(TxBuffer,0,100); 										// Очищаем буфер передачи
	if(strncmp(RxBuffer,"*IDN&", 5) == 0)							//Функция сравнения строк,
		{
			strcpy(TxBuffer, "Stm32f103C8T6 test board");			//
		}
	else if(strncmp(RxBuffer,"start1", 6) == 0)
		{
			strcpy(TxBuffer, "Led 1 On");
			GPIOA ->BSRR|= GPIO_BSRR_BS3;
		}
	else if(strncmp(RxBuffer, "stop1", 5) == 0)
		{
			strcpy(TxBuffer, "Led 1 Off");
			GPIOA -> BSRR|= GPIO_BSRR_BR3;
		}
	else if(strncmp(RxBuffer, "start2", 6) == 0)
		{
			strcpy(TxBuffer, "Led 2 On");
			CMSIS_Timer3_PWM_Init();
			//TIM3->PSC = 1000000;  //регулировка частоты сигнала
			CMSIS_Timer3_PWM_PA6_ON();
		}
	else if(strncmp(RxBuffer, "stop2", 5) == 0)
			{
			strcpy(TxBuffer, "Led 2 Off");
			CMSIS_Timer3_PWM_PA6_Off();
			}
	else if(strncmp(RxBuffer, "dutycycle", 9) == 0)
		{
			//  от 1 до 10
			uint16_t value = 0;
			/*     *-> игнорирование; hu -> полуслово беззнаковое        */
			sscanf(RxBuffer, "%*s %hu", &value);
			if((value >= 1) && (value <=10))
				{
					CMSIS_Timer3_PWM_Init();
					TIM3->CCR1 = value;			// регулировка скважности
					strcpy(TxBuffer,"Input parametr correct" );
					Delay_ms(10);
				}
			else{
				strcpy(TxBuffer,"Invalid parametr");
				Delay_ms(10);
				}
		}
	else if(strncmp(RxBuffer, "ADC", 3) == 0)	//Возвращаем значение в байтах для АЦП
		{
			uint16_t adc_res = channel_Adc_Convert(8); 	//
			sprintf(ADC_char_res, "%d", adc_res);		// "%d" - маска
			strcpy(TxBuffer, ADC_char_res);
		}
	else if(strncmp(RxBuffer, "U", 1) == 0)
		{
			float u_res = (float)channel_Adc_Convert(8) * 3.3f / 4095.0f;
			sprintf(ADC_char_res, "%1.2f", u_res);
			strcpy(TxBuffer, ADC_char_res);
		}
	else if(strncmp(RxBuffer,"reset",5)	== 0)
		{
			strcpy(TxBuffer, "Led 1 Off; Led 2 Off");
			GPIOA -> BSRR|= GPIO_BSRR_BR3;
			CMSIS_Timer3_PWM_PA6_Off();
		}
	else
		{
			strcpy(TxBuffer, "Error! Unknown command! ");
		}
	send_String_DMA1(TxBuffer);
//	txStr(TxBuffer, true);    	//

	memset(RxBuffer, 0, 100);	// 1-ый параметр -> указатель на область памяти, где хотим что то поменять,
						        // 2-ой параметр -> подставляется то, что мы хотим записать
							    // 3-ий параметр -> сколько байт
	CommandReceived = false;
}

void Delay(uint_fast32_t ms)
	{
	 	 for(volatile uint32_t i = 0; i < ms; i++);
	}

void init_ADC2()
	{
		// будем использовать ножку PB0
		// ADC12_IN8 - Datasheet стр 29
		RCC->APB2ENR|= RCC_APB2ENR_IOPBEN;
		RCC->APB2ENR|= RCC_APB2ENR_AFIOEN;
		RCC->APB2ENR|= RCC_APB2ENR_ADC2EN;

		/* см  GPIO configurations for device peripherals стр 166
		 * и конкретно таблицу 22 на стр 169
		*/
		CLEAR_BIT(GPIOB->CRL, GPIO_CRL_CNF0);
		CLEAR_BIT(GPIOB->CRL, GPIO_CRL_MODE0);

		/* Настройка АЦП; регулярное измерение, вкл измерения по программе */
		SET_BIT(ADC2->SMPR2, ADC_SMPR2_SMP8);   //Для 8 канала время преобразования 239.5 цикла
		SET_BIT(ADC2-> CR2, ADC_CR2_EXTSEL);	//   ---> запуск программно по биту SWSTART <----
		SET_BIT(ADC2->CR2, ADC_CR2_EXTTRIG);	//
		ADC2->CR2 |= ADC_CR2_CONT; // непрерывный режим преобразования
		SET_BIT(ADC2-> CR2, ADC_CR2_ADON);		// вкл АЦП
		Delay(3);						 //
		SET_BIT(ADC2->CR2, ADC_CR2_CAL); //вкл калибровку после задержки в 2 цикла (см reference manial стр 223   Calibration)

		// Для каждого while надо делать проверку(с таймером), чтобы мы не зависли в нем
		while(ADC2->CR2 & ADC_CR2_CAL); //Ждем пока бит калибровки не сбросится в 0
	}

uint16_t channel_Adc_Convert(uint8_t channel)	//принимает на вход номер канала с которым мы будем работать и возвращать результат измерения
	{
		uint16_t mass[10] = {0};
												//Надо делать проверки, правильное ли число мы записали в channel
		ADC2->SQR3|= channel;
		for(uint8_t i = 0; i<=9; i++)			//Делаем 10 измерений для надежности
		{
			ADC2->CR2|= ADC_CR2_SWSTART;
			while(!(ADC2->SR & ADC_SR_EOC)){};		//Ждем пока преобразование завершится по биту EOC = 1
			mass[i] = ADC2->DR;
		}
		uint16_t res = (mass[0] + mass[1] + mass[2] + mass[3] + mass[4] + mass[5] + mass[6] + mass[7] + mass[8] + mass[9]) / 10;
		return res;
//		return ADC2->DR;
	}
