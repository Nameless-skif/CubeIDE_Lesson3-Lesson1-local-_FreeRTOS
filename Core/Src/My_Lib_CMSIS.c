#include "My_Lib_CMSIS.h"
#include "stm32f1xx.h"

volatile uint8_t Counter = 0;
volatile uint32_t SysTimer_ms = 0;//переменная, аналогичная HAL_GetTick();
volatile uint32_t Delay_counter_ms = 0;//Счетчие для функции Delay_ms
volatile uint32_t TimeOut_counter_ms = 0;//Переменная для таймаута функции

	void Delay_ms(uint32_t Millisecond){
	Delay_counter_ms = Millisecond;
	 while(Delay_counter_ms != 0){}
	}

//	void SysTick_Handler(void){
//			SysTimer_ms++;
//			if(Delay_counter_ms){
//				Delay_counter_ms--;
//			}
//			if(TimeOut_counter_ms){
//				TimeOut_counter_ms--;
//			}
//		}
	
	void CMSIS_RCC_SystemClock_72MHz(void) {
	/* Настройка тактирования на 72МГц*/
	
	SET_BIT(RCC->CR, RCC_CR_HSION);                             	 //Запустим внутр RC генератор на 8 МГц
	while(READ_BIT(RCC->CR, RCC_CR_HSIRDY) == 0)                	 //дождемся поднятия флага о готовности
	CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);								 //Просто сбросим бит в 0(хотя он и так должен быть в 0)
	SET_BIT(RCC->CR, RCC_CR_HSEON);                             	 //запуск внешнего кварца на 8МГц
	//while(READ_BIT(RCC->CR, RCC_CR_HSERDY) == 0);               	 //ждем поднятия флага о готовности
	SET_BIT(RCC->CR, RCC_CR_CSSON);                             	 //вкл CSS
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);        	 //выберем PLL в качестве System Clock
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL);      	 //используем PLL в качестве System Clock
	MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);   	 //APB Prescaler /1
	MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2);  //010  two wait states, if 48 MHz < SYSCLK <= 72 MHz.
	SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE);                           //Prefetch is enable (В CubeMX вкл и я включил...)
	SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBS);                           //Prefetch buffer is enable (В CubeMX вкл и я включил...)
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV2);      //APB1 Prescaler /2 т.к. PCLK1 max 36MHz
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV1);      //APB2 Prescaler /1 Тут нас ничего не ограничевает
	MODIFY_REG(RCC->CFGR, RCC_CFGR_ADCPRE, RCC_CFGR_ADCPRE_DIV6);    //ADC Prescaler /6, чтобы было 12 МГц, т.к. макс частота тут 14МГц
	SET_BIT(RCC->CFGR, RCC_CFGR_PLLSRC);        				     //В качестве входного сигнала для PLL выберем HSE
	CLEAR_BIT(RCC->CFGR, RCC_CFGR_PLLXTPRE_HSE);				     //  /1 т.к предделение перед PLL нам не нужно
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLMULL, RCC_CFGR_PLLMULL9);	     // т.к. кварц у нас на 8 МГц, а надо 72МГц, то в PLL надо умножить на 9 (8*9=72)
	CLEAR_BIT(RCC->CFGR, RCC_CFGR_USBPRE);							 //Для usb 72/1.5 = 48MHz
	MODIFY_REG(RCC->CFGR, RCC_CFGR_MCO, RCC_CFGR_MCO_PLLCLK_DIV2);	 //В качестве тактирования для MCO выбран PLL. Будет 36 МГц
	SET_BIT(RCC->CR, RCC_CR_PLLON);									 //Запустим PLL
	while(READ_BIT(RCC->CR, RCC_CR_PLLRDY) == 0);					 //Дожидаемся поднятия флага вкл PLL
  }
	
	void CMSIS_SysTick_Timer_Init(void){
		
		
	/* Создание фун-ции задержки delay_ms вместо HAL_Delay
	   Настройка системного таймера
*/
	CLEAR_BIT(SysTick->CTRL,SysTick_CTRL_ENABLE_Msk); // На всякий случай выкл счетчик
	SET_BIT(SysTick->CTRL,SysTick_CTRL_TICKINT_Msk); //Разрешим прерывания по таймеру
	SET_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk); // Выберем без делителя, будет 72 МГц
	MODIFY_REG(SysTick->LOAD, SysTick_LOAD_RELOAD_Msk, 71999 << SysTick_LOAD_RELOAD_Pos); //Настройка на 1 мс
	MODIFY_REG(SysTick->LOAD, SysTick_VAL_CURRENT_Msk, 71999 << SysTick_VAL_CURRENT_Pos);//Начинаем считать с 71999 до 0
	SET_BIT(SysTick->CTRL,SysTick_CTRL_ENABLE_Msk);//Вкл счетчик
	}
	
	void CMSIS_SWD_Init(void) {
	/*Инициализация SWD (PA13, PA14)*/
	RCC->APB2ENR|= RCC_APB2ENR_AFIOEN;//Вкл такт альтернативных функций
	AFIO->MAPR|=AFIO_MAPR_SWJ_CFG_1; //вкл Serial Wire
	}

	void CMSIS_Init_Port_exaple1(void) {/* Включим тактирование порта A*/
	RCC-> APB2ENR|= RCC_APB2ENR_IOPAEN;//ВКЛ такт на порт С
	
	/* 0 пин порта A -> кнопка input */
	GPIOA-> CRL|=GPIO_CRL_CNF0_1;    // 
		GPIOA-> ODR|= 0;//Подтяжка к 0
	
	/*1 пин порта A -> оптореле   output */
	 GPIOA-> CRL&=~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1);   
 	 GPIOA->CRL|= GPIO_CRL_MODE1_1; //cкорость - 10МГц
	
	/*2 пин порта A -> светодиод    output   */
	 GPIOA-> CRL&=~(GPIO_CRL_CNF2 | GPIO_CRL_MODE2); // сбрасываем все в нули
	 GPIOA->CRL|= GPIO_CRL_MODE2_1; //cкорость - 10МГц
	}
	
	void CMSIS_Init_Port_With_EXTI(void){	
		/*Прерывание на ножки PA1, PB0*/
		
		/*Настройка PA1 на вход - floating*/
		SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);//вкл тактирования порта А		
		SET_BIT(GPIOA->CRL, GPIO_CRL_CNF1_0);//	01: floating input
		CLEAR_BIT(GPIOA->CRL, GPIO_CRL_MODE1);//	00: input mode
		
		/* НАстройка PB0 на вход - input with pull- up*/	
		SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPBEN);//вкл тактирования порта B	
		MODIFY_REG(GPIOB->CRL, GPIO_CRL_CNF0, GPIO_CRL_CNF0_1);//	10: input with pull - up
	  CLEAR_BIT(GPIOB->CRL, GPIO_CRL_MODE0);//00: input mode
		GPIOB->BSRR = GPIO_BSRR_BS0; //Вкл подтяжку к питанию
		
		/*Настройка EXTI для ножек PA1 и PB0*/
		//Вкл тактирование альтернативных функций
		SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);
		//Настраиваем ножки на вкл прерываний
		MODIFY_REG(AFIO->EXTICR[0],AFIO_EXTICR1_EXTI0, AFIO_EXTICR1_EXTI0_PB);//Для PB0
		MODIFY_REG(AFIO->EXTICR[0],AFIO_EXTICR1_EXTI1, AFIO_EXTICR1_EXTI1_PA);//Для PA1
		//Вкл прерывания
		SET_BIT(EXTI->IMR, EXTI_IMR_MR0);
		SET_BIT(EXTI->IMR, EXTI_IMR_MR1);
		//Настройка триггера срабатывания - по фронту, спаду, комбинированый
		SET_BIT(EXTI->RTSR, EXTI_RTSR_TR0);//PB0 - только по фронту
		CLEAR_BIT(EXTI->RTSR,EXTI_RTSR_TR1);//PB0 - по спаду выкл
		SET_BIT(EXTI->RTSR, EXTI_RTSR_TR1);//PA1 - по фронту и по спаду
		SET_BIT(EXTI->FTSR, EXTI_FTSR_TR1);
		
		NVIC_EnableIRQ(EXTI0_IRQn);//Вкл прерывание на 1 пины
		NVIC_EnableIRQ(EXTI1_IRQn);//Вкл прерывание на 2 пины
	}
	
	
	void CMSIS_A3_OutPut_PusPull(void){
		SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);//Вкл тактирование
		CLEAR_BIT(GPIOA->CRL, GPIO_CRL_CNF3); //
		SET_BIT(GPIOA->CRL, GPIO_CRL_MODE3);  //50 МГц
	}
	void CMSIS_A3_Blinking(void){
		//Мигаем светодиодом
		GPIOA->BSRR= GPIO_BSRR_BS3;
		Delay_ms(50);
		GPIOA->BSRR= GPIO_BSRR_BR3;
		Delay_ms(50);//
	}
	

	
