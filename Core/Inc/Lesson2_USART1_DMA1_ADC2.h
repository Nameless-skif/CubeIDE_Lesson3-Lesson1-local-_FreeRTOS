/*
 * Lesson2_USART1_DMA1_ADC2.h
 *
 *  Created on: 17 янв. 2024 г.
 *      Author: Win_7
 */

#ifndef INC_LESSON2_USART1_DMA1_ADC2_H_
#define INC_LESSON2_USART1_DMA1_ADC2_H_




void USART1_IRQHandler(void);
void init_USART1(void);
void init_DMA1_USART1(void);
void send_String_DMA1(char *str );
void txStr(char *str, bool crlf);
void ExecuteCommand(void);
void Delay(uint_fast32_t ms);
void init_ADC2();
uint16_t channel_Adc_Convert(uint8_t channel);


#endif /* INC_LESSON2_USART1_DMA1_ADC2_H_ */
