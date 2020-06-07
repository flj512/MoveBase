#include <stdio.h>
#include<stm32f0xx_hal.h>
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
extern UART_HandleTypeDef huart1;
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1 ,1000);
    return ch;
}
int __io_getchar(void)
{
	uint8_t ch;
	if(HAL_UART_Receive(&huart1,&ch,1,0xffff)!=HAL_OK){
		return -1;
	}
	return ch;
}
