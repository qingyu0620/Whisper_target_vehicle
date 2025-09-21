#include "Delay.h"


void Delay_us(uint32_t xus)
{
	uint32_t temp;
	//设定重装载值 72M = 1ns
	SysTick -> LOAD = 200 * xus;						//输入所需定时时间
	//清空当前计数
	SysTick -> VAL = 0;								//要求计数器重所需时间值开始递减计数
	//设置CTRL寄存器位
	SysTick ->CTRL |= 0X05;							//位0 开启计数器，位1 关闭中断，位2 使用1分频
	//等待计数完成，countflag置1
	do{
		temp = SysTick->CTRL;
	}
	while((!(temp & (1 << 16))) && (temp & 0X01));	//确定countflag是否置1，计数器是否开启
	SysTick -> CTRL |= 0X00;
}


void Delay_ms(uint32_t xms)
{
	while(xms--)
		Delay_us(1000);
}
