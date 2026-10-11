#include <ultrasound.h>
#include "intrins.h"

sbit Tx = P1^0;
sbit Rx = P1^1;

void Delay12us()		//@12.000MHz
{
	unsigned char i;

	_nop_();
	_nop_();
	i = 33;
	while (--i);
}

void Ut_Wave_Init() //沿用大赛驱动，产生8个约40kHz的脉冲
{
	unsigned char i;
	for(i=0;i<8;i++)
	{
		Tx = 1;
		Delay12us();
		Tx = 0;
		Delay12us();
	}
}

void Ut_Capture_Init() //PCA独立计时，不占用T0、T1和T2
{
	P_SW1&=0xcf;//CCP0默认映射P1.1
	Tx=0;
	Rx=1;
	CCON=0;
	CMOD=0;//12MHz/12=1MHz，每个计数1us
	CCAPM0=CCAPM1=CCAPM2=0;
	CL=CH=0;
	PPCA=1;//优先处理回波，减少重复回波覆盖首个捕获值
}
