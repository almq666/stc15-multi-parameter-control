#include <Led.h>

/* 继电器和蜂鸣器共用锁存器，一次写完，避免互相覆盖 */
void Relay_Beep(unsigned char state)
{
	P0=state;
	P2=P2&0x1f|0xa0;
	P2&=0x1f;
}

void Led_Disp(unsigned char addr,unsigned char enable)
{
	static unsigned char temp = 0x00;
	static unsigned char temp_old = 0xff;
	if(enable)
		temp |= 0x01 << addr;
	else
		temp &= ~(0x01 << addr);
	if(temp != temp_old)
	{
		P0 = ~temp;
		P2 = P2 & 0x1f | 0x80;
		P2 &= 0x1f;
		temp_old = temp;
	}
}

void Beep(unsigned char flag)
{
	static unsigned char temp = 0x00;
	static unsigned char temp_old = 0xff;
	if(flag)
		temp |= 0x40;
	else
		temp &= ~0x40;
	if(temp != temp_old)
	{
		P0 = temp;
		P2 = P2 & 0x1f | 0xa0;
		P2 &= 0x1f;
		temp_old = temp;		
	}
}

void Relay(unsigned char flag)
{
	static unsigned char temp = 0x00;
	static unsigned char temp_old = 0xff;
	if(flag)
		temp |= 0x10;
	else
		temp &= ~0x10;
	if(temp != temp_old)
	{
		P0 = temp;
		P2 = P2 & 0x1f | 0xa0;
		P2 &= 0x1f;
		temp_old = temp;		
	}	
}
