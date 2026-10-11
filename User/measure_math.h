#ifndef __MEASURE_MATH_H
#define __MEASURE_MATH_H

/* 频率换算：200ms计数乘5，再加校准值 */
static unsigned char freg_calc(unsigned long count,int freg2,unsigned int *freg)
{
	long int temp_freg;//必须有符号，负校准结果不能存进unsigned int
	if(count>12179UL) return 0;//即使减900Hz，也不能达到60000Hz
	temp_freg=(long int)count*5+freg2;
	if(temp_freg<0||temp_freg>=60000L) return 0;
	*freg=(unsigned int)temp_freg;
	return 1;
}

/* 第十五届分段：500Hz以下1V，上限处5V；无效频率0V */
static unsigned char dac_calc(unsigned int freg,unsigned char valid,unsigned int control)
{
	if(valid==0||control<=500) return 0;
	if(freg<500) return 51;
	if(freg>=control) return 255;
	//先用宽中间值，避免乘204时溢出；不用浮点数
	return 51+(unsigned long)(freg-500)*204/(control-500);
}

/* 回差判断：高于上限触发，降到恢复值才解除 */
static unsigned char alarm_compare(unsigned char flag,long int dat,
	long int control,unsigned int back)
{
	long int temp_control;
	temp_control=control-back;
	if(control>=0&&temp_control<0) temp_control=0;
	if(dat>control) return 1;
	else if(dat<=temp_control) return 0;
	else return flag;//在上限与恢复值之间，保持原来的状态
}

/* 报警判断，温度0.1摄氏度，电压0.01V，频率Hz */
static unsigned char alarm_calc(unsigned char alarm_old,int temperature,
	unsigned int voltage,unsigned int freg,unsigned char temperature_flag,
	unsigned char adc_flag,unsigned char freg_flag,unsigned char start_flag,
	unsigned char temperature_control,unsigned char voltage_control,
	unsigned char freg_control)
{
	unsigned char alarm_flag=0;
	if(start_flag==0) return 0;//前2秒不误报警

	if(temperature_flag)
	{
		if(alarm_compare(alarm_old&0x01,temperature,
			(long int)temperature_control*10,20)) alarm_flag|=0x01;
	}
	else alarm_flag|=0x08;//温度数据无效

	if(adc_flag)
	{
		if(alarm_compare(alarm_old&0x02,voltage,
			(long int)voltage_control*10,10)) alarm_flag|=0x02;
	}
	else alarm_flag|=0x10;//AD芯片不应答

	if(freg_flag)
	{
		if(alarm_compare(alarm_old&0x04,freg,
			(long int)freg_control*100,100)) alarm_flag|=0x04;
	}
	//无效频率不参与越限报警；F=X和故障灯单独提示无效状态

	return alarm_flag;
}

/* 散热请求：温度超限或温度探头故障才启动继电器 */
static unsigned char relay_calc(unsigned char start_flag,unsigned char alarm_flag,
	unsigned char temperature_flag)
{
	if(start_flag==0) return 0;
	if((alarm_flag&0x01)||temperature_flag==0) return 1;
	else return 0;
}

/* 消音处理：新增报警重新响铃，全部恢复以后清除消音 */
static unsigned char mute_calc(unsigned char mute_flag,
	unsigned char alarm_old,unsigned char alarm_flag)
{
	if(alarm_flag==0) return 0;
	if(alarm_flag&~alarm_old) return 0;//现在有、上一轮没有的报警位
	return mute_flag;
}

/* CRC校验：温度数据和EEPROM参数都用这个函数 */
static unsigned char crc_read(unsigned char *dat,unsigned char num)
{
	unsigned char crc=0,temp_dat,i,flag;
	while(num--)
	{
		temp_dat=*dat++;
		for(i=0;i<8;i++)
		{
			flag=(crc^temp_dat)&0x01;
			crc>>=1;
			if(flag) crc^=0x8c;
			temp_dat>>=1;
		}
	}
	return crc;
}
#endif
