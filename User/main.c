/* 头文件声明区 */
#include <STC15F2K60S2.H>//单片机寄存器专用头文件
#include <Init.h>//初始化底层驱动
#include <Key.h>//按键底层驱动
#include <Seg.h>//数码管底层驱动
#include <Led.h>//LED、继电器、蜂鸣器底层驱动
#include <iic.h>//AD采集和EEPROM底层驱动
#include <ds1302.h>//实时时钟底层驱动
#include <ultrasound.h>//超声波发射和PCA初始化
#include "measure_math.h"//频率校准、报警回差和CRC校验

/* 单总线底层函数，保留原驱动的时序 */
extern bit init_ds18b20(void);
extern void Write_DS18B20(unsigned char dat);
extern unsigned char Read_DS18B20(void);

/* 变量声明区 */
unsigned char key_val,key_down,key_old,key_up;//消抖后的按键和按下、松开事件
unsigned char key_temp,key_temp_old,key_count;//连续3次读到相同键码才确认
volatile unsigned char key_slow_down;
volatile unsigned char seg_slow_down;
volatile unsigned char led_slow_down;
volatile unsigned char adc_slow_down;
volatile unsigned int temperature_slow_down;//800ms，不能用unsigned char
volatile unsigned int uart_slow_down;//1000ms，不能用unsigned char
volatile unsigned char rtc_slow_down,dac_slow_down;

volatile unsigned char idata seg[8]={10,10,10,10,10,10,10,10};
volatile unsigned char idata seg_point[8]={0,0,0,0,0,0,0,0};
volatile unsigned char idata ucled[8]={0,0,0,0,0,0,0,0};
unsigned char seg_pos;
unsigned char seg_disp_mode;//0温度 1电压 2频率 3参数 4距离 5时间 6报警记录
unsigned char mode;//参数页：0-温度上限 1-电压上限 2-频率上限 3-频率校准
unsigned char xdata record_mode;//0报警类别，1报警时间
unsigned char xdata ucrtc[3];//时分秒，DS1302用BCD数
unsigned char xdata alarm_rtc[3];
unsigned char xdata rtc_flag,record_flag,record_rtc_flag,record_alarm;
unsigned char xdata dac_value,dac_flag;
unsigned int xdata distance;//距离，单位厘米
unsigned char xdata distance_flag;
volatile unsigned char ultrasound_busy,ultrasound_ready;
volatile unsigned int ultrasound_time;//PCA捕获值，单位us
unsigned int xdata ultrasound_timer;//30ms超时和200ms采样间隔

/* 非中断的采集和参数变量放片内扩展RAM，给直接RAM和栈留空间 */
int xdata temperature;//温度放大10倍，例如253表示25.3摄氏度，可保存负温度
unsigned int xdata voltage;//电压放大100倍，例如300表示3.00V
unsigned int xdata freg;//频率，单位Hz
unsigned char xdata temperature_control=30;//温度上限，单位摄氏度
unsigned char xdata voltage_control=30;//电压上限，放大10倍，30表示3.0V
unsigned char xdata freg_control=50;//频率上限，除以100保存，50表示5000Hz
int xdata freg2;//频率校准值，-900到900Hz，每次增减100

unsigned char xdata temperature_flag;//温度是否有效
unsigned char xdata adc_flag;//电压采集是否成功
unsigned char xdata freg_flag;//频率是否有效
unsigned char xdata temperature_start;//是否已经启动过一次温度转换
unsigned char xdata start_flag;//上电2秒以后开始报警判断
unsigned char xdata alarm_flag,alarm_old;//报警位：1温度 2电压 4频率 8温度故障 16电压故障
unsigned char xdata mute_flag;//1-当前报警已消音
unsigned char xdata relay_flag;//1-请求散热
volatile unsigned char output_state;//bit4继电器 bit6蜂鸣器，一起更新防止互相覆盖

volatile unsigned int timer_ms;//1ms计数，16位回绕，用差值计算经过时间
volatile unsigned char timer_200ms;
volatile unsigned char freg_overflow;//T0每溢出一次加1
volatile unsigned long freg_count;//200ms采样窗内的脉冲数
volatile unsigned char freg_ready;//中断采样完成，主循环可以取走

unsigned char save_flag;//当前参数是否已经保存并读回校验
unsigned char save_state;//1-等待EEPROM写入完成
unsigned char save_error;//1-EEPROM读回校验失败
unsigned int save_timer;
unsigned char xdata eeprom_buf[8];//一页8字节，阈值、校准、校验值

volatile unsigned char uart_busy;//上一包数据还没有发完
volatile unsigned char uart_send_index,uart_send_num;
volatile unsigned char xdata uart_send_buf[128];//主循环写、中断读，最多127字节
volatile unsigned char uart_recv_w,uart_recv_r,uart_recv_error;
volatile unsigned char xdata uart_recv_buf[64];//中断写、主循环读，只在主循环识别换行
unsigned char xdata uart_line[32];//最多31字符，不把字符串解析放进中断
unsigned char uart_line_num,uart_line_ready,uart_discard,uart_error_reply;

/* 检查时分秒BCD格式；包括秒寄存器停振位和12小时模式 */
unsigned char rtc_valid(unsigned char *time)
{
	unsigned char i,value;
	for(i=0;i<3;i++)
	{
		if((time[i]&15)>9||(time[i]>>4)>9) return 0;
		value=(time[i]>>4)*10+(time[i]&15);
		if((i==0&&value>23)||(i!=0&&value>59)) return 0;
	}
	return 1;
}

/* 有有效时间就保留，停振或无效时才设置12:00:00 */
void rtc_init()
{
	Read_Rtc(ucrtc);
	if(rtc_valid(ucrtc)==0)
	{
		ucrtc[0]=0x12;ucrtc[1]=ucrtc[2]=0;
		Set_Rtc(ucrtc);
		Read_Rtc(ucrtc);
	}
	rtc_flag=rtc_valid(ucrtc);
}

void rtc_proc()
{
	if(rtc_slow_down) return;
	rtc_slow_down=1;
	Read_Rtc(ucrtc);
	rtc_flag=rtc_valid(ucrtc);
}

/* 读取1ms计数，防止刚读了一半就进入中断 */
unsigned int timer_read()
{
	unsigned int temp;
	EA=0;
	temp=timer_ms;
	EA=1;
	return temp;
}

/* EEPROM上电读取 */
void eeprom_init()
{
	unsigned char temp_buf[8];
	EEPROM_Read(temp_buf,0,8);
	if(temp_buf[0]==0xa7&&temp_buf[1]<=99&&temp_buf[2]<=50
		&&temp_buf[3]>=6&&temp_buf[3]<=200&&temp_buf[4]<=18
		&&temp_buf[7]==crc_read(temp_buf,7))
	{
		temperature_control=temp_buf[1];
		voltage_control=temp_buf[2];
		freg_control=temp_buf[3];
		freg2=((int)temp_buf[4]-9)*100;//0到18还原成-900到900
		save_flag=1;
	}
	//数据不对就使用变量声明区的默认值，不把坏数据当参数
}

/* EEPROM保存，S12按下时调用 */
void eeprom_save()
{
	if(save_state) return;
	eeprom_buf[0]=0xa7;//标记：这一页是本项目的参数
	eeprom_buf[1]=temperature_control;
	eeprom_buf[2]=voltage_control;
	eeprom_buf[3]=freg_control;
	eeprom_buf[4]=freg2/100+9;//负校准值转换成0到18再保存
	eeprom_buf[5]=eeprom_buf[6]=0;
	eeprom_buf[7]=crc_read(eeprom_buf,7);
	EEPROM_Write(eeprom_buf,0,8);
	save_timer=timer_read();
	save_state=1;
	save_flag=save_error=0;
}

/* EEPROM处理函数 */
void eeprom_proc()
{
	unsigned char temp_buf[8],i;
	unsigned int time;
	if(save_state==0) return;
	time=timer_read();
	if((unsigned int)(time-save_timer)<10) return;//没等够10ms先去做其他任务

	EEPROM_Read(temp_buf,0,8);
	save_flag=1;
	for(i=0;i<8;i++)
	{
		if(temp_buf[i]!=eeprom_buf[i]) save_flag=0;
	}
	save_error=!save_flag;
	save_state=0;
}

/* 键盘处理函数 */
void key_proc()
{
	int temp_control;
	if(key_slow_down) return;
	key_slow_down=1;

	key_temp=Key_Read();
	if(key_temp!=key_temp_old)
	{
		key_temp_old=key_temp;
		key_count=0;
	}
	else if(key_count<3) key_count++;
	if(key_count<3) return;//按键还没稳定，不处理

	key_val=key_temp;
	key_down=key_up=0;
	//键码4、5、8、9是编号，不是位掩码；相邻键切换也要得到完整键码
	if(key_val!=key_old)
	{
		key_down=key_val;
		key_up=key_old;
	}
	key_old=key_val;

	if(mode==0) temp_control=temperature_control;
	else if(mode==1) temp_control=voltage_control;
	else if(mode==2) temp_control=freg_control;
	else temp_control=freg2;

	switch(key_down)
	{
		case 4:
			if(++seg_disp_mode==7) seg_disp_mode=0;
		break;
		case 5:
			if(seg_disp_mode==3)
			{
				if(++mode==4) mode=0;
			}
			else if(seg_disp_mode==6) record_mode^=1;
		break;
		case 8:
			if(seg_disp_mode==3&&save_state==0)
			{
				if(mode==0&&temperature_control<99) temperature_control++;
				else if(mode==1&&voltage_control<50) voltage_control++;
				else if(mode==2&&freg_control<200) freg_control++;
				else if(mode==3&&freg2<900) freg2+=100;
			}
		break;
		case 9:
			if(seg_disp_mode==3&&save_state==0)
			{
				if(mode==0&&temperature_control>0) temperature_control--;
				else if(mode==1&&voltage_control>0) voltage_control--;
				else if(mode==2&&freg_control>6) freg_control--;
				else if(mode==3&&freg2>-900) freg2-=100;
			}
		break;
		case 12:
			if(seg_disp_mode==3) eeprom_save();
		break;
		case 13:
			if(alarm_flag) mute_flag^=1;//只消音，LED和散热输出保留
		break;
	}
	//参数实际改变以后，L6熄灭，提醒还没有保存
	if(key_down==8||key_down==9)
	{
		if((mode==0&&temp_control!=temperature_control)
			||(mode==1&&temp_control!=voltage_control)
			||(mode==2&&temp_control!=freg_control)
			||(mode==3&&temp_control!=freg2))
		{
			save_flag=save_error=0;
		}
	}
}

/* 温度处理函数 */
void temperature_proc()
{
	unsigned char temp_buf[9],i;
	int temp_temperature;
	if(temperature_slow_down) return;
	if(ultrasound_busy) return;//回波测量时不执行关中断的单总线事务
	temperature_slow_down=1;

	//底层按字节保护时序，不能一次关中断读完整个9字节数据
	if(init_ds18b20())
	{
		temperature_flag=0;
		temperature_start=0;
		return;
	}
	Write_DS18B20(0xcc);
	if(temperature_start)
	{
		Write_DS18B20(0xbe);
		for(i=0;i<9;i++) temp_buf[i]=Read_DS18B20();
		temperature_flag=(crc_read(temp_buf,8)==temp_buf[8]);
		if(temperature_flag)
		{
			temp_temperature=(int)(((unsigned int)temp_buf[1]<<8)|temp_buf[0]);
			temperature=(long int)temp_temperature*10/16;//转成0.1摄氏度
			if(temperature<-550||temperature>1250) temperature_flag=0;
		}
		if(init_ds18b20())
		{
			temperature_flag=0;
			temperature_start=0;
			return;
		}
		Write_DS18B20(0xcc);
	}
	Write_DS18B20(0x44);//只启动转换，800ms后再回来读
	temperature_start=1;
}

/* 电压处理函数 */
void adc_proc()
{
	unsigned char temp_adc;
	if(adc_slow_down) return;
	adc_slow_down=1;
	adc_flag=0;

	IIC_Start();
	IIC_SendByte(0x90);
	if(IIC_WaitAck()) {IIC_Stop();return;}//芯片不应答就标记无效
	IIC_SendByte(0x43);//电位器通道
	if(IIC_WaitAck()) {IIC_Stop();return;}
	IIC_Start();
	IIC_SendByte(0x91);
	if(IIC_WaitAck()) {IIC_Stop();return;}
	IIC_RecByte();//第一字节是上一次转换，丢掉
	IIC_SendAck(0);
	temp_adc=IIC_RecByte();
	IIC_SendAck(1);
	IIC_Stop();
	voltage=(unsigned long)temp_adc*500/255;//0到500表示0到5.00V
	adc_flag=1;
}

/* 频率处理函数 */
void freg_proc()
{
	unsigned long temp_count;
	EA=0;
	if(freg_ready==0)
	{
		EA=1;
		return;
	}
	temp_count=freg_count;
	freg_ready=0;
	EA=1;
	freg_flag=freg_calc(temp_count,freg2,&freg);
}

/* DAC与ADC共用I2C，但由主循环顺序调用，不会交叉事务 */
void dac_proc()
{
	if(dac_slow_down) return;
	dac_slow_down=1;
	dac_value=dac_calc(freg,freg_flag,(unsigned int)freg_control*100);
	dac_flag=Dac_Write(dac_value);
}

/* 只发短脉冲，等待回波期间主循环继续执行 */
void ultrasound_proc()
{
	unsigned int time,temp_time;
	time=timer_read();
	if(ultrasound_busy)
	{
		EA=0;
		if(ultrasound_ready)
		{
			temp_time=ultrasound_time;
			ultrasound_ready=0;
			ultrasound_busy=0;
			EA=1;
			distance=(unsigned long)temp_time*17/1000;//1us计数换算厘米
			distance_flag=(temp_time>=118&&temp_time<30000);//约2cm到510cm
			return;
		}
		if((unsigned int)(time-ultrasound_timer)>=30)
		{
			CCAPM0=0;CR=0;CCF0=0;
			ultrasound_busy=0;
			distance_flag=0;
		}
		EA=1;
		return;
	}
	if((unsigned int)(time-ultrasound_timer)<200) return;
	ultrasound_timer=time;
	EA=0;
	CCAPM0=0;CR=0;CCF0=0;
	Ut_Wave_Init();//约200us的短发射，不长时间等待回波
	CL=CH=0;
	ultrasound_ready=0;
	ultrasound_busy=1;
	CCAPM0=0x11;//下降沿捕获、允许模块中断；大赛接收电路空闲为高
	CR=1;
	EA=1;
}

/* 信息处理函数 */
void seg_proc()
{
	unsigned char temp_seg[8],temp_point[8],i,point=7;
	unsigned char data_flag=1,negative_flag=0,time_page=0;
	unsigned int temp_value;
	if(seg_slow_down) return;
	seg_slow_down=1;

	for(i=0;i<8;i++)
	{
		temp_seg[i]=10;
		temp_point[i]=0;
	}
	temp_seg[0]=seg_disp_mode+1;//页面编号

	switch(seg_disp_mode)
	{
		case 0://温度界面，保留一位小数
			data_flag=temperature_flag;
			if(temperature<0) {negative_flag=1;temp_value=-temperature;}
			else temp_value=temperature;
			temp_point[6]=1;
			point=6;
		break;
		case 1://电压界面，保留两位小数
			temp_value=voltage;
			data_flag=adc_flag;
			temp_point[5]=1;
			point=5;
		break;
		case 2://频率界面，单位Hz
			temp_value=freg;
			data_flag=freg_flag;
		break;
		case 3://参数界面
			temp_seg[1]=mode+1;
			if(mode==0) temp_value=temperature_control;
			else if(mode==1)
			{
				temp_value=(unsigned int)voltage_control*10;
				temp_point[5]=1;
				point=5;
			}
			else if(mode==2) temp_value=(unsigned int)freg_control*100;
			else
			{
				if(freg2<0) {negative_flag=1;temp_value=-freg2;}
				else temp_value=freg2;
			}
		break;
		case 4://距离界面，单位厘米，不参与报警
			temp_value=distance;
			data_flag=distance_flag;
		break;
		case 5://当前时间，显示HH-MM-SS
			time_page=1;
			data_flag=rtc_flag;
		break;
		case 6://报警类别与报警时刻回显
			if(record_mode)
			{
				time_page=2;
				data_flag=record_flag&&record_rtc_flag;
			}
			else
			{
				temp_value=record_alarm;
				data_flag=record_flag;
			}
		break;
		default:
			temp_value=0;
			data_flag=0;
		break;
	}
	if(time_page)
	{
		for(i=0;i<8;i++) temp_seg[i]=15;
		if(data_flag)
		{
			for(i=0;i<3;i++)
			{
				if(time_page==1) temp_value=ucrtc[i];
				else temp_value=alarm_rtc[i];
				temp_seg[i*3]=temp_value>>4;
				temp_seg[i*3+1]=temp_value&15;
			}
		}
	}
	else if(data_flag)
	{
		temp_seg[3]=temp_value/10000%10;
		temp_seg[4]=temp_value/1000%10;
		temp_seg[5]=temp_value/100%10;
		temp_seg[6]=temp_value/10%10;
		temp_seg[7]=temp_value%10;
		i=3;
		while(i<point&&temp_seg[i]==0)//高位熄灭，小数点前至少保留一个0
		{
			temp_seg[i]=10;
			i++;
		}
		if(negative_flag) temp_seg[2]=15;//负号
	}
	else
	{
		for(i=3;i<8;i++) temp_seg[i]=15;//无效读数显示横线
		temp_point[5]=temp_point[6]=0;
	}
	EA=0;
	for(i=0;i<8;i++)//整组更新，防止中断扫到半新半旧的数据
	{
		seg[i]=temp_seg[i];
		seg_point[i]=temp_point[i];
	}
	EA=1;
}

/* LED、继电器和蜂鸣器处理函数 */
void led_proc()
{
	unsigned int time;
	unsigned char temp_output,i,new_alarm;
	if(led_slow_down) return;
	led_slow_down=1;

	time=timer_read();
	if(start_flag==0&&time>=2000) start_flag=1;
	alarm_old=alarm_flag;
	alarm_flag=alarm_calc(alarm_old,temperature,voltage,freg,
		temperature_flag,adc_flag,freg_flag,start_flag,
		temperature_control,voltage_control,freg_control);
	new_alarm=alarm_flag&~alarm_old;
	if(new_alarm)
	{
		Read_Rtc(ucrtc);//报警触发时再读取，不能只使用旧的缓存时间
		rtc_flag=rtc_valid(ucrtc);
		for(i=0;i<3;i++) alarm_rtc[i]=ucrtc[i];
		record_alarm=new_alarm;
		record_rtc_flag=rtc_flag;
		record_flag=1;
	}
	mute_flag=mute_calc(mute_flag,alarm_old,alarm_flag);//新增报警重新响铃
	relay_flag=relay_calc(start_flag,alarm_flag,temperature_flag);

	ucled[0]=(alarm_flag&0x01)!=0;//温度越限
	ucled[1]=(alarm_flag&0x02)!=0;//电压越限
	ucled[2]=(alarm_flag&0x04)!=0;//频率越限
	ucled[3]=((alarm_flag&0x18)!=0)||(start_flag&&freg_flag==0);//频率无效只提示故障灯
	ucled[4]=relay_flag;//散热请求
	ucled[5]=save_flag;//参数已保存
	ucled[6]=mute_flag;//已消音
	ucled[7]=(time%1000<500);//运行闪烁

	temp_output=0;
	if(relay_flag) temp_output|=0x10;
	if(alarm_flag&&mute_flag==0&&time%1000<100) temp_output|=0x40;
	output_state=temp_output;//中断统一写继电器和蜂鸣器，避免互相覆盖
}

/* 串口数据放入发送数组 */
void uart_put_char(unsigned char dat)
{
	if(uart_send_num<127) uart_send_buf[uart_send_num++]=dat;
}

void uart_put_num(unsigned int dat)
{
	unsigned char temp_buf[5],i=0;
	do
	{
		temp_buf[i++]=dat%10+'0';
		dat/=10;
	}
	while(dat);
	while(i) uart_put_char(temp_buf[--i]);
}

void uart_put_text(char code *text)
{
	while(*text) uart_put_char(*text++);
}

void uart_put_time(unsigned char *time,unsigned char valid)
{
	unsigned char i;
	if(valid==0) {uart_put_char('X');return;}
	for(i=0;i<3;i++)
	{
		if(i) uart_put_char(':');
		uart_put_char((time[i]>>4)+'0');
		uart_put_char((time[i]&15)+'0');
	}
}

void uart_start_send()
{
	uart_put_char('\r');uart_put_char('\n');
	ES=0;
	uart_send_index=0;
	TI=0;
	uart_busy=1;
	SBUF=uart_send_buf[uart_send_index++];
	ES=1;
}

/* 当前数据包，周期上报和GET查询共用 */
void uart_report()
{
	unsigned char state;
	uart_put_char('T');uart_put_char('=');
	if(temperature_flag)
	{
		if(temperature<0)
		{
			uart_put_char('-');
			uart_put_num(-temperature);
		}
		else uart_put_num(temperature);
	}
	else uart_put_char('X');
	uart_put_char(',');uart_put_char('V');uart_put_char('=');
	if(adc_flag) uart_put_num(voltage);
	else uart_put_char('X');
	uart_put_char(',');uart_put_char('F');uart_put_char('=');
	if(freg_flag) uart_put_num(freg);
	else uart_put_char('X');
	uart_put_char(',');uart_put_char('A');uart_put_char('=');
	uart_put_num(alarm_flag);
	uart_put_char(',');uart_put_char('R');uart_put_char('=');
	uart_put_char(relay_flag+'0');
	uart_put_char(',');uart_put_char('M');uart_put_char('=');
	uart_put_char(mute_flag+'0');
	if(save_error) state=3;
	else if(save_state) state=2;
	else if(save_flag) state=1;
	else state=0;
	uart_put_char(',');uart_put_char('S');uart_put_char('=');
	uart_put_char(state+'0');
	uart_put_char(',');uart_put_char('W');uart_put_char('=');
	uart_put_char(start_flag+'0');
	uart_put_text(",D=");
	if(distance_flag) uart_put_num(distance);
	else uart_put_char('X');
	uart_put_text(",DA=");
	if(dac_flag) uart_put_num(dac_value);
	else uart_put_char('X');
	uart_put_text(",TIME=");
	uart_put_time(ucrtc,rtc_flag);
}

/* 命令按整行精确比较，不接受前缀相同的多余字符 */
unsigned char uart_equal(char code *text)
{
	unsigned char i=0;
	while(text[i])
	{
		if(i>=uart_line_num||uart_line[i]!=text[i]) return 0;
		i++;
	}
	return i==uart_line_num;
}

/* 解析有符号十进制，先检查范围再乘10，不让坏输入溢出 */
unsigned char uart_get_num(unsigned char pos,int *num)
{
	unsigned int value=0;
	unsigned char negative=0,digit;
	if(pos>=uart_line_num) return 0;
	if(uart_line[pos]=='-') {negative=1;pos++;}
	if(pos>=uart_line_num) return 0;
	while(pos<uart_line_num)
	{
		if(uart_line[pos]<'0'||uart_line[pos]>'9') return 0;
		digit=uart_line[pos++]-'0';
		if(value>3276||(value==3276&&digit>7)) return 0;
		value=value*10+digit;
	}
	if(negative) *num=-(int)value;
	else *num=value;
	return 1;
}

/* 完整命令只在主循环处理；回复发送期间仍能接收字节 */
void uart_command()
{
	int value;
	unsigned char i,value8;
	unsigned char temp_rtc[3];
	if(uart_equal("GET")) {uart_report();return;}
	if(uart_equal("PAR"))
	{
		uart_put_text("P:T=");uart_put_num(temperature_control);
		uart_put_text(",V=");uart_put_num((unsigned int)voltage_control*10);
		uart_put_text(",F=");uart_put_num((unsigned int)freg_control*100);
		uart_put_text(",C=");
		if(freg2<0) {uart_put_char('-');uart_put_num(-freg2);}
		else uart_put_num(freg2);
		return;
	}
	if(uart_equal("LOG"))
	{
		if(record_flag==0) {uart_put_text("L=NONE");return;}
		uart_put_text("L:A=");uart_put_num(record_alarm);uart_put_char('@');
		uart_put_time(alarm_rtc,record_rtc_flag);
		return;
	}
	if(uart_equal("TIME?"))
	{
		Read_Rtc(ucrtc);rtc_flag=rtc_valid(ucrtc);
		uart_put_text("TIME=");uart_put_time(ucrtc,rtc_flag);
		return;
	}
	if(uart_equal("SAVE"))
	{
		if(save_state) uart_put_text("ERR:BUSY");
		else {eeprom_save();uart_put_text("OK:SAVING");}
		return;
	}
	if(uart_line_num==13&&uart_line[0]=='T'&&uart_line[1]=='I'
		&&uart_line[2]=='M'&&uart_line[3]=='E'&&uart_line[4]=='=')
	{
		if(uart_line[7]!=':'||uart_line[10]!=':') {uart_put_text("ERR:FORMAT");return;}
		for(i=0;i<3;i++)
		{
			value8=5+i*3;
			if(uart_line[value8]<'0'||uart_line[value8]>'9'
				||uart_line[value8+1]<'0'||uart_line[value8+1]>'9')
			{uart_put_text("ERR:FORMAT");return;}
			temp_rtc[i]=(uart_line[value8]-'0')*16+uart_line[value8+1]-'0';
		}
		if(rtc_valid(temp_rtc)==0) {uart_put_text("ERR:RANGE");return;}
		Set_Rtc(temp_rtc);Read_Rtc(ucrtc);rtc_flag=rtc_valid(ucrtc);
		for(i=0;i<3;i++) if(ucrtc[i]!=temp_rtc[i]) break;
		if(i!=3)
		{
			//允许设置后自然过去1秒，包括59秒、59分和23点进位
			for(i=3;i;i--)
			{
				value8=(temp_rtc[i-1]>>4)*10+(temp_rtc[i-1]&15)+1;
				if((i==1&&value8==24)||(i!=1&&value8==60)) temp_rtc[i-1]=0;
				else {temp_rtc[i-1]=value8/10*16+value8%10;break;}
			}
			for(i=0;i<3;i++) if(ucrtc[i]!=temp_rtc[i]) break;
		}
		if(rtc_flag==0||i!=3) uart_put_text("ERR:RTC");
		else uart_put_text("OK");
		return;
	}
	if(uart_line_num<3||uart_line[1]!='='||uart_get_num(2,&value)==0)
	{uart_put_text("ERR:FORMAT");return;}
	if(save_state) {uart_put_text("ERR:BUSY");return;}
	switch(uart_line[0])
	{
		case 'T':
			if(value<0||value>99) {uart_put_text("ERR:RANGE");return;}
			temperature_control=value;
		break;
		case 'V':
			if(value<0||value>500||value%10) {uart_put_text("ERR:RANGE");return;}
			voltage_control=value/10;
		break;
		case 'F':
			if(value<600||value>20000||value%100) {uart_put_text("ERR:RANGE");return;}
			freg_control=value/100;
		break;
		case 'C':
			if(value<-900||value>900||value%100) {uart_put_text("ERR:RANGE");return;}
			freg2=value;
		break;
		default:
			uart_put_text("ERR:FORMAT");return;
	}
	save_flag=save_error=0;//修改立即生效，但只有SAVE或S12才保存
	uart_put_text("OK");
}

void uart_proc()
{
	unsigned char dat;
	if(uart_recv_error)
	{
		ES=0;
		uart_recv_r=uart_recv_w;
		uart_recv_error=0;
		ES=1;
		uart_line_num=uart_line_ready=0;
		uart_discard=1;//溢出后丢弃到换行，不能把命令尾部误当新命令
		uart_error_reply=1;
	}
	while(uart_line_ready==0&&uart_recv_r!=uart_recv_w)
	{
		dat=uart_recv_buf[uart_recv_r];
		uart_recv_r=(uart_recv_r+1)&63;
		if(dat=='\r'||dat=='\n')
		{
			if(uart_discard) {uart_discard=0;uart_line_num=0;}
			else if(uart_line_num) uart_line_ready=1;
		}
		else if(uart_discard==0)
		{
			if(dat<32||dat>126||uart_line_num==31)
			{uart_line_num=0;uart_discard=1;uart_error_reply=1;}
			else uart_line[uart_line_num++]=dat;
		}
	}
	if(uart_busy) return;
	uart_send_num=0;
	if(uart_error_reply)
	{
		uart_error_reply=0;uart_put_text("ERR:RX");
	}
	else if(uart_line_ready)
	{
		uart_command();
		uart_line_num=uart_line_ready=0;
	}
	else
	{
		if(uart_slow_down) return;
		uart_slow_down=1;
		uart_report();
	}
	uart_start_send();
}

/* 定时器0计数初始化 */
void Timer0_Init(void)
{
	AUXR&=0x7f;
	TMOD&=0xf0;
	TMOD|=0x05;//T0外部计数，16位
	P34=1;//释放T0输入，键盘扫描不能再占用P3.4
	TL0=TH0=0;
	TF0=0;
	ET0=1;
	PT0=1;//T0溢出中断优先级高于T1采样
	TR0=1;
}

/* 定时器1初始化，1ms@12MHz */
void TimerInit(void)
{
	AUXR&=0xbf;
	TMOD&=0x0f;
	//本芯片模式0是16位自动重装，FC18在12T、12MHz下为1ms
	TL1=0x18;
	TH1=0xfc;
	TF1=0;
	ET1=1;
	TR1=1;
	EA=1;
}

/* 串口初始化，9600bps@12MHz */
void UartInit(void)
{
	P_SW1&=0x3f;//UART1固定使用P3.0/P3.1，不切换到DS1302引脚
	SCON=0x50;
	AUXR|=0x05;
	T2L=0xc7;
	T2H=0xfe;
	AUXR|=0x10;
	ES=1;
}

/* 定时器0溢出中断 */
void timer0_service() interrupt 1
{
	if(freg_overflow<255) freg_overflow++;
}

/* 定时器1中断服务函数 */
void timer_service() interrupt 3
{
	unsigned long temp_count;
	timer_ms++;
	if(++key_slow_down==10) key_slow_down=0;
	if(++seg_slow_down==10) seg_slow_down=0;
	if(++led_slow_down==10) led_slow_down=0;
	if(++adc_slow_down==100) adc_slow_down=0;
	if(++rtc_slow_down==200) rtc_slow_down=0;
	if(++dac_slow_down==100) dac_slow_down=0;
	if(++temperature_slow_down==800) temperature_slow_down=0;
	if(++uart_slow_down==1000) uart_slow_down=0;

	if(++timer_200ms==200)
	{
		timer_200ms=0;
		TR0=0;
		ET0=0;
		temp_count=((unsigned long)freg_overflow<<16)|((unsigned int)TH0<<8)|TL0;
		if(TF0) temp_count+=65536UL;//还没进入T0中断的溢出也要补上
		TH0=TL0=freg_overflow=0;
		TF0=0;
		ET0=1;
		TR0=1;
		freg_count=temp_count;
		freg_ready=1;
	}
	Seg_Disp(seg_pos,seg[seg_pos],seg_point[seg_pos]);
	Led_Disp(seg_pos,ucled[seg_pos]);
	if(++seg_pos==8) seg_pos=0;
	Relay_Beep(output_state);
}

/* 串口中断服务函数 */
void uart_service() interrupt 4
{
	unsigned char dat,next;
	if(RI)
	{
		dat=SBUF;RI=0;
		next=(uart_recv_w+1)&63;
		if(next==uart_recv_r) uart_recv_error=1;
		else {uart_recv_buf[uart_recv_w]=dat;uart_recv_w=next;}
	}
	if(TI)
	{
		TI=0;
		if(uart_send_index!=uart_send_num)
		{
			SBUF=uart_send_buf[uart_send_index];
			uart_send_index++;
		}
		else uart_busy=0;
	}
}

/* PCA下降沿捕获，取第一组回波后停止捕获 */
void pca_service() interrupt 7
{
	if(CCF0)
	{
		CCAPM0=0;CR=0;
		ultrasound_time=((unsigned int)CCAP0H<<8)|CCAP0L;
		CCF0=0;
		ultrasound_ready=1;
	}
	if(CF) CF=0;
}

/* Main */
void main()
{
	System_Init();
	eeprom_init();
	rtc_init();
	Ut_Capture_Init();
	Timer0_Init();
	TimerInit();
	UartInit();
	temperature_proc();//先启动温度转换，800ms后读取
	uart_slow_down=1;//上电先等一轮，再发送第一包
	while(1)
	{
		key_proc();
		ultrasound_proc();
		temperature_proc();
		adc_proc();
		freg_proc();
		rtc_proc();
		eeprom_proc();
		led_proc();
		dac_proc();
		seg_proc();
		uart_proc();
	}
}
