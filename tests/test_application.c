#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "main_host.inc"

static unsigned char simulated_key;
static unsigned char simulated_rtc[3]={0x12,0x34,0x56};
static unsigned char simulated_eeprom[8];
static unsigned char eeprom_bad_read,dac_ack=1,dac_last;
static unsigned char wave_count;
static unsigned char rtc_write_failed;
static unsigned char rtc_next_day;
static unsigned char iic_sent[8],iic_sent_num,iic_ack_count,iic_fail_at,iic_recv_count;
static unsigned char ds_missing,ds_read_index,ds_bytes[9]={0x50,0x05,0x4b,0x46,0x7f,0xff,0x0c,0x10,0x1c};
static unsigned int extended_checks;
#define CHECK(expression) do {assert(expression);extended_checks++;} while(0)
void System_Init(void) {}
unsigned char Key_Read(void) {return simulated_key;}
void Seg_Disp(unsigned char p,unsigned char d,unsigned char point) {(void)p;(void)d;(void)point;}
void Led_Disp(unsigned char p,unsigned char d) {(void)p;(void)d;}
void Relay_Beep(unsigned char state) {(void)state;}
void IIC_Start(void) {}
void IIC_Stop(void) {}
unsigned char IIC_WaitAck(void) {return ++iic_ack_count==iic_fail_at;}
void IIC_SendAck(unsigned char ack) {(void)ack;}
void IIC_SendByte(unsigned char dat) {if(iic_sent_num<8) iic_sent[iic_sent_num++]=dat;}
unsigned char IIC_RecByte(void) {return iic_recv_count++?153:0;}
void EEPROM_Read(unsigned char *dat,unsigned char addr,unsigned char num)
{(void)addr;memcpy(dat,simulated_eeprom,num);if(eeprom_bad_read) dat[0]^=1;}
void EEPROM_Write(unsigned char *dat,unsigned char addr,unsigned char num)
{(void)addr;memcpy(simulated_eeprom,dat,num);}
unsigned char Dac_Write(unsigned char dat) {dac_last=dat;return dac_ack;}
void Read_Rtc(unsigned char *dat) {memcpy(dat,simulated_rtc,3);}
void Set_Rtc(unsigned char *dat)
{if(!rtc_write_failed) {memcpy(simulated_rtc,dat,3);if(rtc_next_day) memset(simulated_rtc,0,3);}}
void Ut_Wave_Init(void) {wave_count++;}
void Ut_Capture_Init(void) {}
unsigned char init_ds18b20(void) {return ds_missing;}
void Write_DS18B20(unsigned char dat) {(void)dat;}
unsigned char Read_DS18B20(void) {return ds_bytes[ds_read_index++%9];}

static void press(unsigned char key)
{
    unsigned char i;
    simulated_key=key;
    for(i=0;i<4;i++) {key_slow_down=0;key_proc();}
}

static void draw(void) {seg_slow_down=0;seg_proc();}
static void update(void) {led_slow_down=0;led_proc();}

/* 使用真实串口中断逐字节接收，使用真实发送中断取回回复 */
static void receive(const char *text)
{
    while(*text) {SBUF=(unsigned char)*text++;RI=1;uart_service();}
}

static void drain(char *packet)
{
    unsigned int n=0;
    assert(uart_busy);
    packet[n++]=SBUF;
    while(uart_busy)
    {
        TI=1;uart_service();
        if(uart_busy) {assert(n<127);packet[n++]=SBUF;}
    }
    packet[n]=0;
}

static void command(const char *text,const char *expected)
{
    char packet[128];
    assert(!uart_busy);
    receive(text);uart_proc();drain(packet);
    CHECK(strcmp(packet,expected)==0);
}

int main(void)
{
    char packet[128];
    unsigned char n;
    temperature_flag=adc_flag=freg_flag=1;
    temperature=253;voltage=300;freg=5000;
    draw();
    assert(seg[0]==1&&seg[3]==10&&seg[4]==10&&seg[5]==2&&seg[6]==5&&seg[7]==3);
    assert(seg_point[6]==1&&seg_point[5]==0);
    temperature=-55;draw();
    assert(seg[2]==15&&seg[6]==5&&seg[7]==5);
    temperature_flag=0;draw();
    assert(seg[3]==15&&seg[7]==15&&seg_point[6]==0);
    temperature_flag=1;
    seg_disp_mode=1;draw();
    assert(seg[0]==2&&seg[5]==3&&seg[6]==0&&seg[7]==0&&seg_point[5]==1);
    seg_disp_mode=2;draw();
    assert(seg[0]==3&&seg[3]==10&&seg[4]==5&&seg[7]==0);
    freg_flag=0;draw();assert(seg[3]==15&&seg[7]==15);freg_flag=1;
    seg_disp_mode=3;mode=3;freg2=-100;draw();
    assert(seg[0]==4&&seg[1]==4&&seg[2]==15&&seg[5]==1&&seg[6]==0&&seg[7]==0);

    seg_disp_mode=0;mode=0;
    press(4);assert(seg_disp_mode==1);
    press(5);assert(key_down==5); /* Adjacent key codes must remain intact. */
    press(0);seg_disp_mode=3;save_flag=1;
    press(8);assert(temperature_control==31&&save_flag==0);
    press(0);press(9);assert(temperature_control==30);
    press(0);press(5);assert(mode==1);
    press(0);press(8);assert(voltage_control==31);
    press(0);mode=0;temperature_control=99;save_flag=1;
    press(8);assert(temperature_control==99&&save_flag==1);
    press(0);temperature_control=30;voltage_control=30;

    timer_ms=2500;temperature=301;voltage=200;freg=4000;
    update();assert(alarm_flag==1&&relay_flag==1&&ucled[0]==1&&ucled[4]==1);
    press(13);update();assert(mute_flag==1&&(output_state&0x40)==0);
    voltage=301;update();assert(alarm_flag==3&&mute_flag==0);
    temperature=290;voltage=295;update();assert(alarm_flag==3&&relay_flag==1);
    temperature=280;voltage=290;update();assert(alarm_flag==0&&relay_flag==0);

    uart_slow_down=0;uart_proc();n=0;
    packet[n++]=SBUF;
    while(uart_busy)
    {
        TI=1;uart_service();
        if(uart_busy) packet[n++]=SBUF;
    }
    packet[n]=0;
    assert(strcmp(packet,"T=280,V=290,F=4000,A=0,R=0,M=0,S=1,W=1,D=X,DA=X,TIME=12:34:56\r\n")==0);
    /* RTC有效性、保留旧时间、停振后初始化 */
    rtc_init();CHECK(rtc_flag&&ucrtc[0]==0x12&&ucrtc[2]==0x56);
    simulated_rtc[2]=0x80;rtc_init();CHECK(rtc_flag&&ucrtc[0]==0x12&&ucrtc[1]==0&&ucrtc[2]==0);
    simulated_rtc[0]=0x24;rtc_slow_down=0;rtc_proc();CHECK(!rtc_flag);
    simulated_rtc[0]=0x09;simulated_rtc[1]=0x10;simulated_rtc[2]=0x11;
    rtc_slow_down=0;rtc_proc();CHECK(rtc_flag);
    simulated_rtc[1]=0x6a;rtc_slow_down=0;rtc_proc();CHECK(!rtc_flag);
    simulated_rtc[1]=0x10;

    /* 新报警保存时刻；旧报警保持，不反复刷新记录 */
    alarm_flag=0;record_flag=0;temperature=301;voltage=200;freg=4000;
    update();CHECK(record_flag&&record_alarm==1&&record_rtc_flag&&alarm_rtc[2]==0x11);
    simulated_rtc[2]=0x12;update();CHECK(alarm_rtc[2]==0x11);
    freg=5001;update();CHECK(record_alarm==4&&alarm_rtc[2]==0x12);
    command("LOG\r\n","L:A=4@09:10:12\r\n");
    seg_disp_mode=6;record_mode=0;draw();CHECK(seg[0]==7&&seg[7]==4);
    record_mode=1;draw();CHECK(seg[0]==0&&seg[1]==9&&seg[3]==1&&seg[4]==0&&seg[6]==1&&seg[7]==2);
    seg_disp_mode=5;rtc_slow_down=0;rtc_proc();draw();CHECK(seg[2]==15&&seg[5]==15&&seg[7]==2);
    record_rtc_flag=0;seg_disp_mode=6;draw();CHECK(seg[0]==15&&seg[7]==15);

    /* PCA发射、捕获、超时不能改T0/T1/T2 */
    TH0=0x34;TL0=0x56;TH1=0xfc;TL1=0x18;T2H=0xfe;T2L=0xc7;TR0=TR1=1;
    timer_ms=3000;ultrasound_timer=2800;ultrasound_proc();
    CHECK(wave_count==1&&ultrasound_busy&&CCAPM0==0x11&&CR);
    CHECK(TH0==0x34&&TL0==0x56&&TH1==0xfc&&TL1==0x18&&T2H==0xfe&&T2L==0xc7&&TR0&&TR1);
    CCAP0H=5883>>8;CCAP0L=5883&255;CCF0=1;pca_service();
    CHECK(ultrasound_ready&&CCAPM0==0&&!CR);
    timer_ms=3006;ultrasound_proc();CHECK(distance_flag&&distance==100&&!ultrasound_busy);
    seg_disp_mode=4;draw();CHECK(seg[0]==5&&seg[5]==1&&seg[6]==0&&seg[7]==0);
    timer_ms=3200;ultrasound_proc();timer_ms=3229;ultrasound_proc();CHECK(ultrasound_busy);
    timer_ms=3230;ultrasound_proc();CHECK(!ultrasound_busy&&!distance_flag&&CCAPM0==0&&!CR);
    draw();CHECK(seg[3]==15&&seg[7]==15);
    timer_ms=3400;ultrasound_proc();CCAP0H=0;CCAP0L=10;CCF0=1;pca_service();ultrasound_proc();CHECK(!distance_flag);

    /* DAC真实业务调用、芯片不应答和无效频率 */
    freg=1250;freg_flag=1;freg_control=20;dac_slow_down=0;dac_proc();CHECK(dac_value==153&&dac_last==153&&dac_flag);
    freg_flag=0;dac_slow_down=0;dac_proc();CHECK(dac_value==0&&dac_last==0);
    freg_flag=1;dac_ack=0;dac_slow_down=0;dac_proc();CHECK(!dac_flag&&adc_flag);dac_ack=1;

    /* 命令边界、无换行不执行、CRLF不重复执行 */
    command("T=35\r\n","OK\r\n");CHECK(temperature_control==35&&!save_flag);
    command("V=250\n","OK\r\n");CHECK(voltage_control==25);
    command("F=2000\n","OK\r\n");CHECK(freg_control==20);
    command("C=-900\n","OK\r\n");CHECK(freg2==-900);
    command("PAR\n","P:T=35,V=250,F=2000,C=-900\r\n");
    command("T=100\n","ERR:RANGE\r\n");CHECK(temperature_control==35);
    command("T=-1\n","ERR:RANGE\r\n");
    command("V=251\n","ERR:RANGE\r\n");
    command("F=500\n","ERR:RANGE\r\n");
    command("F=20001\n","ERR:RANGE\r\n");
    command("C=-901\n","ERR:RANGE\r\n");
    command("T=327680\n","ERR:FORMAT\r\n");
    command("T=3x\n","ERR:FORMAT\r\n");
    command("GETX\n","ERR:FORMAT\r\n");
    receive("T=40");uart_slow_down=1;uart_proc();CHECK(!uart_busy&&temperature_control==35);
    receive("\n");uart_proc();drain(packet);CHECK(strcmp(packet,"OK\r\n")==0&&temperature_control==40);
    command("TIME=23:59:59\n","OK\r\n");
    command("TIME?\n","TIME=23:59:59\r\n");
    command("TIME=24:00:00\n","ERR:RANGE\r\n");
    command("TIME=12:60:00\n","ERR:RANGE\r\n");
    command("TIME=12-00-00\n","ERR:FORMAT\r\n");
    rtc_write_failed=1;command("TIME=01:02:03\n","ERR:RTC\r\n");rtc_write_failed=0;
    rtc_next_day=1;command("TIME=23:59:59\n","OK\r\n");rtc_next_day=0;

    /* 保存和读回验证，写入期间拒绝修改；修改不自动写EEPROM */
    memset(simulated_eeprom,0,8);timer_ms=4000;
    command("SAVE\n","OK:SAVING\r\n");CHECK(save_state&&simulated_eeprom[1]==40);
    command("T=41\n","ERR:BUSY\r\n");CHECK(temperature_control==40);
    timer_ms=4009;eeprom_proc();CHECK(save_state);
    timer_ms=4010;eeprom_proc();CHECK(!save_state&&save_flag&&!save_error);
    command("T=41\n","OK\r\n");CHECK(!save_flag&&simulated_eeprom[1]==40);
    temperature_control=30;voltage_control=30;freg_control=50;freg2=0;eeprom_init();
    CHECK(temperature_control==40&&voltage_control==25&&freg_control==20&&freg2==-900&&save_flag);
    command("SAVE\n","OK:SAVING\r\n");eeprom_bad_read=1;timer_ms+=10;eeprom_proc();CHECK(save_error&&!save_flag);eeprom_bad_read=0;

    /* 坏帧丢弃到换行，下一条正确命令可以恢复 */
    command("T=12345678901234567890123456789012\n","ERR:RX\r\n");CHECK(temperature_control==40);
    command("T=42\n","OK\r\n");
    for(n=0;n<70;n++) receive("9");
    uart_proc();drain(packet);CHECK(strcmp(packet,"ERR:RX\r\n")==0&&temperature_control==42);
    receive("\n");uart_slow_down=1;uart_proc();CHECK(!uart_busy);
    command("T=43\n","OK\r\n");

    /* 发送期间接收命令，等当前包完成再回复 */
    uart_slow_down=0;uart_proc();CHECK(uart_busy);
    receive("T=44\n");uart_proc();CHECK(uart_busy&&temperature_control==43);
    drain(packet);uart_proc();drain(packet);CHECK(strcmp(packet,"OK\r\n")==0&&temperature_control==44);

    /* T0溢出计入窗口；T1继续运行，捕获逻辑不清零T1 */
    timer_200ms=199;freg_overflow=1;TF0=1;TH0=0;TL0=2;timer_service();
    CHECK(freg_ready&&freg_count==131074&&TR0&&TR1);freg_proc();CHECK(!freg_flag);
    freg_control=50;freg=4000;freg_flag=1;temperature=280;voltage=200;
    update();record_flag=0;command("LOG\n","L=NONE\r\n");

    /* ADC保留DAC使能位和通道3，并处理每个地址阶段不应答 */
    iic_sent_num=iic_ack_count=iic_recv_count=0;iic_fail_at=0;adc_slow_down=0;adc_proc();
    CHECK(adc_flag&&voltage==300&&iic_sent_num==3&&iic_sent[0]==0x90&&iic_sent[1]==0x43&&iic_sent[2]==0x91);
    for(n=1;n<=3;n++)
    {
        iic_ack_count=0;iic_fail_at=n;adc_slow_down=0;adc_proc();CHECK(!adc_flag);
    }
    iic_fail_at=0;

    /* 真正调用温度业务检查CRC、断线与超声波期间让出单总线 */
    ultrasound_busy=0;ds_missing=0;temperature_start=1;temperature_slow_down=0;temperature_proc();
    CHECK(temperature_flag&&temperature==850);
    ds_bytes[8]^=1;temperature_slow_down=0;temperature_proc();CHECK(!temperature_flag);
    ds_bytes[8]^=1;ds_missing=1;temperature_slow_down=0;temperature_proc();CHECK(!temperature_flag&&!temperature_start);
    ds_missing=0;ultrasound_busy=1;temperature_slow_down=0;temperature_proc();CHECK(!temperature_start&&temperature_slow_down==0);
    ultrasound_busy=0;

    /* 最长数据包仍完整结束，且包括新通道与时钟字段 */
    temperature=-550;temperature_flag=1;voltage=500;adc_flag=1;freg=59999;freg_flag=1;
    distance=509;distance_flag=1;dac_value=255;dac_flag=1;alarm_flag=63;
    uart_slow_down=0;uart_proc();drain(packet);
    CHECK(strstr(packet,",D=509,DA=255,TIME=00:00:00\r\n")!=0&&strlen(packet)<128);
    /* 七页完整循环，记录页S5切换，坏时钟显示横线 */
    press(0);seg_disp_mode=0;
    for(n=0;n<7;n++) {press(4);CHECK(seg_disp_mode==(n+1)%7);press(0);}
    seg_disp_mode=6;record_mode=0;press(5);CHECK(record_mode==1);
    press(0);press(5);CHECK(record_mode==0);press(0);
    seg_disp_mode=5;rtc_flag=0;draw();CHECK(seg[0]==15&&seg[7]==15);
    temperature=250;voltage=200;temperature_flag=adc_flag=1;freg_flag=0;alarm_flag=4;
    timer_ms=5500;update();CHECK(alarm_flag==0&&ucled[2]==0&&ucled[3]==1&&(output_state&0x40)==0);
    puts("Existing display/key/alarm/UART regression checks passed");
    printf("%u extended application checks passed: RTC, log, PCA, DAC, commands, EEPROM and RX recovery\n",extended_checks);
    return 0;
}
