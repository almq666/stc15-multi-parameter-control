#ifndef MOCK_STC15_H
#define MOCK_STC15_H
/* Host-only register stand-ins. These are never included in the C51 build. */
#define bit unsigned char
#define code
#define idata
#define xdata
unsigned char EA,ES,AUXR,TMOD,P34,TH0,TL0,TF0,ET0,PT0,TR0;
unsigned char TH1,TL1,TF1,ET1,TR1,SCON,T2L,T2H,RI,TI,SBUF;
unsigned char CCAPM0,CCAP0H,CCAP0L,CCF0,CF,CR,CL,CH;
unsigned char P_SW1;
#endif
