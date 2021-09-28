/*
 * File:   newmain.c
 * Author: Claudio Ohara
 *
 * Created on 27 de Setembro de 2021, 17:49
 */


//INCLUSÃO DE BIBLIOTECAS
#include <xc.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//CONFIGURATION BITS    
//CONFIG1H
#pragma config FOSC = HS
//CONFIG2L
#pragma config PWRT = ON 
#pragma config BOR = ON
#pragma config BORV = 1
//CONFIG2H
#pragma config WDT = OFF
//CONFIG3H
#pragma config PBADEN = ON 
#pragma config MCLRE = OFF
//CONFIG4L
#pragma config LVP = OFF 
#pragma config DEBUG = OFF

//DEFINIÇÕES

#define N           32
#define	n_TMR0      28
#define _XTAL_FREQ  8000000

#define rs          LATD0
#define en          LATD1
#define Idata       LATB
#define vref        5

//#define	CANAL_8		0b00100001
//#define	CANAL_9		0b00100101
//#define	AN_8		0b00000110
//#define	AN_9		0b00000101
#define	CANAL_0		0b00000001
#define	CANAL_1		0b00000101
#define	AN_0		0b00001110
#define	AN_1		0b00001101
#define	DIV16_12TAD	0b10101101

//VARIÁVEIS GLOBAIS

unsigned int a[N], b[N];
char k, canal;

char data[10];
long int digital;
float voltage;

//FUNÇÕES AUXILIARES

void LCD_Init();
void LCD_Command(char);
void LCD_Char(char x);
void LCD_String(const char*);
void LCD_String_xy(char, char, const char*);

void inic_regs();

unsigned int Conv_AD();
void Config_ADCONS(unsigned char, unsigned char, unsigned char);

//INTERRUPÇÃO DE ALTA PRIORIDADE

__interrupt(high_priority) void samples(void){
	unsigned int n;
    if (INTCONbits.TMR0IF)
    {
        INTCONbits.TMR0IF = 0;
        TMR0L = n_TMR0;
        n = Conv_AD();
        if (!canal)
        {
            a[k] = n;
        }
        else
        {
            b[k] = n;
        }
        k++;
    }
}

//FUNÇÃO MAIN
void main(){

	inic_regs();
	LCD_Init();
	LCD_String_xy(1,1,"Voltage is ...");
	
	while(1){
		for (canal = 0; canal < 2; canal++){
			switch (canal){
				case 0: Config_ADCONS(CANAL_0, AN_0, DIV16_12TAD);
                break;
				
				case 1: Config_ADCONS(CANAL_1, AN_1, DIV16_12TAD);
                break;
			}
			
			INTCONbits.GIE = 1;
			TMR0L = n_TMR0;
            k=0;
						
		    while (k < N){
                T0CONbits.TMR0ON = 1;
            }

			INTCONbits.GIE = 0;
		}
	}
}

//DEFINIÇÃO DAS FUNÇÕES AUXILIARES

void inic_regs()
{
    TRISA = 0x3;
    TRISB = 0;
    TRISC = 0;
	TRISD = 0;
	
    OSCCON = 0;
	T0CON = 0b01001000;
    INTCON = 0b00100000;
    INTCON2 = 0b00000100;
    RCON = 0b10000000;
	
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
	PORTD = 0;
    PORTE = 0;
}

unsigned int Conv_AD(){

    unsigned int result_AD;
	
	ADRESH = 0;
	ADRESL = 0;

    ADCON0bits.GO=1;
    
	while(ADCON0bits.GO){
    result_AD=(((unsigned int)ADRESH)<<8)|(ADRESL);
	}
	
	//digital = (ADRESH*256)|(ADRESL);
	digital = result_AD;
	voltage = digital*((float)vref/(float)1023);
	sprintf(data,"%.2f",voltage);
	strcat(data,"V");
	LCD_String_xy(2,4,data);
	
    return	result_AD;
}

void Config_ADCONS(	unsigned char adcon0_config, unsigned char adcon1_config, unsigned char adcon2_config){
    ADCON0=adcon0_config;
    ADCON1=adcon1_config;
    ADCON2=adcon2_config;
    ADCON2bits.ADFM = 1;	//Sempre justificado à direita.
}

void LCD_Init(){
	//TRISB = 0;
	//TRISD = 0;
	__delay_ms(5);
	LCD_Command(0x38);
	LCD_Command(0x01);
	LCD_Command(0x06);
	LCD_Command(0x0C);
}

void LCD_Command(char cmd){
	Idata = cmd;
	rs = 0;
	en = 1;
	__delay_ms(1);
	en = 0;
	__delay_ms(3);
}

void LCD_Char(char dat){
	Idata = dat;
	rs = 1;
	en = 1;
	__delay_ms(1);;
	en = 0;
	__delay_ms(3);
}

void LCD_String(const char *msg){
	while((*msg)!=0){
		LCD_Char(*msg);
		msg++;
	}
}

void LCD_String_xy(char row, char pos, const char *msg){
	char location = 0;
	if(row<=1){
		location = (0x80)| ((pos)&0x0F);
		LCD_Command(location);
	}else{
		location = (0xC0)| ((pos)&0x0F);
		LCD_Command(location);
	}
	LCD_String(msg);
}
