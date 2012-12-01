/*
	Simple Quad Hybrid V1 Motor Control based on RC type Pitch/Roll signals
	
	Pin Connections:
	RA5	- PULSIN[0] - CONTROLS MOTORS 1 & 3 (PITCH FORWARD / BACK)
	RA4	- PULSIN[1] - CONTROLS MOTORS 2 & 4 (ROLL RIGHT / LEFT)
	
								+ Config	X Config
	RC5/CCP1 - PWM0 - Motor 0 	Forward		Fwd+Left
	RC3/CCP2 - PWM1 - Motor 1 	Right		Fwd+Right
	RA2/CCP3 - PWM2 - Motor 2 	Back		Back+Right
	RC1/CCP4 - PWM3 - Motor 3 	Left		Back+Left
	
	
	RA0 - TX (for debug), also used as PGD during programming
	RC4 - Set "X" or "+"  fly configuration:   "+" - WHEN PULLED UP,DEFAULT,  "X"  WHEN PULLED DOWN WITH JUMPER

	X-Conifg		+ Config
	0  1			   0
	 \/				   |
	 /\				3--|--1	
	3  2			   |
					   2
					   
	RC0 - Reserved / Optional debug led
	RC2 - Reserved / Future Use
					   
*/


#include <htc.h>
#include <stdio.h>

#define _XTAL_FREQ 32000000			//32Mhz FOSC

__CONFIG(FOSC_INTOSC &  WDTE_OFF   &  PWRTE_ON  & MCLRE_OFF    & CP_OFF  & CPD_OFF  & BOREN_OFF  & CLKOUTEN_OFF    & IESO_OFF  &  FCMEN_OFF  );
__CONFIG( WRT_OFF  &  PLLEN_ON  & STVREN_OFF /*& BORV_LO*/  & LVP_OFF  );


#define MIN(A,B)  (((A)<(B)) ? (A) : (B) )
#define MAX(A,B)  (((A)>(B)) ? (A) : (B) )
#define PUT_IN_RANGE(V,VMIN,VMAX) MAX(VMIN,MIN(VMAX,V))
#define MAP_TO_RANGE(V,VMIN0,VMAX0,VMIN1,VMAX1) ( (VMIN1) +  ( PUT_IN_RANGE(V,VMIN0,VMAX0) - (VMIN0) ) * ( (VMAX1) - (VMIN1) ) / ( (VMAX0) - (VMIN0) ) )

//******************************************************
//customize putchar() function used by printf()
//******************************************************
void putch(unsigned char c){
	while(!TXIF);
	TXREG=c;
}

//******************************************************
//WARNING THESE VARIABLES ARE VOLATILE , DO NOT ACCESS IN MAIN CODE UNLESS INERRUPTS ARE DISABLED
//******************************************************
#define PULSE_N	2
volatile unsigned int pulseLength[PULSE_N] = {0,0};
volatile unsigned int pulseStart[PULSE_N] = {0,0};
volatile unsigned char pulseOn[PULSE_N] = {0,0};
volatile unsigned char signalAlive = 0;

//******************************************************
// USE THIS FUNCTION TO READ PLUSIN, IT CALCULATES pulsePercent:
// 1000us => -100%  ...  1500us => 0 ... 2000us => 100%
//******************************************************
unsigned int pulseLengthStable[PULSE_N] = {0,0};
int pulsePercent[PULSE_N] = {0,0};
void pulseRead(void){
	unsigned char i;
	int tmp;
	//test if signal is alive
	signalAlive = 0;
	__delay_ms(40);		//in a 20ms interval there must be at least one pulse that would set signalAlive = 1 (see interupt routine)						

	if(signalAlive){
		GIE = 0;					//disable interrupts, copy voltile vars as fast as possible	
		for(i=0;i<PULSE_N;i++) pulseLengthStable[i] = pulseLength[i];
		GIE = 1;					//enable interrupts
	}else{
		for(i=0;i<PULSE_N;i++) pulseLengthStable[i] = 1500;		//assume 0 position
	}

	for(i=0;i<PULSE_N;i++){	//now we have time to play with them
		if( 900  < pulseLengthStable[i]  && pulseLengthStable[i] < 2100){ 	//valid RC pulse 
			tmp = pulseLengthStable[i] - 1000;	// (1000..2000)  =>  (0 .. 1000)
			if(tmp < 0) tmp = 0; 
			if(tmp > 1000) tmp = 1000;	
			tmp = (tmp + 2)/5;					// (0..1000) => (0 .. 200) with rounding
			pulsePercent[i] = tmp - 100;		// (0..200) => (-100 .. 100) 
		}else{
			pulsePercent[i] = 0;
		}
	} 

}

//******************************************************
// MOTOR DUTY 
//******************************************************
int motorDuty[4] ;


void motorsApplyDuty(){
	unsigned char i;
	//TODO: glitch control ?  50, 51 , 0 , 50  => 52 (median) ?

	for(i=0;i<4;i++){
		motorDuty[i] = PUT_IN_RANGE(motorDuty[i],0,100);
		if(motorDuty[i] < 5) motorDuty[i] = 0;	//create a dead-zone up to 5% duty
	}

	//Duty Cycle Ratio =  CCPRxL:CCPxCON<5:4> / (4*(PRx + 1)) = CCPRxL / (PRx + 1) = CCPRxL / 160 =>
	//Note that CCPxCON<5:4> is not significant (we don't need that resolution)
	//CCPRxL = (Duty Ratio) * 160 = (Duty Percent) * 160 / 100 = (Duty Percent) * 16 / 10
	//macro to convert percent to CCPRxL value, with rounding
	#define PERCENT2CCPRxL(percent) ((percent  * 16 + 5) / 10 )	

	CCPR1L = PERCENT2CCPRxL( motorDuty[0]);
	CCPR2L = PERCENT2CCPRxL( motorDuty[1]);
	CCPR3L = PERCENT2CCPRxL( motorDuty[2]);
	CCPR4L = PERCENT2CCPRxL( motorDuty[3]);

}

//******************************************************
// MAIN 
//******************************************************
void main(void){
	unsigned char i;
	//****************************
	//oscilator
	//****************************
	OSCCONbits.IRCF = 0b1110;	//1110 = 8 MHz or 32 MHz HF with 4xPLL enabled
	OSCCONbits.SCS = 0;			//Clock determined by FOSC<2:0> in Configuration Word 1
	while(!OSCSTATbits.HFIOFS);	//wait for oscilator to be 0.5% stable

	//****************************
	//ports
	//****************************
	ANSELA = 0;					//Set RAx Ports to I/O digital (they are analog on reset by default)
	ANSELC = 0;					//Set RCx Ports to I/O digital (they are analog on reset by default)

	TRISA = 0xFF;				//All ports are input by default
	TRISC = 0xFF;

	WPUA = 0;					//All individual pullups disabled
	WPUC = 0;
	nWPUEN = 0;					//Use individual pullups

	WPUC4 =1;					//C4 has a weak pull-up
	
	//TRISC0 = 0;					//OPTIONAL DEBUG LED

	//****************************
	//timer1 (used to measure RC pulse)
	//****************************
	T1CONbits.TMR1CS = 0b00;	//Timer1 source FOSC/4 = 8Mhz
	T1CONbits.T1CKPS = 0b11;	//Timer1 1:8 prescaler => 1Mhz, 1us/tick	
	TMR1 = 0;					//Reset counter
	T1CONbits.TMR1ON = 1;		//Timer1 is on

	//****************************
	//usart (for debug)
	//****************************
	TXCKSEL = 1;				//TX/CK function is on RA0 (RC4 is the default)
	SPBRGH = 0;					//Baud Rate = FOSC / (64*(SPBRGH:SPBRGL + 1)) =
	SPBRGL = 51;				// = 32Mhz / 64 / 52 = 9615 ~ 9600
	TXEN = 1;					//enable transmiter
	SYNC = 0;					//asyncronos opperation
	SPEN = 1;					//enable EUSART module
	TRISA0 = 0;					//make TX (RA0) pin output

	//****************************	
	//interrupt on change for PULSIN FUNCTION
	//****************************
	IOCAP5 = 1;	IOCAN5 = 0;		//first interrupt when RA5 goes up
	IOCAP4 = 1;	IOCAN4 = 0;		//first interrupt when RA4 goes up


	//****************************
	//PWM Module Setup
	//****************************
	CCP2SEL = 0;				//CCP2 function is on RC3 (default)
	//PWM Period = (PRx+1)*4*TOSC*(T2Presc) = (159+1)*4*1/32us*1 = 20us => Freq = 50kHZ	
	PR2 = 159;					//Timer2 Module Period Register
	//CCPx in PWM mode
	CCP1CONbits.CCP1M=0b1100;CCP2CONbits.CCP2M =0b1100;CCP3CONbits.CCP3M=0b1100;CCP4CONbits.CCP4M=0b1100;
	//set PWM duty to zero
	for(i=0;i<4;i++) motorDuty[i]=0;
	motorsApplyDuty();
	//Timer 2 setup & start
	CCPTMRS = 0;				//CCPx is based off Timer2 in PWM Mode
	TMR2IF = 0;	
	T2CONbits.T2OUTPS = 0;		//Timer2 postscaler is 1
	T2CONbits.T2CKPS = 1;		//Timer2 prescaler is 4
	TMR2ON = 1;					//Turn on Timer2
	while(!TMR2IF);				//wait for first Timer2 overflow
	TRISC5 = 0;					//make pins RC5,RC3,RA2,RC1 output
	TRISC3 = 0;	
	TRISA2 = 0;	
	TRISC1 = 0;		


	//****************************	
	//enable interrupts
	//****************************
	IOCIE = 1;				//enable Interrupt On Change
	//PEIE = 1;				//enable peripheral interrupts (needed if timer interrupts are used)
	GIE=1;					//enable interrupts in general

	//****************************
	//main loop
	//****************************
	while(1){

	
		pulseRead();
		
		for(i=0;i<4;i++) motorDuty[i]=0; 

		
		if(RC4){  
			//********************************	
			// "+" config if RC4 is pulled-up (P/X jumper removed)
			//********************************
			//Control pitch: Motors 0 & 2
			if(pulsePercent[0] >= 0){
				motorDuty[2] = pulsePercent[0];
			}else{
				motorDuty[0] = -pulsePercent[0];
			}
	
			//Control roll: Motors 1 & 3
			if(pulsePercent[1] >= 0){
				motorDuty[3] = pulsePercent[1];
			}else{
				motorDuty[1] = -pulsePercent[1];
			}
		}else{		
			//********************************
			//"X" config if RC4 is pulled-down (P/X jumper removed)
			//********************************

			if(pulsePercent[0] >= 0){	//Pitch Forward - motors  2 & 3
				motorDuty[2] = pulsePercent[0];
				motorDuty[3] = pulsePercent[0];
			}else{						//Pitch Back - motors  0 & 1
				motorDuty[0] = -pulsePercent[0];
				motorDuty[1] = -pulsePercent[0];
			}
	
			//Control roll: Motors 1 & 3
			if(pulsePercent[1] >= 0){	//Pitch Right - motors 0 & 3
				motorDuty[0] += pulsePercent[1];
				motorDuty[3] += pulsePercent[1];
			}else{						//Pitch Left - motors 1 & 2		
				motorDuty[1] += -pulsePercent[1];
				motorDuty[2] += -pulsePercent[1];
			}
		}
	
	
		motorsApplyDuty();


		//debug output 9600bps => 960bytes/sec => 19.2 bytes/20ms (pulsin interval)
		for(i=0;i<PULSE_N;i++){ 
			printf("%u,%d ",pulseLengthStable[i],pulsePercent[i]);
		}
		printf("\n");

		
	};
}

//******************************************************
// INTERRUPT 
//******************************************************
void interrupt interrupt_routine(void){
	
	if(IOCIF){ // "interrupt on change" 

		signalAlive = 1;
		
		//NOTE: overflows are fine when computing pulseLength = TMR1 - pulseStart
		//when (TMR1<pulseStart) since for example 0x0007-0xFFFF = 8 (as expected)
		//(as long as pulses are no longer than 0xFFFF ticks)

		IOCIF = 0;		//clear interrupt flag

		//RA5 
		if(IOCAF5){
			if(IOCAP5){	
				pulseStart[0] = TMR1;							
				IOCAP5 = 0; IOCAN5 = 1;		//next interrupt on falling edge
			}else{						
				pulseLength[0] = TMR1 - pulseStart[0];			
				IOCAP5 = 1; IOCAN5 = 0;		//next interrupt on rising edge
			};
			IOCAF5 = 0;
		}

		//RA4 
		if(IOCAF4){
			if(IOCAP4){	
				pulseStart[1] = TMR1;							
				IOCAP4 = 0; IOCAN4 = 1;		//next interrupt on falling edge
			}else{						
				pulseLength[1] = TMR1 - pulseStart[1];			
				IOCAP4 = 1; IOCAN4 = 0;		//next interrupt on rising edge
			};
			IOCAF4 = 0;
		}
	}
}
