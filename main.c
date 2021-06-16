#include "stdio.h"
#include <stdlib.h>
#include "tm4c123gh6pm.h"
#include <string.h>
#include <Math.h>
#include "lcd.h"
#include <stdbool.h>


char distancat[100];
double theta1;
double theta2 ;
double delta1;
double delta2;
double a;
double c ;
double d;
const int R = 6371*1000; //Radius of earth in (km)
const double PI=3.14159265358979323846;
double lat1,lat2;
double lon1,lon2;
double	lati1;
double long1 ;
double findis=0;  //Global Variable holding the total distance taken

int Gpsdata;
unsigned int finished =0;
unsigned int pos_cnt=0;
unsigned int lat_cnt=0;
unsigned int log_cnt=0;
unsigned int f    =0;  // initial flag equal zero
unsigned int commaa=0; //Counting the number of commas starting from zero to parse the char array
char lat[20];          //char array latitude
char lg[20];           //char array longitude

void PORTFinit(){
  volatile uint32_t delay;
	SYSCTL_RCGCGPIO_R|=0x20;
	delay=1;
	GPIO_PORTF_LOCK_R=0x4C4F434B;
	GPIO_PORTF_CR_R=0x1F;
	GPIO_PORTF_AFSEL_R=0;
	GPIO_PORTF_PCTL_R=0;
	GPIO_PORTF_AMSEL_R=0;
	GPIO_PORTF_DIR_R=0x0E;
	GPIO_PORTF_DEN_R=0x1F;
	GPIO_PORTF_PUR_R=0x11;
}
char UART_receivechar ()
{
//Check if FIFO not empty to receive data from UART .. if fifo not empty wait .. if fifo empty return character
while ((UART2_FR_R&	0x10) !=0);
return (char) UART2_DR_R;
}
//while loop kbera 3shan bn3mel parsing lel UART data elly bna5odha mn el (GPS) 3shan na5od bas el arkam elly 3ayzenha elly heya gamb el N wel E
//
void Receive_GPS_Data(){
    while(finished==0){  //while loop a5erha lma awsal l7ad a5er el satr bta3 el gps
        Gpsdata = UART_receivechar();
        f = 1;
       if( Gpsdata=='$' && pos_cnt == 0)   // finding GPRMC header
         pos_cnt=1;
       if( Gpsdata=='G' && pos_cnt == 1)
         pos_cnt=2;
       if( Gpsdata=='P' && pos_cnt == 2)
         pos_cnt=3;
       if( Gpsdata=='R' && pos_cnt == 3)
         pos_cnt=4;
       if( Gpsdata=='M' && pos_cnt == 4)
         pos_cnt=5;
       if( Gpsdata=='C' && pos_cnt==5 )
         pos_cnt=6;
       if(pos_cnt==6 &&  Gpsdata ==','){   // count commas in message
         commaa++;
         f=0; // flag eny 5las gebt GPRMC ... Hyt8yar lma ywsal l 7aga tania
       }

       if(commaa==3 && f==1){ //awl manwsal lel comma no. 3 wel flag yb2a wa7ed ---> ebtedy 7ot el 7arf fel array of characters elly howa lat w zawed el index bta3ha
        lat[lat_cnt++] =  Gpsdata;         // latitude   //
        f=0; // b3mel terminate 3shan mafish 7aga tani td5ol fel array
       }

       if(commaa==5 && f==1){ //by2aked eni wa5ed mn GPRMC mn 3nd el comma el 5amsa
         lg[log_cnt++] =  Gpsdata;         // Longitude
         f=0;
       }

       if( Gpsdata == '*' && commaa >= 5){ // hena ba2olo lma yla2i * y2afel 3ala keda w 5alihom kolhom b zero .. w b shart y3ni nkon 3adena 5 commas 3shan nb2a at3na el 7eta elly e7na 3yzenha
				 lat[lat_cnt] ='\0';             // end of GPRMC message
         lg[log_cnt]  = '\0';
         commaa = 0;                      // end of GPRMC message
         lat_cnt = 0;
         log_cnt = 0;
         f     = 0;
         finished  = 1;   // 3shan a5rog mn el while loop

      }
    }
			finished = 0;
   pos_cnt = 0;
}
void UART_Init(void) {
    SYSCTL_RCGCUART_R |= 0x0004; //enable lel clock bta3et uart2
    SYSCTL_RCGCGPIO_R |= 0x0008; //enable lel clock bta3et Port D
    UART2_CTL_R &= ~0x0001; //diable lel uart uartctl abl ma abtedi a3adel fyh
    //in uart0 use it as alternate function afsel
    //pctl of uart

    UART2_IBRD_R = 104;    // integer baud rate
    UART2_FBRD_R = 11;   // floating baud rate
    UART2_LCRH_R = 0x0070;      //uartlcrh: data lengh 8bit bit5,bit6 ==1 --one stop bit //law bit3 ==0 --> no parity bit1,bit2 ==0 --enable fifo bit4 ==1
    UART2_CTL_R |= 0x0001;         //enable uartctl

    GPIO_PORTD_AFSEL_R |= 0xC0; //dah alternative function selector mn el datasheet
    GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R & 0x00FFFFFF) + 0x11000000;// hena 3ayz asfar awel etneen bits w b3d keda a7ot fyhom wa7ayed 3shan ash8lo uart
    GPIO_PORTD_DEN_R |= 0xC0;      //digital enable
    GPIO_PORTD_AMSEL_R &= ~0XC0;       //analog

}
//initialization of uart0
void UART0_Init(void){
SYSCTL_RCGCUART_R |=0x0001;   //enable uart0 and A
SYSCTL_RCGCGPIO_R |=0x0001;
UART0_CTL_R  &=~0x0001;    //diable uart uartctl
//in uart0 use it as alternate function afsel
//pctl of uart

UART0_IBRD_R = 104;
UART0_FBRD_R = 11;//baudrate
UART0_LCRH_R = 0x0070;//uartlcrh: data lengh 8bit bit5,bit6 ==1 --one stop bit   bit3   ==0 -- no parity  bit1,bit2  ==0 --enable fifo  bit4    ==1
UART0_CTL_R = 0x0301;//enable uartctl

GPIO_PORTA_AFSEL_R  |=  0x03;
GPIO_PORTA_PCTL_R  =  (GPIO_PORTA_PCTL_R &0xFFFFFF00)+0x00000011 ;
GPIO_PORTA_DEN_R  |= 0x03;
GPIO_PORTA_AMSEL_R   &=~ 0X03;

}
//display char on a terminal
void UART_outchar(char data){ //bt recieve character mn lma el fifo yb2a fadi w ywsal 3ala uart0 3shan n3redo 3ala tera term
while((UART0_FR_R &0X0080)==0);
UART0_DR_R =data;
}
//display string on a terminal
void UART_outstring (char  *pt){ //bt recieve string mn
while(*pt){
UART_outchar(*pt);
pt++;
}
}
double convert(double x) { // hena bn8iar el format bta3 el output longit w latit L DD.mmmm l2no dah el format elly bytl3lna distance mazbota
    int y;
    x = x / 100;
    y = (int)x;
    x = (x - y) * 100;
    x = x / 60;
    x = x + y;

    return x;


}
//Function that calculates the total taken distance:

//Function to convert from degree to radian
double deg2rad(double deg){
  return (deg * PI / 180);
}

//Harvesine Formula to calculate the distance between two points
double distance(double lat1, double lon1, double lat2, double lon2){

	lat1=convert(lat1);
	lon1=convert(lon1);
	lat2=convert(lat2);
	lon2=convert(lon2);
   theta1 = deg2rad(lat1);
   theta2 = deg2rad(lat2);
  delta1 = deg2rad(lat2 - lat1);
  delta2 = deg2rad(lon2 - lon1);

  a = sin(delta1 / 2) * sin(delta1 / 2) + cos(theta1) * cos(theta2) * sin(delta2 / 2) * sin(delta2 / 2);
  c = 2 * atan2(sqrt(a), sqrt(1 - a));
 d = R * c;

  return  d;
}


//function that turns on the RED LED when the distance exceeds 100 meters.
void func1(double x){

if(x>=100)  GPIO_PORTF_DATA_R|=0x02 ;
else   GPIO_PORTF_DATA_R &=~0x02;
}
//function that turns the BLUE LED when the distance exceeds 200 meters.
void func2(double x) {

	if (x >= 200)  GPIO_PORTF_DATA_R |= 0x04;
	else   GPIO_PORTF_DATA_R &= ~0x04;


}

void PortEB_Init(){
	// Initializing Clock and wait until get stablized
	SYSCTL_RCGCGPIO_R |= 0x02;
	SYSCTL_RCGCGPIO_R |= 0x10;
		// Initializing Port B  E pins
	GPIO_PORTE_LOCK_R = GPIO_LOCK_KEY;
	GPIO_PORTE_AMSEL_R = 0;
	GPIO_PORTB_AMSEL_R = 0;
	GPIO_PORTE_DIR_R =0x07;
	GPIO_PORTB_DIR_R |= 0xFF;
	GPIO_PORTE_DEN_R |= 0x07;
	GPIO_PORTB_DEN_R |= 0xFF;
	GPIO_PORTB_AFSEL_R |= 0x00;
	GPIO_PORTE_AFSEL_R |= 0x00;
	GPIO_PORTB_PCTL_R =0;
	GPIO_PORTE_PCTL_R =0;

}
double dist;

int main (){
	    PORTFinit();
		UART_Init();
		PortEB_Init();
		LCD_init();

		UART0_Init();
		Receive_GPS_Data();

		LCD_clearScreen();
		LCD_displayString("  Welcome");
		_delay_ms(1000);
		lati1	= strtod(lat,NULL); //awel makan;
		long1=	strtod(lg,NULL) ;
		lat1= lati1;
		lon1= long1;

  	while(1){
		 Receive_GPS_Data();
				_delay_ms(250);
	/*	UART_outstring("Latitiude ");
		UART_outstring(lat);
		UART_outstring("\n");
		UART_outstring("Longitude ");
		UART_outstring(lg);
		UART_outstring("\n");
		UART_outstring("\n");
		*/ //commented because this was for the verification on tera term only
	  lat2 = strtod(lat,NULL);
		lon2=	strtod(lg,NULL);
		dist = distance( lat1,  lon1,  lat2,  lon2);

		if(dist<0.25){ continue ;}
		else{

		findis+= dist ;
		sprintf(distancat, "%3f", (float)findis);
	//a=	(int)findis;
	//distancat=(""+a).toCharArray();
		LCD_clearScreen();
		LCD_displayString("  Distance ");
		LCD_displayString(distancat);
		func1(findis);
		func2(findis); //Blinking blue+red=pink
		lat1=lat2;
		lon1=lon2;
		}
		if((GPIO_PORTF_DATA_R&0x11)==0x10){
LCD_clearScreen();
LCD_displayString("  ");
LCD_displayString(lat);
LCD_displayString("N");
	_delay_ms(1000);
LCD_clearScreen();
LCD_displayString("  ");
LCD_displayString(lg);
LCD_displayString("  E");
	_delay_ms(1000);
LCD_clearScreen();

}
//if((GPIO_PORTF_DATA_R&0x11)==0x01){
//LCD_displayString("  Enjoy GPS");		_delay_ms(1000);
//LCD_clearScreen();
//}

  	}

}
