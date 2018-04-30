//#define VEML6075_ADDR 0x20
//#include <stdlib.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
//#include <avr/wdt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
//#include <stdint.h>
//#include <stdbool.h>
#include <util/twi.h>

#define bat_marking_start 0
#define time_marking_start 20
#define index_marking_start 80
 
#include "u8g.h"
u8g_t u8g;

#define OVERSAMPLING 4
uint8_t error=0;
#include "VEML6075.c"
VEML6075 veml6075 = VEML6075();


const uint8_t PALE_INDEX_NUMBER[5]={3,5,7,254};
const char *PALE_INDEX_NAME[]={"LOW","MEDIUM","HIGH","EXTREME"};

// reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
	int i=0, j=len-1, temp;
	while (i<j)
	{
		temp = str[i];
		str[i] = str[j];
		str[j] = temp;
		i++; j--;
	}
}

void delay_ms(int ms){
	for (int i=0; i < ms; i++){
		_delay_ms(1);
	}
}

void delay_us(int us){
	for (int i=0; i < us; i++){
		_delay_us(1);
	}
}

// Converts a given integer x to string str[].  d is the number
// of digits required in output. If d is more than the number
// of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
	int i = 0;
	while (x)
	{
		str[i++] = (x%10) + '0';
		x = x/10;
	}
	
	// If number of digits required is more, then
	// add 0s at the beginning
	while (i < d)
	str[i++] = '0';
	reverse(str, i);	
	str[i] = '\0';
	return i;
}
// Converts a floating point number to string.
void ftoa(double n, char *res, int afterpoint)
{
	// Extract integer part
	int ipart = (int)n;
	
	// Extract floating part
	double fpart =(double) (n - (double)ipart);
	
	// convert integer part to string

	//int i = intToStr(ipart, res, 0);

	int i = intToStr(ipart, res, 1); //forces to show 0 at the beginning

	// check for display option after point
	if (afterpoint != 0)
	{
		res[i] = '.';  // add dot
		
		// Get the value of fraction part upto given no.
		// of points after dot. The third parameter is needed
		// to handle cases like 233.007
		fpart = fpart * pow(10, afterpoint);
		
		intToStr((int)fpart, res + i + 1, afterpoint);
	}
}	


void LCD_clear(void)
{
	
	u8g_SetDefaultBackgroundColor(&u8g);
	u8g_DrawBox(&u8g,0,0,132,64); //clears the screen
	u8g_SetDefaultForegroundColor(&u8g);
	u8g_SetFont(&u8g, u8g_font_6x10);
	//u8g_DrawBox(&u8g, 5,10,20,10);
	
}


void battery_full_marking(void)
{
	u8g_SetFont(&u8g, u8g_font_7x13);
	u8g_DrawBox(&u8g,bat_marking_start+2,55,12,7);      //up to 93 on the right
	u8g_DrawBox(&u8g,bat_marking_start,57,2,3);
}

void battery_half_marking(void)
{
	u8g_SetFont(&u8g, u8g_font_7x13);
	u8g_DrawFrame(&u8g,bat_marking_start+2,55,12,7);      //up to 93 on the right
	
	u8g_DrawBox(&u8g,bat_marking_start+2,55,6,7);
	u8g_DrawBox(&u8g,bat_marking_start,57,2,3); 
	
}


void battery_low_marking(void)
{
	u8g_SetFont(&u8g, u8g_font_7x13);
	//u8g_DrawFrame(&u8g,68,5,12,7);      //up to 93 on the right
	//u8g_DrawBox(&u8g,66,7,2,3);
	
	u8g_DrawFrame(&u8g,bat_marking_start+2,55,12,7);      //up to 93 on the right
	u8g_DrawBox(&u8g,bat_marking_start,57,2,3);
}



/************************************************************************************************************************************************/
/* Global Objects                                                       																		*/
/************************************************************************************************************************************************/
				   
volatile int16_t   testing_min=0,
				   testing=0;	
				      
				  				   
volatile uint16_t  timer_1s=0,		//these are test counters
				   timer_1min=0,
				   timer_1h=0,
				   UV_lvl=0,
				   battery_voltage_lvl= 0,
				   division=0,
				   timer_seconds=0,	//these house the real time values from when the device was started
				   timer_minutes=0,
				   timer_hours=0;
				   
uint8_t				index=0;

float    		    bat=0.0;

double				UVA=0,
					UVB=0,
					UVgah=0;
					
				   
double				UV=0.0,
					UVtot=0.0,
					UV1=0.0,
					UV_total=0.0,
					UV_t=0.0;				   
	
bool			   cmd_pulse=0,	  
				   batterylevelcheck=0,
				   timernow=0,
				   found=0;
				   
				   
				 
void u8g_setup(void)
{
	
	// SCK: PORTB, Bit 5 --> PN(1,5)
	// MOSI: PORTB, Bit 3 --> PN(1,3)
	// CS: not used
	// A0: PB0 --> PN(1,0) //DC!!
	// RST: PB1,  --> PN(1,1) 
	//    Arguments for u8g_InitHWSPI are: CS, A0, Reset
		
	u8g_InitHWSPI(&u8g, &u8g_dev_ssd1306_128x64_hw_spi,U8G_PIN_NONE,PN(1,0), PN(1,1));
	//u8g_InitHWSPI(&u8g, &u8g_dev_sh1106_128x64_hw_spi,U8G_PIN_NONE,PN(2,1), PN(2,2));
	u8g_SetRot180(&u8g); //flip screen, if required
	//u8g_SetColorIndex(&u8g, 1); 	
}

void system_setup() {
	
//pc4=sda(in),	pc5=scl(in)  
DDRC=0b00000000;
PORTC=0b00100000;//pullups high on SCL I2C 0b00100000;

DDRD = 0b00000000;
PORTD = 0b00000000; //output 0 and input tri-state

//pb3=MOSI(out), pb5=sck(out),pb1=res(out), pb0=dc(out)
DDRB = 0b00101011;
PORTB = 0b00000000; //output 0 and input tri-state


/* reminder
adc6=battery vref40
2.6v=>0.5v
*/

//ADMUX adlar=0, mux 0111 UV-feedback, mux 0110 battery voltage feedback
ADMUX=0b00000110;
ADCSRA=0b11000111; //with 128 clk division from 8MHz now ok

//timer control 8 bit
//output is 8MHz/(256*250)=8ms tick
TCNT0 = 0x00; //set timer to 0
OCR0A = 249; //runs to value 0:249, to get exact ms
TCCR0A = 0b00000010; //CTC operation mode
TCCR0B = 0b10000100; // force compare match A, prescaler to 256
TIMSK0= 0b00000010; //masked compare match A 

}


int main(void){

system_setup();
u8g_setup();

//cli();  // disable interrupts
sei();	//enable interrupts

//I2C setup
//stantard mode <=100khz
//8mhz/(16+2*TWBR*prescaler), prescaler to 1 so prescaler atleast 32
TWBR = 32;
TWSR=0x00; //prescaler to 1
TWAR=0x00;
TWCR=(1<<TWIE)|(1<<TWEN); //interrupts and enabled
//end of I2C setup

veml6075.begin();


//temp chars for printing


char UV_A[10]={0,0,0,0,0,0,0,0,0,0};
char UV_B[10]={0,0,0,0,0,0,0,0,0,0};
char UV_I[10]={0,0,0,0,0,0,0,0,0,0};		
char UV_calc_total[10]={0,0,0,0,0,0,0,0,0,0};
char battery[10]={0,0,0,0,0,0,0,0,0,0};
char secs[10]={0,0,0,0,0,0,0,0,0,0};
char mins[10]={0,0,0,0,0,0,0,0,0,0};
char hours[10]={0,0,0,0,0,0,0,0,0,0};
	
char UV_index[10]={0,0,0,0,0,0,0,0,0,0};	

char UV_test[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //test
//char pulse_test[20]; //test
//char collisions[20]; //test

		//	battery voltage level check at start
		ADCSRA |= (1 << ADSC);
		while(ADCSRA & _BV(ADSC));    // wait until conversion is complete
		battery_voltage_lvl=0;
		for(int i=0;i < OVERSAMPLING;i++ ){
			ADCSRA |= (1<<ADSC);
			while(ADCSRA & _BV(ADSC));    // wait until conversion is complete
			battery_voltage_lvl+=ADC;		
		}
		battery_voltage_lvl=round(battery_voltage_lvl/OVERSAMPLING);
		batterylevelcheck = 1;	

	//main loop
	testing_min=60;
	
	

	 while(1)
	 {
	 
		
		if(cmd_pulse==1)	//triggered in interrupt
		{
	
				veml6075.poll();
				UVA = veml6075.getUVA();
				UVB = veml6075.getUVB();
				UVgah =(double) ((UVA*0.001100110011)+(UVB*0.00125))/2;
				
				//ftoa(UVA, UV_now, 2); //convert float to string
				UVtot=(double)UVtot+UV; //calculating the dose
				
			testing_min++;	
			if (testing_min>=60) //once every minute
			{
			
				UV_total=(double)UV_total+UVtot;
				UV_t=(double)UV_total/60;
						
				testing_min=0;
				ftoa(UV_t,UV_calc_total , 3);
					
			}
			if(batterylevelcheck)
			{
		
				bat=(float)0.0164946*battery_voltage_lvl; //battery voltage level if resistors have low % tolerances
				//10bit adc value is multiplied by 3.23v reference and divided by 0.2=> 3.23*5.25=17 and finally divided by 2^10
				
				ftoa(bat, battery, 2); //convert float to string
				batterylevelcheck=0;	
			}
					
			intToStr(timer_seconds, secs, 2); //seconds
			intToStr(timer_minutes, mins, 2); //mins
			intToStr(timer_hours, hours, 2); //hours

		cmd_pulse=0;
		}
		
		ftoa(UVA, UV_A, 1);
		ftoa(UVB, UV_B, 1);
		ftoa(UVgah, UV_I, 4);
		intToStr(error, UV_test, 3); 
	 
		
		//printing to display
		u8g_FirstPage(&u8g);
		do
		{
			u8g_SetFont(&u8g, u8g_font_7x13);
			u8g_DrawStr(&u8g, time_marking_start, 63, hours);
			u8g_DrawStr(&u8g, time_marking_start+14, 63, ":");
			u8g_DrawStr(&u8g, time_marking_start+21, 63, mins);
			u8g_DrawStr(&u8g, time_marking_start+35, 63, ":");
			u8g_DrawStr(&u8g, time_marking_start+42,63, secs);
			if (bat >= 2.3){			
				battery_full_marking();
			}
			if (bat < 2.3 && bat >= 2.1){			
				battery_half_marking();
			}
			if (bat < 2.1){			
				battery_low_marking();
			}
			//u8g_DrawStr(&u8g, 84, 63, battery);
			//u8g_DrawStr(&u8g, 120, 63, "V");
			
			for(uint8_t i=0;i<sizeof(PALE_INDEX_NUMBER);i++){
				if(UVgah < PALE_INDEX_NUMBER[i]){
					index=i;
					break;
				}		
			}
			//if(index >0){
			//	index--;
			//}
			
			
			intToStr(index,  UV_index, 3);
			
			//u8g_DrawStr(&u8g, index_marking_start, 63, UV_index);
			u8g_DrawStr(&u8g, index_marking_start, 63, PALE_INDEX_NAME[index]);
						
			
			u8g_SetFont(&u8g, u8g_font_fur17);
						
			//display the current UV index
			
			u8g_DrawStr(&u8g, 10, 19, "UV index:");
			
			if(UVgah<10){
				u8g_DrawStr(&u8g, 25, 44, UV_I);
			}
			if(UVgah >=10 ){
				u8g_DrawStr(&u8g, 12, 44, UV_I);
				
			}
			
			u8g_SetFont(&u8g, u8g_font_7x13);
			//u8g_DrawStr(&u8g, 2, 26, "UVA:");
			//u8g_DrawStr(&u8g, 38, 26, UV_A);			
			//u8g_DrawStr(&u8g, 2, 39, "UVB:");
			//u8g_DrawStr(&u8g, 38, 39, UV_B);
			//u8g_DrawStr(&u8g, 2, 29, "error:");
			//u8g_DrawStr(&u8g, 55, 43, UV_test );
			//u8g_DrawStr(&u8g, 2, 42, UV_test); //test
			
			//fix this
			/*
			if(UV_total > 1000)
			{
				UV1=(double)(UV_total/1000);
				ftoa(UV1, UV_calc_total, 2); //convert float to string
				u8g_DrawStr(&u8g, 84, 50, UV_calc_total);
				u8g_DrawStr(&u8g, 100, 50, "W");
			}
			*/
	
		} while ( u8g_NextPage(&u8g) );
		u8g_Delay(200);
		}
		
}


// interrupts with timer0 idea is to check battery voltages
// and also get the time
ISR (TIMER0_COMPA_vect)  //interrupts every 8ms
{

	
	timer_1s++;
	if(timer_1s >= 125)		//625*1.6ms=1s
	{
		cmd_pulse=1; //1 second has passed time to refresh display
		timer_seconds++; //increase if under 59 sec
		
		timer_1s-=125;
		if(timer_seconds > 59)
		{
			timer_seconds-=60; //zero if
			timer_1min++;
			timer_minutes++; //increase if under 59 min
			if(timer_minutes > 59)
			{
				timer_minutes-=60; //zero if
				timer_hours++;
				if(timer_hours > 98) // no more than 2 digits for hours in display
				{
					timer_hours=0; //zero the timer
				}
				
				//1hour reached
				timer_1h=0;
				
			}
			
			battery_voltage_lvl=0;
			for(int i=0;i < OVERSAMPLING;i++ ){
				ADCSRA |= (1<<ADSC);
				while(ADCSRA & _BV(ADSC));    // wait until conversion is complete
				battery_voltage_lvl+=ADC;
			}
			battery_voltage_lvl=round(battery_voltage_lvl/OVERSAMPLING);
			batterylevelcheck = 1;
			timer_1min=0;				  // set minute timer to 0

			
			
			timer_1h++;
		}
	}


	/*
	//zero the timer if 1 second has been reached
	if(timer_1s >= 125)		//125*8ms=1s
	{
		timer_seconds++; //increase if under 59 sec	
		if(timer_seconds > 59)
		{
		timer_seconds=0; //zero if 
		}
		timer_1s=0;
	
		cmd_pulse=1; //1 second has passed time to refresh display	
	}
	if(timer_1min >= 7500)
	{
		timer_minutes++; //increase if under 59 min
		if(timer_minutes > 59)
		{
			timer_minutes=0; //zero if 
		}
		
		battery_voltage_lvl=0;
		for(int i=0;i < OVERSAMPLING;i++ ){
			ADCSRA |= (1<<ADSC);
			while(ADCSRA & _BV(ADSC));    // wait until conversion is complete
			battery_voltage_lvl+=ADC;		
		}
		battery_voltage_lvl=round(battery_voltage_lvl/OVERSAMPLING);
		batterylevelcheck = 1;		
		timer_1min=0;				  // set minute timer to 0
		
	}
	timer_1min++;
	
	if(timer_1h >= 450000) 
	{	
		timer_hours++;
		if(timer_hours > 98) // no more than 2 digits for hours in display
		{
			timer_hours=0; //zero the timer
		}
		
		//1hour reached
		timer_1h=0;
	}
	timer_1h++;
	*/

}


