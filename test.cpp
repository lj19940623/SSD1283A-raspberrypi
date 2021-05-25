
#include "SSD1283A.h"
#include <wiringPi.h> 


#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
int main ()
{
	SSD1283A_GUI mylcd(/*CS=D8*/ 8, /*DC=D3*/ 27, /*RST=D4*/ 22, /*LED=D2*/ 17); //hardware spi,cs,cd,reset,led
	mylcd.init();
	mylcd.Fill_Rect(0,0,130,130,BLACK);
	delay(2000);
	mylcd.Fill_Rect(0,0,130,130,BLUE);
	delay(2000);
	mylcd.Fill_Rect(0,0,130,130,RED);
	delay(2000);
	mylcd.Fill_Rect(0,0,130,130,GREEN);
	delay(2000);
	mylcd.Fill_Rect(0,0,130,130,CYAN);
	delay(2000);
	mylcd.Fill_Rect(0,0,130,130,MAGENTA);
	delay(2000);
	mylcd.Fill_Rect(0,0,130,130,YELLOW);
	delay(2000);
	mylcd.Fill_Rect(0,0,130,130,WHITE);
	delay(2000);
	return 0;
}
