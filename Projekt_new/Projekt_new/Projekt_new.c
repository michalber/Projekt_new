/*
 * Created: 05.06.2018 09:56:59
 * Author : MB JB
 */ 
//------------------------------------------------------------------
#include <avr/io.h>
#include <util/delay.h>
#include "lcd.h"
//------------------------------------------------------------------
//-- Definicje LCD --
#define F_CPU 1000000
#define LCD_RS_PORT PORTC
#define LCD_RW_PORT PORTC
#define LCD_DATA0_PORT PORTC
#define LCD_DATA1_PORT PORTC
#define LCD_DATA2_PORT PORTC
#define LCD_DATA3_PORT PORTC
#define LCD_E_PORT PORTC

#define LCD_DATA0_PIN 0
#define LCD_DATA1_PIN 1
#define LCD_DATA2_PIN 2
#define LCD_DATA3_PIN 3

#define LCD_RS_PIN 4
#define LCD_E_PIN 5
#define LCD_RW_PIN 6
//-------------------

//-- Definicje przetwornika ADC --
#define ADCIN PB3  //definicja ADCIN (wejœcie analogowe PortB03)  
//--------------------------------
#define Vs 5
#define LTHRES 500
#define RTHRES 500
//-------------------------------------------------------------------
// Inicjalizacja przetwornika.
void adc_init()
{
	//AREF = AVcc
	ADMUX = (1<<REFS0);
	
	// ADC w³¹czony,a prescaler ustawiony na 128
	// 16000000/128 = 125000
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}
//------------------------------------------------------------------
// Czytanie wartoœci  przesy³anej na ADC
uint16_t adc_read(uint8_t ch)
{
	// Wybieranie kana³u przetwornika
	// Operacja AND z '7' zawsze da nam wartoœæ odcyztywan¹ przez przetwornik
	// ch pomiêdzy 0 i 7
	//ch &= 0b00000111;  // AND z 7
	ADMUX = (ADMUX)|ch;     // Wyczyszczenie trzech dolnych bitów przed operacj¹ OR
	
	// Pocz¹tek pojedyñczej konwersji
	// Zapisanie jedynki na ADSC
	ADCSRA |= (1<<ADSC);
	
	//Czekanie a¿ konwersja siê zakoñczy
	// ADC znowu zeruje siê
	// Kontynuacja pêtli, a¿ do zakoñczenia
	while(ADCSRA & (1<<ADSC));
	
	return (ADC);
}

//------------------------------------------------------------------
int main()
{
	uint16_t adc_result0;
	char int_buffer[10];
	
	// initialize adc and lcd
	adc_init();
	lcd_init(LCD_DISP_ON_CURSOR);
	lcd_clrscr();
	lcd_home();
	
	// display the labels on LCD
	lcd_puts("Odczyt = ");
	_delay_ms(50);
	
	while(1)
	{
		adc_result0 = adc_read(0);      // odczytaj wartosc z ADC
		
		// wyœwietl na lcd
		itoa(adc_result0, int_buffer, 10);
		adc_result0=(adc_result0+0.095*Vs)/0.009*Vs;
		lcd_gotoxy(9,0);
		lcd_puts(int_buffer);
		
		_delay_ms(1000);
	}
}
//------------------------------------------------------------------
/*

int main(void)
{
	int wynik=0;
	
	ADCSRA = (1<<ADEN) //ADEN=1 w³¹czenie przetwornika ADC)
	|(1<<ADPS0) // ustawienie preskalera na 128
	|(1<<ADPS1)
	|(1<<ADPS2)
	
	//ADMUX  =  (1<<REFS1) | (1<<REFS0) //REFS1:0: Reference Selection Bits
	//Internal 2.56V Voltage Reference with external capacitor at AREF pin
	//|(1<<MUX2) | (1<<MUX0); //Input Channel Selections (ADC5 - Pin 5 )
	
	DDRB &=~ (1<<ADCIN);        //Ustawienie Wejœcia ADC

    while (1) 
    {
		ADCSRA |= (1<<ADSC); //ADSC: uruchomienie pojedynczej konwersji 
		
		 while(ADCSRA & (1<<ADSC)); //czeka na zakoñczenie konwersji  
		 
		 //konwertuj na ciœnienie: Vout=Vs*(0.009*P-0.095)
		 // oblicz P: (Vout+0.095Vs)/0.009Vs, Vs=5;
		 		 
		 //wynik=konwersja;
		 // wyswietl na LCD (biblioteka)
		 //LCD <- wynik;
    }
}

*/