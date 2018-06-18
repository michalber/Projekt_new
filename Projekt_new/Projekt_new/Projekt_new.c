/*
 * Projekt programu do pomiaru ci�nienia wraz z obs�ug� przerwania
   po kt�rym wy�wietlana jest najwi�ksza zmierzona warto��
 *
 * Created: 05.06.2018 09:56:59
 * Author : MB & JB
*/
//------------------------------------------------------------------
//-- Biblioteki -- 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.h"
//------------------------------------------------------------------
//-- Definicje przetwornika ADC --
#define ADCIN PORTB3  //definicja ADCIN (wej�cie analogowe PortB03)  
#define F_CPU 1000000
#define Vs 5.1
//-------------------------------------------------------------------
//-- Zmienne u�ywane w programie -- 
uint16_t adc_result0;
uint16_t max_result;
char int_buffer[10];
//-------------------------------------------------------------------
// Inicjalizacja przetwornika.
void adc_init()
{
	//AREF = AVcc
	ADMUX = (1<<REFS1) |(1<<REFS0)|(1<<MUX2) | (1<<MUX0);	
	// ADC w��czony,a prescaler ustawiony na 128
	// 16000000/128 = 125000
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	//|(1<<MUX2) | (1<<MUX0)
}
//------------------------------------------------------------------
// Czytanie warto�ci  przesy�anej na ADC
uint16_t adc_read(uint8_t ch)
{
	// Wybieranie kana�u przetwornika
	// Operacja AND z '7' zawsze da nam warto�� odcyztywan� przez przetwornik
	// ch pomi�dzy 0 i 7
	ch &= 0b00000111;  // AND z 7
	ADMUX = (ADMUX)|ch;    // Wyczyszczenie trzech dolnych bit�w przed operacj� OR
	//ADMUX = (ADMUX & 0xF8)|ch;
	// Pocz�tek pojedy�czej konwersji
	// Zapisanie jedynki na ADSC
	ADCSRA |= (1<<ADSC);
	
	//Czekanie a� konwersja si� zako�czy
	// ADC znowu zeruje si�
	// Kontynuacja p�tli, a� do zako�czenia
	while(!(ADCSRA & (1<<ADIF)));
	ADCSRA|=(1<<ADIF);
	return (ADC);
}

//------------------------------------------------------------------
//-- Inicjalizacja przerwania -- 
void break_init() {
	 cli();    // na wszelki wypadek blokujemy przyjmowanie przerwa�
	 //MCUCR |= (MCUCR & 0b1111100) | 0b10; // przerwanie przy zmianie INT0 1->0
	 MCUCR |= (1<<ISC01)|(1<<ISC00);
	 GIFR &= ~(1<<INTF0); / // Uaktywniamy przerwanie INT0
	 GICR |= (1<<INT0);
	 sei();   // odblokowujemy przyjmowanie przerwa�
}
// Obs�uga przerwania INT0 --
ISR(INT0_vect)
{
	lcd_clear();
	lcd_puts("Maks. wartosc");
	itoa(max_result, int_buffer, 10);
	lcd_set_cursor(0,1);
	lcd_puts(int_buffer);
	lcd_set_cursor(4,1);
	lcd_puts("hPa");
	
	_delay_ms(2000);
}
//------------------------------------------------------------------
int main()
{	
	max_result = 0;
	// inicjalizuj adc, lcd i przerwania
	break_init(); 
	adc_init();
	lcd_init();
	lcd_on();
	
	_delay_ms(50);
	
	while(1)
	{
		lcd_clear();
		lcd_puts("Odczyt = ");
		adc_result0 = adc_read(0);      // odczytaj wartosc z ADC
		
		// wy�wietl na lcd
		adc_result0=((adc_result0+0.085*Vs)/0.0412*Vs)/100;
		if(adc_result0 > max_result) max_result=adc_result0;
		itoa(adc_result0, int_buffer, 10);
		lcd_set_cursor(9,0);
		lcd_puts(int_buffer);
		lcd_set_cursor(13,0);
		lcd_puts("hPa");
		
		_delay_ms(1000);
	}
}