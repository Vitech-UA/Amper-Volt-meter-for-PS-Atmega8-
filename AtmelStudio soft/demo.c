#include <io.h>
#include <delay.h>
#include <alcd.h>
#include <stdlib.h>
#include <mega8.h>



float current = 0.00;
float voltage = 0.00;

int adc_buffer_curr = 0;
int adc_buffer_volt = 0;



char lcd_buffer_current[20];
char lcd_buffer_voltage[20];


// Declare your global variables here


#define ADC_VREF_TYPE ((0<<REFS1) | (0<<REFS0) | (1<<ADLAR))

void init_Timer()
{
	// Timer/Counter 1 initialization
	// Clock source: System Clock
	// Clock value: 7,813 kHz
	// Mode: Normal top=0xFFFF
	// OC1A output: Disconnected
	// OC1B output: Disconnected
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer Period: 8,3886 s
	// Timer1 Overflow Interrupt: Off
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: On
	// Compare B Match Interrupt: Off
	TCCR1A = (0 << COM1A1) | (0 << COM1A0) | (0 << COM1B1) | (0 << COM1B0) | (0 << WGM11) | (0 << WGM10);
	TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (0 << WGM12) | (1 << CS12) | (0 << CS11) | (1 << CS10);
	TCNT1H = 0x00;
	TCNT1L = 0x00;
	ICR1H = 0x00;
	ICR1L = 0x00;
	OCR1AH = 0x00;
	OCR1AL = 0xD0;
	OCR1BH = 0x00;
	OCR1BL = 0x00;


}

interrupt [TIM1_COMPA] void timer1_compa_isr(void)
{
	//lcd_clear();
	ftoa(voltage, 1, lcd_buffer_voltage);
	lcd_gotoxy(0, 0);
	lcd_puts("U= ");
	lcd_gotoxy(3, 0);
	lcd_puts(lcd_buffer_voltage);


	ftoa(current, 2, lcd_buffer_current);
	lcd_gotoxy(0, 1);
	lcd_puts("I=");
	lcd_gotoxy(3, 1);
	lcd_puts(lcd_buffer_current);




	current = 0.00;
	voltage = 0.00;

	TCNT1H = 0;
	TCNT1L = 0;
}

void init_ADC()
{

	ADMUX = ADC_VREF_TYPE;
	ADCSRA = (1 << ADEN) | (0 << ADSC) | (0 << ADFR) | (0 << ADIF) | (0 << ADIE) | (0 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	SFIOR = (0 << ACME);


}
void init_port()
{
	DDRB = (1 << DDB7) | (1 << DDB6) | (1 << DDB5) | (1 << DDB4) | (1 << DDB3) | (1 << DDB2) | (1 << DDB1) | (1 << DDB0);
	PORTB = (0 << PORTB7) | (0 << PORTB6) | (0 << PORTB5) | (0 << PORTB4) | (0 << PORTB3) | (0 << PORTB2) | (0 << PORTB1) | (0 << PORTB0);

	DDRC = (1 << DDC6) | (1 << DDC5) | (1 << DDC4) | (1 << DDC3) | (1 << DDC2) | (0 << DDC1) | (0 << DDC0);
	PORTC = (0 << PORTC6) | (0 << PORTC5) | (0 << PORTC4) | (0 << PORTC3) | (0 << PORTC2) | (0 << PORTC1) | (0 << PORTC0);

	DDRD = (1 << DDD7) | (1 << DDD6) | (1 << DDD5) | (1 << DDD4) | (1 << DDD3) | (1 << DDD2) | (1 << DDD1) | (1 << DDD0);
	PORTD = (0 << PORTD7) | (0 << PORTD6) | (0 << PORTD5) | (0 << PORTD4) | (0 << PORTD3) | (0 << PORTD2) | (0 << PORTD1) | (0 << PORTD0);
}

unsigned int read_adc(unsigned char adc_input)
{
	ADMUX = adc_input | ADC_VREF_TYPE;
	// Delay needed for the stabilization of the ADC input voltage
	delay_us(10);
	// Start the AD conversion
	ADCSRA |= (1 << ADSC);
	// Wait for the AD conversion to complete
	while ((ADCSRA & (1 << ADIF)) == 0);
	ADCSRA |= (1 << ADIF);
	return ADCH;
}

void main(void)
{
	TIMSK = (0 << OCIE2) | (0 << TOIE2) | (0 << TICIE1) | (1 << OCIE1A) | (0 << OCIE1B) | (0 << TOIE1) | (0 << TOIE0);
	MCUCR = (0 << ISC11) | (0 << ISC10) | (0 << ISC01) | (0 << ISC00);
	UCSRB = (0 << RXCIE) | (0 << TXCIE) | (0 << UDRIE) | (0 << RXEN) | (0 << TXEN) | (0 << UCSZ2) | (0 << RXB8) | (0 << TXB8);

	init_ADC();
	init_port();
	init_Timer();
	lcd_init(12);

#asm("sei");

	while (1)
		{

		adc_buffer_curr = read_adc(1);

		adc_buffer_volt = read_adc(0);


		voltage = (4.97 * 5.00 * adc_buffer_volt) / 256.00;
		current = (4.97 * adc_buffer_curr) / 256.00;





		}
}
