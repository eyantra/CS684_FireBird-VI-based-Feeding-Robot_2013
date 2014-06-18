#define F_CPU 8000000
#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>



#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1) 
#define kPinInitCompleteLED         PA0
#define kDelay                      1000
#define kReverseDelay               3000
#define kLowerLimit                 200                 //  Min 225 == 0.9 ms
#define kUpperLimit                 1000                 //  Max 525 == 2.1 ms

#define motorADir1         PD6
#define motorADir2         PD2
#define motorBDir1         PD7
#define motorBDir2         PD3
#define motorACtr          1
#define motorBCtr          2


void USART_SendByte(unsigned char data)
{
  
  while((UCSRA & (1<<UDRE)) == 0)
  { }
  
   // Transmit data
  
  UDR = data;
  
}
  

void USART_SendString(unsigned char *str)
{
  
  unsigned char *p;
  p = str;
  while (*p != '\0')
  {
	USART_SendByte(*p);
	p++;
  }
}

void USART_Init()
{
    sei();
	//int budPrescale; 
	//budPrescale  = ((F_CPU / (budRate * 16UL))) - 1; 

	UBRRH = (uint8_t)(BAUD_PRESCALE>>8);
	UBRRL = (uint8_t)BAUD_PRESCALE;

	UCSRB = (1 << RXEN) | (1 << TXEN);  
	//UCSRC = (1 << UCSZ1) | (1 << UCSZ0);  
	UCSRC = (1 << URSEL)|(1<<USBS)|(1<<UCSZ0)|(1<<UCSZ1); //
	//UCSRC = (0<<USBS)|(1<<UCSZ1)|(1<<UCSZ0);
// 	TCCR1B |= (1 << CS10); // Set up timer 
}


void PWM_init(){

    //  Set up OCR pins (PD4, PD5) as outputs (00110000)…
    
    DDRD = _BV(PD4) | _BV(PD5);
    
    //  Set up Timer 1. Timer 1 should reset when
    //  it reaches TOP = ICR1 (WGM13:0 = 1110b). On
    //  compare match clear output, at TOP set (COM1A1:0 = 10b).
    
    TCCR1A = _BV(COM1A1) | !_BV(COM1A0)                 //  Both PWM outputs set at TOP,
                | _BV(COM1B1) | !_BV(COM1B0)            //    clear on compare match
                | !_BV(FOC1A) | !_BV(FOC1B)             //  PWM mode, can't force output
                | _BV(WGM11) | !_BV(WGM10);             //  Fast PWM, TOP = ICR1
    

    TCCR1B = !_BV(ICNC1) | !_BV(ICES1)                  //  Disable input capture noise canceler,
                                                        //    edge select to negative.
                | _BV(WGM13) | _BV(WGM12)               //  Fast PWM, TOP = ICR1
                | !_BV(CS12) | _BV(CS10);   //  clk(i/o) / 1024

    
    //  PWM duty cycle…
//	TIMSK != (1 << TOIE1);


    OCR1A = 0;
    OCR1B = 0;
    
    //  PWM period…
    
    ICR1 = 65535;
  
}

unsigned char receiveByte( void )
{
	unsigned char data, status;
	
	while(!(UCSRA & (1<<RXC))); 	// Wait for incomming data
	
	status = UCSRA;
	data = UDR;
	
	return(data);
}

static char byteMap[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };
static int byteMapLen = sizeof(byteMap);

/* Utility function to convert nibbles (4 bit values) into a hex character representation */
char nibbleToChar(uint8_t nibble)
{
	if(nibble < byteMapLen) return byteMap[nibble];
	return '*';
}

void USART_SendInt(int16_t n)
{
	int16_t i = n;
	unsigned char c0,c1,c2,c3;
		
	c0 = nibbleToChar(i & 0x0f);
	i>>= 4;
	c1 = nibbleToChar(i & 0x0f);
	i>>= 4;
	c2 = nibbleToChar(i & 0x0f);
	i>>= 4;
	c3 = nibbleToChar(i & 0x0f);
	USART_SendByte(c3);
	USART_SendByte(c2);
	USART_SendByte(c1);
	USART_SendByte(c0);
}

void adc_init()
{
    // AREF = AVcc
    ADMUX = (1<<REFS0);

    // ADC Enable and prescaler of 128
    // 16000000/128 = 125000
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}


uint16_t adc_read(uint8_t ch)
{
  // select the corresponding channel 0~7
  // ANDing with ’7? will always keep the value
  // of ‘ch’ between 0 and 7
  ch &= 0b00000111;  // AND operation with 7
  ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing

  // start single convertion
  // write ’1? to ADSC
  ADCSRA |= (1<<ADSC);

  // wait for conversion to complete
  // ADSC becomes ’0? again
  // till then, run loop continuously
  while(ADCSRA & (1<<ADSC));
  _delay_ms(1);
  return (ADC);
}


int breakCount(int sensor){

	int ctr =0,ctr2;
    for (int i =0; i < 10 ; i++){
      int sensorValue = adc_read(sensor);
//	  USART_SendByte('+');	
      if  (sensorValue > 100){
	  	ctr2 =0;
        while (sensorValue > 100 && ctr2 <10){
			ctr2 ++;
			sensorValue = adc_read(sensor);
//			USART_SendByte('^');
		}
        if (sensorValue < 100)
			ctr ++;
//		USART_SendByte('.');
      }
//	  _delay_ms(1);	
    }

	return ctr;
}

int main()
{

	unsigned char c,d,e;
	int breakcount;
    
    DDRB |= _BV(kPinInitCompleteLED);
	DDRD |= _BV(motorADir1) | _BV(motorADir2) | _BV(motorBDir1) | _BV(motorBDir2);


	USART_Init();
 	PWM_init();
	adc_init();
	
	   //  Show initialization complete…
    
    PORTB = _BV(kPinInitCompleteLED);
    
    //  Loop forever steering left-to-right-to-left…
    
    while (1)
    {

	
		c = receiveByte(); //'N'
		USART_SendByte(c);
		if (c =='N'){
			c = receiveByte(); // motor no + direction 1,2,3,4
			breakcount = 0;
//			USART_SendByte('A');
			d = receiveByte();	// Duty cycle
//			USART_SendByte('B');
			e = receiveByte();  // duration
//			USART_SendByte('C');
//			USART_SendInt(c);
			if(c == 1){
				breakcount =0;
//				USART_SendByte('D');
				PORTD |= _BV(motorADir1);
				OCR1B = d * 256;
				for (int k =0 ; k < 20; k++){
					breakcount += breakCount(motorACtr);
//  				    USART_SendInt(breakcount);

					//USART_SendByte('E');	
					if (breakcount >= e)
						break;
				}
				// if could not receive required count for too long the motor will stop
				PORTD &= ~_BV(motorADir1);
				OCR1A = 0;
//				USART_SendByte('1');

				if (breakcount >= e){
					USART_SendByte('S');				
				}
				else{
					USART_SendByte('F');
				}

			}
			if(c == 2){
//				USART_SendByte('G');
				PORTD |= _BV(motorADir2);
				OCR1B = d * 256;
				for (int k =0 ; k < 20; k++){
					breakcount += breakCount(motorACtr);
//  				    USART_SendInt(breakcount);
					if (breakcount >= e)
						break;
				}
				// if could not receive required count for too long the motor will stop
				PORTD &= ~_BV(motorADir2);
				OCR1A = 0;
//				USART_SendByte('2');


				if (breakcount >= e){
					USART_SendByte('S');				
				}
				else{
					USART_SendByte('F');
				}
			}
			if(c == 3){
//				USART_SendByte('K');
				PORTD |= _BV(motorBDir1);
				OCR1A = d * 256;
				for (int k =0 ; k < 10; k++){
					breakcount += breakCount(motorBCtr);
					if (breakcount >= e)
						break;
//					USART_SendInt(breakcount);
				}
				// if could not receive required count for too long the motor will stop
				PORTD &= ~_BV(motorBDir1);
				OCR1A = 0;
				for (int k =0 ; k < 20; k++){
					breakcount += breakCount(motorBCtr);
//					USART_SendInt(breakcount);
				}

//				USART_SendByte('3');

				if (breakcount >= e){
					USART_SendByte('S');				
				}
				else{
					USART_SendByte('F');
				}

			}
			if(c == 4){
//				USART_SendByte('P');
				PORTD |= _BV(motorBDir2);
				OCR1A = d * 256;
				for (int k =0 ; k < 20; k++){
					breakcount += breakCount(motorBCtr);
					if (breakcount >= e)
						break;
				}
				// if could not receive required count for too long the motor will stop
				PORTD &= ~_BV(motorBDir2);
				OCR1A = 0;
				USART_SendByte('4');

				if (breakcount >= e){
					USART_SendByte('S');				
				}
				else{
					USART_SendByte('F');
				}

			}

		}


    }
    
    return 0;
}
