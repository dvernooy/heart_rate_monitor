
/* 
******** v8 **********
* currently loaded firmware 
* update 12-24-15 
* 8 bits (1 byte) per sample
* every 12 s ... average of 4 points (every 3 s)
* changing chip to 328 would double resolution to every 6 s
* because 1024 bytes EEPROM available
* To see time history of HB, download EEPROM from AVR studio. Cut and paste from text file
* into excel.

******** v7
* update 12-24-15 for EEPROM save every 8s
* 6 bits
* Note - need to unpack the bits & use decoder
*
* weird behaviour with lcd - won't blank, don't understand it

******* V6
* updated 3-28-15 for refined hardware.
* current version running 

* updated LCD output for no leading zero
* updated trigger level to be lower w/ new hardware
* updated algorithm to track faster 
* updated rate of hunting for signal
* reduced dead window time

* future considerations:
1. even lower trigger level below 225
2. increase dead window back again

* Note pin assignments for PORTB.
*/

#define F_CPU 8000000UL
#include <avr/io.h>
#include <math.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>

// define loop output on PB7
#define LOCKER PD6
// Some macros that make the code more readable
#define OUTPUT_LOW(port,pin) port &= ~(1<<pin)
#define OUTPUT_HIGH(port,pin) port |= (1<<pin)
#define SET_INPUT(portdir,pin) portdir &= ~(1<<pin)
#define SET_OUTPUT(portdir,pin) portdir |= (1<<pin) 

//  portB 0..3 for LCD Comms
#define MOSI PB5
#define SCK PB4
#define SSEL PB2
#define RESET PB3
#define CMD     0 
#define DATA    1 
#define LOWER_LIM 150 //was 575, changed 3/21/15 to 225, then 150 because of new hardware
#define DEAD_WINDOW 40 //was 60, equates to about 40 ms to 70 ms vs. 10 ms burst

#define MOSI0 PORTB &= ~(1<<MOSI)
#define MOSI1 PORTB |= (1<<MOSI)
#define SCK0 PORTB &= ~(1<<SCK)
#define SCK1 PORTB |= (1<<SCK)
#define SSEL0 PORTB &= ~(1<<SSEL)
#define SSEL1 PORTB |= (1<<SSEL)
#define RESET0 PORTB &= ~(1<<RESET)
#define RESET1 PORTB |= (1<<RESET)

void adc_init(void);
void delay_ms(uint16_t);
void InitMono96x65(void);
uint16_t read_adc(void);		// Function Declarations
void ClearDisp(void);
void Startup(void);
void clock_init(void);

uint16_t result;
uint16_t counter;
uint16_t G;
uint16_t H;
uint16_t HB_ON;
double J1;
double coeff;
double Heff;
double Keff;
double tolerance;
double seconds_to_lock;
double attack_max;
double eep_avg;
uint16_t J;
uint16_t K;
uint16_t M;
uint16_t N;
uint16_t running_max;
uint16_t H_max;
uint16_t Half_H_max;


uint8_t n1[4] = {0,0,0,0};
uint8_t m1[3] = {0,0,0};
uint16_t eep_pos;
uint8_t eep_count;
uint16_t ii;
uint16_t current_HB;
uint16_t eep_HB;
uint8_t toggle;
uint8_t HB_locked;
 
const uint8_t NumLookupLow[11][10]={ 
{0xFC, 0xFE, 0x07, 0x03, 0x03, 0x03, 0x03, 0x07, 0xFE, 0xFC}, //0L
{0x07, 0x03, 0x03, 0x03, 0x03, 0xFF, 0xFF, 0x03, 0x03, 0x07}, //1L
{0x1F, 0x3F, 0x73, 0xE3, 0xC3, 0x83, 0x03, 0x03, 0x03, 0x07}, //2L
{0x18, 0x1E, 0x07, 0x83, 0x83, 0x83, 0xC3, 0xE7, 0x7F, 0x3C}, //3L
{0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xFF, 0xFF, 0xC0}, //4L
{0x18, 0x1E, 0x07, 0x03, 0x03, 0x03, 0x03, 0x87, 0xFF, 0xFC}, //5L
{0xFC, 0xFE, 0x87, 0x83, 0x03, 0x03, 0x03, 0x87, 0xFF, 0xFC}, //6L
{0x00, 0x00, 0x00, 0x1F, 0x7F, 0xF0, 0xC0, 0x00, 0x00, 0x00}, //7L
{0x7C, 0xFE, 0xC7, 0x83, 0x83, 0x83, 0x83, 0xC7, 0xFE, 0x7C}, //8L
{0x0E, 0x8F, 0xC7, 0xC3, 0xC3, 0xC3, 0xC3, 0x87, 0xFE, 0xFC}, //9L
{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} //blank
};

const uint8_t NumLookupHigh[11][10]={ 
{0x3F, 0x7F, 0xE0, 0xC0, 0xC0, 0xC0, 0xC0, 0xE0, 0x7F, 0x3F}, //0H
{0x00, 0x60, 0xE0, 0xC0, 0xC0, 0xFF, 0xFF, 0x00, 0x00, 0x00}, //1H
{0x38, 0x78, 0xE0, 0xC1, 0xC3, 0xC7, 0xCE, 0xDC, 0xF8, 0x70}, //2H
{0x38, 0x78, 0xE0, 0xC3, 0xC3, 0xC3, 0xC7, 0xEF, 0xFC, 0x78}, //3H
{0x03, 0x07, 0x1D, 0x19, 0x31, 0x61, 0xE1, 0xFF, 0xFF, 0x01}, //4H
{0xFF, 0xFF, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC7, 0xC3, 0xE1}, //5H
{0x3F, 0x7F, 0xE3, 0xC7, 0xC6, 0xC6, 0xC6, 0xE7, 0xE3, 0x61}, //6H
{0xE0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC3, 0xCF, 0xDE, 0xF8, 0xE0}, //7H
{0x3E, 0x7F, 0xE7, 0xC3, 0xC3, 0xC3, 0xC3, 0xE7, 0x7F, 0x3C}, //8H
{0x3F, 0x7F, 0xE3, 0xC1, 0xC1, 0xC1, 0xC3, 0xE3, 0x7F, 0x3F}, //9H
{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} //blank

};


const uint8_t ImgLookup[6][15]={ 
{0x3E, 0x7F, 0xFF, 0xFF, 0xFF, 0x7F, 0x1F, 0x07, 0x1F, 0x7F, 0xFF, 0xFF, 0xFF, 0x7F, 0x3E}, //HBH
{0x00, 0x80, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF, 0xFE, 0xFC, 0xF8, 0xF0, 0xE0, 0x80, 0x00}, //HBL
{0x00, 0x00, 0x00, 0x00, 0x06, 0x0F, 0x0F, 0x0F, 0x1F, 0x7A, 0x7B, 0x20, 0x00, 0x00, 0x00}, //BKH
{0x3C, 0x42, 0x81, 0x81, 0x42, 0xBC, 0xC0, 0xE0, 0x3C, 0x42, 0x81, 0x81, 0x42, 0x3C, 0x00}, //BKL
{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //BLNK
{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} //FILL

};


void delay_ms(uint16_t ms) { //'kills time' with calibration
 uint16_t delay_count = 500;
   volatile uint16_t i; 
   while (ms != 0) {
     for (i=0; i != delay_count; i++);
     ms--;
   }
 }

void Lcd_Write(char cd,unsigned char c){ 
char i; 
    SSEL0; 
    SCK0; 
    if (cd == 0){
	MOSI0;}
else {MOSI1;}
    SCK1; 

for(i=0;i<8;i++){ 
        SCK0; 
            if(c & 0x80) 
                MOSI1; 
            else 
                MOSI0; 
        SCK1; 
        c <<= 1; 
        //delay_ms(20); 
                } 
        SSEL1; 
        } 

//=================================================================================
//=================================================================================
//	Mono96x65
//

void InitMono96x65()
{
Lcd_Write(CMD,0xE2);
delay_ms(100);  
Lcd_Write(CMD,0xAF); 
Lcd_Write(CMD,0xA4); 
Lcd_Write(CMD,0x2F); 
Lcd_Write(CMD,0xB0); 
Lcd_Write(CMD,0x10); 
Lcd_Write(CMD,0x00);
Lcd_Write(CMD,0xA8);
}


void ClearDisp()
{
	Lcd_Write(CMD,0xB0);
	Lcd_Write(CMD,0x10);
	int i;
	Lcd_Write(CMD,0xAE); 
	for (i = 0; i < 864; i++)
		Lcd_Write(DATA,0x00);
	Lcd_Write(CMD,0xAF); 
}


void BlitMono(uint8_t filldata)
{
	Lcd_Write(CMD,0xB0);
	Lcd_Write(CMD,0x10);
	int i;
	for (i = 0; i < 864; i++)
		Lcd_Write(DATA,filldata);
}

void Gotoxy(uint8_t x,uint8_t y){ 
        Lcd_Write(CMD,(0xB0|(y&0x0F)));         // Y axis initialisation: 1011 yyyy            
        Lcd_Write(CMD,(0x00|(x&0x0F)));         // X axis initialisation: 0000 xxxx ( x3 x2 x1 x0) 
        Lcd_Write(CMD,(0x10|((x>>4)&0x07))); // X axis initialisation: 0001 0xxx  ( x6 x5 x4) 

} 

void print_num_low(int c){ 
    int i; 
            for ( i = 0; i < 10; i++ ){ 
                Lcd_Write(DATA,NumLookupLow[c][i]); 
        } 
} 

void print_num_high(int c){ 
    int i; 
            for ( i = 0; i < 10; i++ ){ 
                Lcd_Write(DATA,NumLookupHigh[c][i]); 
        } 	
} 

void print_img(int c){ 
    int i; 
            for ( i = 0; i < 15; i++ ){ 
                Lcd_Write(DATA,ImgLookup[c][i]); 
        } 
} 


void Startup(){
Lcd_Write(CMD,0xAE); //disable display 	
	Gotoxy(0,0);
	print_img(3);
    Gotoxy(0,1);
	print_img(2);
    
	Gotoxy(16,1);
	print_img(3);
    Gotoxy(16,2);
	print_img(2);

    Gotoxy(32,0);
	print_img(3);
    Gotoxy(32,1);
	print_img(2);

    Gotoxy(48,0);
	print_img(3);
    Gotoxy(48,1);
	print_img(2);

    Gotoxy(80,0);
	print_img(3);
    Gotoxy(80,1);
	print_img(2);
	
	Lcd_Write(CMD,0xAF); //enable display 
}

void parsnum(uint16_t number, uint16_t x, uint16_t y){
uint16_t out;
uint16_t temp;
uint8_t i;
//Lcd_Write(CMD,0xAE); //disable display 

for (i=1;i<4;i++){
	if (i == 1) temp = number;
	else temp = temp/10;
	out = temp % 10;
		if ((i == 3) && (out == 0)) {
		Gotoxy(x-i*12,y);
		print_num_low(10);
		Gotoxy(x-i*12,y+1);
		print_num_high(10);
		}
		else {
 		Gotoxy(x-i*12,y);
		print_num_low(out);
		Gotoxy(x-i*12,y+1);
		print_num_high(out);
		}	
	}
//Lcd_Write(CMD,0xAF); //enable display 
}

/* INIT ADC */
void adc_init(void)
{
	/** Setup and enable ADC **/
	ADMUX = (0<<REFS1)|	// Reference Selection Bits
			(0<<REFS0)|		// AVcc - external cap at AREF
			(0<<ADLAR)|		// ADC Left Adjust Result
			(0<<MUX2)|		// ANalog Channel Selection Bits
			(1<<MUX1)|		// ADC2 (PC2 PIN25)
			(0<<MUX0);
	
	ADCSRA = (1<<ADEN)|	// ADC ENable
			(0<<ADSC)|		// ADC Start Conversion
			(0<<ADATE)|		// ADC Auto Trigger Enable
			(0<<ADIF)|		// ADC Interrupt Flag
			(0<<ADIE)|		// ADC Interrupt Enable
			(1<<ADPS2)|		// ADC Prescaler Select Bits
			(0<<ADPS1)|
			(0<<ADPS0);
}


/* READ ADC PINS */
uint16_t read_adc(void) {
		ADCSRA |= (1<<ADSC);
		while(ADCSRA & (1<<ADIF));
		result = ADC;
		ADCSRA |= (1<<ADIF);
    return result;
}


void LCDcomm(void){	
	SSEL1;
	RESET1;
	MOSI1;
	SCK0;
	DDRB |= 0b00111100;

	delay_ms(500);						// Too long?

	SSEL0;
	Lcd_Write(CMD,0x11); 				// SLEEPOUT
	delay_ms(500);						// Too long?
	SSEL1;

	RESET0;
	delay_ms(500);
	RESET1;
	delay_ms(500);	// too short?

	SCK0;
	SSEL0;
	}


void clock_init(void)
{
        /* write high byte first for 16 bit register access: */
        TCNT1H=0;  /* set counter to zero*/
        TCNT1L=0;
        // Mode 4 table 14-4 page 132. CTC mode and top in OCR1A
        // WGM13=0, WGM12=1, WGM11=0, WGM10=0
        TCCR1A=(0<<COM1B1)|(0<<COM1B0)|(0<<WGM11);
        TCCR1B=(1<<CS12)|(1<<CS10)|(1<<WGM12)|(0<<WGM13); // crystal clock/1024
        // At what value to cause interrupt. You can use this for calibration
        // of the clock. For 8 MHz & 1024 divider, the values 0x5B8D gives 3s overflow
		// to trigger the interrupt
       
        OCR1AH=0x5B;
        OCR1AL=0x8D;
        // interrupt mask bit:
        TIMSK1 = (1 << OCIE1A);
}


// interrupt, step seconds counter
ISR(TIMER1_COMPA_vect){
	eep_avg = eep_avg + (double) current_HB;
	if (eep_count == 3){
       eep_avg = eep_avg/4.0;
       eep_HB = (uint16_t) eep_avg; 	
		if (eep_pos < 511) {
				eeprom_write_byte((uint8_t *)eep_pos,eep_HB); 			
				eep_pos++;
		}
	eep_avg = 0.0;
	eep_count = 0;
	}
	else {
		eep_count++;
	}
}



void HB_disp(uint8_t heartbeat){
//	Lcd_Write(CMD,0xAE); //disable display
	if (heartbeat == 1){
    Gotoxy(2,6);
	print_img(1);
	Gotoxy(2,7);
	print_img(0);
    }
	else {
	Gotoxy(2,6);
	print_img(4);
	Gotoxy(2,7);
	print_img(4);
	}
//    Lcd_Write(CMD,0xAF); //enable display 
}


int main(void) {
	adc_init();
	LCDcomm();
	InitMono96x65();
	ClearDisp();
	Startup();
	
	SET_OUTPUT(DDRD, LOCKER);// initialize PORTB.7 as output  
	delay_ms(25);	// wait a bit then toggle it
    OUTPUT_LOW(PORTD, LOCKER);
    delay_ms(100);
	OUTPUT_HIGH(PORTD, LOCKER);
	delay_ms(100);
	OUTPUT_LOW(PORTD, LOCKER);
	
	/*eeprom_init();*/
	eep_pos = 0;
	eep_count = 0;
	eep_avg = 0.0;
	eep_HB = 0;
	toggle = 1;
	clock_init();

		
//SET DEFAULT HEART RATE
K = 85; //starting HB
H = 300; //inter-beat timer - starting point
H_max = 500;
M = (uint16_t) DEAD_WINDOW; //fix the dead window
G = 0.5*M;//window filter start
N = LOWER_LIM;
running_max = 0;    
HB_ON = 0;//HB display
tolerance = 0.05;
seconds_to_lock = 8.0; //number of seconds to lock onto a new value within tolerance
attack_max = 0.35; //maximum value for coeff
HB_locked = 0;

while (1){
	H++;
	//READ ADC VALUE
	read_adc();
	//delay_ms(1);
	//TRIGGER TEST?
	if ((result > N) && (G == 0)) {
		if ((K<=168) || (K>200)) {
			M = 40;
		}
		else {
			M = 40 + (168-K);
		}	
	G = M;
	if (HB_locked ==0){
		HB_locked = 1;
		for (ii = 0; ii<512; ii++) {
			eeprom_write_byte((uint8_t *)ii,0xFF);
		}
		sei();
	}
	}
	// UPON TRIGGER	... DEAD ZONE ON, DISPLAY HB, RESET DEAD ZONE WIDTH, CALC HR
	if (G == M) {
		OUTPUT_HIGH(PORTD, LOCKER);
		Heff = 37312/K; //means loop time is about a ms
		H_max = (uint16_t) Heff;
		Half_H_max = 0.5*H_max;
		J1 = 37312/H;
		Keff = (double) K;
		J = (uint16_t) J1;
		H = 0;
		    if ((J < 20) || (J > 230)) { //then we have a bad value
			}
				
			else {
				//HEART BEAT FILTER
				//ADJUST HB with a partial step to current value
				if ((J>0.5*K) && (J< 2*K)){
					coeff = 1 - exp(log(tolerance)/(seconds_to_lock*Keff/60)) ;
						if (coeff > attack_max) coeff = attack_max ; 
				}
				
				else coeff = 0.05;
		
				if (K > J) {
					//K --;
					K = K - coeff*(K-J);
				}
				
				if (K < J) {
					//K++;
					K = K + coeff*(J-K);
					}
			}
		}
    current_HB = K;        
	parsnum(K, 85, 5);// OUTPUT CURRENT HR ... once per HB loop --NB, not happening once per HB loop, but once per SW loop!
	if (result > running_max) running_max = result; // keep track of highest sig level
	if (G > 0) G--;
	// HEART BEAT LCD FLASHER - LOGIC
	if ((H < Half_H_max) && ( HB_ON == 1)) {
		HB_disp(0);
		HB_ON = 0;
		}
	if ((H > Half_H_max) && ( HB_ON == 0)) {
		HB_disp(1);
		HB_ON = 1;
		}
	// HEART BEAT SIGNAL TRIGGERED ... START DEAD ZONE
	if (G == 1) {
		// CALCULATE MAX ADC, NEW THRESHOLD & THEN RESET MAX ADC
			//if ((running_max > 1.1*LOWER_LIM) N = 0.9*running_max; // old, do not use
			if (0.5*running_max > LOWER_LIM) N = 0.5*running_max; 
			else N = LOWER_LIM;
		running_max = 0;
		// CALCULATE HEART RATE PER MINUTE
		OUTPUT_LOW(PORTD, LOCKER); // turn off dead zone, turn on trigger zone
		}
	if ((G == 0) && (H > H_max) && (N>LOWER_LIM)) N = 0.95*N; //drop the lower limit aggressively if we don't find a HB signal
	}//while(1)
}//main