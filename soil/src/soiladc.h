#include <SoftwareSerial.h>

// 
//    FILE:
// VERSION: 0.1.00
// PURPOSE: qtouch sensor  for imwave
//
//
// HISTORY:
//

#ifndef imQSoiladc_h
#define imQSoiladc_h

//#include "Arduino.h"
#include "imframe.h"
#include "imdebug.h"
#include "imatmega.h"

uint16_t cpuVin;
uint16_t cpuTemp;
uint16_t cpuVinCycle=0;
volatile uint16_t adcReading;


const uint16_t ledFadeTable[32] = {0, 1, 1, 2, 2, 2, 3, 3, 4, 5, 6, 7, 9, 10, 12, 15, 17, 21, 25, 30, 36, 43, 51, 61, 73, 87, 104, 125, 149, 178, 213, 255}; // this is an exponential series to model the perception of the LED brightness by the human eye

int idx1,idx2,idx3;
int idMax,idMin;
int shift1,shift2;

#define TPIN1 A0
#define TPIN2 2

#define LEDB1 6
#define LEDB2 1

// ADC complete ISR
ISR (ADC_vect)  {
     uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
//  ADCSRA |= (1 << ADIF);     // Clear ADIF
  adcReading = (high<<8) | low;
}  // end of ADC_vect


//imTouch touch;
int adcRead(void){
  ADCSRA  |=_BV(ADSC); // Start conversion
  
  while (!(ADCSRA&_BV(ADIF)));    
  ADCSRA  |=_BV(ADIF); // Clear ADIF

  return ADC;
}


int martinread(byte ADCChannel, int samples=200)
{
 ADCSRA = 0b11000101; // enable ADC (bit7), initialize ADC (bit6), no autotrigger (bit5), don't clear int-flag  (bit4), no interrupt (bit3), clock div by 16@16Mhz=1MHz (bit210) ADC should run at 50kHz to 200kHz, 1MHz gives decreased resolution
 	long _value = 0;
	for(int _counter = 0; _counter < samples; _counter ++)
	{
		// set the analog pin as an input pin with a pullup resistor
		// this will start charging the capacitive element attached to that pin
		pinMode(ADCChannel, INPUT_PULLUP);
		
		// connect the ADC input and the internal sample and hold capacitor to ground to discharge it
		ADMUX |=   0b11111;
		// start the conversion
		ADCSRA |= (1 << ADSC);

		// ADSC is cleared when the conversion finishes
		while((ADCSRA & (1 << ADSC)));

		pinMode(ADCChannel, INPUT);
		_value += analogRead(ADCChannel);
	}
	return _value / samples;
}

int rawAnalog( void )
{
  adcReading=3;
//  https://forum.arduino.cc/index.php?topic=45949.0
 // Generate an interrupt when the conversion is finished
 ADCSRA |= _BV( ADIE );
 // Enable Noise Reduction Sleep Mode
 set_sleep_mode( SLEEP_MODE_ADC );
 sleep_enable();

 // Any interrupt will wake the processor including the millis interrupt so we have to...
 // Loop until the conversion is finished
 do
 {
   // The following line of code is only important on the second pass.  For the first pass it has no effect.
   // Ensure interrupts are enabled before sleeping
   sei();
   // Sleep (MUST be called immediately after sei)
   sleep_cpu();
   // Checking the conversion status has to be done with interrupts disabled to avoid a race condition
   // Disable interrupts so the while below is performed without interruption
   cli();
 }
 // Conversion finished?  If not, loop.
 while( ( (ADCSRA & (1<<ADSC)) != 0 ) );

 // No more sleeping
 sleep_disable();
 // Enable interrupts
 sei();

 // The Arduino core does not expect an interrupt when a conversion completes so turn interrupts off
 ADCSRA &= ~ _BV( ADIE );

 // Return the conversion result
 return( adcReading );
}


int  senseadctwice(void) {
  /*
      Capacitive sensing using charge sharing between 
      the S/H capacitor and an external sensing pad
      
      ATMega 328P
      Float/ref  Analog0  = PC0
      Sense an  Analog1 = PC1
  */  
#define PC0 1
#define PC1 0
  int dat1,dat2;
  
  // Precharge Low
  PORTC = _BV(PC0);    // S/H Charge Vdd (Analog0), Cext (Analog1) gnd
  DDRC = _BV(PC1)|_BV(PC0);
   ADMUX  =_BV(REFS0)|0x0f;
  delayMicroseconds(2);
  ADMUX  =_BV(REFS0)|PC0;  // Charge S/H cap from Analog0
  
  delayMicroseconds(8);
  DDRC  &=~(_BV(PC1));  // float input
              // additional delay due to ADC logic

  ADMUX  =_BV(REFS0)|PC1; // Read Cext from Analog1
//  dat1=adcRead();
   dat1=rawAnalog();
  // Precharge High
  ADMUX  =_BV(REFS0)|PC0;  // Charge S/H cap from Analog0
  
  PORTC = _BV(PC1);    // S/H Charge gnd (Analog0), Cext (Analog1) Vdd
  DDRC = _BV(PC1)|_BV(PC0);
  delayMicroseconds(8);

  DDRC  &=~(_BV(PC1));
  PORTC =0;      // pull up off
            // additional delay due to ADC logic

  ADMUX  =_BV(REFS0)|PC1; // Read Cext from Analog1
//  ADCSRA  |=_BV(ADSC); // Start conversion

  
//  while (!(ADCSRA&_BV(ADIF)));
//  ADCSRA  |=_BV(ADIF); // Clear ADIF

//  dat2=ADC;
//  dat2=adcRead();
  dat2=rawAnalog();
  return (dat2-dat1)*10;
}

int sense(byte ADCChannel, int samples=20)
{
  ACSR=48;
  ADMUX  =_BV(REFS0)|0x0f;
//  ADCSRA  =_BV(ADEN)|_BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0); // Enable ADC, Set prescaler to 128
  ADCSRB=0;
 ADCSRA = 0b11000101; // enable ADC (bit7), initialize ADC (bit6), no autotrigger (bit5), don't clear int-flag  (bit4), no interrupt (bit3), clock div by 16@16Mhz=1MHz (bit210) ADC should run at 50kHz to 200kHz, 1MHz gives decreased resolution
 	long _value = 0;
	for(int _counter = 0; _counter < samples; _counter ++)
	{
            _value+=senseadctwice();

         }
	return _value / samples;
}




void SetupQtouch()
{
  pinMode(LEDB2, OUTPUT);
// pinMode(LEDB1, OUTPUT);
 //  analogWrite(LEDB1, 100);
//  analogWrite(LEDB1, 100);
//  digitalWrite(7, 100);
 // if (funtest()>0){
//  digitalWrite(LEDB2, HIGH);
//  delay(300);
 //  digitalWrite(LEDB2, LOW);
  DBGLEDON();
  delay(100);
 // }
  //  ref1=ADCTouchRead(A1,30);
 //   ref2=ADCTouchRead(A0,30);

//  touch.setup();
 SetupADC();
 DIDR0 = 0x00;
 //DBGLEDON();
  shift1=0;
  sense(TPIN1);
  for (int i=0;i<4;i++){
  delay(200);
   // shift1+=touch.check(TPIN1);
//   shift1+=martinread(TPIN1);
  shift1+=sense(TPIN1);
  }  
   shift1/=4;
   shift1-=10;
   
   cpuTemp=2;
   
   DBGLEDOFF();
}




void LoopQtouch() {
 power_adc_enable(); // ADC converter
//     touch.setup();
  // idx1=touch.check(TPIN1)-shift1;
//     idx1=touch.read(TPIN1,shift1);
//     idx1/=2;
 // idx1=martinread(TPIN1)-shift1;
 
 idx3=idx2;
 idx2=idx1;
 idx1=sense(TPIN1)-shift1;
 if (idMax<idx1)idMax=idx1;
 if (idMin>idx1)idMin=idx1;
 //int idx=sqrt32(idx1);
//   idx2=touch.check(TPIN2);
//   idx3=touch.check(TPIN3);
   
  // calculate the index to the LED fading table
  
   // analogWrite(9, ledFadeTable[idx]);
  //analogWrite(7, ledFadeTable[idx]);
    setSleepModeT2();

    ShutOffADC();
    ADMUX=0;
   // ADCSRB|=ACME;
    power_adc_disable();
}


void PrepareQtouch()
{
  // IMTimer::doneMeasure();
}  


void MeasureVCC(){
    SetupADC();
    cpuVin=internalVcc();
    cpuVin=internalVcc();
    cpuTemp=internalTemp();
    cpuTemp=internalTemp();
    ShutOffADC();
//    ADMUX=0;
    power_adc_disable();
}

void DataQtouch(IMFrame &frame)
{   
  if (cpuVinCycle % 18==2){
      MeasureVCC();
  }
  
   cpuVinCycle++;
  
   IMFrameData *data =frame.Data();
 //      data->w[2]=hh;
   data->w[1]=cpuTemp;
   data->w[0]=cpuVin;
   data->w[2]=0;
   data->w[3]=(uint16_t)idx1;
   data->w[4]=(uint16_t)idMin;
   data->w[5]=(uint16_t)idMax;
   data->w[6]=(uint16_t)shift1;
 //  data->w[7]=shift2;
 //  data->w[8]=shift3;
   data->w[10]=0xB11B;
   idMax=-30000;
   idMin=30000;
}





#endif
//
// END OF FILE
//
