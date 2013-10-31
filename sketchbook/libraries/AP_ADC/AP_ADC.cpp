extern "C" {
  // AVR LibC Includes
  #include <inttypes.h>
  #include <avr/interrupt.h>
  
//  #include "WConstants.h"
}

#include "Arduino.h"
#include "AP_ADC.h"

// Commands for reading ADC channels on ADS7844
static const unsigned char adc_cmd[9]=  { 0x87, 0xC7, 0x97, 0xD7, 0xA7, 0xE7, 0xB7, 0xF7, 0x00 };
static volatile long          adc_value[8]   = { 0, 0, 0, 0, 0, 0, 0, 0 };
static volatile unsigned char adc_counter[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

static unsigned char ADC_SPI_transfer(unsigned char data)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR2A & (1<<UDRE2)) );
	/* Put data into buffer, sends the data */
	UDR2 = data;
	/* Wait for data to be received */
	while ( !(UCSR2A & (1<<RXC2)) );
	/* Get and return received data from buffer */
	return UDR2;
}


ISR (TIMER2_OVF_vect)
{
  uint8_t ch;
  unsigned int adc_tmp;

  //bit_set(PORTL,6);                                 // To test performance
  bit_clear(PORTC,4);                                 // Enable Chip Select (PIN PC4)
  ADC_SPI_transfer(adc_cmd[0]);                       // Command to read the first channel
  for (ch=0;ch<8;ch++)
      {
      if (adc_counter[ch] >= 16)                       // To prevent overflow of adc_value
      {                                                //       
	    adc_value[ch] /= 2;
	    adc_counter[ch] /= 2;
	  }
	  adc_tmp = ADC_SPI_transfer(0)<<8;               // Read first byte
      adc_tmp |= ADC_SPI_transfer(adc_cmd[ch+1]);     // Read second byte and send next command
      adc_value[ch] += adc_tmp>>3;                    // Shift to 12 bits
      adc_counter[ch]++;                              // Number of samples
      }
  bit_set(PORTC,4);                                   // Disable Chip Select (PIN PC4)
  //bit_clear(PORTL,6);                               // To test performance
  TCNT2 = 104;                                        // 400 Hz
}


// Constructors ////////////////////////////////////////////////////////////////
AP_ADC::AP_ADC()
{
}

// Public Methods //////////////////////////////////////////////////////////////
void AP_ADC::init(void)
{
  unsigned char tmp;

  pinMode(ADC_CHIP_SELECT,OUTPUT);

  digitalWrite(ADC_CHIP_SELECT,HIGH);                 // Disable device (Chip select is active low)

  // Setup Serial Port2 in SPI mode
  UBRR2 = 0;
  DDRH |= (1<<PH2);                                   // SPI clock XCK2 (PH2) as output. This enable SPI Master mode
  // Set MSPI mode of operation and SPI data mode 0.
  UCSR2C = (1<<UMSEL21)|(1<<UMSEL20);                 //|(0<<UCPHA2)|(0<<UCPOL2);
  // Enable receiver and transmitter.
  UCSR2B = (1<<RXEN2)|(1<<TXEN2);
  // Set Baud rate
  UBRR2 = 2;                                          // SPI clock running at 2.6MHz


  // Enable Timer2 Overflow interrupt to capture ADC data
  TIMSK2 = 0;                                         // Disable interrupts
  TCCR2A = 0;                                         // normal counting mode
  TCCR2B = _BV(CS21)|_BV(CS22);                       // Set prescaler of 256
  TCNT2  = 0;
  TIFR2  = _BV(TOV2);                                 // clear pending interrupts;
  TIMSK2 = _BV(TOIE2) ;                               // enable the overflow interrupt
}

// Read one channel value
int AP_ADC::Ch(unsigned char ch_num)         
{
  int result;         

  while(adc_counter[ch_num] < 2) { }				// Wait for at least 2 samples in accumlator

  cli();             
  if (adc_counter[ch_num]>0)
	result = adc_value[ch_num]/adc_counter[ch_num];
  else
	result = 0;
	
  adc_value[ch_num] = 0;                              // Initialize for next reading
  adc_counter[ch_num] = 0;
  sei();
  return(result);
}
