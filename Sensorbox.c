/*
;   Project:       Updated software for older Sensorbox.
;                  for use with SmartEVSEv2 with modbus software 2.10+
;                  will also work with old protocol, auto sensing protocol type.
;
;   Date:          26 January 2020
;
;   Changes:
;   1.0  Initial release
;   1.1  Version build with XC8 and MPLABX, based on sensorbox 1.5 code, but for PIC18F14K22
;
;   (C) 2013-2020  Michael Stegen / Stegen Electronics
;
;	Current measurement calculations, from openenergymonitor.org
; 
;   Build with MPLAB X v5.25 and XC8 compiler version 2.10
;
;   If you get "(902) no chip name specified" error messages, make sure the xc8-cc compiler is used.
;
;   set in XC8 global options the C standard to "C90"
;   set XC8 linker memory model settings to: double 32 bit, float 32 bit
;   extended instruction set is not used on XC8
;
;
; 
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
*/

#include <xc.h>
#include <string.h>
#include <math.h>
#include "Sensorbox.h"

// Configuration settings
#pragma	config FCMEN = OFF,	IESO = OFF, PCLKEN = ON
#pragma config PLLEN = ON, FOSC = IRC
#pragma	config BORV = 30, BOREN = OFF, PWRTEN = ON
#pragma	config WDTPS = 2048, WDTEN = OFF     // WDT timeout
#pragma config MCLRE  = OFF, HFOFST = OFF
#pragma config XINST = OFF, BBSIZ = OFF, LVP = OFF, STVREN = ON
#pragma	config CP0 = OFF, CP1 = OFF, CPD = OFF, CPB = OFF
#pragma	config WRT0 = OFF, WRT1 = OFF
#pragma	config WRTD = OFF, WRTB = OFF, WRTC = OFF
#pragma	config EBTR1 = OFF, EBTR0 = OFF
#pragma	config EBTRB = OFF



// Global data
char TXbuffer[50];                                                              // RS485 Transmit buffer
char RXbuffer[50],RXpacket[50];                                                 // RS485 Receive and working buffer
char Tbuffer[50];                                                               // temp buffer
double Irms[3];
int lastSampleI, sampleI, tempI;                                                // sample holds the raw analog read value, lastSample holds the last sample
long filteredI, filtI_div4, tempL;
long sqI;
unsigned char RX1byte, Transmit=0, LegacyProtocol=1;
unsigned char idx = 0, ISRFLAG = 0, ISRTXFLAG = 0, ISRTXLEN = 0;
unsigned long Timer = 0;                                                        // mS counter
unsigned long SecTimer, ModbusTimer, LedTimer;


int sampleI_CT[3]={512,512,512};
long filteredI_CT[3]={0,0,0};


void __interrupt() ISR (void)
{
    while (PIR1bits.RCIF)                                                       // Uart1 receive interrupt? RS485
    {
        RX1byte = RCREG;                                                        // copy received byte

        if (Timer > (ModbusTimer + 3))                                          // last reception more then 3ms ago? 
        {
            idx = 0;                                                            // clear idx in RS485 RX handler
        }  
        if (idx == 50) idx--;                                                   // max 50 bytes in buffer
        RXbuffer[idx++] = RX1byte;                                              // Store received byte in buffer

        ModbusTimer = Timer;
    }

    if (PIR1bits.TXIF && PIE1bits.TXIE)                                         // Uart1 transmit interrupt? RS485
    {
        TXREG1 = TXbuffer[ISRTXFLAG++];                                         // send character
        if ((ISRTXFLAG == ISRTXLEN)|| ISRTXFLAG == 50)                          // end of buffer
        {
            PIE1bits.TXIE = 0;                                                  // clear transmit Interrupt for RS485 after sending last character
            ISRTXFLAG = 0;                                                      // end of transmission.
        }                                                                       // we switch off the transmitter in the ISR loop, after the final character has been sent..
    }
    
    // Timer 2 interrupt, called 1000 times/sec
    if (PIR1bits.TMR2IF)                                                     
    {
        Timer++;                                                                // mSec counter (overflows in 1193 hours)
        
        if (Timer > LedTimer+500) LED_SetLow;                                   // LED off after 0.5 second
                
        if (!ISRTXFLAG && TXSTAbits.TRMT && Transmit) {
            RS485_RECEIVE;                                                      // set RS485 transceiver to receive if the last character has been sent
            
            Transmit = 0;
            if (LegacyProtocol) {
                PIE1bits.RCIE = 0;                                              // Disable receive interrupt
                SPBRG = 0x40;                                                   // set baudrate to 9600 bps
                SPBRGH = 0x03;                                                  // 
                PIE1bits.RCIE = 1;                                              // Enable Receive interrupt    
            }
        }    
                
        PIR1bits.TMR2IF = 0;                                                    // clear interrupt flag
    }
}

void initialize(void) {
    
 	OSCCON = 0b11100000;                                                        // setup 32Mhz internal oscillator (max 64Mhz)
	PORTA = 0;                                                                  // Init PORTA
 	LATA = 0;
	ANSEL = 0b11110000;                                                         // RC0, RC1, RC2, RC3 are analog inputs (pin 16,15,14,7)
	ANSELH = 0;                                                                 // All digital IO
	TRISA = 0b00000000;                                                         // Set portA all outputs
	PORTC = 0;
	TRISC = 0b00001111;                                                         // RC0-RC3 analog inputs
	PORTB = 0;
	TRISB = 0b00000000;                                                         // all outputs

    SPBRG = 0x40;                                                               // set baudrate to 9600 bps
    SPBRGH = 0x03;                                                              // 

	BAUDCON = 0b00001000;                                                       // 16 bit Baudrate register is used
	TXSTA = 0b00100100;                                                         // Enable TX, 8 bit, Asynchronous mode
	RCSTA = 0b10010000;                                                         // Enable serial port TX and RX, 8 bit. 

	ADCON0 = 1;                                                                 // ADC enabled
	ADCON1 = 0;
	ADCON2 = 0b10000010;                                                        // Right justify, Tacq = 0 uS, FOSC/32
 
    PR2 = 0x7C;                                                                 // Timer 2 frequency value -> 1Khz @ 32 Mhz
    T2CON = 0x1E;                                                               // Timer 2 On, 1:16 prescale ,1:4 postscale 8 bit counter
        
    PIE1bits.RCIE = 1;                                                          // enable receive interrupt
    PIE1bits.TMR2IE = 1;                                                        // enable timer 2 interrupt
    INTCONbits.PEIE = 1;                                                        // peripheral interrupts enabled
    INTCONbits.GIE = 1;                                                         // global interrupts enabled
}


unsigned int ReadAnalog(void)                                                   // Start ADC conversion, and return result
{
    ADCON0bits.GO = 1;                                                          // start next conversion on the selected channel
    while(ADCON0bits.GO);                                                       // wait for the adc conversion to finish
    return ADRES;                                                               // return result
}

// Poly used is x^16+x^15+x^2+x
// calculates 16-bit CRC of given data
// used for Frame Check Sequence on data frame
unsigned int crc16(unsigned char *buf, unsigned char len) {
    unsigned int crc = 0xffff;
    
    // Poly used is x^16+x^15+x^2+x
    for (int pos = 0; pos < len; pos++) {
        crc ^= (unsigned int)buf[pos];                                          // XOR byte into least sig. byte of crc

        for (int i = 8; i != 0; i--) {                                          // Loop over each bit
            if ((crc & 0x0001) != 0) {                                          // If the LSB is set
                crc >>= 1;                                                      // Shift right and XOR 0xA001
                crc ^= 0xA001;
            } else                                                              // Else LSB is not set
                crc >>= 1;                                                      // Just shift right
        }
    }        

    return crc;
}

unsigned int crc16sensorbox1(unsigned char *buf, unsigned char len) {
    unsigned int crc = 0xffff;
    
    // Poly used is x^16+x^12+x^5+x
    unsigned int c;
    int i;
    while (len--) {
        c = *buf;
        for (i = 0; i < 8; i++) {
            if ((crc ^ c) & 1) crc = (crc >> 1)^0x8408;
            else crc >>= 1;
            c >>= 1;
        }
        buf++;
    }
    crc = (unsigned int) (crc ^ 0xFFFF);

    return crc;
}




// Create HDLC/modbus frame from data, and copy to output buffer
// Start RS485 transmission, by enabling TX interrupt
void RS485SendBuf(char *buffer, unsigned char len, unsigned char protocol) {
    char ch, index = 0;
    
    while (ISRTXFLAG) {}                                                        // wait if we are already transmitting on the RS485 bus (blocking)
    
    LED_SetHigh;                                                                // LED on
    LedTimer = Timer;                                                           // Set LED timer
    
    if (protocol) {                                                             // Legacy protocol?
        
        PIE1bits.RCIE = 0;                                                      // Disable receive interrupt
        SPBRG = 0x0A;                                                           // set baudrate to 1200 bps
        SPBRGH = 0x1A;                                                          // 
        PIE1bits.RCIE = 1;                                                      // Enable Receive interrupt      
        
        TXbuffer[index++] = 0x7E;                                               // start with sync flag in buffer

        while(len--) {
            ch = *buffer++;                                                     // load next byte

            if ((ch == 0x11) || (ch == 0x12) || (ch == 0x13) || (ch == 0x7E) || (ch == 0x7D)) {	// check for escape character
                ch = ch^0x20;
                TXbuffer[index++] = 0x7D;                                       // insert escape character
            }
            TXbuffer[index++] = ch;                                             // load data in buffer
        }
        TXbuffer[index++] = 0x7E;                                       		// end with sync flag in buffer
    } else {                                                                    // Modbus protocol
        
        PIE1bits.RCIE = 0;                                                      // Disable receive interrupt
        SPBRG = 0x40;                                                           // set baudrate to 9600 bps
        SPBRGH = 0x03;                                                          // 
        PIE1bits.RCIE = 1;                                                      // Enable Receive interrupt 
        
        while (len--) {     
            TXbuffer[index++] = *buffer++;                                      // load next byte
        }
    }

    ISRTXLEN = index;                                                           // number of bytes to transfer
    RS485_TRANSMIT;                                         					// set RS485 transceiver to transmit, will be disabled in main loop

    NOP(); NOP(); NOP();                                                        // small delay
    
    Transmit = 1;
    PIE1bits.TXIE = 1;                                                          // enable transmit Interrupt for RS485
}


// Ch should be 0,1 or 2
double ReadCTnew(unsigned char Ch)
{
	unsigned int n;
    long sumI = 0;
    
    ADCON0bits.CHS = Ch+4;                                                      // select the A/D channel  
	            
	sampleI=sampleI_CT[Ch];                                                     // Get Sample and Filter values
	filteredI=filteredI_CT[Ch];	
           
	for (n = 0; n < SAMPLES; n++)
  	{
        lastSampleI = sampleI;
    	sampleI = ReadAnalog();                                                 // Read analog input
              
        tempI = sampleI-lastSampleI;                                            // the most recent input change
        tempL = (long)tempI<<8;                                                 // re-scale the input change (x256)
        tempL+= filteredI;                                                      // combine with the previous filtered value
        filteredI = tempL-(tempL>>8);                                           // subtract 1/256, same as x255/256
 
        filtI_div4 = filteredI>>2;                                              // now x64
        // Root-mean-square method current
        // 1) square current values
        sqI = filtI_div4 * filtI_div4;
        sqI = sqI >>12;                                                         // scale back
        // 2) sum
        sumI += sqI;
    }
     
    sampleI_CT[Ch]=sampleI;
	filteredI_CT[Ch]=filteredI;                                                 // Store Sample and Filter values
    
    return sqrt((double)sumI/SAMPLES);                                          // Return squareroot of uncalibrated value
    //return ((double)sampleI-512)/4;                                           // Test: We use a potentiometer to manually adjust the value
}



void main(void)
{
    char *pBytes;
	char x,n, Second=0, DataReady=0;
	unsigned int cs;

    
    initialize();
    
    SecTimer = Timer;                                                           // initialize one second Timer
    LegacyProtocol = 1;                                                         // Start with old protocol at 1200 bps
    
    while (1)
    {
        if (RCSTAbits.OERR)                                                     // Uart1 Overrun Error?
        {
            RCSTAbits.CREN = 0;
            RCSTAbits.CREN = 1;                                                 // Restart Uart
            
            SecTimer = Timer+5000;                                              // Wait 5 seconds before sending anything
        }

        // Receive data from modbus
        // last reception more then 3ms ago?                                    // complete packet detected?
        if (idx>6 && Timer > (ModbusTimer + 3)) {
            // store received data packet
            memcpy(RXpacket, RXbuffer, idx);                                    // make local copy
            // set flag to length of data packet
            ISRFLAG = idx;
            idx = 0;                                                            // and make buffer available for new data
            
            cs = crc16(RXpacket, ISRFLAG);                                      // calculate checksum over all data (including crc16)
            if (RXpacket[0]==0x0a && RXpacket[1]==0x04 && RXpacket[5]==0x14 && !cs)   // check CRC
            {
                SecTimer = Timer;                                               // take new measurement after one second.
                
                LegacyProtocol = 0;                                             // Stay at 9600 bps, Sensorbox 2 modbus
                
                                                                                // Setup Modbus data
                Tbuffer[0]= 0x0a;                                               // Fixed Address 10 (0x0a) is Sensorbox
                Tbuffer[1]= 0x04;                                               // function byte
                Tbuffer[2]= 0x28;                                               // takes the bytes from the request. 28h bytes will follow
                Tbuffer[3]= 0x00;                                               // 
                Tbuffer[4]= 0x0A;                                               // Sensorbox version 1.0 = 0x0A
                Tbuffer[5]= 0x00;                                               // DSMR Version (unused)
                Tbuffer[6]= 0x03;                                               // 0x80 = P1, 0x03= 3CT's ,0x83 = P1+ 3CT

                n=7;
                for (x=0; x<(6*4) ;x++) {                                       // P1 data. Volts and Current set to 0
                    Tbuffer[n++] = 0;
                }
                for (x=0; x<3 ;x++) {
                    Irms[x] = Irms[x]* CAL;
                    pBytes = (char*)&Irms[x];                                   // get raw 4 byte Double 
                    Tbuffer[n++] = pBytes[3];                                   // Send MSB first
                    Tbuffer[n++] = pBytes[2];
                    Tbuffer[n++] = pBytes[1];
                    Tbuffer[n++] = pBytes[0];                                   // Send LSB last
                }
                cs = crc16(Tbuffer, n);                                         // calculate CRC16 from data			
                Tbuffer[n++] = ((unsigned char)(cs));
                Tbuffer[n++] = ((unsigned char)(cs>>8));	

                RS485SendBuf(Tbuffer, n, 0);                                    // send buffer to RS485 port
                DataReady = 0;
            }    
        }
    
        
        if (Timer > SecTimer+900 ) {                                            // Every 0.9 second this is executed
            
            SecTimer = Timer;
            
            if (!DataReady) {
                Irms[0] = ReadCTnew(0);                                         // Read the CT's
                Irms[1] = ReadCTnew(1);
                Irms[2] = ReadCTnew(2);
                DataReady = 1;
            }

            if (LegacyProtocol && ++Second >= 2) {                              // Every ~2 seconds Sensorbox 1 data is sent.
                
                Second = 0;
                // Sensorbox 1 code
                
                Tbuffer[0]= 0xff;                                               // Address Field = ff
                Tbuffer[1]= 0x03;                                               // Control Field = 03
                Tbuffer[2]= 0x50;                                               // Protocol = 0x5001
                Tbuffer[3]= 0x01;
                Tbuffer[4]= 0x01;
                Tbuffer[5]= 0x03;                                               // 3 CTs

                n=6;
                for (x=0; x<3; x++) {
		      	pBytes = (char*)&Irms[x];	
            	Tbuffer[n++] = pBytes[0];
                Tbuffer[n++] = pBytes[1];
                Tbuffer[n++] = pBytes[2];
                Tbuffer[n++] = pBytes[3];
                }
                                                                                // Frame Check Sequence (FCS) Field
                cs = crc16sensorbox1(Tbuffer, n);                            	// calculate CRC16 from data			
                Tbuffer[n++] = ((unsigned char)(cs));
                Tbuffer[n++] = ((unsigned char)(cs>>8));	

                RS485SendBuf(Tbuffer, n, 1);                                 	// send buffer to RS485 port
                DataReady = 0;
            }
        } 
        
    } // while(1)
}
/*
 End of File
*/