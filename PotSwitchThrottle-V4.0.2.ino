/*

    Simple XPressNet DCC Throttle

    Version 4.0.2 March 27, 2024

    Copyright (C) 2010, 2014, 2016, 2022, 2023
    John Alexander Stewart, Ottawa, Canada.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

Changes March 27 2024
	- added the ability to have an ESTOP button;
	- frameworks for button press and toggle switch in code;
	- defines for different processors streamlined. 

Notes:

1) You have to set the Locomotive ID, AND the XpressNet Bus address. 
These are hard-coded into the throttle, so you need to keep a table of
XpressNet Bus Addresses in use, and use a free address. These defines are
below, near the top of the actual code.

Example:  #define MY_ADDRESS 7


The Locomotive ID has to be something up to 4 digits. If you have a low
Locomotive ID (such as 'ole Number 10) then set the address to "10". 
Do NOT prepend a zero to it, otherwise the Arduino compiler will presume
you are entering an Octal number. I know from personal experience that this
is hard to debug, so keep this in mind!

 Example: #define LOCO_ID 1670

2) There can be output on the serial console, if you have an Arduino with
more than 1 serial port (like a Mega 2560). If running this from the 
Arduino development GUI, start up the "console", and recompile with "VERBOSE"
defined. For single-port Arduino boards, the serial port is dedicated to 
talking to the XpressNet bus.

3) This file contains lots of prints for possible XPressNet messages.ifdef VERBOSE
Serial.println()stants below
for the delay between last time the locomotive moved and when the headlight is 
turned off.

5) Here are the hardware connections:

You'll need some hardware:
- Arduino, any model;
- 10k Potentiometer, with 3 wires soldered.
- SPDT Centre off switch.
- LED, and associated resistor (10k is usually correct)
- 1N4041 Diode, to ensure that power from XpressNet is correct polarity.
- a case to hold this all in once assembled.

Some ground and VCC connections are assumed;
- "top" connection of Potentiometer - to VCC on Arduino;
- "bottom" connection of Potentiometer - to ground bus;
- "Middle" connection of SPDT centre off to ground;
- ground and VCC lines of Max485; 
  
From the DIN XpressNet plug and cable, we get:
  +12v           - goes to Arduino
  Ground         - goes to Arduino
  Transmit Data  - goes to Max485
  Receive Data   - goes to Max485

On the Arduino, 
   
Digital Pin 12 - to control pin of MAX485 - send or receive control for XPressNet.
Digital Pin 13 - LED - solid on for ok; flashing for "track power off"
           (see more possible blinking messages below)
Analog  Pin A0 - Analog input pin from potentiometer
Digital Pin 2 - "forward" or "off" from SPDT switch (forward == grounded)
Digital Pin 3 - "reverse" or "off" from SPDT switch (reverse == grounded)

GND   - to the throttle common ground, and to the XPressNet "M" connection.
RAW   - voltage input from the XPressNet "L" connection. +12v typical.

VCC   - the Arduino will provide this from its' on-board regulator; it is +5v.

TXn,
RXn   - The serial data port for sending/receiving to the XpressNet bus.

6) and on the MAX485 -

Pin 1  RO Serial rx   -   Arduino Serial receive (Tx, or Tx0, but can be changed
                          in the defines; see notes and code below)
Pin 2  /RE rec ena     -  Arduino Pin 12 (defined in code as MAX485_CONTROL_PIN)
Pin 3  DE send ena     -  Arduino Pin 12 (ditto above)
Pin 4  DI serial tx    -  Arduino Serial transmit (see note above about pin 1)
Pin 5  GND             -  same as the Arduino ground
Pin 6  A               -  XPressNet connector "A" on 5-pin DIN connector
Pin 7  B               -  XPressEnt connector "B" on 5-pin DIN connector
Pin 8  VCC             -  (5v or 3.3v) supplied from the Arduino.

7) 2 inputs  as outlined in the Arduino connections above; one is a SPDT (centre
   off) switch that alternatively grounds either pin 2 or pin 3 for forward/rev/
   neutral; if it is a "typical" switch with 3 contacts on the back, the center
   contact will be the "ground" connection. Look for defines in code for #define
   for pins FORWARD_SWITCH and REVERSE_SWITCH; 2 and 3 in the code below.

   A Linear taper potentiometer; really any value from 10K above should work fine,
   one side tied to ground, the other to vcc (3.3v or 5v) and the wiper to analogue pin
   POT_PIN - A0.

   If you want, a push button switch for toggling ESTOP on/off; one side tied to
   ground, the other to digital pin ESTOP_BUTTON - pin 7.
   
8) We communicate at 62.5k, 1 start bit, 9 data bits, 1 stop bit, no parity. Note that
   the receive data drives the throttle; there are no other timers or interrupts. 
   
NOTE 1: Without the serial receive, nothing happens; plug it into EXpressNet and it will
then work if wired correctly, and programmed! 

NOTE 2: I have used pin numbers and the #define name above, feel free to change them,
or take code for ESTOP, or F/R toggle, and add code to add functionality, such as sound, 
headlight (see the automatic headlight code for on/off working code) or whatever you
want to do.

-----------------------------------------------------------------------------------
Initialization:

At power up, we go through an initialization step:

1)  GET_COMMAND_STATION_STATUS_STEP 
    (ask the command station for its current status)
    
1a)  rinse and repeat step 1) until we are powered up.

2)  GET_COMMAND_STATION_SW_VERSION_STEP
    (ask the command station for its software version)
    
Ensure that we are at V3.0 or above.

3)  GET_LOCOMOTIVE_INFO_STEP
    (we find out our locomotive speed steps, current direction and
    speed, etc)
    
then either, if we are ok: PAST_INITIALIZATION
        if we are in a MU: LOCOMOTIVE_NOT_FREE
        and flash the LED quickly.
  
  
LED Indications:

Solid on:            Everything ok
Slow 50/50 flashing: Track power off
1 quick flash:       Locomotive part of a double header
2 quick flashes:     Locomotive not free - MU, not single locomotive
3 quick flashes:     Locomotive not free - part of a MU.
4 quick flashes:     Speed steps not supported - not 14,27,28,128.
5 quick flashes:     Command station not XPressNet 3.0 or higher
6 quick flashes:     internal error - Speed step not supported, but should have been caught before here.
7 quick flashes:     locomotive used by someone else
*/


///////////////////////////////////////////////
// LOCO ID
//
// DO NOT PUT LEADING ZEROES HERE!!!
// This is our loco id, in decimal format. It will get split into bytes
// later, for sending to the system.

#define LOCO_ID 5541  // NO leading zeroes. This is the address programmed into
                      // the decoder. A leading zero here will try and make the
                      // address as an OCTAL (base 8) address. Don't do this,
                      // as nobody in their right mind would keep track of 
                      // locomotives in octal format. Hex or binary maybe, but not
                      // octal.

/////////////////////////////////////////////////
// XpressNet address: must be in range of 1-31; must be unique. Note that some IDs
// are currently used by default, like 2 for a LH90 or LH100 out of the box, or 30
// for PC interface devices like the XnTCP.
#define MY_ADDRESS 17


// We will turn off the headlight (F0) after so many milliseconds.
// Light off timer starts when we go into neutral, or to zero speed.
// Light automatically on when going into gear (forward or reverse) and
// when we go from speed zero to some speed.
// 1 second, use 1000L, 5 seconds, 5000L, 10 seconds 10000L; basically
// we put three zeroes and an L on the end of the number of seconds.
#define LIGHT_OFF_DELAY 10000L

// control of the XPressNet RS-485 transmission. 
#define MAX485_CONTROL_PIN 12
#define LISTEN_MODE digitalWrite (MAX485_CONTROL_PIN, LOW)
#define SEND_MODE digitalWrite (MAX485_CONTROL_PIN, HIGH)

// status light.
#define LED_PIN 13
#define LED_ON digitalWrite (LED_PIN,HIGH);
#define LED_OFF digitalWrite (LED_PIN,LOW);

// where the potentiometer wiper wire goes:
#define POT_PIN A0

// for the forward/reverse switches = digital input pins
#define FORWARD_SWITCH 2
#define REVERSE_SWITCH 3

// for ESTOP button, if desired
#define ESTOP_BUTTON  7

// Which serial port is used, if we have more than one on the chip?
// 
// ATmega8, maybe others will NOT have UCSR0A defined, so we just use 
// this define to see what the serial port defines are.
// 
#ifdef UCSRA
  #define ANCIENT_BOARD
#else
  #define MODERN_BOARD
#endif
  
#ifdef ANCIENT_BOARD
  // definitely only one serial port here, and uses old #defines for serial.
  // and, do the bit-twiddling ports for this one-serial-port Arduino.
  #define UCSRA_PORT UCSRA
  #define UBRRH_PORT UBRRH
  #define UBRRL_PORT UBRRL
  #define UCSRB_PORT UCSRB
  #define UCSRC_PORT UCSRC
  #define RXEN_PORT RXEN
  #define TXEN_PORT TXEN
  #define RXCIE_PORT RXCIE
  #define UCSZ0_PORT UCSZ0
  #define UCSZ1_PORT UCSZ1
  #define UCSZ2_PORT UCSZ2
  #define UDR_PORT UDR
  #define RXC_PORT RXC
  #define FE_PORT FE
  #define PE_PORT PE
  #define DOR_PORT DOR
  #define UDRE_PORT UDRE
  #define TXC_PORT TXC
#else
  //Maybe we are running on a MEGA chip with more than 1 port? If so, you
  //can put the serial port to (say) port 1, and use the port 0 for status messages
  //to your PC. (look for the #define USE_SERIAL_PORT, and #define VERBOSE for
  //how to do this.
  //
  // Some chips, like the ATMega168, define UCSR0A, but only have 1 serial port, 
  // so you can not use another serial port here. (keep it at 0), or else you 
  // will get messages such as "UBRR3H was not declared in this scope"...

  // ATMega168, has UCSR0A defined but only 1 serial port...
  // ATMega368, ATMega1280, ATMega2560... and more have more ports.
  // 
  // We can put the Xpressnet bus config on another serial port
  // by changing the following #defines to use (say) "1" instead of "0"
  // in the name, eg, below, change UCSR0A to UCSR1A to use port 1 for XPressnet.
  // Right now, #define USE_SERIAL_PORT_0 is set, if you want to use port 3,
  // change that to USE_SERIAL_PORT_3

  #define USE_SERIAL_PORT_0
  #ifdef USE_SERIAL_PORT_0
  #define UCSRA_PORT UCSR0A
  #define UBRRH_PORT UBRR0H
  #define UBRRL_PORT UBRR0L
  #define UCSRB_PORT UCSR0B
  #define UCSRC_PORT UCSR0C
  #define RXEN_PORT RXEN0
  #define TXEN_PORT TXEN0
  #define RXCIE_PORT RXCIE0
  #define UCSZ0_PORT UCSZ00
  #define UCSZ1_PORT UCSZ01
  #define UCSZ2_PORT UCSZ02
  #define UDR_PORT UDR0
  #define RXC_PORT RXC0
  #define FE_PORT FE0
  #define PE_PORT UPE0
  #define DOR_PORT DOR0
  #define UDRE_PORT UDRE0
  #define TXC_PORT TXC0   
  #endif
  #ifdef USE_SERIAL_PORT_3
  #define UCSRA_PORT UCSR3A
  #define UBRRH_PORT UBRR3H
  #define UBRRL_PORT UBRR3L
  #define UCSRB_PORT UCSR3B
  #define UCSRC_PORT UCSR3C
  #define RXEN_PORT RXEN3
  #define TXEN_PORT TXEN3
  #define RXCIE_PORT RXCIE3
  #define UCSZ0_PORT UCSZ30
  #define UCSZ1_PORT UCSZ31
  #define UCSZ2_PORT UCSZ32
  #define UDR_PORT UDR3
  #define RXC_PORT RXC3
  #define FE_PORT FE3
  #define PE_PORT UPE3
  #define DOR_PORT DOR3
  #define UDRE_PORT UDRE3
  #define TXC_PORT TXC3   
  #endif

#endif


// when sending data, do NOT continue until the hardware has sent the data out
#define WAIT_FOR_XMIT_COMPLETE {while (!(UCSRA_PORT & (1<<TXC_PORT))); UCSRA_PORT = (1<<TXC_PORT); UCSRA_PORT = 0;}


// we save incoming data for decoding. I think that the largest packet is 6-7 bytes. 
// If we save 20 bytes, we should be more than fine.
#define MAXSAVED 20

//////////////////////////////////////////////////////////////
// XPressnet Call Bytes.
//
#define CALL_BYTE 0x100

// broadcast to everyone, we save the incoming data and process it later.
#define GENERAL_BROADCAST 0x160

// the address we look for when we are listening for ops to ME - 0x40 + GA.
// again, save the incoming data and process it later.
unsigned int myDirectedOps;

// the address we look for for our Call Byte Window - 0x60 + GA. When
// this sucker comes in, we are able to send data to the XPressNet bus.
unsigned int myCallByteInquiry;

// the address for a request acknowlegement sent from the stn - 0x00 + GA
// if this one comes in, we respond immediately with a "we are here" message.
unsigned int myRequestAck;

// for saving incoming data, we use these 2 variables.
int dataSaved;
int savedData[MAXSAVED];

// for the status LED;
int displayLEDState = LOW;    
boolean displayLEDflashing = false;
long flashInterval = 500;
long currentFlashCounter = 0;

// for the analogue potentiometer
int potValue = 0;                    // as read from the hardware
int oldPotValue = 2000;              // send ONLY changes
unsigned int analogueReadCounter = 0;// only do this every so often

int decoderSpeedSteps = 0;          // 28 bits by default on decoders...

// for the forward/reverse switches
boolean forward_closed = false;
boolean reverse_closed = false;
//int currentDirection = 0; // 3 = stop, 1 == forward, 2 == reverse 0 == invalid
boolean runningForward = true;
boolean inNeutral = true;

boolean speedControlsChanged = false;
boolean lightingChanged = false;

// ESTOP button pushed
boolean setESTOP = false;
boolean clearESTOP = false;
int pressPeriod = 500;
unsigned long pt = 0;
unsigned long rt = 0;

// when should we turn the headlight off, assuming we want it off?
unsigned long lightOffAt = 0;

// from the command station, at initialization, we get:
unsigned char functionGroupA;
boolean headlightOn = false;

// and, we calculate:
unsigned char encodedSpeedValue = 0;

// what state are we in?
#define STATE_NORMAL_OPS 1
#define STATE_POWER_OFF 2
#define STATE_ESTOP 3
#define STATE_SERVICE_MODE 4
int currentState = 0;


// power up initalization variables
#define GET_COMMAND_STATION_STATUS_STEP 0
#define GET_COMMAND_STATION_SW_VERSION_STEP 1
#define LOCOMOTIVE_NOT_FREE 2
#define WAIT_FOR_COMMAND_STATION_POWERUP 3
#define GET_LOCOMOTIVE_INFO_STEP 4
#define PAST_INITIALIZATION 200
int initializationStep = GET_COMMAND_STATION_STATUS_STEP;
int initializeError = 0;  // flashes LED pattern on error

// some pre-computed command sequences. Some of the bytes below
// will be computed and put into these byte strings.
unsigned char commandStatusSequence[] = {0x21, 0x24, 0x05};
unsigned char commandSWVersionSequence[] = {0x21,0x21,0x00};
unsigned char requestAckAck[] = {0x20, 0x20};
unsigned char EST[] = {0x21, 0x80, 0xA1};
unsigned char ESRES[] = {0x21,0x81,0xA0};
unsigned char locoInfoRequest[] = {0xE3, 0x00, 0x00, 0x00, 0x00};
unsigned char locoSpeedOps[] = {0xE4, 0x10, 0x00, 0x00, 0x00, 0x00};
unsigned char locoFunctionSet[] = {0xE4, 0x20, 0x00, 0x00, 0x00, 0x00};


// our locomotive, as computed as 2 bytes
unsigned char locomotiveHighByte = 0;
unsigned char locomotiveLowByte = 0;

// speedsteps for 28 speed steps. Choose the values from this table.
// This is quick, easy, and comes directly from the relevant RFP, so it
// should be correct.
unsigned char twentyEightTable[] = {
        B00000, // 0
        B00010, // 1
        B10010, // 2
        B00011, // 3
        B10011, // 4
        B00100, // 5
        B10100, // 6
        B00101, // 7
        B10101, // 8
        B00110, // 9
        B10110, // 10
        B00111, // 11
        B10111, // 12
        B01000, // 13
        B11000, // 14
        B01001, // 15
        B11001, // 16
        B01010, // 17
        B11010, // 18
        B01011, // 19
        B11011, // 20
        B01100, // 21
        B11100, // 22
        B01101, // 23
        B11101, // 24
        B01110, // 25
        B11110, // 26
        B01111, // 27
        B11111 // 28
        };



////////////////////////////////////////////////////////////////////
// set up the XpressNet interface on Serial 0 or 1 if we have a machine with more
// than 1 serial port; set it for the only serial port if we only have one.
// Ensure that there are no Serial commands "to the console" in this case.

void xpressnetSpeed () {
 // baud rate 

 // Johns oldcode UBRRH_PORT = 0;
 // Johns oldcode UBRRL_PORT = 0x0F;

 // about 6 years after I figured out the above numbers, someone came out
 // with some nice macros to help set for different boards.
 // google "dean Camera using USART in AVR-GCC July 17, 2016"
 // using the 16mhz frequency as F_CPU (standard Arduino, at least old-school)
 // gives us a "BAUD_PRESCALE" of 15, which is 0x0F from above.
 // 
 // this is good, as now there are Arduinos with different clock frequencies.

 // Xpressnet is 62,500 baud.
 #define USART_BAUDRATE 62500
 #define BAUD_PRESCALE ((((F_CPU / 16) + (USART_BAUDRATE / 2)) / (USART_BAUDRATE)) - 1)
 UBRRH_PORT  = (BAUD_PRESCALE >> 8);
 UBRRL_PORT = (BAUD_PRESCALE);

 UCSRA_PORT = 0;

 // from the ATmega8 manual, page 133, "The frame format used by the USART is set by 
 // the UCSZ2:0, UPM1:0 and USBS bits in UCSRB and UCSRC. The Receiver and 
 // Transmitter use the same setting. 

 // Enable receiver and transmitter
 // do NOT enable Receive Complete Interrupt Enable (RXCIE)
 // set UCSZ2 in UCSRB to 1 ...
 UCSRB_PORT = (1<<RXEN_PORT) | (1<<TXEN_PORT) | (0<<RXCIE_PORT) | (1<<UCSZ2_PORT);

 // and UCSZ1 and UCSZ0 both to 1, to ser 9-bit character size.
 UCSRC_PORT = (1<<UCSZ1_PORT) | (1<<UCSZ0_PORT);
}



////////////////////////////////////////////////////////////////////
// calculate the parity bit in the call byte for this guy
unsigned int callByteParity (unsigned int me) {
 int parity = (1==0);
 unsigned int vv;
 me = me & 0x7f;
 vv = me;

 while (vv) {
       parity = !parity;
       vv = vv & (vv-1);
 }
 if (parity) me = me | 0x80;
 return me;
}


////////////////////////////////////////////////////////////////////
void setup() {
  
 // the MAX485 chip for Xpressnet - set this up.
 // set the pin to output, and set it low.
 pinMode(MAX485_CONTROL_PIN, OUTPUT);
 LISTEN_MODE;

 // turn the LED off until we are running
 pinMode(LED_PIN, OUTPUT);
 LED_OFF;
 
 // set up the SPDT direction switch. Turn on pullup resistors.
 pinMode(FORWARD_SWITCH, INPUT);
 pinMode(REVERSE_SWITCH, INPUT);
 digitalWrite(FORWARD_SWITCH, HIGH); // pullup resistors ON
 digitalWrite(REVERSE_SWITCH, HIGH); // pullup resistors ON

 // ESTOP button, if required.
 pinMode (ESTOP_BUTTON, INPUT);
 digitalWrite(ESTOP_BUTTON, HIGH); // pullup resistors ON

 //#define VERBOSE
 #ifdef VERBOSE
 delay (2000); // so we can get the monitor open
 Serial.begin (250000);
 Serial.print ("starting setup\n");
 delay(1000);
 #endif
 
 dataSaved = 0;

  // calculate the locomotive address
  if (LOCO_ID <= 99) {
    locomotiveHighByte = 0;
    locomotiveLowByte = LOCO_ID & 0xFF;
  } else {
    locomotiveHighByte = (LOCO_ID>>8) + 0xC0;
    locomotiveLowByte = (LOCO_ID&0xFF);
  }


  //  locoInfoRequest string
  locoInfoRequest[2] = locomotiveHighByte;
  locoInfoRequest[3] = locomotiveLowByte;
  calculateXOR ( locoInfoRequest, 5);
  
  // locoSpeedOps string - put the loco info here
  locoSpeedOps[2] = locomotiveHighByte;
  locoSpeedOps[3] = locomotiveLowByte;
  
  // locoFunctionSet string - put the loco info here
  locoFunctionSet[2] = locomotiveHighByte;
  locoFunctionSet[3] = locomotiveLowByte;

  // set these up to match call bytes for this device
  myRequestAck = callByteParity (MY_ADDRESS | 0x00) | CALL_BYTE;
  myCallByteInquiry = callByteParity (MY_ADDRESS | 0x40) | CALL_BYTE;
  myDirectedOps = callByteParity (MY_ADDRESS | 0x60) | CALL_BYTE;

  #ifdef VERBOSE
  Serial.println("Start....");
  Serial.println ("myAddress is  "+ String(MY_ADDRESS)); 
  Serial.println("next...");
  Serial.print ("myDirectedOps "); Serial.println (myDirectedOps,HEX);
  Serial.print ("myRequestAck "); Serial.println (myRequestAck,HEX);
  #endif //VERBOSE

  xpressnetSpeed();

  #ifdef VERBOSE
  Serial.println ("finished setup");
  #endif
}


////////////////////////////////////////////////////////////////////
/* From the ATMega datasheet: */

int USART_Receive( void ) {
 unsigned char status, resh, resl;

 /* Wait for data to be received */
 /* get the USART Control and Status Register, and look for the RXC 
    (Receive Character) to be true */
 status = UCSRA_PORT;
 while ( !(status & (1<<RXC_PORT)) ) {status = UCSRA_PORT;}

 /* we have some data, lets look at it */

 /* Get status and 9th bit, then data */
 resh = UCSRB_PORT;
 resl = UDR_PORT;

 /* If error, return -1 */
 if ( status & ((1<<FE_PORT)|(1<<DOR_PORT)|(1<<PE_PORT))) {return -1;}

 /* no error, return the 9 bits of the received data */

 /* get the 9th bit, shift it, then return the 9 bits read */
 resh = (resh >> 1) & 0x01;
 return ((resh << 8) | resl);
}

////////////////////////////////////////////////////////////////////
void USART_Transmit (unsigned char data8) {
  /* wait for empty transmit buffer */
  while (!(UCSRA_PORT & (1<<UDRE_PORT))) {}
  /* put the data into buffer, and send */

 UDR_PORT = data8;
}



////////////////////////////////////////////////////////////////////
// send along a bunch of bytes to the Command Station
void sendCommand(unsigned char *dataString, int byteCount) {
   unsigned int i;
   
   SEND_MODE;
   for (i=0; i< byteCount; i++) {
     USART_Transmit (*dataString);
     dataString ++;
   }
   WAIT_FOR_XMIT_COMPLETE;
   LISTEN_MODE; 
}  

////////////////////////////////////////////////////////////////////
// make up the XOR byte, and put it in the LAST position.  
void calculateXOR (unsigned char *dataString, int byteCount) {
  unsigned int i;
  unsigned char xo;
  
  xo = 0;
  for (i=0; i<(byteCount-1); i++) {
    xo = xo ^ (*dataString);
    dataString ++;
  }
  *dataString = xo;
}

////////////////////////////////////////////////////////////////////
// on a catastrophic error, loop until reset, flashing...
void loopErrorToLED(int pattern) {
  int i;

  #ifdef VERBOSE
  Serial.print ("loopErrorToLED, pattern "); Serial.println (pattern);
  #endif
  
  for (;;) {
    for (i=0; i<pattern; i++) {
      LED_ON;
      delay (200);
      LED_OFF;
      delay (200);
    }
    delay (2000);
  }
}


////////////////////////////////////////////////////////////////////
// Our XpressNet Window is open. Do we have anything to send?
void handleCallByteInquiry () { 
  if (initializationStep != PAST_INITIALIZATION) {
    //Serial.print ("handleCallByte, in initialization, step "); Serial.println (initializationStep);
    
    if ((initializationStep == GET_COMMAND_STATION_STATUS_STEP) ||
       (initializationStep == WAIT_FOR_COMMAND_STATION_POWERUP)) {
      // get the Command Station status 
      sendCommand (commandStatusSequence, 3);    

      // if we are in the waiting for power up, lets assume we are powered up
      // unless told so AGAIN.
      if (initializationStep == WAIT_FOR_COMMAND_STATION_POWERUP) {
        initializationStep = GET_COMMAND_STATION_STATUS_STEP;
      }
      
    } else if (initializationStep == GET_COMMAND_STATION_SW_VERSION_STEP) {
      // get the Command Station sw version 
      sendCommand (commandSWVersionSequence,3);
      
    } else if (initializationStep == GET_LOCOMOTIVE_INFO_STEP) {
      // get the info for locomotive in LOCO_ID
      sendCommand ( locoInfoRequest,5);
      
    } else if (initializationStep == LOCOMOTIVE_NOT_FREE) {
      // we have an error - for instance, the locomotive is in an MU, and
      // we are not going to break this MU here.
     
      loopErrorToLED(initializeError);
    }
    
    
 } else {
   // WE ARE RUNNING, AND OUR WINDOW IS OPEN! YEEEEHAH!
   // anything changed?
   if (speedControlsChanged) {
    sendCommand (locoSpeedOps, 6);
    speedControlsChanged = false;
    
   } else if (lightingChanged) {
    sendCommand (locoFunctionSet, 6);
    lightingChanged = false;
   } else if (setESTOP) {
     sendCommand(EST,3);
     setESTOP = false;
   } else if (clearESTOP) {
     sendCommand (ESRES,3);
     clearESTOP = false;
   }
     // can put in more else if bits here if there are other things like sound
     // commands to send along.
  }
}

////////////////////////////////////////////////////////////////////
// we have data destined to ALL XpressNet devices; track power off, etc...
void handleGeneralBroadcast () {
 int i;

 if (dataSaved == 4) {
   if (savedData[1] == 0x61) {
     if ((savedData[2] == 0x01) && (savedData[3] == 0x60)) {
       // Normal Operation Resumed
       currentState = STATE_NORMAL_OPS;
       return;
       
     } else if ((savedData[2] == 0x00) && (savedData[3] == 0x61)) {
       // Track power off
       currentState = STATE_POWER_OFF;
       return;

     } else if ((savedData[2] == 0x02) && (savedData[3] == 0x63)) {
       // Service Mode Entry
       currentState = STATE_SERVICE_MODE;
       return;

     }
   } else if (savedData[1] == 0x81) {
     if ((savedData[2] == 0x00) && (savedData[3] == 0x81)) {
       currentState = STATE_ESTOP;
       return;
     }
   }
 }

 // There was an error here...

#ifdef VERBOSE
Serial.println("Broadcast error ");
 for (i=0; i<dataSaved; i++) {
   Serial.print(savedData[i],HEX); Serial.write(' ');
 }
 Serial.println();
 #endif
}



////////////////////////////////////////////////////////////////////
// Look at the XpressNet documentation, table 3.
// DATA_INDICATOR is the "header" field;
// dataBytes are the "Data Byte 1..." fields from
// this table.

void handleDirectedOps () {
  int i;
  //unsigned char headerNibbleHigh;
  //unsigned char headerNibbleLow;
  unsigned char dataByte1;
  unsigned char dataByte2;
  #define DATA_INDICATOR savedData[1]
  
  if (dataSaved >= 2) {
     //headerNibbleLow = savedData[1] & 0x0f;
     //headerNibbleHigh = savedData[1] & 0xf0;
     dataByte1 = savedData[2];
     dataByte2 = savedData[3];

     // from the spec:
     // SW Version X-Bus V2 and V2 (0x21)
     // Command Station status indication response (0x22)
     //   
     if (DATA_INDICATOR == 0x62) {
         if (dataByte1 == 0x21) {
            // old version - version 1 and version 2 response.
           loopErrorToLED(5);
           
         } else if (dataByte1 == 0x22) {
           //Serial.print ("62 com stat indication response data: "); Serial.println (dataByte2, HEX);
           // look at the spec - section 2.1.7 for bits;
           // 0 = 1 - emergency off;
           // 1 = 1 - emergency stop;
           // 2 = 1 - automatic mode, =1, manual mode;
           // 3 = 1 - in service mode;
           // 6 = 1 - in power up;
           // 7 = 1 - RAM check error

           //emergencyStop = false;
           //trackPowerOff = false;
           //inServiceMode = false;
           currentState = STATE_NORMAL_OPS;
 
           if (dataByte2 != 0) {
             // is track power turned off?
             if ((dataByte2 & 0x01) == 0x01) {currentState=STATE_POWER_OFF;}
             
             // is it in emergency stop?
             if ((dataByte2 & 0x02) == 0x02) {currentState=STATE_ESTOP;}
             
             // in service mode?
             if ((dataByte2 & 0x08) == 0x08) {currentState=STATE_SERVICE_MODE;}             
             
             // in powerup mode - wait until complete
             if ((dataByte2 & 0x40) == 0x40) {
               // put us in a state where we do the status request again...
               initializationStep = WAIT_FOR_COMMAND_STATION_POWERUP;
             }
           }
           // if we are powered up, and 
           // We do this as part of the initialization; go to next step
           if (initializationStep == GET_COMMAND_STATION_STATUS_STEP) {
             //Serial.println ("got command station status step, going to sw version");
             initializationStep = GET_COMMAND_STATION_SW_VERSION_STEP;
           }
         } // end of dataByte test
         
     } else if (DATA_INDICATOR == 0xE2) {
        // Xpressnet 2.1.14.3 - loco info for the MU address...
         
       // ok - we have info for this guy in initialization.
       if (initializationStep == GET_LOCOMOTIVE_INFO_STEP) {
        initializationStep = LOCOMOTIVE_NOT_FREE;
        initializeError = 2;
       }
       
     } else if (DATA_INDICATOR  == 0x63) {
           if (dataByte1 == 0x21) {
             /* Serial.print ("sw ver, xpressnet "); Serial.print (savedData[2],HEX); Serial.print ("  "); 
             Serial.print (savedData[3],HEX); Serial.print ("  "); 
             Serial.println (savedData[4],HEX);
             */
             
             // we need to be version 3.0 and above; 3.0 is 0x30 in hex.
             if (savedData[3] < 0x30) loopErrorToLED(5);
             
             // We do this as part of the initialization; go to next step
             if (initializationStep == GET_COMMAND_STATION_SW_VERSION_STEP) {
               //Serial.println ("got sw version, go for locomotive info");
               initializationStep = GET_LOCOMOTIVE_INFO_STEP;
             }
           }
           
       } else if (DATA_INDICATOR == 0xE3) {
         if (dataByte1 == 0x40) {
           /* Xpressnet 3.6; section 2.1.13.2; "This response is also sent unrequested,
              to the X-Bus device that is currently controlling the locomotive"
              
              Thus, getting this message is ok. */

           #ifdef VERBOSE       
           Serial.print ("E3!! loco operated by another device ");
           Serial.print (dataByte2,HEX); Serial.print (" ");
           Serial.println (savedData[4],HEX);
           #endif //verbose
         }
         
     }else if (DATA_INDICATOR == 0xE4) {   
         // Xpressnet 2.1.14.1 - locomotive info normal locomotive
         // lets look at the loco ID byte:
         if ((dataByte1 & 0x08) == 0x08) {
           #ifdef VERBOSE
           Serial.println ("E4!! locomotive is being controlled by someone else");
           #endif //VERBOSE
           loopErrorToLED(7);
         }
        
         #ifdef VERBOSE
         Serial.print ("0xE4 - byte "); Serial.println (dataByte1,HEX);
         #endif //VERBOSE
         
         if ((dataByte1 & 0x07) == 0x00) {
           decoderSpeedSteps = 14; 
           locoSpeedOps[1] = 0x10;
         } else if ((dataByte1 & 0x07) == 0x01) {
           decoderSpeedSteps = 27;
           locoSpeedOps[1] = 0x11;
         } else if ((dataByte1 & 0x07) == 0x02) {
           decoderSpeedSteps = 28;
           locoSpeedOps[1] = 0x12;
         } else if ((dataByte1 & 0x07) == 0x04) {
           decoderSpeedSteps = 128;
           locoSpeedOps[1] = 0x13;
         } else {
            // we don't do this speed steps - something wrong here
           loopErrorToLED(4);           
         }
                  
         //Serial.print ("st: "); Serial.println (decoderSpeedSteps);
         if ((savedData[3] & 0xff) == 0) inNeutral = true;
         if ((savedData[3] & 0x80) == 0x80) runningForward = true; 
         else runningForward = false;
         
         /* functionGroupB = savedData[5]; */
         /* currentLocoSpeed = savedData[3]; */

         // Save the functions 0-4, so we can adjust them as we see fit; they
         // are in the order 0 0 0 F0  F4 F3 F2 F1
         functionGroupA = savedData[4];
         headlightOn =  ((functionGroupA & 0xF0) == 0x10);      
         
         // ok - we have info for this guy in initialization.
         if (initializationStep == GET_LOCOMOTIVE_INFO_STEP) {
          initializationStep = PAST_INITIALIZATION; 
         }

     }else if (DATA_INDICATOR == 0xE5) {
         // Xpressnet 2.1.14.2 LOCO is part of an MU!
         
         // ok - we have info for this guy in initialization.
         if (initializationStep == GET_LOCOMOTIVE_INFO_STEP) {
          initializationStep = LOCOMOTIVE_NOT_FREE;
          initializeError = 3;
         }
       

     }else if (DATA_INDICATOR == 0xE6) {

         // Xpressnet 2.1.14.4 - loco info for double header
         // ok - we have info for this guy in initialization.
         if (initializationStep == GET_LOCOMOTIVE_INFO_STEP) {
           initializeError = 1;
          initializationStep = LOCOMOTIVE_NOT_FREE; 
         }
    }
  } // dataSaved >= 2
}


////////////////////////////////////////////////////////////////////
void handleSavedData () {
 // Is this a general broadcast to everyone?
 if (*savedData == GENERAL_BROADCAST) {
   handleGeneralBroadcast();
 } else if (*savedData == myDirectedOps) {
   handleDirectedOps();
 #ifdef VERBOSE  
 } else {
   // else just ignore it for now.
   Serial.write ('?');
 #endif
 }
}


////////////////////////////////////////////////////////////////////
// flash the LED if we are in power-off mode      
void handle_possible_track_power_indications () {    
 // what should we do with the LED pin?

 if (currentState != STATE_NORMAL_OPS) {
   // let the command station give us our clock; we count
   // "ticks" here.
   currentFlashCounter++;
   if (currentFlashCounter >= flashInterval) {
     currentFlashCounter = 0;

     // flip the state.
     displayLEDState = !displayLEDState;
     if (displayLEDState) {LED_ON;} else {LED_OFF;}

   }
  } else {
    // we are in running mode...
    LED_ON;
  }     
}

////////////////////////////////////////////////////////////////////
// read the potentiometer and switches once every so many xpressnet "ticks"
void read_pots_and_switches() {

  analogueReadCounter ++;
  if (analogueReadCounter == 20) {
    // read the switches for direction change
    int tmp;
    tmp = 0;
    if (digitalRead(FORWARD_SWITCH)) tmp += 1;
    if (digitalRead(REVERSE_SWITCH)) tmp += 2;
    // 3 = neutral, 1 == forward, 2 == reverse 0 == invalid


    switch (tmp) {
      // neutral
      case 0:
      case 3:
        if (!inNeutral) speedControlsChanged = true;
        inNeutral = true;
        break;
      // forward
      case 1:
        inNeutral = false;
        // was last movement reverse?
        if (!runningForward) speedControlsChanged = true;
        runningForward = true;
        break;
      case 2:
      // reverse
        inNeutral = false;
        // was last movement forward?
        if (runningForward) speedControlsChanged = true;
        runningForward = false;
        break;
    }
    
  } else if (analogueReadCounter == 30) {
    // read the speed for speed changes
    
    // reset the counter.
    analogueReadCounter = 0;
    potValue = map(analogRead(POT_PIN),0,1023,0,decoderSpeedSteps);
   
    // set to zero speed when direction switch set to neutral
    if (inNeutral) {
          potValue = 0;
    }
        
    if (potValue != oldPotValue) {
      //Serial.print ("pot: "); Serial.println (potValue);
      oldPotValue = potValue;
      speedControlsChanged = true;
      lightOffAt = 0;
    }
  }

  // did the direction switch or pot change?
  if (speedControlsChanged) {
        calculateSpeedDirection();
  }
}

////////////////////////////////////////////////////////////////////
void calculateSpeedDirection() {

    //Serial.print ("direction: "); Serial.println (inNeutral);
    //Serial.print ("pot: "); Serial.println (potValue);
    
    // note that the header byte, identification, AH, AL are set previously
    // See Xpressnet 2.2.20.3
    
    encodedSpeedValue = 0;
    if (decoderSpeedSteps == 28) {
      // from xpressnet 2.1.13.1
      
      if (currentState != STATE_NORMAL_OPS) {
       encodedSpeedValue = 0x01; 
      } else {

        // potValue has been mapped to the range [0-28] via the map function.
        // thus, bounds checking for the following lookup not required.
        
        encodedSpeedValue = twentyEightTable[potValue];
      }   
      
    } else if (decoderSpeedSteps == 128) {
      // from xpressnet 2.1.14.1
      // note that this is from 0 to 127; 127 and 128 we count the same.
      // note that 0 = stop, 1 = emergency stop, that we do not handle yet.
      if (potValue >= 128) {encodedSpeedValue = 0x7f; } 
      else encodedSpeedValue = potValue;
      
      // for now, we do not have an emergency stop button, so we set the
      // speed to zero, and let the locomotive slow down via momentum settings.
      if (encodedSpeedValue == 1) { encodedSpeedValue = 0; } 
      
    } else if (decoderSpeedSteps == 14) {
      // from xpresset 2.1.12.1
      // 0 = stop
      // 1 = emergency stop
      // 2=15 running values
      if (potValue == 0) { encodedSpeedValue = 0;}
      else if (potValue==1) {encodedSpeedValue = 0;}
      else {
         encodedSpeedValue = potValue + 1;
      }
      
    } else if (decoderSpeedSteps == 27) {
            // from xpresset 2.1.12.1
      // 0 = stop
      // 1 = emergency stop
      // 2=15 running values
      if (potValue == 0) { encodedSpeedValue = 0;}
      else if (potValue==1) {encodedSpeedValue = 0;}
      else {
         encodedSpeedValue = potValue + 1;
      }
    } else {
      #ifdef VERBOSE
      Serial.print("huh - speed steps"); Serial.println (decoderSpeedSteps);
      #endif
      // we do not support this speed step; we should have caught this here
      loopErrorToLED(6);  
    }
    
    // do the direction
    if (runningForward) {
      encodedSpeedValue |= 0x80; // set the direction bit
    }        
        
    // fill in the actual value, and send it along
    locoSpeedOps[4] = encodedSpeedValue;
          
    // send the speed to the command station
    calculateXOR (locoSpeedOps, 6);
}

////////////////////////////////////////////////////////////////////
// we turn the headlight on when we are moving, off when we are not
void verifyHeadlightIsOK () {
    if (analogueReadCounter == 20) {
//  Serial.print("HOK "); 
//  Serial.print (encodedSpeedValue); Serial.print(" ")Serial.println(encodedSpeedValue & 0x7f,HEX);
  
      /* should we ensure that the headlight is on? */
      if (((encodedSpeedValue & 0x7f) != 0) && (!headlightOn)) {
        functionGroupA |= 0x10; //high nibble, bit zero is F0 
        locoFunctionSet[4] = functionGroupA;
        headlightOn = true;
        lightOffAt = 0;
        calculateXOR(locoFunctionSet, 6);
        lightingChanged = true;
      }  
            
      /* should we ensure that the headlight is off? */
    if (((encodedSpeedValue & 0x7f) == 0) && (headlightOn)) {
      if (lightOffAt == 0) {
        lightOffAt = millis() + LIGHT_OFF_DELAY;
      }
    }  

  
    if (lightOffAt != 0) {
      if (millis()>lightOffAt) {
          functionGroupA &= 0xEF; //high nibble, bit zero is F0 
          locoFunctionSet[4] = functionGroupA;
          headlightOn = false;
          calculateXOR(locoFunctionSet, 6);
          lightingChanged = true;   
          lightOffAt = 0; 
      }
    }
  }
}

 // ESTOP button
 void handle_ESTOP_button_press() {
  if (!digitalRead(ESTOP_BUTTON)) {
    //pressed yet?
    if (pt==0) {
      // get the time pressed
      pt = millis();
      //tell the system to toggle the ESTOP state
      if (currentState==STATE_NORMAL_OPS) {
        setESTOP = true;
      } else if ((currentState == STATE_ESTOP) ||
                  (currentState == STATE_POWER_OFF)) {
        clearESTOP = true;
      } else {
        #ifdef VERBOSE
        Serial.println ("hmmm how to handle currentState " + String(currentState));
        #endif //VERBOSE
      } 
      //Serial.println ("BUTPRES "+String(currentState));
    }
    // finished debouncing? 
  } else {
    rt=millis();
    if ((pt+pressPeriod+1000) < rt) {
      if (pt != 0) {
        #ifdef VERBOSE
        Serial.println ("button release"); 
        #endif //VERBOSE        
        pt = 0;
      }
         
    }
  }  
}

////////////////////////////////////////////////////////////////////
void loop() {
  
  // get the byte from the XPressnet bus, if there is one.
  int rxdata = USART_Receive();
 
  // is the data ok?
  if (rxdata != -1) {

     // This IS a Call Byte
     if (rxdata >= CALL_BYTE) {

       // is this OUR XpressNet window opening? If so, do we have
       // anything to send?
       
       if (rxdata == myCallByteInquiry) {
         handleCallByteInquiry();
         
       } else if (rxdata == myRequestAck) {
          sendCommand (requestAckAck,2);
       }
       
       // do we have saved data from an old message to me?
       if (dataSaved > 1) {
          handleSavedData();
          dataSaved = 0;
       }
       // is this the start of data coming in for me?  

       if ((rxdata == GENERAL_BROADCAST) ||
          (rxdata == myDirectedOps)) {
            
          savedData[0] = rxdata;
          dataSaved=1;
       
       } 
       
       // if not - maybe the call to another window? we just ignore it

     } else {
      //Serial.println("CB");
       // if savedData[0] is for us, we record rest of packet.
       if (dataSaved !=0) {
        // record this data for later processing
        if (dataSaved < MAXSAVED) {
          savedData[dataSaved] = rxdata;
          dataSaved++;
        }
       }
     }
     
 #ifdef VERBOSE    
 } else {
   // we have a data error... Maybe 2 throttles on same
   // Xpressnet address
   Serial.println("DATA ERROR");
 #endif
 }

 // ok - now that we have done this, lets make sure we 
 if (initializationStep == PAST_INITIALIZATION) {
   
  handle_ESTOP_button_press();
   
   handle_possible_track_power_indications ();

   read_pots_and_switches();
      
   verifyHeadlightIsOK();
 }
 
}
