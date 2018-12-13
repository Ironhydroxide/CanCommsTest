//——————————————————————————————————————————————————————————————————————————————
//  Using ACAN2515 library for CAN comms.
//——————————————————————————————————————————————————————————————————————————————

#include <ACAN2515.h>

//——————————————————————————————————————————————————————————————————————————————
// If you use CAN-BUS shield (http://wiki.seeedstudio.com/CAN-BUS_Shield_V2.0/) with Arduino Uno,
// use B connections for MISO, MOSI, SCK, #10 for CS,
// #2 for INT.
//——————————————————————————————————————————————————————————————————————————————

static const byte MCP2515_CS  = 10 ; // CS input of MCP2515 (adapt to your design) 
static const byte MCP2515_INT =  2 ; // INT output of MCP2515 (adapt to your design)

uint32_t prevmillis = 0;
uint32_t currentmillis = 0;

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2515 can (MCP2515_CS, SPI, MCP2515_INT) ;

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Quartz: adapt to your design
//——————————————————————————————————————————————————————————————————————————————

static const uint32_t QUARTZ_FREQUENCY = 16UL * 1000UL * 1000UL ; // 16 MHz

//——————————————————————————————————————————————————————————————————————————————
//   SETUP
//——————————————————————————————————————————————————————————————————————————————

void setup () {
//--- Start serial
  Serial.begin (115200) ;
//--- Wait for serial
  while (!Serial) {
  }
//--- Begin SPI
  SPI.begin () ;
//--- Configure ACAN2515
  Serial.println ("Configure ACAN2515") ;
  ACAN2515Settings settings (QUARTZ_FREQUENCY, 500UL * 1000UL) ; // CAN bit rate 500 kb/s
  settings.mRequestedMode = ACAN2515RequestedMode::NormalMode ; // Select normal mode
  const uint32_t errorCode = can.begin (settings, [] { can.isr () ; }) ;
  if (errorCode == 0) {
    Serial.print ("Bit Rate prescaler: ") ;
    Serial.println (settings.mBitRatePrescaler) ;
    Serial.print ("Propagation Segment: ") ;
    Serial.println (settings.mPropagationSegment) ;
    Serial.print ("Phase segment 1: ") ;
    Serial.println (settings.mPhaseSegment1) ;
    Serial.print ("Phase segment 2: ") ;
    Serial.println (settings.mPhaseSegment2) ;
    Serial.print ("SJW:") ;
    Serial.println (settings.mSJW) ;
    Serial.print ("Triple Sampling: ") ;
    Serial.println (settings.mTripleSampling ? "yes" : "no") ;
    Serial.print ("Actual bit rate: ") ;
    Serial.print (settings.actualBitRate ()) ;
    Serial.println (" bit/s") ;
    Serial.print ("Exact bit rate ? ") ;
    Serial.println (settings.exactBitRate () ? "yes" : "no") ;
    Serial.print ("Sample point: ") ;
    Serial.print (settings.samplePointFromBitStart ()) ;
    Serial.println ("%") ;
  }else{
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//——————————————————————————————————————————————————————————————————————————————————————————————————————————————————————

static unsigned gBlinkLedDate = 0 ;
static unsigned gReceivedFrameCount = 0 ;
static unsigned gSentFrameCount = 0 ;

//——————————————————————————————————————————————————————————————————————————————

void loop () {
int x = 0;
char output[4] = ""; 
uint16_t sendtime = 1000; //time in ms for sending
  CANMessage frame ;
  
  currentmillis = millis();
  
  frame.ext = true; //set Extended bit. 
  frame.id = 0x0050F038; //From 14, to 0, type 1, table 7, offset 20 (IAT)
  frame.len = 0x03; // 3 Long
  frame.data64 = 0xC1020E; //Block14, Offset22, Byte4
  
  if ((currentmillis - prevmillis) >= sendtime) {
  
  	prevmillis = currentmillis;
	
    const bool ok = can.tryToSend (frame) ;
    if (ok) {
      Serial.print ("Sent: Frame ") ;
	  Serial.println (frame.id, HEX);
    }else{
      Serial.println ("Send failure") ;
    }

  }
  if (can.receive (frame)) {
    gReceivedFrameCount ++ ;
    Serial.print ("Received: ") ;
    Serial.print (frame.id, HEX) ;
	Serial.print (" [");
	Serial.print (frame.len, DEC);
	Serial.print ("] ");
	for (x=0; x<frame.len; x++){
		
		sprintf(output, "%02X", frame.data[x]);
		Serial.print(output);
		//Serial.print (frame.data[x], HEX);

		Serial.print (" ");
		
		
		}
		
	Serial.println ("");
	

  }
}

//——————————————————————————————————————————————————————————————————————————————