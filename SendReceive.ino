//——————————————————————————————————————————————————————————————————————————————
//  IFCT(Improved FlexCAN Teensy) to configure CANBUS for Megasquirt, https://github.com/tonton81/IFCT
//——————————————————————————————————————————————————————————————————————————————

//Error checking, remove comment to output serial debug
//#define ERRORCHECK
//Convert, to output data conversion from Megasquirt 29bit CANBUS
//#define CONVERT
//SEND, to send request for data TO megasquirt
//#define SEND
#include <IFCT.h>

//——————————————————————————————————————————————————————————————————————————————
// Global Variables
// 
// 
//——————————————————————————————————————————————————————————————————————————————
static const byte controllerID = 8;// controllerID
static CAN_message_t outMsg, inMsg;
uint32_t prevmillis = 0;
uint32_t currentmillis = 0;

//——————————————————————————————————————————————————————————————————————————————
//  
//——————————————————————————————————————————————————————————————————————————————


//——————————————————————————————————————————————————————————————————————————————
//  
//——————————————————————————————————————————————————————————————————————————————


//——————————————————————————————————————————————————————————————————————————————
//   SETUP
//——————————————————————————————————————————————————————————————————————————————

void setup () {
//--- Configure ACAN2515
  Serial.println ("Configure IFCT") ;
  Can0.setBaudRate(500000); // CAN bit rate 500 kb/s
  Can0.enableFIFO();//set mode to First in First out (no mailboxes)
  Can0.enableFIFOInterrupt();//set mode to flag interrupt routine on message receipt
  Can0.onReceive(can0ISR);//fill with ISR
  Can0.intervalTimer();//enable queue system and run callback in background (no idea what this does)
}


// ISR
//——————————————————————————————————————————————————————————————————————————————
void can0ISR(const CAN_message_t &inMsg)
{//global callback
	byte fromID = 0x0;			//0-14 allowed
	byte toID = 0x0;			//0-14 allowed
	uint8_t type = 0x00;		//0-14, 0x80, 0x81, and 0x82 allowed
	uint8_t table = 0x00;		//0-31, 0xF0-0xF8 allowed
	uint8_t requestTable = 0x00;
	uint16_t offset = 0x0000;	//0-2056 allowed dependant on Table
	uint16_t requestOffset = 0x0000;
	byte requestLen = 0x0;
	char output[9] = ""; 


	fromID = getFromID(inMsg.id);
	toID = getToID(inMsg.id);
	table = getTable(inMsg.id);
	type = getType(inMsg.id);
	offset = getOffset(inMsg.id);
	
	Serial.print ("Received: ") ;
	Serial.print (toID, HEX) ;
	Serial.print (" [");
	Serial.print (inMsg.len, DEC);
	Serial.print ("] ");
	for (x=0; x<inMsg.len; x++){
		
		sprintf(output, "%02X", inMsg.buf[x]);
		Serial.print(output);
		Serial.print (" ");			
		}
	Serial.println ("");
	if (type == 1)
	{
		requestTable = getRequestTable(inMsg.buf[0]);
		requestOffset = getRequestOffset(inMsg.buf[1], inMsg.buf[2]);
		requestLen = getRequestLength(inMsg.buf[2]);
		
		if(toID == controllerID)
		{
			if (respondToRequest(requestTable, requestOffset, requestLen, fromID) == 1)
			{
				Serial.println ("Reply Success");
			}
			else
			{
				Serial.println("Reply Failed");
			}
		}
	}
	#ifdef CONVERT
		Serial.print("Oneline_f:");
		Serial.print(fromID);
		Serial.print("_t:");
		Serial.print(toID);
		Serial.print("_table:");
		Serial.print(table);
		Serial.print("_type:");
		Serial.print(type);
		Serial.print("_Offset:");
		Serial.println(offset);
		if(type == 1)
		{
			Serial.print("reply to:table:");
			Serial.print(requestTable);
			Serial.print("_offset:");
			Serial.print(requestOffset);
			Serial.print("_length:");
			Serial.println(requestLen);
		}
		
	#endif

}

uint32_t buildCANID(byte fromID, byte toID, uint8_t type, uint8_t table, uint16_t offset) 
{
	uint32_t canSendID = 0x00000000;
	uint32_t tempCanSendID = 0x00000000;
	
	tempCanSendID = offset;
	tempCanSendID = tempCanSendID << 18;//put offset in position in tempCanSendID
	
	#ifdef ERRORCHECK
		Serial.print("tempCanSendID offset shifted:");
		Serial.println(tempCanSendID, HEX);
	#endif
	
	canSendID = canSendID | tempCanSendID;//drop temp into position in canSendID
	
	tempCanSendID = 0x00000000;//clear tempCanSendID
	tempCanSendID = type;//put type in position in tempCanSendID
	tempCanSendID = tempCanSendID << 15;//trial
	
	#ifdef ERRORCHECK
		Serial.print("tempCanSendID type shifted:");
		Serial.println(tempCanSendID, HEX);
	#endif
	
	canSendID = canSendID | tempCanSendID;//drop temp into position in canSendID
	
	tempCanSendID = 0x00000000;//clear tempCanSendID
	tempCanSendID = fromID;
	tempCanSendID = tempCanSendID << 11;//put fromID in position in tempCanSendID
	
	#ifdef ERRORCHECK
		Serial.print("tempCanSendID fromID shifted:");
		Serial.println(tempCanSendID, HEX);
	#endif
	
	canSendID = canSendID | tempCanSendID;//drop temp into position in canSendID
	
	tempCanSendID = 0x00000000;//clear tempCanSendID
	tempCanSendID = toID;
	tempCanSendID = tempCanSendID << 7;//put toID in position in tempCanSendID
	
	#ifdef ERRORCHECK
		Serial.print("tempCanSendID toID shifted:");
		Serial.println(tempCanSendID, HEX);
	#endif
	
	canSendID = canSendID | tempCanSendID;//drop temp into position in canSendID
	
	tempCanSendID = 0x00000000;//clear tempCanSendID
	tempCanSendID = table & 0x000F;//bitwiseand with 0x000F to get last 4 bytes into tempCanSendID
	table = table >> 4;//bitshift table right 4 to get 5th bit to first bit
	tempCanSendID = tempCanSendID << 1;//bitshift tempCanSendID left one to free up first bit
	tempCanSendID = tempCanSendID | table;//bitwiseor table with tempCanSendID to combine
	tempCanSendID = tempCanSendID << 2;//bitshift tempCanSendID left 2 for final position
	
	#ifdef ERRORCHECK
		Serial.print("tempCanSendID Table shifted:");
		Serial.println(tempCanSendID, HEX);
	#endif
	
	canSendID = canSendID | tempCanSendID;//drop temp into position in canSendID
	
	return canSendID;
}

uint8_t getFromID(uint32_t canReceiveID)
{
	uint8_t fromID = 0x00;
	
	canReceiveID = canReceiveID >> 11;//bitshift ID Right by 11
	fromID = canReceiveID & 0x0F;//bitwiseand with 0x0F to get last 4 bits into fromID
	
	return fromID;
}

uint8_t getToID(uint32_t canReceiveID)
{
	uint8_t toID = 0x00;
	
	canReceiveID = canReceiveID >> 7;//bitshift ID Right by 7
	toID = canReceiveID & 0x0F;//bitwiseand with 0x0F to get last 4 bits into toID
	
	return toID;
}

uint8_t getType(uint32_t canReceiveID)
{
	uint8_t type = 0x00;
	
	canReceiveID = canReceiveID >> 15;//bitshift ID Right by 15
	type = canReceiveID & 0x07;//bitwiseand with 0x07 to get last 3 bits into type
	
	return type;
}

uint8_t getTable(uint32_t canReceiveID)
{
	uint8_t table = 0x00;
	uint8_t temp = 0x00;
	
	canReceiveID = canReceiveID >> 2;//bithsift ID right by 2
	temp = canReceiveID & 0x01;//bitwiseand with 0x01 to get 5th bit from right of Table
	canReceiveID = canReceiveID >> 1;//bitshift ID right by 1
	table = canReceiveID & 0x0F;//bitwiseand with 0x0F to get last 4 bits of Table
	temp = temp << 4;//bitshift temp left by 4 to place into 5th bit
	table = table | temp;//bitwiseor table and temp to merge temp/table into final
	
	return table;
}

uint16_t getOffset(uint32_t canReceiveID)
{
	uint16_t offset = 0x0000;
	
	canReceiveID = canReceiveID >> 18;//bitshift ID Right by 15
	offset = canReceiveID & 0x07FF;//bitwiseand with 0x07FF to get last 11 bits into offset
	
	return offset;
}
uint8_t getRequestTable(uint8_t data0)
{
	uint8_t requestTable = 0x00;
	#ifdef ERRORCHECK
		Serial.print("RequestTableBinary:");
		Serial.println(data0, BIN);
	#endif
	data0 = data0 & 0x1F;//bitwiseand with 0x1F to get last 5 bits
	requestTable = data0;
	return requestTable;
}

uint16_t getRequestOffset(uint8_t data1, uint8_t data2)
{
	uint16_t requestOffset = 0x0000;
	#ifdef ERRORCHECK
		Serial.print("RequestOffsetBinary:");
		Serial.println(data1, BIN);
		Serial.println(data2, BIN);
	#endif
	requestOffset = data1 << 8;//bitshift left by 8
	requestOffset = requestOffset | data2;
	requestOffset = requestOffset >> 5;//bitshift to right to move offset to right
	requestOffset = requestOffset & 0x07FF;//bitwiseand with 0x07FF to get last 11 bits
	return requestOffset;
}

byte getRequestLength(uint8_t data2)
{
	byte requestLen = 0x0;
	#ifdef ERRORCHECK
		Serial.print("RequestLengthBinary:");
		Serial.println(data2, BIN);
	#endif
	data2 = data2 & 0x0F;//bitwiseand with 0x0F to get last 4 bits
	requestLen = data2;//drop data to length
	return requestLen;
}

uint32_t buildRequestData(uint8_t requestTable, uint16_t requestOffset, byte requestLen)
{	//NOTE!!!! data must be inverted (first byte to send needs to be last) in order to work correctly with 
	//data32[x] array in ACAN2515 Library
	uint32_t data32 = 0x0000000000000000;
	uint32_t temp = 0x0000000000000000;
	
	//first byte
	temp = 0x0000000000000000;
	temp = requestOffset;
	temp = temp << 5;
	temp = temp & 0x00000000000000E0;//keep the first 3 digits in last byte
	temp = temp | requestLen;//add request Length. 
	temp = temp << 16;//Bitshift left to first byte
	data32 =  temp;
	
	//second byte
	temp = 0x0000000000000000;
	temp = requestOffset;
	temp = temp >> 3; //bitshift right 3 to remove 3 bits in previous byte
	temp = temp << 8; //bitshift left 8 to second byte
	data32 = data32 | temp;
	
	//third byte
	temp = 0x0000000000000000;
	temp = requestTable;
	data32 = data32 | temp;
	
	#ifdef ERRORCHECK
		Serial.print("RequestData:");
		Serial.println(data32, BIN);
	#endif
	return data32;
}

bool respondToRequest(uint8_t requestTable, uint16_t requestOffset, byte requestLen, byte fromID)
{
	static uint32_t egtholder;
	static int x;
	uint32_t temp = 0;
	if (x==1)
	{
		egtholder = egtholder + 1;
		if (egtholder >= 4094)
		{
			x = 0;
		}
	}
	else
	{
		egtholder = egtholder - 1;
		if (egtholder <= 1)
		{
			x = 1;
		}
	}
	CANMessage rspFrame;//setup message frame
	rspFrame.len = requestLen;//put request length as response length
	rspFrame.ext = true; //set Extended bit.
	//uint32_t buildCANID(byte fromID, byte toID, uint8_t type, uint8_t table, uint16_t offset) 
	rspFrame.id = buildCANID(controllerID, fromID, 2, requestTable, requestOffset);
	rspFrame.data[1] = egtholder & 0x00FF;
	temp = egtholder & 0xFF00;
	temp = temp >> 16;
	rspFrame.data[0] = temp;
	#ifdef CONVERT
		Serial.print("egtholder:");
		Serial.println(egtholder);
	#endif
	
	return can.tryToSend (rspFrame);
}
void loop () {
	byte fromID = 0x0;			//0-14 allowed
	byte toID = 0x0;			//0-14 allowed
	uint8_t type = 0x00;		//0-14, 0x80, 0x81, and 0x82 allowed
	uint8_t table = 0x00;		//0-31, 0xF0-0xF8 allowed
	uint8_t requestTable = 0x00;
	uint16_t offset = 0x0000;	//0-2056 allowed dependant on Table
	uint16_t requestOffset = 0x0000;
	byte len = 0x0;				//0-7 allowed
	byte requestLen = 0x0;
	uint8_t data[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //0x0-0x8 allowed for all
	uint32_t canSendID = 0x00000000; 
	uint32_t requestData = 0x00000000;

	int x = 0;
	char output[4] = ""; 
	uint16_t sendtime = 1000; //time in ms for sending
	  
	currentmillis = millis();
	  
	fromID = 8;
	toID = 0;
	type = 1;
	table = 7;
	requestTable = 14;
	offset = 66;//Fuel load (kpa)
	requestOffset = 22;
	requestLen = 2;
	
	
	#ifdef SEND
		//uint32_t buildCANID(byte fromID, byte toID, uint8_t type, uint8_t table, uint16_t offset)
		canSendID = buildCANID(fromID, toID, type, table, offset);
		//uint32_t buildRequestData(uint8_t requestTable, uint16_t requestOffset, byte requestLen)
		requestData = buildRequestData(requestTable, requestOffset, requestLen);
		outMsg.buf[0] = requestData >> 24;
		outMsg.buf[1] = requestData >> 16;
		outMsg.buf[2] = requestData >> 8;
		outMsg.buf[3] = requestData;
		
		outMsg.ext = true; //set Extended bit. 
		outMsg.id = canSendID; //write canSendID to frame
		outMsg.len = 3; // 3 Long
		
		
		if ((currentmillis - prevmillis) >= sendtime) 
		{		  
			prevmillis = currentmillis;
			
			const bool ok = can.tryToSend (frame) ;
			if (ok) {
				Serial.print ("Sent:     ") ;
				Serial.print (outMsg.id, HEX);
				Serial.print (" [");
				Serial.print (outMsg.len, DEC);
				Serial.print ("] ");
				for (x=0; x<outMsg.len; x++){
					sprintf(output, "%02X", outMsg.buf[x]);
					Serial.print(output);
					//Serial.print (frame.data[x], HEX);
					Serial.print (" ");
				}
				Serial.println ("");
				#ifdef CONVERT
					Serial.print("sending_f:");
					Serial.print(fromID);
					Serial.print("_t:");
					Serial.print(toID);
					Serial.print("_table:");
					Serial.print(table);
					Serial.print("_type:");
					Serial.print(type);
					Serial.print("_Offset:");
					Serial.print(offset);
					Serial.print("_ReturnTable:");
					Serial.print(requestTable);
					Serial.print("_ReturnOffset:");
					Serial.print(requestOffset);
					Serial.print("_Ret.DataLength:");
					Serial.println(requestLen);
				#endif
			}else{
				Serial.println ("Send failure") ;
			}
		}
	#endif
	
}

//——————————————————————————————————————————————————————————————————————————————
