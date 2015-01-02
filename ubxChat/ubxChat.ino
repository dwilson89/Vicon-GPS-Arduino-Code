/*
||                  GPS Faker	             ||
||                                           ||
||	An arduino script to intercept VICON	 ||
||	MavLink messages, and spoof UBX GPS.	 ||
||											 ||


//=============//
	  Notes
//=============//

This system is highly dependant on the relaying
of accurate data from a VICON->MavLink encoder.

If there is no VICON data, or the platform is
out of range, the encoder should transmit
FLOAT_MAX, instead of the positional and
rotational measurements.

It is imperative that the system is aligned so
that the x/y/z and rotational data is
transformed to match magnetic North.

Assumes baud rates as follows;
Comms Serial: 57600
Debug Serial: 57600

 - Kye Morton
*/

//NEED TO SEND BAD GPS MESSAGES ON NO VICON!!

#define MUTE_SYS_STATUS 0
#define MUTE_DEBUG_STATUS 0
#define MUTE_RAW_STATUS 0

#define UINT8_MAX 0xFF
#define UINT16_MAX 0xFFFF
#define UINT32_MAX 0xFFFFFFFF
#define UINT64_MAX 0xFFFFFFFFFFFFFFFFULL
#define FLOAT_MAX_HEX 0x7F800000

#define UBX_HEAD_ID 0xB562

#define UBXID_NAV_POSLLH 0x0102 										//!< message id for NAV-POSLLH
#define UBXID_NAV_VELNED 0x0112 										//!< message id for NAV-VELNED
#define UBXID_NAV_STATUS 0x0103 										//!< message id for NAV-STATUS
#define UBXID_NAV_SOL 0x0106 											//!< message id for NAV-SOL
#define UBXID_NAV_PVT 0x0107 											//!< message id for NAV-PVT
#define UBXID_NAV_SVINFO 0x0130											//!< message id for NAV-PVT

#define UBXID_ACK_ACK 0x0501 											//!< message id for NAV-STATUS
#define UBXID_ACK_NACK 0x0500 											//!< message id for NAV-STATUS
#define UBXID_MOD_ACT 0x0A21 											//!< message id to represent the module is active

#define UBXID_CFG_RATE 0x0608 											//!< message id for CFG-RATE
#define UBXID_CFG_PRT 0x0600 											//!< message id for CFG-PRT
#define UBXID_CFG_NAV5 0x0624 											//!< message id for CFG-NAV5
#define UBXID_CFG_MSG 0x0601 											//!< message id for CFG-MSG

#define UBXID_MON_VER 0x0A04 											//!< message id for MON-VER
#define UBXID_MON_HW 0x0A09 											//!< message id for MON-HW

#define UBXID_TIM_TP 0x0D01 											//!< message id for TIM-TP

#define UBX_LEN_NAV_POSLLH 36	 										//!< message length for NAV-POSLLH
#define UBX_LEN_NAV_VELNED 44 											//!< message length for NAV-VELNED
#define UBX_LEN_NAV_STATUS 24 											//!< message length for NAV-STATUS
#define UBX_LEN_NAV_SOL 60	 											//!< message length for NAV-SOL

#define UBX_LEN_NAV_PVT 92	 											//!< message length for NAV-PVT
#define UBX_LEN_MON_HW 72	 											//!< message length for NAV-PVT
#define UBX_LEN_NAV_SVINFO 136	 										//!< message length for NAV-PVT

#define GPS_UBX_NAV_STATUS_FLAGS_GPSFIXOK_MASK 0x01 					//!< Mask for field gpsFixOk in bitmask flags
#define GPS_UBX_NAV_STATUS_FLAGS_DIFFSOLN_MASK 0x02 					//!< Mask for field diffSoln in bitmask flags
#define GPS_UBX_NAV_STATUS_FLAGS_WKNSET_MASK 0x04 						//!< Mask for field wknSet in bitmask flags
#define GPS_UBX_NAV_STATUS_FLAGS_TOWSET_MASK 0x08 						//!< Mask for field towSet in bitmask flags
#define GPS_UBX_NAV_STATUS_DIFFSTAT_DGPSISTAT_MASK 0x03 				//!< Mask for field dgpsIStat in bitmask diffStat

#define DEG_LATTITUDE_AT_0_0 110574.2727
#define DEG_LONGITUDE_AT_0_0 111319.458
#define CMD_LEN_VICON_DATA 32
#define UBXID_REC_OK 	0x0A21 

#define UBX_BAUD 0x00009600

//=============//
//	Includes   //
//=============//

//#include <SoftwareSerial.h>


//=============//
//	 Structs   //
//=============//

struct msgBuff_s														//Temporary Storage While Reading Messages
{
	unsigned int byteCount;
	uint8_t inputs[263];
	unsigned char expByteLeft;
	bool phraseInProgress;
	uint16_t id;
};

struct gpsData_s
{
	uint64_t upTime;													//Timestamp since boot (ms)
	float x;															//X translation (mm)
	float y;															//Y translation (mm)
	float z;															//Z translation (mm)
	float r;															//Roll angle (rad)
	float p;															//Pitch anlge (rad)
	float q;															//Yaw anlge (rad)
	unsigned long readTime;
	unsigned char payloadLen;
	float xold;
	float yold;
	float zold;
	bool newData;
};

struct GPS_UBX_NAV_POSLLH_s
{
	uint32_t iTOW; 														//!< GPS Millisecond Time of Week
	int32_t lon; 														//!< Longitude
	int32_t lat; 														//!< Latitude
	int32_t height; 													//!< Height above Ellipsoid
	int32_t hMSL; 														//!< Height above mean sea level
	uint32_t hAcc; 														//!< Horizontal Accuracy Estimate
	uint32_t vAcc; 														//!< Vertical Accuracy Estimate
};


struct GPS_UBX_NAV_STATUS_s
{
	uint32_t iTOW; 														//!< GPS Millisecond Time of Week
	uint8_t gpsFix; 													//!< GPSfix Type
	uint8_t flags; 														//!< Navigation Status Flags
	uint8_t diffStat; 													//!< Differential Status
	uint8_t res; 														//!< Reserved
	uint32_t ttff; 														//!< Time to first fix (millisecond time tag)
	uint32_t msss; 														//!< Milliseconds since Startup / Reset
};

struct GPS_UBX_NAV_VELNED_s
{
	uint32_t iTOW; 														//!< GPS Millisecond Time of Week
	int32_t velN; 														//!< NED north velocity
	int32_t velE; 														//!< NED east velocity
	int32_t velD; 														//!< NED down velocity
	uint32_t speed; 													//!< Speed (3-D)
	uint32_t gSpeed; 													//!< Ground Speed (2-D)
	int32_t heading; 													//!< Heading 2-D
	uint32_t sAcc; 														//!< Speed Accuracy Estimate
	uint32_t cAcc; 														//!< Course / Heading Accuracy Estimate
};

struct GPS_UBX_NAV_SOL_s
{
	uint32_t iTOW; 
	int32_t fTOW;
	uint16_t week; 														//!< GPS Millisecond Time of Week
	uint8_t gpsFix; 													//!< GPSfix Type
	uint8_t flags; 	
	int32_t ecefX; 
	int32_t ecefY; 
	int32_t ecefZ; 	
	uint32_t pAcc; 
	int32_t ecefVX; 	
	int32_t ecefVY; 	
	int32_t ecefVZ;	
	uint32_t sAcc;													//!< GPS Millisecond Time of Week
	uint16_t pDOP; 													//!< GPS Millisecond Time of Week
	uint8_t res1; 													//!< GPS Millisecond Time of Week
	uint8_t numSV; 													//!< GPS Millisecond Time of Week
	uint32_t res2;
 	
	int32_t ecefXOld; 
	int32_t ecefYOld; 
	int32_t ecefZOld;												//!< Course / Heading Accuracy Estimate
};

struct GPS_UBX_NAV_PVT_s
{
	uint32_t iTOW;
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t valid;
	uint32_t fAcc;
	int32_t nano;
	uint8_t fix;
	uint8_t flag;
	uint8_t res1;
	uint8_t numSV;
	int32_t lon;
	int32_t lat;
	int32_t height;
	int32_t hmsl;
	uint32_t hiAcc;
	uint32_t vAcc;
	int32_t velN;
	int32_t velE;
	int32_t velD;
	int32_t gSpeed;
	int32_t head;
	uint32_t sAcc;
	uint32_t heAcc;
	int16_t pDOP;
	uint16_t res2;
	uint32_t res3;
};

struct GPS_UBX_NAV_SVINFO_s
{
	
};

struct GPS_UBX_MON_HW_s
{
	uint32_t pinSel;
	uint32_t pinBank;
	uint32_t pinDir;
	uint32_t pinVal;
	uint16_t noisePerMS;
	uint16_t agcCnt;
	uint8_t aStatus;
	uint8_t aPower;
	uint8_t flags;
	uint8_t res1;
	uint16_t usedMask;
	uint8_t VP[25];
	uint8_t jamInd;
	uint8_t res2[2];
	uint32_t pinIrq;
	uint32_t pullH;
	uint32_t pullL;
};

//=============//
// Func.Protos //
//=============//
unsigned char comm_receive();
char phraseMessage(unsigned char c, msgBuff_s &inMsg);
void clearMsgBuff(msgBuff_s &msg);
void decodeMessage(msgBuff_s &msgIn);

void spoofGPS(gpsData_s data);
void compileUBXPosllh(msgBuff_s &msg, GPS_UBX_NAV_POSLLH_s data);
void compileUBXStatus(msgBuff_s &msg, GPS_UBX_NAV_STATUS_s data);
void compileUBXVelned(msgBuff_s &msg, GPS_UBX_NAV_VELNED_s data);
void compileUBXSol(msgBuff_s &msg, GPS_UBX_NAV_SOL_s data);
void compileUBXPvt(msgBuff_s &msg, GPS_UBX_NAV_PVT_s data);
void compileUBXSVInfo(msgBuff_s &msg, GPS_UBX_NAV_SVINFO_s data);
void compileUBXMonHW(msgBuff_s &msg, GPS_UBX_MON_HW_s data);

void packUBXUint32(msgBuff_s &msg, uint32_t num, uint32_t offset);
void packUBXUint16(msgBuff_s &msg, uint32_t num, uint32_t offset);
void calcUbxChecksum(uint8_t *checksumPayload, uint8_t payloadSize);

void GPSToECEF(int32_t &XECEF, int32_t &YECEF, int32_t &ZECEF, int32_t lattitude, int32_t longitude, int32_t altitude);

static void writeMessage(msgBuff_s &msg, uint32_t port); 

void clearGPSData(gpsData_s &data);
void getGPSData(msgBuff_s &msg, gpsData_s &data);

void sendInitial();
void sendMonitorVersion();

void sendUBXAck(uint16_t ids);
void sendUBXNack(uint16_t ids);

//=============//
//	Variables  //
//=============//
uint16_t crcAccum = 0;

msgBuff_s inMsgSerial;

gpsData_s leadUAVgps;													//Used to catch vicon data and then spoof the GPS

bool readGPS = 0;

unsigned long startTimeMilli = 0;

unsigned long firstViconFrame = 0;
bool viconReceived = false;

bool usePVT = 1;

bool LEDStatus = false;
unsigned int LEDPin = 13;

//SoftwareSerial gpsSerial(2, 3); // RX, TX

bool MUTE = MUTE_SYS_STATUS;											//Set to '1' to disable debug serial output
bool MUTE_DEBUG = MUTE_DEBUG_STATUS;									//Set to '1' to disable debug serial output
bool MUTE_RAW = MUTE_RAW_STATUS;										//Set to '1' to disable debug serial output


//=============//
//	  Main     //
//=============//
void setup()
{  
	startTimeMilli = millis();
	
	
	
	Serial.begin(57600);												//Monitoring Link

	if(!MUTE)
	{
		Serial.println("Opening communication serial read on Serial Port 0...");
		Serial.println();
	}

	Serial1.begin(38400);												//Monitoring Link

	if(!MUTE)
	{
		Serial.println("Opening communication serial read on Serial Port 1...");
		Serial.println();
	}
	
	pinMode(LEDPin, OUTPUT);
	digitalWrite(LEDPin, LOW);
		
	clearMsgBuff(inMsgSerial);
	inMsgSerial.id = 1;
      
        //delay(2000);
      
	clearGPSData(leadUAVgps);
        delay(2000);
        sendInitial();
        //delay(2000);
}

void loop()
{
	unsigned long startTimeLoop = millis();
	unsigned long startTimeSecs = (startTimeLoop - startTimeMilli)/1000;
	
	if(!MUTE)
	{
		Serial.print("\nSystem has been online for ");
		Serial.print(startTimeSecs);
		Serial.println(" seconds.");
	}
	
	while (millis() - startTimeLoop < (1000/5))								//Holds the program for 1 second to catch new data from the serial buffer
	{
		switch(comm_receive())
		{
			case 1:														//Do something here maybe?
			{
				
				break;
			}
			case 2:														//Do something here maybe?
			{
				
				break;
			}
			default:
			{
				break;
			}
		}
	}
	
 	//Fakeing code
	leadUAVgps.upTime = (unsigned long long)millis();													//Timestamp since boot (ms)
	leadUAVgps.x = 100000;															//X translation (mm)
	leadUAVgps.y = 0;															//Y translation (mm)
	leadUAVgps.z = 0;															//Z translation (mm)
	leadUAVgps.r = 0;															//Roll angle (rad)
	leadUAVgps.p = 0;															//Pitch anlge (rad)
	leadUAVgps.q = 0;															//Yaw anlge (rad)
	leadUAVgps.readTime = (unsigned long long)millis();
	leadUAVgps.xold = 4000;
	leadUAVgps.yold = 4000;
	leadUAVgps.zold = 4000;
	leadUAVgps.newData = true;
	
	spoofGPS(leadUAVgps);
	
	LEDStatus = true;
	
	if(!viconReceived)
	{
		viconReceived = true;
		firstViconFrame = millis();
	}
	//Faking Code
	
	if(!MUTE)
	{
		Serial.print("VICON Status: ");
	
		if(LEDStatus)
		{
			Serial.println("Lock");
		}
		else
		{
			Serial.println("No Lock");
		}
	}

	if(LEDStatus)
	{
		digitalWrite(LEDPin, HIGH);
	}
	else
	{
		digitalWrite(LEDPin, LOW);
	}
	
	LEDStatus = false;
}

unsigned char comm_receive()											//Reads in all available data in the serial buffer and passes it off to to be phrased
{
	while(Serial1.available() > 0) 
	{
		unsigned char c = Serial1.read();

		if(phraseMessage(c, inMsgSerial)>0)								//Will return 1 when a message has completed being read.
		{		
			return 1;
		}
	}
		
	return 0;
}

char phraseMessage(unsigned char c, msgBuff_s &inMsg)
{
	bool messageComplete = 0;
	uint16_t crcTemp = 0;
	uint16_t crcCheck = 0;
	int i = 0;
	
	if((c==(UBX_HEAD_ID>>8))&&(!inMsg.phraseInProgress))							//If it a header byte (0xB5) is detected, and there is no message currently being phrased
	{
        clearMsgBuff(inMsg);											//Reset message buffer and start recording message
		inMsg.phraseInProgress = 1;
	}
	
	if(inMsg.phraseInProgress==1)
	{
		inMsg.inputs[inMsg.byteCount] = c;								//Appends the newest byte to the array
		
		inMsg.byteCount++;

		
		if(inMsg.byteCount>=((((uint16_t)inMsg.inputs[4])|((uint16_t)inMsg.inputs[5]<<8))+8))
		{	
			inMsg.phraseInProgress = 0;
			messageComplete = 1;
		}
	}
	
	if(messageComplete)
	{
		crcTemp = ((uint16_t)inMsg.inputs[inMsg.byteCount-2])|((uint16_t)inMsg.inputs[inMsg.byteCount-1]<<8);		//Store the checksum gathered from the message
		
		calcUbxChecksum(&inMsg.inputs[2], inMsg.byteCount - 4);				//Recalculate the checksum

		crcCheck = ((uint16_t)inMsg.inputs[inMsg.byteCount-2])|((uint16_t)inMsg.inputs[inMsg.byteCount-1]<<8);	//Gather the value of the given CRC
		
		if(crcTemp==crcCheck)											//If the CRCs match, message is valid
		{	
			c = 0;
			
			decodeMessage(inMsg);
			return 1;
		}
	}
	
	return 0;															//Discard message, as it is not valid
}

void decodeMessage(msgBuff_s &msgIn)
{
	uint16_t id = ((uint16_t)msgIn.inputs[2]<<8)|((uint16_t)msgIn.inputs[3]);
	
	if(!MUTE_RAW)
	{
		Serial.print("Receiving: ");
		writeMessage(msgIn,0);
		Serial.println();
	}

	switch(id)															//Add case's to this to give options for different id's to be recognised
	{
		case UBXID_ACK_ACK:
		{
			if(!MUTE_DEBUG)
			{
				Serial.println("ACK-ACK message detected...");
			}

			break;
		}
		case UBXID_ACK_NACK:
		{
			if(!MUTE_DEBUG)
			{
				Serial.println("ACK-NACK message detected...");
			}

			break;
		}
		case UBXID_CFG_RATE:
		{
			if(!MUTE_DEBUG)
			{
				Serial.println("CFG-RATE message detected...");
			}

			sendUBXAck(id);               

			break;
		}
		case UBXID_CFG_PRT:
		{
			if(!MUTE_DEBUG)
			{
				Serial.println("CFG-PRT message detected...");
			}
			
			uint32_t baud = ((uint32_t)msgIn.inputs[13]<<24)|((uint32_t)msgIn.inputs[14]<<16)|((uint32_t)msgIn.inputs[15]<<8)|((uint32_t)msgIn.inputs[16]);
			
			if(baud == UBX_BAUD)
			{
				if(!MUTE_DEBUG)
        		{
    			    Serial.print("Baud rate confirmed at: ");
					Serial.println(UBX_BAUD);
        		}
				
				sendUBXAck(id);
			}
			else
			{
				sendUBXAck(id);
			
				if(!MUTE_DEBUG)
        		{
    			    Serial.print("Unknown CFG-PRT detected: ");
					writeMessage(msgIn, 0);
					Serial.println();
        		}
			}
			
			break;
		}
		case UBXID_CFG_NAV5:
		{
			if(!MUTE_DEBUG)
			{
				Serial.println("CFG-NAV5 message detected...");
			}

			sendUBXAck(id);

			break;
		}
		case UBXID_CFG_MSG:
		{
			if(!MUTE_DEBUG)
			{
				Serial.println("CFG-MSG message detected...");
			}

			uint32_t msgID = ((uint32_t)msgIn.inputs[6]<<8)|((uint32_t)msgIn.inputs[7]);

			switch(msgID)
			{
				case UBXID_NAV_PVT:
				{
					sendUBXAck(id);
					
        			if(!MUTE_DEBUG)
        			{
    			        Serial.println("Using NAV-PVT encoding.");
        			}
					
					usePVT = true;
					break;
				}
				case UBXID_MON_HW:
				{
					sendUBXAck(id);
					
        			if(!MUTE_DEBUG)
        			{
    			        Serial.println("Using MON-HW encoding.");
        			}
					
					break;
				}
				case UBXID_NAV_SVINFO:
				{
					sendUBXAck(id);

        			if(!MUTE_DEBUG)
        			{
    			        Serial.println("Using NAV-SVINFO encoding.");
        			}
					
					break;
				}
				default:
				{
					sendUBXNack(id);
        		
					if(!MUTE_DEBUG)
        			{
    			        Serial.print("Unknown CFG-MSG detected: ");
						writeMessage(msgIn, 0);
						Serial.println();
        			}
					
					break;
				}
			
			}

			break;
		}
		case UBXID_MON_VER:
		{
			if(!MUTE_DEBUG)
			{
				Serial.println("MON-VER message detected...");
			}

            sendMonitorVersion();
			
			break;
		}
		case UBXID_MON_HW:
		{
			if(!MUTE_DEBUG)
			{
				Serial.println("MON-HW message detected...");
			}

			break;
		}
		case UBXID_TIM_TP:
		{
			if(!MUTE_DEBUG)
			{
				Serial.println("TIM-TP message detected...");
			}

			break;
		}		
		default:
		{
			sendUBXNack(id);
			
			if(!MUTE_DEBUG)
			{
				Serial.println("Unknown message detected...");
			}		
			
			break;
		}
	}
}

static void writeMessage(msgBuff_s &msg, uint32_t port)
{
	switch(port)
	{
		case 0:
		{
			for(int i=0;i<msg.byteCount;i++)
			{
				if(msg.inputs[i]<0x10)
				{
					Serial.print("0");
				}
				Serial.print(msg.inputs[i], HEX);
				Serial.print(" ");
			}
			
			break;
		}
		case 1:
		{
			for(int i=0;i<msg.byteCount;i++)
			{
				Serial1.write(msg.inputs[i]);
			}
			
			break;
		}
		case 2:
		{
			for(int i=0;i<msg.byteCount;i++)
			{
				//gpsSerial.write(msg.inputs[i]);
			}
			
			break;			
		}
		default:
		{
			break;
		}
	}	
}

void clearMsgBuff(msgBuff_s &msg)										//Resets the provided buffer struct
{
	msg.byteCount = 0;
	msg.phraseInProgress = 0;
	msg.expByteLeft = 0;
	
  	for(int k=0;k<263;k++)
	{
		msg.inputs[k] = 0;
	}
}

void clearGPSData(gpsData_s &data)										//Resets the provided GPS struct
{
	data.xold = 0;
	data.yold = 0;
	data.zold = 0;

	if(viconReceived)
	{
		data.xold = data.x;
		data.yold = data.y;
		data.zold = data.z;
	}
	
	data.upTime = UINT64_MAX;
	data.x = 0;
	data.y = 0;
	data.z = 0;
	data.p = 0;
	data.q = 0;
	data.r = 0;
	data.readTime = 0;
	data.payloadLen = CMD_LEN_VICON_DATA;
	
	data.newData = false;
}

void spoofGPS(gpsData_s data)
{
	msgBuff_s msg;
	clearMsgBuff(msg);
	
	double pi = 3.14159265359;
	uint32_t gpsTimeOfWeek = millis();
	
	if(!usePVT)
	{
		GPS_UBX_NAV_POSLLH_s GPSPosllhData;
		GPS_UBX_NAV_STATUS_s GPSStatusData;
		GPS_UBX_NAV_VELNED_s GPSVelnedData;
		GPS_UBX_NAV_SOL_s GPSSolData;

		msg.byteCount = UBX_LEN_NAV_POSLLH;
		msg.id = UBXID_NAV_POSLLH;
		
		GPSPosllhData.iTOW = gpsTimeOfWeek;									//Generate fake data
		GPSPosllhData.lat = (int32_t)((data.x/(1000*DEG_LATTITUDE_AT_0_0))*10000000);
		GPSPosllhData.lon = (int32_t)((data.y/(1000*DEG_LONGITUDE_AT_0_0))*10000000);
		GPSPosllhData.height = (int32_t)(data.z);
		GPSPosllhData.hMSL = (int32_t)(data.z);
		GPSPosllhData.hAcc = 0;
		GPSPosllhData.vAcc = 0;

		compileUBXPosllh(msg, GPSPosllhData);						//Compile messages to be sent out
		writeMessage(msg, 1);
		   
		if(!MUTE_RAW)
		{
			Serial.print("Sending: ");
			writeMessage(msg, 0);
			Serial.println();
		}

		clearMsgBuff(msg);
		msg.byteCount = UBX_LEN_NAV_STATUS;
		msg.id = UBXID_NAV_STATUS;

		GPSStatusData.iTOW = gpsTimeOfWeek;
		GPSStatusData.gpsFix = 0x03;										//3D-fix
		GPSStatusData.flags = 0xF;											//Good fix, pretending to use DGPS, valid times.
		GPSStatusData.diffStat = 0x3;										//High accuracy PR+PRR+CP Correction
		GPSStatusData.res = 0;												//Reserved
		GPSStatusData.ttff = firstViconFrame;
		GPSStatusData.msss = millis();
		
		compileUBXStatus(msg, GPSStatusData);
		writeMessage(msg, 1);

		if(!MUTE_RAW)
		{
			Serial.print("Sending: ");
			writeMessage(msg, 0);
			Serial.println();
		}

		clearMsgBuff(msg);
		msg.byteCount = UBX_LEN_NAV_VELNED;
		msg.id = UBXID_NAV_VELNED;

		GPSVelnedData.iTOW = gpsTimeOfWeek;
		GPSVelnedData.velN = (int32_t)(5*(data.y-data.yold)/10);			//Should be ignored, but just in case (cm/s)
		GPSVelnedData.velE = (int32_t)(5*(data.x-data.xold)/10);			//Should be ignored, but just in case (cm/s)
		GPSVelnedData.velD = (int32_t)(-5*(data.z-data.zold)/10);			//Should be ignored, but just in case (cm/s)
		GPSVelnedData.speed = (uint32_t)sqrt((GPSVelnedData.velN*GPSVelnedData.velN) + (GPSVelnedData.velE*GPSVelnedData.velE) + (GPSVelnedData.velD*GPSVelnedData.velD));
		GPSVelnedData.gSpeed = (uint32_t)sqrt((GPSVelnedData.velN*GPSVelnedData.velN) + (GPSVelnedData.velE*GPSVelnedData.velE));
		GPSVelnedData.heading = (int32_t)((data.q*pi/180)*100000);
		GPSVelnedData.sAcc = 0;
		GPSVelnedData.cAcc = 0;

		compileUBXVelned(msg, GPSVelnedData);
		writeMessage(msg, 1);

		if(!MUTE_RAW)
		{
			Serial.print("Sending: ");
			writeMessage(msg, 0);
			Serial.println();
		}

		clearMsgBuff(msg);
		msg.byteCount = UBX_LEN_NAV_SOL;
		msg.id = UBXID_NAV_SOL;

		int32_t XECEF = 0;
		int32_t YECEF = 0;
		int32_t ZECEF = 0;

		GPSToECEF(XECEF, YECEF, ZECEF, GPSPosllhData.lat, GPSPosllhData.lon, GPSPosllhData.height*1000);

		GPSSolData.iTOW = gpsTimeOfWeek; 
		GPSSolData.fTOW = 0;
		GPSSolData.week = 0;													//!< GPS Millisecond Time of Week
		GPSSolData.gpsFix = 0x03; 												//!< GPSfix Type
		GPSSolData.flags = 0xF;
		GPSSolData.ecefXOld = GPSSolData.ecefX; 
		GPSSolData.ecefYOld = GPSSolData.ecefY; 
		GPSSolData.ecefZOld = GPSSolData.ecefZ;
		GPSSolData.ecefX = XECEF; 
		GPSSolData.ecefY = YECEF; 
		GPSSolData.ecefZ = ZECEF;	
		GPSSolData.pAcc = 0; 
		GPSSolData.ecefVX = 5*(GPSSolData.ecefX-GPSSolData.ecefXOld);
		GPSSolData.ecefVY = 5*(GPSSolData.ecefY-GPSSolData.ecefYOld);
		GPSSolData.ecefVZ = 5*(GPSSolData.ecefZ-GPSSolData.ecefZOld);	
		GPSSolData.sAcc = 0;
		GPSSolData.pDOP = 0;
		GPSSolData.res1 = 0;
		GPSSolData.numSV = 11;
		GPSSolData.res2 = 0; 
			
		compileUBXSol(msg, GPSSolData);
		writeMessage(msg, 1);	

		if(!MUTE_RAW)
		{
			Serial.print("Sending: ");
			writeMessage(msg, 0);
			Serial.println();
		}
	}
	else
	{
		GPS_UBX_NAV_PVT_s GPSPvtData;
		GPS_UBX_MON_HW_s GPSHWData;
		GPS_UBX_NAV_SVINFO_s GPSSVData;

		msg.byteCount = UBX_LEN_NAV_PVT;
		msg.id = UBXID_NAV_PVT;
		
		GPSPvtData.iTOW = gpsTimeOfWeek;
		GPSPvtData.year = 1970;
		GPSPvtData.month = 1;
		GPSPvtData.day = 1;
		GPSPvtData.hour = 0;
		GPSPvtData.min = 0;
		GPSPvtData.sec = 0;
		GPSPvtData.valid = 0x07;
		GPSPvtData.fAcc = 0;
		GPSPvtData.nano = 0;
		GPSPvtData.fix = 0x03;
		GPSPvtData.flag = 0xF;
		GPSPvtData.res1 = 0;
		GPSPvtData.numSV = 10;
		GPSPvtData.lon = (int32_t)((data.y/(1000*DEG_LONGITUDE_AT_0_0))*10000000);
		GPSPvtData.lat = (int32_t)((data.x/(1000*DEG_LATTITUDE_AT_0_0))*10000000);
		GPSPvtData.height = (int32_t)(data.z);
		GPSPvtData.hmsl = (int32_t)(data.z);
		GPSPvtData.hiAcc = 0;
		GPSPvtData.vAcc = 0;
		GPSPvtData.velN = (int32_t)(5*(data.y-data.yold)/10);
		GPSPvtData.velE = (int32_t)(5*(data.x-data.xold)/10);
		GPSPvtData.velD = (int32_t)(-5*(data.z-data.zold)/10);
		GPSPvtData.gSpeed = (uint32_t)sqrt((GPSPvtData.velN*GPSPvtData.velN) + (GPSPvtData.velE*GPSPvtData.velE));
		GPSPvtData.head = (int32_t)((data.q*pi/180)*100000);
		GPSPvtData.sAcc = 0;
		GPSPvtData.heAcc = 0;
		GPSPvtData.pDOP = 0;
		GPSPvtData.res2 = 0;
		GPSPvtData.res3 = 0;	
		
		compileUBXPvt(msg, GPSPvtData);
		writeMessage(msg, 1);

		if(!MUTE_RAW)
		{
			Serial.print("Sending: ");
			writeMessage(msg, 0);
			Serial.println();
		}
		
		clearMsgBuff(msg);
		msg.byteCount = UBX_LEN_MON_HW;
		msg.id = UBXID_MON_HW;
		
		GPSHWData.pinSel = UINT32_MAX;
		GPSHWData.pinBank = UINT32_MAX;
		GPSHWData.pinDir = UINT32_MAX;
		GPSHWData.pinVal = UINT32_MAX;
		GPSHWData.noisePerMS = 0;
		GPSHWData.agcCnt = 0;
		GPSHWData.aStatus = 4;
		GPSHWData.aPower = 1;
		GPSHWData.flags = 0x02;
		GPSHWData.res1 = 0;
		GPSHWData.usedMask = UINT32_MAX;
		for(uint16_t i=0;i<25;i++)
		{
			GPSHWData.VP[i] = 1;							//25
		}
		GPSHWData.jamInd = 0;		
		for(uint16_t j=0;j<2;j++)
		{
			GPSHWData.res2[j] = 1;							//2
		}
		GPSHWData.pinIrq = UINT32_MAX;
		GPSHWData.pullH = UINT32_MAX;
		GPSHWData.pullL = UINT32_MAX;
		
		compileUBXMonHW(msg, GPSHWData);
		writeMessage(msg, 1);

		if(!MUTE_RAW)
		{
			Serial.print("Sending: ");
			writeMessage(msg, 0);
			Serial.println();
		}
	}
}

void GPSToECEF(int32_t &XECEF, int32_t &YECEF, int32_t &ZECEF, int32_t lattitude, int32_t longitude, int32_t altitude)
{
    double a = 6378137;

    double e = 8.1819190842622e-2;

    double asq = a*a;
    double esq = e*e;
    double pi = 3.1415926535897;

     double lat = (((double)lattitude)/10000000)*pi/180;
     double lon = (((double)longitude)/10000000)*pi/180;
     double alt = altitude;
  
     double N = a / sqrt(1 - esq * (sin(lat)*sin(lat)));

     XECEF = (int32_t)((N + alt) * cos(lat) * cos(lon) * 10000);
     YECEF = (int32_t)((N + alt) * cos(lat) * sin(lon) * 10000);
     ZECEF = (int32_t)(((1 - esq) * N + alt) * sin(lat) * 10000);
}

void compileUBXPosllh(msgBuff_s &msg, GPS_UBX_NAV_POSLLH_s data)
{
	msg.inputs[0] = (uint8_t)(UBX_HEAD_ID>>8);								//Header
	msg.inputs[1] = (uint8_t)(UBX_HEAD_ID);
	msg.inputs[2] = (uint8_t)(msg.id>>8);									//ID
	msg.inputs[3] = (uint8_t)(msg.id);
	msg.inputs[4] = (uint8_t)((msg.byteCount-8));							//Length
	msg.inputs[5] = (uint8_t)((msg.byteCount-8)>>8);
	
	packUBXUint32(msg, data.iTOW, 0);									//Payload
	packUBXUint32(msg, data.lon, 4);
	packUBXUint32(msg, data.lat, 8);
	packUBXUint32(msg, data.height, 12);
	packUBXUint32(msg, data.hMSL, 16);
	packUBXUint32(msg, data.hAcc, 20);
	packUBXUint32(msg, data.vAcc, 24);

	calcUbxChecksum(&msg.inputs[2], msg.byteCount - 4);					//Checksum
}

void compileUBXStatus(msgBuff_s &msg, GPS_UBX_NAV_STATUS_s data)
{
	msg.inputs[0] = (uint8_t)(UBX_HEAD_ID>>8);								//Header
	msg.inputs[1] = (uint8_t)(UBX_HEAD_ID);
	msg.inputs[2] = (uint8_t)(msg.id>>8);									//ID
	msg.inputs[3] = (uint8_t)(msg.id);
	msg.inputs[4] = (uint8_t)((msg.byteCount-8));							//Length
	msg.inputs[5] = (uint8_t)((msg.byteCount-8)>>8);

	packUBXUint32(msg, data.iTOW, 0);									//Payload
	msg.inputs[10] = data.gpsFix;
	msg.inputs[11] = data.flags;
	msg.inputs[12] = data.diffStat;
	msg.inputs[13] = data.res;
	packUBXUint32(msg, data.ttff, 8);
	packUBXUint32(msg, data.msss, 12);

	calcUbxChecksum(&msg.inputs[2], msg.byteCount - 4);					//Checksum
}

void compileUBXVelned(msgBuff_s &msg, GPS_UBX_NAV_VELNED_s data)
{
	msg.inputs[0] = (uint8_t)(UBX_HEAD_ID>>8);								//Header
	msg.inputs[1] = (uint8_t)(UBX_HEAD_ID);
	msg.inputs[2] = (uint8_t)(msg.id>>8);									//ID
	msg.inputs[3] = (uint8_t)(msg.id);
	msg.inputs[4] = (uint8_t)((msg.byteCount-8));							//Length
	msg.inputs[5] = (uint8_t)((msg.byteCount-8)>>8);

	packUBXUint32(msg, data.iTOW, 0);									//Payload
	packUBXUint32(msg, data.velN, 4);
	packUBXUint32(msg, data.velE, 8);
	packUBXUint32(msg, data.velD, 12);
	packUBXUint32(msg, data.speed, 16);
	packUBXUint32(msg, data.gSpeed, 20);
	packUBXUint32(msg, data.heading, 24);
	packUBXUint32(msg, data.sAcc, 28);
	packUBXUint32(msg, data.cAcc, 32);

	calcUbxChecksum(&msg.inputs[2], msg.byteCount - 4);					//Checksum
}

void compileUBXSol(msgBuff_s &msg, GPS_UBX_NAV_SOL_s data)
{
	msg.inputs[0] = (uint8_t)(UBX_HEAD_ID>>8);								//Header
	msg.inputs[1] = (uint8_t)(UBX_HEAD_ID);
	msg.inputs[2] = (uint8_t)(msg.id>>8);									//ID
	msg.inputs[3] = (uint8_t)(msg.id);
	msg.inputs[4] = (uint8_t)((msg.byteCount-8));							//Length
	msg.inputs[5] = (uint8_t)((msg.byteCount-8)>>8);

	packUBXUint32(msg, data.iTOW, 0);
	packUBXUint32(msg, data.fTOW, 4);
	packUBXUint16(msg, data.week, 8);
	msg.inputs[15] = data.gpsFix;
	msg.inputs[16] = data.flags;
	packUBXUint32(msg, data.ecefX, 12);
	packUBXUint32(msg, data.ecefY, 16);
	packUBXUint32(msg, data.ecefZ, 20);
	packUBXUint32(msg, data.pAcc, 24);
	packUBXUint32(msg, data.ecefVX, 28);
	packUBXUint32(msg, data.ecefVY, 32);
	packUBXUint32(msg, data.ecefVZ, 36);
	packUBXUint32(msg, data.sAcc, 40);
	packUBXUint16(msg, data.pDOP, 44);
	msg.inputs[51] = data.res1;
	msg.inputs[52] = data.numSV;
	packUBXUint32(msg, data.res2, 48);

	calcUbxChecksum(&msg.inputs[2], msg.byteCount - 4);					//Checksum
}

void compileUBXPvt(msgBuff_s &msg, GPS_UBX_NAV_PVT_s data)
{
	msg.inputs[0] = (uint8_t)(UBX_HEAD_ID>>8);								//Header
	msg.inputs[1] = (uint8_t)(UBX_HEAD_ID);
	msg.inputs[2] = (uint8_t)(msg.id>>8);									//ID
	msg.inputs[3] = (uint8_t)(msg.id);
	msg.inputs[4] = (uint8_t)((msg.byteCount-8));							//Length
	msg.inputs[5] = (uint8_t)((msg.byteCount-8)>>8);

	packUBXUint32(msg, data.iTOW, 0);									//Payload
	packUBXUint16(msg, data.year, 4);
	msg.inputs[12] = data.month;
	msg.inputs[13] = data.day;
	msg.inputs[14] = data.hour;
	msg.inputs[15] = data.min;
	msg.inputs[16] = data.sec;
	msg.inputs[17] = data.valid;
	packUBXUint32(msg, data.fAcc, 12);
	packUBXUint32(msg, data.nano, 16);
	msg.inputs[26] = data.fix;
	msg.inputs[27] = data.flag;
	msg.inputs[28] = data.res1;
	msg.inputs[29] = data.numSV;
	packUBXUint32(msg, data.lon, 24);
	packUBXUint32(msg, data.lat, 28);
	packUBXUint32(msg, data.height, 32);
	packUBXUint32(msg, data.hmsl, 36);
	packUBXUint32(msg, data.hiAcc, 40);
	packUBXUint32(msg, data.vAcc, 44);
	packUBXUint32(msg, data.velN, 48);
	packUBXUint32(msg, data.velE, 52);
	packUBXUint32(msg, data.velD, 56);
	packUBXUint32(msg, data.gSpeed, 60);
	packUBXUint32(msg, data.head, 64);
	packUBXUint32(msg, data.sAcc, 68);
	packUBXUint32(msg, data.heAcc, 72);
	packUBXUint16(msg, data.pDOP, 76);
	packUBXUint16(msg, data.res2, 78);
	packUBXUint32(msg, data.res3, 80);

	calcUbxChecksum(&msg.inputs[2], msg.byteCount - 4);					//Checksum
}

void compileUBXMonHW(msgBuff_s &msg, GPS_UBX_MON_HW_s data)
{
	msg.inputs[0] = (uint8_t)(UBX_HEAD_ID>>8);								//Header
	msg.inputs[1] = (uint8_t)(UBX_HEAD_ID);
	msg.inputs[2] = (uint8_t)(msg.id>>8);									//ID
	msg.inputs[3] = (uint8_t)(msg.id);
	msg.inputs[4] = (uint8_t)((msg.byteCount-8));							//Length
	msg.inputs[5] = (uint8_t)((msg.byteCount-8)>>8);

	packUBXUint32(msg, data.pinSel, 0);
	packUBXUint32(msg, data.pinBank, 4);
	packUBXUint32(msg, data.pinDir, 8);
	packUBXUint32(msg, data.pinVal, 12);
	packUBXUint16(msg, data.noisePerMS, 16);
	packUBXUint16(msg, data.agcCnt, 18);
	msg.inputs[26] = data.aStatus;
	msg.inputs[27] = data.aPower;
	msg.inputs[28] = data.flags;
	msg.inputs[29] = data.res1;
	packUBXUint32(msg, data.usedMask, 24);
	for(uint16_t i=0;i<25;i++)
	{
			msg.inputs[i+34] = data.VP[i];			//25
	}
	msg.inputs[59] = data.jamInd;
	msg.inputs[60] = data.res2[0];
	msg.inputs[61] = data.res2[1];
	packUBXUint32(msg, data.pinIrq, 56);
	packUBXUint32(msg, data.pullH, 60);
	packUBXUint32(msg, data.pullL, 64);
	
	calcUbxChecksum(&msg.inputs[2], msg.byteCount - 4);					//Checksum
}

void compileUBXSVInfo(msgBuff_s &msg, GPS_UBX_NAV_SVINFO_s data)
{
	msg.inputs[0] = (uint8_t)(UBX_HEAD_ID>>8);								//Header
	msg.inputs[1] = (uint8_t)(UBX_HEAD_ID);
	msg.inputs[2] = (uint8_t)(msg.id>>8);									//ID
	msg.inputs[3] = (uint8_t)(msg.id);
	msg.inputs[4] = (uint8_t)((msg.byteCount-8));							//Length
	msg.inputs[5] = (uint8_t)((msg.byteCount-8)>>8);


	calcUbxChecksum(&msg.inputs[2], msg.byteCount - 4);					//Checksum
}

void packUBXUint32(msgBuff_s &msg, uint32_t num, uint32_t offset)		//Packs a uint32 into the payload at a byte offset of "offset" from the start of the payload
{
	for(int i=0;i<4;i++)
	{
		msg.inputs[offset+6+i] = (uint8_t)(num>>(8*i));			//+6 to account for the header
	}
}

void packUBXUint16(msgBuff_s &msg, uint32_t num, uint32_t offset)		//Packs a uint16 into the payload at a byte offset of "offset" from the start of the payload
{
	for(int i=0;i<2;i++)
	{
		msg.inputs[offset+6+i] = (uint8_t)(num>>(8*i));			//+6 to account for the header
	}
}

void calcUbxChecksum(uint8_t *checksumPayload, uint8_t payloadSize)		//Magic
{
	byte CK_A = 0, CK_B = 0;
	
	for (int i = 0; i < payloadSize ;i++) 
	{
		CK_A = CK_A + *checksumPayload;
		CK_B = CK_B + CK_A;
		checksumPayload++;												//More Magic
	}
	
	*checksumPayload = CK_A;
	checksumPayload++;
	*checksumPayload = CK_B;
}

void sendUBXAck(uint16_t ids)
{        
    uint8_t byteCount = 10;
    uint8_t inputs[byteCount];
        
  	inputs[0] = (uint8_t)(UBX_HEAD_ID>>8);								//Header
	inputs[1] = (uint8_t)(UBX_HEAD_ID);
	inputs[2] = (uint8_t)(UBXID_ACK_ACK>>8);									//ID
	inputs[3] = (uint8_t)(UBXID_ACK_ACK);
	inputs[4] = (uint8_t)((byteCount-8));							//Length
	inputs[5] = (uint8_t)((byteCount-8)>>8);
	
    inputs[6] = (uint8_t)(ids>>8);
    inputs[7] = (uint8_t)(ids);

	calcUbxChecksum(&inputs[2], byteCount - 4);					//Checksum
        
        for(int i=0;i<byteCount;i++)
       {
          Serial1.write(inputs[i]);
       } 
}

void sendUBXNack(uint16_t ids)
{        
    uint8_t byteCount = 10;
    uint8_t inputs[byteCount];
        
  	inputs[0] = (uint8_t)(UBX_HEAD_ID>>8);								//Header
	inputs[1] = (uint8_t)(UBX_HEAD_ID);
	inputs[2] = (uint8_t)(UBXID_ACK_NACK>>8);									//ID
	inputs[3] = (uint8_t)(UBXID_ACK_NACK);
	inputs[4] = (uint8_t)((byteCount-8));							//Length
	inputs[5] = (uint8_t)((byteCount-8)>>8);
	
    inputs[6] = (uint8_t)(ids>>8);
    inputs[7] = (uint8_t)(ids);

	calcUbxChecksum(&inputs[2], byteCount - 4);					//Checksum
        
        for(int i=0;i<byteCount;i++)
       {
          Serial1.write(inputs[i]);
       } 
}

void sendInitial(){

    uint8_t byteCount = 9;
    uint8_t inputs[byteCount];
        
    inputs[0] = (uint8_t)(UBX_HEAD_ID>>8);								//Header
    inputs[1] = (uint8_t)(UBX_HEAD_ID);
    inputs[2] = (uint8_t)(UBXID_REC_OK>>8);									//ID
    inputs[3] = (uint8_t)(UBXID_REC_OK);
    inputs[4] = (uint8_t)((byteCount-8));							//Length
    inputs[5] = (uint8_t)((byteCount-8)>>8);
	
    inputs[6] = (uint8_t)0x01;

    calcUbxChecksum(&inputs[2], byteCount - 4);					//Checksum
        
    for(int i=0;i<byteCount;i++)
    {
        Serial1.write(inputs[i]);
    }
}

void sendMonitorVersion(){

	char ver[] = {0x37, 0x2E, 0x30, 0x33, 0x20, 0x28, 0x34, 0x35, 0x39, 0x37, 0x30, 0x29, 0x30, 0x30, 0x30, 0x34, 0x30, 0x30, 0x30, 0x37, 0x00, 0x00, 0x37, 0x2E, 0x30, 0x33, 0x20, 0x28, 0x34, 0x35, 0x39, 0x36, 0x39, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
    uint8_t byteCount = 78;
    uint8_t inputs[byteCount];
        
    inputs[0] = (uint8_t)(UBX_HEAD_ID>>8);								//Header
    inputs[1] = (uint8_t)(UBX_HEAD_ID);
    inputs[2] = (uint8_t)(UBXID_MON_VER>>8);									//ID
    inputs[3] = (uint8_t)(UBXID_MON_VER);
    inputs[4] = (uint8_t)((byteCount-8));							//Length
    inputs[5] = (uint8_t)((byteCount-8)>>8);
    
    for(int i=6;i<byteCount-2;i++)
    {
        inputs[i] = ver[i-6];
    }

    calcUbxChecksum(&inputs[2], byteCount - 4);					//Checksum
        
    for(int j=0;j<byteCount;j++)
    {
        Serial1.write(inputs[j]);
    }
}
