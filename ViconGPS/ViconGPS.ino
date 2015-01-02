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


/*
||                  Interceptor              ||
||                                           ||
||  Demonstraion of MavLink packet           ||
||	recognition.							 ||



//=============//
	  Notes
//=============//

Assumes baud rates as follows;
Comms Serial: 57600
Debug Serial: 57600

Serial RX/TX connected to pins 14/15.
Connections must be made opposite to board labelling. i.e. Radio Tx -> Arduino Rx

At no point_s in the /loop/ function is data output via serial. Set an output if desired.

Ground pin 2 on boot to set this UAV as the leader

 - Kye Morton
*/


#define MUTE_SYS_STATUS 1												//Set to '1' to disable system monitoring serial output
#define MUTE_DEBUG_STATUS 1												//Set to '1' to disable debug monitoring serial output
#define MUTE_RAW_STATUS 1												//Set to '1' to disable raw monitoring debug serial output

#define UINT8_MAX 0xFF
#define UINT16_MAX 0xFFFF
#define UINT32_MAX 0xFFFFFFFF
#define UINT64_MAX 0xFFFFFFFFFFFFFFFFULL
#define FLOAT_MAX_HEX 0x7F800000

#define ALLOW_DYNAMIC_SYS 1												//Set to 1 to allow the first heartbeat packets to define the system IDs
#define THIS_UAV_SYS_CONST 0x01											//All these settings will be overwritten if ALLOW_DYNAMIC_SYS is set to 1
#define LEAD_UAV_SYS_CONST 0x00											//	||
#define GCS_SYS_CONST 0xFF												//	\/
#define THIS_UAV_SWARM_POS 0;											//  --

#define SWARM_OFFSET_DISTANCE 10										//Offset distance between last swarm id and this swarm location
#define SWARM_OFFSET_PHI pi/2											//Offset angle between last swarm id and this swarm location
#define SWARM_OFFSET_HEIGHT	10											//Offset vertical between last swarm id and this swarm location

#define LEADER_ID 0

#define COMP_ID_PATH_uint 196											

#define GPS_PAYLOAD_LENGTH 36
#define CMD_LONG_LENGTH 33
#define ATT_PAYLOAD_LENGTH 28
#define CMD_LEN_VICON_DATA 32

#define CMD_ID_SWARM_POSITION 200
#define CMD_ID_HEARTBEAT 0
#define CMD_ID_ATTITUDE 30
#define CMD_ID_GPS_FORMATTED 33
#define CMD_ID_COMMAND_LONG 76
#define CMD_ID_RAW_IMU 27
#define CMD_ID_SCALED_PRESSURE 29
#define CMD_ID_SYS_STATUS 1
#define CMD_ID_MISSION_CURRENT 42
#define CMD_ID_GPS_RAW_INT 24
#define CMD_ID_VFR_HUD 74
#define CMD_ID_SERVO_OUTPUT_RAW 36
#define CMD_ID_RC_CHANNELS_RAW 35
#define CMD_ID_SYSTEM_TIME 2
#define CMD_ID_VICON_DATA 104
#define CMD_ID_SET_SERVO 183

#define MAV_FRAME_GLOBAL_RELATIVE_ALT 3
#define MAV_MODE_FLAG_DECODE_POSITION_GUIDED 8
#define MAV_CMD_OVERRIDE_GOTO 252
#define MAV_GOTO_DO_HOLD 0 												//Hold at the current position
#define MAV_GOTO_DO_CONTINUE 1 											//Continue with the next item in mission execution
#define MAV_GOTO_HOLD_AT_CURRENT_POSITION 2 							//Hold at the current position of the system 
#define MAV_GOTO_HOLD_AT_SPECIFIED_POSITION 3 							//Hold at the position specified in the parameters of the DO_HOLD action

#define MAV_MODE_STABILIZE_ARMED 208 									//System is allowed to be active, under assisted RC control
#define MAV_MODE_GUIDED_ARMED 216

#define MAV_FRAME_GLOBAL 0

#define X25_INIT_CRC 0xffff
#define X25_VALIDATE_CRC 0xf0b8

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
#define UBXID_REC_OK 0x0A21 

#define UBX_BAUD 0x00009600

//=============//
//	Includes   //
//=============//

#include <math.h>
//#include <SoftwareSerial.h>
#include <Servo.h>

//=============//
//	Variables  //
//=============//

/*struct msgBuff_s														//Temporary Storage While Reading Messages
{
	uint32_t byteCount;
	uint8_t *inputs;
	uint8_t expByteLeft;
	bool phraseInProgress;
	uint8_t id;
};*/

struct msgBuff_s														//Temporary Storage While Reading Messages
{
	uint32_t byteCount;
	uint8_t inputs[263];
	uint8_t expByteLeft;
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

//Comms
void sendMessage(msgBuff_s outMsg, uint8_t serialNum);
void printHex(uint8_t c, uint8_t serialNum);
uint8_t comm_receive();

//Mavlink
uint8_t phraseMavlinkMessage(uint8_t c, msgBuff_s &inMsg);
static uint16_t crc_calculate(uint8_t *pBuffer, int32_t length);
static uint16_t crc_accumulate(uint8_t b, uint16_t crc);
void decodeMavLinkMessage(msgBuff_s &msgIn);

//UBX
uint8_t phraseUBXMessage(uint8_t c, msgBuff_s &inMsg);
void calcUbxChecksum(uint8_t *checksumPayload, uint8_t payloadSize);
void decodeUBXMessage(msgBuff_s &msgIn);

//Decoding
void packFloat(msgBuff_s &msg, float num, uint32_t offset);
void packUint32(msgBuff_s &msg, uint32_t num, uint32_t offset);
void packUBXUint32(msgBuff_s &msg, uint32_t num, uint32_t offset);
void packUBXUint16(msgBuff_s &msg, uint32_t num, uint32_t offset);

//Resets
void clearMsgBuff(msgBuff_s &msg);
void clearGPSData(gpsData_s &data);

//GPS Spoofing
void getGPSData(msgBuff_s &msg, gpsData_s &data);
void spoofGPS(gpsData_s data);

//UBX Functions
void GPSToECEF(int32_t &XECEF, int32_t &YECEF, int32_t &ZECEF, int32_t lattitude, int32_t longitude, int32_t altitude);
void compileUBXPosllh(msgBuff_s &msg, GPS_UBX_NAV_POSLLH_s data);
void compileUBXStatus(msgBuff_s &msg, GPS_UBX_NAV_STATUS_s data);
void compileUBXVelned(msgBuff_s &msg, GPS_UBX_NAV_VELNED_s data);
void compileUBXSol(msgBuff_s &msg, GPS_UBX_NAV_SOL_s data);
void compileUBXPvt(msgBuff_s &msg, GPS_UBX_NAV_PVT_s data);
void compileUBXMonHW(msgBuff_s &msg, GPS_UBX_MON_HW_s data);
void sendUBXAck(uint16_t ids);
void sendUBXNack(uint16_t ids);
void sendInitial();
void sendMonitorVersion();
void SendBadGPS(gpsData_s data);

//=============//
//	Variables  //
//=============//
//Message CRC seeds. Each ID has a corresponding seed in this array. Heartbeat(ID:0)=50, Swarm ID (ID:200)=72
const uint8_t MAVLINK_MESSAGE_CRCS[] = {50, 124, 137, 0, 237, 217, 104, 119, 0, 0, 0, 89, 0, 0, 0, 0, 0, 0, 0, 0, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246, 185, 104, 237, 244, 222, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153, 41, 39, 214, 223, 141, 33, 15, 3, 100, 24, 239, 238, 0, 0, 183, 0, 130, 0, 148, 21, 0, 52, 124, 0, 0, 0, 20, 0, 152, 143, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 183, 63, 54, 0, 0, 0, 0, 0, 0, 0, 19, 102, 158, 208, 56, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 134, 219, 208, 188, 84, 22, 19, 21, 134, 0, 78, 68, 189, 127, 42, 21, 21, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 72, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 204, 49, 170, 44, 83, 46, 247};

uint16_t crcAccum = 0;

const double pi = 3.1415926535897;

msgBuff_s inMsgSerial1;
msgBuff_s inMsgSerial2; // RX, TX

uint8_t rxPin = 2;
uint8_t txPin = 3;


//SoftwareSerial Serial2(rxPin, txPin); // RX, TX

uint16_t tempUBX[6];

gpsData_s leadUAVgps;													//Used to catch vicon data and then spoof the GPS

unsigned long startTimeMilli = 0;

bool usePVT = 0;
bool turnOFF = 0;

unsigned long firstViconFrame = 0;
bool viconReceived = false;
bool LEDStatus = false;
unsigned int LEDPin = 13;

unsigned int LEDPinPVT = 6;
unsigned int LEDPinVicon = 7;

bool MUTE = MUTE_SYS_STATUS;
bool MUTE_DEBUG = MUTE_DEBUG_STATUS;
bool MUTE_RAW = MUTE_RAW_STATUS;

//unit8_t * tmp_buff;

Servo myservo;
int pos = 0;

//=============//
//	  Main     //
//=============//
void setup()
{
        myservo.attach(41);
	startTimeMilli = millis();
	
	Serial.begin(57600);												//Monitoring Link

	if(!MUTE)
	{
		Serial.println("Opening communication serial read on Serial Port 0...");
		Serial.println();
	}

	Serial1.begin(57600);												//Monitoring Link

	if(!MUTE)
	{
		Serial.println("Opening communication serial read on Serial Port 1...");
		Serial.println();
	}

	//pinMode(rxPin, INPUT);
	//pinMode(txPin, OUTPUT);
	
	Serial2.begin(38400);												//GPS Link
	
	if(!MUTE)
	{
		Serial.println("Opening communication serial read on Software Serial Port...");
		Serial.println();
	}

	pinMode(LEDPin, OUTPUT);
	digitalWrite(LEDPin, LOW);
	
        pinMode(LEDPinPVT, OUTPUT);
	digitalWrite(LEDPinPVT, LOW);

        pinMode(LEDPinVicon, OUTPUT);
	digitalWrite(LEDPinVicon, LOW);

	clearMsgBuff(inMsgSerial1);
	inMsgSerial1.id = 3;
	clearMsgBuff(inMsgSerial2);
	inMsgSerial2.id = 1;

	//clearGPSData(leadUAVgps);
	//delay(2000);
	sendInitial();
}

void loop()
{
	unsigned long startTimeLoop = millis();
	unsigned long startTimeSecs = (startTimeLoop - startTimeMilli)/1000;
      
	if(!MUTE)
	{
		Serial.print("\nSystem has been on-line for ");
		Serial.print(startTimeSecs);
		Serial.println(" seconds.");
	}
	
	while (millis() - startTimeLoop < 1000)								//Holds the program for 1 second to catch new data from the serial buffer
	{
		unsigned long startTimeLoopInner = millis();

		while (millis() - startTimeLoopInner < 200)								//Holds the program for 0.2 seconds to catch new data from the serial buffer
		{
			switch(comm_receive())
			{
				case 1:
				{				
					clearMsgBuff(inMsgSerial1);
					//Serial.println("Serial 1 buffer cleared"); //remove
					break;
				}
				case 2:
				{
					clearMsgBuff(inMsgSerial2);
					//Serial.println("Serial 2 buffer cleared"); //remove
					break;
				}
				default:
				{
					break;
				}
			}
                    //delay(150);
		}
                
                //Fakeing code
                /*
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
                //delay(200);
	        */
        if(leadUAVgps.newData){
      		spoofGPS(leadUAVgps);
                LEDStatus = true;
                leadUAVgps.newData = false;
                digitalWrite(LEDPinVicon, HIGH);
        } else {
          	//send badgps health
        	SendBadGPS(leadUAVgps);
                LEDStatus = false;
                digitalWrite(LEDPinVicon, LOW);
        }
	       
	}

        if(usePVT){
          digitalWrite(LEDPinPVT, HIGH);
        } else {
          digitalWrite(LEDPinPVT, LOW);
        }

	//LEDStatus = true;
	
	//if(!viconReceived)
	//{
	//	viconReceived = true;
	//	firstViconFrame = millis();
	//}
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


//========//
// Serial //
//========//
void sendMessage(msgBuff_s outMsg, uint8_t serialNum)
{
	uint32_t i = 0;
	uint8_t c = 0xFE;
	
	if(!MUTE_RAW)														//Writes new line for debug packet output
	{		
		switch(serialNum)
		{
			case 0:
				Serial.print("\nSerial 0 | ");	
				break;
			case 1:
				Serial.print("\nSerial 1 | ");	
				break;
			case 2:
				Serial.print("\nSerial 2 | ");	
				break;
		}
	}
		
	for(i=0;i<outMsg.byteCount;i++)
	{
		c = outMsg.inputs[i];
		
		switch(serialNum)
		{
			case 0:
				printHex(c, 0);
				Serial.print(" ");
				break;
			case 1:
				Serial1.write(c);
				break;
			case 2:
				Serial2.write(c);
				break;
		}
	}
}

void printHex(uint8_t c, uint8_t serialNum) 
{
	if(c<0x10)
	{ 
		Serial.print("0");
	}		
	
	switch(serialNum)
	{
		case 0:
			Serial.print(c, HEX);
			break;
		case 1:
			break;
		case 2:
			break;
	}	
}

uint8_t comm_receive()												//Reads in all available data in the serial buffer and passes it off to to be phrased
{
	
	//bool m1Com = 0;

	//bool m2Com = 0;
        //will allow this bit to termiate early - needs to be tested.....
        unsigned long startTimeLoopInner = millis();

	while(((Serial1.available() > 0)||(Serial2.available() > 0)))// && (millis() - startTimeLoopInner < 200)) 
	{
  					//Holds the program for 1 second to catch new data from the serial buffer
		//unsigned long startTimeLoopInner = millis();
                //Serial.println("Time at start loop:");
                //Serial.println(startTimeLoopInner);
		//if(!usePVT){
		  if(Serial1.available() > 0)//&& (m1Com != 1)
		  {
			uint8_t c = Serial1.read();

			if(phraseMavlinkMessage(c, inMsgSerial1)>0)					//Will return 1 when a message has completed being read.
			{		
				return 1;
			}
		  }
                //}
		
                //Serial.println("Time at mid loop:");
                //Serial.println(millis());
                if(!turnOFF){
		  if(Serial2.available() > 0)//&& (m2Com != 1)
		  {
			uint8_t d = Serial2.read();

			//Serial.print("receiving byte: ");
			//printHex(d,0);
			//Serial.println("");

			if(phraseUBXMessage(d, inMsgSerial2)>0)					//Will return 1 when a message has completed being read.
			{		
				//return 1;
				return 2;// I believe this should be 2 not 1
			}
		  }
                }

		//if((inMsgSerial1.phraseInProgress == 0 )&& (inMsgSerial2.phraseInProgress == 0){

		//	return 0;
		//}
	}
		
	return 0;
}

//===============//
// Mavlink Comms //
//===============//

uint8_t phraseMavlinkMessage(uint8_t c, msgBuff_s &inMsg)
{
	bool messageComplete = 0;
	bool goodMessage = 0;
	uint16_t crcTemp = 0;
	uint16_t crcCheck = 0;
	int32_t i = 0;

	if((c==0xFE)&&(!inMsg.phraseInProgress))						//If it a header byte (0xFE) is detected, and there is no message currently being phrased
	{
        clearMsgBuff(inMsg);
		inMsg.phraseInProgress = 1;
                
	}
	
	if(inMsg.phraseInProgress==1)
	{
		inMsg.inputs[inMsg.byteCount++] = c;						//Appends the newest byte to the array
		
		//inMsg.byteCount++;

		if(inMsg.byteCount>=(inMsg.inputs[1]+8))					//The expected packet length has been collected. "inMsg.inputs[1]" is "len", +8 for rest of header
		{	
			inMsg.phraseInProgress = 0;
			messageComplete = 1;
		}
	}
	
	if(messageComplete)
	{
                //Serial.println("Mavlink Message Complete");
  
		crcTemp = crc_calculate(inMsg.inputs, inMsg.byteCount-2);	//Calculate and CRC for the gathered messsage, excluding the sent CRC
		
		if (inMsg.byteCount > 5 && inMsg.inputs[0] == 254)			//Make sure there is at least a header, and the first byte is 0xFE
		{
			crcTemp = crc_accumulate(MAVLINK_MESSAGE_CRCS[inMsg.inputs[5]], crcTemp);	//Use magic to add to the crcTemp value
		}

		crcCheck = (inMsg.inputs[inMsg.byteCount-1] << 8) | (inMsg.inputs[inMsg.byteCount-2]);	//Gather the value of the given CRC
		
		if(crcTemp==crcCheck)										//If the CRCs match, message is valid
		{	
			c = 0;
			
			decodeMavLinkMessage(inMsg);
			goodMessage = 1;
		}
		else
		{
			if(!MUTE_DEBUG)
			{
				Serial.print("Invalid checksum...");
				sendMessage(inMsg, 0);
                                Serial.println();
			}
		}
	}
		
	return goodMessage;														//Discard message, as it is not valid
}

static uint16_t crc_calculate(uint8_t* pBuffer, int32_t length)	//Magic
{
	if (length < 1)
	{
		return 0xffff;
	}

	uint16_t crcTmp;
	int32_t i;

	crcTmp = X25_INIT_CRC;

	for (i = 1; i < length; i++) // skips header
	{
		crcTmp = crc_accumulate(pBuffer[i], crcTmp);
	}

	return (crcTmp);
}

static uint16_t crc_accumulate(uint8_t b, uint16_t crc)		//More Magic
{
		uint8_t ch = (uint8_t)(b ^ (uint8_t)(crc & 0x00ff));
		ch = (uint8_t)(ch ^ (ch << 4));
		return (uint16_t)((crc >> 8) ^ (ch << 8) ^ (ch << 3) ^ (ch >> 4));
}

void decodeMavLinkMessage(msgBuff_s &msgIn)
{
        //Serial.println("decoding Mavlink message ");
	uint8_t sys = msgIn.inputs[3];
	uint8_t cmp = msgIn.inputs[4];
	uint8_t id = msgIn.inputs[5];
	
	switch(id)														//Add case's to this to give options for different id's to be recognised
	{
		case CMD_ID_VICON_DATA:											//Global formatted GPS data
		{
			if(!MUTE_DEBUG)
			{
				Serial.println("VICON MavLink message detected...");
			}
			
			if(!viconReceived)
			{
				viconReceived = true;
				firstViconFrame = millis();
			}
			getGPSData(msgIn, leadUAVgps);
			
			break;
		}
		case CMD_ID_HEARTBEAT:										//Heartbeat
		{
			break;
		}
		case CMD_ID_ATTITUDE:										//Global formatted GPS data
		{
			break;
		}
		case CMD_ID_GPS_FORMATTED:									//Global formatted GPS data
		{		
			break;
		}
		case CMD_ID_RAW_IMU:
		{
			break;
		}
		case CMD_ID_SCALED_PRESSURE:
		{
			break;
		}
		case CMD_ID_SYS_STATUS:
		{
			break;
		}
		case CMD_ID_MISSION_CURRENT:
		{
			break;
		}
		case CMD_ID_VFR_HUD:
		{
			break;
		}
		case CMD_ID_SERVO_OUTPUT_RAW:
		{
			break;
		}
		case CMD_ID_RC_CHANNELS_RAW:
		{
			break;
		}
		case CMD_ID_SYSTEM_TIME:
		{		
			break;
		}
                case CMD_ID_SET_SERVO:
                {
                  Serial.println("Set Servo message detected from this UAV");
                        for(pos = 0; pos < 180; pos += 1)  // goes from 0 degrees to 180 degrees 
                        {                                  // in steps of 1 degree 
                          myservo.write(pos);              // tell servo to go to position in variable 'pos' 
                          delay(15);                       // waits 15ms for the servo to reach the position 
                        } 
                        break;
                }
                case CMD_ID_COMMAND_LONG:
                {
                  Serial.println("Set Servo message detected from this UAV");
                        for(pos = 0; pos < 180; pos += 1)  // goes from 0 degrees to 180 degrees 
                        {                                  // in steps of 1 degree 
                          myservo.write(pos);              // tell servo to go to position in variable 'pos' 
                          delay(15);                       // waits 15ms for the servo to reach the position 
                        } 
                        break;
                }
		default:
		{
			if(!MUTE)
			{
				Serial.println("Undefined message detected from this UAV");
				Serial.print("Message: ");
				sendMessage(msgIn, 0);					
			}
			
			break;
		}
	}
}


//============//
// UBX Comms  //
//============//

uint8_t phraseUBXMessage(uint8_t c, msgBuff_s &inMsg)
{
	bool messageComplete = 0;
	uint16_t crcTemp = 0;
	uint16_t crcCheck = 0;
	int i = 0;
	bool goodMessage = 0;
	
	if((c==(UBX_HEAD_ID>>8))&&(!inMsg.phraseInProgress))							//If it a header byte (0xB5) is detected, and there is no message currently being phrased
	{
        clearMsgBuff(inMsg);											//Reset message buffer and start recording message
		inMsg.phraseInProgress = 1;
		//Serial.println("New message incoming... ");
		//Serial.print(inMsg.byteCount,DEC);
		//Serial.print(" ");
	}
	
	if(inMsg.phraseInProgress==1)
	{
		if(inMsg.byteCount<=5)
		{
			tempUBX[inMsg.byteCount] = c;

			//Serial.print("Current Byte: ");
			//Serial.print(inMsg.byteCount, DEC);
			//Serial.print(" ");
			//printHex(c,0);
			//Serial.println(" ");
		}
		
		if(inMsg.byteCount==5)
		{
			if(((((uint16_t)tempUBX[4])|((uint16_t)tempUBX[5]<<8))+8)>0xFF)
			{
				inMsg.phraseInProgress = 0;
				messageComplete = 0;
				//Serial.println("Bad Message: too big");
			}
			
			if(inMsg.phraseInProgress)
			{

				uint16_t temp = (((uint16_t)tempUBX[4])|((uint16_t)tempUBX[5]<<8));

				//Serial.print("Length: ");
				//printHex(tempUBX[4],0);
				//printHex(tempUBX[5],0);
				//Serial.println("");

				//Serial.print("calculated length: ");
				//Serial.print(temp, DEC);

				//Serial.println("");

				//inMsg.inputs = (uint8_t*)malloc((temp + 8)*sizeof(uint8_t));

				//if(inMsg.inputs == NULL){

				//	Serial.println("memory allocation failed");
				//}
				
				//Serial.println("Assign 6 bytes to input");
				for(int i=0;i<6;i++)
				{
					inMsg.inputs[i] = tempUBX[i];
					//printHex(inMsg.inputs[i],0);
				}
				//Serial.println("");
			}
		}
		
		if(inMsg.byteCount>=6)
		{
			inMsg.inputs[inMsg.byteCount] = c;	
		}							//Appends the newest byte to the array
		
		inMsg.byteCount++;

                //Serial.print("Message Byte Count: ");
	        //Serial.println(inMsg.byteCount,DEC);

		if((inMsg.byteCount>=((((uint16_t)inMsg.inputs[4])|((uint16_t)inMsg.inputs[5]<<8))+8)))//&&(inMsg.byteCount>=8))
		{	
			inMsg.phraseInProgress = 0;
			messageComplete = 1;
			//Serial.println("Message Complete");
		}
	}
	
	if(messageComplete)
	{

		if(!MUTE_DEBUG)
		{
			//Serial.print("Phrasing Message...");
			Serial.println();
		}
	
		crcTemp = ((uint16_t)inMsg.inputs[inMsg.byteCount-2])|((uint16_t)inMsg.inputs[inMsg.byteCount-1]<<8);	//Store the checksum gathered from the message
		
		calcUbxChecksum(&inMsg.inputs[2], inMsg.byteCount - 4);		//Recalculate the checksum

		crcCheck = ((uint16_t)inMsg.inputs[inMsg.byteCount-2])|((uint16_t)inMsg.inputs[inMsg.byteCount-1]<<8);	//Gather the value of the given CRC
		
		if(crcTemp==crcCheck)										//If the CRCs match, message is valid
		{	
			c = 0;
			
			decodeUBXMessage(inMsg);
			goodMessage = 1;
			//Serial.println("Good Message");
		}		
		else
		{
			if(!MUTE_DEBUG)
			{
				Serial.print("Invalid checksum...");
				Serial.println();
			}
		}
	}
	return goodMessage;												//Discard message, as it is not valid
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

void decodeUBXMessage(msgBuff_s &msgIn)
{
	uint16_t id = ((uint16_t)msgIn.inputs[2]<<8)|((uint16_t)msgIn.inputs[3]);
	
	if(!MUTE_RAW)
	{
		Serial.print("Receiving: ");
		sendMessage(msgIn,0);
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
					sendMessage(msgIn, 0);
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
                                        //digitalWrite(LEDPinPVT, HIGH);
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
						sendMessage(msgIn, 0);
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
                        turnOFF = true;
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


//==========//
// Decoding //
//==========//
void packFloat(msgBuff_s &msg, float num, uint32_t offset)		//Packs a uint32 into the payload at a byte offset of "offset" from the start of the payload
{
	uint32_t hex = *((uint32_t*)&num);

	for(int i=0;i<4;i++)
	{
		msg.inputs[offset+6+i] = (uint8_t)(hex>>(8*i));			//+6 to account for the header
	}
}

void packUint32(msgBuff_s &msg, uint32_t num, uint32_t offset)		//Packs a uint32 into the payload at a byte offset of "offset" from the start of the payload
{
	for(int i=0;i<4;i++)
	{
		msg.inputs[offset+6+i] = (uint8_t)(num>>(8*i));			//+6 to account for the header
	}
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


//==========//
//	Resets  //
//==========//

void clearMsgBuff(msgBuff_s &msg)									//Resets the provided buffer struct
{
	msg.byteCount = 0;
	msg.phraseInProgress = 0;
	msg.expByteLeft = 0;
	
	//free(msg.inputs);

	//if(msg.inputs == NULL){

	//	Serial.println("Buffer Cleared");
	//}
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


//=================//
//	GPS Spoofing   //
//=================//

void getGPSData(msgBuff_s &msg, gpsData_s &data)						//Uses the last GPS message data collected  and stores them as the actual float variables
{
	clearGPSData(data);
	
	data.upTime = (uint64_t)(((uint64_t)msg.inputs[13]<<56)|((uint64_t)msg.inputs[12]<<48)|((uint64_t)msg.inputs[11]<<40)|((uint64_t)msg.inputs[10]<<32)|((uint64_t)msg.inputs[9]<<24)|((uint64_t)msg.inputs[8]<<16)|((uint64_t)msg.inputs[7]<<8)|((uint64_t)msg.inputs[6]));
	uint32_t x = (((uint32_t)msg.inputs[17]<<24)|((uint32_t)msg.inputs[16]<<16)|((uint32_t)msg.inputs[15]<<8)|((uint32_t)msg.inputs[14]));
	uint32_t y = (((uint32_t)msg.inputs[21]<<24)|((uint32_t)msg.inputs[20]<<16)|((uint32_t)msg.inputs[19]<<8)|((uint32_t)msg.inputs[18]));
	uint32_t z = (((uint32_t)msg.inputs[25]<<24)|((uint32_t)msg.inputs[24]<<16)|((uint32_t)msg.inputs[23]<<8)|((uint32_t)msg.inputs[22]));
	uint32_t r = (((uint32_t)msg.inputs[29]<<24)|((uint32_t)msg.inputs[28]<<16)|((uint32_t)msg.inputs[27]<<8)|((uint32_t)msg.inputs[26]));
	uint32_t p = (((uint32_t)msg.inputs[33]<<24)|((uint32_t)msg.inputs[32]<<16)|((uint32_t)msg.inputs[31]<<8)|((uint32_t)msg.inputs[30]));
	uint32_t q = (((uint32_t)msg.inputs[37]<<24)|((uint32_t)msg.inputs[36]<<16)|((uint32_t)msg.inputs[35]<<8)|((uint32_t)msg.inputs[34]));
	
	data.x = *((float*)&x);
	data.y = *((float*)&y);
	data.z = *((float*)&z);
	data.r = *((float*)&r);
	data.p = *((float*)&p);
	data.q = *((float*)&q);       

        // need to assign old data instead of new if its bad gps
	
	//if((x!=FLOAT_MAX_HEX)&&(y!=FLOAT_MAX_HEX)&&(z!=FLOAT_MAX_HEX)&&(r!=FLOAT_MAX_HEX)&&(p!=FLOAT_MAX_HEX)&&(q!=FLOAT_MAX_HEX))
	if((data.x!=FLOAT_MAX_HEX)&&(data.y!=FLOAT_MAX_HEX)&&(data.z!=FLOAT_MAX_HEX)&&(data.r!=FLOAT_MAX_HEX)&&(data.p!=FLOAT_MAX_HEX)&&(data.q!=FLOAT_MAX_HEX))
        {
		data.newData = true;
	}
	else
	{
		data.newData = false;
                data.x = data.xold;
	        data.y = data.yold;
	        data.z = data.zold;
	}
}


void spoofGPS(gpsData_s data)
{
	if(!MUTE_DEBUG)
	{
		Serial.println("Transmitting spoofed data...");
	}
	
	msgBuff_s msg;
	clearMsgBuff(msg);
	
	uint32_t gpsTimeOfWeek = millis();
	
	if(!usePVT)
	{/*
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
		sendMessage(msg, 2);
		   
		if(!MUTE_RAW)
		{
			Serial.print("Sending GPSPosllhData: ");
			sendMessage(msg, 0);
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
		sendMessage(msg, 2);

		if(!MUTE_RAW)
		{
			Serial.print("Sending GPSStatusData:  ");
			sendMessage(msg, 0);
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
		sendMessage(msg, 2);

		if(!MUTE_RAW)
		{
			Serial.print("Sending GPSVelnedData: ");
			sendMessage(msg, 0);
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
		sendMessage(msg, 2);	

		if(!MUTE_RAW)
		{
			Serial.print("Sending GPSSolData: ");
			sendMessage(msg, 0);
			Serial.println();
		}*/
	}
	else
	{
		GPS_UBX_NAV_PVT_s GPSPvtData;
		GPS_UBX_MON_HW_s GPSHWData;

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
		sendMessage(msg, 2);

		if(!MUTE_RAW)
		{
			//Serial.print("Sending PVT: ");
			//sendMessage(msg, 0);
			//Serial.println();
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
		sendMessage(msg, 2);

		if(!MUTE_RAW)
		{
			//Serial.print("Sending MonHW: ");
			//sendMessage(msg, 0);
			//Serial.println();
		}
	}
}

void GPSToECEF(int32_t &XECEF, int32_t &YECEF, int32_t &ZECEF, int32_t lattitude, int32_t longitude, int32_t altitude)
{
    double a = 6378137;

    double e = 8.1819190842622e-2;

    double asq = a*a;
    double esq = e*e;

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
	//msg.inputs = (uint8_t*)malloc(msg.byteCount*sizeof(uint8_t));
	
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
	//msg.inputs = (uint8_t*)malloc(msg.byteCount*sizeof(uint8_t));
	
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
	//msg.inputs = (uint8_t*)malloc(msg.byteCount*sizeof(uint8_t));
	
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
	//msg.inputs = (uint8_t*)malloc(msg.byteCount*sizeof(uint8_t));
	
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
	//msg.inputs = (uint8_t*)malloc(msg.byteCount*sizeof(uint8_t));
	
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
	//msg.inputs = (uint8_t*)malloc(msg.byteCount*sizeof(uint8_t));
	
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
          Serial2.write(inputs[i]);
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
          Serial2.write(inputs[i]);
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
        Serial2.write(inputs[i]);
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
        Serial2.write(inputs[j]);
    }
}

void SendBadGPS(gpsData_s data){
	
	msgBuff_s msg;
	clearMsgBuff(msg);

	uint32_t gpsTimeOfWeek = millis();

	GPS_UBX_NAV_PVT_s GPSPvtData;
	GPS_UBX_MON_HW_s GPSHWData;

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
	GPSPvtData.fix = 0x00;
	GPSPvtData.flag = 0x00;
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
	sendMessage(msg, 2);

	if(!MUTE_RAW)
	{
	//Serial.print("Sending PVT: ");
	//sendMessage(msg, 0);
	//Serial.println();
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
	sendMessage(msg, 2);

	if(!MUTE_RAW)
	{
	//	Serial.print("Sending MonHW: ");
	//	sendMessage(msg, 0);
	//	Serial.println();
	} 
}
