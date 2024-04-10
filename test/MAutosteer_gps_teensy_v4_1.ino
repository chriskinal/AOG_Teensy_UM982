// Single antenna, IMU, & dual antenna code for AgOpenGPS
// If dual right antenna is for position (must enter this location in AgOpen), left Antenna is for heading & roll
//
// connection plan:
// Teensy Serial 7 RX (28) to F9P Position receiver TX1 (Position data)
// Teensy Serial 7 TX (29) to F9P Position receiver RX1 (RTCM data for RTK)
// Teensy Serial 2 RX (7) to F9P Heading receiver TX1 (Relative position from left antenna to right antenna)
// Teensy Serial 2 TX (8) to F9P Heading receiver RX1
// F9P Position receiver TX2 to F9P Heading receiver RX2 (RTCM data for Moving Base)
//
// Configuration of receiver
// Position F9P
// CFG-RATE-MEAS - 100 ms -> 10 Hz
// CFG-UART1-BAUDRATE 460800
// Serial 1 In - RTCM (Correction Data from AOG)
// Serial 1 Out - NMEA GGA
// CFG-UART2-BAUDRATE 460800
// Serial 2 Out - RTCM 1074,1084,1094,1230,4072.0 (Correction data for Heading F9P, Moving Base)  
// 1124 is not needed (China’s BeiDou system) - Save F9P brain power 
//
// Heading F9P
// CFG-RATE-MEAS - 100 ms -> 10 Hz
// CFG-UART1-BAUDRATE 460800
// Serial 1 Out - UBX-NAV-RELPOSNED
// CFG-UART2-BAUDRATE 460800
// Serial 2 In RTCM

/*
  * Modified for:
  *  Lexion 575R
  *    Analog input for feederhouse height WORK_SW trigger using factory height sensor
  *      Set workAnalogThresh & workAnalogHyst via user machine parameters in AoG
  *      Set WORK_ANALOG_PIN to correct analog input
*/

/************************* User Settings *************************/
// Serial Ports
#define SerialAOG Serial                //AgIO USB conection
#define SerialRTK Serial3               //RTK radio
HardwareSerial* SerialGPS = &Serial7;   //Main postion receiver (GGA) (Serial2 must be used here with T4.0 / Basic Panda boards - Should auto swap)
HardwareSerial* SerialGPS2 = &Serial2;  //Dual heading receiver 
HardwareSerial* SerialGPSTmp = NULL;
//HardwareSerial* SerialAOG = &Serial;

const int32_t baudAOG = 115200;
const int32_t baudGPS = 460800;
const int32_t baudRTK = 115200;     // most are using Xbee radios with default of 115200

// Baudrates for detecting UBX receiver
uint32_t baudrates[]
{
  4800,
  9600,
  19200,
  38400,
  57600,
  115200,
  230400,
  460800,
  921600
};

const uint32_t nrBaudrates = sizeof(baudrates)/sizeof(baudrates[0]);

#define ImuWire Wire        //SCL=19:A5 SDA=18:A4
#define RAD_TO_DEG_X_10 572.95779513082320876798154814105

//Swap BNO08x roll & pitch?
//const bool swapRollPitch = false;

const bool invertRoll= true;  //Used for IMU with dual antenna
#define baseLineLimit 5       //Max CM differance in baseline

#define REPORT_INTERVAL 20    //BNO report time, we want to keep reading it quick & offen. Its not timmed to anything just give constant data.
uint32_t READ_BNO_TIME = 0;   //Used stop BNO data pile up (This version is without resetting BNO everytime)

//Status LED's
#define GGAReceivedLED 13         //Teensy onboard LED
#define Power_on_LED 5            //Red
#define Ethernet_Active_LED 6     //Green
#define GPSRED_LED 9              //Red (Flashing = NO IMU or Dual, ON = GPS fix with IMU)
#define GPSGREEN_LED 10           //Green (Flashing = Dual bad, ON = Dual good)
#define AUTOSTEER_STANDBY_LED 11  //Red
#define AUTOSTEER_ACTIVE_LED 12   //Green
uint32_t gpsReadyTime = 0;        //Used for GGA timeout

//for v2.2
// #define Power_on_LED 22
// #define Ethernet_Active_LED 23
// #define GPSRED_LED 20
// #define GPSGREEN_LED 21
// #define AUTOSTEER_STANDBY_LED 38
// #define AUTOSTEER_ACTIVE_LED 39

/*****************************************************************/

// Ethernet Options (Teensy 4.1 Only)
#ifdef ARDUINO_TEENSY41
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

struct ConfigIP {
    uint8_t ipOne = 192;
    uint8_t ipTwo = 168;
    uint8_t ipThree = 5;
};  ConfigIP networkAddress;   //3 bytes

// IP & MAC address of this module of this module
byte Eth_myip[4] = { 0, 0, 0, 0}; //This is now set via AgIO
byte mac[] = {0x00, 0x00, 0x56, 0x00, 0x00, 0x78};

unsigned int portMy = 5120;             // port of this module
unsigned int AOGNtripPort = 2233;       // port NTRIP data from AOG comes in
unsigned int AOGAutoSteerPort = 8888;   // port Autosteer data from AOG comes in
unsigned int portDestination = 9999;    // Port of AOG that listens
char Eth_NTRIP_packetBuffer[512];       // buffer for receiving ntrip data

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Eth_udpPAOGI;     //Out port 5544
EthernetUDP Eth_udpNtrip;     //In port 2233
EthernetUDP Eth_udpAutoSteer; //In & Out Port 8888

IPAddress Eth_ipDestination;
#endif // ARDUINO_TEENSY41

byte CK_A = 0;
byte CK_B = 0;
int relposnedByteCount = 0;

//Speed pulse output
elapsedMillis speedPulseUpdateTimer = 0;
byte velocityPWM_Pin = 36;      // Velocity (MPH speed) PWM pin

#include "zNMEAParser.h"
#include <Wire.h>
#include "BNO08x_AOG.h"
#include <Streaming.h>
#include <FlexCAN_T4.h>
// CAN3 is CRX1/CTX1 on Teensy pin-out
// CAN2 is CRX2/CTX2 on Teensy pin-out
// CAN1 is CRX3/CTX3 on Teensy pin-out
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_256> Keya_Bus;    // CAN3 works for CRX1/CTX1 on PCB v4.1
float KeyaCurrentSensorReading = -1; //-1 means no Keya detected, data from Keya motor returns >-1
bool keyaDetected = false;

//Used to set CPU speed
extern "C" uint32_t set_arm_clock(uint32_t frequency); // required prototype

bool useDual = false;
bool dualReadyGGA = false;
bool dualReadyRelPos = false;

// booleans to see if we are using CMPS or BNO08x
bool useCMPS = false;
bool useBNO08x = false;

//CMPS always x60
#define CMPS14_ADDRESS 0x60

// BNO08x address variables to check where it is
const uint8_t bno08xAddresses[] = { 0x4A, 0x4B };
const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses) / sizeof(bno08xAddresses[0]);
uint8_t bno08xAddress;
BNO080 bno08x;

//Dual
double headingcorr = 900;  //90deg heading correction (90deg*10)
// Heading correction 180 degrees, because normally the heading antenna is in front, but we have it at the back
//double headingcorr = 1800;  // 180deg heading correction (180deg*10)

double baseline = 0;
double rollDual = 0;
double relPosD = 0;
double heading = 0;

byte ackPacket[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

constexpr int serial_buffer_size = 512;
uint8_t GPSrxbuffer[serial_buffer_size];    //Extra serial rx buffer
uint8_t GPStxbuffer[serial_buffer_size];    //Extra serial tx buffer
uint8_t GPS2rxbuffer[serial_buffer_size];   //Extra serial rx buffer
uint8_t GPS2txbuffer[serial_buffer_size];   //Extra serial tx buffer
uint8_t RTKrxbuffer[serial_buffer_size];    //Extra serial rx buffer

/* A parser is declared with 3 handlers at most */
NMEAParser<2> parser;

bool isTriggered = false;
bool blink = false;

bool Autosteer_running = true; //Auto set off in autosteer setup
bool Ethernet_running = false; //Auto set on in ethernet setup
bool GGA_Available = false;    //Do we have GGA on correct port?
uint32_t PortSwapTime = 0;

float roll = 0;
float pitch = 0;
float yaw = 0;

//Fusing BNO with Dual
double rollDelta;
double rollDeltaSmooth;
double correctionHeading;
double gyroDelta;
double imuGPS_Offset;
double gpsHeading;
double imuCorrected;
#define twoPI 6.28318530717958647692
#define PIBy2 1.57079632679489661923

// Buffer to read chars from Serial, to check if "!AOG" is found
uint8_t aogSerialCmd[4] = { '!', 'A', 'O', 'G'};
uint8_t aogSerialCmdBuffer[6];
uint8_t aogSerialCmdCounter = 0;

// Booleans to indictate to passthrough GPS or GPS2
bool passThroughGPS = false;
bool passThroughGPS2 = false;

//-=-=-=-=- UBX binary specific variables
struct ubxPacket
{
	uint8_t cls;
	uint8_t id;
	uint16_t len; //Length of the payload. Does not include cls, id, or checksum bytes
	uint16_t counter; //Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
	uint16_t startingSpot; //The counter value needed to go past before we begin recording into payload array
	uint8_t *payload; // We will allocate RAM for the payload if/when needed.
	uint8_t checksumA; //Given to us from module. Checked against the rolling calculated A/B checksums.
	uint8_t checksumB;
    
	////sfe_ublox_packet_validity_e valid;			 //Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
	////sfe_ublox_packet_validity_e classAndIDmatch; // Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID
};

// Setup procedure ------------------------
void setup()
{
    delay(500);                         //Small delay so serial can monitor start up
    //set_arm_clock(150000000);           //Set CPU speed to 150mhz
    //Serial.print("CPU speed set to: ");
    //Serial.println(F_CPU_ACTUAL);

  pinMode(GGAReceivedLED, OUTPUT);
  pinMode(Power_on_LED, OUTPUT);
  pinMode(Ethernet_Active_LED, OUTPUT);
  pinMode(GPSRED_LED, OUTPUT);
  pinMode(GPSGREEN_LED, OUTPUT);
  pinMode(AUTOSTEER_STANDBY_LED, OUTPUT);
  pinMode(AUTOSTEER_ACTIVE_LED, OUTPUT);

  // the dash means wildcard
  parser.setErrorHandler(errorHandler);
  parser.addHandler("G-GGA", GGA_Handler);
  parser.addHandler("G-VTG", VTG_Handler);

  delay(10);
  Serial.begin(baudAOG);
  delay(10);
  Serial.println("Start setup");

  SerialGPS->begin(baudGPS);
  SerialGPS->addMemoryForRead(GPSrxbuffer, serial_buffer_size);
  SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);

  delay(10);
  SerialRTK.begin(baudRTK);
  SerialRTK.addMemoryForRead(RTKrxbuffer, serial_buffer_size);

  delay(10);
  SerialGPS2->begin(baudGPS);
  SerialGPS2->addMemoryForRead(GPS2rxbuffer, serial_buffer_size);
  SerialGPS2->addMemoryForWrite(GPS2txbuffer, serial_buffer_size);

  Serial.println("SerialAOG, SerialRTK, SerialGPS and SerialGPS2 initialized");

  Serial.println("\r\nStarting AutoSteer...");
  autosteerSetup();
  
  Serial.println("\r\nStarting Ethernet...");
  EthernetStart();

  Serial.println("\r\nStarting IMU...");
  //test if CMPS working
  uint8_t error;

  ImuWire.begin();
  
  //Serial.println("Checking for CMPS14");
  ImuWire.beginTransmission(CMPS14_ADDRESS);
  error = ImuWire.endTransmission();

  if (error == 0)
  {
    //Serial.println("Error = 0");
    Serial.print("CMPS14 ADDRESs: 0x");
    Serial.println(CMPS14_ADDRESS, HEX);
    Serial.println("CMPS14 Ok.");
    useCMPS = true;
  }
  else
  {
    //Serial.println("Error = 4");
    Serial.println("CMPS not Connected or Found");
  }

  if (!useCMPS)
  {
      for (int16_t i = 0; i < nrBNO08xAdresses; i++)
      {
          bno08xAddress = bno08xAddresses[i];

          //Serial.print("\r\nChecking for BNO08X on ");
          //Serial.println(bno08xAddress, HEX);
          ImuWire.beginTransmission(bno08xAddress);
          error = ImuWire.endTransmission();

          if (error == 0)
          {
              //Serial.println("Error = 0");
              Serial.print("0x");
              Serial.print(bno08xAddress, HEX);
              Serial.println(" BNO08X Ok.");

              // Initialize BNO080 lib
              if (bno08x.begin(bno08xAddress, ImuWire)) //??? Passing NULL to non pointer argument, remove maybe ???
              {
                  //Increase I2C data rate to 400kHz
                  ImuWire.setClock(400000); 

                  delay(300);

                  // Use gameRotationVector and set REPORT_INTERVAL
                  bno08x.enableGameRotationVector(REPORT_INTERVAL);
                  useBNO08x = true;
              }
              else
              {
                  Serial.println("BNO080 not detected at given I2C address.");
              }
          }
          else
          {
              //Serial.println("Error = 4");
              Serial.print("0x");
              Serial.print(bno08xAddress, HEX);
              Serial.println(" BNO08X not Connected or Found");
          }
          if (useBNO08x) break;
      }
  }

  delay(100);
  Serial.print("\r\nuseCMPS = ");
  Serial.println(useCMPS);
  Serial.print("useBNO08x = ");
  Serial.println(useBNO08x);

  Serial.println("Right... time for some CANBUS! And, we're dedicated to Keya here");
  CAN_Setup();

  String sketchNameString = __BASE_FILE__;
  sketchNameString.remove(0, sketchNameString.lastIndexOf('\\')+1); // trim off beginning file path, from beginning of String to last back slash
  //sketchNameString.remove(sketchNameString.lastIndexOf('ino')-3, sketchNameString.length()); // trim off ending '.ino.cpp'
  sketchNameString.remove(sketchNameString.length() - 8);
  Serial.print("\r\n");
  Serial.println(sketchNameString);
  delay(500);

  Serial.println("\r\nEnd setup, waiting for GPS...\r\n");
}

void loop()
{
    KeyaBus_Receive();

    if (GGA_Available == false && !passThroughGPS && !passThroughGPS2)
    {
        if (systick_millis_count - PortSwapTime >= 10000)
        {
            //Serial.println("Swapping GPS ports...\r\n");
            SerialGPSTmp = SerialGPS;
            SerialGPS = SerialGPS2;
            SerialGPS2 = SerialGPSTmp;
            PortSwapTime = systick_millis_count;
        }
    }

    // Pass NTRIP etc to GPS
    if (SerialAOG.available())
    {
        uint8_t incoming_char = SerialAOG.read();

        // Check incoming char against the aogSerialCmd array
        // The configuration utility will send !AOGR1, !AOGR2 or !AOGED (close/end)
        if (aogSerialCmdCounter < 4 && aogSerialCmd[aogSerialCmdCounter] == incoming_char)
        {
            aogSerialCmdBuffer[aogSerialCmdCounter] = incoming_char;
            aogSerialCmdCounter++;
        }
        // Whole command prefix is in, handle it
        else if (aogSerialCmdCounter == 4)
        {
            aogSerialCmdBuffer[aogSerialCmdCounter] = incoming_char;
            aogSerialCmdBuffer[aogSerialCmdCounter + 1] = SerialAOG.read();

            if (aogSerialCmdBuffer[aogSerialCmdCounter] == 'R')
            {
                HardwareSerial* autoBaudSerial = NULL;

                // Reset SerialGPS and SerialGPS2
                SerialGPS = &Serial7;
                SerialGPS2 = &Serial2;

                if (aogSerialCmdBuffer[aogSerialCmdCounter + 1] == '1')
                {
                    passThroughGPS = true;
                    passThroughGPS2 = false;
                    autoBaudSerial = SerialGPS;
                }
                else if (aogSerialCmdBuffer[aogSerialCmdCounter + 1] == '2')
                {
                    passThroughGPS = false;
                    passThroughGPS2 = true;
                    autoBaudSerial = SerialGPS2;
                }
				
				const uint8_t UBX_SYNCH_1 = 0xB5;
                const uint8_t UBX_SYNCH_2 = 0x62;
                const uint8_t UBX_CLASS_ACK = 0x05;
                const uint8_t UBX_CLASS_CFG = 0x06;
                const uint8_t UBX_CFG_RATE = 0x08;

                ubxPacket packetCfg{};

                packetCfg.cls = UBX_CLASS_CFG;
                packetCfg.id = UBX_CFG_RATE;
                packetCfg.len = 0;
                packetCfg.startingSpot = 0;

                calcChecksum(&packetCfg);

                byte mon_rate[] = {0xB5, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                mon_rate[2] = packetCfg.cls; 
                mon_rate[3] = packetCfg.id; 
                mon_rate[4] = packetCfg.len & 0xFF; 
                mon_rate[5] = packetCfg.len >> 8;
                mon_rate[6] = packetCfg.checksumA; 
                mon_rate[7] = packetCfg.checksumB; 

                // Check baudrate
                bool communicationSuccessfull = false;
				uint32_t baudrate = 0;                

				for (uint32_t i = 0; i < nrBaudrates; i++)
				{
					baudrate = baudrates[i];

					Serial.print(F("Checking baudrate: "));
					Serial.println(baudrate);

					autoBaudSerial->begin(baudrate);
					delay(100);

					// first send dumb data to make sure its on
					autoBaudSerial->write(0xFF);

					// Clear
					while (autoBaudSerial->available() > 0)
					{
						autoBaudSerial->read();
					}

					// Send request
					autoBaudSerial->write(mon_rate, 8);

					uint32_t millis_read = systick_millis_count;
					constexpr uint32_t UART_TIMEOUT = 1000;
					int ubxFrameCounter = 0;
					bool isUbx = false;
					uint8_t incoming = 0;

					uint8_t requestedClass = packetCfg.cls;
					uint8_t requestedID = packetCfg.id;

					uint8_t packetBufCls = 0;
					uint8_t packetBufId = 0;

					do
					{
						while (autoBaudSerial->available() > 0)
						{
							incoming = autoBaudSerial->read();

							if (!isUbx && incoming == UBX_SYNCH_1) // UBX binary frames start with 0xB5, aka μ
							{
								ubxFrameCounter = 0;
								isUbx = true;
							}

							if (isUbx)
							{
								// Decide what type of response this is
								if ((ubxFrameCounter == 0) && (incoming != UBX_SYNCH_1))      // ISO 'μ'
								{
									isUbx = false;                                            // Something went wrong. Reset.
								}
								else if ((ubxFrameCounter == 1) && (incoming != UBX_SYNCH_2)) // ASCII 'b'
								{
									isUbx = false;                                            // Something went wrong. Reset.
								}
								else if (ubxFrameCounter == 1 && incoming == UBX_SYNCH_2)
								{
									// Serial.println("UBX_SYNCH_2");
									// isUbx should be still true
								}
								else if (ubxFrameCounter == 2) // Class
								{
									// Record the class in packetBuf until we know what to do with it
									packetBufCls = incoming; // (Duplication)
								}
								else if (ubxFrameCounter == 3) // ID
								{
									// Record the ID in packetBuf until we know what to do with it
									packetBufId = incoming; // (Duplication)

									// We can now identify the type of response
									// If the packet we are receiving is not an ACK then check for a class and ID match
									if (packetBufCls != UBX_CLASS_ACK)
									{
										// This is not an ACK so check for a class and ID match
										if ((packetBufCls == requestedClass) && (packetBufId == requestedID))
										{
											// This is not an ACK and we have a class and ID match
											communicationSuccessfull = true;
										}
										else
										{
											// This is not an ACK and we do not have a class and ID match
											// so we should keep diverting data into packetBuf and ignore the payload
											isUbx = false;
										}
									}
								}
							}

							// Finally, increment the frame counter
							ubxFrameCounter++;
						}
					} while (systick_millis_count - millis_read < UART_TIMEOUT);

					if (communicationSuccessfull)
					{
						break;
					}
				}

				if (communicationSuccessfull)
				{
					SerialAOG.write(aogSerialCmdBuffer, 6);
					SerialAOG.print(F("Found reciever at baudrate: "));
					SerialAOG.println(baudrate);

					// Let the configuring program know it can proceed
					SerialAOG.println("!AOGOK");
				}
				else
				{
					SerialAOG.println(F("u-blox GNSS not detected. Please check wiring."));
				}

				aogSerialCmdCounter = 0;
			}
            // END command. maybe think of a different abbreviation
            else if (aogSerialCmdBuffer[aogSerialCmdCounter] == 'E' && aogSerialCmdBuffer[aogSerialCmdCounter + 1] == 'D')
            {
                passThroughGPS = false;
                passThroughGPS2 = false;
                aogSerialCmdCounter = 0;
            }
        }
        else
        {
            aogSerialCmdCounter = 0;
		}

        if (passThroughGPS)
        {
            SerialGPS->write(incoming_char);
        }
        else if (passThroughGPS2)
        {
            SerialGPS2->write(incoming_char);
        }
        else
        {
            SerialGPS->write(incoming_char);
        }
    }

    // Read incoming nmea from GPS
    if (SerialGPS->available())
    {
        if (passThroughGPS)
        {
            SerialAOG.write(SerialGPS->read());
        }
        else
        {
            parser << SerialGPS->read();
        }
    }

    udpNtrip();

    // Check for RTK Radio
    if (SerialRTK.available())
    {
        SerialGPS->write(SerialRTK.read());
    }

    // If both dual messages are ready, send to AgOpen
    if (dualReadyGGA == true && dualReadyRelPos == true)
    {
        BuildNmea();
        dualReadyGGA = false;
        dualReadyRelPos = false;
    }

    // If anything comes in SerialGPS2 RelPos data
    if (SerialGPS2->available())
    {
        uint8_t incoming_char = SerialGPS2->read();  //Read RELPOSNED from F9P

        if (passThroughGPS2)
        {
            SerialAOG.write(incoming_char);
        }
        else
        {
            // Just increase the byte counter for the first 3 bytes
            if (relposnedByteCount < 4 && incoming_char == ackPacket[relposnedByteCount])
            {
                relposnedByteCount++;
            }
            else if (relposnedByteCount > 3)
            {
                // Real data, put the received bytes in the buffer
                ackPacket[relposnedByteCount] = incoming_char;
                relposnedByteCount++;
            }
            else
            {
                // Reset the counter, becaues the start sequence was broken
                relposnedByteCount = 0;
            }
        }
    }

    // Check the message when the buffer is full
    if (relposnedByteCount > 71)
    {
        if (calcChecksum())
        {
            //if(deBug) Serial.println("RelPos Message Recived");
            digitalWrite(GPSRED_LED, LOW);   //Turn red GPS LED OFF (we are now in dual mode so green LED)
            useDual = true;
            relPosDecode();
        }
        /*  else {
          if(deBug) Serial.println("ACK Checksum Failure: ");
          }
        */
        relposnedByteCount = 0;
    }

    //GGA timeout, turn off GPS LED's etc
    if((systick_millis_count - gpsReadyTime) > 10000) //GGA age over 10sec
    {
      digitalWrite(GPSRED_LED, LOW);
      digitalWrite(GPSGREEN_LED, LOW);
      useDual = false;
    }

    //Read BNO
    if((systick_millis_count - READ_BNO_TIME) > REPORT_INTERVAL && useBNO08x)
    {
      READ_BNO_TIME = systick_millis_count;
      readBNO();
    }
    
    if (Autosteer_running) autosteerLoop();
    else ReceiveUdp();
    
  if (Ethernet.linkStatus() == LinkOFF) 
  {
    digitalWrite(Power_on_LED, 1);
    digitalWrite(Ethernet_Active_LED, 0);
  }
  if (Ethernet.linkStatus() == LinkON) 
  {
    digitalWrite(Power_on_LED, 0);
    digitalWrite(Ethernet_Active_LED, 1);
  }
}//End Loop
//**************************************************************************

bool calcChecksum()
{
  CK_A = 0;
  CK_B = 0;

  for (int i = 2; i < 70; i++)
  {
    CK_A = CK_A + ackPacket[i];
    CK_B = CK_B + CK_A;
  }

  return (CK_A == ackPacket[70] && CK_B == ackPacket[71]);
}

//Given a message, calc and store the two byte "8-Bit Fletcher" checksum over the entirety of the message
//This is called before we send a command message
void calcChecksum(ubxPacket *msg)
{
  msg->checksumA = 0;
  msg->checksumB = 0;

  msg->checksumA += msg->cls;
  msg->checksumB += msg->checksumA;

  msg->checksumA += msg->id;
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len & 0xFF);
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len >> 8);
  msg->checksumB += msg->checksumA;

  for (uint16_t i = 0; i < msg->len; i++)
  {
    msg->checksumA += msg->payload[i];
    msg->checksumB += msg->checksumA;
  }
}
