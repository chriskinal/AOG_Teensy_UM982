// UM982 Connection plan:
// Teensy Serial 7 RX (28) to F9P Position receiver TX1 (Position data)
// Teensy Serial 7 TX (29) to F9P Position receiver RX1 (RTCM data for RTK)
//
#include "zNMEAParser.h"
#include "UM982_Parser.h"
#include "Adafruit_BNO08x_RVC.h"
#include <elapsedMillis.h>
#include <SimpleKalmanFilter.h>

// Ethernet Options (Teensy 4.1 Only)
#ifdef ARDUINO_TEENSY41
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#endif // ARDUINO_TEENSY41

/************************* User Settings *************************/
bool udpPassthrough = false;  // False = GPS neeeds to send GGA, VTG & HPR messages. True = GPS needs to send KSXT messages only.
bool makeOGI = false;         //Set to true to make PAOGI messages. Else PNADA message will be made.
bool baseLineCheck = false;   //Set to true to use IMU fusion with UM982
const bool invertRoll= true;  //Used for IMU with dual antenna
#define baseLineLimit 5       //Max CM differance in baseline
byte report_interval = 100; // Currently AOG supports up to 10Hz. Anything below 100ms may produce unpredictable results in AOG.

// Heading correction can be enetered into the UM982 config or AOG GUI so this can be 0. If not in UM982 config or AOG GUI, set here.
// Negative number = west, positive number = east.
double headingcorr = 0;
// double headingcorr = 900;  //90deg heading correction (90deg*10)
// Roll correction can be entered in the AOG GUI. If not enter roll correction here.
// Roll correction. Negative number = left; positive number = right.
//double rollcorr = 50;

// Kalman Filtering
// e_mea: Measurement Uncertainty - How much do we expect to our measurement vary
// e_est: Estimation Uncertainty - Can be initilized with the same value as e_mea since the kalman filter will adjust its value.
// q: Process Variance - usually a small number between 0.001 and 1 - how fast your measurement moves. Recommended 0.01. Should be tunned to your needs.
bool filterRoll = false;
float rollMEA = 1;
float rollEST = 1;
float rollQ = 0.01;

bool filterHeading = false;
float headingMEA = 1;
float headingEST = 1;
float headingQ = 0.01;

// Serial Ports
#define SerialAOG Serial                //AgIO USB conection
#define SerialRTK Serial3               //RTK radio
HardwareSerial* SerialGPS = &Serial7;   //Main postion receiver (GGA)
HardwareSerial* SerialRVC = &Serial5;   //RVC port
const int32_t baudAOG = 115200;         //USB connection speed
const int32_t baudGPS = 460800;         //UM982 connection speed
//const int32_t baudGPS = 921600;         //UM982 connection speed
const int32_t baudRTK = 9600;           // most are using Xbee radios with default of 115200
const int64_t baudRVC = 115200;         //RVC speed

int rvcStartbyte = 0;
int rvcDataread = 0;

// Send data to AgIO via usb
bool sendUSB = false;

struct ConfigIP {
    uint8_t ipOne = 192;
    uint8_t ipTwo = 168;
    uint8_t ipThree = 5;
}; 
/************************* End User Settings *********************/

SimpleKalmanFilter rollFilter(rollMEA, rollEST, rollQ);
SimpleKalmanFilter headingFilter(headingMEA, headingEST, headingQ);

Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();
BNO08x_RVC_Data rvcData;
elapsedMillis bnoTimer;
bool greenLight = true;
float reqData;
float recData;

//char latitude[15];
//char longitude[15];

bool gotCR = false;
bool gotLF = false;
bool gotDollar = false;
char msgBuf[254];
int msgBufLen = 0;

#define ImuWire Wire        //SCL=19:A5 SDA=18:A4
#define RAD_TO_DEG_X_10 572.95779513082320876798154814105

//Status LED's
#define GGAReceivedLED 13         //Teensy onboard LED
#define Power_on_LED 5            //Red
#define Ethernet_Active_LED 6     //Green
#define GPSRED_LED 9              //Red (Flashing = NO IMU or Dual, ON = GPS fix with IMU)
#define GPSGREEN_LED 10           //Green (Flashing = Dual bad, ON = Dual good)
#define AUTOSTEER_STANDBY_LED 11  //Red
#define AUTOSTEER_ACTIVE_LED 12   //Green
uint32_t gpsReadyTime = 0;        //Used for GGA timeout

//Event output to UM982
#define EventOUT 23 //Trigger UM982 event

void errorHandler();
void GGA_Handler();
void VTG_Handler();
void HPR_Handler();
void SXT_Handler();
void EVT_Handler();
void imuHandler();
void autosteerSetup();
void EthernetStart();
void udpNtrip();
void BuildNmea();
void autosteerLoop();
void ReceiveUdp();

ConfigIP networkAddress;   //3 bytes

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

byte CK_A = 0;
byte CK_B = 0;
int relposnedByteCount = 0;

//Speed pulse output
elapsedMillis speedPulseUpdateTimer = 0;
byte velocityPWM_Pin = 36;      // Velocity (MPH speed) PWM pin

//Used to set CPU speed
extern "C" uint32_t set_arm_clock(uint32_t frequency); // required prototype

bool dualReadyGGA = false;
bool dualReadyRelPos = false;

// booleans to see if we are using BNO08x
bool useBNO08x = false;

double baseline = 0;
double rollDual = 0;
double pitchDual = 0;
double relPosD = 0;
double heading = 0;

byte ackPacket[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

constexpr int serial_buffer_size = 512;
uint8_t GPSrxbuffer[serial_buffer_size];    //Extra serial rx buffer
uint8_t GPStxbuffer[serial_buffer_size];    //Extra serial tx buffer
uint8_t GPS2rxbuffer[serial_buffer_size];   //Extra serial rx buffer
uint8_t GPS2txbuffer[serial_buffer_size];   //Extra serial tx buffer
uint8_t RTKrxbuffer[serial_buffer_size];    //Extra serial rx buffer
uint8_t RVCrxbuffer[serial_buffer_size];    //Extra serial rx buffer

/* A parser is declared with 3 handlers at most */
NMEAParser<5> parser;
UM982Parser<3> UMparser;

bool isTriggered = false;
bool blink = false;
bool blinkRVC = false;

bool Autosteer_running = true; //Auto set off in autosteer setup
bool Ethernet_running = false; //Auto set on in ethernet setup
bool GGA_Available = false;    //Do we have GGA on correct port?
uint32_t PortSwapTime = 0;

double roll = 0;
double pitch = 0;
double yaw = 0;

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

// Setup procedure ---------------------------------------------------------------------------------------------------------------
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
  pinMode(EventOUT, OUTPUT);

  digitalWrite(EventOUT, LOW);

  // the dash means wildcard
 
  parser.setErrorHandler(errorHandler);
  parser.addHandler("G-GGA", GGA_Handler);
  parser.addHandler("G-VTG", VTG_Handler);
  parser.addHandler("G-HPR", HPR_Handler);
  parser.addHandler("KSXT", SXT_Handler);

  UMparser.setErrorHandler(errorHandler);
  UMparser.addHandler("EVENTSLNA", EVT_Handler);

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
  SerialRVC->begin(baudRVC);
  SerialRVC->addMemoryForRead(RVCrxbuffer, serial_buffer_size);

  Serial.println("SerialAOG, SerialRTK, SerialGPS, SerialRVC initialized");

  Serial.println("\r\nStarting AutoSteer...");
  autosteerSetup();
  
  Serial.println("\r\nStarting Ethernet...");
  EthernetStart();

  Serial.println("\r\nStarting BNO08x...");

  if (!rvc.begin(SerialRVC))
  { // connect to the sensor over hardware serial
    Serial.println("Could not find RVC BNO08x!");
    while (1)
      delay(10);
  }
  else
  {
    Serial.println("RVC BNO08x found!");
    useBNO08x = true;
  }

  Serial.println("\r\nEnd setup, waiting for GPS...\r\n");
}

void loop()
{
  if ( rvc.read(&rvcData) && (bnoTimer >= report_interval) )
  {
    Serial.println( millis() );
    bnoTimer = bnoTimer - report_interval;
    digitalWrite(EventOUT, HIGH); // Send begining of pulse to UM982. Triggers on rising edge.
    digitalWrite(EventOUT, LOW); //End of UM982 event pulse.
    roll = rvcData.roll;
    pitch = rvcData.pitch;
    yaw = rvcData.yaw;
    if (blink)
    {
        digitalWrite(GGAReceivedLED, HIGH);
    }
    else
    {
        digitalWrite(GGAReceivedLED, LOW);
    }
    blink = !blink;
  }

  while (SerialGPS->available())
  {
    char incoming = SerialGPS->read();
    UMparser << incoming;
    //Serial.println(incoming);
    
    digitalWrite(GPSGREEN_LED, HIGH);   //Turn green GPS LED ON

    switch (incoming) 
    {
        case '#':
        msgBuf[msgBufLen] = incoming;
        msgBufLen ++;
        gotDollar = true;
        break;
        case '\r':
        msgBuf[msgBufLen] = incoming;
        msgBufLen ++;
        gotCR = true;
        gotDollar = false;
        break;
        case '\n':
        msgBuf[msgBufLen] = incoming;
        msgBufLen ++;
        gotLF = true;
        gotDollar = false;
        break;
        default:
        if (gotDollar)
            {
            msgBuf[msgBufLen] = incoming;
            msgBufLen ++;
            }
        break;
    }

    if (gotCR && gotLF)
    {
      //Serial.print(msgBuf);
      gotCR = false;
      gotLF = false;
      gotDollar = false;
      memset( msgBuf, 0, 254 );
      msgBufLen = 0;
    }
  }

    udpNtrip();

    // Check for RTK Radio
    if (SerialRTK.available())
    {
        SerialGPS->write(SerialRTK.read());
    }

    // If both dual messages are ready, send to AgOpen
    //Serial.print(dualReadyGGA);
    //Serial.println(dualReadyRelPos);
    //delay(10);
    if (dualReadyGGA == true && dualReadyRelPos == true )
    {
        imuHandler();
        BuildNmea();
        dualReadyGGA = false;
        dualReadyRelPos = false;
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