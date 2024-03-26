// UM982 Connection plan:
// Teensy Serial 7 RX (28) to F9P Position receiver TX1 (Position data)
// Teensy Serial 7 TX (29) to F9P Position receiver RX1 (RTCM data for RTK)
//
#include "zNMEAParser.h"
#include <Wire.h>
#include "BNO08x_AOG.h"
// Ethernet Options (Teensy 4.1 Only)
#ifdef ARDUINO_TEENSY41
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#endif // ARDUINO_TEENSY41

/************************* User Settings *************************/
bool udpPassthrough = false;  // False = GPS neeeds to send GGA, VTG & HPR messages. True = GPS needs to send KSXT messages only.
bool makeOGI = true;          // Set to true to make PAOGI messages. Else PANDA message will be made.
const bool invertRoll = true; // Used for IMU with dual antenna
bool baseLineCheck = false;   // Set to true to use IMU fusion with UM982
#define baseLineLimit 5       // Max CM differance in baseline for fusion

// Heading correction can be enetered into the UM982 config or AOG GUI so this can be 0. If not in UM982 config or AOG GUI, set here.
// Negative number = west, positive number = east.
double headingcorr = 0;
// Roll correction can be entered in the AOG GUI. If not enter roll correction here.
// Roll correction. Negative number = left; positive number = right.
// double rollcorr = 50;

// Serial Ports
#define SerialAOG Serial              // AgIO USB conection
#define SerialRTK Serial3             // RTK radio
HardwareSerial *SerialGPS = &Serial7; // Main postion receiver (GGA)
const int32_t baudAOG = 115200;       // USB connection speed
const int32_t baudGPS = 460800;       // UM982 connection speed
const int32_t baudRTK = 9600;         // most are using Xbee radios with default of 115200

// Send data to AgIO via usb
bool sendUSB = true;

struct ConfigIP
{
  uint8_t ipOne = 192;
  uint8_t ipTwo = 168;
  uint8_t ipThree = 5;
};
/************************* End User Settings *********************/

bool gotCR = false;
bool gotLF = false;
bool gotDollar = false;
char msgBuf[254];
int msgBufLen = 0;

#define ImuWire Wire // SCL=19:A5 SDA=18:A4
#define RAD_TO_DEG_X_10 572.95779513082320876798154814105

#define REPORT_INTERVAL 20  // BNO report time, we want to keep reading it quick & offen. Its not timmed to anything just give constant data.
uint32_t READ_BNO_TIME = 0; // Used stop BNO data pile up (This version is without resetting BNO everytime)

// Status LED's
#define GGAReceivedLED 13        // Teensy onboard LED
#define Power_on_LED 5           // Red
#define Ethernet_Active_LED 6    // Green
#define GPSRED_LED 9             // Red (Flashing = NO IMU or Dual, ON = GPS fix with IMU)
#define GPSGREEN_LED 10          // Green (Flashing = Dual bad, ON = Dual good)
#define AUTOSTEER_STANDBY_LED 11 // Red
#define AUTOSTEER_ACTIVE_LED 12  // Green
uint32_t gpsReadyTime = 0;       // Used for GGA timeout

void errorHandler();
void GGA_Handler();
void VTG_Handler();
void HPR_Handler();
void autosteerSetup();
void EthernetStart();
void udpNtrip();
void BuildNmea();
void readBNO();
void autosteerLoop();
void ReceiveUdp();

ConfigIP networkAddress; // 3 bytes

// IP & MAC address of this module of this module
byte Eth_myip[4] = {0, 0, 0, 0}; // This is now set via AgIO
byte mac[] = {0x00, 0x00, 0x56, 0x00, 0x00, 0x78};

unsigned int portMy = 5120;           // port of this module
unsigned int AOGNtripPort = 2233;     // port NTRIP data from AOG comes in
unsigned int AOGAutoSteerPort = 8888; // port Autosteer data from AOG comes in
unsigned int portDestination = 9999;  // Port of AOG that listens
char Eth_NTRIP_packetBuffer[512];     // buffer for receiving ntrip data

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Eth_udpPAOGI;     // Out port 5544
EthernetUDP Eth_udpNtrip;     // In port 2233
EthernetUDP Eth_udpAutoSteer; // In & Out Port 8888

IPAddress Eth_ipDestination;

byte CK_A = 0;
byte CK_B = 0;
int relposnedByteCount = 0;

// Speed pulse output
elapsedMillis speedPulseUpdateTimer = 0;
byte velocityPWM_Pin = 36; // Velocity (MPH speed) PWM pin

// Used to set CPU speed
extern "C" uint32_t set_arm_clock(uint32_t frequency); // required prototype

bool dualReadyGGA = false;
bool dualReadyRelPos = false;

// booleans to see if we are using BNO08x
bool useBNO08x = false;

// BNO08x address variables to check where it is
const uint8_t bno08xAddresses[] = {0x4A, 0x4B};
const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses) / sizeof(bno08xAddresses[0]);
uint8_t bno08xAddress;
BNO080 bno08x;

double baseline = 0;
double rollDual = 0;
double pitchDual = 0;
double heading = 0;

byte ackPacket[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

constexpr int serial_buffer_size = 2040;
uint8_t GPSrxbuffer[serial_buffer_size];  // Extra serial rx buffer
uint8_t GPStxbuffer[serial_buffer_size];  // Extra serial tx buffer
uint8_t GPS2rxbuffer[serial_buffer_size]; // Extra serial rx buffer
uint8_t GPS2txbuffer[serial_buffer_size]; // Extra serial tx buffer
uint8_t RTKrxbuffer[serial_buffer_size];  // Extra serial rx buffer
const int tmp_serial_buffer_size = 2048;
uint8_t tmpGPSrxbuffer[tmp_serial_buffer_size];    //Temp serial rx buffer for detecting / configuring the UM982
uint8_t tmpGPStxbuffer[tmp_serial_buffer_size];    //Temp serial tx buffer for detecting / configuring the UM982

// Baudrates for detecting UM982 receiver
uint32_t baudrates[]
{
  460800,
  115200
};
const uint32_t nrBaudrates = sizeof(baudrates)/sizeof(baudrates[0]);

// Bools for detecting / configuring the UM982
bool gotUM982 = false;
bool setUM982 = false;

/* A parser is declared with 4 handlers at most */
NMEAParser<4> parser;

bool isTriggered = false;
bool blink = false;

bool Autosteer_running = true; // Auto set off in autosteer setup
bool Ethernet_running = false; // Auto set on in ethernet setup
bool GGA_Available = false;    // Do we have GGA on correct port?
uint32_t PortSwapTime = 0;

float roll = 0;
float pitch = 0;
float yaw = 0;

// Fusing BNO with Dual
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
uint8_t aogSerialCmd[4] = {'!', 'A', 'O', 'G'};
uint8_t aogSerialCmdBuffer[6];
uint8_t aogSerialCmdCounter = 0;

//-=-=-=-=- UBX binary specific variables
struct ubxPacket
{
  uint8_t cls;
  uint8_t id;
  uint16_t len;          // Length of the payload. Does not include cls, id, or checksum bytes
  uint16_t counter;      // Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
  uint16_t startingSpot; // The counter value needed to go past before we begin recording into payload array
  uint8_t *payload;      // We will allocate RAM for the payload if/when needed.
  uint8_t checksumA;     // Given to us from module. Checked against the rolling calculated A/B checksums.
  uint8_t checksumB;

  ////sfe_ublox_packet_validity_e valid;			 //Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
  ////sfe_ublox_packet_validity_e classAndIDmatch; // Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID
};

// Setup procedure ---------------------------------------------------------------------------------------------------------------
void setup()
{
  delay(500); // Small delay so serial can monitor start up
  
  Serial.println("Start setup");
  Serial.println();

  //Check for UM982
  uint32_t baudrate = 0;
  for (uint32_t i = 0; i < nrBaudrates; i++)
  {
    baudrate = baudrates[i];
    Serial.print(F("Checking for UM982 at baudrate: "));
		Serial.println(baudrate);
    SerialGPS->begin(baudrate);
    // Increase the size of the serial buffer to hold longer UM982 config messages
    SerialGPS->addMemoryForRead(tmpGPSrxbuffer, tmp_serial_buffer_size);
    SerialGPS->addMemoryForWrite(tmpGPStxbuffer, tmp_serial_buffer_size);
    delay(100);
    SerialGPS->write("VERSION\r\n");
    delay(100);
    while (SerialGPS->available())
    {
      char incoming[100];
      SerialGPS->readBytesUntil('\n', incoming, 100);
      //Serial.println(incoming);
      if ( strstr(incoming, "UM982") != NULL)
      {
        if (baudrate != 460800)
          {
            Serial.println("UM982 baudrate wrong for AOG. Setting to 460800 bps for AOG");
            SerialGPS->write("CONFIG COM1 460800\r\n");
            delay(100);
            SerialGPS->begin(baudGPS);
          }
        gotUM982 = true;
        break;
      }
      if ( gotUM982 ) {break;}
    }
    if ( gotUM982 ) {break;}
  }

  // Configure UM982
  if ( gotUM982 )
  {
    SerialGPS->write("CONFIG\r\n"); // Request the UM982 Configuration
    delay(200);

    while ( SerialGPS->available() && !setUM982 )
    {
      char incoming[100];
      SerialGPS->readBytesUntil('\n', incoming, 100);
      //Serial.println(incoming);

      //Check the "UM982 configured" flag.
      if ( strstr(incoming, "CONFIG ANTENNADELTAHEN") != NULL)
      {
        Serial.println("Got the config line");
        if ( strstr(incoming, "ANTENNADELTAHEN 0.0099 0.0099 0.0099") != NULL)
        {
          Serial.println("And it is already configured");
          Serial.println();
          
          // Reset serial buffer size
          SerialGPS->addMemoryForRead(GPSrxbuffer, serial_buffer_size);
          SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);
          setUM982 = true;
        }
        else
        {
          Serial.println("And it is not already configured");

          //Clear out the serial channel
          SerialGPS->write("UNLOGALL\r\n");
          while ( ( SerialGPS->available()))
          {
            SerialGPS->read();
          }
          // Set UM982 operating mode
          Serial.println("Setting mode");
          SerialGPS->write("MODE ROVER AUTOMOTIVE\r\n");
          delay(100);

          // Set heading to tractor
          Serial.println("Setting heading to tractor");
          SerialGPS->write("CONFIG HEADING TRACTOR\r\n");
          delay(100);

          // Set heading smoothing
          Serial.println("Setting heading smoothing");
          SerialGPS->write("CONFIG SMOOTH HEADING 10\r\n");
          delay(100);

          // Set COM1 to 460800
          Serial.println("Setting COM1 to 460800 bps");
          SerialGPS->write("CONFIG COM1 460800\r\n");
          delay(100);

          // Set COM2 to 460800
          Serial.println("Setting COM2 to 460800 bps");
          SerialGPS->write("CONFIG COM2 460800\r\n");
          delay(100);

          // Set COM3 to 460800
          Serial.println("Setting COM3 to 460800 bps");
          SerialGPS->write("CONFIG COM3 460800\r\n");
          delay(100);

          // Set GGA message and rate
          Serial.println("Setting GGA");
          SerialGPS->write("GPGGA COM1 0.1\r\n");
          delay(100);

          // Set VTG message and rate
          Serial.println("Setting VTG");
          SerialGPS->write("GPVTG COM1 0.1\r\n");
          delay(100);

          // Set HPR message and rate
          Serial.println("Setting HPR");
          SerialGPS->write("GPHPR COM1 0.1\r\n");
          delay(100);

          // Setting the flag to signal UM982 is configured for AOG
          Serial.println("Setting UM982 configured flag");
          SerialGPS->write("CONFIG ANTENNADELTAHEN 0.0099 0.0099 0.0099\r\n");
          delay(100);

          // Saving the configuration in the UM982
          Serial.println("Saving the configuration");
          SerialGPS->write("SAVECONFIG\r\n");
          Serial.println();
          delay(100);

          // Reset the serial buffer size
          SerialGPS->addMemoryForRead(GPSrxbuffer, serial_buffer_size);
          SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);

        }
      }
    }

  }

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
  parser.addHandler("G-HPR", HPR_Handler);

  delay(10);
  Serial.begin(baudAOG);
  delay(10);

  SerialGPS->begin(baudGPS);
  SerialGPS->addMemoryForRead(GPSrxbuffer, serial_buffer_size);
  SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);

  delay(10);
  SerialRTK.begin(baudRTK);
  SerialRTK.addMemoryForRead(RTKrxbuffer, serial_buffer_size);

  Serial.println("SerialAOG, SerialRTK, SerialGPS initialized");

  Serial.println("\r\nStarting AutoSteer...");
  autosteerSetup();

  Serial.println("\r\nStarting Ethernet...");
  EthernetStart();

  Serial.println("\r\nStarting BNO085...");

  // Initialize BNO085 if present.
  uint8_t error;
  ImuWire.begin();

  for (int16_t i = 0; i < nrBNO08xAdresses; i++)
  {
    bno08xAddress = bno08xAddresses[i];

    // Serial.print("\r\nChecking for BNO08X on ");
    // Serial.println(bno08xAddress, HEX);
    ImuWire.beginTransmission(bno08xAddress);
    error = ImuWire.endTransmission();

    if (error == 0)
    {
      // Serial.println("Error = 0");
      Serial.print("0x");
      Serial.print(bno08xAddress, HEX);
      Serial.println(" BNO08X Ok.");

      // Initialize BNO080 lib
      if (bno08x.begin(bno08xAddress, ImuWire)) //??? Passing NULL to non pointer argument, remove maybe ???
      {
        // Increase I2C data rate to 400kHz
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
      // Serial.println("Error = 4");
      Serial.print("0x");
      Serial.print(bno08xAddress, HEX);
      Serial.println(" BNO08X not Connected or Found");
    }
    if (useBNO08x)
      break;
  }

  delay(100);
  Serial.print("useBNO08x = ");
  Serial.println(useBNO08x);

  Serial.println("\r\nEnd setup, waiting for GPS...\r\n");
}

void loop()
{
  // Read incoming nmea from GPS
  if (SerialGPS->available())
  {
    if (udpPassthrough)
    {
      // char mChar;
      char incoming = SerialGPS->read();
      // Serial.println(incoming);
      switch (incoming)
      {
      case '$':
        msgBuf[msgBufLen] = incoming;
        msgBufLen++;
        gotDollar = true;
        break;
      case '\r':
        msgBuf[msgBufLen] = incoming;
        msgBufLen++;
        gotCR = true;
        gotDollar = false;
        break;
      case '\n':
        msgBuf[msgBufLen] = incoming;
        msgBufLen++;
        gotLF = true;
        gotDollar = false;
        break;
      default:
        if (gotDollar)
        {
          msgBuf[msgBufLen] = incoming;
          msgBufLen++;
        }
        break;
      }
      if (gotCR && gotLF)
      {
        // Serial.print(msgBuf);
        // Serial.println(msgBufLen);
        if (sendUSB)
        {
          SerialAOG.write(msgBuf);
        } // Send USB GPS data if enabled in user settings
        if (Ethernet_running)
        {
          Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
          Eth_udpPAOGI.write(msgBuf, msgBufLen);
          Eth_udpPAOGI.endPacket();
        }
        gotCR = false;
        gotLF = false;
        gotDollar = false;
        memset(msgBuf, 0, 254);
        msgBufLen = 0;
        if (blink)
        {
          digitalWrite(GGAReceivedLED, HIGH);
        }
        else
        {
          digitalWrite(GGAReceivedLED, LOW);
        }

        blink = !blink;
        digitalWrite(GPSGREEN_LED, HIGH); // Turn green GPS LED ON
      }
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
  // Serial.print(dualReadyGGA);
  // Serial.println(dualReadyRelPos);
  // delay(10);
  if (dualReadyGGA == true && dualReadyRelPos == true)
  {
    imuHandler();
    BuildNmea();
    dualReadyGGA = false;
    dualReadyRelPos = false;
  }

  // Read BNO
  if ((systick_millis_count - READ_BNO_TIME) > REPORT_INTERVAL && useBNO08x)
  {
    READ_BNO_TIME = systick_millis_count;
    readBNO();
  }

  if (Autosteer_running)
    autosteerLoop();
  else
    ReceiveUdp();

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
} // End Loop
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

// Given a message, calc and store the two byte "8-Bit Fletcher" checksum over the entirety of the message
// This is called before we send a command message
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