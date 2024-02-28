#include <Arduino.h>

// Conversion to Hexidecimal
const char* umasciiHex = "0123456789ABCDEF";

// the new PANDA sentence buffer
char umnmea[100];

// GGA
char umfixTime[12];
char umlatitude[16];
char umlatNS[3];
char umlongitude[16];
char umlonEW[3];
char umfixQuality[2];
char umnumSats[4];
char umHDOP[5];
char umaltitude[12];
char umageDGPS[12];
double testAge;

// VTG
char umvtgHeading[12] = { };
char umspeedKnots[10] = { };
double speedKnotsTmp;

// IMU
char umimuHeading[12];
char umimuRoll[10];
char umimuPitch[10];
char umimuYawRate[10] = "0";

//bool useDual = true;

//UM982Parser<5> umparser;

// Unicore parsing variables
char hr[3];
char mins[3];
char sec[3];
char solStat[17];
char posType[13];
double maAnt[3];
double maAntLLA[3];
double slAnt[3];
double slAntLLA[3];
double latTmp;
double lonTmp;
double baseLine;
double slHeight;
double maHeight;
int tenths = 0;

void shiftChar(char *arr, int size, int dataLen)
{
    for (int i = (dataLen - 1); i >= 0; i--)
    {
        memmove(arr + i + size, arr + i, 1);
    }
    for (int i = 0; i < size; i++)
    {
        memcpy(arr + i, "0", 1);
    }
}

char* replace_char(char* str, char find, char replace){
    char *current_pos = strchr(str,find);
    while (current_pos) {
        *current_pos = replace;
        current_pos = strchr(current_pos,find);
    }
    return str;
}

void umerrorHandler()
{
  //Serial.print("*** Error : ");
  //Serial.println(umparser.error());
}

void umunknownCommand()
{
  Serial.print("*** Unkown command : ");
  char buf[13];
  umparser.getType(buf);
  Serial.println(buf);
}

void UNIHEADINGA_Handler(){
  int satTrack;
  int satSol;
  int satAbv;
  int satL2;

  useDual = true;

  if ( umparser.getArg(18, satTrack)){ Serial.println(satTrack);}
  if ( umparser.getArg(19, satSol)){ Serial.println(satSol);}
  if ( umparser.getArg(20, satAbv)){ Serial.println(satAbv);}
  if ( umparser.getArg(21, satL2)){ Serial.println(satL2);}


  if ( umparser.getArg(12, heading)){}
  //Serial.println(heading);
  heading += (headingcorr * 0.1);
  if (heading >= 360) heading -= 360;
  if (heading < 0) heading += 360;
  //Serial.println(heading);

  if ( umparser.getArg(11, baseline)){}
  if (baseline == 0){baseline += 0.01;}

  double antDelta;
  antDelta = maHeight - slHeight;
 
  if ( umparser.getArg(9, solStat)){}
  if ( umparser.getArg(10, posType)){}
  Serial.print(solStat);
  Serial.println(posType);
  Serial.println(strcmp(solStat, "SOL_COMPUTED"));
  Serial.println(strcmp(posType, "L1_INT"));
  Serial.println(strcmp(posType, "NARROW_INT"));
  if ( strcmp(solStat, "SOL_COMPUTED") == 0 ){
  //if ( strcmp(solStat, "SOL_COMPUTED") == 0 && (strcmp(posType, "L1_INT") == 0 || strcmp(posType, "NARROW_INT") == 0)){
    Serial.print("Ant Delta: ");
    Serial.println(antDelta);
    // Serial.println(baseline);
    double div = antDelta / baseline;
    // Serial.println(div);
    // Serial.println(asin(antDelta / baseline));
    rollDual = (asin(div)) * -RAD_TO_DEG;
    //Serial.print("Roll Dual: ");
    //Serial.println(rollDual);
    digitalWrite(GPSGREEN_LED, HIGH); //Turn green GPS LED ON
    imuHandler(); //No IMU so use dual data direct
    dualReadyRelPos = true; //RelPos ready is true so PAOGI will send when the GGA is also ready
  }
    else{
      rollDual *= 0.9;
      digitalWrite(GPSGREEN_LED, blink);  //Flash the green GPS LED
      dualReadyRelPos = false;
    }
}

void BESTNAVXYZHA_Handler(){
  //Serial.println("Best");
  if ( umparser.getArg(13, slHeight)){}
  //Serial.print("Slave Hgt: ");
  //Serial.println(slHeight);
}

void BESTNAVXYZA_Handler(){
  //Serial.println("Best");
  if ( umparser.getArg(13, maHeight)){}
  //Serial.print("Master Hgt: ");
  //Serial.println(maHeight);
}

void AGRICA_Handler()
{
  //Serial.println("Agric");
  // Reset ready flags
  agricReady = false;
  rollReady = false;
  headingReady = false;

  //Fixtime
  memset(umfixTime, 0, sizeof(umfixTime));
  if (umparser.getArg(14, hr)){}
  if (umparser.getArg(15, mins)){}
  if (umparser.getArg(16, sec)){}
  if ( strlen(hr) == 1 ){
    shiftChar(hr, 1, 3);
  }
  if ( strlen(mins) == 1 ){
    shiftChar(mins, 1, 3);
  }
  if ( strlen(sec) == 1 ){
    shiftChar(sec, 1, 3);
  }
  char ts[2];
  tenths = tenths + 1;
  if (tenths > 9){tenths = 0;}
  sprintf(ts, "%d", tenths );
  strncat(umfixTime, hr, 2);
  strncat(umfixTime, mins, 2);
  strncat(umfixTime, sec, 2);
  strncat(umfixTime, ".", 1);
  strncat(umfixTime, ts, 1);
  strncat(umfixTime, "0", 1);

  //Serial.print(umfixTime);
  //Serial.print(",");

  // Latitude and direction
  memset(umlatitude, 0, sizeof(umlatitude));
  if ( umparser.getArg(38, latTmp)){
    latAvg += latTmp;
    }
  if ( latTmp < 0 ) {
    sprintf(umlatitude, "%.7f", latAvg() * -1 * 100 );
    umlatNS[0] = 'S';
    }
    else {
      sprintf(umlatitude, "%.7f", latAvg() * 100 );
      umlatNS[0] = 'N';
      }
    //Serial.print(umlatitude);
    //Serial.print(",");

  // Longitude and direction
  if ( umparser.getArg(39, lonTmp)){
    lonAvg += lonTmp;
  }
  if ( lonTmp < 0 ) {
    sprintf(umlongitude, "%.7f", lonAvg() * -1 * 100 );
    umlonEW[0] = 'W';
    }
    else {
      sprintf(umlongitude, "%.7f", lonAvg() *100 );
      umlonEW[0] = 'E';
      }
    //Serial.print(umlongitude);
    //Serial.print(",");

  //Fix Quality
  memset(umfixQuality, 0, sizeof(umfixQuality));  
  if (umparser.getArg(17, umfixQuality)){}

  //Number of satellites
  memset(umnumSats, 0, sizeof(umnumSats));
  int tmpNumsats1, tmpNumsats2, tmpNumsats3, tmpNumsats4;
  if (umparser.getArg(19, tmpNumsats1)){}
  if (umparser.getArg(20, tmpNumsats2)){}
  if (umparser.getArg(21, tmpNumsats3)){}
  if (umparser.getArg(62, tmpNumsats4)){}
  String tmpNumsats = tmpNumsats1 + tmpNumsats2 + tmpNumsats3 + tmpNumsats4;
  strcpy(umnumSats, tmpNumsats.c_str());

  //Calculate HDOP
  memset(umHDOP, 0, sizeof(umHDOP));
  double tmpHDOP1, tmpHDOP2;
  if (umparser.getArg(44, tmpHDOP1)){}
  if (umparser.getArg(45, tmpHDOP2)){}
  String tmpHDOP = sqrt(tmpHDOP1 * tmpHDOP1 + tmpHDOP2 * tmpHDOP2);
  strcpy(umHDOP, tmpHDOP.c_str());

  // Altitude
  memset(umaltitude, 0, sizeof(umaltitude));
  if (umparser.getArg(40, umaltitude)){}
  //Serial.println(umaltitude);

  // Age
  memset(umageDGPS, 0, sizeof(umageDGPS));
  if (umparser.getArg(57, umageDGPS)){}

  // Speed
  if (umparser.getArg(31, speedKnotsTmp)){
    speedKnotsTmp = speedKnotsTmp * 1.944;
    dtostrf(speedKnotsTmp, -4, 1, umspeedKnots);
    replace_char(umspeedKnots, ' ', NULL);
  }

  // Heading
  //char headingStat[2];
  double heading;
  //if (umparser.getArg(18, headingStat));
  //Serial.println(headingStat);
  if ( umfixQuality == "4" || umfixQuality == "5"){
    if (umparser.getArg(28, heading)){}
    headingReady = true;
  }
  else{
    heading = 0.0;
  }
  dtostrf(heading, 4, 2, umimuHeading);
  
  //  Roll
  double roll;
  if (umparser.getArg(43, maHeight)){}
  roll = (maHeight - slHeight)/baseLine;
  dtostrf(roll, 4, 2, umimuRoll);
  //Serial.println(roll);

  // Pitch
  if (umparser.getArg(29, umimuPitch)){}

  agricReady = true;
  umBuildNmea();

}

void umCalculateChecksum()
{
  int16_t sum = 0;
  int16_t inx = 0;
  char tmp;

  // The checksum calc starts after '$' and ends before '*'
  for (inx = 1; inx < 200; inx++)
  {
    tmp = umnmea[inx];

    // * Indicates end of data and start of checksum
    if (tmp == '*')
    {
      break;
    }

    sum ^= tmp;    // Build checksum
  }

  byte chk = (sum >> 4);
  char hex[2] = { umasciiHex[chk], 0 };
  strcat(umnmea, hex);

  chk = (sum % 16);
  char hex2[2] = { umasciiHex[chk], 0 };
  strcat(umnmea, hex2);
}

void umBuildNmea()
{
    strcpy(umnmea, "");

    //strcat(umnmea, "$PAOGI,");
    strcat(umnmea, "$PANDA,");
    
    strcat(umnmea, umfixTime);
    strcat(umnmea, ",");

    strcat(umnmea, umlatitude);
    strcat(umnmea, ",");

    strcat(umnmea, umlatNS);
    strcat(umnmea, ",");

    strcat(umnmea, umlongitude);
    strcat(umnmea, ",");

    strcat(umnmea, umlonEW);
    strcat(umnmea, ",");

    // 6
    strcat(umnmea, umfixQuality);
    strcat(umnmea, ",");

    strcat(umnmea, umnumSats);
    strcat(umnmea, ",");

    strcat(umnmea, umHDOP);
    strcat(umnmea, ",");

    strcat(umnmea, umaltitude);
    strcat(umnmea, ",");

    //10
    strcat(umnmea, umageDGPS);
    strcat(umnmea, ",");

    //11
    strcat(umnmea, umspeedKnots);
    strcat(umnmea, ",");

    //12
    strcat(umnmea, umimuHeading);
    strcat(umnmea, ",");

    //13
    strcat(umnmea, umimuRoll);
    strcat(umnmea, ",");

    //14
    strcat(umnmea, umimuPitch);
    strcat(umnmea, ",");

    //15
    strcat(umnmea, umimuYawRate);

    strcat(umnmea, "*");

    umCalculateChecksum();

    strcat(umnmea, "\r\n");

    Serial.print(umnmea);

    if (!passThroughGPS && !passThroughGPS2)
    {
        if (sendUSB) { SerialAOG.write(umnmea); } // Send USB GPS data if enabled in user settings
    }

    if (Ethernet_running)   //If ethernet running send the GPS there
    {
        int len = strlen(umnmea);
        Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
        Eth_udpPAOGI.write(umnmea, len);
        Eth_udpPAOGI.endPacket();
    }
}



/*
  $PANDA
  (1) Time of fix

  position
  (2,3) 4807.038,N Latitude 48 deg 07.038' N
  (4,5) 01131.000,E Longitude 11 deg 31.000' E

  (6) 1 Fix quality:
    0 = invalid
    1 = GPS fix(SPS)
    2 = DGPS fix
    3 = PPS fix
    4 = Real Time Kinematic
    5 = Float RTK
    6 = estimated(dead reckoning)(2.3 feature)
    7 = Manual input mode
    8 = Simulation mode
  (7) Number of satellites being tracked
  (8) 0.9 Horizontal dilution of position
  (9) 545.4 Altitude (ALWAYS in Meters, above mean sea level)
  (10) 1.2 time in seconds since last DGPS update
  (11) Speed in knots

  FROM IMU:
  (12) Heading in degrees
  (13) Roll angle in degrees(positive roll = right leaning - right down, left up)

  (14) Pitch angle in degrees(Positive pitch = nose up)
  (15) Yaw Rate in Degrees / second

  CHKSUM
*/

/*
  $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M ,  ,*47
   0     1      2      3    4      5 6  7  8   9    10 11  12 13  14
        Time      Lat       Lon     FixSatsOP Alt
  Where:
     GGA          Global Positioning System Fix Data
     123519       Fix taken at 12:35:19 UTC
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     1            Fix quality: 0 = invalid
                               1 = GPS fix (SPS)
                               2 = DGPS fix
                               3 = PPS fix
                               4 = Real Time Kinematic
                               5 = Float RTK
                               6 = estimated (dead reckoning) (2.3 feature)
                               7 = Manual input mode
                               8 = Simulation mode
     08           Number of satellites being tracked
     0.9          Horizontal dilution of position
     545.4,M      Altitude, Meters, above mean sea level
     46.9,M       Height of geoid (mean sea level) above WGS84
                      ellipsoid
     (empty field) time in seconds since last DGPS update
     (empty field) DGPS station ID number
      47          the checksum data, always begins with


  $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
  0      1    2   3      4    5      6   7     8     9     10   11
        Time      Lat        Lon       knots  Ang   Date  MagV

  Where:
     RMC          Recommended Minimum sentence C
     123519       Fix taken at 12:35:19 UTC
     A            Status A=active or V=Void.
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     022.4        Speed over the ground in knots
     084.4        Track angle in degrees True
     230394       Date - 23rd of March 1994
     003.1,W      Magnetic Variation
      6A          The checksum data, always begins with

  $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48

    VTG          Track made good and ground speed
    054.7,T      True track made good (degrees)
    034.4,M      Magnetic track made good
    005.5,N      Ground speed, knots
    010.2,K      Ground speed, Kilometers per hour
     48          Checksum
*/

// void VTG_Handler()
// {
//   // vtg heading
//   parser.getArg(0, vtgHeading);

//   // vtg Speed knots
//   parser.getArg(4, speedKnots);


// }
