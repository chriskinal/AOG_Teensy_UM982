// Conversion to Hexidecimal
const char* asciiHex = "0123456789ABCDEF";

// the new PANDA sentence buffer
char nmea[100];

// GGA
char fixTime[12];
char latitude[15];
char latNS[3];
char longitude[15];
char lonEW[3];
char fixQuality[2];
char numSats[4];
char HDOP[5];
char altitude[12];
char ageDGPS[10];

// VTG
char vtgHeading[12] = { };
char speedKnots[10] = { };

// HPR
char umHeading[8];
char umRoll[8];
int solQuality;

//SXT
char timeFix[18];
// char fixTime[18];
// char longitude[13];
// char latitude[12];
// char height[11];
// char heading[7];
// char pitch[7];
// char track[7];
// char velocity[8];
// char roll[7];
// char posQual[2];
// char headQual[2];
// char hSats[3];
// char mSats[3];
// char east[8];
// char north[8];
// char up[8];
// char eastVel[8];
// char northVel[8];
// char upVel[8];

// IMU
char imuHeading[6];
char imuRoll[6];
char imuPitch[6];
char imuYawRate[6];

// If odd characters showed up.
void errorHandler()
{
  Serial.print("*** Error : ");
  Serial.println(UMparser.error());
}

void GGA_Handler() //Rec'd GGA
{
    // fix time
    parser.getArg(0, fixTime);

    // latitude
    parser.getArg(1, latitude);
    parser.getArg(2, latNS);

    // longitude
    parser.getArg(3, longitude);
    parser.getArg(4, lonEW);

    // fix quality
    parser.getArg(5, fixQuality);

    // satellite #
    parser.getArg(6, numSats);

    // HDOP
    parser.getArg(7, HDOP);

    // altitude
    parser.getArg(8, altitude);

    // time of last DGPS update
    parser.getArg(12, ageDGPS);

    if (blink)
    {
        digitalWrite(GGAReceivedLED, HIGH);
    }
    else
    {
        digitalWrite(GGAReceivedLED, LOW);
    }

    blink = !blink;
    GGA_Available = true;

    dualReadyGGA = true;
   
    gpsReadyTime = systick_millis_count;    //Used for GGA timeout (LED's ETC) 
}

void VTG_Handler()
{
  // vtg heading
  parser.getArg(0, vtgHeading);

  // vtg Speed knots
  parser.getArg(4, speedKnots);
}

//UM982 Support
void HPR_Handler()
{ 
  dualReadyRelPos = true;
  digitalWrite(GPSRED_LED, LOW);   //Turn red GPS LED OFF (we are now in dual mode so green LED)

  // HPR Heading
  parser.getArg(1, umHeading);
  heading = atof(umHeading);
  if ( filterHeading )
    {
      float tempHeading;
      tempHeading = headingFilter.updateEstimate(heading);
      heading = tempHeading;
    }

  // HPR Substitute pitch for roll
  if ( parser.getArg(2, umRoll) )
  {
    rollDual = atof(umRoll);
    digitalWrite(GPSGREEN_LED, HIGH);   //Turn green GPS LED ON
    if ( filterRoll )
      {
        float tempRoll;
        tempRoll = rollFilter.updateEstimate(rollDual);
        rollDual = tempRoll;
      }
  }
  else
  {
    digitalWrite(GPSGREEN_LED, blink);  //Flash the green GPS LED
  }

  // Solution quality factor
  parser.getArg(4, solQuality);

  if (solQuality >= 4)
    {
       if (useBNO08x)
       {
           if (baseLineCheck)
           {
               imuDualDelta();         //Find the error between latest IMU reading and this dual message
              dualReadyRelPos = false;  //RelPos ready is false because we just saved the error for running from the IMU
           }

       }
       else
       {
           imuHandler();             //No IMU so use dual data direct
           dualReadyRelPos = true;   //RelPos ready is true so PAOGI will send when the GGA is also ready
       }
    }
}

void SXT_Handler()
{
  // parser.getArg(0, timeFix);
  parser.getArg(1, longitude);
  parser.getArg(2, latitude);
  // parser.getArg(3, height);
  // parser.getArg(4, heading);
  // parser.getArg(5, pitch);
  // parser.getArg(6, track);
  // parser.getArg(7, velocity);
  // parser.getArg(8, roll);
  // parser.getArg(9, posQual);
  // parser.getArg(10, headQual);
  // parser.getArg(11, hSats);
  // parser.getArg(12, mSats);
  // parser.getArg(13, east);
  // parser.getArg(14, north);
  // parser.getArg(15, up);
  // parser.getArg(16, eastVel);
  // parser.getArg(17, northVel);
  // parser.getArg(18, upVel);
  // calculateIMU();
  // imuDualDelta();
  // BuildKsxt();
}

void EVT_Handler()
{
  char fakeKSXT[254];
  memset( fakeKSXT, 0, 254 );

  char tmpSeconds[11];
  char tmpLat[16];
  char tmpLon[16];
  char tmpFixq[16];
  char tmpPosq[2];
  char tmpAlt[7];
  float tmpEvel;
  float tmpNvel;
  float tmpVel;
  char tmpSpeed[10];
  char tmpYaw[7];
  char tmpRoll[7];
  char tmpPitch[7];
  String ksxtCRC = "";
  String message = "";

  UMparser.getArg(14, tmpSeconds);
  UMparser.getArg(20, tmpFixq);
  UMparser.getArg(21, tmpLat);
  UMparser.getArg(22, tmpLon);
  UMparser.getArg(23, tmpAlt);
  UMparser.getArg(30, ageDGPS);
  UMparser.getArg(33, numSats);
  UMparser.getArg(36, tmpEvel);
  UMparser.getArg(37, tmpNvel);

  tmpVel = sqrt( tmpNvel*tmpNvel + tmpEvel*tmpEvel );
  dtostrf( tmpVel, 3, 2, tmpSpeed);
  yaw += headingcorr;
  dtostrf( yaw, 3, 2, tmpYaw);
  if ( swap_roll_pitch )
  {
    dtostrf( pitch, 3, 2, tmpRoll);
  }
  else
  {
    dtostrf( roll, 3, 2, tmpRoll);
  }

  memset(tmpPosq, 0, 2);
  if ( strcmp(tmpFixq, "NONE") == 0 ) { tmpPosq[0] = '0'; }
  if ( strcmp(tmpFixq, "SINGLE") == 0 ) { tmpPosq[0] = '1'; }
  if ( strcmp(tmpFixq, "L1_INT") == 0 ) { tmpPosq[0] = '2'; }
  if ( strcmp(tmpFixq, "NARROW_FLOAT") == 0 ) { tmpPosq[0] = '2'; }
  if ( strcmp(tmpFixq, "NARROW_INT") == 0 ) { tmpPosq[0] = '3'; }

  strcat( fakeKSXT, "$KSXT,"); //0
  strcat( fakeKSXT, tmpSeconds); //1
  strcat( fakeKSXT, ",");
  strcat( fakeKSXT, tmpLon);//2
  strcat( fakeKSXT, ",");
  strcat( fakeKSXT, tmpLat);//3
  strcat( fakeKSXT, ",");
  strcat( fakeKSXT, tmpAlt);//4
  strcat( fakeKSXT, ",");
  strcat( fakeKSXT, tmpYaw);//5
  strcat( fakeKSXT, ",");
  strcat( fakeKSXT, tmpRoll);//6
  strcat( fakeKSXT, ",000.00,");//7
  strcat( fakeKSXT, tmpSpeed);//8
  strcat( fakeKSXT, ",0,");//9
  strcat( fakeKSXT, tmpPosq);//10
  strcat( fakeKSXT, ",3,0,");//11,12
  strcat( fakeKSXT, numSats);//13
  strcat( fakeKSXT, ",0,0,0,0,0,0,,*");//14,15,16,17,18,19,20,21
  //Serial.println(fakeKSXT);
  //Serial.println(withChecksum(fakeKSXT, false));
  message = withChecksum(fakeKSXT, false);
  memset(fakeKSXT, 0, 254);
  message.toCharArray(fakeKSXT, 254);
  //Serial.print(fakeKSXT);
  if (sendUSB) { SerialAOG.write(fakeKSXT); } // Send USB GPS data if enabled in user settings
  if (Ethernet_running){
      Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
      Eth_udpPAOGI.write(fakeKSXT, strlen(fakeKSXT));
      Eth_udpPAOGI.endPacket();
  }



}

String withChecksum(String sentence, bool printLogs) {
  // http://www.hhhh.org/wiml/proj/nmeaxor.html
  bool started = false;
  char checksum = 0;
  for (unsigned int index = 0; index < sentence.length(); index++) {
    if (index > 0 && sentence[index - 1] == '$') {
      if (printLogs) Serial.println("Found first checksum char:");
      if (printLogs) Serial.println(sentence[index]);
      if (printLogs) Serial.println(sentence[index], HEX);
      if (printLogs) Serial.println("Set as initial 'last step result'.");
      if (printLogs) Serial.println();
      checksum = sentence[index];
      started = true;
      continue; // Skip the rest of this loop iteration.
    }
    
    if (sentence[index] == '*') {
      if (printLogs) Serial.println("Reached the end of checksum chars.");
      if (printLogs) Serial.println("Final checksum:");
      if (printLogs) Serial.println(checksum, HEX);
      if (printLogs) Serial.println();
      break; // Exit the loop.
    }
    
    // Ignore everything preceeding '$'.
    if (!started) {
      continue; // Skip the rest of this loop iteration.
    }
    
    if (printLogs) Serial.println("Xoring last step result and current char.");
    if (printLogs) Serial.println(checksum, HEX);
    if (printLogs) Serial.println(sentence[index]);
    if (printLogs) Serial.println(sentence[index], HEX);
    
    checksum = checksum xor sentence[index];
    if (printLogs) Serial.println("Got new last step result:");
    if (printLogs) Serial.println(checksum, HEX);
    if (printLogs) Serial.println();
  }
  
  String sentenceWithChecksum = sentence + (checksum < 10 ? "0" : "") + String(checksum, HEX) + "\r\n";
  if (printLogs) Serial.println("Sentence with checksum:");
  if (printLogs) Serial.println(sentenceWithChecksum);
  return sentenceWithChecksum;
}

void imuHandler()
{
  int16_t temp = 0;

  if (useBNO08x)
  {
      //BNO is reading in its own timer    
      // Fill rest of Panda Sentence - Heading
      temp = yaw;
      itoa(temp, imuHeading, 10);

      // the pitch x10
      temp = (int16_t)pitch;
      itoa(temp, imuPitch, 10);

      // the roll x10
      temp = (int16_t)roll;
      itoa(temp, imuRoll, 10);

      // YawRate - 0 for now
      itoa(0, imuYawRate, 10);
  }

  // No else, because we want to use dual heading and IMU roll when both connected
  // We have a IMU so apply the dual/IMU roll/heading error to the IMU data.
  if ( useBNO08x && baseLineCheck)
  {
      float dualTemp;   //To convert IMU data (x10) to a float for the PAOGI so we have the decamal point
              
      // the IMU heading raw
      // dualTemp = yaw * 0.1;
      // dtostrf(dualTemp, 3, 1, imuHeading);          

      // the IMU heading fused to the dual heading
      fuseIMU();
      dtostrf(imuCorrected, 3, 1, imuHeading);
    
      // the pitch
      dualTemp = (int16_t)pitch * 0.1;
      dtostrf(dualTemp, 3, 1, imuPitch);

      // the roll
      dualTemp = (int16_t)roll * 0.1;
      //If dual heading correction is 90deg (antennas left/right) correct the IMU roll
      if(headingcorr == 900)
      {
        dualTemp += rollDeltaSmooth;
      }
      dtostrf(dualTemp, 3, 1, imuRoll);

  }
  else  //Not using IMU so put dual Heading & Roll in direct.
  {
      // the roll
      if (makeOGI)
      {
        dtostrf(rollDual, 4, 2, imuRoll);
      }
      else
      {
        itoa(rollDual * 10, imuRoll, 10);
      }

      // the Dual heading raw
      if (makeOGI)
      {
        dtostrf(heading, 4, 2, imuHeading);
      }
      else
      {
        itoa(heading * 10, imuHeading, 10);
      }

      // the pitch
      dtostrf(pitchDual, 4, 4, imuPitch);
  }
}

void imuDualDelta()
{
                                       //correctionHeading is IMU heading in radians
   gpsHeading = heading * DEG_TO_RAD;  //gpsHeading is Dual heading in radians

   //Difference between the IMU heading and the GPS heading
   gyroDelta = (correctionHeading + imuGPS_Offset) - gpsHeading;
   if (gyroDelta < 0) gyroDelta += twoPI;

   //calculate delta based on circular data problem 0 to 360 to 0, clamp to +- 2 Pi
   if (gyroDelta >= -PIBy2 && gyroDelta <= PIBy2) gyroDelta *= -1.0;
   else
   {
       if (gyroDelta > PIBy2) { gyroDelta = twoPI - gyroDelta; }
       else { gyroDelta = (twoPI + gyroDelta) * -1.0; }
   }
   if (gyroDelta > twoPI) gyroDelta -= twoPI;
   if (gyroDelta < -twoPI) gyroDelta += twoPI;

   //if the gyro and last corrected fix is < 10 degrees, super low pass for gps
   if (abs(gyroDelta) < 0.18)
   {
       //a bit of delta and add to correction to current gyro
       imuGPS_Offset += (gyroDelta * (0.1));
       if (imuGPS_Offset > twoPI) imuGPS_Offset -= twoPI;
       if (imuGPS_Offset < -twoPI) imuGPS_Offset += twoPI;
   }
   else
   {
       //a bit of delta and add to correction to current gyro
       imuGPS_Offset += (gyroDelta * (0.2));
       if (imuGPS_Offset > twoPI) imuGPS_Offset -= twoPI;
       if (imuGPS_Offset < -twoPI) imuGPS_Offset += twoPI;
   }

   //So here how we have the difference between the IMU heading and the Dual GPS heading
   //This "imuGPS_Offset" will be used in imuHandler() when the GGA arrives 

   //Calculate the diffrence between dual and imu roll
   float imuRoll;
   imuRoll = (int16_t)roll * 0.1;
   rollDelta = rollDual - imuRoll;
   rollDeltaSmooth = (rollDeltaSmooth * 0.7) + (rollDelta * 0.3);
}

void fuseIMU()
{     
   //determine the Corrected heading based on gyro and GPS
   imuCorrected = correctionHeading + imuGPS_Offset;
   if (imuCorrected > twoPI) imuCorrected -= twoPI;
   if (imuCorrected < 0) imuCorrected += twoPI;

   imuCorrected = imuCorrected * RAD_TO_DEG; 
}

void BuildNmea(void)
{
    strcpy(nmea, "");

    if (makeOGI) strcat(nmea, "$PAOGI,");
    else strcat(nmea, "$PANDA,");

    strcat(nmea, fixTime);
    strcat(nmea, ",");

    strcat(nmea, latitude);
    strcat(nmea, ",");

    strcat(nmea, latNS);
    strcat(nmea, ",");

    strcat(nmea, longitude);
    strcat(nmea, ",");

    strcat(nmea, lonEW);
    strcat(nmea, ",");

    // 6
    strcat(nmea, fixQuality);
    strcat(nmea, ",");

    strcat(nmea, numSats);
    strcat(nmea, ",");

    strcat(nmea, HDOP);
    strcat(nmea, ",");

    strcat(nmea, altitude);
    strcat(nmea, ",");

    //10
    strcat(nmea, ageDGPS);
    strcat(nmea, ",");

    //11
    strcat(nmea, speedKnots);
    strcat(nmea, ",");

    //12
    strcat(nmea, imuHeading);
    strcat(nmea, ",");

    //13
    strcat(nmea, imuRoll);
    strcat(nmea, ",");

    //14
    strcat(nmea, imuPitch);
    strcat(nmea, ",");

    //15
    strcat(nmea, imuYawRate);

    strcat(nmea, "*");

    CalculateChecksum();

    strcat(nmea, "\r\n");

    if (sendUSB) { SerialAOG.write(nmea); } // Send USB GPS data if enabled in user settings
    
    if (Ethernet_running)   //If ethernet running send the GPS there
    {
        int len = strlen(nmea);
        Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
        Eth_udpPAOGI.write(nmea, len);
        Eth_udpPAOGI.endPacket();
    }
}

void CalculateChecksum(void)
{
  int16_t sum = 0;
  int16_t inx = 0;
  char tmp;

  // The checksum calc starts after '$' and ends before '*'
  for (inx = 1; inx < 200; inx++)
  {
    tmp = nmea[inx];

    // * Indicates end of data and start of checksum
    if (tmp == '*')
    {
      break;
    }

    sum ^= tmp;    // Build checksum
  }

  byte chk = (sum >> 4);
  char hex[2] = { asciiHex[chk], 0 };
  strcat(nmea, hex);

  chk = (sum % 16);
  char hex2[2] = { asciiHex[chk], 0 };
  strcat(nmea, hex2);
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
