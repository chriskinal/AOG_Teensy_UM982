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

// IMU
char imuHeading[6];
char imuRoll[6];
char imuPitch[6];
char imuYawRate[6];

// If odd characters showed up.
void errorHandler()
{
  //nothing at the moment
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

void readBNO()
{
          if (bno08x.dataAvailable() == true)
        {
            float dqx, dqy, dqz, dqw, dacr;
            uint8_t dac;

            //get quaternion
            bno08x.getQuat(dqx, dqy, dqz, dqw, dacr, dac);
/*            
            while (bno08x.dataAvailable() == true)
            {
                //get quaternion
                bno08x.getQuat(dqx, dqy, dqz, dqw, dacr, dac);
                //Serial.println("Whiling");
                //Serial.print(dqx, 4);
                //Serial.print(F(","));
                //Serial.print(dqy, 4);
                //Serial.print(F(","));
                //Serial.print(dqz, 4);
                //Serial.print(F(","));
                //Serial.println(dqw, 4);
            }
            //Serial.println("End of while");
*/            
            float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
            dqw = dqw / norm;
            dqx = dqx / norm;
            dqy = dqy / norm;
            dqz = dqz / norm;

            float ysqr = dqy * dqy;

            // yaw (z-axis rotation)
            float t3 = +2.0 * (dqw * dqz + dqx * dqy);
            float t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
            yaw = atan2(t3, t4);

            // Convert yaw to degrees x10
            correctionHeading = -yaw;
            yaw = (int16_t)((yaw * -RAD_TO_DEG_X_10));
            if (yaw < 0) yaw += 3600;

            // pitch (y-axis rotation)
            float t2 = +2.0 * (dqw * dqy - dqz * dqx);
            t2 = t2 > 1.0 ? 1.0 : t2;
            t2 = t2 < -1.0 ? -1.0 : t2;
//            pitch = asin(t2) * RAD_TO_DEG_X_10;

            // roll (x-axis rotation)
            float t0 = +2.0 * (dqw * dqx + dqy * dqz);
            float t1 = +1.0 - 2.0 * (dqx * dqx + ysqr);
//            roll = atan2(t0, t1) * RAD_TO_DEG_X_10;

            if(steerConfig.IsUseY_Axis)
            {
              roll = asin(t2) * RAD_TO_DEG_X_10;
              pitch = atan2(t0, t1) * RAD_TO_DEG_X_10;
            }
            else
            {
              pitch = asin(t2) * RAD_TO_DEG_X_10;
              roll = atan2(t0, t1) * RAD_TO_DEG_X_10;
            }
            
            if(invertRoll)
            {
              roll *= -1;
            }
        }
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
