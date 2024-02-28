#include <Arduino.h>

// Unicore parsing variables
char solStat[17];
char posType[13];
double slHeight;
double maHeight;

void umerrorHandler()
{
  //Serial.print("*** Error : ");
  //Serial.println(umparser.error());
}

void UNIHEADINGA_Handler(){
  int satTrack;
  int satSol;
  int satAbv;
  int satL2;

  useDual = true;

  // if ( umparser.getArg(18, satTrack)){ Serial.println(satTrack);}
  // if ( umparser.getArg(19, satSol)){ Serial.println(satSol);}
  // if ( umparser.getArg(20, satAbv)){ Serial.println(satAbv);}
  // if ( umparser.getArg(21, satL2)){ Serial.println(satL2);}


  if ( umparser.getArg(12, heading)){}

  if ( umparser.getArg(13, pitchDual)){}

  //Serial.println(heading);
  heading += (headingcorr * 0.1);
  if (heading >= 360) heading -= 360;
  if (heading < 0) heading += 360;
  //Serial.println(heading);

  if ( umparser.getArg(11, baseline)){}
  if (baseline == 0){baseline += 0.01;}

  double antDelta;
  antDelta = maHeight - slHeight;
  if ( antDelta == 0 ){antDelta += 0.01;}
 
  if ( umparser.getArg(9, solStat)){}
  if ( umparser.getArg(10, posType)){}
  // Serial.print(solStat);
  // Serial.println(posType);
  // Serial.println(strcmp(solStat, "SOL_COMPUTED"));
  // Serial.println(strcmp(posType, "L1_INT"));
  // Serial.println(strcmp(posType, "NARROW_INT"));
  //if ( strcmp(solStat, "SOL_COMPUTED") == 0 ){
  if ( strcmp(solStat, "SOL_COMPUTED") == 0 && (strcmp(posType, "L1_INT") == 0 || strcmp(posType, "NARROW_INT") == 0)){
    //useDual = true;
    // Serial.print("Ant Delta: ");
    // Serial.println(antDelta);
    // Serial.println(baseline);
    // double div = antDelta / baseline;
    // Serial.println(div);
    // Serial.println(asin(antDelta / baseline));
    rollDual = (asin(antDelta / baseline)) * -RAD_TO_DEG + rollcorr;
    //Serial.print("Roll Dual: ");
    //Serial.println(rollDual);
    digitalWrite(GPSGREEN_LED, HIGH); //Turn green GPS LED ON
    imuHandler(); //No IMU so use dual data direct
    dualReadyRelPos = true; //RelPos ready is true so PAOGI will send when the GGA is also ready
  }
    else{
      rollDual *= 0.9;
      digitalWrite(GPSGREEN_LED, blink);  //Flash the green GPS LED
      //useDual = false;
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