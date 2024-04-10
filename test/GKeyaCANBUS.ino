
// KeyaCANBUS
// Trying to get Keya to steer the tractor over CANBUS

#define lowByte(w) ((uint8_t)((w) & 0xFF))
#define highByte(w) ((uint8_t)((w) >> 8))

//Enable	0x23 0x0D 0x20 0x01 0x00 0x00 0x00 0x00
//Disable	0x23 0x0C 0x20 0x01 0x00 0x00 0x00 0x00
//Fast clockwise	0x23 0x00 0x20 0x01 0xFC 0x18 0xFF 0xFF (0xfc18 signed dec is - 1000
//Anti - clockwise	0x23 0x00 0x20 0x01 0x03 0xE8 0x00 0x00 (0x03e8 signed dec is 1000
//Slow clockwise	0x23 0x00 0x20 0x01 0xFE 0x0C 0xFF 0xFF (0xfe0c signed dec is - 500)
//Slow anti - clockwise	0x23 0x00 0x20 0x01 0x01 0xf4 0x00 0x00 (0x01f4 signed dec is 500)

uint8_t KeyaSteerPGN[] = { 0x23, 0x00, 0x20, 0x01, 0,0,0,0 }; // last 4 bytes change ofc
uint8_t KeyaHeartbeat[] = { 0, 0, 0, 0, 0, 0, 0, 0, };

// templates for matching responses of interest
uint8_t keyaCurrentResponse[] = { 0x60, 0x12, 0x21, 0x01 };

uint64_t KeyaPGN = 0x06000001;

const bool debugKeya = true;

void keyaSend(uint8_t data[]) {
	//TODO Use this optimisation function once we're happy things are moving the right way
	CAN_message_t KeyaBusSendData;
	KeyaBusSendData.id = KeyaPGN;
	KeyaBusSendData.flags.extended = true;
	KeyaBusSendData.len = 8;
	memcpy(KeyaBusSendData.buf, data, sizeof(data));
	Keya_Bus.write(KeyaBusSendData);
}

void CAN_Setup() {
	Keya_Bus.begin();
	Keya_Bus.setBaudRate(250000);
	// Dedicated bus, zero chat from others. No need for filters
//	CAN_message_t msgV;
//	msgV.id = KeyaPGN;
//	msgV.flags.extended = true;
//	msgV.len = 8;
//	// claim an address. Don't think I need to do this tho
//	// anyway, just pinched this from Claas address. TODO, looks like we can do without, ditch this
//	msgV.buf[0] = 0x00;
//	msgV.buf[1] = 0x00;
//	msgV.buf[2] = 0xC0;
//	msgV.buf[3] = 0x0C;
//	msgV.buf[4] = 0x00;
//	msgV.buf[5] = 0x17;
//	msgV.buf[6] = 0x02;
//	msgV.buf[7] = 0x20;
//	Keya_Bus.write(msgV);
	delay(1000);
	if (debugKeya) Serial.println("Initialised Keya CANBUS");
}

bool isPatternMatch(const CAN_message_t& message, const uint8_t* pattern, size_t patternSize) {
	return memcmp(message.buf, pattern, patternSize) == 0;
}

void disableKeyaSteer() {
	CAN_message_t KeyaBusSendData;
	KeyaBusSendData.id = KeyaPGN;
	KeyaBusSendData.flags.extended = true;
	KeyaBusSendData.len = 8;
	KeyaBusSendData.buf[0] = 0x23;
	KeyaBusSendData.buf[1] = 0x0c;
	KeyaBusSendData.buf[2] = 0x20;
	KeyaBusSendData.buf[3] = 0x01;
	KeyaBusSendData.buf[4] = 0;
	KeyaBusSendData.buf[5] = 0;
	KeyaBusSendData.buf[6] = 0;
	KeyaBusSendData.buf[7] = 0;
	Keya_Bus.write(KeyaBusSendData);
	//if (debugKeya) Serial.println("Disabled Keya motor");
}

void disableKeyaSteerTEST() {
  CAN_message_t KeyaBusSendData;
  KeyaBusSendData.id = KeyaPGN;
  KeyaBusSendData.flags.extended = true;
  KeyaBusSendData.len = 8;
  KeyaBusSendData.buf[0] = 0x03;
  KeyaBusSendData.buf[1] = 0x0d;
  KeyaBusSendData.buf[2] = 0x20;
  KeyaBusSendData.buf[3] = 0x11;
  KeyaBusSendData.buf[4] = 0;
  KeyaBusSendData.buf[5] = 0;
  KeyaBusSendData.buf[6] = 0;
  KeyaBusSendData.buf[7] = 0;
  Keya_Bus.write(KeyaBusSendData);
  //if (debugKeya) Serial.println("Disabled Keya motor");
}

void enableKeyaSteer() {
	CAN_message_t KeyaBusSendData;
	KeyaBusSendData.id = KeyaPGN;
	KeyaBusSendData.flags.extended = true;
	KeyaBusSendData.len = 8;
	KeyaBusSendData.buf[0] = 0x23;
	KeyaBusSendData.buf[1] = 0x0d;
	KeyaBusSendData.buf[2] = 0x20;
	KeyaBusSendData.buf[3] = 0x01;
	KeyaBusSendData.buf[4] = 0;
	KeyaBusSendData.buf[5] = 0;
	KeyaBusSendData.buf[6] = 0;
	KeyaBusSendData.buf[7] = 0;
	Keya_Bus.write(KeyaBusSendData);
	if (debugKeya) Serial.println("Enabled Keya motor");
}

void SteerKeya(int steerSpeed) {
	int actualSpeed = map(steerSpeed, -255, 255, -995, 998);
	if (pwmDrive == 0) {
		disableKeyaSteer();
		//if (debugKeya) Serial.println("pwmDrive zero - disabling");
		return; // don't need to go any further, if we're disabling, we're disabling
	}
	if (debugKeya) Serial.println("told to steer, with " + String(steerSpeed) + " so....");
	if (debugKeya) Serial.println("I converted that to speed " + String(actualSpeed));

	CAN_message_t KeyaBusSendData;
	KeyaBusSendData.id = KeyaPGN;
	KeyaBusSendData.flags.extended = true;
	KeyaBusSendData.len = 8;
	KeyaBusSendData.buf[0] = 0x23;
	KeyaBusSendData.buf[1] = 0x00;
	KeyaBusSendData.buf[2] = 0x20;
	KeyaBusSendData.buf[3] = 0x01;
	if (steerSpeed < 0) {
		KeyaBusSendData.buf[4] = highByte(actualSpeed); // TODO take PWM in instead for speed (this is -1000)
		KeyaBusSendData.buf[5] = lowByte(actualSpeed);
		KeyaBusSendData.buf[6] = 0xff;
		KeyaBusSendData.buf[7] = 0xff;
		if (debugKeya) Serial.println("pwmDrive < zero - clockwise - steerSpeed " + String(steerSpeed));
	}
	else {
		KeyaBusSendData.buf[4] = highByte(actualSpeed);
		KeyaBusSendData.buf[5] = lowByte(actualSpeed);
		KeyaBusSendData.buf[6] = 0x00;
		KeyaBusSendData.buf[7] = 0x00;
		if (debugKeya) Serial.println("pwmDrive > zero - anticlock-clockwise - steerSpeed " + String(steerSpeed));
	}
	Keya_Bus.write(KeyaBusSendData);
	enableKeyaSteer();
}


void KeyaBus_Receive() {
	CAN_message_t KeyaBusReceiveData;
	if (Keya_Bus.read(KeyaBusReceiveData)) {
		// parse the different message types
		// heartbeat 0x07000001
   // change heartbeat time in the software, default is 20ms
		if (KeyaBusReceiveData.id == 0x07000001) {
			// 0-1 - Cumulative value of angle (360 def / circle)
			// 2-3 - Motor speed, signed int eg -500 or 500
			// 4-5 - Motor current, with "symbol" ? Signed I think that means, but it does appear to be a crap int. 1, 2 for 1, 2 amps etc
			//		is that accurate enough for us?
			// 6-7 - Control_Close (error code)
			// TODO Yeah, if we ever see something here, fire off a disable, refuse to engage autosteer or..?
			//KeyaCurrentSensorReading = abs((int16_t)((KeyaBusReceiveData.buf[5] << 8) | KeyaBusReceiveData.buf[4]));
			//if (KeyaCurrentSensorReading > 255) KeyaCurrentSensorReading -= 255;
			if (KeyaBusReceiveData.buf[4] == 0xFF) {
				KeyaCurrentSensorReading = (0.8 * KeyaCurrentSensorReading  ) + ( 0.2 *  (256 - KeyaBusReceiveData.buf[5]) * 20);
			}
			else {
				KeyaCurrentSensorReading = (0.8 * KeyaCurrentSensorReading  ) + ( 0.2 * KeyaBusReceiveData.buf[5] * 20);
			}
			//if (debugKeya) Serial.println("Heartbeat current is " + String(KeyaCurrentSensorReading));
		}

		// response from most commands 0x05800001
		// could have been separate PGNs, but oh no...

		//if (KeyaBusReceiveData.id == 0x05800001) {
		//	// response to current request (this is also in heartbeat)
		//	if (isPatternMatch(KeyaBusReceiveData, keyaCurrentResponse, sizeof(keyaCurrentResponse))) {
		//		// Current is unsigned float in [4]
		//		// set the motor current variable, when you find out what that is
		//		KeyaCurrentSensorReading = KeyaBusReceiveData.buf[4];
		//		if (debugKeya) Serial.println("Returned current is " + KeyaCurrentSensorReading);
		//	}
		//	else if (1 == 0) {
		//		// placeholder for more checks
		//	}
		//}
	}
}