
// KeyaCANBUS
// Trying to get Keya to steer the tractor over CANBUS

#define IsNewModel 1

#define lowByte(w) ((uint8_t)((w) & 0xFF))
#define highByte(w) ((uint8_t)((w) >> 8))

// Enable	0x23 0x0D 0x20 0x01 0x00 0x00 0x00 0x00
// Disable	0x23 0x0C 0x20 0x01 0x00 0x00 0x00 0x00
// Fast clockwise	0x23 0x00 0x20 0x01 0xFC 0x18 0xFF 0xFF (0xfc18 signed dec is - 1000
// Anti - clockwise	0x23 0x00 0x20 0x01 0x03 0xE8 0x00 0x00 (0x03e8 signed dec is 1000
// Slow clockwise	0x23 0x00 0x20 0x01 0xFE 0x0C 0xFF 0xFF (0xfe0c signed dec is - 500)
// Slow anti - clockwise	0x23 0x00 0x20 0x01 0x01 0xf4 0x00 0x00 (0x01f4 signed dec is 500)

// Note, new model swaps order of speed bytes. Above is [4][5], below is [6][7]

// Slow clockwise 0x23 0x00 0x20 0x01 0xFF 0xFF 0xF8 0xFF  (08G-v1 model)
// Fast clockwise 0x23 0x00 0x20 0x01 0xFF 0xFF 0xD8 0xF0  (08G-v1 model)

uint8_t KeyaSteerPGN[] = {0x23, 0x00, 0x20, 0x01, 0, 0, 0, 0}; // last 4 bytes change ofc
uint8_t KeyaHeartbeat[] = {
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
};

uint8_t keyaDisableCommand[] = {0x23, 0x0C, 0x20, 0x01};
uint8_t keyaDisableResponse[] = {0x60, 0x0C, 0x20, 0x00};

uint8_t keyaEnableCommand[] = {0x23, 0x0D, 0x20, 0x01};
uint8_t keyaEnableResponse[] = {0x60, 0x0D, 0x20, 0x00};

uint8_t keyaSpeedCommand[] = {0x23, 0x00, 0x20, 0x01};
uint8_t keyaSpeedResponse[] = {0x60, 0x00, 0x20, 0x00};

uint8_t keyaCurrentQuery[] = {0x40, 0x00, 0x21, 0x01};
uint8_t keyaCurrentResponse[] = {0x60, 0x00, 0x21, 0x01};
// templates for matching responses of interest
// uint8_t keyaCurrentResponse[] = { 0x60, 0x12, 0x21, 0x01 };

uint8_t keyaFaultQuery[] = {0x40, 0x12, 0x21, 0x01};
uint8_t keyaFaultResponse[] = {0x60, 0x12, 0x21, 0x01};

uint8_t keyaVoltageQuery[] = {0x40, 0x0D, 0x21, 0x02};
uint8_t keyaVoltageResponse[] = {0x60, 0x0D, 0x21, 0x02};

uint8_t keyaTemperatureQuery[] = {0x40, 0x0F, 0x21, 0x01};
uint8_t keyaTemperatureResponse[] = {0x60, 0x0F, 0x21, 0x01};

uint8_t keyaVersionQuery[] = {0x40, 0x01, 0x11, 0x11};
uint8_t keyaVersionResponse[] = {0x60, 0x01, 0x11, 0x11};

uint32_t KeyaStatusUpdate = millis();

uint64_t KeyaPGN = 0x06000001;

CAN_message_t KeyaBusSendData; // this is the stub message we'll send to the Keya bus

const bool debugKeya = true;
bool keyaMotorStatus = false;

bool isPatternMatch(const CAN_message_t &message, const uint8_t *pattern, size_t patternSize)
{
	return memcmp(message.buf, pattern, patternSize) == 0;
}

void CAN_Setup()
{
	Serial.println("In Keya CAN-Setup");
	Keya_Bus.begin();
	Keya_Bus.setBaudRate(250000);
	delay(1000);
	KeyaBusSendData.id = KeyaPGN;
	KeyaBusSendData.flags.extended = true;
	KeyaBusSendData.len = 8;
	if (debugKeya)
		Serial.println("Initialised CANBUS");
}

void keyaSend(uint8_t data[8])
{
	memcpy(KeyaBusSendData.buf, data, 8);
	Keya_Bus.write(KeyaBusSendData);
}

void disableKeyaSteer()
{
#ifdef IsNewModel
	uint8_t buf[] = {0x23, 0x0c, 0x20, 0x01, 0, 0, 0, 0};
#else
	uint8_t buf[] = {0x03, 0x0d, 0x20, 0x11, 0, 0, 0, 0};
#endif
	keyaSend(buf);
}

void enableKeyaSteer()
{
	uint8_t buf[] = {0x23, 0x0d, 0x20, 0x01, 0, 0, 0, 0};
	keyaSend(buf);
}

void SteerKeya(int steerSpeed, float fSteerSpeed)
{
	if (!keyaDetected)
		return;
#ifdef IsNewModel
	int actualSpeed = map(fSteerSpeed, -255, 255, -9995, 9998);
#else
	int actualSpeed = map(steerSpeed, -255, 255, -995, 998);
#endif
	if (pwmDrive == 0)
	{
		disableKeyaSteer();
		// if (debugKeya) Serial.println("pwmDrive zero - disabling");
		return; // don't need to go any further, if we're disabling, we're disabling
	}
	// if (debugKeya) Serial.println("told to steer, with " + String(steerSpeed) + " so I converted that to speed " + String(actualSpeed));

	uint8_t buf[] = {0x23, 0x00, 0x20, 0x01, 0, 0, 0, 0};
	if (steerSpeed < 0)
	{
#ifdef IsNewModel
		buf[6] = highByte(actualSpeed);
		buf[7] = lowByte(actualSpeed);
		buf[4] = 0xff;
		buf[5] = 0xff;
#else
		buf[4] = highByte(actualSpeed);
		buf[5] = lowByte(actualSpeed);
		buf[6] = 0xff;
		buf[7] = 0xff;
#endif
		// if (debugKeya) Serial.println("pwmDrive < zero - clockwise - steerSpeed " + String(steerSpeed));
	}
	else
	{
#ifdef IsNewModel
		buf[6] = highByte(actualSpeed);
		buf[7] = lowByte(actualSpeed);
		buf[4] = 0x00;
		buf[5] = 0x00;
#else
		buf[4] = highByte(actualSpeed);
		buf[5] = lowByte(actualSpeed);
		buf[6] = 0x00;
		buf[7] = 0x00;
#endif
		// if (debugKeya) Serial.println("pwmDrive > zero - anti-clockwise - steerSpeed " + String(steerSpeed));
	}
	keyaSend(buf);
	enableKeyaSteer();
}

void UpdateKeyaStatus(const char *messageString)
{
	// this will be a simple rate-limited message, we don't want everything getting printed
	// yeah, we might miss something interesting, but who cares
	if (millis() - KeyaStatusUpdate > 2000)
	{
		Serial.println(messageString);
	}
	KeyaStatusUpdate = millis();
}

void KeyaBus_Receive()
{
	CAN_message_t KeyaBusReceiveData;
	if (Keya_Bus.read(KeyaBusReceiveData))
	{
		// parse the different message types
		// heartbeat 0x07000001
		// change heartbeat time in the software, default is 20ms
		if (KeyaBusReceiveData.id == 0x07000001)
		{
			keyaMotorStatus = !bitRead(KeyaBusReceiveData.buf[7], 0);
			if (!keyaDetected)
			{
				if (debugKeya)
					Serial.println("Keya heartbeat detected! Enabling Keya canbus & using reported motor current for disengage");
				// SendUdpFreeForm("Keya motor signature detected - I'll steer that way!", Eth_ipDestination, portDestination);
				keyaDetected = true;
			}
			// 0-1 - Cumulative value of angle (360 def / circle)
			// 2-3 - Motor speed, signed int eg -500 or 500
			// 4-5 - Motor current, with "symbol" ? Signed I think that means, but it does appear to be a crap int. 1, 2 for 1, 2 amps etc
			//		is that accurate enough for us?
			// 6-7 - Control_Close (error code)
			// TODO Yeah, if we ever see something here, fire off a disable, refuse to engage autosteer or..?
			// KeyaCurrentSensorReading = abs((int16_t)((KeyaBusReceiveData.buf[5] << 8) | KeyaBusReceiveData.buf[4]));
			// if (KeyaCurrentSensorReading > 255) KeyaCurrentSensorReading -= 255;
			// KeyaCurrentSensorReading = abs(KeyaBusReceiveData.buf[4]) * 20;

			if (KeyaBusReceiveData.buf[4] == 0xFF)
			{
				KeyaCurrentSensorReading = (256 - KeyaBusReceiveData.buf[5]) * 40;
			}
			else
			{
				KeyaCurrentSensorReading = KeyaBusReceiveData.buf[5] * 40;
			}
			// if (debugKeya) Serial.println("Heartbeat current is " + String(KeyaCurrentSensorReading));

			if (KeyaBusReceiveData.buf[7] != 0)
			{

				// motor disabled bit
				if (bitRead(KeyaBusReceiveData.buf[7], 0))
				{
					if (steerSwitch == 0 && keyaMotorStatus == 1)
					{
						Serial.print("\r\nMotor disabled");
						Serial.print(" - set AS off");
						steerSwitch = 1; // turn off AS if motor's internal shutdown triggers
						currentState = 1;
						previous = 0;
					}
				}
			}
		}

		// response from most commands 0x05800001
		// could have been separate codes, but oh no...

		if (KeyaBusReceiveData.id == 0x05800001)
		{
			// response to current request (this is also in heartbeat)
			// if (isPatternMatch(KeyaBusReceiveData, keyaCurrentResponse, sizeof(keyaCurrentResponse))) {
			//	// Current is unsigned float in [4]
			//	// set the motor current variable, when you find out what that is

			//	if (debugKeya) {
			//		Serial.println("Returned current is " + KeyaCurrentSensorReading);
			//	}
			//}
		}
	}
}