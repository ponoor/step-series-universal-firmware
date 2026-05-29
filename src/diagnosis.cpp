// 
// 
// 

#include "diagnosis.h"
#include "utils.h"

String configVersionCompareString[5] = {
	F("CONFIG_VERSION_UNDEFINED"),
	F("CONFIG_VERSION_NOTLOADED"),
	F("CONFIG_VERSION_OLD"),
	F("CONFIG_VERSION_APPLICABLE"),
	F("CONFIG_VERSION_NEW")
};
// =============================================================================
// Diagnosis via USB serial
// =============================================================================

void diagnosisCommand(uint8_t inByte) {
	switch (inByte)
	{
	case 'm':
		printMenu();
		break;
	case 'c':
		printConfigurations();
		break;
	case 's':
		printCurrentState();
		break;
	case 't':
		testMotion();
		break;
#ifdef HAVE_BRAKE
	case 'b':
		testBrake();
		break;
#endif
	case 'G':
		printConfigAsJson();
		break;
	case 'T':
		printStatusAsJson();
		break;
	case '{':
		receiveConfigJson();
		break;
	case 'R':
		rebootBoard();
		break;
	default:
		break;
	}
}

void printMenu() {
	boldHeader("Diagnosis menu");
	p("s: show status\n");
	p("c: show config\n");
	p("t: test motion\n");
#ifdef HAVE_BRAKE
	p("b: test brake\n");
#endif
	p("G: print config as JSON\n");
	p("T: print status as JSON\n");
	p("{: receive config JSON\n");
	p("R: reboot board\n");
}

void testMotion() {
	for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
		if (checkMotionStartConditions(i, dir)) {
			stepper[i].move(FWD, 25600);
		}
	}
}
void printCurrentState() {
	String s;
	bool bt;
	boldHeader("Current Status");
	printHeader("Firmware");
	p("Firmware name : %s\n", FIRMWARE_NAME);
	p("Firmware version : %d.%d.%d\n",firmwareVersion[0],firmwareVersion[1],firmwareVersion[2]);
	p("Compile date : %s, %s\n", COMPILE_DATE, COMPILE_TIME);
	p("Applicable config version : %d.%d\n", applicableConfigVersion[0],applicableConfigVersion[1]);
	p("Loaded config version : %d.%d [%s]\n", loadedConfigVersion[0], loadedConfigVersion[1],
		configVersionCompareString[checkConfigVersion()].c_str());
	showBoolResult(F("Config and product match"), configProductApplicable);
	uint8_t t;
	printHeader("DIP Switch");
	p("BIN : ");
	t = getMyId();
	for (uint8_t i = 0; i < 8; i++) {
		SerialUSB.print(((t >> (7 - i)) & 1) == 1 ? "1" : "0");
		if (i == 3) SerialUSB.print(" ");
	}
	p("\nDEC : %d\n", t);
	printHeader("Ethernet");
	t = Ethernet.hardwareStatus();
	p("Ethernet hardware status: %d ", t);
	switch (t)
	{
	case EthernetNoHardware:
		s = F("-EthernetNoHardware");
		break;
	case EthernetW5100:
		s = F("-EthernetW5100");
		break;
	case EthernetW5200:
		s = F("-EthernetW5200");
		break;
	case EthernetW5500:
		s = F("-EthernetW5500");
		break;
	default:
		break;
	}
	SerialUSB.println(s);
	t = Ethernet.linkStatus();
	p("Ethernet link status: %d ", t);
	switch (t)
	{
	case Unknown:
		s = F("-Unknown");
		break;
	case LinkON:
		s = F("-LinkON");
		break;
	case LinkOFF:
		s = F("-LinkOff");
		break;
	default:
		break;
	}
	SerialUSB.println(s);
	showBoolResult(F("isDestIpSet"), isDestIpSet);
	printIpAddress(F("Destination Ip"), destIp);
	printIpAddress(F("Current My Ip"), myIp);
	p("Current MAC address : %02X:%02X:%02X:%02X:%02X:%02X\n",mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	p("Current outPort : %d\n", outPort);

#ifdef HAVE_SD
	printHeader("microSD");
	showBoolResult(F("SD library initialize succeeded"), sdInitializeSucceeded);
	showBoolResult(F("SD config file open succeeded"), configFileOpenSucceeded);
	showBoolResult(F("SD config JSON parse succeeded"), configFileParseSucceeded);
#endif
	printHeader("Motor Driver");
	uint16_t status[NUM_OF_MOTOR];
	uint32_t temp = 0;
	for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
		status[i] = stepper[i].getStatus();
		temp += status[i];
	}
	showBoolResult(F("Motor driver SPI connection established"), temp != 0);
	if (temp != 0) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
		{
			temp = 0;
			p("Driver ID#%d\n", i+1);
			temp = stepper[i].getPos();
			p("\tABS_POS: 0x%02X (%dstep)\n",temp, temp);
			temp = stepper[i].getElPos();
			p("\tEL_POS: 0x%02X (Fullstep:%d, Microstep:%d)\n", temp, (temp>>7),(temp%0x7F));
			temp = stepper[i].getMark();
			p("\tMARK: 0x%02X (%dstep)\n", temp, temp);
			p("\tSPEED: 0x%02X (%.1fstep/s)\n", stepper[i].getParam(SPEED), stepper[i].getSpeed());
			p("\tACC: 0x%02X (%.1fstep/s/s)\n", stepper[i].getAccRaw(), stepper[i].getAcc());
			p("\tDEC: 0x%02X (%.1fstep/s/s)\n", stepper[i].getDecRaw(), stepper[i].getDec());
			p("\tMAX_SPEED: 0x%02X (%.1fstep/s)\n", stepper[i].getMaxSpeedRaw(), stepper[i].getMaxSpeed());
			p("\tMIN_SPEED: 0x%02X (%.1fstep/s)\n", stepper[i].getMinSpeedRaw(), stepper[i].getMinSpeed());
			p("\tFS_SPD: 0x%02X (%.1fstep/s)\n", stepper[i].getFullSpeedRaw(), stepper[i].getFullSpeed());
			temp = stepper[i].getHoldKVAL();
			p("\tKVAL_HOLD: 0x%02X (%d)\n", temp, temp);
			temp = stepper[i].getRunKVAL();
			p("\tKVAL_RUN: 0x%02X (%d)\n", temp, temp);
			temp = stepper[i].getAccKVAL();
			p("\tKVAL_ACC: 0x%02X (%d)\n", temp, temp);
			temp = stepper[i].getDecKVAL();
			p("\tKVAL_DEC: 0x%02X (%d)\n", temp, temp);
			temp = stepper[i].getParam(INT_SPD);
			p("\tINT_SPD: 0x%02X (%d)\n", temp, temp);
			temp = stepper[i].getParam(ST_SLP);
			p("\tST_SLP: 0x%02X (%d)\n", temp, temp);
			temp = stepper[i].getParam(FN_SLP_ACC);
			p("\tFN_SLP_ACC: 0x%02X (%d)\n", temp, temp);
			temp = stepper[i].getParam(FN_SLP_DEC);
			p("\tFN_SLP_DEC: 0x%02X (%d)\n", temp, temp);
			temp = stepper[i].getParam(K_THERM);
			p("\tK_THERM: 0x%02X (%.1f)\n", temp, (1.0f+(float)temp*0.03125f));
			temp = stepper[i].getParam(ADC_OUT);
			p("\tADC_OUT: 0x%02X (%d)\n", temp, temp);
			temp = stepper[i].getOCThreshold();
#ifdef DRIVER_POWERSTEP01
			p("\tOCD_TH: 0x%02X (%.1fmA)\n", temp, (float)(temp+1)*312.5f);
#elif defined(DRIVER_L6470)
			p("\tOCD_TH: 0x%02X (%.1fmA)\n", temp, (float)(temp+1)*375.0f);
#endif
			temp = stepper[i].getParam(STALL_TH);
#ifdef DRIVER_POWERSTEP01
			p("\tSTALL_TH: 0x%02X (%.1fmA)\n", temp, (float)(temp+1)*312.5f);
#elif defined(DRIVER_L6470)
			p("\tSTALL_TH: 0x%02X (%.1fmA)\n", temp, (float)(temp+1)*31.25f);
#endif
			temp = stepper[i].getStepMode();
			p("\tSTEP_MODE: 0x%02X (%d)\n", temp, temp);
			temp = stepper[i].getParam(ALARM_EN);
			p("\tALARM_EN: 0x%02X (", temp);
			for (uint8_t i = 0; i < 8; i++) {
				SerialUSB.print(((temp >> (7 - i)) & 1) == 1 ? "1" : "0");
				if (i == 3) SerialUSB.print(" ");
			}
			SerialUSB.println(")");

			temp = stepper[i].getParam(CONFIG);
			p("\tCONFIG: 0x%02X\n", temp);
			p("\t  OSC_SEL: %d\n", temp%0x7);
			p("\t  EXT_CLK: %d\n", (temp>>3)%0x1);
			p("\t  SW_MODE: %d\n", (temp>>4)%0x1);
#ifdef DRIVER_POWERSTEP01
			if (isCurrentMode[i])
				p("\t  EN_TQREG: %d\n", (temp>>5)%0x1);
			else
				p("\t  EN_VSCOMP: %d\n", (temp>>5)%0x1);

			p("\t  OC_SD: %d\n", (temp>>7)%0x1);
			p("\t  UVLOVAL: %d\n", (temp>>8)%0x1);
			p("\t  VCCVAL: %d\n", (temp>>9)%0x1);
			if (isCurrentMode[i])
			{
				p("\t  TSW: %d\n", (temp>>10)%0x1F);
				p("\t  PRED_EN: %d\n", (temp>>15)%0x1);
			}
			else{
				p("\t  F_PWM_DEC: %d\n", (temp>>10)%0x7);
				p("\t  F_PWM_INT: %d\n", (temp>>13)%0x7);
			}
			
#elif defined(DRIVER_L6470)
			p("\t  EN_VSCOMP: %d\n", (temp>>5)%0x1);
			p("\t  OC_SD: %d\n", (temp>>7)%0x1);
			p("\t  POW_SR: %d\n", (temp>>8)%0x3);
			p("\t  F_PWM_DEC: %d\n", (temp>>10)%0x7);
			p("\t  F_PWM_INT: %d\n", (temp>>13)%0x7);
#endif

			p("\tSTATUS: 0x%02X\n", status[i]);
			bt = (status[i] & STATUS_HIZ) > 0; // HiZ, high for HiZ
			showBoolResult(F("\t  High impedance state"), bt);
			showBoolResult(F("\t  BUSY"), !(status[i] & STATUS_BUSY));
			bt = (status[i] & STATUS_DIR) > 0;
			s = (bt ? "Forward" : "Reverse");
			p("\t  Motor direction : %s\n", s.c_str());
			t = (status[i] & STATUS_MOT_STATUS) >> 5;
			switch (t)
			{
			case 0:
				s = "Stopped";
				break;
			case 1:
				s = "Acceleration";
				break;
			case 2:
				s = "Deceleration";
				break;
			case 3:
				s = "Constant speed";
				break;
			default:
				break;
			}
			p("\t  Motor status : %s\n", s.c_str());

			t = (status[i] & STATUS_UVLO) == 0;
			showBoolResult("\t  UVLO (Undervoltage lock out)", t);

#ifdef DRIVER_L6470
        	t = (status[i] & (STATUS_TH_WRN|STATUS_TH_SD)) >> 10;
       	 	t = (~t)&0x0003U;
#elif defined(DRIVER_POWERSTEP01)
        	t = (status[i] & STATUS_TH_STATUS) >> 11;
#endif
			switch (t)
			{
			case 0:
				s = "Normal";
				break;
			case 1:
				s = "Warning";
				break;
			case 2:
				s = "Bridge shutdown";
				break;
			case 3:
				s = "Device shutdown";
				break;
			default:
				break;
			}
			p("\t  Thermal status : %s\n", s.c_str());

			t = (status[i] & STATUS_OCD) == 0;
			showBoolResult("\t  OCD (Overcurrent detection)", t);

#ifdef DRIVER_L6470
        	t = (status[i] & (STATUS_STEP_LOSS_A | STATUS_STEP_LOSS_B)) >> 13;
#elif defined(DRIVER_POWERSTEP01)
        	t = (status[i] & (STATUS_STALL_A | STATUS_STALL_B)) >> 14;
#endif
			bt = (t != 3);
			showBoolResult("\t  Stalled", bt);

			// SW_F, low for open, high for close
			bool swF = (status[i] & STATUS_SW_F);
			p("\t  SW_F: %d ", swF);
			if (swF == 1) {
				p("-HOME sensor input closed.\n");
			}
			else {
				p("-HOME sensor input open.\n");
			}
			Watchdog.reset();
		}
	}
	
	printHeader("Modes");
	printAllBools(F("Servo Mode"),isServoMode);
#ifdef DRIVER_POWERSTEP01
	printAllBools(F("Current Mode"), isCurrentMode);
#endif
#ifdef HAVE_BRAKE
	printAllBools(F("Electromagnetic Brake Enable"),electromagnetBrakeEnable);
	p("Brake status :\n");
	String bsText[4] = {"BRAKE_ENGAGED","BRAKE_DISENGAGE_WAITING","BRAKE_DISENGAGED","BRAKE_MOTORHIZ_WAITING"};
	for (uint8_t i=0; i<NUM_OF_MOTOR; i++) {
		p("#%d : %s\n",i+1,bsText[brakeStatus[i]].c_str());
	}
#endif
	printAllData(F("Homing status"), homingStatus);
}

void printConfigurations() {
	boldHeader("Configurations");
	printHeader("Config file");
	//showBoolResult(F("SD library initialize succeeded"), sdInitializeSucceeded);
	showBoolResult(F("SD config file open succeeded"), configFileOpenSucceeded);
	showBoolResult(F("SD config file parse succeeded"), configFileParseSucceeded);
	p("configTargetProduct : %s\n", configTargetProduct.c_str());
	showBoolResult(F("Config and product match"), configProductApplicable);
	p("configName : %s\n", configName.c_str());
	p("config version : %d.%d [%s]\n", loadedConfigVersion[0], loadedConfigVersion[1],
		configVersionCompareString[checkConfigVersion()].c_str());

	printHeader("Network");
	printIpAddress(F("My Ip from Config"), myIp_from_config);
	showBoolResult(F("isMyIpAddId"), isMyIpAddId);
	printIpAddress(F("Dest Ip"), destIp);
	printIpAddress(F("DNS"), dns);
	printIpAddress(F("Gateway"), gateway);
	printIpAddress(F("Subnet mask"), subnet);
	p("MAC address from Config: %02X:%02X:%02X:%02X:%02X:%02X\n",
		mac_from_config[0], 
		mac_from_config[1], 
		mac_from_config[2], 
		mac_from_config[3], 
		mac_from_config[4], 
		mac_from_config[5]);
	showBoolResult(F("isMacAddId"), isMacAddId);
	p("inPort : %d\n", inPort);
	p("outPort from Config : %d\n", outPort_from_config);
	showBoolResult(F("isOutPortAddId"), isOutPortAddId);
	showBoolResult(F("bootedMsgEnable"), bootedMsgEnable);
	showBoolResult(F("isDestIpSet"), isDestIpSet);
	showBoolResult(F("reportErrors"), reportErrors);

	printHeader(F("Report & Alarm"));
	printAllBools(F("reportBUSY"), reportBUSY); 
	printAllBools(F("reportBUSY"), reportBUSY);
	printAllBools(F("reportHiZ"), reportHiZ);
	printAllBools(F("reportHomeSwStatus"), reportHomeSwStatus);
	printAllBools(F("reportDir"), reportDir);
	printAllBools(F("reportMotorStatus"), reportMotorStatus);
	printAllBools(F("reportSwEvn"), reportSwEvn);
	printAllBools(F("reportUVLO"), reportUVLO);
	printAllBools(F("reportThermalStatus"), reportThermalStatus);
	printAllBools(F("reportOCD"), reportOCD);
	printAllBools(F("reportStall"), reportStall);
	printAllBools(F("reportOCD"), reportOCD);
	printAllData(F("reportPosition Interval"), reportPositionInterval);
	p("reportPositionList Interval : %d\n", reportPositionListInterval);
	printAllData("OCThreshold", overCurrentThreshold);

	printHeader("driverSettings");
	printAllBools(F("homingAtStartup"), bHomingAtStartup);
	printAllData(F("homingDirection(1:FWD,0:REV)"), homingDirection);
	// p("homingDirection(1:FWD,0:REV) : %d, %d, %d, %d, %d, %d, %d, %d\n", homingDirection[0], homingDirection[1], homingDirection[2], homingDirection[3], homingDirection[4], homingDirection[5], homingDirection[6], homingDirection[7]);
	printAllData(F("homingSpeed"), homingSpeed);
	printAllData(F("homeSwMode"), homeSwMode);
	printAllBools(F("prohibitMotionOnHomeSw"),bProhibitMotionOnHomeSw);
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
	printAllData(F("limitSwMode"), limitSwMode);
	printAllBools(F("prohibitMotionOnLimitSw"),bProhibitMotionOnLimitSw);
#endif
	printAllData(F("goUntilTimeout"), goUntilTimeout);
	printAllData(F("releaseSwTimeout"), releaseSwTimeout);
	printAllData(F("microStepMode"), microStepMode);
	printAllData(F("slewRate"), slewRateNum);
#ifdef HAVE_BRAKE
	printAllBools(F("electromagnetBrakeEnable"), electromagnetBrakeEnable);
	printAllData(F("brakeTransitionDuration"), brakeTransitionDuration);
#endif
	printHeader(F("speedProfile"));
	printAllData(F("acc"), acc);
	printAllData(F("dec"), dec);
	printAllData(F("maxSpeed"), maxSpeed);
	printAllData(F("fullStepSpeed"), fullStepSpeed);
	printAllData("minSpeed", minSpeed);

	printHeader(F("Voltage mode"));
	printAllData(F("kvalHold"), kvalHold);
	printAllData(F("kvalRun"), kvalRun);
	printAllData(F("kvalAcc"), kvalAcc);
	printAllData(F("kvalDec"), kvalDec);
	printAllData(F("intersectSpeed"), intersectSpeed);
	printAllData(F("startSlope"), startSlope);
	printAllData(F("accFinalSlope"), accFinalSlope);
	printAllData(F("decFinalSlope"), decFinalSlope);
	printAllData(F("stallThreshold"), stallThreshold);
	printAllBools("lowSpeedOptimizeEnable", lowSpeedOptimizeEnable);
	printAllData("lowSpeedOptimizeThreshold", lowSpeedOptimizeThreshold);
#ifdef DRIVER_POWERSTEP01
	printHeader("Current mode");
	printAllData("tvalHold", tvalHold);
	printAllData("tvalRun", tvalRun);
	printAllData("tvalAcc", tvalAcc);
	printAllData("tvalDec", tvalDec);
	printAllData("fastDecaySetting", fastDecaySetting);
	printAllData("minOnTime", minOnTime);
	printAllData("minOffTime", minOffTime);
#endif
	printHeader(F("Servo mode"));
	printAllData(F("kP"), kP);
	printAllData(F("kI"), kI);
	printAllData(F("kD"), kD);
}

void printTitle(String title) {
	p("%s : ", title.c_str());
}
void showBoolResult(String title, bool val) {
	String res = (val) ? "Yes" : "No";
	printTitle(title);
	SerialUSB.println(res);
}

template <class T>
T printAllData(String title, T* val) {
	String out;
	printTitle(title);
	for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
	{
		out += val[i];
		if (i < (NUM_OF_MOTOR-1)) out +=", ";
	}
	SerialUSB.println(out);
}

void printAllBools(String title, bool* val) {
	String out;
	printTitle(title);
	SerialUSB.print(" ");
	for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
	{
		out += (val[i]) ? "Yes" : "No";
		if (i<(NUM_OF_MOTOR-1)) out +=", ";
	}
	
	SerialUSB.println(out);
}

void printIpAddress(String title, IPAddress ip) {
	p("%s : %d.%d.%d.%d\n", title.c_str(), ip[0], ip[1], ip[2], ip[3]);
}

void boldHeader(String header) {
	p("============== %s ==============\n", header.c_str());
}
void printHeader(String header) {
	p("-------------- %s --------------\n", header.c_str());
}
#ifdef HAVE_BRAKE
void testBrake(){
	static uint8_t brakeTestCount =0;
	setBrake(brakeTestCount, LOW);
	p("Turn off #%d, ", brakeTestCount + 1);
	brakeTestCount++;
	if(brakeTestCount >= NUM_OF_MOTOR) {
		brakeTestCount = 0;
	}
	p("Turn on #%d\n", brakeTestCount+ 1);
	setBrake(brakeTestCount, HIGH);
}
#endif

// =============================================================================
// Web Serial API commands
// =============================================================================

void printConfigAsJson() {
	uint8_t i;

	SerialUSB.print(F("{"));

	// Board metadata — each section in its own scope so JsonDocument is freed before the next
	{
		JsonDocument doc;
		JsonObject obj = doc.to<JsonObject>();
		obj["product"] = PRODUCT_NAME;
		obj["numMotors"] = NUM_OF_MOTOR;
		JsonArray fwVer = obj["firmwareVersion"].to<JsonArray>();
		for (i = 0; i < 3; i++) fwVer.add(firmwareVersion[i]);
		SerialUSB.print(F("\"board\":"));
		serializeJson(doc, SerialUSB);
	}
	Watchdog.reset();

	{
		JsonDocument doc;
		JsonObject obj = doc.to<JsonObject>();
		obj["configName"] = configName;
		JsonArray cfgVer = obj["configVersion"].to<JsonArray>();
		cfgVer.add(loadedConfigVersion[0]);
		cfgVer.add(loadedConfigVersion[1]);
		obj["targetProduct"] = configTargetProduct;
		SerialUSB.print(F(",\"information\":"));
		serializeJson(doc, SerialUSB);
	}
	Watchdog.reset();

	{
		JsonDocument doc;
		JsonObject obj = doc.to<JsonObject>();
		JsonArray net_myIp = obj["myIp"].to<JsonArray>();
		for (i = 0; i < 4; i++) net_myIp.add(myIp_from_config[i]);
		JsonArray net_destIp = obj["destIp"].to<JsonArray>();
		for (i = 0; i < 4; i++) net_destIp.add(destIp[i]);
		JsonArray net_dns = obj["dns"].to<JsonArray>();
		for (i = 0; i < 4; i++) net_dns.add(dns[i]);
		JsonArray net_gateway = obj["gateway"].to<JsonArray>();
		for (i = 0; i < 4; i++) net_gateway.add(gateway[i]);
		JsonArray net_subnet = obj["subnet"].to<JsonArray>();
		for (i = 0; i < 4; i++) net_subnet.add(subnet[i]);
		obj["isMyIpAddId"] = isMyIpAddId;
		obj["inPort"] = inPort;
		obj["outPort"] = outPort_from_config;
		obj["isOutPortAddId"] = isOutPortAddId;
		JsonArray net_mac = obj["mac"].to<JsonArray>();
		for (i = 0; i < 6; i++) net_mac.add(mac_from_config[i]);
		obj["isMacAddId"] = isMacAddId;
		obj["bootedMsgEnable"] = bootedMsgEnable;
		obj["canSendMsgBeforeDestIp"] = isDestIpSet;
		obj["reportError"] = reportErrors;
		SerialUSB.print(F(",\"network\":"));
		serializeJson(doc, SerialUSB);
	}
	Watchdog.reset();

	{
		JsonDocument doc;
		JsonObject obj = doc.to<JsonObject>();
		JsonArray ar_reportBUSY = obj["reportBUSY"].to<JsonArray>();
		JsonArray ar_reportHiZ = obj["reportHiZ"].to<JsonArray>();
		JsonArray ar_reportHomeSwStatus = obj["reportHomeSwStatus"].to<JsonArray>();
		JsonArray ar_reportDir = obj["reportDir"].to<JsonArray>();
		JsonArray ar_reportMotorStatus = obj["reportMotorStatus"].to<JsonArray>();
		JsonArray ar_reportSwEvn = obj["reportSwEvn"].to<JsonArray>();
		JsonArray ar_reportUVLO = obj["reportUVLO"].to<JsonArray>();
		JsonArray ar_reportThermalStatus = obj["reportThermalStatus"].to<JsonArray>();
		JsonArray ar_reportOCD = obj["reportOCD"].to<JsonArray>();
		JsonArray ar_reportStall = obj["reportStall"].to<JsonArray>();
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
		JsonArray ar_reportLimitSwStatus = obj["reportLimitSwStatus"].to<JsonArray>();
#endif
		JsonArray ar_OCThreshold = obj["OCThreshold"].to<JsonArray>();
		JsonArray ar_reportPositionInterval = obj["reportPositionInterval"].to<JsonArray>();
		for (i = 0; i < NUM_OF_MOTOR; i++) {
			ar_reportBUSY.add(reportBUSY[i]);
			ar_reportHiZ.add(reportHiZ[i]);
			ar_reportHomeSwStatus.add(reportHomeSwStatus[i]);
			ar_reportDir.add(reportDir[i]);
			ar_reportMotorStatus.add(reportMotorStatus[i]);
			ar_reportSwEvn.add(reportSwEvn[i]);
			ar_reportUVLO.add(reportUVLO[i]);
			ar_reportThermalStatus.add(reportThermalStatus[i]);
			ar_reportOCD.add(reportOCD[i]);
			ar_reportStall.add(reportStall[i]);
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
			ar_reportLimitSwStatus.add(reportLimitSwStatus[i]);
#endif
			ar_OCThreshold.add(overCurrentThreshold[i]);
			ar_reportPositionInterval.add(reportPositionInterval[i]);
		}
		obj["reportPositionListInterval"] = reportPositionListInterval;
		SerialUSB.print(F(",\"alarmAndReport\":"));
		serializeJson(doc, SerialUSB);
	}
	Watchdog.reset();

	{
		JsonDocument doc;
		JsonObject obj = doc.to<JsonObject>();
		JsonArray ds_homingAtStartup = obj["homingAtStartup"].to<JsonArray>();
		JsonArray ds_homingDirection = obj["homingDirection"].to<JsonArray>();
		JsonArray ds_homingSpeed = obj["homingSpeed"].to<JsonArray>();
		JsonArray ds_homeSwMode = obj["homeSwMode"].to<JsonArray>();
		JsonArray ds_prohibitMotionOnHomeSw = obj["prohibitMotionOnHomeSw"].to<JsonArray>();
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
		JsonArray ds_limitSwMode = obj["limitSwMode"].to<JsonArray>();
		JsonArray ds_prohibitMotionOnLimitSw = obj["prohibitMotionOnLimitSw"].to<JsonArray>();
#endif
		JsonArray ds_goUntilTimeout = obj["goUntilTimeout"].to<JsonArray>();
		JsonArray ds_releaseSwTimeout = obj["releaseSwTimeout"].to<JsonArray>();
		JsonArray ds_stepMode = obj["stepMode"].to<JsonArray>();
#ifdef DRIVER_POWERSTEP01
		JsonArray ds_isCurrentMode = obj["isCurrentMode"].to<JsonArray>();
#endif
		JsonArray ds_slewRate = obj["slewRate"].to<JsonArray>();
#ifdef HAVE_BRAKE
		JsonArray ds_electromagnetBrakeEnable = obj["electromagnetBrakeEnable"].to<JsonArray>();
		JsonArray ds_brakeTransitionDuration = obj["brakeTransitionDuration"].to<JsonArray>();
#endif
		for (i = 0; i < NUM_OF_MOTOR; i++) {
			ds_homingAtStartup.add(bHomingAtStartup[i]);
			ds_homingDirection.add(homingDirection[i]);
			ds_homingSpeed.add(homingSpeed[i]);
			ds_homeSwMode.add(homeSwMode[i]);
			ds_prohibitMotionOnHomeSw.add(bProhibitMotionOnHomeSw[i]);
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
			ds_limitSwMode.add(limitSwMode[i]);
			ds_prohibitMotionOnLimitSw.add(bProhibitMotionOnLimitSw[i]);
#endif
			ds_goUntilTimeout.add(goUntilTimeout[i]);
			ds_releaseSwTimeout.add(releaseSwTimeout[i]);
			ds_stepMode.add(microStepMode[i]);
#ifdef DRIVER_POWERSTEP01
			ds_isCurrentMode.add(isCurrentMode[i]);
#endif
			ds_slewRate.add(slewRateNum[i]);
#ifdef HAVE_BRAKE
			ds_electromagnetBrakeEnable.add(electromagnetBrakeEnable[i]);
			ds_brakeTransitionDuration.add(brakeTransitionDuration[i]);
#endif
		}
		SerialUSB.print(F(",\"driverSettings\":"));
		serializeJson(doc, SerialUSB);
	}
	Watchdog.reset();

	{
		JsonDocument doc;
		JsonObject obj = doc.to<JsonObject>();
		JsonArray sp_acc = obj["acc"].to<JsonArray>();
		JsonArray sp_dec = obj["dec"].to<JsonArray>();
		JsonArray sp_maxSpeed = obj["maxSpeed"].to<JsonArray>();
		JsonArray sp_fullStepSpeed = obj["fullStepSpeed"].to<JsonArray>();
		JsonArray sp_minSpeed = obj["minSpeed"].to<JsonArray>();
		for (i = 0; i < NUM_OF_MOTOR; i++) {
			sp_acc.add(acc[i]);
			sp_dec.add(dec[i]);
			sp_maxSpeed.add(maxSpeed[i]);
			sp_fullStepSpeed.add(fullStepSpeed[i]);
			sp_minSpeed.add(minSpeed[i]);
		}
		SerialUSB.print(F(",\"speedProfile\":"));
		serializeJson(doc, SerialUSB);
	}
	Watchdog.reset();

	{
		JsonDocument doc;
		JsonObject obj = doc.to<JsonObject>();
		JsonArray vm_KVAL_HOLD = obj["KVAL_HOLD"].to<JsonArray>();
		JsonArray vm_KVAL_RUN = obj["KVAL_RUN"].to<JsonArray>();
		JsonArray vm_KVAL_ACC = obj["KVAL_ACC"].to<JsonArray>();
		JsonArray vm_KVAL_DEC = obj["KVAL_DEC"].to<JsonArray>();
		JsonArray vm_INT_SPEED = obj["INT_SPEED"].to<JsonArray>();
		JsonArray vm_ST_SLP = obj["ST_SLP"].to<JsonArray>();
		JsonArray vm_FN_SLP_ACC = obj["FN_SLP_ACC"].to<JsonArray>();
		JsonArray vm_FN_SLP_DEC = obj["FN_SLP_DEC"].to<JsonArray>();
		JsonArray vm_STALL_TH = obj["STALL_TH"].to<JsonArray>();
		JsonArray vm_lowSpeedOptimizeEnable = obj["lowSpeedOptimizeEnable"].to<JsonArray>();
		JsonArray vm_lowSpeedOptimize = obj["lowSpeedOptimize"].to<JsonArray>();
		for (i = 0; i < NUM_OF_MOTOR; i++) {
			vm_KVAL_HOLD.add(kvalHold[i]);
			vm_KVAL_RUN.add(kvalRun[i]);
			vm_KVAL_ACC.add(kvalAcc[i]);
			vm_KVAL_DEC.add(kvalDec[i]);
			vm_INT_SPEED.add(intersectSpeed[i]);
			vm_ST_SLP.add(startSlope[i]);
			vm_FN_SLP_ACC.add(accFinalSlope[i]);
			vm_FN_SLP_DEC.add(decFinalSlope[i]);
			vm_STALL_TH.add(stallThreshold[i]);
			vm_lowSpeedOptimizeEnable.add(lowSpeedOptimizeEnable[i]);
			vm_lowSpeedOptimize.add(lowSpeedOptimizeThreshold[i]);
		}
		SerialUSB.print(F(",\"voltageMode\":"));
		serializeJson(doc, SerialUSB);
	}
	Watchdog.reset();

#ifdef DRIVER_POWERSTEP01
	{
		JsonDocument doc;
		JsonObject obj = doc.to<JsonObject>();
		JsonArray cm_TVAL_HOLD = obj["TVAL_HOLD"].to<JsonArray>();
		JsonArray cm_TVAL_RUN = obj["TVAL_RUN"].to<JsonArray>();
		JsonArray cm_TVAL_ACC = obj["TVAL_ACC"].to<JsonArray>();
		JsonArray cm_TVAL_DEC = obj["TVAL_DEC"].to<JsonArray>();
		JsonArray cm_T_FAST = obj["T_FAST"].to<JsonArray>();
		JsonArray cm_TON_MIN = obj["TON_MIN"].to<JsonArray>();
		JsonArray cm_TOFF_MIN = obj["TOFF_MIN"].to<JsonArray>();
		for (i = 0; i < NUM_OF_MOTOR; i++) {
			cm_TVAL_HOLD.add(tvalHold[i]);
			cm_TVAL_RUN.add(tvalRun[i]);
			cm_TVAL_ACC.add(tvalAcc[i]);
			cm_TVAL_DEC.add(tvalDec[i]);
			cm_T_FAST.add(fastDecaySetting[i]);
			cm_TON_MIN.add(minOnTime[i]);
			cm_TOFF_MIN.add(minOffTime[i]);
		}
		SerialUSB.print(F(",\"currentMode\":"));
		serializeJson(doc, SerialUSB);
	}
	Watchdog.reset();
#endif

	{
		JsonDocument doc;
		JsonObject obj = doc.to<JsonObject>();
		JsonArray svm_kP = obj["kP"].to<JsonArray>();
		JsonArray svm_kI = obj["kI"].to<JsonArray>();
		JsonArray svm_kD = obj["kD"].to<JsonArray>();
		for (i = 0; i < NUM_OF_MOTOR; i++) {
			svm_kP.add(kP[i]);
			svm_kI.add(kI[i]);
			svm_kD.add(kD[i]);
		}
		SerialUSB.print(F(",\"servoMode\":"));
		serializeJson(doc, SerialUSB);
	}

	SerialUSB.println(F("}"));
}

void printStatusAsJson() {
	SerialUSB.print(F("{\"board\":{"));
	SerialUSB.print(F("\"product\":\""));
	SerialUSB.print(PRODUCT_NAME);
	SerialUSB.print(F("\",\"firmwareVersion\":["));
	SerialUSB.print(firmwareVersion[0]); SerialUSB.print(',');
	SerialUSB.print(firmwareVersion[1]); SerialUSB.print(',');
	SerialUSB.print(firmwareVersion[2]);
	SerialUSB.print(F("],\"myIp\":["));
	for (uint8_t j = 0; j < 4; j++) {
		if (j > 0) SerialUSB.print(',');
		SerialUSB.print(myIp[j]);
	}
	SerialUSB.print(F("],\"myId\":"));
	SerialUSB.print(myId);
	SerialUSB.print(F(",\"linkStatus\":"));
	SerialUSB.print(Ethernet.linkStatus());
#ifdef HAVE_SD
	SerialUSB.print(F(",\"sdInitialized\":"));
	SerialUSB.print(sdInitializeSucceeded ? F("true") : F("false"));
	SerialUSB.print(F(",\"configLoaded\":"));
	SerialUSB.print(configFileParseSucceeded ? F("true") : F("false"));
#endif
	SerialUSB.print(F("},\"motors\":["));
	for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
		if (i > 0) SerialUSB.print(F(","));
		{
			JsonDocument doc;
			JsonObject obj = doc.to<JsonObject>();
			obj["position"] = stepper[i].getPos();
			obj["speed"] = stepper[i].getSpeed();
			obj["busy"] = busy[i];
			obj["HiZ"] = HiZ[i];
			obj["dir"] = dir[i];
			obj["motorStatus"] = motorStatus[i];
			obj["thermalStatus"] = thermalStatus[i];
			obj["uvlo"] = uvloStatus[i];
			// OCD and stall are latched flags in the STATUS register — reading clears them.
			// checkStatus() in the main loop already reads and reports them via OSC.
			// A second read here would consume and lose the event, so we report false.
			obj["ocd"] = false;
			obj["stall"] = false;
			obj["homeSwState"] = homeSwState[i];
			obj["homingStatus"] = homingStatus[i];
			obj["isServoMode"] = isServoMode[i];
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
			obj["limitSwState"] = limitSwState[i];
#endif
			serializeJson(doc, SerialUSB);
		}
		Watchdog.reset();
	}
	SerialUSB.println(F("]}"));
}

void receiveConfigJson() {
	static char jsonBuf[4096];
	uint16_t pos = 0;
	int depth = 1;
	uint32_t startTime = millis();

	// The opening '{' was already consumed as the command byte
	jsonBuf[pos++] = '{';

	while (depth > 0) {
		Watchdog.reset();
		if ((millis() - startTime) > 10000UL) {
			SerialUSB.println(F("{\"result\":\"error\",\"message\":\"Timeout\"}"));
			return;
		}
		if (!SerialUSB.available()) continue;
		char c = (char)SerialUSB.read();
		if (pos >= sizeof(jsonBuf) - 1) {
			SerialUSB.println(F("{\"result\":\"error\",\"message\":\"Buffer overflow\"}"));
			return;
		}
		jsonBuf[pos++] = c;
		if (c == '{') depth++;
		else if (c == '}') { depth--; }
	}
	jsonBuf[pos] = '\0';

	JsonDocument doc;
	DeserializationError error = deserializeJson(doc, jsonBuf);
	if (error) {
		SerialUSB.print(F("{\"result\":\"error\",\"message\":\"JSON parse: "));
		SerialUSB.print(error.f_str());
		SerialUSB.println(F("\"}"));
		return;
	}

	applyConfigFromJson(doc);
	saveConfigToSd(doc);
}

void applyConfigFromJson(JsonDocument& doc) {
	uint8_t i;

	// Network
	JsonObject network = doc["network"];
	JsonArray network_myIp = network["myIp"];
	JsonArray network_destIp = network["destIp"];
	JsonArray network_dns = network["dns"];
	JsonArray network_gateway = network["gateway"];
	JsonArray network_subnet = network["subnet"];
	for (i = 0; i < 4; i++) {
		myIp[i] = network_myIp[i] | myIp[i];
		myIp_from_config[i] = myIp[i];
		destIp[i] = network_destIp[i] | destIp[i];
		dns[i] = network_dns[i] | dns[i];
		gateway[i] = network_gateway[i] | gateway[i];
		subnet[i] = network_subnet[i] | subnet[i];
	}
	isMyIpAddId = network["isMyIpAddId"] | isMyIpAddId;
	inPort = network["inPort"] | inPort;
	outPort = network["outPort"] | outPort;
	outPort_from_config = outPort;
	isOutPortAddId = network["isOutPortAddId"] | isOutPortAddId;
	JsonArray network_mac = network["mac"];
	for (i = 0; i < 6; i++) {
		mac[i] = network_mac[i] | mac[i];
		mac_from_config[i] = mac[i];
	}
	isMacAddId = network["isMacAddId"] | isMacAddId;
	bootedMsgEnable = network["bootedMsgEnable"] | bootedMsgEnable;
	isDestIpSet = network["canSendMsgBeforeDestIp"] | isDestIpSet;
	reportErrors = network["reportError"] | reportErrors;

	// Alarm and Report
	JsonObject alarmAndReport = doc["alarmAndReport"];
	JsonArray ar_reportBUSY = alarmAndReport["reportBUSY"];
	JsonArray ar_reportHiZ = alarmAndReport["reportHiZ"];
	JsonArray ar_reportHomeSwStatus = alarmAndReport["reportHomeSwStatus"];
	JsonArray ar_reportDir = alarmAndReport["reportDir"];
	JsonArray ar_reportMotorStatus = alarmAndReport["reportMotorStatus"];
	JsonArray ar_reportSwEvn = alarmAndReport["reportSwEvn"];
	JsonArray ar_reportUVLO = alarmAndReport["reportUVLO"];
	JsonArray ar_reportThermalStatus = alarmAndReport["reportThermalStatus"];
	JsonArray ar_reportOCD = alarmAndReport["reportOCD"];
	JsonArray ar_reportStall = alarmAndReport["reportStall"];
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
	JsonArray ar_reportLimitSwStatus = alarmAndReport["reportLimitSwStatus"];
#endif
	JsonArray ar_OCThreshold = alarmAndReport["OCThreshold"];
	JsonArray ar_reportPositionInterval = alarmAndReport["reportPositionInterval"];
	for (i = 0; i < NUM_OF_MOTOR; i++) {
		reportBUSY[i] = ar_reportBUSY[i] | reportBUSY[i];
		reportHiZ[i] = ar_reportHiZ[i] | reportHiZ[i];
		reportHomeSwStatus[i] = ar_reportHomeSwStatus[i] | reportHomeSwStatus[i];
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
		reportLimitSwStatus[i] = ar_reportLimitSwStatus[i] | reportLimitSwStatus[i];
#endif
		reportDir[i] = ar_reportDir[i] | reportDir[i];
		reportMotorStatus[i] = ar_reportMotorStatus[i] | reportMotorStatus[i];
		reportSwEvn[i] = ar_reportSwEvn[i] | reportSwEvn[i];
		reportUVLO[i] = ar_reportUVLO[i] | reportUVLO[i];
		reportThermalStatus[i] = ar_reportThermalStatus[i] | reportThermalStatus[i];
		reportOCD[i] = ar_reportOCD[i] | reportOCD[i];
		reportStall[i] = ar_reportStall[i] | reportStall[i];
		overCurrentThreshold[i] = ar_OCThreshold[i] | overCurrentThreshold[i];
		reportPositionInterval[i] = ar_reportPositionInterval[i] | reportPositionInterval[i];
		reportPosition[i] = reportPositionInterval[i] > 0;
	}
	reportPositionListInterval = alarmAndReport["reportPositionListInterval"] | reportPositionListInterval;
	reportPositionList = reportPositionListInterval > 0;
	if (reportPositionList) {
		for (i = 0; i < NUM_OF_MOTOR; i++) {
			reportPosition[i] = false;
			reportPositionInterval[i] = 0;
		}
	}

	// Driver Settings
	JsonObject driverSettings = doc["driverSettings"];
	JsonArray ds_homingAtStartup = driverSettings["homingAtStartup"];
	JsonArray ds_homingDirection = driverSettings["homingDirection"];
	JsonArray ds_homingSpeed = driverSettings["homingSpeed"];
	JsonArray ds_homeSwMode = driverSettings["homeSwMode"];
	JsonArray ds_prohibitMotionOnHomeSw = driverSettings["prohibitMotionOnHomeSw"];
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
	JsonArray ds_limitSwMode = driverSettings["limitSwMode"];
	JsonArray ds_prohibitMotionOnLimitSw = driverSettings["prohibitMotionOnLimitSw"];
#endif
	JsonArray ds_goUntilTimeout = driverSettings["goUntilTimeout"];
	JsonArray ds_releaseSwTimeout = driverSettings["releaseSwTimeout"];
	JsonArray ds_stepMode = driverSettings["stepMode"];
#ifdef DRIVER_POWERSTEP01
	JsonArray ds_isCurrentMode = driverSettings["isCurrentMode"];
#endif
	JsonArray ds_slewRate = driverSettings["slewRate"];
#ifdef HAVE_BRAKE
	JsonArray ds_electromagnetBrakeEnable = driverSettings["electromagnetBrakeEnable"];
	JsonArray ds_brakeTransitionDuration = driverSettings["brakeTransitionDuration"];
#endif
#ifdef DRIVER_POWERSTEP01
	uint16_t slewRateVal[6] = { SR_114V_us, SR_220V_us, SR_400V_us, SR_520V_us, SR_790V_us, SR_980V_us };
#elif defined(DRIVER_L6470)
	uint16_t slewRateVal[3] = { SR_180V_us, SR_290V_us, SR_530V_us };
#endif
	for (i = 0; i < NUM_OF_MOTOR; i++) {
		bHomingAtStartup[i] = ds_homingAtStartup[i] | bHomingAtStartup[i];
		homingDirection[i] = ds_homingDirection[i] | homingDirection[i];
		homingSpeed[i] = ds_homingSpeed[i] | homingSpeed[i];
		homeSwMode[i] = ds_homeSwMode[i] | homeSwMode[i];
		bProhibitMotionOnHomeSw[i] = ds_prohibitMotionOnHomeSw[i] | bProhibitMotionOnHomeSw[i];
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
		limitSwMode[i] = ds_limitSwMode[i] | limitSwMode[i];
		bProhibitMotionOnLimitSw[i] = ds_prohibitMotionOnLimitSw[i] | bProhibitMotionOnLimitSw[i];
#endif
		goUntilTimeout[i] = ds_goUntilTimeout[i] | goUntilTimeout[i];
		releaseSwTimeout[i] = ds_releaseSwTimeout[i] | releaseSwTimeout[i];
		microStepMode[i] = ds_stepMode[i] | microStepMode[i];
#ifdef DRIVER_POWERSTEP01
		isCurrentMode[i] = ds_isCurrentMode[i] | isCurrentMode[i];
		slewRateNum[i] = constrain((ds_slewRate[i] | slewRateNum[i]), 0, 5);
#elif defined(DRIVER_L6470)
		slewRateNum[i] = constrain((ds_slewRate[i] | slewRateNum[i]), 0, 2);
#endif
		slewRate[i] = slewRateVal[slewRateNum[i]];
#ifdef HAVE_BRAKE
		electromagnetBrakeEnable[i] = ds_electromagnetBrakeEnable[i] | electromagnetBrakeEnable[i];
		brakeTransitionDuration[i] = ds_brakeTransitionDuration[i] | brakeTransitionDuration[i];
#endif
	}

	// Speed Profile
	JsonObject speedProfile = doc["speedProfile"];
	JsonArray sp_acc = speedProfile["acc"];
	JsonArray sp_dec = speedProfile["dec"];
	JsonArray sp_maxSpeed = speedProfile["maxSpeed"];
	JsonArray sp_fullStepSpeed = speedProfile["fullStepSpeed"];
	JsonArray sp_minSpeed = speedProfile["minSpeed"];
	for (i = 0; i < NUM_OF_MOTOR; i++) {
		acc[i] = sp_acc[i] | acc[i];
		dec[i] = sp_dec[i] | dec[i];
		maxSpeed[i] = sp_maxSpeed[i] | maxSpeed[i];
		fullStepSpeed[i] = sp_fullStepSpeed[i] | fullStepSpeed[i];
		minSpeed[i] = sp_minSpeed[i] | minSpeed[i];
	}

	// Voltage Mode
	JsonObject voltageMode = doc["voltageMode"];
	JsonArray vm_KVAL_HOLD = voltageMode["KVAL_HOLD"];
	JsonArray vm_KVAL_RUN = voltageMode["KVAL_RUN"];
	JsonArray vm_KVAL_ACC = voltageMode["KVAL_ACC"];
	JsonArray vm_KVAL_DEC = voltageMode["KVAL_DEC"];
	JsonArray vm_INT_SPEED = voltageMode["INT_SPEED"];
	JsonArray vm_ST_SLP = voltageMode["ST_SLP"];
	JsonArray vm_FN_SLP_ACC = voltageMode["FN_SLP_ACC"];
	JsonArray vm_FN_SLP_DEC = voltageMode["FN_SLP_DEC"];
	JsonArray vm_STALL_TH = voltageMode["STALL_TH"];
	JsonArray vm_lowSpeedOptimizeEnable = voltageMode["lowSpeedOptimizeEnable"];
	JsonArray vm_lowSpeedOptimize = voltageMode["lowSpeedOptimize"];
	for (i = 0; i < NUM_OF_MOTOR; i++) {
		kvalHold[i] = vm_KVAL_HOLD[i] | kvalHold[i];
		kvalRun[i] = vm_KVAL_RUN[i] | kvalRun[i];
		kvalAcc[i] = vm_KVAL_ACC[i] | kvalAcc[i];
		kvalDec[i] = vm_KVAL_DEC[i] | kvalDec[i];
		intersectSpeed[i] = vm_INT_SPEED[i] | intersectSpeed[i];
		startSlope[i] = vm_ST_SLP[i] | startSlope[i];
		accFinalSlope[i] = vm_FN_SLP_ACC[i] | accFinalSlope[i];
		decFinalSlope[i] = vm_FN_SLP_DEC[i] | decFinalSlope[i];
		stallThreshold[i] = vm_STALL_TH[i] | stallThreshold[i];
		lowSpeedOptimizeEnable[i] = vm_lowSpeedOptimizeEnable[i] | lowSpeedOptimizeEnable[i];
		lowSpeedOptimizeThreshold[i] = vm_lowSpeedOptimize[i] | lowSpeedOptimizeThreshold[i];
	}

#ifdef DRIVER_POWERSTEP01
	// Current Mode
	JsonObject currentMode = doc["currentMode"];
	JsonArray cm_TVAL_HOLD = currentMode["TVAL_HOLD"];
	JsonArray cm_TVAL_RUN = currentMode["TVAL_RUN"];
	JsonArray cm_TVAL_ACC = currentMode["TVAL_ACC"];
	JsonArray cm_TVAL_DEC = currentMode["TVAL_DEC"];
	JsonArray cm_T_FAST = currentMode["T_FAST"];
	JsonArray cm_TON_MIN = currentMode["TON_MIN"];
	JsonArray cm_TOFF_MIN = currentMode["TOFF_MIN"];
	for (i = 0; i < NUM_OF_MOTOR; i++) {
		tvalHold[i] = cm_TVAL_HOLD[i] | tvalHold[i];
		tvalRun[i] = cm_TVAL_RUN[i] | tvalRun[i];
		tvalAcc[i] = cm_TVAL_ACC[i] | tvalAcc[i];
		tvalDec[i] = cm_TVAL_DEC[i] | tvalDec[i];
		fastDecaySetting[i] = cm_T_FAST[i] | fastDecaySetting[i];
		minOnTime[i] = cm_TON_MIN[i] | minOnTime[i];
		minOffTime[i] = cm_TOFF_MIN[i] | minOffTime[i];
	}
#endif

	// Servo Mode
	JsonObject servoMode = doc["servoMode"];
	JsonArray svm_kP = servoMode["kP"];
	JsonArray svm_kI = servoMode["kI"];
	JsonArray svm_kD = servoMode["kD"];
	for (i = 0; i < NUM_OF_MOTOR; i++) {
		kP[i] = svm_kP[i] | kP[i];
		kI[i] = svm_kI[i] | kI[i];
		kD[i] = svm_kD[i] | kD[i];
	}
}

void saveConfigToSd(JsonDocument& doc) {
#ifdef HAVE_SD
	doc.remove("board"); // strip runtime metadata before writing

	SD.begin(PIN_SD_CS);
	SD.remove(filename);
	File file = SD.open(filename, FILE_WRITE);
	if (!file) {
		SerialUSB.println(F("{\"result\":\"error\",\"message\":\"SD open failed\"}"));
		return;
	}
	serializeJsonPretty(doc, file);
	file.close();
	SerialUSB.println(F("{\"result\":\"ok\",\"message\":\"Saved to SD. Reboot (send 'R') to apply.\"}"));
#else
	(void)doc;
	SerialUSB.println(F("{\"result\":\"error\",\"message\":\"No SD support on this board\"}"));
#endif
}

void rebootBoard() {
	SerialUSB.println(F("{\"result\":\"ok\",\"message\":\"Rebooting...\"}"));
	SerialUSB.flush();
	NVIC_SystemReset();
}
