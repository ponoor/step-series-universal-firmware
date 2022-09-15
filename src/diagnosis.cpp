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
