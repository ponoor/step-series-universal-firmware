// oscRoutes.h
// OSC address table, in the order of the former OSCMessage::route() calls.
// The order matters when a received address pattern matches several entries
// (pattern matching path of oscRouter.cpp): handlers are called in this order.
//
// Usage: define OSC_ROUTE(address, handler, args) and include this file. No include guard.
// args: the arguments the handler needs, one character each (oscArgs.h): m motor ID,
// n number (int or float), b boolean-like (number or OSC true / false), s string,
// L one number per motor. Checked before the handler is called; extra arguments are ignored.

// some possible frequent messages
OSC_ROUTE("/setTargetPosition", setTargetPosition, "mn")
OSC_ROUTE("/setTargetPositionList", setTargetPositionList, "L")
OSC_ROUTE("/getPosition", getPosition, "m")
OSC_ROUTE("/getPositionList", getPositionList, "")

OSC_ROUTE("/getSpeed", getSpeed, "m")
OSC_ROUTE("/run", run, "mn")
OSC_ROUTE("/runRaw", runRaw, "mn")

// combined
OSC_ROUTE("/combine/move", combinedMove, "mmn")
OSC_ROUTE("/combine/goTo", combinedGoTo, "mmn")
OSC_ROUTE("/combine/goToDir", combinedGoToDir, "mmbn")
OSC_ROUTE("/combine/run", combinedRun, "mmn")
OSC_ROUTE("/combine/runRaw", combinedRunRaw, "mmn")
OSC_ROUTE("/combine/hardHiZ", combinedHardHiZ, "mm")
OSC_ROUTE("/combine/hardStop", combinedHardStop, "mm")
OSC_ROUTE("/combine/softHiZ", combinedSoftHiZ, "mm")
OSC_ROUTE("/combine/softStop", combinedSoftStop, "mm")

// motion
OSC_ROUTE("/move", move, "mn")
OSC_ROUTE("/goTo", goTo, "mn")
OSC_ROUTE("/goToDir", goToDir, "mbn")
OSC_ROUTE("/goUntil", goUntil, "mbn")
OSC_ROUTE("/goUntilRaw", goUntilRaw, "mbn")
OSC_ROUTE("/releaseSw", releaseSw, "mbb")
OSC_ROUTE("/goHome", goHome, "m")
OSC_ROUTE("/goMark", goMark, "m")
OSC_ROUTE("/setMark", setMark, "mn")
OSC_ROUTE("/getMark", getMark, "m")
OSC_ROUTE("/setPosition", setPosition, "mn")
OSC_ROUTE("/resetPos", resetPos, "m")
OSC_ROUTE("/softStop", softStop, "m")
OSC_ROUTE("/hardStop", hardStop, "m")
OSC_ROUTE("/softHiZ", softHiZ, "m")
OSC_ROUTE("/hardHiZ", hardHiZ, "m")
#ifdef HAVE_BRAKE
OSC_ROUTE("/activate", activate, "mb")
OSC_ROUTE("/free", free, "m")
#endif
OSC_ROUTE("/homing", homing, "m")
// servo mode
OSC_ROUTE("/enableServoMode", enableServoMode, "mb")
OSC_ROUTE("/setServoParam", setServoParam, "mnnn")
OSC_ROUTE("/getServoParam", getServoParam, "m")

// speed
OSC_ROUTE("/setSpeedProfile", setSpeedProfile, "mnnn")
OSC_ROUTE("/setMaxSpeed", setMaxSpeed, "mn")
OSC_ROUTE("/setMinSpeed", setMinSpeed, "mn")
OSC_ROUTE("/getMinSpeed", getMinSpeed, "m")
OSC_ROUTE("/setFullstepSpeed", setFullstepSpeed, "mn")
OSC_ROUTE("/getFullstepSpeed", getFullstepSpeed, "m")
OSC_ROUTE("/setAcc", setAcc, "mn")
OSC_ROUTE("/setDec", setDec, "mn")
OSC_ROUTE("/getSpeedProfile", getSpeedProfile, "m")

// Kval
OSC_ROUTE("/setKval", setKval, "mnnnn")
OSC_ROUTE("/setAccKval", setAccKval, "mn")
OSC_ROUTE("/setDecKval", setDecKval, "mn")
OSC_ROUTE("/setRunKval", setRunKval, "mn")
OSC_ROUTE("/setHoldKval", setHoldKval, "mn")
OSC_ROUTE("/getKval", getKval, "m")

// TVAL
#ifdef DRIVER_POWERSTEP01
OSC_ROUTE("/setTval", setTval, "mnnnn")
OSC_ROUTE("/setAccTval", setAccTval, "mn")
OSC_ROUTE("/setDecTval", setDecTval, "mn")
OSC_ROUTE("/setRunTval", setRunTval, "mn")
OSC_ROUTE("/setHoldTval", setHoldTval, "mn")
OSC_ROUTE("/getTval", getTval, "m")
OSC_ROUTE("/getTval_mA", getTval_mA, "m")
#endif
// config
OSC_ROUTE("/setDestIp", setDestIp, "")
OSC_ROUTE("/getVersion", getVersion, "")
OSC_ROUTE("/getConfigName", getConfigName, "")
OSC_ROUTE("/setConfigName", setConfigName, "s")
OSC_ROUTE("/getConfigRegister", getConfigRegister, "m")
OSC_ROUTE("/getStatus", getStatus, "m")
OSC_ROUTE("/getStatusList", getStatusList, "")
OSC_ROUTE("/reportError", reportError, "b")
OSC_ROUTE("/getHomeSw", getHomeSw, "m")
OSC_ROUTE("/getBusy", getBusy, "m")
OSC_ROUTE("/getDir", getDir, "m")
OSC_ROUTE("/getHiZ", getHiZ, "m")
OSC_ROUTE("/getUvlo", getUvlo, "m")
OSC_ROUTE("/getMotorStatus", getMotorStatus, "m")
OSC_ROUTE("/getThermalStatus", getThermalStatus, "m")
OSC_ROUTE("/resetMotorDriver", resetMotorDriver, "m")
OSC_ROUTE("/enableBusyReport", enableBusyReport, "mb")
OSC_ROUTE("/enableHizReport", enableHizReport, "mb")
OSC_ROUTE("/enableHomeSwReport", enableHomeSwReport, "mb")
OSC_ROUTE("/enableDirReport", enableDirReport, "mb")
OSC_ROUTE("/enableMotorStatusReport", enableMotorStatusReport, "mb")
OSC_ROUTE("/enableSwEventReport", enableSwEventReport, "mb")
OSC_ROUTE("/enableUvloReport", enableUvloReport, "mb")
OSC_ROUTE("/enableThermalStatusReport", enableThermalStatusReport, "mb")
OSC_ROUTE("/enableOverCurrentReport", enableOverCurrentReport, "mb")
OSC_ROUTE("/enableStallReport", enableStallReport, "mb")
OSC_ROUTE("/setPositionReportInterval", setPositionReportInterval, "mn")
OSC_ROUTE("/setPositionListReportInterval", setPositionListReportInterval, "n")
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
OSC_ROUTE("/getLimitSw", getLimitSw, "m")
OSC_ROUTE("/getLimitSwMode", getLimitSwMode, "m")
OSC_ROUTE("/setLimitSwMode", setLimitSwMode, "mb")
OSC_ROUTE("/enableLimitSwReport", enableLimitSwReport, "mb")
#endif
#ifdef HAVE_BRAKE
OSC_ROUTE("/enableElectromagnetBrake", enableElectromagnetBrake, "mb")
OSC_ROUTE("/setBrakeTransitionDuration", setBrakeTransitionDuration, "mn")
OSC_ROUTE("/getBrakeTransitionDuration", getBrakeTransitionDuration, "m")
OSC_ROUTE("/getBrakeStatus", getBrakeStatus, "m")
#endif
OSC_ROUTE("/setMicrostepMode", setMicrostepMode, "mn")
OSC_ROUTE("/getMicrostepMode", getMicrostepMode, "m")
OSC_ROUTE("/getHomeSwMode", getHomeSwMode, "m")
OSC_ROUTE("/setHomeSwMode", setHomeSwMode, "mb")
OSC_ROUTE("/setStallThreshold", setStallThreshold, "mn")
OSC_ROUTE("/getStallThreshold", getStallThreshold, "m")
OSC_ROUTE("/setOverCurrentThreshold", setOverCurrentThreshold, "mn")
OSC_ROUTE("/getOverCurrentThreshold", getOverCurrentThreshold, "m")
OSC_ROUTE("/enableLowSpeedOptimize", enableLowSpeedOptimize, "mb")
OSC_ROUTE("/setLowSpeedOptimizeThreshold", setLowSpeedOptimizeThreshold, "mn")
OSC_ROUTE("/getLowSpeedOptimizeThreshold", getLowSpeedOptimizeThreshold, "m")

OSC_ROUTE("/setSpeedProfileRaw", setSpeedProfileRaw, "mnnn")
OSC_ROUTE("/setMaxSpeedRaw", setMaxSpeedRaw, "mn")
OSC_ROUTE("/setMinSpeedRaw", setMinSpeedRaw, "mn")
OSC_ROUTE("/setFullstepSpeedRaw", setFullstepSpeedRaw, "mn")
OSC_ROUTE("/setAccRaw", setAccRaw, "mn")
OSC_ROUTE("/setDecRaw", setDecRaw, "mn")
OSC_ROUTE("/getSpeedProfileRaw", getSpeedProfileRaw, "m")

OSC_ROUTE("/setBemfParam", setBemfParam, "mnnnn")
OSC_ROUTE("/getBemfParam", getBemfParam, "m")
#ifdef DRIVER_POWERSTEP01
OSC_ROUTE("/setVoltageMode", setVoltageMode, "m")
OSC_ROUTE("/setCurrentMode", setCurrentMode, "m")
OSC_ROUTE("/setDecayModeParam", setDecayModeParam, "mnnn")
OSC_ROUTE("/getDecayModeParam", getDecayModeParam, "m")
#endif
OSC_ROUTE("/getAdcVal", getAdcVal, "m")
OSC_ROUTE("/setGoUntilTimeout", setGoUntilTimeout, "mn")
OSC_ROUTE("/getGoUntilTimeout", getGoUntilTimeout, "m")
OSC_ROUTE("/setReleaseSwTimeout", setReleaseSwTimeout, "mn")
OSC_ROUTE("/getReleaseSwTimeout", getReleaseSwTimeout, "m")
OSC_ROUTE("/setHomingDirection", setHomingDirection, "mb")
OSC_ROUTE("/getHomingDirection", getHomingDirection, "m")
OSC_ROUTE("/setHomingSpeed", setHomingSpeed, "mn")
OSC_ROUTE("/getHomingSpeed", getHomingSpeed, "m")
OSC_ROUTE("/getHomingStatus", getHomingStatus, "m")
OSC_ROUTE("/setProhibitMotionOnHomeSw", setProhibitMotionOnHomeSw, "mb")
OSC_ROUTE("/getProhibitMotionOnHomeSw", getProhibitMotionOnHomeSw, "m")
#if defined(HAVE_LIMIT_ADC) || defined(HAVE_LIMIT_GPIO)
OSC_ROUTE("/setProhibitMotionOnLimitSw", setProhibitMotionOnLimitSw, "mb")
OSC_ROUTE("/getProhibitMotionOnLimitSw", getProhibitMotionOnLimitSw, "m")
#endif
OSC_ROUTE("/getElPos", getElPos, "m")
OSC_ROUTE("/setElPos", setElPos, "mnn")
OSC_ROUTE("/resetDevice", resetDevice, "")
OSC_ROUTE("/saveConfig", saveConfig, "")
