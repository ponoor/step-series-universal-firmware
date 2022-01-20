// Program demonstrating how to control a powerSTEP01-based ST X-NUCLEO-IHM03A1 
// stepper motor driver shield on an Arduino Zero-compatible board

#include <Ponoor_PowerSTEP01Library.h>
#include "wiring_private.h" // pinPeripheral() function
#include <SPI.h>

// SPI setup for zero/M0
#define MISO1 12  // D6 /SERCOM3/PAD[2] miso
#define MOSI1 11  // D11/SERCOM3/PAD[0] mosi
#define SCK1  13  // D12/SERCOM3/PAD[3] sck
SPIClass altSPI (&sercom1, MISO1, SCK1, MOSI1, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);

// Pin definitions for the X-NUCLEO-IHM03A1 connected to an Uno-compatible board
#define nCS_PIN 10
#define STCK_PIN 9
#define nSTBY_nRESET_PIN 8
#define nBUSY_PIN 4

// powerSTEP library instance, parameters are distance from the end of a daisy-chain
// of drivers, !CS pin, !STBY/!Reset pin
powerSTEP driver(0, nCS_PIN, nSTBY_nRESET_PIN);

void setup() 
{
  // Start serial
  SerialUSB.begin(9600);
  SerialUSB.println("powerSTEP01 Arduino control initialising...");

  // Prepare pins
  pinMode(nSTBY_nRESET_PIN, OUTPUT);
  pinMode(nCS_PIN, OUTPUT);

  // Reset powerSTEP and set CS
  digitalWrite(nSTBY_nRESET_PIN, HIGH);
  digitalWrite(nSTBY_nRESET_PIN, LOW);
  digitalWrite(nSTBY_nRESET_PIN, HIGH);
  digitalWrite(nCS_PIN, HIGH);

  // Start SPI for PowerSTEP
  altSPI.begin();
  pinPeripheral(MOSI1, PIO_SERCOM);
  pinPeripheral(SCK1, PIO_SERCOM);
  pinPeripheral(MISO1 , PIO_SERCOM);
  altSPI.setDataMode(SPI_MODE3);

  // Configure powerSTEP
  driver.SPIPortConnect(&altSPI); // give library the SPI port (only the one on an Uno)
  
  driver.configSyncPin(BUSY_PIN, 0); // use SYNC/nBUSY pin as nBUSY, 
                                     // thus syncSteps (2nd paramater) does nothing
                                     
  driver.configStepMode(STEP_FS_16); // 1/16 microstepping, full steps = STEP_FS,
                                // options: 1, 1/2, 1/4, 1/8, 1/16, 1/32, 1/64, 1/128
                                // in the current mode, microstepping is limited minimum 1/16
                                
  driver.setMaxSpeed(650); // max speed in units of full steps/s 
  driver.setFullSpeed(2000); // full steps/s threshold for disabling microstepping
  driver.setAcc(10000); // full steps/s^2 acceleration
  driver.setDec(10000); // full steps/s^2 deceleration
  
  driver.setSlewRate(SR_520V_us); // faster may give more torque (but also EM noise),
                                  // options are: 114, 220, 400, 520, 790, 980(V/us)
                                  
  driver.setOCThreshold(8); // over-current threshold for the 2.8A NEMA23 motor
                            // used in testing. If your motor stops working for
                            // no apparent reason, it's probably this. Start low
                            // and increase until it doesn't trip, then maybe
                            // add one to avoid misfires. Can prevent catastrophic
                            // failures caused by shorts
  driver.setOCShutdown(OC_SD_ENABLE); // shutdown motor bridge on over-current event
                                      // to protect against permanant damage
  
  driver.setPWMFreq(PWM_DIV_1, PWM_MUL_0_75); // 16MHz*0.75/(512*1) = 23.4375kHz 
                            // power is supplied to stepper phases as a sin wave,  
                            // frequency is set by two PWM modulators,
                            // Fpwm = Fosc*m/(512*N), N and m are set by DIV and MUL,
                            // options: DIV: 1, 2, 3, 4, 5, 6, 7, 
                            // MUL: 0.625, 0.75, 0.875, 1, 1.25, 1.5, 1.75, 2
                            
  driver.setVoltageComp(VS_COMP_DISABLE); // no compensation for variation in Vs as
                                          // ADC voltage divider is not populated
                                          
  driver.setSwitchMode(SW_USER); // switch doesn't trigger stop, status can be read.
                                 // SW_HARD_STOP: TP1 causes hard stop on connection 
                                 // to GND, you get stuck on switch after homing
                                      
  driver.setOscMode(INT_16MHZ); // 16MHz internal oscillator as clock source

  // KVAL registers set the power to the motor by adjusting the PWM duty cycle,
  // use a value between 0-255 where 0 = no power, 255 = full power.
  // Start low and monitor the motor temperature until you find a safe balance
  // between power and temperature. Only use what you need
  driver.setRunKVAL(64);
  driver.setAccKVAL(64);
  driver.setDecKVAL(64);
  driver.setHoldKVAL(8);

  driver.setParam(ALARM_EN, 0x8F); // disable ADC UVLO (divider not populated),
                                   // disable stall detection (not configured),
                                   // disable switch (not using as hard stop)

  driver.getStatus(); // clears error flags

  SerialUSB.println(F("Initialisation complete"));
}

void oneMotion() {
  driver.move(FWD, 12800); // move forward 12800 microsteps
  while(driver.busyCheck()); // wait fo the move to finish
  driver.softStop(); // soft stops prevent errors in the next operation
  while(driver.busyCheck());
  
  driver.move(REV, 12800); // reverse back
  while(driver.busyCheck());
  driver.softStop();
  while(driver.busyCheck());
}

void loop() 
{ 
  SerialUSB.println("Voltage mode");
  driver.softHiZ();
  driver.setMaxSpeed(650.);
  driver.setRunKVAL(64);
  driver.setAccKVAL(64);
  driver.setDecKVAL(64);
  driver.setHoldKVAL(8);
  driver.setVoltageMode();
  oneMotion();
  delay(1000);

  SerialUSB.println("Current mode");
  driver.softHiZ();
  driver.setMaxSpeed(3000.); // In general, the current mode spins much faster.
  driver.setRunTVAL(8);
  driver.setAccTVAL(8);
  driver.setDecTVAL(8);
  driver.setHoldTVAL(2);
  driver.setCurrentMode();
  oneMotion();
  delay(1000);
}
