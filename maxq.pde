#include <avr/pgmspace.h>
#include <avr/io.h>
#include "utils.h"
#include <WProgram.h>
#include <Wire.h>
#include <DS1307.h>
#include <EEPROM.h>

#include <SoftwareSerial.h>

#define rxPin 4
#define txPin 3

SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);

#if (ENABLE_WATCHDOG)
  #include <avr/wdt.h> 
#endif

bufferMeas ppMAXQ[MAX_BUFFER_RAM_MEAS]; // cantidad de dispositivos buffereados en mediciones
bufferMeas _ppMAXQ; // temp to process queues

Queue_MNG QueueDataToSync_MNG; // store pointers to manage memory EEPROM and RAM

unsigned long time_from_last_sample = 30000; // time from last MAXQ sample
unsigned long last_sync_check = 0; // last time sync with server
unsigned long last_time_energy_overflow_check = 0; // store time from last overflow register checking
unsigned long last_time_datetime_synced = 0;
unsigned long last_time_cummulative_eeprom_synced = 0; // store last time cummulative overflow registers has been eeprom saved (locally)
boolean calibration_enabled = false;
boolean sampling_on_boot = true;
boolean sync_time_at_boot = true;

BufferRS232Packet bufferIn;
BufferRS232Packet bufferOut;


// first structure saved in EPROM contains pointers to get calibration params and vars to process
#if (RESET_EEPROM)
  CAL calidef[MAX_MAXQ_CALI_ALLOWED]= CAL_ADDRESS; // INITIAL calibration data   
  MAXQVars varsDefintion[MAX_MAXQ_VARS_ALLOWED] = VARS_DEF; // initial VARSDEF (addres, len, shift)
  STR_MNG structManager = {13,24}; // 13 cali params, 24 variables to acquire per sampling
  OVERFLOW_s OVERFLOW = {0}; // store energy overflow registers
#else 
  CAL calidef[MAX_MAXQ_CALI_ALLOWED];  
  MAXQVars varsDefintion[MAX_MAXQ_VARS_ALLOWED];
  STR_MNG structManager;
  OVERFLOW_s OVERFLOW;  
#endif


void setup() {
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  
  mySerial.begin(4800);
  #if (ENABLE_WATCHDOG)
    MCUSR = 0;
    wdt_disable();
  #endif
  
  Serial.begin(9600);
  #if (DEBUG)
    mySerial.println("SETUP...");
    Serial.println("SETUP...");
  #endif
  pinMode(SPISS,OUTPUT);
  pinMode(SPISS0,OUTPUT);
  pinMode(SPIOUT, OUTPUT);
  pinMode(SPIDIN, INPUT);
  pinMode(SPICLK,OUTPUT);
  
  digitalWrite(SPISS0,HIGH); // disable device at startup prevent noise 
  
  
  SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0); // SPI Settings, enable, master 
  byte clr;
  clr=SPSR; // clear SPI past operation
  clr=SPDR; // clear SPI past operation
  delay(10);
  randomSeed(analogRead(0));
    
  #if (RESET_EEPROM)
    backupData();
  #else
    loadStructuresFromEprom(); // extarct pointer to get calibration params and Vars definition (variables to acquire during each MAXQ sampling procedure)
    #if (ENABLE_CALIBRATE_ON_BOOT)
      calibrate(false); // send calibration params to MAXQ
    #endif
    loadVarsDefinition(); // load vars definition from EPROM  
    #if PRINT_TIME_AT_BOOT
      byte now[5];
      printQueuePointers();
      getDateTime(now);
      mySerial.print("\t(Now):");
      printArrayasHEX(now,0,5,true);
    #endif
  #endif
  
  //calibrate(); // send calibration params to MAXQ


  #if (ENABLE_WATCHDOG)
    wdt_enable(WDTO_8S);
  #endif

}

void loop() {

  #if (ENABLE_WATCHDOG)
    wdt_reset();
  #endif
  
  #if (!RESET_EEPROM)
    checkNewRS232Request();
    
    if (!calibration_enabled) {
      
      #if (ENABLE_DATESYNC) 
        keepDatetimeUpdated();
      #endif
      #if (ENABLE_SAMPLING)
        startSampling();
      #endif
      
      #if (ENABLE_CHECK_PENDANT_MAXQ_TO_SYNC)
        checkPendantMAXQtoSync();
      #endif
      
      #if (ENABLE_CUMMULATIVE_EEPROM_BACKUP) 
        syncCummulativeOverflowCounterEEPROM();
      #endif
      
      #if (ENABLE_CHECKOVERFLOW)
        checkOverflow();
      #endif
    }
  #endif

}









