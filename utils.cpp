#include <stdlib.h>
#include <avr/pgmspace.h>
#include "utils.h"
#include <avr/io.h>
#include <WProgram.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <DS1307.h>
#if (ENABLE_WATCHDOG) 
  #include <avr/wdt.h> 
#endif

extern BufferRS232Packet bufferIn;
extern BufferRS232Packet bufferOut;
extern unsigned long time_from_last_sample;
extern unsigned long last_sync_check;
extern unsigned long last_time_datetime_synced;
extern unsigned long last_time_energy_overflow_check;
extern unsigned long last_time_cummulative_eeprom_synced;

extern bufferMeas ppMAXQ[MAX_BUFFER_RAM_MEAS]; // cantidad de dispositivos buffereados en mediciones
extern bufferMeas _ppMAXQ;
extern CAL calidef[MAX_MAXQ_CALI_ALLOWED]; // INITIAL calibrationa  data   
extern MAXQVars varsDefintion[MAX_MAXQ_VARS_ALLOWED]; // initial VARSDEF (addres, len, shift)
extern STR_MNG structManager;
extern Queue_MNG QueueDataToSync_MNG;

extern unsigned long last_time_datetime_synced; // if this variable is false trigger syndate packet every day
extern boolean sync_time_at_boot; // flag false when datetime has been adjusted at startup 
extern boolean sampling_on_boot;
extern boolean calibration_enabled;

extern SoftwareSerial mySerial;

extern OVERFLOW_s OVERFLOW;


void keepDatetimeUpdated() {
  if ((sync_time_at_boot & (millis() - last_time_datetime_synced > 10000)) | (millis() - last_time_datetime_synced > INTERVAL_DATETIME_SYNC)) {
    #if (DEBUG)
      mySerial.println("\t(DateSync)");
    #endif
    bufferOut.Method = MAXQ_UPDATE_DATETIME_METHOD;
    bufferOut.len_params = 1;
    bufferOut.params[0] = MY_DEFAULT_SLOT;
    bufferOut.Id = 66;
    sendPacket();
    last_time_datetime_synced = millis();
  }
}

void checkNewRS232Request() {
  readPacket(100); //Lee Uart y pone en la estructura el packete si este fue bien enviado
  if (bufferIn.isAvailable) {
    switch (bufferIn.Method) {
    case MAXQ_UPDATE_DATETIME_METHOD: // Nueva hora para actualizar reloj
      mySerial.println("\tUpdDate");
      setDateTime();
      break;
    case MAXQ_UPDATE_CALI_PARAMS_METHOD: // receive new calibration params and reload MAXQ
      mySerial.println("\tUpdCali");
      if (processCalibrationParams()) {
        sendAck();
      }
      else {
        sendNAck();
      }
      break;
    case MAXQ_WRITE_VARIABLE_METHOD:
      processWriteMAXQ();
      break;
    case MAXQ_GET_NOW_SAMPLE_METHOD: // this method is used to attend new MAXQ sample request or by confirm an acknowledge from a prevous sync
      if (bufferIn.params[1] == 1) { // ack from a previous sync, this is sent by internet-gateway
        updatePreviousMAXQSync();
      }
      else {
        processGetNowMAXQSample();
      }
      break;
    case CALIBRATION_STATUS:
      mySerial.println("\tCali");
      if (bufferIn.params[0] == 1) {
        calibration_enabled = true;
      }else {
        calibration_enabled = false;
      }
      sendAck();
      break;
    case SEND_OVERFLOWS_COUNTER_METHOD:
      mySerial.println("\tTXov");
      sendCummulativeOverfkowCounter();
      break;
    case INIT_EEPROM_METHOD:
      mySerial.println("\tInitEE");
      writeEEPROM();
      break;
    case MAXQ_GET_VARIABLE_METHOD: // get a single param and send it back to gateway, (used for calibration porpuses)
      processGetMAXQVar();
    default:
      break;
    }
  }
  // packet has been processed
  bufferIn.isAvailable = false;
}

void startSampling() {

  //MAXQVars vars_def[NUM_VALUES_TO_GET_FROM_MAXQ] = VARS_DEF;
  unsigned long data; // store long variable with data acquired with multiplied by 10
  
  if (time_from_last_sample > millis()) {
    time_from_last_sample = millis();
  }

  if (sampling_on_boot | (millis() - time_from_last_sample > INTERVAL_MAXQ_SAMPLES)) {
    #if (DEBUG)
      mySerial.println("\t(Samp)");
    #endif
	sampling_on_boot = false;
    for (byte r=0;r<structManager.num_vars_to_sample-3;r++) {
      checkNewRS232Request();
      if (calibration_enabled) {
        return;
      }
      data = 0; // clean var
      data = getVarFromMAXQ(r);
      // store data in RAM (ppMAXQ) in current producer_RAM
      _ppMAXQ.signals[r] = data;
      #if (CHECK_OVERFLOW_INTER_SAMPLING) 
        _checkOverflow(); // check overflow
      #endif
    }
    
    // attach num overflows since last measurememts
    for (byte f=0;f<3;f++) {
      _ppMAXQ.signals[structManager.num_vars_to_sample-(3-f)] = OVERFLOW.TX[f];      
    }
    // store sample procedure into queue (RAM if its full use EEPROM)
    
    if (QueueDataToSync_MNG.RAM_is_full) { // save _ppMAXQ in EEPROM queue
      #if (DEBUG)
        mySerial.println("\tRAMfulluseEEPROM");
      #endif
      
      if (QueueDataToSync_MNG.EEPROM_is_full) {
        #if (DEBUG)
          mySerial.println("\tDisscardMEMFull");
        #endif
        return;
      }
      
      EEPROM_writeAnything(BASE_ADDRESS_STRUCT_QUEUE_EEPROM + QueueDataToSync_MNG.producer_EEPROM * sizeof(_ppMAXQ), _ppMAXQ); 
      
      
      if (QueueDataToSync_MNG.producer_EEPROM == MAX_BUFFER_EEPROM_MEAS - 1) {
        QueueDataToSync_MNG.producer_EEPROM = 0;
      }
      else {
        QueueDataToSync_MNG.producer_EEPROM++;        
      }
    
      QueueDataToSync_MNG.waiting_EEPROM++;
      
      // check if it's full
      if (QueueDataToSync_MNG.waiting_EEPROM  == MAX_BUFFER_EEPROM_MEAS) {
        QueueDataToSync_MNG.EEPROM_is_full = true;
      }
      
      // update eeprom pointers
      EEPROM_writeAnything(BASE_ADDRESS_STRUCT_QUEUE_MNG,QueueDataToSync_MNG); 
      
      #if (DEBUG)    
        mySerial.print("\n\t->Ewaiting:");mySerial.println(QueueDataToSync_MNG.waiting_EEPROM,DEC);
        mySerial.print("\n\t->EP/C:");mySerial.print(QueueDataToSync_MNG.producer_EEPROM,DEC);mySerial.print("/");mySerial.println(QueueDataToSync_MNG.consumer_EEPROM,DEC);
      #endif
    }
    else { // save _ppMAXQ in RAM queue
      ppMAXQ[QueueDataToSync_MNG.producer_RAM] = _ppMAXQ;
      if (QueueDataToSync_MNG.producer_RAM == MAX_BUFFER_RAM_MEAS - 1) {
        QueueDataToSync_MNG.producer_RAM = 0;
      }
      else {
        QueueDataToSync_MNG.producer_RAM++;        
      }
      QueueDataToSync_MNG.waiting_RAM++;
      
      // check if it's full
      if (QueueDataToSync_MNG.waiting_RAM  == MAX_BUFFER_RAM_MEAS) {
        QueueDataToSync_MNG.RAM_is_full = true;
      }
      #if (DEBUG)
        mySerial.print("\n\t->Rwaiting:");mySerial.print(QueueDataToSync_MNG.waiting_RAM,DEC);
        mySerial.print("\n\t->RP/C:");mySerial.print(QueueDataToSync_MNG.producer_RAM,DEC);mySerial.print("/");mySerial.println(QueueDataToSync_MNG.consumer_RAM,DEC);
      #endif
    }
    
    // send data to gateway
    
    time_from_last_sample = millis();
  }   
  
  digitalWrite(SPISS0,HIGH); // enable MAXQ CHIP for operation
}

long getVarFromMAXQ(byte var_index){
  byte dataToRead[8] = {0};
  unsigned long HR;
  boolean success;
  //dataToRead[8] = 0;
  success =  readfromMAXQ(varsDefintion[var_index].address, dataToRead, varsDefintion[var_index].len, varsDefintion[var_index].shift_bytes);
  HR = humanReading(dataToRead,varsDefintion[var_index].len); // convert dataToRead to long readeable
  #if (DEBUG)
    mySerial.print(var_index,DEC);mySerial.print(" A:");mySerial.print(varsDefintion[var_index].address[0],HEX);mySerial.print("-");mySerial.print(varsDefintion[var_index].address[1],HEX);mySerial.print(": (");mySerial.print(HR,DEC);mySerial.print("L) ");printArrayasHEX(dataToRead,0,varsDefintion[var_index].len, false);mySerial.print("\tOK?:");mySerial.println(success,DEC);
  #endif
  return HR;
}

void checkPendantMAXQtoSync() {

  // check QueueManager (this object stores info about the index of pointers to producer and consumer of each queue
  //byte opID = random(255);
  byte opID = 55;
  if (last_sync_check > millis()) {
    last_sync_check = millis();
  }
  if (millis() - last_sync_check > INTERVAL_BETWEEN_SYNC) {
    if (QueueDataToSync_MNG.waiting_EEPROM>0) {
      #if (DEBUG)
        mySerial.println("\t(SyncEE)");
      #endif
      sendMAXQDataFromMemory(QueueDataToSync_MNG.consumer_EEPROM,false, MAXQ_GET_NOW_SAMPLE_METHOD,opID);
      last_sync_check = millis();
      return; // uno a la vez, if an ack has been received from gateway later, update pointers and clean this sample
    }
   
    if (QueueDataToSync_MNG.waiting_RAM>0) {
      #if (DEBUG)
        mySerial.println("\t(SyncRAM)");
      #endif
      sendMAXQDataFromMemory(QueueDataToSync_MNG.consumer_RAM,true, MAXQ_GET_NOW_SAMPLE_METHOD,opID);
      last_sync_check = millis();
      return; // uno a la vez, if an ack has been received from gateway later, update pointers and clean this sample
    }
    
    
  }

  

  // ask data to gateway (optional) to avoid using the EPROM all the time
  // backup BufferMediciones
  //for (int t=0;t<MAX_BUFFER_MEAS;t++) {
  //  EEPROM_readAnything(eprom_index, BufferMediciones[t]);    
  //  eprom_index +=sizeof(BufferMediciones);
  //}
}

boolean processCalibrationParams() {
  byte params_to_store;
  byte start_index = 0;
  // store calibration params in local structure then and backup structures in EPROM
  // check if cali params exceed max allowed
  if (start_index + (bufferIn.len_params-1)/5 > MAX_MAXQ_CALI_ALLOWED) {
    #if (DEBUG)
      mySerial.println("\tParExceed");
    #endif
    return false;
  }
  else {
    start_index = bufferIn.params[0];
    params_to_store = (bufferIn.len_params-1)/5;
    for (byte t=start_index;t<params_to_store+start_index;t++) {
      calidef[t].address[0] = bufferIn.params[t*5+1];
      calidef[t].address[1] = bufferIn.params[t*5+2];
      calidef[t].data[0] = bufferIn.params[t*5+3];
      calidef[t].data[1] = bufferIn.params[t*5+4];
      calidef[t].len = bufferIn.params[t*5+5];
      #if (DEBUG)
        mySerial.print("\tA: ");printArrayasHEX(calidef[t].address,0,2,false);
        mySerial.print("\tD: ");printArrayasHEX(calidef[t].data,0,2,false);
        mySerial.print("\tL: ");mySerial.println(calidef[t].len,DEC);
      #endif
    } // end calibration update
    
    backupData();
    calibrate(true); // load params into MAXQ from EPROM
    
    return true;
  }

}

void updatePreviousMAXQSync() {// refresh pointers when a MAXQ sample has been correctly synced in the server, allowing overriding
  
  if (QueueDataToSync_MNG.waiting_EEPROM > 0) { // a MAXQ sample located in EEPROM has been correctly synced
    #if (DEBUG)
      mySerial.println("\tSyncServer");
    #endif
    if (QueueDataToSync_MNG.consumer_EEPROM == MAX_BUFFER_EEPROM_MEAS - 1) {
      QueueDataToSync_MNG.consumer_EEPROM = 0;
    }
    else {
      QueueDataToSync_MNG.consumer_EEPROM++;        
    }
    QueueDataToSync_MNG.waiting_EEPROM--;
    
    // refresh full flag
    if (QueueDataToSync_MNG.waiting_EEPROM < MAX_BUFFER_EEPROM_MEAS) {
      QueueDataToSync_MNG.EEPROM_is_full = false;
    }
    //printQueuePointers();

    // update EEPROM pointers
    EEPROM_writeAnything(BASE_ADDRESS_STRUCT_QUEUE_MNG,QueueDataToSync_MNG);
    return;
  }
  
  if (QueueDataToSync_MNG.waiting_RAM > 0) { // a MAXQ sample located in RAM has been correctly synced
    #if (DEBUG)
      mySerial.println("\tSYNCPakRX");
    #endif
    if (QueueDataToSync_MNG.consumer_RAM == MAX_BUFFER_RAM_MEAS - 1) {
      QueueDataToSync_MNG.consumer_RAM = 0;
    }
    else {
      QueueDataToSync_MNG.consumer_RAM++;        
    }
    QueueDataToSync_MNG.waiting_RAM--;
    
    // refresh full flag
    if (QueueDataToSync_MNG.waiting_RAM < MAX_BUFFER_RAM_MEAS) {
      QueueDataToSync_MNG.RAM_is_full = false;
    }
    //printQueuePointers();
    return;
  }

}
void processGetNowMAXQSample() {
  // extract measurent directly from MAXQ and build bufferOut.params
  #if (DEBUG)
    mySerial.println("\tGetMAXQSample");
  #endif
  // send data to user with same opID and method
  
}

void processGetMAXQVar() {
  #if (DEBUG)
    mySerial.println("\tGetMAXQVar");
  #endif
  byte address[2] = {bufferIn.params[0],bufferIn.params[1]};
  byte dataToRead[8] = {0};
  unsigned long HR;
  readfromMAXQ(address, dataToRead, bufferIn.params[2], bufferIn.params[3]);

 
  HR = humanReading(dataToRead,bufferIn.params[2]); // convert dataToRead to long readeable
  
  #if (DEBUG)
    mySerial.print(" A:");mySerial.print(address[0],HEX);mySerial.print("-");mySerial.print(address[1],HEX);mySerial.print(": (");mySerial.print(HR,DEC);mySerial.print("L) ");printArrayasHEX(dataToRead,0,bufferIn.params[2],true);
  #endif
  
  bufferOut.params[0] = address[0];
  bufferOut.params[1] = address[1];
  
  for (byte f=0;f<bufferIn.params[2];f++) {
    bufferOut.params[f+2] = dataToRead[f];
    mySerial.print(bufferOut.params[f+2],HEX);mySerial.print("-");
  }
  
  bufferOut.Method = bufferIn.Method;
  bufferOut.Id = bufferIn.Id;
  bufferOut.len_params = 2+bufferIn.params[2];
  sendPacket();
}

void processWriteMAXQ() {
  byte dataToRead[8] = {0};
  #if (DEBUG)
    mySerial.println("\tWriteMAXQVar");
  #endif
  byte address[2] = {bufferIn.params[0],bufferIn.params[1]};
  byte data[4] = {0};
  mySerial.print("WrMAXQ:");
  for (byte r=0;r<bufferIn.len_params-4;r++) {
    data[r] = bufferIn.params[bufferIn.len_params-1-r];
    mySerial.print(data[r],HEX);mySerial.print("-");
  }
  #if (DEBUG)
    mySerial.print("\t\tA:");mySerial.print(address[0],HEX);mySerial.print("-");mySerial.print(address[1],HEX);
    mySerial.print("\t\tD:");printArrayasHEX(data,0,bufferIn.len_params-4,false);
    mySerial.print("\t\tOK?:");mySerial.println((int)writetoMAXQ(address,data,bufferIn.len_params-4),DEC);
    readfromMAXQ(address, dataToRead, bufferIn.params[2], bufferIn.params[3]);
    for (byte g=0;g<bufferIn.params[2];g++) {
      //mySerial.print("dataToRead[g]:");mySerial.print(dataToRead[g],HEX);mySerial.print("\tbufferIn.params[4+g]:");mySerial.println(bufferIn.params[4+g],HEX);
      if (dataToRead[g] != bufferIn.params[4+g]) {
        sendNAck();
        //return;
      }
    }
    sendAck();
  #endif

}

void sendAck(){
  #if (DEBUG)
    mySerial.println("\tTXAck");
  #endif
  bufferOut.Method = bufferIn.Method;
  bufferOut.len_params = 1;
  bufferOut.params[0] = 0;
  bufferOut.Id = bufferIn.Id;
  sendPacket();
}

void sendNAck(){
  #if (DEBUG)
    mySerial.println("\tTXNAck");
  #endif
  
  bufferOut.Method = bufferIn.Method;
  bufferOut.len_params = 1;
  bufferOut.params[0] = ERROR_CODE_NACK;
  bufferOut.Id = bufferIn.Id;
  sendPacket();
}

void sendMAXQDataFromMemory(byte index_meas,boolean from_ram, byte method,byte opID) {
  // send data from global structure with the last MAXQ aquisition
  bufferOut.Method = method;
  bufferOut.len_params = structManager.num_vars_to_sample * 4; // send a long per each VARS

  if (from_ram) { // copy element
    _ppMAXQ = ppMAXQ[index_meas];
  }
  else { // copy element from eprom
    EEPROM_readAnything(BASE_ADDRESS_STRUCT_QUEUE_EEPROM + index_meas*sizeof(_ppMAXQ), _ppMAXQ); 
  }

  // attach datetime
  
  // attach vars and send multiple packets if (len datetime + len signals * 4) > payload
  byte total_bytes = (6 + structManager.num_vars_to_sample * 4);
  byte num_packets = total_bytes / (PARSER_RS232_PARAMS_MAX-2);
  
  if (total_bytes % num_packets > 0) {
    num_packets++;
  }
  
  byte params_index;
  byte vars_index = 0;
  byte prev_index;
  byte y;
  //mySerial.print("structManager.num_vars_to_sample");mySerial.print(structManager.num_vars_to_sample,DEC);
  for (byte r=1;r<=10;r++) {
    params_index = 0;
    #if (DEBUG)
      mySerial.print("\tTX_P");mySerial.println(r,DEC);
    #endif

    bufferOut.params[params_index] = r; // paquete numero r
    params_index++;
    params_index++;
    
    if (r == 1) { // attach datetime
      byte now[5];
      getDateTime(now);
      bufferOut.params[params_index-1] = MY_DEFAULT_SLOT;
      for (byte g=0;g<5;g++) {
        bufferOut.params[params_index] = now[g];
        params_index++;
      }
    }

    for (y=vars_index;y<structManager.num_vars_to_sample;y++) {
      for (byte f=0;f<4;f++) { // only save a long data variable
        bufferOut.params[params_index] = (byte)(_ppMAXQ.signals[y]>>(8*(3-f)));
        params_index++;
      }
      
      bufferOut.len_params = params_index;
      if (params_index + 4 > PARSER_RS232_PARAMS_MAX) {
        params_index = 0;
        prev_index = vars_index;
        vars_index = y+1;
        break;
      }
    }
   
    bufferOut.Id = opID;
    
    
    
    if (y == structManager.num_vars_to_sample) {
      bufferOut.params[1] = MAXQ_END_STREAM; 
    }
    else {
      bufferOut.params[1] = 0;
    }
    
    #if (DEBUG)
      //mySerial.print(prev_index,DEC);mySerial.print("-");mySerial.println(vars_index-1,DEC);
      mySerial.print("\t");mySerial.print(bufferOut.Method,DEC);mySerial.print(", ");printArrayasHEX(bufferOut.params,0,bufferOut.len_params, false);mySerial.print(", ");mySerial.println(bufferOut.Id,DEC);
    #endif
    sendPacket();

    if (y == structManager.num_vars_to_sample) {
      break;
    }
  }
  

}

void setDateTime(){
  // Year, Month, Day, DayOfWeek, Hour, Minute
  RTC.stop();
  RTC.set(DS1307_YR, (int)bufferIn.params[0]);         //set the year
  RTC.set(DS1307_MTH, (int)bufferIn.params[1]);        //set the month
  RTC.set(DS1307_DATE, (int)bufferIn.params[2]);       //set the date
  RTC.set(DS1307_HR, (int)bufferIn.params[3]);       //set the hours
  RTC.set(DS1307_MIN, (int)bufferIn.params[4]);     //set the minutes
  RTC.set(DS1307_SEC, 1);        //set the seconds
  RTC.start();
  sync_time_at_boot = false;
  byte now[5];
  getDateTime(now);
  mySerial.print("\t(Now):");
  for (byte r=0;r<5;r++) {
    mySerial.print(now[r],DEC);mySerial.print(" ");
  }
  mySerial.println();
}

void getDateTime(byte* data) {
  data[0] = (byte)(RTC.get(DS1307_YR,true)-2000);
  data[1] = (byte)RTC.get(DS1307_MTH,true);
  data[2] = (byte)RTC.get(DS1307_DATE,true);
  data[3] = (byte)RTC.get(DS1307_HR,true);
  data[4] = (byte)RTC.get(DS1307_MIN,true);
}



////////////////////////////////////////////////////////////////
/////////////////// Comunicacion RS232 /////////////////////////
////////////////////////////////////////////////////////////////


// START_BYTE|Lenght|Metodo and data|
void readPacket(int timeToWait) {
  byte _checksumTotal = 0;
  byte _pos = 0;
  byte b = 0;
  boolean _escape = false; 
  // reset previous response
  if (bufferIn.isAvailable || (bufferIn.ErrorCode > 1)) {
    // Si nadie leyo el mensaje en un loop el mensaje es desechado
    bufferIn.isAvailable = false;
    bufferIn.ErrorCode = NO_ERROR;
  }
  if(Serial.available())delay(timeToWait);//Espera a que llegue todo el mensaje
  while (Serial.available()) {
    b = Serial.read();
    if (_pos == 0 && b != START_BYTE) {
      bufferIn.ErrorCode = UNEXPECTED_START_BYTE;
      return;
    }
    if (_pos > 0 && b == START_BYTE) {
      // new packet start before previous packeted completed -- discard previous packet and start over
      bufferIn.ErrorCode = UNEXPECTED_START_BYTE;
      _pos = 0;
      _checksumTotal = 0;
    }
    if (_pos > 0 && b == ESCAPE) {
      if (Serial.available()) {
        b = Serial.read();
        b = 0x20 ^ b;
      } 
      else {
        // escape byte.  next byte will be
        _escape = true;
        continue;
      }
    }
    if (_escape == true) {
      b = 0x20 ^ b;
      _escape = false;
    }
    // checksum includes all bytes after len
    if (_pos >= 2) {
      _checksumTotal+= b;
    }
    switch(_pos) {
    case 0:
      if (b == START_BYTE) {
        _pos++;
      }
      break;
    case 1:
      // length msb
      bufferIn.len_params = b;
      _pos++;
      break;
    case 2:
      bufferIn.Method = b;
      _pos++;
      break;
    default:
      // check if we're at the end of the packet
      // packet length does not include start, length, method or checksum bytes, so add 3
      if (_pos == (bufferIn.len_params + 4)) {
        // verify checksum
        if ((_checksumTotal & 0xff) == 0xff) {
          bufferIn.isAvailable = true;
          bufferIn.ErrorCode = NO_ERROR;
          bufferIn.Id = bufferIn.params[bufferIn.len_params];
        } 
        else {
          // checksum failed
          bufferIn.ErrorCode = CHECKSUM_FAILURE;
        }
        // reset state vars
        _pos = 0;
        _checksumTotal = 0;
        return;
      } 
      else {
        // add to params array, Sin start byte, lenght, method ni checksum
        bufferIn.params[_pos - 3] = b;
        _pos++;
      }
    }
  }
}

void sendPacket() {
  byte checksum = 0;
  sendByte(START_BYTE, false);
  // send length
  sendByte(bufferOut.len_params, true);
  // Metodo
  sendByte(bufferOut.Method, true);
  // compute checksum
  checksum+= bufferOut.Method;
  for (int i = 0; i < bufferOut.len_params; i++) {
    sendByte(bufferOut.params[i], true);
    checksum+= bufferOut.params[i];
  }
  sendByte(bufferOut.Id,true); // always attach an Id (could be not useful but the gateway can handle several connections and an ID is neceesary for this porpuse
  checksum+= bufferOut.Id;

  checksum = 0xff - checksum;// perform 2s complement
  sendByte(checksum, true);// send checksum
  // send packet
}

void sendByte(byte b, boolean escape) {
  if (escape && (b == START_BYTE || b == ESCAPE || b == XON || b == XOFF)) {
    Serial.print(ESCAPE, BYTE);
    Serial.print(b ^ 0x20, BYTE);
  } 
  else {
    Serial.print(b, BYTE);
  }
}


void backupData() {
  
  #if (DEBUG)
    mySerial.println("\tBCK");
  #endif

  int cali_index = BASE_ADDRESS_STRUCT_CALI;
  int varsdef_index = BASE_ADDRESS_STRUCT_VARSDEF;

  //structManager.num_vars_to_sample = 21;
  #if (DEBUG)
    mySerial.print("StrMNG:");mySerial.print(BASE_ADDRESS_STRUCT_MANAGER,DEC);
  #endif
  EEPROM_writeAnything(BASE_ADDRESS_STRUCT_MANAGER, structManager);    
  #if (DEBUG)
  mySerial.print("-");mySerial.print(sizeof(structManager) + BASE_ADDRESS_STRUCT_MANAGER,DEC);mySerial.println("b");
  #endif
  #if (DEBUG) 
    mySerial.print("cali:");mySerial.print(BASE_ADDRESS_STRUCT_CALI,DEC);
  #endif
  for (int m=0;m<structManager.num_cal_params;m++) {
    EEPROM_writeAnything(cali_index, calidef[m]);    
    cali_index+=sizeof(calidef[m]);
  }
  #if (DEBUG)
    mySerial.print("-");mySerial.print(cali_index,DEC);mySerial.println("b");
  #endif

  // save varsdef
  #if (DEBUG) 
    mySerial.print("vars:");mySerial.print(BASE_ADDRESS_STRUCT_VARSDEF,DEC);
  #endif
  for (int r=0;r<structManager.num_vars_to_sample;r++) {
    EEPROM_writeAnything(varsdef_index, varsDefintion[r]); 
    varsdef_index+=sizeof(varsDefintion[r]);
  }
  #if (DEBUG) 
    mySerial.print("-");mySerial.print(varsdef_index,DEC);mySerial.println("b");
  #endif

  // save QUEUE manager
  #if (DEBUG)
    mySerial.print("QueuMan:");mySerial.print(BASE_ADDRESS_STRUCT_QUEUE_MNG,DEC);
  #endif
  EEPROM_writeAnything(BASE_ADDRESS_STRUCT_QUEUE_MNG,QueueDataToSync_MNG); 
  #if (DEBUG)
    mySerial.print("-");mySerial.print(sizeof(QueueDataToSync_MNG) + BASE_ADDRESS_STRUCT_QUEUE_MNG,DEC);mySerial.println("b");
  #endif
  

  
  // save OVERFLOW Struc
  #if (DEBUG)
    mySerial.print("OVERFLOW:");mySerial.print(BASE_ADDRESS_STRUCT_CUMMULATIVE_ENERGY_OVERFLOW,DEC);
  #endif
  EEPROM_writeAnything(BASE_ADDRESS_STRUCT_CUMMULATIVE_ENERGY_OVERFLOW,OVERFLOW); 
  #if (DEBUG)
    mySerial.print("-");mySerial.print(sizeof(OVERFLOW) + BASE_ADDRESS_STRUCT_CUMMULATIVE_ENERGY_OVERFLOW,DEC);mySerial.println("b");
  #endif
  
  
}

void calibrate(boolean forced) { // load calibration params from EPROM and send them to MAXQ
  byte index_cal = BASE_ADDRESS_STRUCT_CALI;
  byte data[2];
  #if (DEBUG)
    mySerial.println("\tCal..");
  #endif
  digitalWrite(SPISS0,LOW); // enable MAXQ CHIP for operation
  
  for (int r=0;r<structManager.num_cal_params;r++) {
    EEPROM_readAnything(index_cal, calidef[r]);
    index_cal+=sizeof(calidef[r]);
    if (calibration_enabled & !forced) {
      break;
    }
    data[0] = calidef[r].data[1];
    data[1] = calidef[r].data[0];
    #if (DEBUG)
      mySerial.print("\t\tA:");mySerial.print(calidef[r].address[0],HEX);mySerial.print("-");mySerial.print(calidef[r].address[1],HEX);
      mySerial.print("\t\tD:");printArrayasHEX(calidef[r].data,0,2,false);
      mySerial.print("\t\tOK?:");mySerial.println((int)writetoMAXQ(calidef[r].address,data,2),DEC);
    #endif
    delay(200);
  }
  
  digitalWrite(SPISS0,HIGH); // enable MAXQ CHIP for operation
}

void loadStructuresFromEprom() { // extract pointers manager from EPROM during a boot
  EEPROM_readAnything(BASE_ADDRESS_STRUCT_MANAGER, structManager); 
  #if (DEBUG)
    mySerial.print("StruMNG(");mySerial.print(BASE_ADDRESS_STRUCT_MANAGER,DEC);mySerial.println(")");
    mySerial.print("\tCalpar:");mySerial.println(structManager.num_cal_params,DEC);
    mySerial.print("\tVardef:");mySerial.println(structManager.num_vars_to_sample,DEC);
  #endif
  
  mySerial.print("QueueMNG(");mySerial.print(BASE_ADDRESS_STRUCT_QUEUE_MNG,DEC);mySerial.println(")");
  EEPROM_readAnything(BASE_ADDRESS_STRUCT_QUEUE_MNG, QueueDataToSync_MNG);
 // clean RAM pointer, start from the start
  QueueDataToSync_MNG.producer_RAM = 0;
  QueueDataToSync_MNG.consumer_RAM = 0;
  QueueDataToSync_MNG.waiting_RAM = 0;
  QueueDataToSync_MNG.RAM_is_full = false;

  
  // read cummulative overflow energy counters

  mySerial.print("CUMMOV(");mySerial.print(BASE_ADDRESS_STRUCT_CUMMULATIVE_ENERGY_OVERFLOW,DEC);mySerial.println(")");
  EEPROM_readAnything(BASE_ADDRESS_STRUCT_CUMMULATIVE_ENERGY_OVERFLOW,OVERFLOW);
  
}

void loadVarsDefinition(){ // extract vars definition from EPROM
  byte index_varsdef = BASE_ADDRESS_STRUCT_VARSDEF;
  #if (DEBUG)
    mySerial.println("\tLVarDef");
  #endif
  for (int r=0;r<structManager.num_vars_to_sample;r++) {
    EEPROM_readAnything(index_varsdef, varsDefintion[r]);
    index_varsdef+=sizeof(varsDefintion[r]);
    #if (DEBUG)
      mySerial.print("\t\tA: ");mySerial.print(varsDefintion[r].address[0],HEX);mySerial.print("-");mySerial.print(varsDefintion[r].address[1],HEX);mySerial.print("\tsh:");mySerial.println(varsDefintion[r].shift_bytes,DEC);
    #endif
      
    delay(100);
  }
}

boolean readfromMAXQ(byte* address, byte* dataToRead, byte len, byte shift){
  byte DATA_TX_RX;
  unsigned long last_maxq_access = millis();
  unsigned long tempTime;
  long dataT = 0;
  byte temp = 0;
  for (byte r=0;r<MAXQ_WRITE_RETRIES;r++) {
    if (last_maxq_access > millis()) {
      last_maxq_access = millis();
    }
    while((millis()-last_maxq_access)<500);
    if (len==8) {
      DATA_TX_RX = READ + 0x30 + address[0];
    }
    else {
      DATA_TX_RX = READ + (0x30 & (len<<3)) + address[0];
    }
    digitalWrite(SPISS0,LOW); 
  
    DATA_TX_RX=spi_transfer(DATA_TX_RX);
    if (DATA_TX_RX==0xC1){
      delayMicroseconds(500); 
      DATA_TX_RX=spi_transfer(address[1]);
      if (DATA_TX_RX==0xC2){
        DATA_TX_RX = MAXQ_NACK;
        tempTime = millis();
        while (DATA_TX_RX!=MAXQ_ACK) { //0x4E Nack No ready 0x41 Ack Ready
          delayMicroseconds(500); 
          DATA_TX_RX = spi_transfer(0x00);
          if ((millis()-tempTime)>2000)break; //Error de comunicación SPI con Maxq
        }
        #if SHOW_SPI_DATA_TRANSFER
          mySerial.print(shift,DEC);mySerial.print("/");mySerial.println(len,DEC);
        #endif
        if (DATA_TX_RX==MAXQ_ACK){
          for (int i=0;i<len;i++) {
            delay(1);
            temp = spi_transfer(0x00); //get data byte
            #if SHOW_SPI_DATA_TRANSFER
              mySerial.print(temp,HEX);mySerial.print("-");
            #endif
            if (i>shift-1) {
              if (len-1-i+shift>7) {
                mySerial.println("Error");break;
              }
              dataToRead[len-1-i+shift] = temp;
            }
            temp = 0;
          }
          delay(1);
          DATA_TX_RX = spi_transfer(0x00); //checksum optional
          digitalWrite(SPISS0,HIGH); //release chip, signal end transfer
          last_maxq_access=millis();
          return true;
        }
      }
    }
    digitalWrite(SPISS0,HIGH); //release chip, signal end transfer
    last_maxq_access=millis();
  }
  return false;
}

boolean writetoMAXQ(byte* address, byte* dataToSend, byte len){
  byte DATA_TX_RX;
  unsigned long last_maxq_access = millis();
  unsigned long tempTime;
  for (byte r=0;r<MAXQ_WRITE_RETRIES;r++) {
    if (last_maxq_access > millis()) {
      last_maxq_access = millis();
    }
    while((millis()-last_maxq_access)<500);

    if (len==8) {
      DATA_TX_RX = WRITE + 0x30 + (address[0]);
    }
    else {
      DATA_TX_RX = WRITE + (0x30 & (len<<3)) + (address[0]);
    }
    digitalWrite(SPISS0,LOW);
    DATA_TX_RX=spi_transfer(DATA_TX_RX);
    if (DATA_TX_RX==0xC1){
      delayMicroseconds(500); 
      DATA_TX_RX=spi_transfer(address[1]);
      if (DATA_TX_RX==0xC2){
        for (byte i=0;i<len;i++) {
          delayMicroseconds(500);
          DATA_TX_RX = spi_transfer(dataToSend[i]); 
          if (DATA_TX_RX != MAXQ_ACK) break;
        }  
        if (DATA_TX_RX == MAXQ_ACK){ // all send success 
          DATA_TX_RX = MAXQ_NACK;
          tempTime = millis();
          while (DATA_TX_RX!=MAXQ_ACK) { //0x4E Nack No ready 0x41 Ack Ready
            delayMicroseconds(500); 
            DATA_TX_RX = spi_transfer(0x00);
            if ((millis()-tempTime)>2000)break; //Error de comunicación SPI con Maxq
          }
          if (DATA_TX_RX==MAXQ_ACK){ // transfer SUCCESS RETURN
            digitalWrite(SPISS0,HIGH); //release chip, signal end transfer
            last_maxq_access=millis();
            return true;
          }
        }
      }
    }
    digitalWrite(SPISS0,HIGH); //release chip, signal end transfer beacuse of error
    last_maxq_access=millis();
  }
  return false;
}




char spi_transfer(volatile char data) {// 1 byte 
  SPDR = data;                    // Start the transmission
  unsigned long spi_time_wait_limit = millis();
  while (!(SPSR & (1<<SPIF)))     // Wait the end of the transmission
  {
    #if (ENABLE_WATCHDOG) 
      wdt_reset();
    #endif
    if (millis() - spi_time_wait_limit > MAX_SPI_WAIT_LIMIT) {
      break;
    }
  };
  return SPDR;                    // return the received byte
}

void printArrayasHEX(byte* cadena,int from, int len, boolean br) {
  for (int j=from;j<len-1;j++) {
    #if (DEBUG)
      mySerial.print(cadena[j],HEX);mySerial.print(", ");
    #endif
  }
  #if (DEBUG)
    mySerial.print(cadena[len-1],HEX);
  #endif
  if (br) {
    #if (DEBUG)
      mySerial.println("");
    #endif
  }
}

unsigned long humanReading(byte *data, byte len) { // convert a 8byte array to a signed long
  unsigned long data_out = 0;
  if (len==8) {
      for (int g=4;g<len;g++) {
        data_out =  data_out | data[g]<<((len-g-1)*8);
      }
      return data_out;  
  }
  if (len==2) {
    for (int g=0;g<len;g++) {
      data_out =  data_out | data[g]<<((len-g-1)*8);
    }
    return data_out;  
  }
  if (len==1) {
    return data[0];  
  }
  if (len==4) {
    return (data[0]<<24) + (data[1]<<16) + (data[2]<<8) + (data[3]);
  }
}

void printQueuePointers() {
  #if (DEBUG)
    mySerial.print("\tPRAM:");mySerial.println(QueueDataToSync_MNG.producer_RAM,DEC);
    mySerial.print("\tCRAM:");mySerial.println(QueueDataToSync_MNG.consumer_RAM,DEC);
    mySerial.print("\tWRAM:");mySerial.println(QueueDataToSync_MNG.waiting_RAM,DEC);
    mySerial.print("\tRAM full:");mySerial.println((int)QueueDataToSync_MNG.RAM_is_full,DEC);
  
    mySerial.print("\tP EEPROM:");mySerial.println(QueueDataToSync_MNG.producer_EEPROM,DEC);
    mySerial.print("\tC EEPROM:");mySerial.println(QueueDataToSync_MNG.consumer_EEPROM,DEC);
    mySerial.print("\tW EEPROM:");mySerial.println(QueueDataToSync_MNG.waiting_EEPROM,DEC);
    mySerial.print("\tEEPROM full:");mySerial.println((int)QueueDataToSync_MNG.EEPROM_is_full,DEC);
  #endif
}

void checkOverflow() {
  if (last_time_energy_overflow_check > millis()) {
    last_time_energy_overflow_check = millis();
  }

  // check overflow register fast an stores pos or neg changes in a RAM variable ((int)OVERFLOW.A (int)OVERFLOW.B (int)OVERFLOW.C)
  if (millis() - last_time_energy_overflow_check > INTERVAL_OVERFLOW_CHECK) { // start sync into EEPROM
    #if (DEBUG)
      mySerial.println("\t(ChovTimer)");
      // save energy measurement in Econsumer
    #endif
    _checkOverflow();
    last_time_energy_overflow_check = millis();
  }
  
  
}


void syncCummulativeOverflowCounterEEPROM() { // Sync every day cummulative energy overflow counter in EEPROM
  if (last_time_cummulative_eeprom_synced > millis()) {
    last_time_cummulative_eeprom_synced = millis();
  }
  if (millis() - last_time_cummulative_eeprom_synced > INTERVAL_CUMMULATIVE_EEPROM_SYNCED) {
    #if (DEBUG)
      mySerial.print("SCumEE:B");mySerial.print(BASE_ADDRESS_STRUCT_CUMMULATIVE_ENERGY_OVERFLOW,DEC);mySerial.print("-");mySerial.println(sizeof(OVERFLOW),DEC);
    #endif
    EEPROM_writeAnything(BASE_ADDRESS_STRUCT_CUMMULATIVE_ENERGY_OVERFLOW, OVERFLOW);        
    last_time_cummulative_eeprom_synced = millis();
  }
}


void sendCummulativeOverfkowCounter() { // process server request to send  "energy overflow cummulative counter" 
  bufferOut.Method = MAXQ_CUMMULATIVE_OVERFLOW_METHOD;
  bufferOut.len_params = 4*3;
  for (byte e=0;e<3;e++) {
    for (byte j=0;j<4;j++) {
      bufferOut.params[e*4+j] = (byte)(OVERFLOW.TX[e]>>(8*(3-j)));    
    }
  }
  bufferOut.Id = 68;
  sendPacket();
}

void _checkOverflow(){ // ask to MAXQ overflow registers and refresh OVERFLOW.dX and OVERFLOW.TX structures 
  #if (DEBUG)
    mySerial.println("\t(ChOv)");
  #endif
  unsigned long data;
  byte mask[1] = {0};
  // update dX
  for (byte f=0;f<3;f++) {
    data = getVarFromMAXQ(structManager.num_vars_to_sample-3+f);
    if (CHECK_BIT(data,0)) {
      OVERFLOW.TX[f]++;
      EEPROM_writeAnything(BASE_ADDRESS_STRUCT_CUMMULATIVE_ENERGY_OVERFLOW, OVERFLOW);
      mask[0] = data & 0xFFFFFFFE;
      writetoMAXQ(varsDefintion[structManager.num_vars_to_sample-3+f].address, mask, 1);
    } else {
      if (CHECK_BIT(data,1)) {
        OVERFLOW.TX[f]--;
        EEPROM_writeAnything(BASE_ADDRESS_STRUCT_CUMMULATIVE_ENERGY_OVERFLOW, OVERFLOW);      
        mask[0] = data & 0xFFFFFFFD;
        writetoMAXQ(varsDefintion[structManager.num_vars_to_sample-3+f].address, mask, 1);
      }
    }
    mySerial.print("\tf:");mySerial.print(f,DEC);mySerial.print("\t#over:");mySerial.println(OVERFLOW.TX[f],DEC);
    // clear flag
    
  }
}

void writeEEPROM() {
  byte mask=0;
  byte index;
  switch (bufferIn.params[0]) {
    case 0: // clean overflow cumm
      mySerial.println("Clr:Over");
      for (byte r=0;r<12;r++) {
        EEPROM_writeAnything(BASE_ADDRESS_STRUCT_CUMMULATIVE_ENERGY_OVERFLOW + r,mask);
        OVERFLOW.TX[r] = 0;
      }
      break;
    case 1: // clean measurements pointers
      
      QueueDataToSync_MNG.producer_RAM = 0;
      QueueDataToSync_MNG.consumer_RAM = 0;
      QueueDataToSync_MNG.waiting_RAM = 0;
      QueueDataToSync_MNG.EEPROM_is_full = false;
      
      QueueDataToSync_MNG.producer_EEPROM = 0;
      QueueDataToSync_MNG.consumer_EEPROM = 0;
      QueueDataToSync_MNG.waiting_EEPROM = 0;
      QueueDataToSync_MNG.EEPROM_is_full = false;
      
      EEPROM_writeAnything(BASE_ADDRESS_STRUCT_QUEUE_MNG,QueueDataToSync_MNG);  
      break;
  }
  sendAck();
  // soft reset
}





