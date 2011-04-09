#ifndef utils_h
#define utils_h
#include <WProgram.h>
#include <Wire.h>
#include <DS1307.h>
#include <EEPROM.h>


#define DEBUG true
#define ENABLE_DATESYNC true
#define ENABLE_CUMMULATIVE_EEPROM_BACKUP true
#define ENABLE_SAMPLING true
#define ENABLE_CHECK_PENDANT_MAXQ_TO_SYNC true
#define ENABLE_CHECKOVERFLOW true 
#define ENABLE_CALIBRATE_ON_BOOT true
#define RESET_EEPROM false
#define ENABLE_WATCHDOG false
#define CHECK_OVERFLOW_INTER_SAMPLING true
#define SHOW_SPI_DATA_TRANSFER false
#define PRINT_TIME_AT_BOOT true

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

#define INTERVAL_DATETIME_SYNC 86400000 // once a day
#define INTERVAL_CUMMULATIVE_EEPROM_SYNCED 86400000 // once a day
#define DEAD_TIME_LIMIT 12000 // 3 minutos (debug)
#define INTERVAL_OVERFLOW_CHECK 10000 // 30 seg

#define PARSER_RS232_PARAMS_MAX 68 // maximo numero de parametros que se pueden almacenar por RS232 request
#define MY_DEFAULT_SLOT 1


#define START_BYTE 0x7e
#define ESCAPE 0x7d
#define XON 0x11
#define XOFF 0x13

#define NO_ERROR 0
#define UNEXPECTED_START_BYTE 1
#define CHECKSUM_FAILURE 2
#define PACKET_EXCEEDS_BYTE_ARRAY_LENGTH 3

//opcodes
#define READ (0<<6) 
#define WRITE (2<<6)

#define BYTES1 (0<<4)
#define BYTES2 (1<<4)
#define BYTES4 (2<<4)
#define BYTES8 (3<<4)

#define SPISS 10//SLAVESELECT Maxq 4
#define SPISS0 7//SLAVESELECT Maxq 4
#define SPIOUT 11//MOSI
#define SPIDIN 12//MISO 
#define SPICLK 13//sck

#define MAXQ_ACK 0x41
#define MAXQ_NACK 0x4E
#define MAXQ_WRITE_RETRIES 2
#define MAXQ_READ_RETRIES 2
#define MAX_SPI_WAIT_LIMIT 100

// used to define ZONES in EPROM to store these variables
#define MAX_MAXQ_VARS_ALLOWED 30
#define MAX_MAXQ_CALI_ALLOWED 15

#define INTERVAL_MAXQ_SAMPLES 600000
#define INTERVAL_BETWEEN_SYNC 30000
// ERROR CODES
#define ERROR_CODE_NACK 69

// metodos
#define MAXQ_GET_VARIABLE_METHOD 98 // get a single param and send it back to gateway, (used for calibration porpuses)
#define MAXQ_WRITE_VARIABLE_METHOD 95 // write parameter to MAXQ
#define MAXQ_GET_NOW_SAMPLE_METHOD 99 // get all params available and send them to gateway
#define MAXQ_UPDATE_CALI_PARAMS_METHOD 100 // update calibration params and reload MAXQ
#define MAXQ_UPDATE_DATETIME_METHOD 97 // method to update datetime
#define MAXQ_CUMMULATIVE_OVERFLOW_METHOD 96 // method to send cummulative energy
#define MAXQ_END_STREAM 45
#define CALIBRATION_STATUS 230 // inicia calibracion y detiene todo menos la escucha
#define SEND_OVERFLOWS_COUNTER_METHOD 231 // envia overflow counter to requester
#define INIT_EEPROM_METHOD 232 // resetea eeprom

//
#define SEND_MAXQ_SAMPLE_METHOD 99 // method to 

#define DEFAULT_OPID_MAXQ 13 // opID por defeceto cuando se envia un paquete MAXQ al gateway
// cantidad de grupos de mediciones de MAXQ almacenadas en RAM (FIFO, buffer 
// circular, util en caso de perdida de comunicacion, en caso que la perdida 
// de comunicacion sea extendida, se guardan datos de energia y demanda (fundamentales) en la EPROM)
#define MAX_BUFFER_RAM_MEAS 1
#define MAX_BUFFER_EEPROM_MEAS 4

#define BASE_ADDRESS_STRUCT_MANAGER 0
#define BASE_ADDRESS_STRUCT_CALI 2
#define BASE_ADDRESS_STRUCT_VARSDEF 85
#define BASE_ADDRESS_STRUCT_QUEUE_MNG BASE_ADDRESS_STRUCT_VARSDEF + sizeof(MAXQVars) * MAX_MAXQ_VARS_ALLOWED
#define BASE_ADDRESS_STRUCT_QUEUE_EEPROM BASE_ADDRESS_STRUCT_QUEUE_MNG+sizeof(QueueDataToSync_MNG)
#define BASE_ADDRESS_STRUCT_CUMMULATIVE_ENERGY_OVERFLOW BASE_ADDRESS_STRUCT_QUEUE_EEPROM+sizeof(_ppMAXQ)*MAX_BUFFER_EEPROM_MEAS

// maximo numero de señales por muestreo de MAXQ que se estan almacenando
#define NUM_VALUES_TO_GET_FROM_MAXQ 24 // 
// direcciones en MAXQ de cada variable

#define VARS_DEF {0x08,0x31,8,  2,0x08,0x32,8,2,  0x08,0x34,8,2,  0x08,0x41,8,2,  0x08,0x42,8,2,  0x08,0x44,8,2, 0x08,0x01,8,2,  0x08,0x02,8,2,  0x08,0x04,8,2,  0x08,0x11,8,2,  0x08,0x12,8,2,  0x08,0x14,8,2,  0x08,0x21,8,2,  0x08,0x22,8,2,  0x08,0x24,8,2,  0x08,0xC1,8,3,  0x08,0xC2,8,3,  0x08,0xC4,8,3,  0x01,0xC6,2,0,  0x02,0xB2,2,0,  0x03,0x9E,2,0, 0x01,0x46,1,0,  0x02,0x32,1,0, 0x03,0x1E,1,0}
#define TAGS_MEAS {"VA","VB","VC","IA","IB","IC","PAA","PAB","PAC","PRA","PRB","PRC","PPA","PPB","PPC","ERA","ERB","ERC","PFA","PFB","PFC", "AOV","BOV","COV"}

// VX.GAIN // AX.GAIN // VOLT_CC // AMP_CC // ENR_CC // PWR_CC  // PHASE ANGLE COMPMENSANTION
//#define CAL_ADDRESS {0x01,0x32,0x3d,0x4d,2,0x02,0x1E,0x3d,0x4d,2,0x03,0x0A,0x3d,0x4d,2,0x01,0x30,0xF1,0x45,2,0x02,0x1C,0xF1,0x45,2,0x03,0x08,0xF1,0x45,2,0x00,0x14,0x27,0x00,2, 0x00,0x16,0x06,0x00,2, 0x00,0x1A,0x24,0x00,2, 0x00,0x18,0x18,0x00,2,0x01,0x3E,0x3D,0x4D,2,0x02,0x2A,0x3D,0x4D,2,0x03,0x16,0x3D,0x4D,2}
#define CAL_ADDRESS {0x00,0x16,0x0,0x6,2,0x00,0x14,0x00,0x15,2,0x00,0x18,0x0,0xd,2,0x00,0x1A,0x0,0x14,2,0x01,0x32,0x41,0x5a,2,0x02,0x1E,0x3f,0x22,2,0x03,0x0A,0x3f,0xbd,2,0x01,0x30,0x40,0x9a,2,0x02,0x1C,0x40,0xfa,2,0x03,0x08,0x41,0x3f,2,0x01,0x3E,0xff,0xc3,2,0x02,0x2A,0x0,0xa0,2,0x03,0x16,0x0,0x9d,2}
//AMP_CC  0x0016  0x6
//VOLT_CC 0x0014  0x15
//PWR_CC  0x0018  0xd
//ENR_CC  0x001A  0x14
//VA_GAIN 0x0132  0x415a
//VB_GAIN 0x021E  0x3f22
//VC_GAIN 0x030A  0x3fbd
//IA_GAIN 0x0130  0x409a
//IB_GAIN 0x021C  0x40fa
//IC_GAIN 0x0308  0x413f
//A_PA0   0x013E  0xffffffc3
//B_PA0   0x022A  0xa0
//C_PA0   0x0316  0x9d

/* STRUCTURES */
typedef struct STRUCT_MANAGER_struct {
  byte num_cal_params; // numero de parametros de calibracion 
  byte num_vars_to_sample; // numero de variables a adquirir en cada muestreo MAXQ
}STR_MNG;

typedef struct CAL_struct {
  byte address[2];  
  byte data[2];
  byte len; // 8 virtual
}CAL;

typedef struct MAXQVARS_struct {
  byte address[2]; // address to ask to MAXQ
  byte len;// 8 virtual length of the expected variable
  byte shift_bytes; // bytes to shift right (sometimes MAXQ send padding bytes that need to be removed), tipically 2 bytes from a virtual MAXQ register
}MAXQVars;

typedef struct BufferRS232Packet_struct {
  boolean isAvailable;
  byte ErrorCode;
  byte Method;
  byte params[PARSER_RS232_PARAMS_MAX];
  byte len_params;
  byte Id;
}BufferRS232Packet;

typedef struct bufferMeas_struct {
  byte datetimestamp[5]; // YY/MM/DD/HH/MM
  unsigned long signals[MAX_MAXQ_VARS_ALLOWED]; // señales (1 byte)
}bufferMeas;

typedef struct QueueDataToSync_MNG_struct {
  byte producer_RAM;
  byte consumer_RAM;
  byte waiting_RAM;
  boolean RAM_is_full;
  byte producer_EEPROM;
  byte consumer_EEPROM;
  byte waiting_EEPROM;
  boolean EEPROM_is_full;
}Queue_MNG;


typedef struct OVERFLOW_struct {
  long TX[3];
}OVERFLOW_s;
/* Funciones utils */

/* MACROS */

void keepDatetimeUpdated(); // ask once a day the time to server to keep watches synced

void checkNewRS232Request(); // atiende las peticios por I2C

void startSampling(); // start MAXQ sampling periodically

long getVarFromMAXQ(byte var_index); // get a single var_def[var_index] from maxq

void checkPendantMAXQtoSync(); // check if there are pendant MAXQ samples to sync with gateway (this data is comming from RAM and EPROM depending if the lack of connection has been expensive)

boolean processCalibrationParams(); // get all calibration params, store them in EPROM and reload MAXQ
void updatePreviousMAXQSync(); // refresh pointers when a MAXQ sample has been correctly synced in the server, allowing overriding
void processGetNowMAXQSample(); // get a complete sample of all variables in MAXQ and send them back to gateway
void processGetMAXQVar(); // get a unique variable from maxq and send it back to gateway, used for calibration porpuses

void processWriteMAXQ(); // escribe n bytes de data en address

void sendAck(); // envia un ack al master del canal I2C con el mismo metodo, id y un parametro nulo
void sendNAck(); // envia mismo metodo y opID pero atacha un parametro de error 69;

void sendMAXQDataFromMemory(byte index_meas, boolean from_ram, byte method, byte opID); // envia todas las variables adquiridas MAXQ de la sample index_meas hacia el gateway que este almacenada en la posicion index de la memoria

void setDateTime(); // ajusta la hora y fecha
void getDateTime(byte* data); // return *data with current date

void readPacket(int timeToWait); // lee un packete y lo parsea
void sendPacket(); // pack and send
void sendByte(byte b, boolean escape); // send byte

void backupData(); // start saving energy and power if connection has been lost with gateway

void calibrate(boolean forced); // load calibration params from EPROM and send them to MAXQ
void loadStructuresFromEprom(); // extract pointers manager from EPROM during a boot
void loadVarsDefinition(); // extract vars definition from EPROM


boolean writetoMAXQ(byte* address, byte* dataToSend, byte len); // write a set of bytes stored in data  to MAXQ
boolean readfromMAXQ(byte* address, byte* dataToRead, byte len, byte shift); // read a set of params into data from MAXQ
char spi_transfer(volatile char data);
/* REad and write anything from eprom */
template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
	  EEPROM.write(ee++, *p++);
    return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
	  *p++ = EEPROM.read(ee++);
    return i;
} 

void printArrayasHEX(byte* cadena,int from, int len, boolean br);

unsigned long humanReading(byte *data,byte len);

void printQueuePointers();

void syncCummulativeOverflowCounterEEPROM(); // Sync every day cummulative energy overflow counter in EEPROM

void sendCummulativeOverfkowCounter(); // process server request to send  "energy overflow cummulative counter" 

void checkOverflow(); // timer executes _checkOverflow() periodically

void _checkOverflow(); // execute overflow check

void writeEEPROM(); // write params directly into eeprom

#endif








