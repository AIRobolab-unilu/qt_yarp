
#ifndef Herkulex_h
#define Herkulex_h

#include <string>
#include <yarp/dev/PolyDriver.h>
//#include <yarp/dev/SerialInterfaces.h>
#include<serial.h>

#ifndef byte
    typedef unsigned char byte;
#endif

#define DATA_SIZE	 30		// buffer for input data
#define DATA_MOVE  	 100    // max 20 servos (5 x 20)

// SERVO HERKULEX COMMAND - See Manual p40
#define HEEPWRITE    0x01 	//Rom write
#define HEEPREAD     0x02 	//Rom read
#define HRAMWRITE	 0x03 	//Ram write
#define HRAMREAD	 0x04 	//Ram read
#define HIJOG		 0x05 	//Write n servo with different timing
#define HSJOG		 0x06 	//Write n servo with same time
#define HSTAT	 	 0x07 	//Read error
#define HROLLBACK	 0x08 	//Back to factory value
#define HREBOOT	 	 0x09 	//Reboot

#define ACK_NO_REPLY        0
#define ACK_REPLY_ON_REQ    1
#define ACK_ALWAYS_REPLY    2

#define MODEL_DRS0101       1
#define MODEL_DRS0201       2

// HERKULEX LED - See Manual p29
static int LED_OFF =	 0x00;
static int LED_GREEN =	 0x01;
static int LED_BLUE  =   0x02;
static int LED_CYAN  =   0x03;
static int LED_RED   = 	 0x04;
static int LED_GREEN2= 	 0x05;
static int LED_PINK  =   0x06;
static int LED_WHITE =   0x07;

// HERKULEX STATUS ERROR - See Manual p39
static byte H_STATUS_OK					= 0x00;
static byte H_ERROR_INPUT_VOLTAGE 		= 0x01;
static byte H_ERROR_POS_LIMIT			= 0x02;
static byte H_ERROR_TEMPERATURE_LIMIT	= 0x04;
static byte H_ERROR_INVALID_PKT			= 0x08;
static byte H_ERROR_OVERLOAD			= 0x10;
static byte H_ERROR_DRIVER_FAULT  		= 0x20;
static byte H_ERROR_EEPREG_DISTORT		= 0x40;

// HERKULEX Broadcast Servo ID
static byte BROADCAST_ID = 0xFE;

class Herkulex {

public:

  Herkulex() {
      serial = NULL;
  }

  ~Herkulex() {
      close();
  }

  bool open(const char* serialDeviceName,
       int baudrate=115200, int serialReadTimeout=1000,
       int ackMode=ACK_REPLY_ON_REQ);
  bool close();

  bool stat(int servoID, byte *stat);
  void  setACKMode(int valueACK);
  byte  model(int id);
  void  set_ID(int ID_Old, int ID_New);
  void  clearError(int servoID);
  
  void  torqueON(int servoID);
  void  torqueOFF(int servoID);
  
  bool moveAll(int servoID, int Goal, int iLed);
  bool moveSpeedAll(int servoID, int Goal, int iLed);
  bool moveAllAngle(int servoID, float angle, int iLed);
  bool actionAll(int pTime);
  
  bool moveSpeedOne(int servoID, int Goal, int pTime, int iLed);
  bool moveOne(int servoID, int Goal, int pTime, int iLed);
  bool moveOneAngle(int servoID, float angle, int pTime, int iLed);
  
  bool getPosition(int servoID, int *position);
  bool getAngle(int servoID, float *angle);
  bool getSpeed(int servoID, int* speed);
		
  bool  setPositionLimit(int servoID, float min, float max);
  bool  getPositionLimit(int servoID, float *min, float *max);

  bool  setPID(int servoID, int kp, int ki, int kd);
  bool  getPID(int servoID, int *kp, int *ki, int *kd);

  void  reboot(int servoID);
  void  setLed(int servoID, int valueLed);
 
  bool  writeRegistryRAM(int servoID, int address, int writeByte);
  bool  writeWordRegistryRAM(int servoID, int address, unsigned int writeWord);
  bool  writeRegistryEEP(int servoID, int address, int writeByte);

  bool readRegistryRAM(int servoID, int address, int len, int* value);
  bool readRegistryEEP(int servoID, int address, int len, int* value);

  void delay(unsigned int t);

  bool getVoltage(int servoID, float *volt);
  bool getTemprature(int servoID, float *temp);

  //void test();

// private area  
private:  
  bool sendData(byte* buffer, int lenght);
  bool readData(int size);
  void addData(int GoalLSB, int GoalMSB, int set, int servoID);
  int  checksum1(byte* data, int lenghtString);
  int  checksum2(int XOR);
  void clearBuffer();
  
  int pSize;
  int pID;
  int cmd;
  int lenghtString;
  int ck1;
  int ck2;
  
  int conta;
  
  int XOR;
  int playTime;
    
  byte data[DATA_SIZE]; 
  byte dataEx[DATA_MOVE+8];
  byte moveData[DATA_MOVE];

  int serialReadTimeout;
  serial::Serial* serial;
  //yarp::dev::ISerialDevice *iSerial;
  //yarp::dev::PolyDriver driver;

};

#endif    // Herkulex_h
