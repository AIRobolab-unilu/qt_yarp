
#include "Herkulex.h"

#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <yarp/os/LogStream.h>

using namespace yarp::os;
using namespace serial;

void Herkulex::delay(unsigned int t) {
    yarp::os::Time::delay((double)t/1000.0);
}

// initialize servos
bool Herkulex::open(const char* serialDeviceName,
                    int serialBaudRate, int serialReadTimeout,
                    int ackMode)
{    
    Herkulex::serialReadTimeout = serialReadTimeout;

    conta=0;
    lenghtString=0;

    /*
    Property prop;
    prop.put("device", "serialport");
    prop.put("comport", serialDeviceName);
    prop.put("baudrate", serialBaudRate);
    prop.put("readtimeoutmsec", serialReadTimeout);
    prop.put("paritymode", "NONE");
    prop.put("databits", 8);
    prop.put("stopbits", 1);    
    //prop.put("rcvenb", 1);
    prop.put("verbose", 0);

    driver.open(prop);
    if(!driver.isValid()){
        fprintf(stderr, "Herkulex: Error opening PolyDriver check parameters\n");
        return false;
    }

    driver.view(iSerial);
    if(!iSerial) {
        fprintf(stderr, "Error opening serial driver. Device not available\n");
        return false;
    }
    */
    try {
        //serial->open();
        serial = new Serial(serialDeviceName,
                            115200,
                            Timeout::simpleTimeout(serialReadTimeout));
    }
    catch(serial::SerialException) {
        yError()<<" Herkulex: Failed to open serial device"<<serialDeviceName;
        return false;
    }
    catch(serial::IOException) {
        yError()<<" Herkulex: Failed to open serial device"<<serialDeviceName <<"(IO exception)";
        return false;
    }
    catch(std::invalid_argument) {
        yError()<<" Herkulex: Failed to open serial device"<<serialDeviceName<<"(invalid argument)";
        return false;
    }

    if(!serial->isOpen()) {
        yError()<<"Herkulex: Cannot open serial device";
        return false;
    }
    serial->flush();

    delay(500);
    clearError(BROADCAST_ID);	// clear error for all servos
    delay(10);
    setACKMode(ackMode);						// set ACK
    delay(10);
    //torqueON(BROADCAST_ID);		// torqueON for all servos
    //delay(10);
    return true;
}

bool Herkulex::close() {
    if(serial && serial->isOpen()) {
        serial->close();
        delete serial;
    }
    serial = NULL;
    //driver.close();
}

// stat
bool Herkulex::stat(int servoID, byte *stat)
{
	pSize    = 0x07;			//3.Packet size
	pID      = servoID;			//4.Servo ID - 0XFE=All servos
	cmd      = HSTAT;			//5.CMD
	
	ck1=(pSize^pID^cmd)&0xFE;
        ck2=(~(pSize^pID^cmd))&0xFE ; 
  
	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	     
	sendData(dataEx, pSize);
	delay(2);

    if(!readData(9)) 				// read 9 bytes from serial
        return false;
	
	pSize = dataEx[2];           // 3.Packet size 7-58
	pID   = dataEx[3];           // 4. Servo ID
	cmd   = dataEx[4];           // 5. CMD
	data[0]=dataEx[7];
    data[1]=dataEx[8];
    lenghtString=2;

	
    ck1 = (dataEx[2]^dataEx[3]^dataEx[4]^dataEx[7]^dataEx[8]) & 0xFE;
	ck2=checksum2(ck1);			
	
	if (ck1 != dataEx[5]) return -1; //checksum verify
	if (ck2 != dataEx[6]) return -2;

    *stat = dataEx[7];			// return status
    return true;
}

// torque on - 
void Herkulex::torqueON(int servoID)
{
	pSize = 0x0A;               // 3.Packet size 7-58
	pID   = servoID;            // 4. Servo ID
	cmd   = HRAMWRITE;          // 5. CMD
	data[0]=0x34;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	data[2]=0x60;               // 10. 0x60=Torque ON
	lenghtString=3;             // lenghtData
  	
	ck1=checksum1(data,lenghtString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Torque ON

	sendData(dataEx, pSize);
}

// torque off - the torque is FREE, not Break
void Herkulex::torqueOFF(int servoID)
{
	pSize = 0x0A;               // 3.Packet size 7-58
	pID   = servoID;            // 4. Servo ID
	cmd   = HRAMWRITE;          // 5. CMD
	data[0]=0x34;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	data[2]=0x00;               // 10. 0x00=Torque Free
	lenghtString=3;             // lenghtData
  	
	ck1=checksum1(data,lenghtString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Torque Free

    sendData(dataEx, pSize);

}

// ACK  - 0=No Replay, 1=Only reply to READ CMD, 2=Always reply
void Herkulex::setACKMode(int valueACK)
{
	pSize = 0x0A;               // 3.Packet size 7-58
	pID   = 0xFE;	            // 4. Servo ID
	cmd   = HRAMWRITE;          // 5. CMD
	data[0]=0x34;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	data[2]=valueACK;           // 10.Value. 0=No Replay, 1=Only reply to READ CMD, 2=Always reply
	lenghtString=3;             // lenghtData
  	
	ck1=checksum1(data,lenghtString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Value

 	sendData(dataEx, pSize);
}

// model - 1=0101 - 2=0201
byte Herkulex::model(int id)
{
	pSize = 0x09;               // 3.Packet size 7-58
    pID   = id;	            // 4. Servo ID
	cmd   = HEEPREAD;           // 5. CMD
	data[0]=0x00;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	lenghtString=2;             // lenghtData
  	
	ck1=checksum1(data,lenghtString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address
	dataEx[8] = data[1]; 		// Length

    sendData(dataEx, pSize);

    delay(1);
    readData(9);
	
	pSize = dataEx[2];           // 3.Packet size 7-58
	pID   = dataEx[3];           // 4. Servo ID
	cmd   = dataEx[4];           // 5. CMD
	data[0]=dataEx[7];           // 8. 1st byte
	lenghtString=1;              // lenghtData
  	
	ck1=checksum1(data,lenghtString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	if (ck1 != dataEx[5]) return -1; //checksum verify
	if (ck2 != dataEx[6]) return -2;
		
	return dataEx[7];			// return status
}

// setID - Need to restart the servo
void Herkulex::set_ID(int ID_Old, int ID_New)
{
	pSize = 0x0A;               // 3.Packet size 7-58
	pID   = ID_Old;		        // 4. Servo ID OLD - original servo ID
	cmd   = HEEPWRITE;          // 5. CMD
	data[0]=0x06;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	data[2]=ID_New;             // 10. ServoID NEW
	lenghtString=3;             // lenghtData
  	
	ck1=checksum1(data,lenghtString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Value

	sendData(dataEx, pSize);
}

// clearError
void Herkulex::clearError(int servoID)
{
	pSize = 0x0B;               // 3.Packet size 7-58
	pID   = servoID;     		// 4. Servo ID - 253=all servos
	cmd   = HRAMWRITE;          // 5. CMD
	data[0]=0x30;               // 8. Address
	data[1]=0x02;               // 9. Lenght
	data[2]=0x00;               // 10. Write error=0
	data[3]=0x00;               // 10. Write detail error=0
	
	lenghtString=4;             // lenghtData
  	
	ck1=checksum1(data,lenghtString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Value1
	dataEx[10]= data[3]; 		// Value2

	sendData(dataEx, pSize);
}

// move all servo at the same time to a position: servo list building
bool Herkulex::moveAll(int servoID, int Goal, int iLed)
{
    if (Goal > 1023 || Goal < 0)
        return false;						 //0 <--> 1023 range

    int iMode=0;                   //mode=position
    int iStop=0;                   //stop=0


    // Position definition
    int posLSB=Goal & 0X00FF;					// MSB Pos
    int posMSB=(Goal & 0XFF00) >> 8;			// LSB Pos

    //led
    int iBlue=0;
    int iGreen=0;
    int iRed=0;
    switch (iLed) {
        case 1:
        iGreen=1;
        break;
        case 2:
        iBlue=1;
        break;
        case 3:
        iRed=1;
        break;
    }

    int SetValue=iStop+iMode*2+iGreen*4+iBlue*8+iRed*16;	//assign led value

    addData(posLSB, posMSB, SetValue, servoID);	//add servo data to list, pos mode
    return true;
}

// move all servo at the same time to a position: servo list building
bool Herkulex::moveAllAngle(int servoID, float angle, int iLed)
{
    if (angle > 160.0|| angle < -160.0)
        return false; // out of the range
    int position = (int)(angle/0.325) + 512;
    moveAll(servoID, position, iLed);
    return true;
}



// move all servo at the same time with different speeds: servo list building
bool Herkulex::moveSpeedAll(int servoID, int Goal, int iLed)
{
	  if (Goal > 1023 || Goal < -1023)
        return false;								 //-1023 <--> 1023 range

	  int iMode=1;                  		// mode=continous rotation
	  int iStop=0;                  		// Stop=0

	  // Speed definition
	  int GoalSpeedSign;
	  if (Goal < 0) {
		GoalSpeedSign = (-1)* Goal ;
		GoalSpeedSign |= 0x4000;  //bit n\B014 
	  } 
	  else {
		GoalSpeedSign = Goal;
	  }

	  int speedGoalLSB=GoalSpeedSign & 0X00FF; 	      		 // MSB speedGoal 
	  int speedGoalMSB=(GoalSpeedSign & 0xFF00) >> 8;        // LSB speedGoal 

	  //led 
	  int iBlue=0;
	  int iGreen=0;
	  int iRed=0;
	  switch (iLed) {
	  case 1:
		iGreen=1;
		break;
	  case 2:
		iBlue=1;
		break;
	  case 3:
		iRed=1;
		break;
	  }

	  int SetValue=iStop+iMode*2+iGreen*4+iBlue*8+iRed*16;	//assign led value
	  addData(speedGoalLSB, speedGoalMSB, SetValue, servoID);		//add servo data to list, speed mode
      return true;
}



// move all servo with the same execution time
bool Herkulex::actionAll(int pTime)
{
    if ((pTime <0) || (pTime > 2856))
        return false;

    pSize = 0x08 + conta;     	    // 3.Packet size 7-58
	cmd   = HSJOG;		 			// 5. CMD SJOG Write n servo with same execution time
	playTime=int((float)pTime/11.2);// 8. Execution time
 
    pID=0xFE^playTime;
    ck1=checksum1(moveData,conta);	//6. Checksum1
	ck2=checksum2(ck1);				//7. Checksum2

    pID=0xFE;
	dataEx[0] = 0xFF;				// Packet Header
	dataEx[1] = 0xFF;				// Packet Header	
	dataEx[2] = pSize;	 			// Packet Size
	dataEx[3] = pID;				// Servo ID
	dataEx[4] = cmd;				// Command Ram Write
	dataEx[5] = ck1;				// Checksum 1
	dataEx[6] = ck2;				// Checksum 2
	dataEx[7] = playTime;			// Execution time	
	
	for (int i=0; i < conta; i++)
		dataEx[i+8]=moveData[i];	// Variable servo data

	sendData(dataEx, pSize);
	conta=0; 						//reset counter   
    return true;

}

bool Herkulex::getVoltage(int servoID, float *volt) {
    int data;
    if(!readRegistryRAM(servoID, 54, 1, &data))
        return false;
    *volt = data * 0.074;
    return true;
}

bool Herkulex::getTemprature(int servoID, float *temp) {
    int data;
    if(!readRegistryRAM(servoID, 55, 1, &data))
        return false;
    *temp = data;
    return true;
}

bool Herkulex::readRegistryEEP(int servoID, int address, int len, int* value) {
    pSize = 0x09;               // 3.Packet size 7-58
    pID   = servoID;            // 4. Servo ID
    cmd   = HEEPREAD;           // 5. CMD
    data[0]= address;           // 8. Address
    data[1]= len;               // 9. Lenght
    lenghtString=2;             // lenghtData

    ck1=checksum1(data,lenghtString);	//6. Checksum1
    ck2=checksum2(ck1);					//7. Checksum2

    dataEx[0] = 0xFF;			// Packet Header
    dataEx[1] = 0xFF;			// Packet Header
    dataEx[2] = pSize;	 		// Packet Size
    dataEx[3] = pID;			// Servo ID
    dataEx[4] = cmd;			// Command Ram Write
    dataEx[5] = ck1;			// Checksum 1
    dataEx[6] = ck2;			// Checksum 2
    dataEx[7] = data[0]; 		// Address
    dataEx[8] = data[1]; 		// Length

    sendData(dataEx, pSize);

    delay(1);
    if(!readData(11+len))
        return false;

    /*
    pSize = dataEx[2];           // 3.Packet size 7-58
    pID   = dataEx[3];           // 4. Servo ID
    cmd   = dataEx[4];           // 5. CMD
    data[0]=dataEx[7];           // 8. 1st byte
    lenghtString=1;              // lenghtData
    */

    pSize = dataEx[2];           // 3.Packet size 7-58
    pID   = dataEx[3];           // 4. Servo ID
    cmd   = dataEx[4];           // 5. CMD
    data[0]=dataEx[7];
    data[1]=dataEx[8];
    data[2]=dataEx[9];
    data[3]=dataEx[10];
    data[4]=dataEx[11];
    data[5]=dataEx[12];

    ck1=checksum1(data,lenghtString);	//6. Checksum1
    ck2=checksum2(ck1);					//7. Checksum2

    //if (ck1 != dataEx[5]) return -1; //checksum verify
    //if (ck2 != dataEx[6]) return -2;

    *value = (len == 1) ? dataEx[9] : (dataEx[10]<<8) | dataEx[9];
    return true;
}

bool Herkulex::readRegistryRAM(int servoID, int address, int len, int* value) {

    pSize = 0x09;               // 3.Packet size 7-58
    pID   = servoID;     	    // 4. Servo ID - 253=all servos
    cmd   = HRAMREAD;           // 5. CMD
    data[0]=address;            // 8. Address
    data[1]=len;        // 9. Lenght

    lenghtString=2;             // lenghtData

    ck1=checksum1(data,lenghtString);	//6. Checksum1
    ck2=checksum2(ck1);					//7. Checksum2

    dataEx[0] = 0xFF;			// Packet Header
    dataEx[1] = 0xFF;			// Packet Header
    dataEx[2] = pSize;	 		// Packet Size
    dataEx[3] = pID;			// Servo ID
    dataEx[4] = cmd;			// Command Ram Write
    dataEx[5] = ck1;			// Checksum 1
    dataEx[6] = ck2;			// Checksum 2
    dataEx[7] = data[0];      	// Address
    dataEx[8] = data[1]; 		// Length

    sendData(dataEx, pSize);

    delay(1);
    if(!readData(12))
        return false;

    pSize = dataEx[2];           // 3.Packet size 7-58
    pID   = dataEx[3];           // 4. Servo ID
    cmd   = dataEx[4];           // 5. CMD
    data[0]=dataEx[7];
    data[1]=dataEx[8];
    data[2]=dataEx[9];
    data[3]=dataEx[10];
    data[4]=dataEx[11];
    data[5]=dataEx[12];
    lenghtString=6;

    ck1=checksum1(data,lenghtString);	//6. Checksum1
    ck2=checksum2(ck1);					//7. Checksum2

    if (ck1 != dataEx[5]) return -1;
    if (ck2 != dataEx[6]) return -1;

    //printf("%x, %x\n", dataEx[9]&&0xFF, dataEx[8]&&0xFF);

    *value = (len == 1) ? dataEx[9] : (dataEx[10]<<8) | dataEx[9];
    return true;
}

/*
void Herkulex::test() {
    char cmd[] = {0xFF, 0xFF, 0x09, 0xFD, 0x04, 0xC4, 0x3A, 0x35, 0x01};
    iSerial->send(cmd, 9);
    readData(12);
}
*/

// get Position
bool Herkulex::getPosition(int servoID, int* position) {

    pSize = 0x09;               // 3.Packet size 7-58
	pID   = servoID;     	    // 4. Servo ID - 253=all servos
	cmd   = HRAMREAD;           // 5. CMD
	data[0]=0x3A;               // 8. Address
	data[1]=0x02;               // 9. Lenght
	
	lenghtString=2;             // lenghtData
  	
	ck1=checksum1(data,lenghtString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0];      	// Address  
	dataEx[8] = data[1]; 		// Length
	
	sendData(dataEx, pSize);

    delay(1);
    if(!readData(13))
        return false;

        	
	pSize = dataEx[2];           // 3.Packet size 7-58
	pID   = dataEx[3];           // 4. Servo ID
	cmd   = dataEx[4];           // 5. CMD
	data[0]=dataEx[7];
    data[1]=dataEx[8];
    data[2]=dataEx[9];
    data[3]=dataEx[10];
    data[4]=dataEx[11];
    data[5]=dataEx[12];
    lenghtString=6;

    ck1=checksum1(data,lenghtString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

    if (ck1 != dataEx[5]) return -1;
	if (ck2 != dataEx[6]) return -1;

    *position = ((dataEx[10]&0x03)<<8) | dataEx[9];
    return true;
	
}

bool Herkulex::getAngle(int servoID, float* angle) {
    int pos;
    if(!getPosition(servoID, &pos))
        return false;
    *angle = (pos-512) * 0.325;
    return true;
}

// reboot single servo - pay attention 253 - all servos doesn't work!
void Herkulex::reboot(int servoID) {
        
    pSize = 0x07;               // 3.Packet size 7-58
	pID   = servoID;     	    // 4. Servo ID - 253=all servos
	cmd   = HREBOOT;            // 5. CMD
    ck1=(pSize^pID^cmd)&0xFE;
    ck2=(~(pSize^pID^cmd))&0xFE ; ;	

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	
	sendData(dataEx, pSize);

}

// LED  - see table of colors 
void Herkulex::setLed(int servoID, int valueLed)
{
	pSize   = 0x0A;               // 3.Packet size 7-58
	pID     = servoID;            // 4. Servo ID
	cmd     = HRAMWRITE;          // 5. CMD
	data[0] = 0x35;               // 8. Address 53
    data[1] = 0x01;               // 9. Lenght
	data[2] = valueLed;           // 10.LedValue
	lenghtString=3;               // lenghtData
  	  	
	ck1=checksum1(data,lenghtString);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = pSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0];        // Address
	dataEx[8] = data[1];       	// Length
	dataEx[9] = data[2];        // Value

	sendData(dataEx, pSize);
}

// get the speed for one servo - values betweeb -1023 <--> 1023
bool Herkulex::getSpeed(int servoID, int* speed) {
  //int speedy  = 0;

  pSize = 0x09;               // 3.Packet size 7-58
  pID   = servoID;     	   	  // 4. Servo ID 
  cmd   = HRAMREAD;           // 5. CMD
  data[0]=0x40;               // 8. Address
  data[1]=0x02;               // 9. Lenght

  lenghtString=2;             // lenghtData

  ck1=checksum1(data,lenghtString);		//6. Checksum1
  ck2=checksum2(ck1);					//7. Checksum2

  dataEx[0] = 0xFF;			// Packet Header
  dataEx[1] = 0xFF;			// Packet Header	
  dataEx[2] = pSize;		// Packet Size
  dataEx[3] = pID;			// Servo ID
  dataEx[4] = cmd;			// Command Ram Write
  dataEx[5] = ck1;			// Checksum 1
  dataEx[6] = ck2;			// Checksum 2
  dataEx[7] = data[0]; 	    // Address  
  dataEx[8] = data[1]; 		// Length

  sendData(dataEx, pSize);

  delay(1);
  if(!readData(13))
      return false;

  pSize = dataEx[2];           // 3.Packet size 7-58
  pID   = dataEx[3];           // 4. Servo ID
  cmd   = dataEx[4];           // 5. CMD
  data[0]=dataEx[7];
  data[1]=dataEx[8];
  data[2]=dataEx[9];
  data[3]=dataEx[10];
  data[4]=dataEx[11];
  data[5]=dataEx[12];
  lenghtString=6;

  ck1=checksum1(data,lenghtString);	//6. Checksum1
  ck2=checksum2(ck1);				//7. Checksum2

  if (ck1 != dataEx[5]) return -1;
  if (ck2 != dataEx[6]) return -1;

  *speed = ((dataEx[10]&0xFF)<<8) | dataEx[9];
  return true;
}



// move one servo with continous rotation
bool Herkulex::moveSpeedOne(int servoID, int Goal, int pTime, int iLed)
{
  if (Goal > 1023 || Goal < -1023) return false;              // speed (goal) non correct
  if ((pTime <0) || (pTime > 2856)) return false;

  int GoalSpeedSign;
  if (Goal < 0) {
    GoalSpeedSign = (-1)* Goal ;
    GoalSpeedSign |= 0x4000;  //bit n\B014 
  } 
  else {
    GoalSpeedSign = Goal;
  }
  int speedGoalLSB=GoalSpeedSign & 0X00FF; 		       // MSB speedGoal 
  int speedGoalMSB=(GoalSpeedSign & 0xFF00) >> 8;      // LSB speedGoal 

  //led 
  int iBlue=0;
  int iGreen=0;
  int iRed=0;
  switch (iLed) {
  case 1:
    iGreen=1;
    break;
  case 2:
    iBlue=1;
    break;
  case 3:
    iRed=1;
    break;
  }
  int SetValue=2+iGreen*4+iBlue*8+iRed*16;		//assign led value 

  playTime=int((float)pTime/11.2);				// 8. Execution time

  pSize = 0x0C;              					// 3.Packet size 7-58
  cmd   = HSJOG;      					        // 5. CMD

  data[0]=speedGoalLSB;            			    // 8. speedLSB
  data[1]=speedGoalMSB;              			// 9. speedMSB
  data[2]=SetValue;                          	// 10. Mode=0;
  data[3]=servoID;                    			// 11. ServoID

  pID=servoID^playTime;

  lenghtString=4;             					// lenghtData

  ck1=checksum1(data,lenghtString);				//6. Checksum1
  ck2=checksum2(ck1);							//7. Checksum2

  pID=servoID;

  dataEx[0] = 0xFF;				// Packet Header
  dataEx[1] = 0xFF;				// Packet Header	
  dataEx[2] = pSize;	 		// Packet Size
  dataEx[3] = pID;				// Servo ID
  dataEx[4] = cmd;				// Command Ram Write
  dataEx[5] = ck1;				// Checksum 1
  dataEx[6] = ck2;				// Checksum 2
  dataEx[7] = playTime;  		// Execution time	
  dataEx[8] = data[0];
  dataEx[9] = data[1];
  dataEx[10] = data[2];
  dataEx[11] = data[3];
  
  sendData(dataEx, pSize);
  return true;
}

// move one servo at goal position 0 - 1024
bool Herkulex::moveOne(int servoID, int Goal, int pTime, int iLed)
{
  if (Goal > 1023 || Goal < 0) return false;              // speed (goal) non correct
  if ((pTime <0) || (pTime > 2856)) return false;

  // Position definition
  int posLSB=Goal & 0X00FF;								// MSB Pos
  int posMSB=(Goal & 0XFF00) >> 8;						// LSB Pos

  //led 
  int iBlue=0;
  int iGreen=0;
  int iRed=0;
  switch (iLed) {
  case 1:
    iGreen=1;
    break;
  case 2:
    iBlue=1;
    break;
  case 3:
    iRed=1;
    break;
  }
  int SetValue=iGreen*4+iBlue*8+iRed*16;	//assign led value 

  playTime=int((float)pTime/11.2);			// 8. Execution time

  pSize = 0x0C;          			    	// 3.Packet size 7-58
  cmd   = HSJOG;              				// 5. CMD

  data[0]=posLSB;               			// 8. speedLSB
  data[1]=posMSB;               			// 9. speedMSB
  data[2]=SetValue;                         // 10. Mode=0;
  data[3]=servoID;                    		// 11. ServoID

  pID=servoID^playTime;

  lenghtString=4;             				// lenghtData

  ck1=checksum1(data,lenghtString);			//6. Checksum1
  ck2=checksum2(ck1);						//7. Checksum2

  pID=servoID;

  dataEx[0] = 0xFF;				// Packet Header
  dataEx[1] = 0xFF;				// Packet Header	
  dataEx[2] = pSize;	 		// Packet Size
  dataEx[3] = pID;				// Servo ID
  dataEx[4] = cmd;				// Command Ram Write
  dataEx[5] = ck1;				// Checksum 1
  dataEx[6] = ck2;				// Checksum 2
  dataEx[7] = playTime;  		// Execution time	
  dataEx[8] = data[0];
  dataEx[9] = data[1];
  dataEx[10] = data[2];
  dataEx[11] = data[3];

  sendData(dataEx, pSize);
  return true;
}

// move one servo to an angle between -160 and 160
bool Herkulex::moveOneAngle(int servoID, float angle, int pTime, int iLed) {
    if (angle > 160.0|| angle < -160.0)
        return false;
	int position = (int)(angle/0.325) + 512;
	moveOne(servoID, position, pTime, iLed);
    return true;
}

bool  Herkulex::setPositionLimit(int servoID, float min, float max) {
    if (min > 160.0|| min < -160.0)
        return false;
    if (max > 160.0|| max < -160.0)
        return false;

    int min_pos = (int)(min/0.325) + 512;
    int max_pos = (int)(max/0.325) + 512;
    bool ret = writeWordRegistryRAM(servoID, 20, min_pos);
    delay(20);
    ret &= writeWordRegistryRAM(servoID, 22, max_pos);
    return ret;
}

bool Herkulex::getPositionLimit(int servoID, float *min, float *max)
{
    int min_pos;
    int max_pos;
    if(!readRegistryRAM(servoID, 20, 2, &min_pos))
        return false;

    if(!readRegistryRAM(servoID, 22, 2, &max_pos))
        return false;
    *min = (min_pos-512) * 0.325;
    *max = (max_pos-512) * 0.325;
    return true;
}

bool  Herkulex::setPID(int servoID, int kp, int ki, int kd) {
    bool ret = writeWordRegistryRAM(servoID, 24, kp);
    ret &= writeWordRegistryRAM(servoID, 26, kd);
    ret &= writeWordRegistryRAM(servoID, 28, ki);
    return ret;
}

bool Herkulex::getPID(int servoID, int *kp, int *ki, int *kd) {
    if(!readRegistryRAM(servoID, 24, 2, kp))
        return false;
    if(!readRegistryRAM(servoID, 26, 2, kd))
        return false;
    if(!readRegistryRAM(servoID, 28, 2, ki))
        return false;
    return true;
}


// write registry in the RAM: one byte 
bool Herkulex::writeRegistryRAM(int servoID, int address, int writeByte)
{
    pSize = 0x0A;               	// 3.Packet size 7-58
    pID   = servoID;     			// 4. Servo ID - 253=all servos
    cmd   = HRAMWRITE;          	// 5. CMD
    data[0]=address;              // 8. Address
    data[1]=0x01;               	// 9. Lenght
    data[2]=writeByte;            // 10. Write error=0

    lenghtString=3;             	// lenghtData

    ck1=checksum1(data,lenghtString);	//6. Checksum1
    ck2=checksum2(ck1);				//7. Checksum2

    dataEx[0] = 0xFF;			// Packet Header
    dataEx[1] = 0xFF;			// Packet Header
    dataEx[2] = pSize;	 	// Packet Size
    dataEx[3] = pID;			// Servo ID
    dataEx[4] = cmd;			// Command Ram Write
    dataEx[5] = ck1;			// Checksum 1
    dataEx[6] = ck2;			// Checksum 2
    dataEx[7] = data[0]; 		// Address 52
    dataEx[8] = data[1]; 		// Length
    dataEx[9] = data[2]; 		// Value1
    dataEx[10]= data[3]; 		// Value2

    return sendData(dataEx, pSize);
}

// write registry in the RAM: one word
bool Herkulex::writeWordRegistryRAM(int servoID, int address, unsigned int writeWord)
{
    bool ret = writeRegistryRAM(servoID, address, writeWord & 0xFF);
    ret &= writeRegistryRAM(servoID, address+1, writeWord >> 8);
    return ret;
}

// write registry in the EEP memory (ROM): one byte 
bool Herkulex::writeRegistryEEP(int servoID, int address, int writeByte)
{
  pSize = 0x0A;                  // 3.Packet size 7-58
  pID   = servoID;     	         // 4. Servo ID - 253=all servos
  cmd   = HEEPWRITE;             // 5. CMD
  data[0]=address;               // 8. Address
  data[1]=0x01;                  // 9. Lenght
  data[2]=writeByte;             // 10. Write error=0
 
  lenghtString=3;           	 // lenghtData

  ck1=checksum1(data,lenghtString);	//6. Checksum1
  ck2=checksum2(ck1);				//7. Checksum2

  dataEx[0] = 0xFF;			// Packet Header
  dataEx[1] = 0xFF;			// Packet Header	
  dataEx[2] = pSize;		// Packet Size
  dataEx[3] = pID;			// Servo ID
  dataEx[4] = cmd;			// Command Ram Write
  dataEx[5] = ck1;			// Checksum 1
  dataEx[6] = ck2;			// Checksum 2
  dataEx[7] = data[0]; 		// Address 52
  dataEx[8] = data[1]; 		// Length
  dataEx[9] = data[2]; 		// Value1
  dataEx[10]= data[3]; 		// Value2

  return sendData(dataEx, pSize);
}


// Private Methods //////////////////////////////////////////////////////////////

// checksum1
int Herkulex::checksum1(byte* data, int lenghtString)
{
  XOR = 0;	
  XOR = XOR ^ pSize;
  XOR = XOR ^ pID;
  XOR = XOR ^ cmd;
  for (int i = 0; i < lenghtString; i++) 
  {
    XOR = XOR ^ data[i];
  }
  return XOR&0xFE;
}


// checksum2
int Herkulex::checksum2(int XOR)
{
  return (~XOR)&0xFE;
}

// add data to variable list servo for syncro execution
void Herkulex::addData(int GoalLSB, int GoalMSB, int set, int servoID)
{
  moveData[conta++]=GoalLSB;  
  moveData[conta++]=GoalMSB;
  moveData[conta++]=set;
  moveData[conta++]=servoID;
}

// Sending the buffer long lenght to Serial port
bool Herkulex::sendData(byte* buffer, int lenght)
{
    //return iSerial->send((char*)buffer, lenght);
    try{
        serial->write(buffer, lenght);
    }
    catch(serial::IOException) {
        yWarning()<<" IO excpetion happened while reading from serial port";
        return false;
    }
    catch(serial::SerialException) {
        yWarning()<<"Serial excpetion happened while reading from serial port";
        return false;
    }
    catch(serial::PortNotOpenedException) {
        yError()<<" Serial port is not open";
        return false;
    }
    return true;
}

// * Receiving the lenght of bytes from Serial port
bool Herkulex::readData(int size)
{
    bool beginsave = false;
    int len = 0;
    byte prev_char = 0;
    double packetReadTimeout = 3.0*serialReadTimeout/1000.0;
    double t_start = Time::now();
    while (len < size) {

        if((Time::now() - t_start) > packetReadTimeout) {
            yError()<<"Herkulex: Could not read any valid packet after "<<packetReadTimeout<<"!";
            return false;
        }

        /*
        double t_read = Time::now();        
        while (iSerial->receiveChar(c) == 0) {            
            Herkulex::delay(1);
            if((Time::now() - t_read) > (serialReadTimeout/1000.0)) {
                yError()<<"Herkulex: timeout while reading serial port!";
                return false;
            }
        }
        */
        byte cur_char;
        try{
            int len = serial->read(&cur_char, 1);
            if(len!=1) {
                yWarning()<<"Timeout while reading serial port";
                continue;
            }
        }
        catch(serial::SerialException) {
            yWarning()<<"Serial excpetion happened while reading from serial port";
            continue;
        }
        catch(serial::PortNotOpenedException) {
            yError()<<" Serial port is not open";
            return false;
        }

       // printf("%x, ", cur_char & 0xff);
        if ( (prev_char == (byte)0xFF) && (cur_char == (byte)0xFF) ){
                beginsave = true;
                dataEx[0] = 0xFF;
                dataEx[1] = 0xFF;
                len=2; 				 // if found new header, begin again
        } else if (beginsave == true) {
               dataEx[len++] = cur_char;
        }
        prev_char = cur_char;
    }
    //printf("\n");
    return true;
}

//clear buffer in the serial port - better - try to do this
void Herkulex::clearBuffer()
{
//    Serial3.flush();
//    while (Serial3.available()){
//        Serial3.read();
//        delayMicroseconds(200);
//    }
}

