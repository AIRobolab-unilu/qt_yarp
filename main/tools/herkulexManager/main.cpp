/*
 *  Yarp Modules Manager
 *  Copyright: (C) 2010 RobotCub Consortium
 *				Italian Institute of Technology (IIT)
 *				Via Morego 30, 16163, 
 *				Genova, Italy
 * 
 *  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *  Authors: Ali Paikan <ali.paikan@iit.it>
 *
 */

#include <stdio.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <Herkulex.h>

using namespace yarp::os;
using namespace std;

#define MOTOR_DEFAULT   0xFD


/*
    bool configure(ResourceFinder &rf) {
        
        Property prop;        
        prop.put("device", "serialport");
        prop.put("comport", "/dev/ttyUSB1");
        prop.put("baudrate", 115200);
        prop.put("paritymode", "NONE");
        prop.put("databits", 8);
        prop.put("stopbits", 1);
        prop.put("readtimeoutmsec", 1000);
        //prop.put("rcvenb", 1);
        prop.put("verbose", 0);

        driver.open(prop);
        if(!driver.isValid()){
            fprintf(stderr, "Error opening PolyDriver check parameters\n");
            return false;
        }

        driver.view(iSerial);  
        if(!iSerial)
        {
            fprintf(stderr, "Error opening serial driver. Device not available\n");
            return false;
        }        
        //char led[] = {0xFF, 0xFF, 0x0A, 0xFD, 0x03, 0xC0, 0x3E, 0x35, 0x01, 0x01};
        //iSerial->send(led, 10);

        motor = new Herkulex(iSerial);
        motor->delay(500);
        motor->reboot(MOTOR_DEFAULT);
        motor->delay(500);
        motor->initialize();
        motor->delay(500);
        //motor->set_ID(MOTOR_DEFAULT, MOTOR_1);
        //motor->delay(500);
        //return false;

        motor->torqueOFF(MOTOR_1);
        motor->torqueOFF(MOTOR_2);
        motor->delay(10);

        // seeting Ki, kp
        motor->writeWordRegistryRAM(MOTOR_1, 28, 150);
        motor->writeWordRegistryRAM(MOTOR_1, 24, 500);

        motor->writeWordRegistryRAM(MOTOR_2, 28, 150);
        motor->writeWordRegistryRAM(MOTOR_2, 24, 500);

        //int kp = motor->readRegistryRAM(MOTOR_1, 24, 2);
        //int kd = motor->readRegistryRAM(MOTOR_1, 26, 2);
        //int ki = motor->readRegistryRAM(MOTOR_1, 28, 2);
        //printf("Kp: %d, Ki: %d, Kd: %d\n", kp, ki, kd);

        // setting compliant mode
        motor->writeRegistryRAM(MOTOR_1, 11, 150);
        motor->writeWordRegistryRAM(MOTOR_1, 12, 150);
        motor->writeRegistryRAM(MOTOR_2, 11, 250);
        motor->writeWordRegistryRAM(MOTOR_2, 12, 1500);

       // int offset = motor->readRegistryRAM(MOTOR_1, 11, 1);
       // int slope = motor->readRegistryRAM(MOTOR_1, 12, 2);
       // printf("offset: %d, slope: %d\n", offset, slope);

        // setting pmw offset
        //motor->writeRegistryRAM(MOTOR_1, 14, -100);
        //printf("pwm offset: %d\n", motor->readRegistryRAM(MOTOR_1, 14, 1));

        printf("voltage: %f\n", motor->getVoltage(MOTOR_1));

        motor->torqueON(MOTOR_1);
        motor->moveOneAngle(MOTOR_1, 0, 1000, LED_GREEN);
        motor->torqueON(MOTOR_2);
        motor->moveOneAngle(MOTOR_2, -10, 1000, LED_GREEN);

        motor->delay(500);
        
    
        return true;
    }
*/

void printHelp() {
    printf("usage: herkulexManager [options] ... \n");
    printf("options:\n");
    printf("   --device   [name]   Serial device name (e.g., /dev/ttyUSB0)\n");
    printf("   --baudrate [value]  Serial device baudrate (e.g., 115200\n");
    printf("   --id                Herkulex device id (default is 253)\n");
    printf("   --info              Retrieve current herkulex setting\n");
    printf("   --set-id [id]       Sets the new device ID\n");
    printf("   --reset-factory     Reset herkulex to factory default setting\n");
    printf("   --set-pos [pos]     Move the motor to a position\n");
    printf("   --wait [sec]        Wait for seconds\n");
}

int main(int argc, char* argv[])
{
    yarp::os::Network yarp;
    ResourceFinder rf;
    rf.configure(argc, argv);
    if((argc < 2) || rf.check("help")) {
        printHelp();
        return 0;
    }

    if(!rf.check("device")) {
        yError()<< " Serial device name must be given!";
        printHelp();
        return 0;
    }

    Herkulex herkulex;
    int baudrate = rf.check("baudrate") ? rf.find("baudrate").asInt() : 115200;
    int id = rf.check("id") ? rf.find("id").asInt() : 253;

    if(!herkulex.open(rf.find("device").asString().c_str(), baudrate)) {
        yError()<<" Failed to open device "<<rf.find("device").asString();
        return 0;
    }

    herkulex.reboot(id);
    Time::delay(1.0);
    byte model = herkulex.model(id);
    if(model==MODEL_DRS0101)
        yInfo()<<" Found Herkulex DRS0101";
    else if(model==MODEL_DRS0201)
        yInfo()<<" Found Herkulex DRS0201";
    else
        yWarning()<<" Found an unknown device ("<<model<<")";

    if(rf.check("set-id")) {
        yInfo()<< " Setting the device ID to"<<rf.find("set-id").asInt();
        herkulex.set_ID(id, rf.find("set-id").asInt());
        herkulex.delay(500);
        herkulex.close();
        return 0;
    }

    if(rf.check("set-pos")) {
        yInfo()<< " Moving the motor to "<< rf.find("set-pos").asDouble() <<" position";
        herkulex.torqueON(id);
        herkulex.delay(100);
        if(! herkulex.moveOneAngle(id, rf.find("set-pos").asDouble(), 500, LED_GREEN))
            yError()<<"Failed to set the position";
        else {
            Time::delay(1);
            if(rf.check("wait"))
                Time::delay(rf.find("wait").asDouble());
            herkulex.torqueOFF(id);
            herkulex.setLed(id, LED_OFF);            
            float pos;
            if(!herkulex.getAngle(id, &pos))
                yError()<<" Failed to read motor postion";
            else
                yInfo()<<" Current motor postion: "<<pos;
        }
        herkulex.close();
        return 0;
    }

    herkulex.close();
    return 0;
}

