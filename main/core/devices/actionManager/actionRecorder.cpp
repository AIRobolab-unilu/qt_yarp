// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2016 Lux Future Robotic
* Authors: Ali Paikan
* email:   ali.paikan@iit.it
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <string>
//#include <algorithm>

#include <string>
#include <fstream>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>

#include <actionRecorder.h>

#define MAX_RECORD_COUNT    1e6

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace cuddie::dev;

ActionRecorder::ActionRecorder(yarp::os::Property& prop,
                               yarp::dev::IEncoders *iEnc) : RateThread(prop.find("sample-rate").asInt()) {
    yAssert(iEnc != NULL);
    ActionRecorder::iEnc = iEnc;
    axes = prop.find("axes").asInt();
    encPoses = new double[axes];
    config = prop;
}

ActionRecorder::~ActionRecorder() {
    delete[] encPoses;
}

bool ActionRecorder::threadInit() {
    return true;
}

void ActionRecorder::threadRelease() {
}

bool ActionRecorder::save() {
    if(isRunning()) {
        yWarning()<<"Stop the recording thread before calling save()";
        return false;
    }

    recorderMutex.lock();
    if(recordedData.size()) { // we have some data to save
        Property* header = recordedData.get(0).asDict();
        ofstream file;
        std::string filename = header->find("filename").asString();
        file.open(filename.c_str());
        if(!file.is_open()) {
            yError()<<"Cannot save recorded data to"<<filename;
            recordedData.clear();
            recorderMutex.unlock();
            return false;
        }
        header->unput("timeout");
        header->unput("t_start");
        header->unput("filename");
        file<<header->toString()<<endl;
        for(int i=1; i<recordedData.size(); i++) {
            Bottle* bt = recordedData.get(i).asList();
            file<<bt->toString().c_str()<<endl;
        }
        file.close();
    }
    recorderMutex.unlock();
    return true;
}

void ActionRecorder::run() {
    // limit the max recording count
    recorderMutex.lock();
    Property* header = recordedData.get(0).asDict();
    bool shouldStop = (recordedData.size() > MAX_RECORD_COUNT);
    if(header->check("timeout"))
        shouldStop |= ((Time::now() - header->find("t_start").asDouble()) > header->find("timeout").asDouble());
    if(shouldStop) {
        yInfo()<<"Stop recording the action (user-defined timeout or MAX_RECORD_COUNT)";
        RateThread::askToStop();
        recorderMutex.unlock();
        return;
    }
    // get the encoder values
    if(iEnc->getEncoders(encPoses)) {
        Bottle& sample = recordedData.addList();
        sample.addInt(recordedData.size()+1);
        sample.addDouble(Time::now());
        for(int i=0; i<axes; i++)
            sample.addDouble(encPoses[i]);
    }
    recorderMutex.unlock();
}
