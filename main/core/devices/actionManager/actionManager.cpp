// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2016 Lux Future Robotic
* Authors: Ali Paikan
* email:   ali.paikan@iit.it
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>

#include <string>
#include <algorithm>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/LogStream.h>

#include <actionManager.h>

#define MAX_RECORD_COUNT    1e6

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace cuddie::dev;

/*
bool ActionManager::threadInit() {
    return true;
}

void ActionManager::threadRelease() {
    rpcPort.close();
}

void ActionManager::run() {
    // limit the max recording count
    mainMutex.lock();
    Property* header = recordedData.get(0).asDict();
    shouldStop |= (recordedData.size() > MAX_RECORD_COUNT);
    if(header->check("timeout"))
        shouldStop |= ((Time::now() - header->find("t_start").asDouble()) > header->find("timeout").asDouble());
    if(shouldStop) {
        yInfo()<<"Stop recording the action (user-defined timeout or MAX_RECORD_COUNT)";
        RateThread::askToStop();
        mainMutex.unlock();
        return;
    }
    yInfo()<<"recording...";
    // get the encoder values
    if(iEnc->getEncoders(encPoses)) {
        Bottle& sample = recordedData.addList();
        sample.addInt(recordedData.size()+1);
        sample.addDouble(Time::now());
        for(int i=0; i<axes; i++)
            sample.addDouble(encPoses[i]);
    }
    mainMutex.unlock();
}
*/

/****************************************************************
 * @brief The cuddie::dev::ActionManager class
 */
ActionManager::ActionManager(){
    recorder = NULL;
}

ActionManager::~ActionManager() {
    close();
}


bool ActionManager::open(yarp::os::Searchable &config)
{    
    ActionManager::config.fromString(config.toString());

    actionsPath = "/home/actions";
    if(config.check("actions-path"))
        actionsPath = config.find("actions-path").asString();

    useStiffMode = false;
    if(config.check("stiff-mode"))
        useStiffMode = config.find("stiff-mode").asBool();
    yInfo()<<"Using stiff mode to play actions:"<<useStiffMode;

    simFramePeriod = 1.0;
    if(config.check("sim-frame-period"))
        simFramePeriod = config.find("sim-frame-period").asDouble();


    //TODO: make this one local!!!
    Property option("(device remote_controlboard)");
    option.put("remote", "/cuddie/motors");
    //option.put("remote", "/icubSim/left_arm");
    option.put("local","/actionManager/motors");

    // create and open the remote driver
    if (!motorDriver.open(option)) {
        yError()<<"Cannot open the remote driver!";
        return false;
    }

    // create the desired interfaces
    bool ret = motorDriver.view(iPos);
    ret &= motorDriver.view(iEnc);
    ret &= motorDriver.view(iCtrl);
    ret &= motorDriver.view(iInteract);
    if(!ret){
        yError()<<"Cannot open some of the interfaces!";
        motorDriver.close();
        return false;
    }

    if(!iPos->getAxes(&axes)) {
        yError()<<"Cannot get the number of axes!";
        motorDriver.close();
        return false;
    }

    this->yarp().attachAsServer(rpcPort);
    if(!rpcPort.open("/cuddie/actionManager:rpc")) {
        yError()<<"Cannot open port /cuddie/actionManager:rpc";
        motorDriver.close();
        return false;
    }

    // create the recorder thread
    Property prop;
    prop.put("axes", axes);
    prop.put("sample-rate", 100);
    recorder = new cuddie::dev::ActionRecorder(prop, iEnc);

    return true;
}

bool ActionManager::close()
{
    yInfo()<<"closing ActionManager!";
    if(recorder) {
        recorder->stop();
        delete recorder;
        recorder = NULL;
    }
    motorDriver.close();    
    return true;
}

bool ActionManager::play(const std::string& name,
                         const double speed, const bool sync) {
    std::string fileName = actionsPath+"/" + name + ".log";
    return playFile(fileName, speed, sync);

}

bool ActionManager::playFile(const std::string& filename,
                             const double speed, const bool sync) {
    yarp::os::Bottle data;
    if(loadSimAction(filename, data)) {
        return playSimAction(data, speed, sync);
        //yDebug()<<"Debuging and not running the action!";
        return true;
    }

    data.clear();
    if(loadAction(filename, data))
        return playAction(data, speed, sync);

    return false;
}

bool ActionManager::playAction(yarp::os::Bottle &data, const double speed, const bool sync) {
    for(int i=0; i<axes; i++) {

        //Fix 1. 06.2017
        // This was the original approach
        //iInteract->setInteractionMode(i, (useStiffMode) ? VOCAB_IM_STIFF : VOCAB_IM_COMPLIANT);

        iInteract->setInteractionMode(i, VOCAB_IM_STIFF );
        iCtrl->setControlMode(i, VOCAB_CM_POSITION);
        iPos->setRefSpeed(i, speed*100.0);
    }
    double t_prev = -1.0;
    for(int i=0; i<data.size(); i++) {
        
        //Fix 1. 06.2017 
        // This is a patch to move the arms smoothly until it reach the initial position. 
        if (i==0)
        {
            for(int j=0; j<axes; j++) {
                iPos->setRefSpeed(j, speed*10.0);
            }
        }   
        else if (i==1)
        {
            for(int j=0; j<axes; j++) {
                iPos->setRefSpeed(j, speed*100.0);
            }
        }

        Bottle* sample = data.get(i).asList();
        if(sample->isNull() || sample->size()!=axes+2) {
            yWarning()<<"playFile: wrong data type!";
            continue;
        }
        //yInfo()<<sample->toString();
        double poses[8];
        for(int j=0; j<8; j++)
            poses[j] = sample->get(j+2).asDouble();
        iPos->positionMove(poses);
        // set the delay between
        if(t_prev > 0.0)
            Time::delay(sample->get(1).asDouble() - t_prev);
        t_prev = sample->get(1).asDouble();
    }

    for(int i=0; i<axes; i++)
        iInteract->setInteractionMode(i, yarp::dev::VOCAB_IM_COMPLIANT);

    return true;
}

bool ActionManager::playSimAction(yarp::os::Bottle &data, const double speed, const bool sync) {
    for(int i=0; i<axes; i++) {
        iInteract->setInteractionMode(i, (useStiffMode) ? VOCAB_IM_STIFF : VOCAB_IM_COMPLIANT);
        iCtrl->setControlMode(i, VOCAB_CM_POSITION);
        iPos->setRefSpeed(i, speed*100.0);
    }

    for(int i=0; i<data.size(); i++) {
        Bottle* sample = data.get(i).asList();
        if(sample->isNull() || sample->size()!=axes+2) {
            yWarning()<<"playFile: wrong data type!";
            continue;
        }
        //yInfo()<<sample->toString();
        double poses[8];
        for(int j=0; j<8; j++)
            poses[j] = sample->get(j+2).asDouble();
        // set the delay for the current frame
        Time::delay(sample->get(1).asDouble());
        iPos->positionMove(poses);
    }

    for(int i=0; i<axes; i++)
        iInteract->setInteractionMode(i, yarp::dev::VOCAB_IM_COMPLIANT);

    return true;
}


bool ActionManager::idleParts(const std::string& parts) {
    for(int i=0; i<axes; i++)
        iCtrl->setControlMode(i, VOCAB_CM_IDLE);
    return true;
}

bool ActionManager::activeParts(const std::string& parts){
    for(int i=0; i<axes; i++)
        iCtrl->setControlMode(i, VOCAB_CM_POSITION);
    return true;
}

bool ActionManager::record(const std::string& name,
                           const std::string& parts, const double timeout){
    recorder->getMutex().lock();
    Bottle& data = recorder->getRecordedData();
    data.clear();
    Property& header = data.addDict();
    header.put("type", "robot");
    header.put("parts", parts);
    if(timeout>0.0){
        header.put("timeout", timeout);
        header.put("t_start", Time::now());
    }
    header.put("filename", actionsPath+"/"+name+".log");
    recorder->getMutex().unlock();
    yAssert(recorder != NULL);
    return recorder->start();
}

bool ActionManager::stopPlaying() {
    return false;
}

bool ActionManager::stopRecording() {
    yAssert(recorder != NULL);
    recorder->stop();
    return recorder->save();
    return true;
}

std::vector<std::string> ActionManager::getAvailableActions(){
    std::vector<std::string> actions;
    return actions;
}

void ActionManager::setRecordingRate(const int16_t rate_ms) {
    recorder->setRate(rate_ms);
}

bool ActionManager::loadAction(const std::string filename, yarp::os::Bottle &data) {

    fstream file;
    file.open(filename.c_str());
    if (!file.is_open()) {
        yWarning()<<"Cannot open action file"<<filename;
        return false;
    }

    data.clear();
    string line;
    unsigned int count = 0;
    while(getline(file, line)) {
        count++;
        Bottle sample(line.c_str());
        if(sample.size() == axes+2)
            data.addList() = sample;
        else
            yWarning()<<"Wrong action data at line"<<count;
    }
    file.close();
    return (data.size()>0);
}

bool ActionManager::loadSimAction(const std::string filename, yarp::os::Bottle &data) {
    fstream file;
    file.open(filename.c_str());
    if (!file.is_open()) {
        yWarning()<<"Cannot open action file"<<filename;
        return false;
    }
    data.clear();
    string line;
    if(!getline(file, line)) {
        yError()<<"Cannot read the frame count at line 0";
        file.close();
        return false;
    }

    Bottle rawSample;
    vector<double> values(8); //TODO: change it to values(axes) for double check!
    yAssert(values.size()==8);
    unsigned int count = atoi(line.c_str());
    unsigned int prevFrameNum = 0;
    for(int i=0; i<count; i++) {
        // read frame number
        if(!getline(file, line)) {
            yError()<<"Cannot read the frame number at line "<<i+1;
            file.close();
            return false;
        }      
        int frameNum = atoi(line.c_str());

        // read joints number
        getline(file, line);

        for(int j=0; j<5; j++) {
            // read left shoulder
            getline(file, line);
            if(!extractJointData(line, rawSample))
                continue;
            std::string tag = rawSample.get(0).asString();
            if(tag=="Head") {
                int pos = rawSample.get(6).asDouble();
                pos = (pos > 180) ? 360-pos : -1*pos;
                values[0] = pos;
                pos = rawSample.get(5).asDouble();
                pos = (pos > 180) ? 360-pos : -1*pos;
                values[1] = pos * -1;
            }
            else if(tag=="Right Shoulder") {
                values[2] = (360 - (int)rawSample.get(5).asDouble()) % 360;
                values[3] = -70.0 + rawSample.get(7).asDouble();
            }
            else if(tag=="Right Arm") {
                int pos = (int) rawSample.get(6).asDouble();
                if(pos<180)
                    pos = -1 * pos;
                else
                    pos = 360 - pos;
                values[4] = -1 * pos;
            }
            else if(tag=="Left Shoulder") {
                int pos = rawSample.get(5).asDouble();
                pos = (pos == 0) ? pos : pos - 360;
                values[5] = pos;
                pos = rawSample.get(7).asDouble();
                pos = (pos == 0) ? 360 : pos;
                values[6] = pos - 290;
            }
            else if(tag=="Left Arm") {
                int pos = (int) rawSample.get(6).asDouble();
                if(pos<180)
                    pos = -1 * pos;
                else
                    pos = 360 - pos;
                values[7] = -1 * pos;
            }
            else
                yWarning()<<"Unknown tag '"<<tag<<"' in action file"<<filename;
        }
        // adjust the joint limit
        //for(int k=0; k<8; k++)
        //    values[k] = (values[k]>180.0) ? values[k] - 360.0 : values[k];

        Bottle& sample = data.addList();
        sample.clear();
        sample.addInt(i);                                         // id
        sample.addDouble((frameNum-prevFrameNum)*simFramePeriod); // time
        for(int j=0; j<values.size(); j++)
            sample.addDouble(values[j]);
        yInfo()<<sample.toString();
        prevFrameNum = frameNum;
    }

    file.close();
    return (data.size()>0);

}

inline bool ActionManager::extractJointData(const std::string str, Bottle& sample) {
    sample.clear();
    std::istringstream ss(str);
    std::string token;
    // read the tag
    if(!std::getline(ss, token, ',')) {
        yWarning()<<"Cannot read the tag!";
        return false;
    }
    sample.addString(token);

    while(std::getline(ss, token, ','))
        sample.addDouble(atof(token.c_str()));
    return true;
}
