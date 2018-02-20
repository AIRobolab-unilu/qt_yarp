    // -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2016 Lux Future Robotic
* Authors: Ali Paikan
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*
*/

#include <cmath>
#include <algorithm>

#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>


#include <herkulexMotionControl.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;
using namespace cuddie::dev;
using namespace cuddie::dev::impl;



bool herkulexMotionControl::NOT_YET_IMPLEMENTED(const char *txt)
{
    if(verbose)
        yError() << txt << " is not yet implemented for herkulexMotionControl";
    return false;
}

bool herkulexMotionControl::DEPRECATED(const char *txt)
{
    if(verbose)
        yError() << txt << " has been deprecated for herkulexMotionControl";
    return false;
}


//generic function that check is key1 is present in input bottle and that the result has size elements
// return true/false
bool herkulexMotionControl::extractGroup(Bottle &input,
                                         Bottle &out, const std::string &key1,
                                         const std::string &txt, int size)
{
    size++;
    Bottle &tmp=input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull()) {
        yError () << key1.c_str() << " parameter not found";
        return false;
    }

    if(tmp.size()!=size) {
        yError () << key1.c_str() << " incorrect number of entries for param" << key1.c_str() << " expected size is " << size-1 << " while size of param in config file was " << tmp.size() -1;
        return false;
    }

    out=tmp;
    return true;
}


bool herkulexMotionControl::alloc(int nj)
{
    _axisMap = allocAndCheck<int>(nj);
    _angleToEncoder = allocAndCheck<double>(nj);
    _encodersStamp = allocAndCheck<double>(nj);
    _stamps = allocAndCheck<double>(nj);
    motorIDs = allocAndCheck<int>(nj);
    motorStats = allocAndCheck<int>(nj);
    motorPositions = allocAndCheck<double>(nj);
    motorVelocities = allocAndCheck<double>(nj);
    motorRefSpeeds = allocAndCheck<double>(nj);    
    motorControlModes = allocAndCheck<int>(nj);
    motorPosLimitMaxs = allocAndCheck<double>(nj);
    motorPosLimitMins = allocAndCheck<double>(nj);
    motorPosHome = allocAndCheck<double>(nj);
    motorPosPark = allocAndCheck<double>(nj);
    motorIntractMode = allocAndCheck<yarp::dev::InteractionModeEnum>(nj);
    motorSatOffsets = allocAndCheck<int>(nj);
    motorSatSlopes = allocAndCheck<int>(nj);
    motorKps = allocAndCheck<int>(nj);
    motorKis = allocAndCheck<int>(nj);
    motorKds = allocAndCheck<int>(nj);
    return true;
}

bool herkulexMotionControl::dealloc()
{
    checkAndDestroy(_axisMap);
    checkAndDestroy(_angleToEncoder);
    checkAndDestroy(_encodersStamp);
    checkAndDestroy(_stamps);
    checkAndDestroy(motorIDs);
    checkAndDestroy(motorPositions);
    checkAndDestroy(motorStats);
    checkAndDestroy(motorVelocities);
    checkAndDestroy(motorRefSpeeds);
    checkAndDestroy(motorControlModes);
    checkAndDestroy(motorPosLimitMaxs);
    checkAndDestroy(motorPosLimitMins);
    checkAndDestroy(motorPosHome);
    checkAndDestroy(motorPosPark);
    checkAndDestroy(motorIntractMode);
    checkAndDestroy(motorSatOffsets);
    checkAndDestroy(motorSatSlopes);
    checkAndDestroy(motorKps);
    checkAndDestroy(motorKis);
    checkAndDestroy(motorKds);
    return true;
}

herkulexMotionControl::herkulexMotionControl() :
    ImplementControlCalibration2<herkulexMotionControl, IControlCalibration2>(this),
    ImplementEncodersTimed(this),
    ImplementPositionControl2(this),
    ImplementVelocityControl2(this),
    ImplementControlMode2(this),
    ImplementControlLimits2(this),
    ImplementInteractionMode(this),
    RateThread(100)
{
    verbose             = false;    
    _angleToEncoder     = NULL;
    _njoints            = 0;
    _axisMap            = NULL;
    _encodersStamp      = NULL;
    _stamps             = NULL;
    motorIDs = NULL;
    motorPositions = NULL;
    motorVelocities = NULL;
    motorStats = NULL;
    motorControlModes = NULL;
    motorRefSpeeds = NULL;
    motorPosLimitMins = NULL;
    motorPosLimitMaxs = NULL;
    motorPosHome = NULL;
    motorPosPark = NULL;
    motorIntractMode = NULL;
    motorSatOffsets = NULL;
    motorSatSlopes = NULL;
    motorKps = NULL;
    motorKis = NULL;
    motorKds = NULL;
    motorVelocityScale = 1.0;
}

herkulexMotionControl::~herkulexMotionControl()
{
    yTrace();
    //dealloc();
}


bool herkulexMotionControl::open(yarp::os::Searchable &config)
{
    
    check_value_stiffness = 0;
    
    
    Bottle &general = config.findGroup("general");
    if(general.isNull()) {
        yError() << "No general group found in the config file!";
        return false;
    }

    if(general.find("verbose").asBool()) {
        yInfo() << "***********************\nRunning in verbose mode!\n***********************\n";
        verbose = true;
    }

    //  Read Configuration params from file    
    if(!general.check("Joints") ) {
        yError() << "Missing 'Joints' param in 'GENERAL' group from config file!";
        return false;
    }

    _njoints = general.find("Joints").asInt();

    if(!alloc(_njoints)) {
        yError() << "Malloc failed";
        return false;
    }

    // set the default Control mode
    for(int i=0; i<_njoints; i++)
        motorControlModes[i] = VOCAB_CM_POSITION;

    if(general.check("VelocityScale"))
        motorVelocityScale = general.find("VelocityScale").asDouble();

    if(!fromConfig(config)) {
        yError() << "herkulexMotionControl: Missing parameters in config file";
        return false;
    }

    Bottle &hardware = config.findGroup("hardware");
    if(!hardware.check("SerialDevice") ) {
        yError() << "Missing 'SerialDevice' param in 'hardware' group from config file!";
        return false;
    }

    Bottle xtmp;
    //motorIDs
    if (!extractGroup(hardware, xtmp, "MotorIDs", "a list of motor IDs", _njoints))
        return false;
    for (int i = 1; i < xtmp.size(); i++) {
        if (xtmp.get(i).asInt() < 0)
            yWarning("Motor ID should be positive!");
        motorIDs[i-1] = xtmp.get(i).asInt();
    }

    // motorSatOffsets
    if (!extractGroup(hardware, xtmp, "SatOffset", "a list of motor saturation offsets", _njoints))
        return false;
    for (int i = 1; i < xtmp.size(); i++)
        motorSatOffsets[i-1] = xtmp.get(i).asInt();

    // motorSatSlopes
    if (!extractGroup(hardware, xtmp, "SatSlope", "a list of motor saturation slope", _njoints))
        return false;
    for (int i = 1; i < xtmp.size(); i++)
        motorSatSlopes[i-1] = xtmp.get(i).asInt();

    // motorKps
    if (!extractGroup(hardware, xtmp, "Kp", "a list of motor Kp", _njoints))
        return false;
    for (int i = 1; i < xtmp.size(); i++)
        motorKps[i-1] = xtmp.get(i).asInt();

    // motorKis
    if (!extractGroup(hardware, xtmp, "Ki", "a list of motor Ki", _njoints))
        return false;
    for (int i = 1; i < xtmp.size(); i++)
        motorKis[i-1] = xtmp.get(i).asInt();

    // motorKds
    if (!extractGroup(hardware, xtmp, "Kd", "a list of motor Kd", _njoints))
        return false;
    for (int i = 1; i < xtmp.size(); i++)
        motorKds[i-1] = xtmp.get(i).asInt();

    std::string serialDevice = hardware.find("SerialDevice").asString();
    int baudRate = hardware.check("SerialBaudRate") ? hardware.find("SerialBaudRate").asInt() : 115200;
    int readTimeout = hardware.check("SerialReadTimeout") ? hardware.find("SerialReadTimeout").asInt() : 1000;

    if(!herkulex.open(serialDevice.c_str(), baudRate, readTimeout)) {
        yError() << "Failed to open herkulex device driver!";
        yInfo() <<"SerialDevice :" << serialDevice <<", SerialBaudRate: " << baudRate;
        return false;
    }

    for(int i=0; i<_njoints; i++)
        herkulex.reboot(motorIDs[i]);
    herkulex.delay(1000);


    bool ret = true;
    yInfo()<<"herkulexMotionControl: Setting joints position limits";
    for(int i=0; i<_njoints; i++)
        ret &= herkulex.setPositionLimit(motorIDs[i],
                                         (float)motorPosLimitMins[i],
                                         (float)motorPosLimitMaxs[i]);
    if(!ret) {
        yError() << "Failed to set motor position limits";
        return false;
    }

    yInfo()<<"herkulexMotionControl: Setting PID Values";
    for(int i=0; i<_njoints; i++)
        herkulex.setPID(motorIDs[i], motorKps[i], motorKis[i], motorKds[i]);

    yInfo()<<"herkulexMotionControl: Moving joints to the home position";
    setPositionModeRaw();
    for(int i=0; i<_njoints; i++) {
        herkulex.torqueON(motorIDs[i]);
        herkulex.moveOneAngle(motorIDs[i], motorPosHome[i], 2000, LED_GREEN);
    }
    Time::delay(2.5);

    yInfo()<<"herkulexMotionControl: Setting joints interaction mode";
    setInteractionModesRaw(motorIntractMode);

    // clearing any error after putting joints to the home position
    Time::delay(2.0);
    for(int j=0; j<_njoints; j++) { 
        herkulex.clearError(motorIDs[j]);
        herkulex.torqueON(motorIDs[j]);
    }

    //  INIT ALL INTERFACES
    ImplementControlCalibration2<herkulexMotionControl, IControlCalibration2>::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
    ImplementEncodersTimed::initialize(_njoints, _axisMap, _angleToEncoder, NULL);    
    ImplementPositionControl2::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
    ImplementControlMode2::initialize(_njoints, _axisMap);
    ImplementVelocityControl2::initialize(_njoints, _axisMap, _angleToEncoder, NULL);    
    ImplementControlLimits2::initialize(_njoints, _axisMap, _angleToEncoder, NULL);
    ImplementInteractionMode::initialize(_njoints, _axisMap, _angleToEncoder, NULL);

    int readPeriod = hardware.check("ReadPeriod") ? hardware.find("ReadPeriod").asInt() : 100;
    int readPriority = hardware.check("ReadPriority") ? hardware.find("ReadPriority").asInt() : 0;
    RateThread::setRate(readPeriod);
    if(readPriority != 0) {
        if(!setPriority(readPriority, 1))
            yWarning()<< "Failed to set the real-time priority of the serial device reader thread";
    }
    if(!RateThread::start()) {
        yError() << "Failed to start the the serial device reader thread";
        return false;
    }

    return true;
}


bool herkulexMotionControl::fromConfig(yarp::os::Searchable &config)
{
    Bottle xtmp;
    int i;
    Bottle general = config.findGroup("general");

    // leggere i valori da file
    if (!extractGroup(general, xtmp, "AxisMap", "a list of reordered indices for the axes", _njoints))
        return false;

    for (i = 1; i < xtmp.size(); i++)
        _axisMap[i-1] = xtmp.get(i).asInt();

    // Encoder scales
    if (!extractGroup(general, xtmp,
                      "Encoder", "a list of scales for the encoders", _njoints)) {
        return false;
    }

    for (i = 1; i < xtmp.size(); i++) {
        if (xtmp.get(i).asDouble() < 0)
            yWarning("Encoder parameter should be positive!");       
        _angleToEncoder[i - 1] = xtmp.get(i).asDouble();
    }

    // leggere i valori da file
    if (!extractGroup(general, xtmp, "RefSpeed", "a list of motor default reference speed", _njoints))
        return false;
    for (i = 1; i < xtmp.size(); i++)
        motorRefSpeeds[i-1] = xtmp.get(i).asDouble();

    if (!extractGroup(general, xtmp, "MaxPos","The maximum position for the joint", _njoints))
        return false;
    for (i = 1; i < xtmp.size(); i++)
        motorPosLimitMaxs[i-1] = xtmp.get(i).asDouble();

    if (!extractGroup(general, xtmp, "MinPos","The minimum position for the joint", _njoints))
        return false;
    for (i = 1; i < xtmp.size(); i++)
        motorPosLimitMins[i-1] = xtmp.get(i).asDouble();

    if (!extractGroup(general, xtmp, "HomePos","The home position for the joint", _njoints))
        return false;
    for (i = 1; i < xtmp.size(); i++)
        motorPosHome[i-1] = xtmp.get(i).asDouble();

    if (!extractGroup(general, xtmp, "ParkPos","The prking position for the joint", _njoints))
        return false;
    for (i = 1; i < xtmp.size(); i++)
        motorPosPark[i-1] = xtmp.get(i).asDouble();

    if (!extractGroup(general, xtmp, "InteractMode","The interaction mode for the joint", _njoints))
        return false;
    for (i = 1; i < xtmp.size(); i++)
        motorIntractMode[i-1] = (xtmp.get(i).asInt() == 0) ? VOCAB_IM_STIFF : VOCAB_IM_COMPLIANT;

    return true;
}


bool herkulexMotionControl::close()
{
    yTrace();
    RateThread::stop();

    yInfo()<<"herkulexMotionControl: Moving joints to the parking position";
    setPositionModeRaw();
    for(int i=0; i<_njoints; i++) {
        herkulex.torqueON(motorIDs[i]);
        herkulex.moveOneAngle(motorIDs[i], motorPosPark[i], 2000, LED_OFF);
    }
    Time::delay(3);

    for(int i=0; i<_njoints; i++) {
        herkulex.torqueOFF(motorIDs[i]);
    }

    herkulex.close();

    ImplementControlMode2::uninitialize();
    ImplementEncodersTimed::uninitialize();    
    ImplementPositionControl2::uninitialize();
    ImplementVelocityControl2::uninitialize();
    ImplementControlLimits2::uninitialize();
    ImplementControlCalibration2<herkulexMotionControl, IControlCalibration2>::uninitialize();
    dealloc();
    return true;
}


///////////////////////////////////////
/// RateThread
////

bool herkulexMotionControl::threadInit() {
    return true;
}

void herkulexMotionControl::threadRelease() {

}

void herkulexMotionControl::run() {
    mutexHerkulex.lock();
    for(int i=0; i<_njoints; i++) {
        // read status
        int stat;
        if(herkulex.readRegistryRAM(motorIDs[i], 48, 2, &stat))
        //if(herkulex.stat(motorIDs[i], &stat))
            motorStats[i] = stat;
        else
            yWarning()<<"herkulexMotionControl::run : failed to read status of joint "<<i;

        //herkulex.delay(1);
        // read position
        float pos;
        if(herkulex.getAngle(motorIDs[i], &pos))
            motorPositions[i] = (double)pos;
        else
            yWarning()<<"herkulexMotionControl::run : failed to read position of joint "<<i;
        //herkulex.delay(1);
        // read velocity
        int speed;
        if(herkulex.getSpeed(motorIDs[i], &speed))
            motorVelocities[i] = (double)speed;
        else
            yWarning()<<"herkulexMotionControl::run : failed to read speed of joint "<<i;
        //herkulex.delay(1);
    }    
    mutexHerkulex.unlock();

}

////////////////////////////////////////
//    Velocity control interface raw  //
////////////////////////////////////////

bool herkulexMotionControl::setVelocityModeRaw()
{
    // I guess this is too dangerous to be used with this device.    
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool herkulexMotionControl::velocityMoveRaw(int j, double sp)
{
    // I guess this is too dangerous to be used with this device.
    //motorControlModes[j] = VOCAB_CM_VELOCITY;
    //herkulex.moveSpeedOne(motorIDs[j], )
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool herkulexMotionControl::velocityMoveRaw(const double *sp)
{
    // I guess this is too dangerous to be used with this device.
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}


////////////////////////////////////////
//     Position control interface     //
////////////////////////////////////////
bool herkulexMotionControl::getAxes(int *ax)
{
    *ax=_njoints;
    return true;
}

bool herkulexMotionControl::setPositionModeRaw()
{
    for(int i=0; i<_njoints; i++)
        setControlModeRaw(i, VOCAB_CM_POSITION);
    return true;
}

bool herkulexMotionControl::positionMoveRaw(int j, double ref)
{    
    mutexHerkulex.lock();    
    float max_time = fabsf(motorPositions[j]-ref) * 2.7* 1.0 / motorVelocityScale; //ms
    int ptime = (int)floorf(100.0 / motorRefSpeeds[j] * max_time);
    ptime = (ptime<2856) ? ptime : 2856;
    //yDebug()<<"dist:"<<fabsf(motorPositions[j]-ref)<<"max_time:"<<max_time<<"speed:"<<motorRefSpeeds[j]<<"ptime:"<<ptime;
    bool ret = herkulex.moveOneAngle(motorIDs[j], (float)ref, ptime, LED_GREEN);
    mutexHerkulex.unlock();
    return ret;
}

bool herkulexMotionControl::positionMoveRaw(const double *refs)
{
    bool ret = true;
    mutexHerkulex.lock();
    for(int i=0; i<_njoints; i++) {
        float max_time = fabsf(motorPositions[i]-refs[i]) * 2.7* 1.0 / motorVelocityScale; //ms
        int ptime = (int)floorf(100.0 / motorRefSpeeds[i] * max_time);
        ptime = (ptime<2856) ? ptime : 2856;
        ret &= herkulex.moveOneAngle(motorIDs[i], (float)refs[i], ptime, LED_GREEN);
    }
    mutexHerkulex.unlock();
    return ret;
}

bool herkulexMotionControl::relativeMoveRaw(int j, double delta)
{
    mutexHerkulex.lock();
    float pos;
    if(!herkulex.getAngle(motorIDs[j], &pos))
        return false;
    float max_time = fabsf(delta) * 2.7* 1.0 / motorVelocityScale; //ms
    int ptime = (int)floorf(100.0 / motorRefSpeeds[j] * max_time);
    ptime = (ptime<2856) ? ptime : 2856;
    pos += (float) delta;
    bool ret = herkulex.moveOneAngle(motorIDs[j], pos, ptime, LED_GREEN);
    mutexHerkulex.unlock();
    return ret;
}

bool herkulexMotionControl::relativeMoveRaw(const double *deltas)
{
    bool ret = true;
    mutexHerkulex.lock();
    for(int i=0; i<_njoints; i++) {
        float pos;
        if(!herkulex.getAngle(motorIDs[i], &pos))
            return false;
        float max_time = fabsf(deltas[i]) * 2.7* 1.0 / motorVelocityScale; //ms
        int ptime = (int)floorf(100.0 / motorRefSpeeds[i] * max_time);
        ptime = (ptime<2856) ? ptime : 2856;
        pos += (float) deltas[i];
        ret &= herkulex.moveOneAngle(motorIDs[i], pos, ptime, LED_GREEN);
    }
    mutexHerkulex.unlock();
    return ret;
}

bool herkulexMotionControl::checkMotionDoneRaw(int j, bool *flag)
{
    mutexHerkulex.lock();
    int stat;
    bool ret = herkulex.readRegistryRAM(motorIDs[j], 48, 2, &stat);
    mutexHerkulex.unlock();    
    return (stat | 0x0002) && ret;
}

bool herkulexMotionControl::checkMotionDoneRaw(bool *flag)
{
    bool done = true;
    mutexHerkulex.lock();
    for(int i=0; i<_njoints; i++) {
        int stat;
        bool ret = herkulex.readRegistryRAM(motorIDs[i], 48, 2, &stat);
        done &= (stat | 0x0002) && ret;
    }
    mutexHerkulex.unlock();
    return done;
}

bool herkulexMotionControl::setRefSpeedRaw(int j, double sp)
{    
    mutexHerkulex.lock();
    motorRefSpeeds[j] = (float) sp;
    mutexHerkulex.unlock();
    return true;
}

bool herkulexMotionControl::setRefSpeedsRaw(const double *spds)
{

    return true;
}

bool herkulexMotionControl::setRefAccelerationRaw(int j, double acc)
{
    //return _device.pos->setRefAcceleration(j, acc);
    return true;
}

bool herkulexMotionControl::setRefAccelerationsRaw(const double *accs)
{
    //return _device.pos->setRefAccelerations(accs);
    return true;
}

bool herkulexMotionControl::getRefSpeedRaw(int j, double *spd)
{
    mutexHerkulex.lock();
    *spd = motorRefSpeeds[j];
    mutexHerkulex.unlock();
    return false;
}

bool herkulexMotionControl::getRefSpeedsRaw(double *spds)
{
    mutexHerkulex.lock();    mutexHerkulex.lock();
    for(int i=0; i<_njoints; i++)
        motorRefSpeeds[i] = (float) spds[i];
    mutexHerkulex.unlock();
    for(int i=0; i<_njoints; i++)
         spds[i] = motorRefSpeeds[i];
    mutexHerkulex.unlock();
    return true;
}

bool herkulexMotionControl::getRefAccelerationRaw(int j, double *acc)
{
    //return _device.pos->getRefAcceleration(j, acc);
    return true;
}

bool herkulexMotionControl::getRefAccelerationsRaw(double *accs)
{
    //return _device.pos->getRefAccelerations(accs);
    return true;
}

bool herkulexMotionControl::stopRaw(int j)
{
    //return _device.pos->stop(j);
    return true;
}

bool herkulexMotionControl::stopRaw()
{
    //return _device.pos->stop();
    return true;
}


////////////////////////////////////////
//     Position control2 interface    //
////////////////////////////////////////

bool herkulexMotionControl::positionMoveRaw(const int n_joint, const int *joints, const double *refs)
{
    bool ret = true;
    mutexHerkulex.lock();
    for(int i=0; i<n_joint; i++) {
        float max_time = fabsf(motorPositions[joints[i]]-refs[i]) * 2.7* 1.0 / motorVelocityScale; //ms
        int ptime = (int)floorf(100.0 / motorRefSpeeds[joints[i]] * max_time);
        ptime = (ptime<2856) ? ptime : 2856;
        ret &= herkulex.moveOneAngle(motorIDs[joints[i]], (float)refs[i], ptime, LED_GREEN);
    }
    
    mutexHerkulex.unlock();
    check_value_stiffness++;
    
    if (! (check_value_stiffness % 10))
    {
        refreshMotorStiffness();
        check_value_stiffness = 0;
        //         yInfo()<<__LINE__<< ":"<<check_value_stiffness;
    }    
    
    return ret;
}

bool herkulexMotionControl::relativeMoveRaw(const int n_joint, const int *joints, const double *deltas)
{
    bool ret = true;
    yInfo()<<__LINE__;
    mutexHerkulex.lock();
    /*
    for(int i=0, index=0; i< n_joint; i++, index++)
    {
        _userRef_positions[joints[i]] += deltas[i];
    }
    */
    /*
    if(!herkulex_user2HW(_userRef_positions, _robotRef_positions))
    {
        yError() << "Requested position is not reachable";
    }

    compute_speeds(_robotRef_positions, _lastRobot_encoders);
    */

    // all joints may need to move in order to achieve the new requested position
    // even if only one user virtual joint has got new reference.
    //ret &= _device.pos2->setRefSpeeds(_njoints, _axisMap, _robotRef_speeds.data());
    //ret &= _device.pos2->positionMove(_njoints, _axisMap, _robotRef_positions.data());

    mutexHerkulex.unlock();
    return ret;
}

bool herkulexMotionControl::checkMotionDoneRaw(const int n_joint, const int *joints, bool *flag)
{
    yInfo()<<__LINE__;
    //return _device.pos2->checkMotionDone(n_joint, joints, flag);
    return true;
}

bool herkulexMotionControl::setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds)
{
    mutexHerkulex.lock();
    for(int i=0; i<n_joint; i++)
        motorRefSpeeds[joints[i]] = (float) spds[i];
    mutexHerkulex.unlock();
    return true;
}

bool herkulexMotionControl::setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs)
{
    //return _device.pos2->setRefAccelerations(n_joint, joints, accs);
    return true;
}

bool herkulexMotionControl::getRefSpeedsRaw(const int n_joint, const int *joints, double *spds)
{
    /*
    for(int i=0; i<n_joint; i++)
        spds[i] = _refSpeed;
    */
    return true;
}

bool herkulexMotionControl::getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs)
{
    //return _device.pos2->getRefAccelerations(n_joint, joints, accs);
    return true;
}

bool herkulexMotionControl::stopRaw(const int n_joint, const int *joints)
{
    //return _device.pos2->stop(n_joint, joints);
    return true;
}
///////////// END Position Control INTERFACE  //////////////////

// ControlMode
bool herkulexMotionControl::setPositionModeRaw(int j)
{
    return DEPRECATED("setPositionModeRaw");
}

bool herkulexMotionControl::setVelocityModeRaw(int j)
{
    return DEPRECATED("setVelocityModeRaw");
}

bool herkulexMotionControl::setTorqueModeRaw(int j)
{
    return DEPRECATED("setTorqueModeRaw");
}

bool herkulexMotionControl::setImpedancePositionModeRaw(int j)
{
    return DEPRECATED("setImpedancePositionModeRaw");
}

bool herkulexMotionControl::setImpedanceVelocityModeRaw(int j)
{
    return DEPRECATED("setImpedanceVelocityModeRaw");
}

bool herkulexMotionControl::setOpenLoopModeRaw(int j)
{
    return DEPRECATED("setOpenLoopModeRaw");
}

bool herkulexMotionControl::getControlModeRaw(int j, int *v)
{
    if((j < 0) || (j > _njoints))
        return false;
    mutexHerkulex.lock();
    //printf("stat: %x\n", motorStats[j]);
    if(motorStats[j] & 0x40)
        *v = VOCAB_CM_IDLE;
    //else if(motorStats[j] & 0xFFFF0000)
    //    *v = VOCAB_CM_HW_FAULT;
    else {
        *v = motorControlModes[j];
    }
    mutexHerkulex.unlock();
    return true;
}

// IControl Mode 2
bool herkulexMotionControl::getControlModesRaw(int* v)
{
    mutexHerkulex.lock();
    for(int i=0; i<_njoints; i++) {
        if(motorStats[i] & 0x40)
            v[i] = VOCAB_CM_IDLE;
        //else if(motorStats[i] & 0xFFFF0000)
        //    v[i] = VOCAB_CM_HW_FAULT;
        else
            v[i] = motorControlModes[i];
    }
    mutexHerkulex.unlock();
    return true;
}

bool herkulexMotionControl::getControlModesRaw(const int n_joint, const int *joints, int *modes)
{
    if((n_joint < 0) || (n_joint >= _njoints))
        return false;

    mutexHerkulex.lock();
    for(int i=0; i<n_joint; i++) {
        if(motorStats[joints[i]] & 0x40)
            modes[i] = VOCAB_CM_IDLE;
        //else if(motorStats[joints[i]] & 0xFFFF0000)
        //    modes[i] = VOCAB_CM_HW_FAULT;
        else
            modes[i] = motorControlModes[joints[i]];
    }
    mutexHerkulex.unlock();
    return true;
}

bool herkulexMotionControl::setControlModeRaw(const int j, const int mode)
{
    if((j < 0) || (j > _njoints))
        return false;

    mutexHerkulex.lock();
    motorControlModes[j] = mode;
    switch(mode) {
    case VOCAB_CM_IDLE:        
        herkulex.torqueOFF(motorIDs[j]);
        herkulex.setLed(motorIDs[j], LED_PINK);
        herkulex.clearError(motorIDs[j]);
        break;
    case VOCAB_CM_POSITION:        
        herkulex.torqueON(motorIDs[j]);
        herkulex.setLed(motorIDs[j], LED_GREEN);
        //herkulex.writeWordRegistryRAM(motorIDs[j], 56, );
        break;
    case VOCAB_CM_VELOCITY:
        herkulex.torqueON(motorIDs[j]);
        herkulex.setLed(motorIDs[j], LED_CYAN);
        //herkulex.writeWordRegistryRAM(motorIDs[j], 56, );
        break;
    default:
        mutexHerkulex.unlock();
        return false;
    };
    mutexHerkulex.unlock();
    return true;
}

bool herkulexMotionControl::setControlModesRaw(const int n_joint, const int *joints, int *modes)
{
    yInfo()<<__LINE__;
    //return _device.iMode2->setControlModes(n_joint, joints, modes);
    return true;
}

bool herkulexMotionControl::setControlModesRaw(int *modes)
{
    mutexHerkulex.lock();
    bool ret = true;
    for(int i=0; i<_njoints; i++) {
        motorControlModes[i] = modes[i];
        switch(modes[i]) {
        case VOCAB_CM_IDLE:
            herkulex.torqueOFF(motorIDs[i]);
            herkulex.setLed(motorIDs[i], LED_PINK);
            break;
        case VOCAB_CM_POSITION:
            herkulex.torqueON(motorIDs[i]);
            herkulex.setLed(motorIDs[i], LED_GREEN);
            //herkulex.writeWordRegistryRAM(motorIDs[j], 56, );
            break;
        case VOCAB_CM_VELOCITY:
            herkulex.setLed(motorIDs[i], LED_CYAN);
            herkulex.torqueON(motorIDs[i]);
            //herkulex.writeWordRegistryRAM(motorIDs[j], 56, );
            break;
        default:
            ret = false;
        };
    }
    mutexHerkulex.unlock();
    return ret;
}

//////////////////////// BEGIN EncoderInterface

bool herkulexMotionControl::setEncoderRaw(int j, double val)
{
    return NOT_YET_IMPLEMENTED("setEncoder");
}

bool herkulexMotionControl::setEncodersRaw(const double *vals)
{
    return NOT_YET_IMPLEMENTED("setEncoders");
}

bool herkulexMotionControl::resetEncoderRaw(int j)
{
    return NOT_YET_IMPLEMENTED("resetEncoder");
}

bool herkulexMotionControl::resetEncodersRaw()
{
    return NOT_YET_IMPLEMENTED("resetEncoders");
}

bool herkulexMotionControl::getEncoderRaw(int j, double *value)
{
   mutexHerkulex.lock();
   *value = motorPositions[j];
   mutexHerkulex.unlock();
   return true;
}

bool herkulexMotionControl::getEncodersRaw(double *encs)
{
    mutexHerkulex.lock();
    for(int i=0; i<_njoints; i++)
        encs[i] = motorPositions[i];
    mutexHerkulex.unlock();
    return true;
}

bool herkulexMotionControl::getEncoderSpeedRaw(int j, double *sp)
{
    mutexHerkulex.lock();
    *sp = motorVelocities[j];
    mutexHerkulex.unlock();
    return true;
}

bool herkulexMotionControl::getEncoderSpeedsRaw(double *spds)
{
    mutexHerkulex.lock();
    for(int i=0; i<_njoints; i++)
        spds[i] = motorVelocities[i];
    mutexHerkulex.unlock();
    return true;
}

bool herkulexMotionControl::getEncoderAccelerationRaw(int j, double *acc)
{
    *acc = 0;
    return true;
}

bool herkulexMotionControl::getEncoderAccelerationsRaw(double *accs)
{
    memset(accs, 0, _njoints*sizeof(double));
    return true;
}

///////////////////////// END Encoder Interface

bool herkulexMotionControl::getEncodersTimedRaw(double *encs, double *stamps)
{
    bool ret = getEncoders(encs);
    double t= Time::now();
    for(int i=0; i<_njoints; i++)
        stamps[i] = t;
    return ret;
}

bool herkulexMotionControl::getEncoderTimedRaw(int j, double *value, double *stamp)
{    
    *stamp = Time::now();
    return getEncoder(j, value);
}


// IVelocityControl2
bool herkulexMotionControl::velocityMoveRaw(const int n_joint, const int *joints, const double *spds)
{
    // I guess this is too dangerous to be used with this device.
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool herkulexMotionControl::setVelPidRaw(int j, const Pid &pid)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool herkulexMotionControl::setVelPidsRaw(const Pid *pids)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool herkulexMotionControl::getVelPidRaw(int j, Pid *pid)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}

bool herkulexMotionControl::getVelPidsRaw(Pid *pids)
{
    return NOT_YET_IMPLEMENTED(__YFUNCTION__);
}


// Limit interface
bool herkulexMotionControl::setLimitsRaw(int j, double min, double max)
{
    if((j < 0) || (j > _njoints))
        return false;
    mutexHerkulex.lock();
    herkulex.torqueOFF(motorIDs[j]);
    bool ret = herkulex.setPositionLimit(motorIDs[j], (float)min, (float)max);
    herkulex.torqueON(motorIDs[j]);
    mutexHerkulex.unlock();
    return ret;
}

bool herkulexMotionControl::getLimitsRaw(int j, double *min, double *max)
{
    if((j < 0) || (j > _njoints))
        return false;
    mutexHerkulex.lock();
    float fmin, fmax;
    bool ret = herkulex.getPositionLimit(motorIDs[j], &fmin, &fmax);
    *min = (float)fmin;
    *max = (float)fmax;
    mutexHerkulex.unlock();
    return ret;
}

// IControlLimits2
bool herkulexMotionControl::setVelLimitsRaw(int axis, double min, double max)
{
    return false;
}

bool herkulexMotionControl::getVelLimitsRaw(int axis, double *min, double *max)
{
    *min = 0.0;
    *max = 100.0;
    return true;
}

// InteractionMode
bool herkulexMotionControl::getInteractionModeRaw(int j, yarp::dev::InteractionModeEnum* mode)
{
    if((j < 0) || (j > _njoints))
        return false;
    mutexHerkulex.lock();
    *mode = motorIntractMode[j];
    mutexHerkulex.unlock();
    return true;
}

bool herkulexMotionControl::getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    mutexHerkulex.lock();
    for(int i=0; i<n_joints; i++)
        modes[i] = motorIntractMode[joints[i]];
    mutexHerkulex.unlock();
    return false;
}

bool herkulexMotionControl::getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
    mutexHerkulex.lock();
    for(int i=0; i<_njoints; i++)
        modes[i] = motorIntractMode[i];
    mutexHerkulex.unlock();
    return true;
}

bool herkulexMotionControl::setInteractionModeRaw(int j, yarp::dev::InteractionModeEnum mode)
{
    if((j < 0) || (j > _njoints))
        return false;
    mutexHerkulex.lock();
    motorIntractMode[j] = mode;
    bool ret = true;
    if(mode == VOCAB_IM_COMPLIANT) {
       ret &= herkulex.writeRegistryRAM(motorIDs[j], 11, motorSatOffsets[j]);
       ret &= herkulex.writeWordRegistryRAM(motorIDs[j], 12, motorSatSlopes[j]);
    }else if(mode == VOCAB_IM_STIFF) {
        ret &= herkulex.writeRegistryRAM(motorIDs[j], 11, 0);
        ret &= herkulex.writeWordRegistryRAM(motorIDs[j], 12, 0);
    }else
        ret = false;
    mutexHerkulex.unlock();
}

bool herkulexMotionControl::setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    bool ret = true;
    for(int i=0; i<n_joints; i++)
        ret &= setInteractionModeRaw(joints[i], modes[i]);
    return ret;
}

bool herkulexMotionControl::setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
    bool ret = true;
    for(int i=0; i<_njoints; i++)
        ret &= setInteractionModeRaw(i, modes[i]);
    return ret;
}

bool herkulexMotionControl::refreshMotorStiffness()
{
    mutexHerkulex.lock();
    byte dataStat[DATA_MOVE+8];
    int max_id_motor = 9;
    for(int i=2; i<max_id_motor; i++) {
        herkulex.stat(i, dataStat);
        if (dataStat[0])
        {
            cerr<<"Motor ["<< i << "] suffered the stifness issue" << endl;
            
            herkulex.reboot(i);    
            herkulex.clearError(i);
            herkulex.delay(500); //2s
            herkulex.torqueON(i);
        }
    }
    mutexHerkulex.unlock();
    return true;
}

// eof

