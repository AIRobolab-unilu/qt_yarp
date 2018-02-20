// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/* Copyright (C) 2015  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Ali Paikan
 * email: ali.paikan@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://github.com/robotology/cuddie/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */


/**
 * @ingroup icub_hardware_modules
 * \defgroup herkulex motion device
 *
 *
 * Copyright (C) 2015  iCub Facility, Istituto Italiano di Tecnologia
 *
 * Author: Ali Paikan
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This device is designed to interface the user input expressed in torso elevation plus
 * orientation into 3 elongation of the real actuator. This conversion is done by
 * a Inverse Kinematic (IK) library. The result will then be sent to the electronic
 * boards controlling the motors, using needed protocol, could it be can or ethernet.
 */


#ifndef __herkulexMotionControlh__
#define __herkulexMotionControlh__

//  Yarp stuff
#include <stdint.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include <yarp/dev/ControlBoardInterfacesImpl.inl>
#include <yarp/dev/IInteractionMode.h>
#include <yarp/sig/Vector.h>

#include<Herkulex.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

namespace cuddie {
    namespace dev  {
    class herkulexMotionControl;
        namespace impl {
            class HW_deviceHelper;
        }
    }
}

using namespace yarp::dev;

class cuddie::dev::herkulexMotionControl:   public DeviceDriver,
                                        public ImplementControlCalibration2<herkulexMotionControl, IControlCalibration2>,
                                        public IEncodersTimedRaw,
                                        public ImplementEncodersTimed,
                                        public IPositionControl2Raw,
                                        public ImplementPositionControl2,
                                        public IVelocityControl2Raw,
                                        public ImplementVelocityControl2,
                                        public IControlMode2Raw,
                                        public ImplementControlMode2,
                                        public IControlLimits2Raw,
                                        public ImplementControlLimits2,
                                        public IInteractionModeRaw,
                                        public ImplementInteractionMode,
                                        public RateThread
{
private:
    bool verbose;

    yarp::os::Mutex mutexHerkulex;

    int     *_axisMap;                          /** axis remapping lookup-table */
    double  *_angleToEncoder;                   /** angle conversion factor, if any */
    double  *_encodersStamp;                    /** keep information about acquisition time for encoders read */


    // basic knowledge of my joints
    int   _njoints;                             // Number of joints handled by this device; this values will be extracted by the config file
    double  *_stamps;

    int *motorIDs;
    double* motorPositions;
    double* motorVelocities;
    int *motorStats;
    int* motorControlModes;
    double* motorRefSpeeds;
    double* motorPosLimitMins;
    double* motorPosLimitMaxs;
    double* motorPosHome;
    double* motorPosPark;
    yarp::dev::InteractionModeEnum*  motorIntractMode;
    int* motorSatOffsets;
    int* motorSatSlopes;
    int* motorKps;
    int* motorKis;
    int* motorKds;
    double motorVelocityScale;
    Herkulex herkulex;

private:

    inline bool NOT_YET_IMPLEMENTED(const char *txt);
    inline bool DEPRECATED(const char *txt);

    bool extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size);
    bool parsePositionPidsGroup(Bottle& pidsGroup, Pid myPid[]);
    bool parseTorquePidsGroup(Bottle& pidsGroup, Pid myPid[], double kbemf[], double ktau[], int filterType[]);

    bool alloc(int njoints);
    bool dealloc();

    bool fromConfig(yarp::os::Searchable &config);

public:

    herkulexMotionControl();
    ~herkulexMotionControl();

    // Device Driver
    virtual bool open(yarp::os::Searchable &par);
    virtual bool close();

    //RateThread
    virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();


    /////////// POSITION CONTROL INTERFACE RAW
    virtual bool getAxes(int *ax);
    virtual bool setPositionModeRaw();
    virtual bool positionMoveRaw(int j, double ref);
    virtual bool positionMoveRaw(const double *refs);
    virtual bool relativeMoveRaw(int j, double delta);
    virtual bool relativeMoveRaw(const double *deltas);
    virtual bool checkMotionDoneRaw(bool *flag);
    virtual bool checkMotionDoneRaw(int j, bool *flag);
    virtual bool setRefSpeedRaw(int j, double sp);
    virtual bool setRefSpeedsRaw(const double *spds);
    virtual bool setRefAccelerationRaw(int j, double acc);
    virtual bool setRefAccelerationsRaw(const double *accs);
    virtual bool getRefSpeedRaw(int j, double *ref);
    virtual bool getRefSpeedsRaw(double *spds);
    virtual bool getRefAccelerationRaw(int j, double *acc);
    virtual bool getRefAccelerationsRaw(double *accs);
    virtual bool stopRaw(int j);
    virtual bool stopRaw();

    // Position Control2 Interface
    virtual bool positionMoveRaw(const int n_joint, const int *joints, const double *refs);
    virtual bool relativeMoveRaw(const int n_joint, const int *joints, const double *deltas);
    virtual bool checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags);
    virtual bool setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds);
    virtual bool setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs);
    virtual bool getRefSpeedsRaw(const int n_joint, const int *joints, double *spds);
    virtual bool getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs);
    virtual bool stopRaw(const int n_joint, const int *joints);

    //  Velocity control interface raw
    virtual bool setVelocityModeRaw();
    virtual bool velocityMoveRaw(int j, double sp);
    virtual bool velocityMoveRaw(const double *sp);

    // ControlMode
    virtual bool setPositionModeRaw(int j);
    virtual bool setVelocityModeRaw(int j);
    virtual bool setTorqueModeRaw(int j);
    virtual bool setImpedancePositionModeRaw(int j);
    virtual bool setImpedanceVelocityModeRaw(int j);
    virtual bool setOpenLoopModeRaw(int j);
    virtual bool getControlModeRaw(int j, int *v);
    virtual bool getControlModesRaw(int *v);

    // ControlMode 2
    virtual bool getControlModesRaw(const int n_joint, const int *joints, int *modes);
    virtual bool setControlModeRaw(const int j, const int mode);
    virtual bool setControlModesRaw(const int n_joint, const int *joints, int *modes);
    virtual bool setControlModesRaw(int *modes);

    //////////////////////// BEGIN EncoderInterface
    virtual bool resetEncoderRaw(int j);
    virtual bool resetEncodersRaw();
    virtual bool setEncoderRaw(int j, double val);
    virtual bool setEncodersRaw(const double *vals);
    virtual bool getEncoderRaw(int j, double *v);
    virtual bool getEncodersRaw(double *encs);
    virtual bool getEncoderSpeedRaw(int j, double *sp);
    virtual bool getEncoderSpeedsRaw(double *spds);
    virtual bool getEncoderAccelerationRaw(int j, double *spds);
    virtual bool getEncoderAccelerationsRaw(double *accs);


    virtual bool getEncodersTimedRaw(double *encs, double *stamps);
    virtual bool getEncoderTimedRaw(int j, double *encs, double *stamp);


    // IVelocityControl2
    bool velocityMoveRaw(const int n_joint, const int *joints, const double *spds);
    bool setVelPidRaw(int j, const Pid &pid);
    bool setVelPidsRaw(const Pid *pids);
    bool getVelPidRaw(int j, Pid *pid);
    bool getVelPidsRaw(Pid *pids);

    // calibration2raw
    //virtual bool setCalibrationParametersRaw(int axis, const CalibrationParameters& params);
    //virtual bool calibrate2Raw(int axis, unsigned int type, double p1, double p2, double p3);
    //virtual bool doneRaw(int j);

    // Limits
    bool setLimitsRaw(int axis, double min, double max);
    bool getLimitsRaw(int axis, double *min, double *max);

    // Limits 2
    bool setVelLimitsRaw(int axis, double min, double max);
    bool getVelLimitsRaw(int axis, double *min, double *max);

    // InteractionMode interface
    bool getInteractionModeRaw(int j, yarp::dev::InteractionModeEnum* _mode);
    bool getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    bool getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes);
    bool setInteractionModeRaw(int j, yarp::dev::InteractionModeEnum _mode);
    bool setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    bool setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes);
    bool refreshMotorStiffness();
    
    int check_value_stiffness;

};

#endif // include guard

