/*
 * Copyright (C) 2016 Lux Future Robotic
 * Author:  Ali Paikan
 * email:   ali.paikan@gmail.com
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef  ACTION_MANAGER_H
#define  ACTION_MANAGER_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/Property.h>
#include <yarp/os/RpcServer.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/dev/IPositionControl.h>      // for controlling robot in position mode
#include <yarp/dev/IEncoders.h>             // for reading motor encoders
#include <yarp/dev/IControlMode2.h>         // for setting the control modes (postion, velocity, idle)
#include <yarp/dev/IInteractionMode.h>      // for setting the interaction modes (stiff, complient)

#include <string>
#include <vector>

#include <ActionManager_IDL.h>
#include <actionRecorder.h>

namespace cuddie{
    namespace dev{
        class ActionManager;
    }
}

/****************************************************************
 * @brief The cuddie::dev::ActionManager class
 */
class cuddie::dev::ActionManager : public yarp::dev::DeviceDriver,
                                public ActionManager_IDL {

public:
    ActionManager();
    ~ActionManager();

    // Device Driver interface
    virtual bool open(yarp::os::Searchable &config);
    virtual bool close();

    // ActionManager_IDL
    virtual bool play(const std::string& name, const double speed = 0.5, const bool sync = 1);
    virtual bool playFile(const std::string& filename, const double speed = 0.5, const bool sync = 1);
    virtual bool idleParts(const std::string& parts);
    virtual bool activeParts(const std::string& parts);
    virtual bool record(const std::string& name, const std::string& parts, const double timeout = 0);
    virtual bool stopRecording();
    virtual bool stopPlaying();
    virtual std::vector<std::string> getAvailableActions();
    virtual void setRecordingRate(const int16_t rate_ms);

private:
    bool loadAction(const std::string filename, yarp::os::Bottle &data);
    bool loadSimAction(const std::string filename, yarp::os::Bottle &data);
    bool playSimAction(yarp::os::Bottle &data,
                       const double speed, const bool sync);
    bool playAction(yarp::os::Bottle &data,
                    const double speed, const bool sync);
    inline bool extractJointData(const std::string str, yarp::os::Bottle& sample);

private:
    yarp::os::Property config;
    yarp::os::RpcServer rpcPort;
    std::string actionsPath;
    bool useStiffMode;
    int axes;
    double simFramePeriod;
    //syarp::os::Mutex mainMutex;
    yarp::dev::PolyDriver motorDriver;
    yarp::dev::IPositionControl *iPos;
    yarp::dev::IEncoders *iEnc;
    yarp::dev::IControlMode2 *iCtrl;
    yarp::dev::IInteractionMode* iInteract;
    cuddie::dev::ActionRecorder* recorder;
};

#endif //ACTION_MANAGER_H
