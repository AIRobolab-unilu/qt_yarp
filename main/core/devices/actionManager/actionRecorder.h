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

#ifndef ACTION_RECORDRE_H
#define ACTION_RECORDRE_H

#include <yarp/os/Bottle.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/Property.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IEncoders.h>             // for reading motor encoders
//#include <yarp/dev/IPositionControl.h>      // for controlling robot in position mode
//#include <yarp/dev/IControlMode2.h>         // for setting the control modes (postion, velocity, idle)
//#include <yarp/dev/IInteractionMode.h>      // for setting the interaction modes (stiff, complient)

namespace cuddie{
    namespace dev{
        class ActionRecorder;
    }
}


/****************************************************************
 * @brief The cuddie::dev::ActionRecorder class
 */
class cuddie::dev::ActionRecorder : public yarp::os::RateThread {

public:
    ActionRecorder(yarp::os::Property& prop,
                   yarp::dev::IEncoders *iEnc);
    ~ActionRecorder();

    bool save();
    virtual bool threadInit();
    virtual void run();
    virtual void threadRelease();

    yarp::os::Bottle& getRecordedData() {
        return recordedData;
    }

    yarp::os::Mutex& getMutex() {
        return recorderMutex;
    }

private:    
    yarp::os::Mutex recorderMutex;
    yarp::os::Bottle recordedData;
    int axes;
    double* encPoses;
    yarp::dev::IEncoders *iEnc;
    //yarp::dev::IPositionControl *iPos;
    //yarp::dev::IControlMode2 *iCtrl;
    //yarp::dev::IInteractionMode* iInteract;
    //bool shouldStop;
    yarp::os::Property config;
};

#endif //ACTION_RECORDRE_H
