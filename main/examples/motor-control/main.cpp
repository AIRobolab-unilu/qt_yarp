/*
 *  A simple example to control Cuddie motors
 *  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *  Authors: Ali Paikan <ali.paikan@gmail.com>
 *
 */

#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>            // to open a generic remote driver
#include <yarp/dev/IPositionControl.h>      // for controlling robot in position mode
#include <yarp/dev/IEncoders.h>             // for reading motor encoders
#include <yarp/dev/IControlMode2.h>         // for setting the control modes (postion, velocity, idle)
#include <yarp/dev/IInteractionMode.h>      // for setting the interaction modes (stiff, complient)

using namespace yarp::os;

int main(int argc, char* argv[])
{
    // initializing yarp network
    yarp::os::Network yarp;

    // Note: robotInterface should be already up
    // configure the drivers' options
    Property option("(device remote_controlboard)");
    option.put("remote", "/cuddie/motors");     // must be "/cuddie/motors"
    option.put("local","/motorControl/motors"); // can be any yarp port name (e.g. /foo/bar)
    // NOTE: all the yarp ports should be started with a '/'.

    // create and open the remote driver
    yarp::dev::PolyDriver driver;
    if (!driver.open(option)) {
        fprintf(stdout, "Cannot open the remote driver!\n");
        return 0;
    }

    // create the desired interfaces
    yarp::dev::IPositionControl *iPos;
    yarp::dev::IEncoders *iEnc;
    yarp::dev::IControlMode2 *iCtrl;
    yarp::dev::IInteractionMode* iInteract;
    driver.view(iPos);
    driver.view(iEnc);
    driver.view(iCtrl);
    driver.view(iInteract);


    // setting the control mode of joint 2 (right shoulder) to Position control mode
    // alternatively iCtrl->setPositionMode(2);
    iCtrl->setControlMode(2, VOCAB_CM_POSITION);

    // move the joint to angle 30 with 50% of velocity
    iPos->setRefSpeed(2, 50);
    iPos->positionMove(2, 30);

    // wait for two seconds until the motor reach the desire position
    // alternatively we could cal iPos->checkMotionDone(2, &ret) to
    // check whether the motor is in position or not.
    // However, this has not been tested on the reobot yet!!!
    Time::delay(2.0);

    // reading the encoders
    // NOTE: due to gravity force and frictions in the complient mode,
    // the actual motor position may not be exactly the same as the desired one.
    // more accuracy can be achieved by controling  the motor in stiff mode and moving
    // the joint with higher velocity (reference speed).
    double pos;
    iEnc->getEncoder(2, &pos);
    fprintf(stdout, "Motor is in position %.2f\n", pos);

    //////////////////////////////////////////////////////////////////////////////////
    // some more examples
    //////////////////////////////////////////////////////////////////////////////////

    // setting the joint 2 in idle mode
    // NOTE: this is also clear any hardware fault and error of the motor (no more blinking red light :p)
    // NOTE: velocity mode has not been tested yet and better to be avoided
    // Especially when the motor is in stiff mode, any malfunction in this mode
    // can cause severe damage to the motor!!!
    iCtrl->setControlMode(2, VOCAB_CM_IDLE);
    // now the motor torque is released and you can move
    // the joint by hand (the motor LED should turn to pink)
    Time::delay(5.0);

    // setting back the joint 2 in pposition mode (the motor LED should turn to green)
    iCtrl->setControlMode(2, VOCAB_CM_POSITION);


    // setting the joint 2  in stiff mode
    iInteract->setInteractionMode(2, yarp::dev::VOCAB_IM_STIFF);
    // setting back the joint 2 in complient mode
    iInteract->setInteractionMode(2, yarp::dev::VOCAB_IM_COMPLIANT);


    // All of the above interfaces has the batch functionality to control
    // the multiple joints. For example to move all the joint to the home position,
    // e.g., 0.0  -15  0.0    -60.0  30.0   0.0     60.0   -30.0
    double speeds[] = {30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0};
    double poses[] = {0.0, -15.0, 0.0, -60.0, 30.0, 0.0, 60.0, -30.0};
    iPos->setRefSpeeds(speeds);
    iPos->positionMove(poses);
    Time::delay(2.0);
    double encs[8];
    iEnc->getEncoders(encs);

    // closing the driver
    driver.close();
    return 0;
}

