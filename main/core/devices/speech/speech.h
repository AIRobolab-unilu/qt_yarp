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

#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/Property.h>
#include <yarp/os/RpcServer.h>
#include <yarp/dev/PolyDriver.h>

#include <string>
#include <vector>

#include <Speech_IDL.h>

#include <picoapi.h>
#include <picoapid.h>
#include <picoos.h>

namespace cuddie{
    namespace dev{
        class Speech;
    }
}



/****************************************************************
 * @brief The cuddie::dev::Speech class
 */
class cuddie::dev::Speech : public yarp::dev::DeviceDriver,
                                public yarp::os::Thread,
                                public Speech_IDL {

public:
    Speech();
    ~Speech();

    // Device Driver interface
    virtual bool open(yarp::os::Searchable &config);
    virtual bool close();

    // Speech_IDL
    virtual bool setLanguage(const std::string& language);
    virtual std::vector<std::string>  getSupportedLang();
    virtual bool say(const std::string& text);
    virtual bool setSpeed(const int16_t speed);
    virtual bool setPitch(const int16_t pitch);
    virtual int16_t getSpeed();
    virtual int16_t getPitch();
    virtual bool play(const std::string& filename);
    virtual bool pause();
    virtual bool stop();

private:
    virtual bool threadInit();
    virtual void run();
    virtual void threadRelease();

private:
    /**
     * @brief renderSpeech
     * @param text
     * @return the rendered wave file path
     */
    const std::string renderSpeech(const std::string& text);
    bool playWav(const std::string& filename);
    void releasePico();
private:
    yarp::os::Property config;
    yarp::os::RpcServer rpcPort;
    std::string language;
    std::string pcmDevice;
    int pitch, speed;
    std::vector<std::string> supportedLangs;
    std::string audioPath;

private: // picotts
    /* adapation layer global variables */
    void *          picoMemArea;
    pico_System     picoSystem;
    pico_Resource   picoTaResource;
    pico_Resource   picoSgResource;
    pico_Resource   picoUtppResource;
    pico_Engine     picoEngine;
    pico_Char *     picoTaFileName;
    pico_Char *     picoSgFileName;
    pico_Char *     picoUtppFileName;
    pico_Char *     picoTaResourceName;
    pico_Char *     picoSgResourceName;
    pico_Char *     picoUtppResourceName;
    int picoSynthAbort;
    std::string lingwarePath;
};
