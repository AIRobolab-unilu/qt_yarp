// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2016 Lux Future Robotic
 * Author:  Ali Paikan
 * email:   ali.paikan@iit.it
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
#include <yarp/os/RateThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/Property.h>
#include <yarp/os/RpcServer.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/FrameGrabberInterfaces.h>

#include <string>
#include <vector>
#include <map>

#include <esUtil.h>
#include <faceViewer_IDL.h>

namespace cuddie{
    namespace dev{
        class faceViewer;
        class VideoFrameGrabber;
    }
}

#define DEFAULT_PERIOD 66   //ms == 15FPS

/***************************************************************
 * @brief The UserData class
 */
class  UserData {
public:
   // Handle to a program object
   GLuint programObject;
   // Attribute locations
   GLint  positionLoc;
   GLint  texCoordLoc;
   // Sampler locations
   GLint samplerLoc;
   // Texture handle
   GLuint textureID;
   bool needUpdate;
   bool useStreamData;

public:
    void lock() { mutex.lock(); }
    void unlock() { mutex.unlock(); }

private:
    yarp::os::Mutex mutex;
};


/***************************************************************
 * @brief The ImagePort class
 */
class ImagePort : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > {
public:
    ImagePort(UserData* user_data) {
        userData = user_data;
        useCallback();
    }
    virtual ~ImagePort() { }

    virtual void onRead(yarp::sig::ImageOf<yarp::sig::PixelRgb>& img);
private:
    UserData* userData;
};


/***************************************************************
 * @brief The VideoFrameGrabber class
 */
class cuddie::dev::VideoFrameGrabber {
public:
    VideoFrameGrabber() : frameIndex(0), speed(1) { }
    virtual ~VideoFrameGrabber() { close(); }
    const size_t frameCount() { return frames.size(); }
    const size_t currentFrameIndex() { return frameIndex; }
    void reseFrameIndex() { frameIndex = 0; }
    void setSpeed(size_t speed);
    const size_t getSpeed() { return speed; }
    const std::string getFileName() { return videoFileName; }
    bool open(const std::string filename, int maxFrame=-1);
    bool getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image);
    bool getImageAt(size_t index, yarp::sig::ImageOf<yarp::sig::PixelRgb>& image);
    void close();    

private:
    std::string videoFileName;
    yarp::dev::PolyDriver driver;
    yarp::dev::IFrameGrabberImage *grabber;
    size_t frameIndex;
    size_t speed;
    std::vector<yarp::sig::ImageOf<yarp::sig::PixelRgb> > frames;
};

/****************************************************************
 * @brief The cuddie::dev::faceViewer class
 */
class cuddie::dev::faceViewer : public yarp::dev::DeviceDriver,
                                public yarp::os::RateThread, 
                                public faceViewer_IDL {
public:
    typedef std::map<std::string, cuddie::dev::VideoFrameGrabber*> EmotionContainer;
    typedef EmotionContainer::iterator EmotionIterator;

public:
    faceViewer();
    ~faceViewer();

    // Device Driver interface
    virtual bool open(yarp::os::Searchable &config);
    virtual bool close();

    // faceViewer_IDL
    /**
     * set the cuddie defualt emotions
     * @return true/false on success/failure
     */
    virtual bool setEmotion(const std::string& emoname, const int16_t speed = 1);
    /**
     * set the cuddie emotion from a file
     * @return true/false on success/failure
     */
    virtual bool setEmotionFromFile(const std::string& filename, const int16_t speed = 1);

    static GLuint loadTextureFromYarpImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& img);

private:
    virtual bool threadInit();
    virtual void run();
    virtual void threadRelease();

    int init(ESContext *esContext);
    static void draw(ESContext *esContext);
    //GLuint loadTexture(const char *fileName);    
    bool loadEmotionFile(const std::string filename);
    void releaseEmotion(const std::string& emoname);

private:
    yarp::os::Property config;
    ImagePort imgPort;
    yarp::os::RpcServer rpcPort;
    ESContext esContext;
    UserData  userData;
    std::string emotionsPath;
    std::string emotionName;
    cuddie::dev::VideoFrameGrabber* currentEmotionVideo;    
    double tnormal;
    EmotionContainer emotionVideos;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> img;
};
