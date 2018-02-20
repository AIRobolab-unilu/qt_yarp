// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2016 Lux Future Robotic
* Authors: Ali Paikan
* email:   ali.paikan@iit.it
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#include <string>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Random.h>

#include <faceViewer.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace cuddie::dev;



/***************************************************************
 * @brief The ImagePort class
 */
void ImagePort::onRead(yarp::sig::ImageOf<yarp::sig::PixelRgb>& img) {
    userData->lock();
    glDeleteTextures(1, &userData->textureID);
    userData->textureID = faceViewer::loadTextureFromYarpImage(img);
    userData->needUpdate = true;
    userData->useStreamData = true;
    userData->unlock();
}


/***************************************************************
 * @brief The VideoFrameGrabber class
 */
bool VideoFrameGrabber::open(const std::string filename, int maxFrame) {
    frameIndex = -1;
    driver.close();
    Property prop;
    prop.put("device", "opencv_grabber");
    prop.put("movie", filename);
    prop.put("no_drop", true);
    prop.put("framerate", 5);
    if(!driver.open(prop)) {
        yError()<<"Cannot open opencv_grabber";
        return false;
    }
    if( !driver.view(grabber) ) {
        yError()<<"Cannot view IFrameGrabber";
        return false;
    }

    ImageOf<PixelRgb> img;
    while (grabber->getImage(img)) {
        frames.push_back(img);
        if((maxFrame > 0) && (frames.size() >= maxFrame))
            break;
    }
    videoFileName = filename;
    driver.close();
    return (frameCount() > 0);
}

bool VideoFrameGrabber::getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image) {
    if(frameCount() == 0) {
        yWarning()<<"VideoFrameGrabber is closed or could not load any frame!";
        return false;
    }

    if(frameIndex == frameCount())
        return false;
   bool ret = getImageAt(frameIndex, image);
   frameIndex = (frameIndex+speed >= frameCount()) ? frameCount()-1 : frameIndex+speed;
   return ret;
}

bool VideoFrameGrabber::getImageAt(size_t index, yarp::sig::ImageOf<yarp::sig::PixelRgb>& image) {
    if(frameCount() == 0) {
        yWarning()<<"VideoFrameGrabber is closed or could not load any frame!";
        return false;
    }
    if(index >= frameCount()) {
        yWarning()<<"frame index is out of frame counts";
        return false;
    }
    image = frames.at(index);
    return true;
}

void VideoFrameGrabber::setSpeed(size_t speed) {
    if(speed <= 0){
        yWarning()<<"Speed value must be bigger than 0. Reseting speed to 1";
        VideoFrameGrabber::speed = 1;
    }
    else
        VideoFrameGrabber::speed = speed;
}

void VideoFrameGrabber::close() {
    driver.close();
    frameIndex = -1;
}


/****************************************************************
 * @brief The cuddie::dev::faceViewer class
 */
faceViewer::faceViewer() : RateThread(DEFAULT_PERIOD), imgPort(&userData) {
    esContext.userData = NULL;    
    userData.needUpdate = true;
    userData.useStreamData = false;
    currentEmotionVideo = NULL;
    tnormal = 3.0;
}

faceViewer::~faceViewer()
{
    close();
}


bool faceViewer::open(yarp::os::Searchable &config)
{
    faceViewer::config.fromString(config.toString());
    int period;
    if(config.check("period")) {
        period = config.find("period").asInt();
        setRate(period);
    }
    else
        yInfo() << "Using default period of " << DEFAULT_PERIOD << " ms";

    if(!RateThread::start())
        return false;

    yInfo() << "Chaching defualt emotions";
    bool ret = loadEmotionFile(emotionsPath+"/normal.png");
    ret &= loadEmotionFile(emotionsPath+"/blink.avi");
    ret &= loadEmotionFile(emotionsPath+"/angry.avi");
    ret &= loadEmotionFile(emotionsPath+"/disgust.avi");
    ret &= loadEmotionFile(emotionsPath+"/feer.avi");
    ret &= loadEmotionFile(emotionsPath+"/happy.avi");
    ret &= loadEmotionFile(emotionsPath+"/sad.avi");
    ret &= loadEmotionFile(emotionsPath+"/surprise.avi");
    if(!ret)
        yWarning()<<"Some of the default emotions could not be loaded";

    std::string defemo = "normal";
    if(config.check("emotion-default"))
        defemo = config.find("emotion-default").asString();
    if(!setEmotion(defemo))
        return false;
    releaseEmotion("logo");
    return true;
}

bool faceViewer::close()
{
    yInfo()<<"closing faceViewer!";
    faceViewer::stop();

    EmotionIterator it;
    for (it=emotionVideos.begin(); it!=emotionVideos.end(); ++it)
        delete it->second;
    currentEmotionVideo = NULL;
    emotionVideos.clear();
    return true;
}

bool faceViewer::threadInit() {
    esContext.userData = malloc(sizeof(UserData));  
    esInitContext (&esContext);
    esContext.userData = &userData;


    yError()<<"faceViewer cannot initialize open-gl context";


    int width = config.check("width") ? config.find("width").asInt() : 1280;
    int height = config.check("height") ? config.find("height").asInt() : 720;

    esCreateWindow(&esContext, "MultiTexture", width, height, ES_WINDOW_RGB );
    if (!init(&esContext)) {
        yError()<<"faceViewer cannot initialize open-gl context";
       return false;
    }
    esRegisterDrawFunc(&esContext, faceViewer::draw);


    userData.textureID = 0;
    emotionsPath = "/home/emotions";
    if(config.check("emotions-path"))
        emotionsPath = config.find("emotions-path").asString();

    if(!setEmotion("logo"))
        return false;

    std::string portName = "/cuddie/face:i";
    if(config.check("image-port"))
        portName = config.find("iamge-port").asString();
    if(!imgPort.open(portName.c_str())) {
        yError()<<"Cannot open port"<<portName;
        return false;
    }

    this->yarp().attachAsServer(rpcPort);
    if(!rpcPort.open("/cuddie/face:rpc")) {
        yError()<<"Cannot open port /cuddie/face:rpc";
        return false;
    }

    return true;
}

void faceViewer::threadRelease() {
    rpcPort.close();
    imgPort.close();    
    if(esContext.userData) {
        UserData *userData = (UserData*) esContext.userData;
        // Delete texture object
        glDeleteTextures(1, &userData->textureID);
        // Delete program object
        glDeleteProgram(userData->programObject);
        free(esContext.userData);
        esContext.userData = NULL;
    }    
}

void faceViewer::run()
{    
    // check if we use streaming or rpc data
    userData.lock();
    bool streaming = userData.useStreamData;
    bool needUpdate = userData.needUpdate;
    userData.unlock();

    // we are using streaming data and need to update
    if(streaming) {
        //if(!needUpdate) return;
        userData.lock();
        if (esContext.drawFunc != NULL)
            esContext.drawFunc(&esContext);
        eglSwapBuffers(esContext.eglDisplay, esContext.eglSurface);
        userData.needUpdate = false;
        userData.unlock();
        return;
    }

    // Fix 2. 
    // ADded a blink_df (default blinking) to the system, this can be reframe, needs 30fps
    // The reason is the new design of of blinking face, it is too quickly
            
    // we are using rpc data
    static double t1 = yarp::os::Time::now();
    if (emotionName == "normal") {
        if((yarp::os::Time::now() - t1) > tnormal) {
            t1 = yarp::os::Time::now();
            
            setEmotion("blink_df", yarp::os::Random::uniform(2, 4));
        }
    }
    else if (emotionName == "blink_df") {
        if(currentEmotionVideo->currentFrameIndex() >= currentEmotionVideo->frameCount()-1) {
            t1 = yarp::os::Time::now();
            setEmotion("normal");
            // Fix 2. 
            // Previously: tnormal = yarp::os::Random::uniform(5, 10);
            tnormal = yarp::os::Random::uniform(2, 3);
        }
    }
    else if (emotionName == "logo") {
        if(currentEmotionVideo->currentFrameIndex() == (currentEmotionVideo->frameCount()-3))
            currentEmotionVideo->reseFrameIndex();
    }
    else if (emotionName == "blink") {
        // Fix 2.
        //This will return to the normal state, when the last command from tablet was blink.
        // We don't have implemented return to normal state from other emotions
        setEmotion("blink_df", tnormal);
    }

    userData.lock();
    if(currentEmotionVideo == NULL) {
        userData.unlock();
        return;
    }

    if(!currentEmotionVideo->getImage(img)) {
        userData.unlock();
        return;
    }
    //yInfo()<<"rendering"<<currentEmotionVideo->currentFrameIndex()<<"out of"<<currentEmotionVideo->frameCount()<<currentEmotionVideo->getFileName();
    glDeleteTextures(1, &userData.textureID);
    userData.textureID = faceViewer::loadTextureFromYarpImage(img);

    if (esContext.drawFunc != NULL)
        esContext.drawFunc(&esContext);
    eglSwapBuffers(esContext.eglDisplay, esContext.eglSurface);
    userData.unlock();
}

bool faceViewer::loadEmotionFile(const std::string filename) {
    // we need to load the video if it has not been loaded yet
    if(emotionVideos.find(filename) == emotionVideos.end()) {
        VideoFrameGrabber* grabber = new VideoFrameGrabber();
        if(!grabber->open(filename)) {
            delete grabber;
            return false;
        }
        emotionVideos[filename] = grabber;
        yInfo()<<grabber->getFileName()<<": frames"<<grabber->frameCount();
    }
    return true;
}

void faceViewer::releaseEmotion(const std::string &emoname) {
    userData.lock();
    if(emoname == emotionName) {
        userData.unlock();
        return;
    }
    std::string fileName = emotionsPath+"/" + emoname +
                           ((emoname == "normal") ? ".png" : ".avi");
    EmotionIterator it = emotionVideos.find(fileName);
    if(it != emotionVideos.end()) {
        delete it->second;
        emotionVideos.erase(it);
    }
    userData.unlock();
}

bool faceViewer::setEmotion(const std::string &emoname, const int16_t speed) {
    emotionName = emoname;
    std::string fileName = emotionsPath+"/" + emoname +
                           ((emoname == "normal") ? ".png" : ".avi");
    return setEmotionFromFile(fileName, speed);
}

bool faceViewer::setEmotionFromFile(const std::string& filename, const int16_t speed) {
    userData.lock();
    if(!loadEmotionFile(filename)) {
        userData.unlock();
        return false;
    }
    currentEmotionVideo = emotionVideos[filename];
    currentEmotionVideo->reseFrameIndex();
    currentEmotionVideo->setSpeed((size_t)speed);
    userData.useStreamData = false;
    userData.unlock();
    return true;
}

// Initialize the shader and program object
int faceViewer::init(ESContext *esContext)
{
    UserData *userData = (UserData*) esContext->userData;
    GLbyte vShaderStr[] =  
    "attribute vec4 a_position;   \n"
    "attribute vec2 a_texCoord;   \n"
    "varying vec2 v_texCoord;     \n"
    "void main()                  \n"
    "{                            \n"
    "   gl_Position = a_position; \n"
    "   v_texCoord = a_texCoord;  \n"
    "}                            \n";


   GLbyte fShaderStr[] =  
    "precision mediump float;                            \n"
    "varying vec2 v_texCoord;                            \n"
    "uniform sampler2D s_texture;                        \n"
    "void main()                                         \n"
    "{                                                   \n"
    "  gl_FragColor = texture2D( s_texture, v_texCoord );\n"
    "}                                                   \n";

    // Load the shaders and get a linked program object
    userData->programObject = esLoadProgram ( (const char*)vShaderStr, (const char*)fShaderStr );

    // Get the attribute locations
    userData->positionLoc = glGetAttribLocation ( userData->programObject, "a_position" );
    userData->texCoordLoc = glGetAttribLocation ( userData->programObject, "a_texCoord" );

    // Get the sampler location
    userData->samplerLoc = glGetUniformLocation ( userData->programObject, "s_texture" );

   // Load the texture
   //userData->textureID = loadTexture("/home/odroid/Public/basemap.tga");
   glClearColor ( 1.0f, 1.0f, 1.0f, 0.0f );

   return true;
}



// Draw a triangle using the shader pair created in Init()
void faceViewer::draw(ESContext *esContext)
{
    UserData *userData = (UserData*) esContext->userData;
    if(userData->textureID == 0 )
        return; 

   GLfloat vVertices[] = { -1.0f,  1.0f, 0.0f,  // Position 0
                            0.0f,  0.0f,        // TexCoord 0 
                           -1.0f, -1.0f, 0.0f,  // Position 1
                            0.0f,  1.0f,        // TexCoord 1
                            1.0f, -1.0f, 0.0f,  // Position 2
                            1.0f,  1.0f,        // TexCoord 2
                            1.0f,  1.0f, 0.0f,  // Position 3
                            1.0f,  0.0f         // TexCoord 3
                         };

   GLushort indices[] = { 0, 1, 2, 0, 2, 3 };
      
   // Set the viewport
   glViewport ( 0, 0, esContext->width, esContext->height );
   
   // Clear the color buffer
   glClear ( GL_COLOR_BUFFER_BIT );

   // Use the program object
   glUseProgram ( userData->programObject );

   // Load the vertex position
   glVertexAttribPointer ( userData->positionLoc, 3, GL_FLOAT, 
                           GL_FALSE, 5 * sizeof(GLfloat), vVertices );
   // Load the texture coordinate
   glVertexAttribPointer ( userData->texCoordLoc, 2, GL_FLOAT,
                           GL_FALSE, 5 * sizeof(GLfloat), &vVertices[3] );

   glEnableVertexAttribArray ( userData->positionLoc );
   glEnableVertexAttribArray ( userData->texCoordLoc );

   // Bind the texture
   glActiveTexture ( GL_TEXTURE0 );
   glBindTexture ( GL_TEXTURE_2D, userData->textureID );

   // Set the sampler texture unit to 0
   glUniform1i ( userData->samplerLoc, 0 );

   glDrawElements ( GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, indices );

}

GLuint faceViewer::loadTextureFromYarpImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& img) {
    GLuint texId;
    glGenTextures(1, &texId);
    glBindTexture(GL_TEXTURE_2D, texId);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img.width(), img.height(),
                  0, GL_RGB, GL_UNSIGNED_BYTE, img.getRawImage());
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
    return texId;
}

/*
GLuint faceViewer::loadTexture(const char *fileName )
{
   int width, height;
   char *buffer = esLoadTGA ( (char*) fileName, &width, &height );
   GLuint texId;

   if ( buffer == NULL ) {
      yWarning()<<"Cannot load the image from "<<fileName;
      return 0;
   }
   glGenTextures ( 1, &texId );
   glBindTexture ( GL_TEXTURE_2D, texId );
   glTexImage2D  ( GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, buffer );
   glTexParameteri ( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
   glTexParameteri ( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
   glTexParameteri ( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
   glTexParameteri ( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
   free ( buffer );
   return texId;
}
*/
