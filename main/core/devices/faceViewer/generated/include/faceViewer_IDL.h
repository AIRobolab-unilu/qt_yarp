// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_faceViewer_IDL
#define YARP_THRIFT_GENERATOR_faceViewer_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class faceViewer_IDL;


/**
 * faceViewer_IDL
 */
class faceViewer_IDL : public yarp::os::Wire {
public:
  faceViewer_IDL();
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
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
