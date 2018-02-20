// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_ActionManager_IDL
#define YARP_THRIFT_GENERATOR_ActionManager_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class ActionManager_IDL;


/**
 * ActionManager_IDL
 */
class ActionManager_IDL : public yarp::os::Wire {
public:
  ActionManager_IDL();
  /**
   * play a recorded action using robot or simulator
   */
  virtual bool play(const std::string& name, const double speed = 0.5, const bool sync = 1);
  /**
   * play a recorded action from a file using robot or simulator
   */
  virtual bool playFile(const std::string& filename, const double speed = 0.5, const bool sync = 1);
  /**
   * idle parts: put the corresponding joint in idle mode
   * @return true on success
   */
  virtual bool idleParts(const std::string& parts);
  /**
   * activate parts: put the corresponding joint in controlled mode
   * @return true on success
   */
  virtual bool activeParts(const std::string& parts);
  /**
   * record a new action
   * @return true on success
   */
  virtual bool record(const std::string& name, const std::string& parts, const double timeout = 0);
  /**
   * stop recording the action
   * @return true on success
   */
  virtual bool stopRecording();
  /**
   * stop playing the action
   * @return true on success
   */
  virtual bool stopPlaying();
  /**
   * get the available actions
   * @return a list of the available language
   */
  virtual std::vector<std::string>  getAvailableActions();
  /**
   * set the recording sample rate
   * @param rate is the thread recoding rate in ms
   */
  virtual void setRecordingRate(const int16_t rate_ms);
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
