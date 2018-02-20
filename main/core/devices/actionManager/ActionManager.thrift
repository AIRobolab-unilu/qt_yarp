# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ali Paikan
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# ActionManager.thrift

/**
* ActionManager_IDL 
*/

service ActionManager_IDL
{

 /**
  * play a recorded action using robot or simulator
  */ 
  bool play(string name, double speed=0.5, bool sync=true);

 /**
  * play a recorded action from a file using robot or simulator
  */ 
  bool playFile(string filename, double speed=0.5, bool sync=true);

  /**
   * idle parts: put the corresponding joint in idle mode
   * @return true on success
   */
  bool idleParts(string parts);

  /**
   * activate parts: put the corresponding joint in controlled mode
   * @return true on success
   */
  bool activeParts(string parts);

 /**
  * record a new action
  * @return true on success
  */ 
  bool record(string name, string parts, double timeout=0.0);

  /**
  * stop recording the action
  * @return true on success
  */ 
  bool stopRecording();

  /**
  * stop playing the action
  * @return true on success
  */
  bool stopPlaying();

  /**
  * get the available actions
  * @return a list of the available language
  */
  list<string> getAvailableActions();

 /**
  * set the recording sample rate
  * @param rate is the thread recoding rate in ms
  */
  void setRecordingRate(i16 rate_ms);
}


