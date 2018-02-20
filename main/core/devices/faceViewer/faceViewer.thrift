# Copyright: (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ali Paikan
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# faceViewer.thrift

/**
* faceViewer_IDL 
*/

service faceViewer_IDL
{

  /**
  * set the cuddie defualt emotions
  * @return true/false on success/failure
  */
  bool setEmotion(string emoname, i16 speed=1);
  
  /**
  * set the cuddie emotion from a file
  * @return true/false on success/failure
  */
  bool setEmotionFromFile(string filename, i16 speed=1);

}

